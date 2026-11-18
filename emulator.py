#!/usr/bin/env python3
"""Minimal MSP430 emulator for testing the Zwift RunPod bootloader offline.

Example (see test_bootloader.py)

    from emulator import Pod
    pod = Pod("backups/flash_2100_ffff.bin", "backups/info_1000_10ff.bin")
    pod.reset(watchdog=True)
    print(pod.command(0x01).hex())

The emulator models the base MSP430 instruction set with approximate cycle
counts, USCI_A0 UART timing and overruns, P1.4 transmit acknowledgments, and
optional P1.3 flow control for incoming UART bytes. It also models Timer0_A
and USCI_A0 interrupts, Port 1 interrupts, low-power sleep, flash writes, segment erases and
locks, watchdog resets, and the USCI_B0 I2C controller with a simulated
MMA845xQ. Flash writes and erases are logged.

By default, emulation stops when the bootloader jumps to 0x2124. The stock
application requires peripherals such as the ADC that are outside the model.
Set run_application=True to keep running when testing the dice firmware.
The assumed chip ID at 0x0FF0 is 29 55 (MSP430G2955), based on the stock
application's response to command 01.
"""
import random
import struct

MCLK = 1_000_000          # Hz after clock initialization (8 MHz DCO divided by eight).
UART_BYTE_CYCLES = MCLK * 10 // 115_200   # Ten UART bits per byte.
BOOTLOADER_REGION = (0xEF00, 0xFFFF)
APPLICATION_ADDRESS = 0x2124
ACK_TIMEOUT = MCLK      # Assumed timeout before sending the next byte without an acknowledgment.
FLASH_TARGET_INSTRUCTIONS = (0xF654, 0xF9FA)   # Bootloader flash write (f_f610) and erase (f_f9dc).
INFINITY = float("inf")


class Accelerometer:
    """Simulate an MMA845xQ at 0x1C. orientation(t) returns X, Y, Z, and noise in g."""

    def __init__(self):
        self.regs = bytearray(0x40)
        self.regs[0x0D] = 0x2A                # WHO_AM_I (MMA8452Q)
        self.pointer = 0
        self.first = False
        self.orientation = lambda t: (0.0, 0.0, 1.0, 0.01)
        self.time_source = lambda: 0.0
        self.rng = random.Random(1)
        self.writes = []                   # Register and value.

    def start_transaction(self, writing):
        self.first = writing

    def write(self, byte):
        if self.first:
            self.pointer = byte & 0x3F
            self.first = False
        else:
            self.regs[self.pointer] = byte
            self.writes.append((self.pointer, byte))
            self.pointer = (self.pointer + 1) & 0x3F

    def _axis_value(self, axis):
        gx, gy, gz, noise = self.orientation(self.time_source())
        g = (gx, gy, gz)[axis] + self.rng.gauss(0, noise)
        per_g = {0: 64, 1: 32, 2: 16}.get(self.regs[0x0E] & 3, 16)
        return max(-128, min(127, round(g * per_g))) & 0xFF

    def read(self):
        p = self.pointer
        if p in (0x01, 0x03, 0x05):
            v = self._axis_value((p - 1) // 2) if self.regs[0x2A] & 1 else 0
        else:
            v = self.regs[p]
        fast_mode = self.regs[0x2A] & 0x02
        if fast_mode and p in (0x01, 0x03):
            self.pointer = p + 2
        else:
            self.pointer = (p + 1) & 0x3F
        return v


class I2CController:
    """Model the USCI_B0 I2C controller for the firmware's transactions."""

    def __init__(self, device):
        self.device = device
        self.ctl1 = 0x01
        self.busy = False
        self.txifg = False
        self._rxifg = False
        self._next_rx_poll = False
        self.rxbuf = 0
        self.transmitting = False
        self.sda_hold_clocks = 0

    @property
    def rxifg(self):
        if not self._rxifg and self.busy and not self.transmitting:
            if self._next_rx_poll:
                self._next_rx_poll = False
            else:
                self.rxbuf = self.device.read()
                self._rxifg = True
        return self._rxifg

    def write_ctl1(self, v):
        self.ctl1 = v
        if v & 0x01:                          # UCSWRST
            self.busy = self.txifg = self._rxifg = self.transmitting = False
            return
        if v & 0x02:                          # UCTXSTT sends START and the address.
            self.busy = True
            self.ctl1 &= ~0x02
            if v & 0x10:                      # Transmit mode.
                self.transmitting = True
                self.device.start_transaction(True)
                self.txifg = True
            else:                             # Receive mode. One byte arrived.
                self.transmitting = self.txifg = False
                self.device.start_transaction(False)
                self.rxbuf = self.device.read()
                self._rxifg = True
        if v & 0x04 and self.transmitting:    # Receiver STOP completes with its final byte.
            self.busy = self.transmitting = self.txifg = False
            self.ctl1 &= ~0x04

    def write_tx(self, v):
        if self.busy and self.transmitting:
            self.device.write(v)
            self.txifg = True

    def read_rx(self):
        self._rxifg = False
        self._next_rx_poll = True
        if self.ctl1 & 0x04:
            self.busy = False
            self.ctl1 &= ~0x04
        return self.rxbuf


class EmulationStopped(Exception):
    pass


class Pod:
    def __init__(self, flash_bin, info_bin=None, chip_id=b"\x29\x55"):
        self.mem = bytearray(0x10000)
        flash = open(flash_bin, "rb").read()
        self.mem[0x2100:0x2100 + len(flash)] = flash
        if info_bin:
            info = open(info_bin, "rb").read()
            self.mem[0x1000:0x1000 + len(info)] = info
        self.mem[0x0FF0:0x0FF2] = chip_id
        self.flash_log = []      # Pairs of operation ("E" or "W") and address.
        self.violations = []         # Writes or erases in the bootloader region.
        self.outside_flash = []         # Writes outside flash while FCTL1.WRT is set.
        self.cycles = 0
        self.pc_instr = 0
        self.watchdog_resets = 0
        self.run_application = False
        self.flow_control = False   # Require a P1.3 toggle before the next byte.
        self.rx_pending = []
        self.mma = Accelerometer()
        self.mma.time_source = lambda: self.cycles / MCLK
        self.reset(watchdog=False)

    # ------------------------------------------------------------------ reset
    def reset(self, watchdog):
        self.r = [0] * 16
        self.per = bytearray(0x200)
        self.per[0x02] = 1 if watchdog else 0       # IFG1.WDTIFG
        self.per[0x03] = 0x02                               # IFG2.UCA0TXIFG
        self.fctl1, self.fctl3 = 0x00, 0x18                 # LOCK + LOCKA
        self.rx = []                 # Pairs of cycle when available and byte.
        self.rx_pending = []         # Bytes held by the Bluetooth chip.
        self.rx_fallback = False   # Next byte scheduled at the fallback deadline.
        self.tx = bytearray()
        self.overruns = 0
        self.p1in = 0x00
        self.timer_start = None
        self.timer_ack = 0
        self.i2c = I2CController(self.mma)
        self.interrupts = {}
        self.wdt_deadline = self.cycles + 32768          # PUC : SMCLK / 32768
        self.next_event = INFINITY
        self.r[0] = self.word(0xFFFE)
        self.application_started = False
        self.reset_count = getattr(self, "reset_count", -1) + 1

    # Memory
    def word(self, a):
        return self.mem[a] | (self.mem[(a + 1) & 0xFFFF] << 8)

    def _ticks_aclk(self):
        return self.cycles * 32768 // MCLK

    def _timer_wraps(self):
        if self.timer_start is None:
            return 0, 0
        n = self._ticks_aclk() - self.timer_start
        period = (self.per[0x172] | (self.per[0x173] << 8)) + 1
        return n // period, n % period

    def read8(self, a):
        a &= 0xFFFF
        if a >= 0x200:
            return self.mem[a]
        if a == 0x03:                                     # IFG2
            v = (self.per[0x03] & ~0x0D) | 0x02
            if self.rx and self.rx[0][0] <= self.cycles:
                v |= 0x01
            if self.i2c.rxifg:
                v |= 0x04
            if self.i2c.txifg:
                v |= 0x08
            return v
        if a == 0x69:
            return self.i2c.ctl1
        if a == 0x6D:                                     # UCB0STAT : UCBBUSY
            return 0x10 if self.i2c.busy or self.i2c.sda_hold_clocks else 0
        if a == 0x18:                                     # Released P3 I2C pins have external pull-ups.
            port_output = self.per[0x1A] & ~self.per[0x1B]
            pullups = 0x04 if self.i2c.sda_hold_clocks else 0x06
            return (pullups & ~port_output) | (self.per[0x19] & port_output)
        if a == 0x6E:
            return self.i2c.read_rx()
        if a == 0x66:                                     # UCA0RXBUF
            arrives = [x for x in self.rx if x[0] <= self.cycles]
            if not arrives:
                return 0
            # The register holds one byte. Earlier unread bytes are lost.
            self.overruns += len(arrives) - 1
            last = arrives[-1]
            self.rx = [x for x in self.rx if x[0] > self.cycles]
            if self.flow_control and self.rx_pending and not self.rx:
                # Wait for the P1.3 acknowledgment or send on timeout.
                self.rx.append((self.cycles + ACK_TIMEOUT, self.rx_pending.pop(0)))
                self.rx_fallback = True
            self._update_events()
            return last[1]
        if a == 0x20:                                     # P1IN includes input and output levels.
            return (self.p1in & ~self.per[0x22]) | (self.per[0x21] & self.per[0x22])
        if a in (0x162, 0x163):                           # TACCTL0
            w, _ = self._timer_wraps()
            v = self.per[0x162] & ~1
            if w > self.timer_ack:
                v |= 1
            return v if a == 0x162 else self.per[0x163]
        if a in (0x170, 0x171):                           # TAR
            _, tar = self._timer_wraps()
            return tar & 0xFF if a == 0x170 else tar >> 8
        if a in (0x128, 0x129):
            return self.fctl1 if a == 0x128 else 0x96
        if a in (0x12C, 0x12D):
            return self.fctl3 if a == 0x12C else 0x96
        return self.per[a]

    def read16(self, a):
        a &= 0xFFFE
        return self.read8(a) | (self.read8(a + 1) << 8)

    def _flash(self, a, v, is_byte):
        in_flash = 0x1000 <= a < 0x1100 or a >= 0x2100
        if not in_flash:
            if self.pc_instr in FLASH_TARGET_INSTRUCTIONS:
                self.outside_flash.append(a)
            return False
        if self.fctl3 & 0x10:
            self.fctl3 |= 0x04                            # ACCVIFG
            return True
        if BOOTLOADER_REGION[0] <= a <= BOOTLOADER_REGION[1]:
            self.violations.append(("E" if self.fctl1 & 0x02 else "W", a))
        if self.fctl1 & 0x02:                             # ERASE affects one segment.
            size = 64 if a < 0x1100 else 512
            start = a & ~(size - 1)
            for i in range(max(start, 0x2100 if a >= 0x2100 else 0x1000), start + size):
                self.mem[i] = 0xFF
            self.flash_log.append(("E", start))
            self.cycles += 14_500                         # About 4,819 FTG cycles at 333 kHz.
        elif self.fctl1 & 0x40:                           # WRT
            self.mem[a] &= v & 0xFF
            if not is_byte:
                self.mem[a + 1] &= v >> 8
            self.flash_log.append(("W", a))
            self.cycles += 90                             # ~30 cycles FTG
        else:
            self.fctl3 |= 0x04
        return True

    def write8(self, a, v, is_byte=True):
        a &= 0xFFFF
        v &= 0xFF if is_byte else 0xFFFF
        if a >= 0x200:
            if self._flash(a, v, is_byte):
                return
            self.mem[a] = v & 0xFF
            if not is_byte:
                self.mem[(a + 1) & 0xFFFF] = v >> 8
            return
        if self.pc_instr in FLASH_TARGET_INSTRUCTIONS:
            self.outside_flash.append(a)                     # The write targeted a register instead of flash.
        if a == 0x120:                                    # WDTCTL
            if (v >> 8) != 0x5A or is_byte:
                self.reset(watchdog=True)
                raise _ResetInProgress()
            self.per[0x120] = v & 0xFF
            if v & 0x90:                                  # Watchdog stopped or in interval mode.
                self.wdt_deadline = None
            elif v & 0x08 or self.wdt_deadline is None:   # Clear the watchdog counter.
                div = (32768, 8192, 512, 64)[v & 3]
                self.wdt_deadline = self.cycles + (div * MCLK // 32768 if v & 0x04 else div)
            return
        if a == 0x128:
            if (v >> 8) == 0xA5:
                self.fctl1 = v & 0xFF
            else:
                self.reset(watchdog=False)          # An invalid flash key triggers a PUC reset.
                raise _ResetInProgress()
            return
        if a == 0x12C:
            if (v >> 8) == 0xA5:
                self.fctl3 = v & 0xFF
            return
        if a == 0x69:
            self.i2c.write_ctl1(v & 0xFF)
            return
        if a == 0x6F:
            self.i2c.write_tx(v & 0xFF)
            return
        if a == 0x1A and not (self.per[0x1B] | self.per[0x19] | v) & 0x04:
            if self.per[0x1A] & 0x04 and self.i2c.sda_hold_clocks:
                self.i2c.sda_hold_clocks -= 1              # SCL rises when GPIO releases it.
        if a == 0x21 and (v ^ self.per[0x21]) & 0x08 and self.flow_control:
            # Once P1.3 toggles, the Bluetooth chip sends the next byte immediately.
            if self.rx_fallback and self.rx:
                self.rx[0] = (self.cycles + UART_BYTE_CYCLES, self.rx[0][1])
                self.rx_fallback = False
            elif self.rx_pending and not self.rx:
                self.rx.append((self.cycles + UART_BYTE_CYCLES, self.rx_pending.pop(0)))
            self.per[0x21] = v & 0xFF
            self._update_events()
            return
        if a == 0x67:                                     # UCA0TXBUF
            self.tx.append(v & 0xFF)
            self.p1in ^= 0x10                             # Acknowledge on P1.4.
            return
        if a == 0x160 and not is_byte:                      # TACTL
            if v & 0x30:
                self.timer_start = self._ticks_aclk()
                self.timer_ack = 0
            else:
                self.timer_start = None
        if a == 0x162 and not is_byte and not (v & 1):      # Acknowledge CCIFG.
            w, _ = self._timer_wraps()
            self.timer_ack = w
        self.per[a] = v & 0xFF
        if not is_byte:
            self.per[a + 1] = v >> 8
        if a in (0x01, 0x23, 0x25, 0x160, 0x162, 0x172):
            self._update_events()

    def set_p1_input(self, mask, high):
        """Drive external Port 1 pins and latch edges matching P1IES."""
        before = self.p1in
        self.p1in = (before | mask) if high else (before & ~mask)
        changed = (before ^ self.p1in) & ~self.per[0x22]
        self.per[0x23] |= changed & (self.p1in ^ self.per[0x24])
        self._update_events()

    # Interrupts
    def _update_events(self):
        evts = [INFINITY]
        if self.per[0x162] & 0x10 and self.timer_start is not None:
            period = (self.per[0x172] | (self.per[0x173] << 8)) + 1
            t = self.timer_start + (self.timer_ack + 1) * period
            evts.append(-(-t * MCLK // 32768))
        if self.per[0x01] & 0x01 and self.rx:
            evts.append(self.rx[0][0])
        if self.per[0x25] & self.per[0x23]:
            evts.append(self.cycles)
        self.next_event = min(evts)

    def _interrupt(self):
        r = self.r
        w, _ = self._timer_wraps()
        if self.per[0x162] & 0x10 and w > self.timer_ack:
            idx = 9
            self.timer_ack = w
        elif self.per[0x01] & 0x01 and self.rx and self.rx[0][0] <= self.cycles:
            idx = 7
        elif self.per[0x25] & self.per[0x23]:
            idx = 2
        else:
            self._update_events()
            return
        r[1] = (r[1] - 2) & 0xFFFF
        self.write16(r[1], r[0])
        r[1] = (r[1] - 2) & 0xFFFF
        self.write16(r[1], r[2])
        r[2] = 0
        r[0] = self.word(0xFFE0 + 2 * idx)
        self.cycles += 6
        self.interrupts[idx] = self.interrupts.get(idx, 0) + 1
        self._update_events()

    def write16(self, a, v):
        self.write8(a & 0xFFFE, v, is_byte=False)

    # ------------------------------------------------------------- CPU
    def _src(self, mode, reg, bw):
        """Return the operand value and its address, if it has one."""
        r = self.r
        if reg == 3:
            return ([0, 1, 2, 0xFFFF][mode] & (0xFF if bw else 0xFFFF)), None
        if reg == 2 and mode >= 2:
            return (4 if mode == 2 else 8), None
        if mode == 0:
            v = r[reg]
            return (v & 0xFF if bw else v), None
        if mode == 1:
            x = self.word(r[0]); r[0] = (r[0] + 2) & 0xFFFF
            if reg == 2:
                a = x
            elif reg == 0:
                a = (r[0] - 2 + x) & 0xFFFF
            else:
                a = (r[reg] + x) & 0xFFFF
            self.cycles += 2
        else:
            if reg == 0 and mode == 3:                    # Immediate operand.
                x = self.word(r[0]); r[0] = (r[0] + 2) & 0xFFFF
                self.cycles += 1
                return (x & 0xFF if bw else x), None
            a = r[reg]
            if mode == 3:
                r[reg] = (r[reg] + (2 if (not bw or reg in (0, 1)) else 1)) & 0xFFFF
            self.cycles += 1
        v = self.read8(a) if bw else self.read16(a)
        return v, a

    def _dst_address(self, ad, reg):
        if ad == 0:
            return None
        x = self.word(self.r[0]); self.r[0] = (self.r[0] + 2) & 0xFFFF
        self.cycles += 3
        if reg == 2:
            return x
        if reg == 0:
            return (self.r[0] - 2 + x) & 0xFFFF
        return (self.r[reg] + x) & 0xFFFF

    def _write_dst(self, reg, a, v, bw):
        if a is None:
            if reg == 3:
                return
            self.r[reg] = (v & 0xFF) if bw else (v & 0xFFFF)
            if reg == 0:
                self.r[0] &= 0xFFFE
        elif bw:
            self.write8(a, v)
        else:
            self.write16(a, v)

    def _flags(self, result, bw, c=None, v=None):
        m = 0x80 if bw else 0x8000
        mask = 0xFF if bw else 0xFFFF
        sr = self.r[2] & ~0x0107
        if result & m: sr |= 0x04
        if (result & mask) == 0: sr |= 0x02
        if c: sr |= 0x01
        if v: sr |= 0x100
        self.r[2] = sr

    def step(self):
        r = self.r
        if self.wdt_deadline is not None and self.cycles >= self.wdt_deadline:
            self.reset(watchdog=True)               # Watchdog overflow.
            self.watchdog_resets += 1
            return
        sr = r[2]
        if sr & 0x08 and self.cycles >= self.next_event:
            self._interrupt()
            return
        if sr & 0x10:                                     # CPUOFF waits for an event.
            end = min(self.next_event if sr & 0x08 else INFINITY,
                      self.wdt_deadline if self.wdt_deadline is not None else INFINITY)
            if end == INFINITY:
                raise EmulationStopped(f"no wake-up event from low-power mode at 0x{r[0]:04x}")
            self.cycles = max(self.cycles, int(end))
            return
        pc = self.pc_instr = r[0]
        if pc == APPLICATION_ADDRESS and not self.application_started:
            self.application_started = True
            if not self.run_application:
                raise EmulationStopped("application started (0x2124)")
        op = self.word(pc)
        r[0] = (pc + 2) & 0xFFFF
        self.cycles += 1
        if 0x2000 <= op < 0x4000:                         # Jumps
            off = op & 0x3FF
            if off & 0x200: off -= 0x400
            c = (op >> 10) & 7
            sr = r[2]
            C, Z, N, V = sr & 1, (sr >> 1) & 1, (sr >> 2) & 1, (sr >> 8) & 1
            taken = [not Z, Z, not C, C, N, not (N ^ V), N ^ V, True][c]
            if taken:
                if off == -1 and c == 7:
                    raise EmulationStopped(f"infinite loop at 0x{pc:04x}")
                r[0] = (r[0] + 2 * off) & 0xFFFF
            self.cycles += 1
            return
        if 0x1000 <= op < 0x1400:                         # format II
            o = (op >> 7) & 7
            bw = (op >> 6) & 1
            mode, reg = (op >> 4) & 3, op & 15
            if o == 6:                                    # RETI
                r[2] = self.read16(r[1]); r[1] += 2
                r[0] = self.read16(r[1]); r[1] += 2
                self.cycles += 4
                return
            if o in (4, 5):                               # PUSH, CALL
                v, _ = self._src(mode, reg, bw)
                r[1] = (r[1] - 2) & 0xFFFF
                if o == 4:
                    if bw: self.write8(r[1], v)
                    else: self.write16(r[1], v)
                else:
                    self.write16(r[1], r[0])
                    r[0] = v & 0xFFFE
                self.cycles += 2
                return
            v, a = self._src(mode, reg, bw)
            mask = 0xFF if bw else 0xFFFF
            m = 0x80 if bw else 0x8000
            if o == 0:                                    # RRC
                result = (v >> 1) | (m if r[2] & 1 else 0)
                self._flags(result, bw, c=v & 1)
            elif o == 1:                                  # SWPB
                result = ((v & 0xFF) << 8) | (v >> 8)
            elif o == 2:                                  # RRA
                result = (v >> 1) | (v & m)
                self._flags(result, bw, c=v & 1)
            elif o == 3:                                  # SXT
                result = (v & 0xFF) | (0xFF00 if v & 0x80 else 0)
                self._flags(result, False, c=result != 0)
            else:
                raise EmulationStopped(f"unknown instruction {op:04x} at 0x{pc:04x}")
            if a is None:
                self._write_dst(reg, None, result & mask, bw and o != 3)
            else:
                self._write_dst(None, a, result & mask, bw)
            return
        if op < 0x4000:
            raise EmulationStopped(f"invalid opcode {op:04x} at 0x{pc:04x}")
        o = op >> 12                                      # format I
        s, ad, bw, as_, d = (op >> 8) & 15, (op >> 7) & 1, (op >> 6) & 1, (op >> 4) & 3, op & 15
        vs, _ = self._src(as_, s, bw)
        a = self._dst_address(ad, d)
        mask = 0xFF if bw else 0xFFFF
        m = 0x80 if bw else 0x8000
        if o == 4:                                        # MOV
            self._write_dst(d, a, vs, bw)
            return
        if a is None:
            vd = r[d] & mask
        else:
            vd = self.read8(a) if bw else self.read16(a)
        C = r[2] & 1
        if o in (5, 6, 7, 8, 9):
            if o in (7, 8, 9):
                s2 = (~vs) & mask
                carry = C if o == 7 else 1
            else:
                s2 = vs
                carry = C if o == 6 else 0
            raw = vd + s2 + carry
            result = raw & mask
            v = ((~(vd ^ s2)) & (vd ^ result) & m) != 0
            self._flags(result, bw, c=raw > mask, v=v)
            if o != 9:
                self._write_dst(d, a, result, bw)
            return
        if o == 0xA:                                      # DADD
            result, c = 0, C
            for i in range(0, 16 if not bw else 8, 4):
                n = ((vd >> i) & 15) + ((vs >> i) & 15) + c
                c = 1 if n > 9 else 0
                if c: n -= 10
                result |= (n & 15) << i
            self._flags(result, bw, c=c)
            self._write_dst(d, a, result, bw)
            return
        if o in (0xB, 0xF):                               # BIT, AND
            result = vd & vs
            self._flags(result, bw, c=result != 0)
            if o == 0xF:
                self._write_dst(d, a, result, bw)
            return
        if o == 0xC:                                      # BIC
            self._write_dst(d, a, vd & ~vs & mask, bw)
            return
        if o == 0xD:                                      # BIS
            self._write_dst(d, a, vd | vs, bw)
            return
        if o == 0xE:                                      # XOR
            result = vd ^ vs
            self._flags(result, bw, c=result != 0, v=(vd & m) and (vs & m))
            self._write_dst(d, a, result, bw)
            return

    # Execution
    def run(self, cycles_max, condition=None):
        """Run for up to cycles_max cycles or until condition() returns true."""
        end = self.cycles + cycles_max
        while self.cycles < end:
            try:
                for _ in range(200):
                    self.step()
            except _ResetInProgress:
                pass
            if condition and condition():
                return True
        return False

    def send(self, data):
        """Queue UART bytes with 115,200 baud timing.
        With flow control enabled, each byte waits for a P1.3 toggle.
        """
        if self.flow_control:
            data = bytes(data)
            if self.rx or self.rx_pending:
                self.rx_pending.extend(data)
                return
            self.rx.append((self.cycles + UART_BYTE_CYCLES, data[0]))
            self.rx_pending.extend(data[1:])
            self._update_events()
            return
        t = max([self.cycles] + [x[0] for x in self.rx])
        for i, b in enumerate(data):
            self.rx.append((t + (i + 1) * UART_BYTE_CYCLES, b))
        self._update_events()

    def received_frame(self):
        """Return the first complete BA frame sent by the pod, or None."""
        tx = self.tx
        i = tx.find(0xBA)
        if i < 0 or len(tx) < i + 2 or len(tx) < i + 2 + tx[i + 1]:
            return None
        return bytes(tx[i:i + 2 + tx[i + 1]])

    def command(self, cmd, params=b"", seq=None, cycles_max=3 * MCLK):
        """Send AB LEN SEQ CMD parameters and return the BA reply or None."""
        self.seq = getattr(self, "seq", 0) + 1 if seq is None else seq
        body = bytes([self.seq & 0xFF, cmd]) + bytes(params)
        self.tx.clear()
        self.send(bytes([0xAB, len(body)]) + body)
        self.run(cycles_max, lambda: self.received_frame() is not None)
        return self.received_frame()


class _ResetInProgress(Exception):
    pass
