; Six-sided Bluetooth remote for the Zwift RunPod (MSP430G2955), D1 revision 3.
; Build with python3 -I asm430.py dice.s dice_v3.mp --list dice_v3.lst
;
; Revision 2 (18 November 2026) toggles P1.3 to acknowledge each received
; UART byte, as the bootloader (f_fb90 -> f_f850) and stock application
; (f_6a3c -> f_5c8a) do. In revision 1 the Bluetooth chip stopped responding
; after the first command, as observed on the pod.
;
; Read the MMA845xQ at I2C address 0x1C 25 times per second (±2 g, 8 bits).
; After eight steady readings (320 ms), the dominant gravity axis gives the face.
; Send a 0x84 RSC frame for a new stable face after the client subscribes.
; Cadence holds the face and distance holds the count of stable face changes.
; After two minutes without motion, request client disconnection and enter LPM3
; with the accelerometer in low-power mode.
; Commands: 01 version, 02 reboot, 05 memory read, 30 ready, 31 face received.
; If the watchdog detects a hang, it resets to the bootloader.
;
; Hardware setup follows the bootloader and stock application (../technical.md).
; UART is 115,200 baud. The transmit handshake uses P1.4, P1.1, and P3.0.
; The receive acknowledgment toggles P1.3. I2C uses USCI_B0 on P3.1 and P3.2.

; Registers
.equ IE2, 0x0001
.equ IFG1, 0x0002
.equ IFG2, 0x0003
.equ P3REN, 0x0010
.equ P3OUT, 0x0019
.equ P3DIR, 0x001A
.equ P3SEL, 0x001B
.equ P1IN, 0x0020
.equ P1OUT, 0x0021
.equ P1DIR, 0x0022
.equ P1IFG, 0x0023
.equ P1IES, 0x0024
.equ P1IE, 0x0025
.equ P1SEL, 0x0026
.equ P1REN, 0x0027
.equ P2OUT, 0x0029
.equ P2DIR, 0x002A
.equ P2SEL, 0x002E
.equ BCSCTL3, 0x0053
.equ DCOCTL, 0x0056
.equ BCSCTL1, 0x0057
.equ BCSCTL2, 0x0058
.equ UCA0CTL1, 0x0061
.equ UCA0BR0, 0x0062
.equ UCA0BR1, 0x0063
.equ UCA0MCTL, 0x0064
.equ UCA0STAT, 0x0065
.equ UCA0RXBUF, 0x0066
.equ UCA0TXBUF, 0x0067
.equ UCB0CTL0, 0x0068
.equ UCB0CTL1, 0x0069
.equ UCB0BR0, 0x006A
.equ UCB0BR1, 0x006B
.equ UCB0STAT, 0x006D
.equ UCB0RXBUF, 0x006E
.equ UCB0TXBUF, 0x006F
.equ UCB0I2CSA, 0x011A
.equ WDTCTL, 0x0120
.equ TACTL, 0x0160
.equ TACCTL0, 0x0162
.equ TACCR0, 0x0172
.equ CALDCO_1MHZ, 0x10FE
.equ CALBC1_1MHZ, 0x10FF

.equ WDT_STOP, 0x5A80          ; Password and WDTHOLD.
.equ WDT_FEED, 0x5A0C            ; Password, WDTCNTCL, and ACLK divided by 32,768 for a one-second interval.

; Settings
.equ TIMER_PERIOD, 1310        ; 32768 / 1311 = 25 Hz
.equ SLEEP_PERIOD, 8191        ; 4 Hz fallback sampling for gradual orientation changes.
.equ IDLE_TICKS, 3000           ; 120 seconds at 25 Hz.
.equ TICKS_TX, 13               ; Transmit timeout is 520 ms (500 ms in the stock firmware).
.equ MOTION_THRESHOLD, 10       ; Sum of differences from the reference sample (64 = 1 g).
.equ DOMINANT_THRESHOLD, 45              ; Dominant axis ≥ 0.7 g.
.equ OTHER_AXIS_THRESHOLD, 28            ; Other axes ≤ 0.44 g.
.equ STEADY_SAMPLES, 8                 ; Number of steady samples before publishing a face.
.equ I2C_TIMEOUT, 2000          ; I2C timeout loop iterations, about 12 ms.
.equ ACCEL_INT, 0x04            ; P1.2 is the accelerometer interrupt, as in stock 0x481e.
.equ CMD_READY, 0x30
.equ CMD_ACK, 0x31
.equ STATUS_IDLE, 0x02          ; Sent on fff1 to ask the subscribed client to disconnect.

; ---------------------------------------------------------------- RAM
.equ V_FACE, 0x1100             ; Last stable face (zero until first detection).
.equ V_STEADY, 0x1101
.equ V_SLEEPING, 0x1102
.equ V_RECONFIGURE, 0x1103           ; Reconfigure the accelerometer.
.equ V_X, 0x1104                ; Most recent signed X, Y, and Z readings.
.equ V_Y, 0x1105
.equ V_Z, 0x1106
.equ V_ERR_I2C, 0x1107
.equ V_CHANGES, 0x1108           ; Stable face changes, 32 bits.
.equ V_TICK, 0x110C             ; Incremented by the 25 Hz timer.
.equ V_LAST_TICK, 0x110E
.equ V_READY, 0x1110             ; Client confirmed that its notifications are enabled.
.equ V_READY_SEQ, 0x1111        ; Sequence of the current client's Ready command.
.equ V_TX_DEADLINE, 0x1112
.equ V_TX_P14, 0x1114
.equ V_SEQ_EVT, 0x1115
.equ V_PENDING, 0x1116          ; Latest face has not been acknowledged by the client.
.equ V_RX_STATE, 0x1117
.equ V_RX_LEN, 0x1118
.equ V_RX_N, 0x1119
.equ V_RING_HEAD, 0x111A         ; Head written by the interrupt handler.
.equ V_RING_TAIL, 0x111B         ; Tail read by the main loop.
.equ V_FRAME_COUNT, 0x111C        ; 16 bits
.equ V_PX, 0x111E               ; Reference sample for motion detection.
.equ V_PY, 0x111F
.equ V_PZ, 0x1120
.equ V_LED, 0x1121
.equ V_CANDIDATE, 0x1122        ; Candidate face from the last sample.
.equ V_INIT, 0x1123             ; Zero means the next sample only establishes a baseline.
.equ V_IDLE_TICKS, 0x1124       ; Ticks without motion or a change of candidate face.
.equ V_SENT, 0x1126             ; Pending face sent once during this subscription.
.equ V_WAKE, 0x1127
.equ RING, 0x1130             ; 32-byte receive buffer.
.equ FRAME, 0x1150              ; Received frame body, up to 22 bytes.
.equ REPLY, 0x1170            ; Outgoing frame, up to 20 bytes.

; Interrupt vectors
; The bootloader dispatches hardware vector N using the word at 0x2104 + 2N.
.org 0x2104
.word isr_default, isr_default, isr_motion, isr_default    ; 0–3 (2 is Port 1).
.word isr_default, isr_default, isr_default, isr_rx       ; 4–7 (7 is USCI receive).
.word isr_default, isr_timer, isr_default, isr_default    ; 8–11 (9 is Timer0_A CCR0).
.word isr_default, isr_default, isr_default, reset        ; 12–15 (15 is reset).

; Startup
.org 0x2124                     ; The bootloader always jumps here.
reset:
    MOV #0x2100, SP
    MOV #WDT_STOP, &WDTCTL
    DINT
    MOV #0x1100, R15            ; Clear application RAM.
clear_ram:
    CLR 0(R15)
    INCD R15
    CMP #0x1200, R15
    JNE clear_ram

    CLR.B &DCOCTL               ; MCLK and SMCLK use the calibrated 1 MHz DCO.
    MOV.B &CALBC1_1MHZ, &BCSCTL1
    MOV.B &CALDCO_1MHZ, &DCOCTL
    CLR.B &BCSCTL2
    MOV.B #0xC0, &P2SEL         ; 32,768 Hz crystal on XIN/XOUT, as in the bootloader.
    BIS.B #0x04, &BCSCTL3
    MOV #1000, R14              ; Wait up to 1 s for the crystal. Otherwise use VLO for ACLK.
wait_for_crystal:
    BIC.B #0x02, &IFG1          ; OFIFG
    MOV #333, R15
crystal_delay:
    DEC R15
    JNZ crystal_delay
    BIT.B #0x01, &BCSCTL3       ; LFXT1OF
    JZ crystal_ready
    DEC R14
    JNZ wait_for_crystal
    BIS.B #0x20, &BCSCTL3       ; LFXT1S_2 selects VLO (about 12 kHz).
crystal_ready:

    BIC.B #0x0C, &P2OUT         ; Set outputs P2.2 and P2.3 (LED) low.
    BIS.B #0x0C, &P2DIR
    BIC.B #0x10, &P1REN         ; P1.4 is the Bluetooth chip's acknowledgment input.
    BIC.B #0x10, &P1SEL
    BIC.B #0x10, &P1DIR
    BIC.B #0x2A, &P1REN         ; P1.1, P1.3, and P1.5 are low outputs.
    BIC.B #0x2A, &P1SEL
    BIC.B #0x2A, &P1OUT
    BIS.B #0x2A, &P1DIR
    BIC.B #0x01, &P3REN         ; P3.0 is a low output.
    BIC.B #0x01, &P3SEL
    BIC.B #0x01, &P3OUT
    BIS.B #0x01, &P3DIR
    BIC.B #ACCEL_INT, &P1IE
    BIC.B #ACCEL_INT, &P1SEL
    BIC.B #ACCEL_INT, &P1DIR
    BIC.B #ACCEL_INT, &P1REN
    BIS.B #ACCEL_INT, &P1IES    ; The accelerometer drives its interrupt low.

    MOV.B #1, &UCA0CTL1         ; UART initialization from bootloader routine f_f3a0.
    BIS.B #0x30, &P3SEL
    BIS.B #0x80, &UCA0CTL1
    MOV.B #8, &UCA0BR0
    CLR.B &UCA0BR1
    MOV.B #0x0C, &UCA0MCTL
    BIC.B #1, &UCA0CTL1
    BIS.B #1, &IE2              ; UCA0RXIE

    BIC.B #0x06, &P3REN         ; Idle I2C bus (P3.1 and P3.2 high), as in f_5dbe.
    BIC.B #0x06, &P3SEL
    BIS.B #0x06, &P3OUT
    BIS.B #0x06, &P3DIR
    CALL #i2c_init
    MOV.B #1, &V_RECONFIGURE         ; Configure the accelerometer in the main loop.

    MOV #TIMER_PERIOD, &TACCR0 ; Timer0_A in up mode at 25 Hz from ACLK.
    MOV #0x0010, &TACCTL0
    MOV #0x0114, &TACTL
    MOV #WDT_FEED, &WDTCTL       ; One-second watchdog.
    EINT

; Main loop
main_loop:
    MOV #WDT_FEED, &WDTCTL
    CALL #handle_rx
    CALL #report_pending
    CMP &V_TICK, &V_LAST_TICK
    JEQ wait_active
    MOV &V_TICK, &V_LAST_TICK
    CALL #update_led
    TST.B &V_RECONFIGURE
    JZ loop_sample
    CALL #mma_config
loop_sample:
    INC &V_IDLE_TICKS
    CALL #sample_axes
    CMP #IDLE_TICKS, &V_IDLE_TICKS
    JLO main_loop
    CALL #standby
    JMP main_loop
wait_active:
    DINT
    NOP
    CMP.B &V_RING_HEAD, &V_RING_TAIL
    JNE wake
    CMP &V_TICK, &V_LAST_TICK
    JNE wake
    BIS #0x18, SR               ; Enter LPM0 and enable interrupts in one instruction.
    NOP
    JMP main_loop
wake:
    EINT
    JMP main_loop

; Request that the client drop its BLE link, then stop SMCLK and the UART
; until the accelerometer detects movement. Keep the last face and pending data.
standby:
    CALL #mma_sleep
    TST R12
    JNZ standby_error
    TST.B &V_READY
    JZ standby_enter
    MOV.B &V_READY_SEQ, &REPLY
    MOV.B #CMD_READY, &REPLY+1
    MOV.B #STATUS_IDLE, &REPLY+2
    MOV #REPLY, R12
    MOV #3, R13
    CALL #send_frame
standby_enter:
    DINT
    NOP
standby_tx_drain:
    BIT.B #1, &UCA0STAT         ; Wait for the last stop bit before disabling UART.
    JNZ standby_tx_drain
    BIC.B #1, &IE2
    BIS.B #1, &UCA0CTL1
    CLR.B &V_READY
    CLR.B &V_RX_STATE
    MOV.B &V_RING_HEAD, &V_RING_TAIL
    MOV.B #1, &V_SLEEPING
    CLR.B &V_WAKE
    MOV #SLEEP_PERIOD, &TACCR0
    MOV #0x0114, &TACTL
    MOV &V_TICK, &V_LAST_TICK
    MOV #WDT_STOP, &WDTCTL
    BIC.B #ACCEL_INT, &P1IFG
    BIS.B #ACCEL_INT, &P1IE
    BIT.B #ACCEL_INT, &P1IN      ; Handle an interrupt already latched by the sensor.
    JZ standby_exit
standby_wait:
    BIS #0xD8, SR               ; LPM3 with interrupts enabled.
    NOP
    TST.B &V_WAKE
    JNZ standby_exit
    MOV &V_TICK, &V_LAST_TICK
    CALL #sample_axes
    DINT
    NOP
    TST.B &V_WAKE
    JZ standby_wait
standby_exit:
    DINT
    NOP
    BIC.B #ACCEL_INT, &P1IE
    CLR.B &V_SLEEPING
    MOV #TIMER_PERIOD, &TACCR0
    MOV #0x0114, &TACTL
    BIC.B #1, &UCA0CTL1
    BIS.B #1, &IE2
    MOV #WDT_FEED, &WDTCTL
    EINT
    CALL #wake_radio
standby_error:
    CLR &V_IDLE_TICKS
    MOV.B #1, &V_RECONFIGURE
    RET

; Interrupt handlers
isr_rx:                         ; Received a byte from the Bluetooth chip.
    PUSH R15
    PUSH R14
    MOV.B &UCA0RXBUF, R14
    XOR.B #0x08, &P1OUT         ; Toggle P1.3 to acknowledge the byte.
    MOV.B &V_RING_HEAD, R15
    MOV.B R14, RING(R15)
    INC.B R15
    AND.B #0x1F, R15
    CMP.B &V_RING_TAIL, R15
    JEQ rx_full                ; Drop the byte if the ring buffer is full.
    MOV.B R15, &V_RING_HEAD
rx_full:
    BIC #0x10, 4(SP)            ; Leave LPM0 on return.
    POP R14
    POP R15
    RETI

isr_timer:
    INC &V_TICK
    BIC #0xD0, 0(SP)
    RETI

isr_motion:
    BIC.B #ACCEL_INT, &P1IE
    BIC.B #ACCEL_INT, &P1IFG
    MOV.B #1, &V_WAKE
    BIC #0xD0, 0(SP)
    RETI

isr_default:
    RETI

; Receive
handle_rx:
rx_loop:
    MOV.B &V_RING_TAIL, R15
    CMP.B &V_RING_HEAD, R15
    JEQ rx_done
    MOV.B RING(R15), R12
    INC.B R15
    AND.B #0x1F, R15
    MOV.B R15, &V_RING_TAIL
    MOV.B &V_RX_STATE, R15
    CMP.B #1, R15
    JEQ rx_length
    CMP.B #2, R15
    JEQ rx_body
    CMP.B #0xAB, R12            ; State 0 waits for 0xAB.
    JNE rx_loop
    MOV.B #1, &V_RX_STATE
    JMP rx_loop
rx_length:
    CMP.B #0x17, R12            ; Reject lengths ≥ 0x17, as in the bootloader.
    JHS rx_reset
    MOV.B R12, &V_RX_LEN
    CLR.B &V_RX_N
    MOV.B #2, &V_RX_STATE
    TST.B R12
    JZ rx_complete
    JMP rx_loop
rx_body:
    MOV.B &V_RX_N, R15
    MOV.B R12, FRAME(R15)
    INC.B R15
    MOV.B R15, &V_RX_N
    CMP.B &V_RX_LEN, R15
    JNE rx_loop
rx_complete:
    CLR.B &V_RX_STATE
    CALL #execute_command
    JMP rx_loop
rx_reset:
    CLR.B &V_RX_STATE
    JMP rx_loop
rx_done:
    RET

execute_command:
    INC &V_FRAME_COUNT
    CMP.B #2, &V_RX_LEN
    JHS command_valid
    RET
command_valid:
    MOV.B &FRAME, &REPLY      ; SEQ
    MOV.B &FRAME+1, &REPLY+1  ; CMD
    CLR.B &REPLY+2            ; Status 0.
    MOV.B &FRAME+1, R12
    CMP.B #0x01, R12
    JEQ cmd_version
    CMP.B #0x02, R12
    JEQ cmd_reboot
    CMP.B #0x05, R12
    JEQ cmd_read
    CMP.B #CMD_READY, R12
    JEQ cmd_ready
    CMP.B #CMD_ACK, R12
    JEQ cmd_ack
command_invalid:
    MOV.B #1, &REPLY+2        ; Unknown command.
    MOV #3, R13
    JMP command_reply
cmd_ready:
    CMP.B #2, &V_RX_LEN
    JNE command_invalid
    MOV.B #1, &V_READY
    MOV.B &FRAME, &V_READY_SEQ
    CLR.B &V_SENT              ; A new subscription can receive an unconfirmed face.
    MOV #3, R13
    JMP command_reply
cmd_ack:
    CMP.B #7, &V_RX_LEN
    JNE command_invalid
    CMP.B &V_FACE, &FRAME+2
    JNE command_invalid
    ; The count starts at byte 3, so compare as bytes rather than odd words.
    MOV #V_CHANGES, R12
    MOV #FRAME+3, R13
    MOV #4, R14
ack_compare:
    CMP.B @R12+, 0(R13)
    JNE command_invalid
    INC R13
    DEC R14
    JNZ ack_compare
    TST.B &V_SENT              ; Reject an acknowledgment for a face not sent yet.
    JZ command_invalid
    CLR.B &V_PENDING
    MOV #3, R13
    JMP command_reply
cmd_version:                    ; D1 03 04 02 and the chip ID (DicePod revision 3).
    MOV.B #0xD1, &REPLY+3
    MOV.B #0x03, &REPLY+4
    MOV.B #0x04, &REPLY+5
    MOV.B #0x02, &REPLY+6
    MOV.B &0x0FF0, &REPLY+7
    MOV.B &0x0FF1, &REPLY+8
    MOV #9, R13
    JMP command_reply
cmd_read:                       ; Read 13 bytes at 0x2100 + OFF.
    CLR R14
    CMP.B #4, &V_RX_LEN
    JLO read_offset_ready
    MOV.B &FRAME+2, R14
    MOV.B &FRAME+3, R15
    SWPB R15
    BIS R15, R14
read_offset_ready:
    MOV.B R14, &REPLY+3
    MOV R14, R15
    SWPB R15
    MOV.B R15, &REPLY+4
    ADD #0x2100, R14
    MOV #REPLY+5, R15
    MOV #13, R11
read_loop:
    MOV.B @R14+, 0(R15)
    INC R15
    DEC R11
    JNZ read_loop
    MOV #18, R13
    JMP command_reply
cmd_reboot:                 ; Same behavior as command 02 in stock firmware.
    CLR R14
    CMP.B #4, &V_RX_LEN
    JLO check_reboot_mode
    MOV.B &FRAME+2, R14
    BIS.B &FRAME+3, R14
check_reboot_mode:
    DINT
    NOP
    TST.B R14
    JZ reboot_normal
    MOV #0, &WDTCTL             ; An invalid password resets with WDTIFG set, keeping the bootloader active.
reboot_normal:
    MOV #WDT_STOP, &WDTCTL
    BIC.B #0x01, &IFG1          ; Clear WDTIFG so the bootloader starts this application again.
    BR &0xFFFE
command_reply:
    MOV #REPLY, R12
    CALL #send_frame
    RET

; Transmit
report_pending:
    TST.B &V_READY
    JZ report_done
    TST.B &V_PENDING
    JZ report_done
    TST.B &V_SENT
    JNZ report_done
    MOV.B #1, &V_SENT
    CALL #send_measurement
report_done:
    RET

send_measurement:                 ; Frame 0x84 contains a standard RSC measurement.
    MOV #REPLY, R15
    MOV.B &V_SEQ_EVT, 0(R15)
    INC.B &V_SEQ_EVT
    MOV.B #0x84, 1(R15)
    MOV.B #0x03, 2(R15)         ; Flags indicate stride length and total distance fields are present.
    CLR.B 3(R15)                ; Speed is zero.
    CLR.B 4(R15)
    MOV.B &V_FACE, 5(R15)       ; Cadence holds the face.
    CLR.B 6(R15)                ; Stride length is zero.
    CLR.B 7(R15)
    MOV.B &V_CHANGES, 8(R15)  ; Distance holds the face-change count.
    MOV.B &V_CHANGES+1, 9(R15)
    MOV.B &V_CHANGES+2, 10(R15)
    MOV.B &V_CHANGES+3, 11(R15)
    MOV #REPLY, R12
    MOV #12, R13
    JMP send_frame

send_frame:                  ; Send BA LEN BODY with R12 pointing to the body and R13 holding its length.
    PUSH R10
    PUSH R9
    MOV R12, R9
    MOV R13, R10
    MOV.B #0xBA, R12
    CALL #send_byte
    MOV.B R10, R12
    CALL #send_byte
frame_loop:
    MOV.B @R9+, R12
    CALL #send_byte
    DEC R10
    JNZ frame_loop
    POP R9
    POP R10
    RET

; Send a byte using the handshake from bootloader routine f_f40e. Wait for
; P1.4 to change or for the deadline. On timeout, wake the chip by pulling
; P3.0 low, pulsing P1.1 low twice, waiting 2 ms, then pulling P1.1 low.
; The wait does not feed the watchdog and is limited to 520 ms while the
; timer is running.
send_byte:                  ; R12 holds the byte. Clobbers R14 and R15.
tx_wait:
    BIT.B #0x02, &IFG2          ; UCA0TXIFG
    JZ tx_wait
    MOV &V_TICK, R15
    SUB &V_TX_DEADLINE, R15
    JGE tx_wake
    MOV.B &P1IN, R14
    AND.B #0x10, R14
    CMP.B &V_TX_P14, R14
    JEQ tx_wait
    JMP tx_send
tx_wake:
    CALL #wake_radio
tx_send:
    MOV.B &P1IN, R14
    AND.B #0x10, R14
    MOV.B R14, &V_TX_P14
    MOV &V_TICK, R15
    ADD #TICKS_TX, R15
    MOV R15, &V_TX_DEADLINE
    MOV.B R12, &UCA0TXBUF
    MOV #WDT_FEED, &WDTCTL       ; Feed the watchdog after transmitting a byte.
    RET

wake_radio:                  ; Request the existing Bluetooth controller's wake handshake.
    BIC.B #0x01, &P3OUT
    CALL #pulse_p11
    CALL #pulse_p11
    MOV #666, R15             ; About 2 ms.
radio_delay:
    DEC R15
    JNZ radio_delay
    BIC.B #0x02, &P1OUT
    RET

pulse_p11:                  ; Hold P1.1 low for about 100 µs, then raise it.
    BIC.B #0x02, &P1OUT
    MOV #33, R15
pulse_delay:
    DEC R15
    JNZ pulse_delay
    BIS.B #0x02, &P1OUT
    RET

; Face detection
sample_axes:
    MOV #0x01, R12              ; Read the X, Y, and Z MSBs in fast 8-bit mode.
    MOV #V_X, R13
    MOV #3, R14
    CALL #i2c_read
    TST R12
    JZ sample_valid
    MOV.B #1, &V_RECONFIGURE    ; Keep the last face on a sensor failure.
    MOV.B #1, &V_WAKE
    CLR &V_IDLE_TICKS
    CLR.B &V_INIT
    CLR.B &V_CANDIDATE
    CLR.B &V_STEADY
    RET
sample_valid:
    TST.B &V_INIT               ; The first sample establishes the baseline without detecting motion.
    JNZ compare_samples
    MOV.B #1, &V_INIT
    MOV.B &V_X, &V_PX
    MOV.B &V_Y, &V_PY
    MOV.B &V_Z, &V_PZ
    RET
compare_samples:
    MOV.B &V_X, R4
    SXT R4
    MOV.B &V_Y, R5
    SXT R5
    MOV.B &V_Z, R6
    SXT R6
    MOV.B &V_PX, R7             ; Compare against a fixed reference to catch slow rotations.
    SXT R7
    SUB R4, R7
    CALL #abs_r7
    MOV R7, R8
    MOV.B &V_PY, R7
    SXT R7
    SUB R5, R7
    CALL #abs_r7
    ADD R7, R8
    MOV.B &V_PZ, R7
    SXT R7
    SUB R6, R7
    CALL #abs_r7
    ADD R7, R8
    CMP #MOTION_THRESHOLD+1, R8
    JL sample_within_threshold
    MOV.B R4, &V_PX
    MOV.B R5, &V_PY
    MOV.B R6, &V_PZ
    CLR.B &V_STEADY              ; Movement detected.
    CLR.B &V_CANDIDATE
    CLR &V_IDLE_TICKS
    MOV.B #1, &V_WAKE
    RET
sample_within_threshold:
    TST.B &V_SLEEPING
    JNZ motion_done
    MOV R4, R7                  ; R8 = |x|, R9 = |y|, R10 = |z|
    CALL #abs_r7
    MOV R7, R8
    MOV R5, R7
    CALL #abs_r7
    MOV R7, R9
    MOV R6, R7
    CALL #abs_r7
    MOV R7, R10
    CMP R8, R10                 ; Z dominates if |z| ≥ |x| and |z| ≥ |y|.
    JL check_x_or_y
    CMP R9, R10
    JL check_x_or_y
    MOV R10, R11                ; Dominant magnitude in R11, others in R13/R14, index in R12.
    MOV R8, R13
    MOV R9, R14
    MOV #4, R12
    TST R6
    JGE check_thresholds
    MOV #5, R12
    JMP check_thresholds
check_x_or_y:
    CMP R9, R8                  ; X dominates if |x| ≥ |y|.
    JL select_y
    MOV R8, R11
    MOV R9, R13
    MOV R10, R14
    CLR R12
    TST R4
    JGE check_thresholds
    MOV #1, R12
    JMP check_thresholds
select_y:
    MOV R9, R11
    MOV R8, R13
    MOV R10, R14
    MOV #2, R12
    TST R5
    JGE check_thresholds
    MOV #3, R12
check_thresholds:
    CMP #DOMINANT_THRESHOLD, R11
    JL no_face
    CMP #OTHER_AXIS_THRESHOLD+1, R13
    JGE no_face
    CMP #OTHER_AXIS_THRESHOLD+1, R14
    JGE no_face
    MOV.B TABLE_FACES(R12), R12
    JMP check_candidate
no_face:
    CLR R12
check_candidate:
    CMP.B &V_CANDIDATE, R12
    JEQ candidate_steady
    MOV.B R12, &V_CANDIDATE
    MOV.B #1, &V_STEADY
    CLR &V_IDLE_TICKS
    RET
candidate_steady:
    TST.B R12
    JZ motion_done
    CMP.B #STEADY_SAMPLES, &V_STEADY
    JHS motion_done
    INC.B &V_STEADY
    CMP.B #STEADY_SAMPLES, &V_STEADY
    JLO motion_done
    CMP.B &V_FACE, R12
    JEQ motion_done
    TST.B &V_FACE
    JNZ changed_face
    MOV.B R12, &V_FACE          ; Establish a baseline at power-up.
    RET
changed_face:
    INC &V_CHANGES
    ADC &V_CHANGES+2
    MOV.B R12, &V_FACE          ; New stable face.
    MOV.B #1, &V_PENDING
    CLR.B &V_SENT
    MOV.B #3, &V_LED
    BIS.B #0x08, &P2OUT         ; Turn the LED on P2.3 on for three ticks.
motion_done:
    RET

abs_r7:
    TST R7
    JGE abs_done
    INV R7
    INC R7
abs_done:
    RET

update_led:
    TST.B &V_LED
    JZ led_done
    DEC.B &V_LED
    JNZ led_done
    BIC.B #0x08, &P2OUT
led_done:
    RET

; Accelerometer
mma_config:
    CLR.B &V_RECONFIGURE
    CLR.B &V_INIT
    CLR.B &V_CANDIDATE
    CLR.B &V_STEADY
    MOV #mma_active_config, R10
    JMP mma_apply_config
mma_sleep:
    MOV #0x1E, R12             ; Clear any latched transient from the previous sleep.
    MOV #V_WAKE, R13
    MOV #1, R14
    CALL #i2c_read
    TST R12
    JNZ mma_configured
    MOV #mma_sleep_config, R10
mma_apply_config:
    MOV.B @R10+, R12
    TST.B R12
    JZ mma_configured
    MOV.B @R10+, R13
    CALL #i2c_write
    TST R12
    JZ mma_apply_config
    MOV.B #1, &V_RECONFIGURE
mma_configured:
    RET

mma_active_config:
    .byte 0x2A, 0x00            ; Standby while changing mode.
    .byte 0x0E, 0x00            ; ±2 g range.
    .byte 0x2B, 0x00            ; Normal mode.
    .byte 0x2D, 0x00            ; Disable accelerometer interrupts while active.
    .byte 0x2A, 0x23            ; Fast 8-bit reads, 50 Hz.
    .byte 0
mma_sleep_config:
    .byte 0x2A, 0x00
    .byte 0x2B, 0x03            ; Low-power oversampling.
    .byte 0x2C, 0x00            ; Active-low, push-pull interrupt.
    .byte 0x1D, 0x1E            ; Latch high-pass transient events on any axis.
    .byte 0x1F, 0x02            ; 0.126 g threshold.
    .byte 0x20, 0x01            ; One sample over the threshold.
    .byte 0x2E, 0x00            ; Route the transient interrupt to INT2 as in stock.
    .byte 0x2D, 0x20            ; Enable the transient interrupt.
    .byte 0x2A, 0x33            ; Fast 8-bit reads, 6.25 Hz.
    .byte 0
    .even

; ---------------------------------------------------------------- I2C (USCI_B0)
; Use the stock application's sequence (f_678e, f_667c, p_62f2, f_5d72,
; f_6e2a) with timeouts. Return R12 = 0 on success or 1 on error.
i2c_init:
    BIS.B #0x06, &P3SEL
    MOV.B #1, &UCB0CTL1         ; UCSWRST
    MOV.B #0x0F, &UCB0CTL0      ; Synchronous I2C controller mode.
    BIS.B #0x80, &UCB0CTL1      ; SMCLK
    MOV.B #3, &UCB0BR0          ; 333 kHz
    CLR.B &UCB0BR1
    MOV #0x1C, &UCB0I2CSA
    BIC.B #1, &UCB0CTL1
    RET

wait_set:                    ; Wait for (@R11 & R14) ≠ 0. Clobbers R15.
    MOV #I2C_TIMEOUT, R15
wait_set_loop:
    BIT.B R14, 0(R11)
    JNZ wait_set_ok
    DEC R15
    JNZ wait_set_loop
    MOV #1, R12
    RET
wait_set_ok:
    CLR R12
    RET

wait_clear:                  ; Wait for (@R11 & R14) = 0. Clobbers R15.
    MOV #I2C_TIMEOUT, R15
wait_clear_loop:
    BIT.B R14, 0(R11)
    JZ wait_clear_ok
    DEC R15
    JNZ wait_clear_loop
    MOV #1, R12
    RET
wait_clear_ok:
    CLR R12
    RET

i2c_start:                      ; R12 holds the register. Send START and write the register address.
    PUSH R12
    MOV #UCB0STAT, R11
    MOV #0x10, R14              ; UCBBUSY
    CALL #wait_clear
    TST R12
    JNZ i2c_start_error
    CALL #i2c_init
    BIS.B #0x10, &UCB0CTL1      ; UCTR
    BIS.B #0x02, &UCB0CTL1      ; UCTXSTT
    MOV #IFG2, R11
    MOV #0x08, R14              ; UCB0TXIFG
    CALL #wait_set
    TST R12
    JNZ i2c_start_error
    POP R12
    MOV.B R12, &UCB0TXBUF
    CALL #wait_set
    TST R12
    JNZ i2c_start_done
    MOV #UCB0CTL1, R11
    MOV #0x02, R14              ; UCTXSTT has cleared.
    JMP wait_clear
i2c_start_error:
    INCD SP
i2c_start_done:
    RET

i2c_write:                     ; Register in R12 and value in R13.
    PUSH R13
    CALL #i2c_start
    POP R13
    TST R12
    JNZ i2c_error
    BIS.B #0x10, &UCB0CTL1
    MOV #IFG2, R11
    MOV #0x08, R14
    CALL #wait_set
    TST R12
    JNZ i2c_error
    MOV.B R13, &UCB0TXBUF
    CALL #wait_set
    TST R12
    JNZ i2c_error
    JMP i2c_stop

i2c_read:                       ; Register in R12, destination in R13, byte count in R14.
    PUSH R10
    PUSH R9
    MOV R13, R9
    MOV R14, R10
    CALL #i2c_start
    TST R12
    JNZ i2c_read_error
    BIC.B #0x10, &UCB0CTL1      ; Receive mode.
i2c_read_loop:
    BIS.B #0x02, &UCB0CTL1      ; Issue (re)START for every byte, as in f_5d72.
    MOV #IFG2, R11
    MOV #0x04, R14              ; UCB0RXIFG
    CALL #wait_set
    TST R12
    JNZ i2c_read_error
    MOV.B &UCB0RXBUF, 0(R9)
    CALL #wait_clear
    TST R12
    JNZ i2c_read_error
    INC R9
    DEC R10
    JNZ i2c_read_loop
    POP R9
    POP R10
    JMP i2c_stop
i2c_read_error:
    POP R9
    POP R10
    JMP i2c_error

i2c_stop:
    MOV #I2C_TIMEOUT, R15
stop_loop:
    BIS.B #0x04, &UCB0CTL1      ; UCTXSTP
    BIT.B #0x10, &UCB0STAT
    JZ stop_done
    DEC R15
    JNZ stop_loop
    BIC.B #0x04, &UCB0CTL1
    JMP i2c_error
stop_done:
    BIC.B #0x04, &UCB0CTL1
    CLR R12
    RET

i2c_error:
    INC.B &V_ERR_I2C
    CALL #i2c_init
    MOV #1, R12
    RET

; Data
; Index by upward axis (+X, -X, +Y, -Y, +Z, -Z). Opposite faces sum to seven.
; Adjust the mapping after checking the orientation of the pod in the cube.
TABLE_FACES:
.byte 2, 5, 3, 4, 1, 6
.even
