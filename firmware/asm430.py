#!/usr/bin/env python3
"""Minimal assembler for the RunPod die firmware (base MSP430 instruction set).

Usage: python3 -I asm430.py SOURCE.s OUTPUT.mp [--list LISTING.txt]

TI-style syntax
  ; comment
  label:
  .equ NAME, value          define a constant
  .org ADDRESS              advance the current address
  .word v1, v2, …            16-bit words
  .byte v1, v2, …            bytes
  .even                     align to a word boundary
  MOV.B #0x12, &P1OUT       instruction (.B or .W suffix)

Operands: Rn (R0–R15, PC, SP, SR), @Rn, @Rn+, #expr, &expr, expr(Rn).
Jumps (JMP, JEQ, etc.) take a label. Use &label or #label for other
references. PC-relative symbolic addressing is unsupported.
Expressions use Python operators (+, -, |, &, <<, >>, ~), constants,
and labels. Immediate sources can use the constant generator values
#0, #1, #2, #4, #8, and #-1.

The .mp output starts with a 16-bit length and Fletcher-16 checksum.
The image starts with the vector table at 0x2104 and enters at 0x2124.
"""
import re
import struct
import sys

REGS = {f"R{i}": i for i in range(16)}
REGS.update({"PC": 0, "SP": 1, "SR": 2, "CG": 3})
F1 = {"MOV": 4, "ADD": 5, "ADDC": 6, "SUBC": 7, "SUB": 8, "CMP": 9, "DADD": 10,
      "BIT": 11, "BIC": 12, "BIS": 13, "XOR": 14, "AND": 15}
F2 = {"RRC": 0, "SWPB": 1, "RRA": 2, "SXT": 3, "PUSH": 4, "CALL": 5, "RETI": 6}
JUMPS = {"JNE": 0, "JNZ": 0, "JEQ": 1, "JZ": 1, "JNC": 2, "JLO": 2, "JC": 3, "JHS": 3,
         "JN": 4, "JGE": 5, "JL": 6, "JMP": 7}
# Emulated mnemonics map to a real instruction and operands derived from {d}.
EMULATED = {
    "NOP": ("MOV", "#0, R3"), "RET": ("MOV", "@SP+, PC"), "EINT": ("BIS", "#8, SR"),
    "DINT": ("BIC", "#8, SR"), "CLRC": ("BIC", "#1, SR"), "SETC": ("BIS", "#1, SR"),
    "POP": ("MOV", "@SP+, {d}"), "BR": ("MOV", "{d}, PC"), "CLR": ("MOV", "#0, {d}"),
    "INC": ("ADD", "#1, {d}"), "INCD": ("ADD", "#2, {d}"), "DEC": ("SUB", "#1, {d}"),
    "DECD": ("SUB", "#2, {d}"), "TST": ("CMP", "#0, {d}"), "INV": ("XOR", "#-1, {d}"),
    "RLA": ("ADD", "{d}, {d}"), "RLC": ("ADDC", "{d}, {d}"), "ADC": ("ADDC", "#0, {d}"),
    "SBC": ("SUBC", "#0, {d}"),
}


class AssemblyError(Exception):
    pass


def evaluate(expr, symbols, pass_number):
    expr = expr.strip()
    try:
        return int(eval(expr, {"__builtins__": {}}, symbols)) & 0xFFFF
    except NameError:
        if pass_number == 1:
            return 0
        raise AssemblyError(f"unknown symbol in {expr!r}")
    except Exception as e:
        raise AssemblyError(f"invalid expression {expr!r}: {e}")


def split_operands(text):
    parts, depth, current = [], 0, ""
    for c in text:
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
        if c == "," and depth == 0:
            parts.append(current.strip())
            current = ""
        else:
            current += c
    if current.strip():
        parts.append(current.strip())
    return parts


def operand(text, symbols, pass_number, source, is_byte):
    """Return the As/Ad mode, register, and extension word if present."""
    t = text.strip()
    if t.upper() in REGS:
        return 0, REGS[t.upper()], None
    if t.startswith("@"):
        r = t[1:].rstrip("+").strip().upper()
        if r not in REGS:
            raise AssemblyError(f"unknown register {t!r}")
        return (3 if t.endswith("+") else 2), REGS[r], None
    if t.startswith("#"):
        v = evaluate(t[1:], symbols, pass_number)
        if source and pass_number == 2 or source:
            cg = {0: (0, 3), 1: (1, 3), 2: (2, 3), 4: (2, 2), 8: (3, 2), 0xFFFF: (3, 3)}
            vv = v & (0xFF if is_byte else 0xFFFF)
            if is_byte and vv == 0xFF:
                vv = 0xFFFF
            # The constant generator can only be selected consistently in both passes
            # when the expression contains no unresolved symbols.
            if source and vv in cg and not re.search(r"[A-Za-z_]", t[1:].replace("0x", "")):
                m, r = cg[vv]
                return m, r, None
        return 3, 0, v
    if t.startswith("&"):
        return 1, 2, evaluate(t[1:], symbols, pass_number)
    m = re.match(r"^(.*)\((\w+)\)$", t)
    if m and m.group(2).upper() in REGS:
        return 1, REGS[m.group(2).upper()], evaluate(m.group(1) or "0", symbols, pass_number)
    raise AssemblyError(f"unsupported operand {t!r} (use &address or #value)")


def assemble_line(mnem, ops_txt, address, symbols, pass_number):
    mnem = mnem.upper()
    is_byte = mnem.endswith(".B")
    base = mnem[:-2] if mnem.endswith((".B", ".W")) else mnem
    if base in EMULATED:
        real_instruction, template = EMULATED[base]
        ops_txt = template.replace("{d}", ops_txt.strip()) if "{d}" in template else template
        base = real_instruction
    ops = split_operands(ops_txt)
    words = []
    if base in JUMPS:
        target = evaluate(ops[0], symbols, pass_number)
        displacement = (target - (address + 2)) // 2 if pass_number == 2 else 0
        if pass_number == 2 and not -512 <= displacement <= 511:
            raise AssemblyError(f"jump target {ops[0]} is out of range")
        return [0x2000 | (JUMPS[base] << 10) | (displacement & 0x3FF)]
    if base in F2:
        if base == "RETI":
            return [0x1300]
        m, r, ext = operand(ops[0], symbols, pass_number, source=True, is_byte=is_byte)
        words = [0x1000 | (F2[base] << 7) | (int(is_byte) << 6) | (m << 4) | r]
        if ext is not None:
            words.append(ext)
        return words
    if base in F1:
        if len(ops) != 2:
            raise AssemblyError(f"{base} takes two operands")
        ms, rs, es = operand(ops[0], symbols, pass_number, source=True, is_byte=is_byte)
        md, rd, ed = operand(ops[1], symbols, pass_number, source=False, is_byte=is_byte)
        if md in (2, 3):
            if md == 2 and rd != 0:      # Destination @Rn becomes 0(Rn).
                md, ed = 1, 0
            else:
                raise AssemblyError(f"unsupported destination {ops[1]!r}")
        if md == 3:
            raise AssemblyError("immediate destination is invalid")
        words = [(F1[base] << 12) | (rs << 8) | (md << 7) | (int(is_byte) << 6) | (ms << 4) | rd]
        if es is not None:
            words.append(es)
        if ed is not None:
            words.append(ed)
        return words
    raise AssemblyError(f"unknown instruction {mnem!r}")


def assemble(source):
    """Return the address-to-byte map, symbols, and listing."""
    lines = source.splitlines()
    symbols = {}
    for pass_number in (1, 2):
        mem, listing = {}, []
        address = 0
        for num, raw_line in enumerate(lines, 1):
            line = raw_line.split(";")[0].rstrip()
            try:
                m = re.match(r"^\s*([A-Za-z_]\w*):(.*)$", line)
                if m:
                    if pass_number == 1:
                        if m.group(1) in symbols:
                            raise AssemblyError(f"duplicate label {m.group(1)!r}")
                        symbols[m.group(1)] = address
                    line = m.group(2)
                line = line.strip()
                if not line:
                    continue
                mnem, _, remaining = line.partition(" ")
                directive = mnem.lower()
                if directive == ".equ":
                    name, val = [x.strip() for x in remaining.split(",", 1)]
                    symbols[name] = evaluate(val, symbols, 2)
                    continue
                if directive == ".org":
                    new_address = evaluate(remaining, symbols, 2)
                    if new_address < address:
                        raise AssemblyError(".org cannot decrease the address")
                    address = new_address
                    continue
                if directive == ".even":
                    address += address & 1
                    continue
                if directive in (".word", ".byte"):
                    for v in split_operands(remaining):
                        val = evaluate(v, symbols, pass_number)
                        if directive == ".word":
                            if address & 1:
                                raise AssemblyError(".word at an odd address")
                            mem[address], mem[address + 1] = val & 0xFF, val >> 8
                            address += 2
                        else:
                            mem[address] = val & 0xFF
                            address += 1
                    continue
                if address & 1:
                    raise AssemblyError("instruction at an odd address (add .even)")
                words = assemble_line(mnem, remaining, address, symbols, pass_number)
                if pass_number == 2:
                    listing.append(f"{address:04x}: {' '.join(f'{w & 0xFF:02x} {w >> 8:02x}' for w in words):<30} {line}")
                for w in words:
                    if address in mem:
                        raise AssemblyError(f"overlapping data at 0x{address:04x}")
                    mem[address], mem[address + 1] = w & 0xFF, w >> 8
                    address += 2
            except AssemblyError as e:
                raise AssemblyError(f"line {num}: {e}\n  {raw_line.strip()}")
    return mem, symbols, listing


def fletcher(data):
    s1 = s2 = 0xFF
    for i in range(0, len(data), 20):
        for b in data[i:i + 20]:
            s1 += b
            s2 += s1
        s1 = (s1 & 0xFF) + (s1 >> 8)
        s2 = (s2 & 0xFF) + (s2 >> 8)
    s1 = (s1 & 0xFF) + (s1 >> 8)
    s2 = (s2 & 0xFF) + (s2 >> 8)
    return ((s2 & 0xFF) << 8) | (s1 & 0xFF)


def image_mp(mem):
    """Build an .mp header and body starting at 0x2104, padded to even length."""
    if min(mem) < 0x2104:
        raise AssemblyError("the code must start at 0x2104")
    end = max(mem) + 1
    end += end & 1
    body = bytes(mem.get(a, 0xFF) for a in range(0x2104, end))
    return struct.pack("<HH", len(body), fletcher(body)) + body


def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        return
    if len(args) != 2 and not (len(args) == 4 and args[2] == "--list"):
        raise SystemExit(__doc__)
    src = open(args[0], encoding="utf-8").read()
    mem, symbols, listing = assemble(src)
    mp = image_mp(mem)
    open(args[1], "wb").write(mp)
    if len(args) == 4:
        open(args[3], "w").write("\n".join(listing) + "\n")
    print(f"{args[1]}: {len(mp) - 4} bytes (0x2104–0x{0x2104 + len(mp) - 5:04x}),"
          f" Fletcher-16 0x{struct.unpack_from('<H', mp, 2)[0]:04x}")


if __name__ == "__main__":
    main()
