from bitstring import Bits
import time
import argparse
import struct

def two_comp_tostring(number):
    return Bits(bin=bin(number)[2:]).int

def INPUT(PORT):
    print("INPUT(0x%02X):" % (PORT,))
    return 0

def OUTPUT(port, val):
    print("OUTPUT(0x%02X): %d" % (port, val))

class SCOMP_STATE:
    def __init__(self):
        self.PC = 0
        self.MAR = 0
        self.AC = 0
        self.memory = [0]*2048
        self.stack = []
        self.EI = False
        self.INT=0
        pass

    def disassemble(self, opcode, data):
        imm = (data | (-(data & (1 << 10)))) & 0xFFFF
        if opcode == 0x0:
            return "NOP"
        elif opcode == 0x1:
            return "LOAD " + "[%04X]" % (data,)
        elif opcode == 0x2:
            return "STORE [%04X]" % (data,)
        elif opcode == 0x3: # ADD
            return "ADD %04X" % (two_comp_tostring(imm),)
        elif opcode == 0x4: # SUB
            return "SUB %04X" % (two_comp_tostring(imm),)
        elif opcode == 0x5: # JUMP
            return "JUMP %04X" % (data,)
        elif opcode == 0x6: # JNEG
            return "JNEG %04X" % (data,)
        elif opcode == 0x7: # JPOS
            return "JPOS %04X" % (data,)
        elif opcode == 0x8: # JZERO
            return "JZERO %04X" % (data,)
        elif opcode == 0x9: # AND
            return "AND [%04X]" % (imm, )
        elif opcode == 0xa: # OR
            return "OR [%04X]" % (imm, )
        elif opcode == 0xb: # XOR
            return "XOR [%04X]" % (imm, )
        elif opcode == 0xc: # SHIFT
            if (imm & 0x8000) > 0:
                return "RSHIFT %d" % (imm & 0xF,)
            else:
                return "LSHIFT %d" % (imm & 0xF,)
        elif opcode == 0xd: # ADDI
            return "XOR %04X" % (imm, )
        elif opcode == 0xe: # ILOAD
            return "ILOAD %04X" % (data, )
        elif opcode == 0xf: # ISTORE
            return "ISTORE %04X" % (data, )
        elif opcode == 0x10: # CALL
            return "CALL %04X" % (data, )
        elif opcode == 0x11: # RET
            return "RET %04X" % (data, )
        elif opcode == 0x12: # IN
            return "IN %02X" % (data & 0xFF,)
        elif opcode == 0x13: # OUT
            return "OUT %02X" % (data & 0xFF,)
        elif opcode == 0x14: # CLI (DI)
            return "CLI (DI)"
        elif opcode == 0x15: # SEI (EI)
            return "SEI (EI)"
        elif opcode == 0x16: # RETI
            return "RETI"
        elif opcode == 0x17: # LOADI
            return "LOADI %04X (%d)" % (imm, two_comp_tostring(imm),)

    def execute_instruction(self, opcode, data):
        imm = (data | (-(data&(1<<10)))) & 0xFFFF
        if opcode == 0x0:
            pass
        elif opcode == 0x1: # LOAD
            self.AC = self.memory[data]
        elif opcode == 0x2: # STORE
            self.memory[data] = self.AC
        elif opcode == 0x3: # ADD
            self.AC += imm
        elif opcode == 0x4: # SUB
            self.AC -= imm
        elif opcode == 0x5: # JUMP
            self.PC = data
        elif opcode == 0x6: # JNEG
            if self.AC < 0: self.PC = imm
        elif opcode == 0x7: # JPOS
            if self.AC > 0: self.PC = imm
        elif opcode == 0x8: # JZERO
            if self.AC == 0: self.PC = data
        elif opcode == 0x9: # AND
            self.AC &= self.memory[data]
        elif opcode == 0xa: # OR
            self.AC |= self.memory[data]
        elif opcode == 0xb: # XOR
            self.AC ^= self.memory[data]
        elif opcode == 0xc: # SHIFT
            if (imm & 0x8000) > 0:
                self.AC //= (1 << (imm & 0xF))
            else:
                self.AC <<= (imm & 0xF)
        elif opcode == 0xd: # ADDI
            self.AC = self.AC + imm
        elif opcode == 0xe: # ILOAD
            self.AC = self.memory[self.memory[data]]
        elif opcode == 0xf: # ISTORE
            self.memory[self.memory[data]] = self.AC
        elif opcode == 0x10: # CALL
            self.stack.append(self.PC)
            self.PC = data
        elif opcode == 0x11: # RET
            self.PC = self.stack.pop()
        elif opcode == 0x12: # IN
            self.AC = INPUT(data & 0xFF)
            pass
        elif opcode == 0x13:
            OUTPUT(data & 0xFF, self.AC)
            pass
        elif opcode == 0x14: # CLI (DI)
            self.EI = False
            pass
        elif opcode == 0x15: # SEI (EI)
            self.EI = True
            pass
        elif opcode == 0x16: # RETI
            self.PC = self.INT
            self.INT = None
        elif opcode == 0x17: # LOADI
            self.PC = imm
        self.AC &= 0xFFFF
        self.PC &= 0xFFFF
        return "%04X: %s" % (self.PC, self.disassemble(opcode, data))


    def step(self):
        mem_data = self.memory[self.PC]
        opcode = (mem_data >> 11) & 0x1F
        data = (mem_data & 0x7FF)
        disassembled = self.execute_instruction(opcode, data)
        print(disassembled)
        self.PC += 1
        self.PC &= 0x7FF
        pass

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "PC = 0x%04X; MAR = 0x%04X; AC = 0x%04X" % (self.PC, self.MAR, self.AC)


if __name__ == '__main__':
    argp = argparse.ArgumentParser()
    argp.add_argument("file", type=str)
    args = argp.parse_args()

    if args.file is None:
        exit(1)

    scomp = SCOMP_STATE()
    # LOAD
    with open(args.file, 'rb') as file:
        while True:
            b1 = file.read(2)
            if len(b1) != 2:
                break
            val = struct.unpack(">H", b1)
            print("%04X" % (val[0],))
        pass


    scomp.memory[0] = 0x0
    while True:
        scomp.step()
    print(scomp)
