from bitstring import Bits
import time
import argparse
import struct
import curses

enable_screen = False

def two_comp_tostring(number):
    return Bits(uint=number, length=16).int

class SCOMP_STATE:
    def __init__(self):
        self.PC = 0
        self.AC = 0
        self.memory = [0]*2048
        self.stack = []
        self.EI = 0
        self.INT = None
        self.IRQ = None
        self.ticks = 0

        self.device_mem = [0x0] * 256

        self.devices = {
            "SWITCHES": 0x0,
            "LEDS": 0x01,
            "TIMER": 0x02,
            "XIO": 0x03,
            "SSEG1": 0x04,
            "SSEG2": 0x05,
            "LCD": 0x06,
            "XLEDS": 0x07,
            "BEEP": 0x0A,
            "CTIMER": 0x0C,
            "LPOS": 0x80,
            "LVEL": 0x82,
            "LVELCMD": 0x83,
            "I2C_CMD": 0x90,
            "I2C_DATA": 0x91,
            "I2C_RDY": 0x92,
            "UART_DAT": 0x98,
            "UART_RDY": 0x99,
            "DIST0": 0xA8,
            "DIST1": 0xA9,
            "DIST2": 0xAA,
            "DIST3": 0xAB,
            "DIST4": 0xAC,
            "DIST5": 0xAD,
            "DIST6": 0xAE,
            "DIST7": 0xAF,
            "SONARALARM": 0xB0,
            "SONARINT": 0xB1,
            "SONAREN": 0xB2,
            "XPOS": 0xC0,
            "YPOS": 0xC1,
            "THETA": 0xC2,
            "RESETPOS": 0xC3,
            "IR_HI": 0xD0,
            "IR_LO": 0xD1
        }

    def INPUT(self, PORT):
        if not (enable_screen):
            print("INPUT(0x%02X):" % (PORT,))
        return self.device_mem[PORT]

    def OUTPUT(self, port, val):
        if not (enable_screen):
            print("OUTPUT(0x%02X): %d" % (port, val))

        if self.devices["SSEG1"] == port:
            print(f"SSEG1: {val}")
        if self.devices["SSEG2"] == port:
            print(f"SSEG2: {val}")

        self.device_mem[port] = val

    def reset(self):
        self.PC = 0
        self.AC = 0
        self.stack = []
        self.EI = 0
        self.INT = None
        self.IRQ = None

    def disassemble(self, opcode, data):
        imm = (data | (-(data & (1 << 10)))) & 0xFFFF
        if opcode == 0x0:
            return "NOP"
        elif opcode == 0x1:
            return "LOAD " + "[%04X]" % (data,)
        elif opcode == 0x2:
            return "STORE [%04X]" % (data,)
        elif opcode == 0x3: # ADD
            return "ADD [%04X] (%d)" % (data, self.memory[data],)
        elif opcode == 0x4: # SUB
            return "SUB [%04X]" % (two_comp_tostring(imm),)
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
            return "CLI (DI) %s" % (Bits(uint=data & 0xF, length=4).bin,)
        elif opcode == 0x15: # SEI (EI)
            return "SEI (EI) %s" % (Bits(uint=data & 0xF, length=4).bin,)
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
            self.AC += self.memory[data]
        elif opcode == 0x4: # SUB
            self.AC -= self.memory[data]
        elif opcode == 0x5: # JUMP
            self.PC = data
        elif opcode == 0x6: # JNEG
            if self.AC < 0:
                self.PC = imm
        elif opcode == 0x7: # JPOS
            if self.AC > 0:
                self.PC = imm
        elif opcode == 0x8: # JZERO
            if self.AC == 0:
                self.PC = data
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
            self.AC = self.INPUT(data & 0xFF)
            pass
        elif opcode == 0x13:
            self.OUTPUT(data & 0xFF, self.AC)
            pass
        elif opcode == 0x14: # CLI (DI)
            self.EI &= (~(data & 0xF) & 0xF)
            pass
        elif opcode == 0x15: # SEI (EI)
            self.EI |= data & 0xF
            pass
        elif opcode == 0x16: # RETI
            if self.INT is not None:
                self.PC = self.INT
                self.INT = None
        elif opcode == 0x17: # LOADI
            self.AC = imm
        self.AC &= 0xFFFF
        self.PC &= 0xFFFF
        return self.disassemble(opcode, data)

    def interrupt(self, location): # location = target address (1-4)
        self.IRQ = location
        pass

    def instruction_at_location(self, loc):
        mem_data = self.memory[loc]
        opcode = (mem_data >> 11) & 0x1F
        data = (mem_data & 0x7FF)
        return self.disassemble(opcode, data)

    def check_interrupt(self):
        # TIMER
        if self.ticks % (TICKS_PER_SEC / 10):
            self.interrupt(2)

    def step(self):
        if self.IRQ is not None:
            if self.EI and self.INT is None:
                self.INT = self.PC
                self.PC = self.IRQ
            self.IRQ = None
        mem_data = self.memory[self.PC]
        opcode = (mem_data >> 11) & 0x1F
        data = (mem_data & 0x7FF)
        self.PC = (self.PC + 1) & 0x7FF
        disassembled = self.execute_instruction(opcode, data)
        self.ticks += 1
        self.check_interrupt()
        return disassembled

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "PC = 0x%04X; AC = 0x%04X (%d); INT_VEC: %s" % \
               (self.PC, self.AC, two_comp_tostring(self.AC), Bits(uint=self.EI & 0xF, length=4).bin)

    def lookup(self, param, signed=True):
        if signed:
            return two_comp_tostring(self.device_mem[self.devices[param]])
        else:
            return self.device_mem[self.devices[param]]

    def set(self, param, val):
        self.device_mem[self.devices[param]] = val & 0xFFFF


TICKS_PER_SEC = 4500000

if __name__ == '__main__':
    argp = argparse.ArgumentParser()
    argp.add_argument("file", type=str)
    args = argp.parse_args()

    if args.file is None:
        exit(1)

    scomp = SCOMP_STATE()
    # LOAD
    with open(args.file, 'rb') as file:
        addr = 0
        while True:
            b1 = file.read(2)
            if len(b1) != 2:
                break
            val = struct.unpack(">H", b1)
            scomp.memory[addr] = val[0]
            addr += 1
        print("Program Loaded: %d words" % (addr - 1))

    try:
        screen = curses.initscr()
        enable_screen = True
    except:
        enable_screen = False

    if enable_screen:
        screen.clear()
        screen.nodelay(True)
        core_win = curses.newwin(60, 60, 0, 0)

    run = False
    while True:
        if scomp.PC == 0x7FF:
            break
        if not(enable_screen):
            dis = scomp.step()
            # print('%s --> %s' % (scomp.__str__(), dis), end='\n')
        else:
            cmd = screen.getch(0,0)
            if cmd == ord('s'):
                if not run:
                    dis = scomp.step()
            elif cmd == ord('r'):
                run = not run
            elif cmd == ord('R'):
                run = False
                scomp.reset()
            elif cmd == ord('q'):
                exit(0)
            dis = scomp.instruction_at_location(scomp.PC)
            if run:
                dis = scomp.step()
            core_win.clear()
            core_win.addstr(0, 0, scomp.__str__())
            core_win.addstr(1, 0, dis)
            core_win.addstr(2, 0, "STACK SIZE : %d" % (len(scomp.stack),))
            core_win.hline(3, 0, '-', 60)
            for i in range(-10, 20, 1):
                val = ""
                if (scomp.PC + i) >= 0:
                    if i != 0:
                        val = "(  ) %04X: %s" % (scomp.PC + i, scomp.instruction_at_location(scomp.PC + i),)
                    else:
                        val = "(->) %04X: %s" % (scomp.PC + i, scomp.instruction_at_location(scomp.PC + i),)
                    core_win.addstr(14+i, 0, val)
            core_win.refresh()
            screen.refresh()
    print(scomp)
