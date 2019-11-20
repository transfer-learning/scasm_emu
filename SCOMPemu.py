from queue import Queue
import math
from threading import Thread
import numpy as np

from bitstring import Bits
import time
import argparse
import struct
import curses
from sim import SensorInfo, simulation_thread

enable_screen = False
controls_q = Queue(1)
sensor_q = Queue(1)
cur_control = np.asmatrix([[0, 0]]).T
cur_sensors = SensorInfo([5 for i in range(8)], 0, 0)
theta_offset = 0

counter = 0


def get_raw_theta() -> int:
    rad = cur_sensors.theta
    deg = int(rad * 180 / math.pi)
    deg = (deg + 360) % 360
    return deg & 0xFFFF


def two_comp_tostring(number):
    return Bits(uint=number, length=16).int


def put_control_queue(control):
    if controls_q.full():
        controls_q.get_nowait()
    controls_q.put_nowait(control)


class SCOMP_STATE:
    def __init__(self):
        self.PC = 0
        self.AC = 0
        self.memory = [0] * 2048
        self.stack = []
        self.EI = 0
        self.INT = None
        self.INT_AC = None
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
            "RVELCMD": 0x8b,
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
        self.device_mem[self.devices['XIO']] = 0b10000

    def INPUT(self, port) -> int:
        # if not (enable_screen):
        #     print("INPUT(0x%02X):" % (PORT,))
        if self.devices["THETA"] == port:
            deg = get_raw_theta()
            deg = deg - theta_offset
            deg = (deg + 360) % 360
            print(f"theta: {deg}")
            return deg & 0xFFFF
        if self.devices["DIST0"] <= port <= self.devices["DIST7"]:
            num = port - self.devices["DIST0"]
            reading = int(1000 * cur_sensors.sonar[num])
            if reading == 5000:
                reading = 0x7FFF
            if num == 0:
                print(f"Sonar {num}: {reading}, original: {cur_sensors.sonar[num]}")
            # if reading < 400:
            #     print(f"SONAR {num}: {reading}")
            return reading & 0x7FFF
        return self.device_mem[port]

    def OUTPUT(self, port, val):
        global theta_offset
        # if not (enable_screen):
        #     print("OUTPUT(0x%02X): 0x%04X (%d)" % (port, val, two_comp_tostring(val)))

        if self.devices["SSEG1"] == port:
            print("SSEG1: 0x%04X (%d)" % (val, two_comp_tostring(val)))
        if self.devices["SSEG2"] == port:
            print("SSEG2: 0x%04X (%d)" % (val, two_comp_tostring(val)))
        if self.devices["LCD"] == port:
            print("LCD: 0x%04X (%d)" % (val, two_comp_tostring(val)))
        if self.devices['TIMER'] == port:
            self.device_mem[self.devices['TIMER']] = 0
        if self.devices["LVELCMD"] == port:
            cur_control[0, 0] = two_comp_tostring(val)
            put_control_queue(cur_control)
            # print(f"[L] Trying to out {cur_control.T}")
        if self.devices["RVELCMD"] == port:
            cur_control[1, 0] = two_comp_tostring(val)
            put_control_queue(cur_control)
            # print(f"[R] Trying to out {cur_control.T}")
        if self.devices["RESETPOS"] == port:
            theta_offset = get_raw_theta()
            print(f"resetpos called. theta offset: {theta_offset}")

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
            return "NOP (0x%04X)" % (opcode << 11 | data)
        elif opcode == 0x1:
            return "LOAD " + "[0x%04X]" % (data,)
        elif opcode == 0x2:
            return "STORE [0x%04X]" % (data,)
        elif opcode == 0x3:  # ADD
            return "ADD [0x%04X] (%d)" % (data, self.memory[data],)
        elif opcode == 0x4:  # SUB
            return "SUB [0x%04X]" % (two_comp_tostring(imm),)
        elif opcode == 0x5:  # JUMP
            return "JUMP 0x%04X" % (data,)
        elif opcode == 0x6:  # JNEG
            return "JNEG 0x%04X" % (data,)
        elif opcode == 0x7:  # JPOS
            return "JPOS 0x%04X" % (data,)
        elif opcode == 0x8:  # JZERO
            return "JZERO 0x%04X" % (data,)
        elif opcode == 0x9:  # AND
            return "AND [0x%04X]" % (imm,)
        elif opcode == 0xa:  # OR
            return "OR [0x0x%04X]" % (imm,)
        elif opcode == 0xb:  # XOR
            return "XOR [0x%04X]" % (imm,)
        elif opcode == 0xc:  # SHIFT
            if (imm & 0x8000) > 0:
                return "RSHIFT %d" % (imm & 0xF,)
            else:
                return "LSHIFT %d" % (imm & 0xF,)
        elif opcode == 0xd:  # ADDI
            return "ADDI 0x%04X" % (imm,)
        elif opcode == 0xe:  # ILOAD
            return "ILOAD 0x%04X" % (data,)
        elif opcode == 0xf:  # ISTORE
            return "ISTORE 0x%04X" % (data,)
        elif opcode == 0x10:  # CALL
            return "CALL 0x%04X" % (data,)
        elif opcode == 0x11:  # RET
            return "RET 0x%04X" % (data,)
        elif opcode == 0x12:  # IN
            return "IN 0x%02X" % (data & 0xFF,)
        elif opcode == 0x13:  # OUT
            return "OUT 0x%02X" % (data & 0xFF,)
        elif opcode == 0x14:  # CLI (DI)
            return "CLI (DI) %s" % (Bits(uint=data & 0xF, length=4).bin,)
        elif opcode == 0x15:  # SEI (EI)
            return "SEI (EI) %s" % (Bits(uint=data & 0xF, length=4).bin,)
        elif opcode == 0x16:  # RETI
            return "RETI"
        elif opcode == 0x17:  # LOADI
            return "LOADI %04X (%d)" % (imm, two_comp_tostring(imm),)

    def execute_instruction(self, opcode, data):
        imm = (data | (-(data & (1 << 10)))) & 0xFFFF
        if opcode == 0x0:
            pass
        elif opcode == 0x1:  # LOAD
            self.AC = self.memory[data]
        elif opcode == 0x2:  # STORE
            self.memory[data] = self.AC
        elif opcode == 0x3:  # ADD
            self.AC += self.memory[data]
        elif opcode == 0x4:  # SUB
            self.AC -= self.memory[data]
        elif opcode == 0x5:  # JUMP
            self.PC = data
        elif opcode == 0x6:  # JNEG
            if two_comp_tostring(self.AC) < 0:
                self.PC = imm
        elif opcode == 0x7:  # JPOS
            # print(f"AC: {self.AC}, comp: {two_comp_tostring(self.AC)}")
            if two_comp_tostring(self.AC) > 0:
                self.PC = imm
        elif opcode == 0x8:  # JZERO
            if self.AC == 0:
                self.PC = data
        elif opcode == 0x9:  # AND
            self.AC &= self.memory[data]
        elif opcode == 0xa:  # OR
            self.AC |= self.memory[data]
        elif opcode == 0xb:  # XOR
            self.AC ^= self.memory[data]
        elif opcode == 0xc:  # SHIFT
            if (imm & 0x8000) > 0:
                self.AC = (two_comp_tostring(self.AC) >> (imm & 0xF)) & 0xFFFF
            else:
                self.AC <<= (imm & 0xF)
        elif opcode == 0xd:  # ADDI
            self.AC = self.AC + imm
        elif opcode == 0xe:  # ILOAD
            self.AC = self.memory[self.memory[data]]
        elif opcode == 0xf:  # ISTORE
            self.memory[self.memory[data]] = self.AC
        elif opcode == 0x10:  # CALL
            self.stack.append(self.PC)
            self.PC = data
        elif opcode == 0x11:  # RET
            self.PC = self.stack.pop()
        elif opcode == 0x12:  # IN
            self.AC = self.INPUT(data & 0xFF)
            pass
        elif opcode == 0x13:
            self.OUTPUT(data & 0xFF, self.AC)
            pass
        elif opcode == 0x14:  # CLI (DI)
            self.EI &= (~(data & 0xF) & 0xF)
            pass
        elif opcode == 0x15:  # SEI (EI)
            self.EI |= data & 0xF
            pass
        elif opcode == 0x16:  # RETI
            if self.INT is not None:
                self.PC = self.INT
                self.AC = self.INT_AC
                self.INT = None
        elif opcode == 0x17:  # LOADI
            self.AC = imm
        self.AC &= 0xFFFF
        self.PC &= 0xFFFF
        return self.disassemble(opcode, data)

    def interrupt(self, location):  # location = target address (1-4)
        self.IRQ = location
        pass

    def instruction_at_location(self, loc):
        mem_data = self.memory[loc]
        opcode = (mem_data >> 11) & 0x1F
        data = (mem_data & 0x7FF)
        return self.disassemble(opcode, data)

    def check_interrupt(self):
        # TIMER
        if self.device_mem[self.devices['CTIMER']] > 0:
            if self.ticks % (TICKS_PER_TEN_MS * self.device_mem[self.devices['CTIMER']]) == 0:
                self.interrupt(2)

    def step(self):
        global counter
        if self.IRQ is not None:
            if self.EI and self.INT is None:
                self.INT = self.PC
                self.INT_AC = self.AC
                self.AC = 0
                self.PC = self.IRQ
                print(f'INT HAPPEN {counter}')
                counter += 1
            self.IRQ = None
        mem_data = self.memory[self.PC]
        opcode = (mem_data >> 11) & 0x1F
        data = (mem_data & 0x7FF)
        self.PC = (self.PC + 1) & 0x7FF
        disassembled = self.execute_instruction(opcode, data)
        self.ticks += 1
        self.check_interrupt()
        if self.ticks % 400000 == 0:
            self.device_mem[self.devices['TIMER']] += 1
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


TICKS_PER_TEN_MS = 40


def print_screen(screen, core_win, scomp, run):
    time.sleep(0.01)
    cmd = screen.getch(0, 0)
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
                val = "(  ) 0x%04X: %s" % (scomp.PC + i, scomp.instruction_at_location(scomp.PC + i),)
            else:
                val = "(->) 0x%04X: %s" % (scomp.PC + i, scomp.instruction_at_location(scomp.PC + i),)
            core_win.addstr(14 + i, 0, val)
    core_win.refresh()
    screen.refresh()


def get_sensor_data():
    global cur_sensors

    if not sensor_q.empty():
        cur_sensors = sensor_q.get_nowait()
        # print(f"cur_sensors: {cur_sensors.sonar}")


def main():
    global enable_screen

    argp = argparse.ArgumentParser()
    argp.add_argument("file", type=str)
    argp.add_argument("-sim", action='store_true')
    args = argp.parse_args()

    if args.file is None:
        exit(1)

    # simulator thread
    sim_thread = None
    if args.sim:
        sim_thread = Thread(target=simulation_thread, args=(controls_q, sensor_q))
        sim_thread.start()

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
        if not enable_screen:
            dis = scomp.step()
            # print("0x%04X: %s == AC: 0x%04X" % (scomp.PC, scomp.instruction_at_location(scomp.PC ), scomp.AC))
        else:
            print_screen(screen, core_win, scomp, run)
        get_sensor_data()
        time.sleep(0.00001)
    print(scomp)


if __name__ == '__main__':
    main()
