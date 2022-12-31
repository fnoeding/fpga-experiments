import pytest

from amaranth import Signal, Module, Elaboratable, ClockDomain, signed, Mux
from amaranth.build import Platform
from amaranth.cli import main
from amaranth.sim import Simulator, Settle, Tick


OPCODE_OP_IMM = 0b0010011
OPCODE_OP = 0b0110011
OPCODE_LOAD = 0b0000011
OPCODE_STORE = 0b0100011
OPCODE_BRANCH = 0b1100011
OPCODE_CUSTOM0 = 0b0001011
OPCODE_LUI = 0b0110111
OPCODE_AUIPC = 0b0010111
OPCODE_JALR = 0b1100111
OPCODE_JAL = 0b1101111


FUNCT3_OP_ADD = 0b000
FUNCT3_OP_SLL = 0b001
FUNCT3_OP_SLT = 0b010
FUNCT3_OP_SLTU = 0b011
FUNCT3_OP_XOR = 0b100
FUNCT3_OP_SRL = 0b101
FUNCT3_OP_OR = 0b110
FUNCT3_OP_AND = 0b111


def SignedSignal(bits):
    return Signal(shape=signed(bits))


class ALU(Elaboratable):
    def __init__(self):
        self.i_funct3 = Signal(3)
        self.i_funct7 = Signal(7)
        self.i_data1 = SignedSignal(32)
        self.i_data2 = SignedSignal(32)
        self.o_result = SignedSignal(32)

        self.ports = [self.i_funct3, self.i_funct7, self.i_data1, self.i_data2, self.o_result]

    def elaborate(self, _: Platform) -> Module:
        """
        purely combinational, returns the result of "data1 $OP data2"
        """

        m = Module()

        with m.Switch(self.i_funct3):
            with m.Case(FUNCT3_OP_ADD):
                m.d.comb += self.o_result.eq(self.i_data1 + self.i_data2)
                # TODO need funct7 support
            with m.Case(FUNCT3_OP_SLL):
                shift_amount = self.i_data2[0:5]
                m.d.comb += self.o_result.eq(self.i_data1.as_unsigned() << shift_amount)
            with m.Case(FUNCT3_OP_SLT):
                m.d.comb += self.o_result.eq(self.i_data1 < self.i_data2)
            with m.Case(FUNCT3_OP_SLTU):
                m.d.comb += self.o_result.eq(self.i_data1.as_unsigned() < self.i_data2.as_unsigned())
            with m.Case(FUNCT3_OP_XOR):
                m.d.comb += self.o_result.eq(self.i_data1 ^ self.i_data2)
            with m.Case(FUNCT3_OP_SRL):
                shift_amount = self.i_data2[0:5]
                m.d.comb += self.o_result.eq(self.i_data1 >> shift_amount)
            with m.Case(FUNCT3_OP_OR):
                m.d.comb += self.o_result.eq(self.i_data1 | self.i_data2)
            with m.Case(FUNCT3_OP_AND):
                m.d.comb += self.o_result.eq(self.i_data1 & self.i_data2)

        return m


class RegisterFile(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")
        # TODO reset

        self.i_select_rs1 = Signal(5)
        self.i_select_rs2 = Signal(5)
        self.i_select_rd = Signal(5)

        self.i_we = Signal(1)
        self.i_data = SignedSignal(32)

        self.o_rs1_value = SignedSignal(32)
        self.o_rs2_value = SignedSignal(32)

        self.registers = Signal(32 * 32)

        self.ports = [self.sync, self.i_select_rs1, self.i_select_rs2, self.i_select_rd, self.i_data, self.i_we]

    def elaborate(self, _: Platform) -> Module:
        """
        on clock edge if i_we is set: stores i_data at reg[i_select_rd]
        combinationally returns register values
        """

        m = Module()

        m.d.comb += [
            self.o_rs1_value.eq(self.registers.word_select(self.i_select_rs1, 32)),
            self.o_rs2_value.eq(self.registers.word_select(self.i_select_rs2, 32)),
        ]

        with m.If((self.i_we == 1) & (self.i_select_rd != 0)):
            m.d.sync += self.registers.word_select(self.i_select_rd, 32).eq(self.i_data)

        return m


class InstructionDecoder(Elaboratable):
    def __init__(self):
        self.i_instruction = Signal(32)

        self.o_rs1 = Signal(5)
        self.o_rs2 = Signal(5)
        self.o_rd = Signal(5)
        self.o_rd_we = Signal(1)
        self.o_funct3 = Signal(3)
        self.o_funct7 = Signal(7)
        self.o_invalid = Signal(1)
        self.o_imm = SignedSignal(32)
        self.o_has_imm = Signal(1)

        self.ports = []
        for x in dir(self):
            if x.startswith("o_") or x.startswith("i_"):
                self.ports.append(getattr(self, x))

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.d.comb += self.o_invalid.eq(0)

        opcode = self.i_instruction[0:7]

        with m.Switch(opcode):
            with m.Case(OPCODE_OP_IMM):
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:11]),
                    self.o_rd_we.eq(1),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(self.i_instruction[20:32]),
                    self.o_has_imm.eq(1),
                    self.o_funct7.eq(0),
                ]
            with m.Case(OPCODE_OP):
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:11]),
                    self.o_rd_we.eq(1),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(self.i_instruction[20:25]),
                    self.o_imm.eq(0),
                    self.o_has_imm.eq(0),
                    self.o_funct7.eq(self.i_instruction[25:32]),
                ]
            with m.Default():
                m.d.comb += self.o_invalid.eq(1)

        return m


class ProgramCounter(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")
        self.o_instruction_address = Signal(32, reset=0x1000)

        self.ports = [self.o_instruction_address]

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.d.sync += self.o_instruction_address.eq(self.o_instruction_address + 4)

        return m


class CPU(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")

        self.alu = ALU()
        self.decoder = InstructionDecoder()
        self.pc = ProgramCounter()
        self.registers = RegisterFile()

        self.i_tmp_instruction = Signal(32)
        self.o_tmp_pc = Signal(32)

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.submodules.alu = self.alu
        m.submodules.decoder = self.decoder
        m.submodules.registers = self.registers
        m.submodules.pc = self.pc

        r = self.registers
        d = self.decoder
        a = self.alu
        p = self.pc
        m.d.comb += [
            # decoder to register file
            r.i_select_rd.eq(d.o_rd),
            r.i_select_rs1.eq(d.o_rs1),
            r.i_select_rs2.eq(d.o_rs2),
            r.i_we.eq(d.o_rd_we),
            # decoder to alu
            a.i_funct3.eq(d.o_funct3),
            a.i_funct7.eq(d.o_funct7),
            # register file to alu
            a.i_data1.eq(r.o_rs1_value),
            # register file / decoder to alu via mux
            a.i_data2.eq(Mux(d.o_has_imm, d.o_imm, r.o_rs2_value)),
            # alu to register file
            r.i_data.eq(a.o_result),
        ]

        # temporary for testing
        m.d.comb += [
            d.i_instruction.eq(self.i_tmp_instruction),
            self.o_tmp_pc.eq(p.o_instruction_address),
        ]

        return m


def test_alu():
    dut = ALU()
    sim = Simulator(dut)

    def check(a, b, expected, signed=True):
        yield dut.i_data1.eq(a)
        yield dut.i_data2.eq(b)
        yield Settle()
        if signed:
            result = yield dut.o_result
        else:
            result = yield dut.o_result.as_unsigned()
        assert result == expected

    def bench():
        # ADD
        yield dut.i_funct3.eq(FUNCT3_OP_ADD)
        yield dut.i_funct7.eq(0)

        yield from check(3, 5, 8)
        yield from check(5, 3, 8)
        yield from check(2 ** 31, 2 ** 31, 0)  # overflow
        yield from check(2 ** 31, 2 ** 31 + 1, 1)  # overflow

        # TODO ADD with FUNCT7 --> SUB
        # ...

        # SLL
        yield dut.i_funct3.eq(FUNCT3_OP_SLL)
        yield dut.i_funct7.eq(0)
        for i in range(32):
            yield from check(1, i, 1 << i, signed=False)

        # SLT
        yield dut.i_funct3.eq(FUNCT3_OP_SLT)
        yield dut.i_funct7.eq(0)
        yield from check(0, 5, 1)
        yield from check(5, 0, 0)
        yield from check(-5, -5, 0)
        yield from check(-5, -4, 1)
        yield from check(2 ** 31 - 1, -(2 ** 31), 0)

        # SLTU
        yield dut.i_funct3.eq(FUNCT3_OP_SLTU)
        yield dut.i_funct7.eq(0)
        yield from check(0, 5, 1)
        yield from check(5, 0, 0)
        yield from check(2 ** 31 + 5, 2 ** 31 + 5, 0)
        yield from check(2 ** 31 + 4, 2 ** 31 + 5, 1)

    sim.add_process(bench)
    sim.run()


def test_register_file():
    dut = RegisterFile()
    sim = Simulator(dut)
    sim.add_clock(1e-6)

    def bench():
        yield dut.i_select_rs1.eq(0)
        yield dut.i_select_rs2.eq(1)
        yield dut.i_select_rd.eq(1)
        yield dut.i_data.eq(42)
        yield dut.i_we.eq(0)
        yield Settle()

        # all registers are reset to zero
        assert (yield dut.o_rs1_value) == 0
        assert (yield dut.o_rs2_value) == 0

        # write to register 1
        yield dut.i_we.eq(1)
        assert (yield dut.o_rs2_value) == 0, "register is only written on pos edge"
        yield Tick()
        yield Settle()
        assert (yield dut.o_rs2_value) == 42

        # write to register 0 - which is hard coded to zero
        yield dut.i_select_rd.eq(0)
        yield Tick()
        yield Settle()
        assert (yield dut.o_rs1_value) == 0, "register zero cannot be written"

        # data is only written on Tick
        yield dut.i_select_rd.eq(1)
        yield dut.i_data.eq(21)
        yield dut.i_we.eq(0)
        yield Tick()
        yield Settle()
        assert (yield dut.o_rs2_value) == 42, "data is only written when we is on"

    sim.add_process(bench)
    sim.run()


def test_instruction_decoder():
    dut = InstructionDecoder()
    sim = Simulator(dut)

    def bench():
        # invalid
        yield dut.i_instruction.eq(0)
        yield Settle()
        assert (yield dut.o_invalid) == 1

        # addi x1, x0, 42
        yield dut.i_instruction.eq(0x02A00093)
        yield Settle()

        assert (yield dut.o_rs1) == 0
        assert (yield dut.o_rs2) == 0
        assert (yield dut.o_rd) == 1
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == FUNCT3_OP_ADD
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 42
        assert (yield dut.o_has_imm) == 1

        # add x2, x0, x1
        yield dut.i_instruction.eq(0x00100133)
        yield Settle()

        assert (yield dut.o_rs1) == 0
        assert (yield dut.o_rs2) == 1
        assert (yield dut.o_rd) == 2
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == FUNCT3_OP_ADD
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 0
        assert (yield dut.o_has_imm) == 0

    sim.add_process(bench)
    sim.run()


def test_cpu():
    dut = CPU()
    sim = Simulator(dut)

    rom = [
        # addi x1, x0, 42 --> x1 = 42
        0x02A00093,
        # add x2, x0, x1 --> x2 = x0 + x1 = 42
        0x00100133,
        # last entry - stop value
        0,
    ]

    def bench():
        assert (yield dut.o_tmp_pc) == 0x1000

        while True:
            instr_addr = yield dut.o_tmp_pc
            rom_addr = (instr_addr - 0x1000) // 4

            if rom[rom_addr] == 0:
                break

            yield dut.i_tmp_instruction.eq(rom[rom_addr])
            yield Settle()

            yield Tick()
            yield Settle()

            assert (yield dut.o_tmp_pc) == instr_addr + 4

        assert (yield dut.registers.registers.word_select(1, 32)), 42

    sim.add_clock(1e-6)
    sim.add_process(bench)
    with sim.write_vcd("dump.vcd", "dump.gtkw"):
        sim.run()
