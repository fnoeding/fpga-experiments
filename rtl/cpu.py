import pytest

from amaranth import Signal, Module, Elaboratable, ClockDomain, signed
from amaranth.build import Platform
from amaranth.cli import main
from amaranth.sim import Simulator, Settle, Tick


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

    # def check(a, b, expected, signed=True):
    #    yield dut.i_data1.eq(a)
    #    yield dut.i_data2.eq(b)
    #    yield Settle()
    #    if signed:
    #        result = yield dut.o_result
    #    else:
    #        result = yield dut.o_result.as_unsigned()
    #    assert result == expected

    def bench():
        yield dut.i_select_rs1.eq(0)
        yield dut.i_select_rs2.eq(1)
        yield dut.i_select_rd.eq(1)
        yield dut.i_data.eq(42)
        yield dut.i_we.eq(0)
        yield Settle()

        # all registers are reset to zero
        rs1 = yield dut.o_rs1_value
        rs2 = yield dut.o_rs2_value
        assert rs1 == 0
        assert rs2 == 0

        # write to register 1
        yield dut.i_we.eq(1)
        rs2 = yield dut.o_rs2_value
        assert rs2 == 0, "register is only written on pos edge"
        yield Tick()
        yield Settle()
        rs2 = yield dut.o_rs2_value
        assert rs2 == 42

        # write to register 0 - which is hard coded to zero
        yield dut.i_select_rd.eq(0)
        yield Tick()
        yield Settle()
        rs1 = yield dut.o_rs1_value
        assert rs1 == 0, "register zero cannot be written"

    sim.add_process(bench)
    sim.run()
