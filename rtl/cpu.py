import pytest

from amaranth import Signal, Module, Elaboratable, ClockDomain, signed, unsigned, Mux, Cat, Const, Repl, Memory
from amaranth.build import Platform
from amaranth.sim import Simulator, Settle, Tick, Passive


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

FUNCT3_LOAD_B = 0b000
FUNCT3_LOAD_H = 0b001
FUNCT3_LOAD_W = 0b010
FUNCT3_LOAD_BU = 0b100
FUNCT3_LOAD_HU = 0b101

FUNCT3_STORE_B = 0b000
FUNCT3_STORE_H = 0b001
FUNCT3_STORE_W = 0b010

FUNCT3_BRANCH_EQ = 0b000
FUNCT3_BRANCH_NE = 0b001
FUNCT3_BRANCH_LT = 0b100
FUNCT3_BRANCH_GE = 0b101
FUNCT3_BRANCH_LTU = 0b110
FUNCT3_BRANCH_GEU = 0b111

FUNCT3_LOAD_B = 0b000
FUNCT3_LOAD_H = 0b001
FUNCT3_LOAD_W = 0b010
FUNCT3_LOAD_BU = 0b100
FUNCT3_LOAD_HU = 0b101
FUNCT3_STORE_B = 0b000
FUNCT3_STORE_H = 0b001
FUNCT3_STORE_W = 0b010

INTERNAL_FUNCT7_BRANCH_ALWAYS = 0b1

INTERNAL_LSU_MODE_DISABLED = 0
INTERNAL_LSU_MODE_LOAD = 1
INTERNAL_LSU_MODE_STORE = 2


def SignedSignal(bits):
    return Signal(shape=signed(bits))


class ALU(Elaboratable):
    def __init__(self):
        self.i_is_branch = Signal(1)  # if set to 0, then normal ALU operation, otherwise treat funct3 as branch condition operator
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

        with m.If(self.i_is_branch == 0):
            # normal ALU
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
                    # TODO need funct7 support
                    shift_amount = self.i_data2[0:5]
                    m.d.comb += self.o_result.eq(self.i_data1 >> shift_amount)
                with m.Case(FUNCT3_OP_OR):
                    m.d.comb += self.o_result.eq(self.i_data1 | self.i_data2)
                with m.Case(FUNCT3_OP_AND):
                    m.d.comb += self.o_result.eq(self.i_data1 & self.i_data2)
        with m.Else():
            # branch unit mode
            with m.If(self.i_funct7 == INTERNAL_FUNCT7_BRANCH_ALWAYS):
                m.d.comb += self.o_result.eq(1)
            with m.Else():
                with m.Switch(self.i_funct3):
                    with m.Case(FUNCT3_BRANCH_EQ):
                        m.d.comb += self.o_result.eq(self.i_data1 == self.i_data2)
                    with m.Case(FUNCT3_BRANCH_NE):
                        m.d.comb += self.o_result.eq(self.i_data1 != self.i_data2)
                    with m.Case(FUNCT3_BRANCH_LT):
                        m.d.comb += self.o_result.eq(self.i_data1 < self.i_data2)
                    with m.Case(FUNCT3_BRANCH_LTU):
                        m.d.comb += self.o_result.eq(self.i_data1.as_unsigned() < self.i_data2.as_unsigned())
                    with m.Case(FUNCT3_BRANCH_GE):
                        m.d.comb += self.o_result.eq(self.i_data1 >= self.i_data2)
                    with m.Case(FUNCT3_BRANCH_GEU):
                        m.d.comb += self.o_result.eq(self.i_data1.as_unsigned() >= self.i_data2.as_unsigned())

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
        self.i_instruction_address = Signal(32)

        self.o_rs1 = Signal(5)
        self.o_rs2 = Signal(5)
        self.o_rd = Signal(5)
        self.o_rd_we = Signal(1)
        self.o_funct3 = Signal(3)
        self.o_funct7 = Signal(7)
        self.o_invalid = Signal(1)
        self.o_imm = SignedSignal(32)
        self.o_has_imm = Signal(1)
        self.o_is_branch = Signal(1)
        self.o_is_memory = Signal(2)

        self.ports = []
        for x in dir(self):
            if x.startswith("o_") or x.startswith("i_"):
                self.ports.append(getattr(self, x))

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.d.comb += self.o_invalid.eq(0)
        m.d.comb += self.o_is_branch.eq(0)

        opcode = self.i_instruction[0:7]

        with m.Switch(opcode):
            with m.Case(OPCODE_OP_IMM):
                # rd = rs1 $OP imm
                # use ALU with immediate
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(self.i_instruction[20:32]),
                    self.o_has_imm.eq(1),
                    self.o_funct7.eq(0),
                ]
            with m.Case(OPCODE_OP):
                # rd = rs1 $OP rs2
                # use ALU with second register
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(self.i_instruction[20:25]),
                    self.o_imm.eq(0),
                    self.o_has_imm.eq(0),
                    self.o_funct7.eq(self.i_instruction[25:32]),
                ]
            with m.Case(OPCODE_LUI):
                # rd = immediate << 12
                # use ALU to add x0 (zero) to immediate
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_funct3.eq(FUNCT3_OP_ADD),
                    self.o_rs1.eq(0),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(self.i_instruction[12:32] << 12),
                    self.o_has_imm.eq(1),
                    self.o_funct7.eq(0),
                ]
            with m.Case(OPCODE_BRANCH):
                # if condition: pc = pc + imm; else pc = pc + 4
                # use ALU in branch mode
                instr = self.i_instruction
                imm_sign = instr[31]
                imm_b11_l1 = instr[7]  # bit 11, length 1
                imm_b1_l4 = instr[8:12]
                imm_b5_l6 = instr[25:31]

                m.d.comb += [
                    self.o_rd.eq(0),
                    self.o_rd_we.eq(0),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(self.i_instruction[20:25]),
                    self.o_imm.eq(Cat(Const(0, shape=unsigned(1)), imm_b1_l4, imm_b5_l6, imm_b11_l1, Repl(imm_sign, 20))),
                    # though we have an immediate, the ALU must be told to operate on rs2, not imm
                    self.o_has_imm.eq(0),
                    self.o_funct7.eq(0),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_is_branch.eq(1),
                ]
            with m.Case(OPCODE_JAL):
                # pc = pc + imm
                # we re-use the ALU in branch mode, but force it to be always true
                instr = self.i_instruction
                imm_sign = instr[31]
                imm_b1_l10 = instr[21:31]
                imm_b11_l1 = instr[20]
                imm_b12_l8 = instr[12:20]

                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_rs1.eq(0),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(Cat(Const(0, shape=unsigned(1)), imm_b1_l10, imm_b11_l1, imm_b12_l8, Repl(imm_sign, 12))),
                    self.o_has_imm.eq(0),
                    self.o_funct7.eq(INTERNAL_FUNCT7_BRANCH_ALWAYS),
                    self.o_funct3.eq(0),
                    self.o_is_branch.eq(1),
                ]
            with m.Case(OPCODE_JALR):
                # pc = rs1 + imm
                # see OPCODE_JAL
                # TODO decide how to connect rs1 to pc
                #   1) add a now signal, that configures pc to use pc or rs1 as the base
                #   2) add an input to decoder to take the current pc, and subtract it from imm. Then later on it can be re-added
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(self.i_instruction[20:32] - self.i_instruction_address),
                    self.o_has_imm.eq(0),
                    self.o_funct7.eq(INTERNAL_FUNCT7_BRANCH_ALWAYS),
                    self.o_funct3.eq(0),
                    self.o_is_branch.eq(1),
                ]
            with m.Case(OPCODE_LOAD):
                # rd = *mem[rs1 + offset]
                m.d.comb += [
                    self.o_rd.eq(self.i_instruction[7:12]),
                    self.o_rd_we.eq(1),
                    self.o_rs1.eq(self.i_instruction[15:20]),
                    self.o_rs2.eq(0),
                    self.o_imm.eq(self.i_instruction[20:31]),
                    self.o_has_imm.eq(1),
                    self.o_funct7.eq(0),
                    self.o_funct3.eq(self.i_instruction[12:15]),
                    self.o_is_branch.eq(0),
                    self.o_is_memory.eq(INTERNAL_LSU_MODE_LOAD),
                ]
            with m.Default():
                m.d.comb += self.o_invalid.eq(1)

        return m


class ProgramCounter(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")
        self.i_offset = SignedSignal(32)
        self.o_instruction_address = Signal(32, reset=0x1000)

        self.ports = [self.o_instruction_address]

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.d.sync += self.o_instruction_address.eq(self.o_instruction_address + self.i_offset)

        return m


class LoadStoreUnit(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")

        # inside CPU
        self.i_lsu_mode = Signal(2)  # INTERNAL_LSU_MODE_...
        self.i_funct3 = Signal(3)
        self.i_address_base = Signal(32)
        self.i_address_offset = SignedSignal(12)
        self.i_data = Signal(32)  # XXX signed or unsigned?
        self.o_data = Signal(32)  # XXX dito
        self.o_done = Signal(1)

        # to external bus / RAM
        # self.o_ram_address = Signal(32)
        # self.o_ram_mask = Signal(4)  # each bit maps to a byte for writing
        # self.o_ram_mode = Signal(2)  # INTERNAL_LSU_MODE
        # self.o_ram_w_data = Signal(32)
        # self.i_ram_r_data = Signal(32)
        # self.i_ram_ready = Signal(1)

        self.ports = []
        for x in dir(self):
            if x.startswith("o_") or x.startswith("i_"):
                self.ports.append(getattr(self, x))

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        # m.d.sync += [
        #    self.o_ram_address.eq(self.i_address_base + self.i_address_offset),
        #    self.o_data.eq(self.i_ram_r_data),
        # ]

        return m


class CPU(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")

        self.alu = ALU()
        self.decoder = InstructionDecoder()
        self.pc = ProgramCounter()
        self.registers = RegisterFile()
        self.lsu = LoadStoreUnit()

        self.i_tmp_instruction = Signal(32)
        self.o_tmp_pc = Signal(32)

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        m.submodules.alu = a = self.alu
        m.submodules.decoder = d = self.decoder
        m.submodules.registers = r = self.registers
        m.submodules.pc = p = self.pc
        m.submodules.lsu = lsu = self.lsu

        m.d.comb += [
            # register file
            r.i_select_rd.eq(d.o_rd),
            r.i_select_rs1.eq(d.o_rs1),
            r.i_select_rs2.eq(d.o_rs2),
            r.i_we.eq(d.o_rd_we),
            # ALU
            a.i_funct3.eq(d.o_funct3),
            a.i_funct7.eq(d.o_funct7),
            a.i_is_branch.eq(d.o_is_branch),
            a.i_data1.eq(r.o_rs1_value),
            a.i_data2.eq(Mux(d.o_has_imm, d.o_imm, r.o_rs2_value)),
            # LSU
            lsu.i_lsu_mode.eq(d.o_is_memory),
            lsu.i_funct3.eq(d.o_funct3),
            lsu.i_address_base.eq(r.o_rs1_value),
            lsu.i_address_offset.eq(d.o_imm),
            lsu.i_data.eq(r.o_rs2_value),
            # decoder
            d.i_instruction_address.eq(p.o_instruction_address),
        ]

        # register write value
        with m.If(d.o_is_branch):
            # save return address
            m.d.comb += r.i_data.eq(p.o_instruction_address + 4)
        with m.Elif(d.o_is_memory.any()):
            # save value loaded from memory
            m.d.comb += r.i_data.eq(lsu.o_data)
        with m.Else():
            # save the alu result
            m.d.comb += r.i_data.eq(a.o_result)

        # branch logic: alu is in branch mode, returning 1 if branch should be taken
        m.d.comb += p.i_offset.eq(4)
        with m.If(d.o_is_branch):
            with m.If(a.o_result == 1):
                with m.If(d.o_funct7 == INTERNAL_FUNCT7_BRANCH_ALWAYS):
                    # JAL / JALR case, only JALR has rs1 set to something != 0
                    m.d.comb += p.i_offset.eq(d.o_imm + r.o_rs1_value)
                with m.Else():
                    # BEQ / BNEQ / ...
                    m.d.comb += p.i_offset.eq(d.o_imm)

        # stall logic: writes might take many cycles
        # do NOT advance program counter
        with m.If(d.o_is_memory.any() & (lsu.o_done == 0)):
            m.d.comb += p.i_offset.eq(0)

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
        yield dut.i_is_branch.eq(0)

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

        # TODO additional tests, especially for branch conditions

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
        assert (yield dut.o_is_branch) == 0
        assert (yield dut.o_is_memory) == 0

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
        assert (yield dut.o_is_branch) == 0
        assert (yield dut.o_is_memory) == 0

        # lui x3, 123451b7
        yield dut.i_instruction.eq(0x123451B7)
        yield Settle()

        assert (yield dut.o_rs1) == 0
        assert (yield dut.o_rs2) == 0
        assert (yield dut.o_rd) == 3
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == FUNCT3_OP_ADD
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 0x12345000
        assert (yield dut.o_has_imm) == 1
        assert (yield dut.o_is_branch) == 0
        assert (yield dut.o_is_memory) == 0

        # beq x31, x30, -8
        yield dut.i_instruction.eq(0xFFEF8CE3)
        yield Settle()

        print(hex((yield dut.i_instruction)))

        assert (yield dut.o_rs1) == 31
        assert (yield dut.o_rs2) == 30
        assert (yield dut.o_rd) == 0
        assert (yield dut.o_rd_we) == 0
        assert (yield dut.o_funct3) == FUNCT3_BRANCH_EQ
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == -8
        assert (yield dut.o_has_imm) == 0, "ALU must use rs2, not imm"
        assert (yield dut.o_is_branch) == 1
        assert (yield dut.o_is_memory) == 0

        # beq x1, x2, 8
        yield dut.i_instruction.eq(0x00208463)
        yield Settle()

        assert (yield dut.o_rs1) == 1
        assert (yield dut.o_rs2) == 2
        assert (yield dut.o_rd) == 0
        assert (yield dut.o_rd_we) == 0
        assert (yield dut.o_funct3) == FUNCT3_BRANCH_EQ
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 8
        assert (yield dut.o_has_imm) == 0, "ALU must use rs2, not imm"
        assert (yield dut.o_is_branch) == 1
        assert (yield dut.o_is_memory) == 0

        # jal x6, -8
        yield dut.i_instruction.eq(0xFF9FF36F)
        yield Settle()

        assert (yield dut.o_rs1) == 0
        assert (yield dut.o_rs2) == 0
        assert (yield dut.o_rd) == 6
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == 0
        assert (yield dut.o_funct7) == INTERNAL_FUNCT7_BRANCH_ALWAYS
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == -8
        assert (yield dut.o_has_imm) == 0
        assert (yield dut.o_is_branch) == 1
        assert (yield dut.o_is_memory) == 0

        # jalr x8 x7 52
        yield dut.i_instruction_address.eq(0x1000)
        yield dut.i_instruction.eq(0x03438467)
        yield Settle()

        assert (yield dut.o_rs1) == 7
        assert (yield dut.o_rs2) == 0
        assert (yield dut.o_rd) == 8
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == 0
        assert (yield dut.o_funct7) == INTERNAL_FUNCT7_BRANCH_ALWAYS
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 52 - 0x1000  # imm is added to pc, so we have to subtract pc
        assert (yield dut.o_has_imm) == 0
        assert (yield dut.o_is_branch) == 1
        assert (yield dut.o_is_memory) == 0

        # lw x10 50 x9
        yield dut.i_instruction.eq(0x0324A503)
        yield Settle()

        assert (yield dut.o_rs1) == 9
        assert (yield dut.o_rs2) == 0
        assert (yield dut.o_rd) == 10
        assert (yield dut.o_rd_we) == 1
        assert (yield dut.o_funct3) == FUNCT3_LOAD_W
        assert (yield dut.o_funct7) == 0
        assert (yield dut.o_invalid) == 0
        assert (yield dut.o_imm) == 50  # 0x32
        assert (yield dut.o_has_imm) == 1
        assert (yield dut.o_is_branch) == 0
        assert (yield dut.o_is_memory) == INTERNAL_LSU_MODE_LOAD

    sim.add_process(bench)
    sim.run()


def test_load_store_unit():
    # for now we are simply emulating memory, not actually implementing it
    return

    dut = LoadStoreUnit()
    sim = Simulator(dut)
    sim.add_clock(1.0e-6)

    def bench():
        # disabled
        yield dut.i_lsu_mode.eq(INTERNAL_LSU_MODE_DISABLED)
        yield dut.i_ram_r_data.eq(0xDEADBEEF)  # garbage data
        yield dut.i_ram_ready.eq(0)

        yield Settle()
        yield Tick()
        yield Settle()

        assert (yield dut.o_data) == 0
        assert (yield dut.o_ram_address) == 0
        assert (yield dut.o_ram_mask) == 0
        assert (yield dut.o_ram_mode) == 0
        assert (yield dut.o_ram_w_data) == 0

        # load
        # ... request memory controller to do something
        yield dut.i_lsu_mode.eq(INTERNAL_LSU_MODE_LOAD)
        yield dut.i_funct3.eq(FUNCT3_LOAD_W)
        yield dut.i_address_base.eq(0x1000_0000)
        yield dut.i_address_offset.eq(0x32)
        yield dut.i_ram_ready.eq(0)

        yield Settle()
        yield Tick()
        yield Settle()

        # ... check request to memory controller
        assert (yield dut.o_data) == 0
        assert (yield dut.o_ram_address) == 0x1000_0032
        assert (yield dut.o_ram_mask) == 0
        assert (yield dut.o_ram_mode) == INTERNAL_LSU_MODE_LOAD
        assert (yield dut.o_ram_w_data) == 0

        yield Settle()
        yield Tick()
        yield Settle()

        # ... not ready yet, so output data still zero
        assert (yield dut.o_data) == 0

        # ... but let's say we are ready now
        yield dut.i_ram_r_data.eq(0xC0FF_EE00)
        yield dut.i_ram_ready.eq(1)

        yield Settle()
        yield Tick()
        yield Settle()

        assert (yield dut.o_data) == 0xC0FF_EEE

    sim.add_process(bench)
    sim.run()


def test_cpu():
    dut = CPU()
    sim = Simulator(dut)

    rom = [
        0x02A00093,  # addi x1 x0 42        --> x1 = 42
        0x00100133,  # add x2 x0 x1         --> x2 = 42
        0x123451B7,  # lui x3 0x12345       --> x3 = 0x12345
        0x00208463,  # beq x1 x2 8          --> skip the next instruction
        0x00700193,  # addi x3 x0 7         [skipped]
        0x00424233,  # xor x4 x4 x4         --> x4 = 0
        0x00A00293,  # addi x5 x0 10        --> x5 = 10
        0x00120213,  # addi x4 x4 1         --> x4 = x4 + 1
        0x00520463,  # beq x4 x5 8          --> skip the next instruction
        0xFF9FF36F,  # jal x6 -8            --> jump up; effectively setting x4 = 10, also setting x6 = pc + 4
        0x000013B7,  # lui x7 0x1           --> x7 = 0x1000
        0x03438467,  # jalr x8 x7 52        --> skip the next instruction
        0x00634333,  # xor x6 x6 x6         [skipped]
        0x100004B7,  # lui x9 0x10000       --> x9 = 0x1000_0000
        0x0324A503,  # lw x10 50 x9         --> x10 = *((int32*)(mem_u8_ptr[x9 + 0x32]))
        0x00000013,  # nop
        0,
    ]

    ram = [0 for x in range(128)]
    ram[0x32 + 3], ram[0x32 + 2], ram[0x32 + 1], ram[0x32] = 0xC0, 0xFF, 0xEE, 0x42

    done = [0]

    def bench():
        assert (yield dut.o_tmp_pc) == 0x1000

        while True:
            instr_addr = yield dut.o_tmp_pc
            print("instr addr: ", hex(instr_addr))
            rom_addr = (instr_addr - 0x1000) // 4

            # TODO
            # need to make it so that JALR adds imm to rs1, not imm to pc
            # TODO

            if rom[rom_addr] == 0:
                done[0] = 1
                print("bench: done.")
                break

            print("instr: ", hex(rom[rom_addr]))

            yield dut.i_tmp_instruction.eq(rom[rom_addr])
            yield Settle()

            assert (yield dut.decoder.o_invalid) == False

            yield Tick()
            yield Settle()

        read_reg = lambda x: dut.registers.registers.word_select(x, 32)

        assert (yield read_reg(1)) == 42
        assert (yield read_reg(2)) == 42
        assert (yield read_reg(3)) == 0x12345000
        assert (yield read_reg(5)) == 10
        assert (yield read_reg(4)) == 10
        assert (yield read_reg(6)) == 0x1000 + 4 * rom.index(0xFF9FF36F) + 4
        assert (yield read_reg(7)) == 0x1000
        assert (yield read_reg(8)) == 0x1000 + 4 * rom.index(0x03438467) + 4
        assert (yield read_reg(9)) == 0x1000_0000
        assert (yield read_reg(10)) == 0xC0FFEE42

        yield Passive()

    def memory_cosim():
        lsu = dut.lsu

        was_busy = False

        while not done[0]:
            lsu_mode = yield lsu.i_lsu_mode
            if lsu_mode == INTERNAL_LSU_MODE_DISABLED:
                was_busy = False
                yield lsu.o_data.eq(0)
                yield lsu.o_done.eq(0)
            elif lsu_mode == INTERNAL_LSU_MODE_LOAD and was_busy is False:
                was_busy = True
                base = yield lsu.i_address_base
                offset = yield lsu.i_address_offset
                addr = base + offset
                funct3 = yield lsu.i_funct3
                print(f"memory read request: addr={hex(addr)}")

                yield Tick()  # a read takes a while
                yield Tick()
                yield Tick()

                ram_offset = addr - 0x10000000
                if funct3 == FUNCT3_LOAD_W:
                    value = (ram[ram_offset + 3] << 24) | (ram[ram_offset + 2] << 16) | (ram[ram_offset + 1] << 8) | ram[ram_offset]
                else:
                    value = 0xDEADDEAD

                yield lsu.o_data.eq(value)
                yield lsu.o_done.eq(1)

            yield Tick()
        print("memory_cosim: done.")

        yield Passive()

    sim.add_clock(1e-6)
    sim.add_process(bench)
    sim.add_process(memory_cosim)
    with sim.write_vcd("dump.vcd", "dump.gtkw"):
        sim.run()
