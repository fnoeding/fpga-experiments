# to run tests: python3 -m pytest add3.py

import pytest

from amaranth import Signal, Module, Elaboratable, ClockDomain
from amaranth.build import Platform
from amaranth.sim import Simulator, Settle, Tick


class Add3Comb(Elaboratable):
    def __init__(self):
        self.count_1 = Signal(32)
        self.count_2 = Signal(32)
        self.count_3 = Signal(32)
        self.result = Signal(32)

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        # technically this is not needed: a second `+` below would do
        #     but let's build the circuit exactly as shown above
        temp_sum = Signal(32)

        # define how our logic works
        m.d.comb += temp_sum.eq(self.count_1 + self.count_2)
        m.d.comb += self.result.eq(self.count_3 + temp_sum)

        return m


def test_add3comb():
    # set up our device under test
    dut = Add3Comb()

    def bench():
        # set inputs to defined values
        yield dut.count_1.eq(7)
        yield dut.count_2.eq(14)
        yield dut.count_3.eq(21)

        # let the simulation settle down, i.e. arrive at a defined state
        yield Settle()

        # check that the sum is the expected value
        assert (yield dut.result) == 42

    sim = Simulator(dut)
    sim.add_process(bench)
    sim.run()


class Add3Sync(Elaboratable):
    def __init__(self):
        self.sync = ClockDomain("sync")
        self.count_1 = Signal(32)
        self.count_2 = Signal(32)
        self.count_3 = Signal(32)
        self.result = Signal(32)

    def elaborate(self, _: Platform) -> Module:
        m = Module()

        # technically this is not needed: a second `+` below would do
        #     but let's build the circuit exactly as shown above
        temp_sum = Signal(32)

        # define how our logic works
        m.d.sync += temp_sum.eq(self.count_1 + self.count_2)
        m.d.sync += self.result.eq(self.count_3 + temp_sum)

        return m


def test_add3sync():
    # set up our device under test
    dut = Add3Sync()

    def bench():
        # set inputs to defined values
        yield dut.count_1.eq(7)
        yield dut.count_2.eq(14)
        yield dut.count_3.eq(21)

        # let the simulation settle down, i.e. arrive at a defined state
        yield Settle()

        # no positive edge yet, so still at reset value
        assert (yield dut.result) == 0

        # trigger a positive edge on the clock and wait for things to settle down
        yield Tick()
        yield Settle()

        # count3 is reflect in output, since temp sum is still zero
        assert (yield dut.result) == 21

        yield Tick()
        yield Settle()

        # now both count3 and temp sum will be reflected in the output
        assert (yield dut.result) == 42

    sim = Simulator(dut)
    sim.add_process(bench)
    sim.add_clock(1e-6)
    sim.run()