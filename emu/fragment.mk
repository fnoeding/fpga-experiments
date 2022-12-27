

TARGETS += bin/rv32emu.so

bin/rv32emu.so: emu/rv32emu.cpp emu/rv32emu.h
	${CXX} -shared -O3 -o $@ $<


