
# risc v toolchain
RV32I_CC := riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -nostdlib -ffreestanding -Tprograms/memory_map.ld
RV32I_OBJCOPY := riscv64-unknown-elf-objcopy
RV32I_READELF := riscv64-unknown-elf-readelf

# host toolchain

CC := gcc -Wall
CXX := g++ -Wall



TARGETS := bin/life.rv32.elf bin/life.rv32.bin bin/life

include */fragment.mk

# fragments define goals first, so we need to make "all" the default one here
.DEFAULT_GOAL := all
all: ${TARGETS}


bin/life.rv32.elf: programs/life.c
	@# configure architecture RV32I ISA with ABI ilp32e
	@# disable standard library
	@# instruct compiler to assume freestanding mode to warn about use of system headers
	@# FREESTANDING is our own flag to enable this mode in the code
	@# XXX might need to also include "-nostartfiles"
	${RV32I_CC} -g -D FREESTANDING -o $@ $<

	# ensure _start is actually at 0x1000'
	${RV32I_READELF} -s $@ | grep '_start$$' | grep -q ' 00001000 '


bin/life.rv32.bin: bin/life.rv32.elf
	${RV32I_OBJCOPY} -O binary $< $@


bin/life: programs/life.c
	${CC} -g -o $@ $<


.PHONY: clean

clean:
	rm -f ${TARGETS}
