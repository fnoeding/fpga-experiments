
# risc v toolchain
RV32I_CC := riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32e -nostdlib -ffreestanding -Tprograms/memory_map.ld
RV32I_OBJCOPY := riscv64-unknown-elf-objcopy
RV32I_READELF := riscv64-unknown-elf-readelf

# host toolchain

CC := gcc -Wall



TARGETS := bin/life.rv32.elf bin/life.rv32.bin bin/life

all: ${TARGETS}


bin/life.rv32.elf: programs/life.c
	@# configure architecture RV32I ISA with ABI ilp32e
	@# disable standard library
	@# instruct compiler to assume freestanding mode to warn about use of system headers
	@# FREESTANDING is our own flag to enable this mode in the code
	@# XXX might need to also include "-nostartfiles"
	${RV32I_CC} -D FREESTANDING -o $@ $<

	# ensure _start is actually at 0x1000'
	${RV32I_READELF} -s $@ | grep '_start$$' | grep -q ' 00001000 '

bin/life.rv32.bin: bin/life.rv32.elf
	${RV32I_OBJCOPY} -O binary $< $@



bin/life: programs/life.c
	${CC} -o $@ $<




.PHONY: clean

clean:
	rm -f ${TARGETS}
