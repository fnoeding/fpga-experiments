
all: life.riscv-elf life.riscv-bin life.x64

life.riscv-elf: life.c
	# configure architecture RV32I ISA with ABI ilp32e
	# disable standard library
	# instruct compiler to assume freestanding mode to warn about use of system headers
	# FREESTANDING is our own flag to enable this mode in the code
	# XXX might need to also include "-nostartfiles"
	riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32e -nostdlib -ffreestanding -D FREESTANDING -o life.riscv-elf /data/life.c

life.riscv-bin: life.c

life.x64: life.c
	gcc -o life.x64 life.c




.PHONY: clean

clean:
	rm -f life.riscv-elf life.riscv-bin life.x64
