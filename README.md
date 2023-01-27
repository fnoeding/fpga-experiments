# FPGA Experiments

This repository contains my experiment to understand [How a CPU works: bare metal C on my RISC-V
CPU](https://florian.noeding.com/posts/risc-v-toy-cpu/cpu-from-scratch/). My blog post explains it
in more detail and also with cleaner code than in this repo.

I've written this code as a learning experiment with limited time. I hope it is useful, but it does
not match my usual quality standard.

## Usage

```sh
git clone $repo
cd $repo

# build the docker image
docker build -t myeda .

# run
docker run --rm -it -v $(PWD):/data myeda

######
# all following commands are run in the docker container
######

cd /data
make clean && make

# 1) run Conway's game of life on host
./bin/life

# 2) run Conway's game of life on emulator
#     resize console to make the output fit the screen, otherwise the output will look broken
#     if the console is broken after running the program, use `reset`
#
# emulator commands:
#     r         run
#     b $hex    sets breakpoint at hex address $hex (use readelf / objdump to figure out where to break)
#     s         steps the program
#     r4 $hex   reads 4 bytes from memory at hex address $hex
./bin/rv32emu --bin bin/life.rv32.bin

# 3) test CPU
python3 -m pytest rtl/cpu.py


```


