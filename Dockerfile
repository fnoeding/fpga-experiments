FROM vmunoz82/eda_tools:0.2

RUN apt-get update
RUN apt-get install -y python3 gcc-riscv64-unknown-elf make git 
RUN pip3 install pytest cffi
