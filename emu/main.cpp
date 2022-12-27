#include <stdio.h>

#include "rv32emu.h"


int main() {
    RV32EmuState *state = rv32emu_init();

    //load_demo(state);

    while(true) {
        rv32emu_print(state);
        printf("\n");
        rv32emu_step(state);
    }

    rv32emu_free(state);

    return 0;
}