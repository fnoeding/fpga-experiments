#ifndef RV32EMU_HEADER
#define RV32EMU_HEADER


struct RV32EmuState;

#define RV32EMU_ERROR -1
#define RV32EMU_OK 0
#define RV32EMU_BREAKPOINT 1

extern "C" RV32EmuState* rv32emu_init();
extern "C" void rv32emu_free(RV32EmuState *state);

extern "C" void rv32emu_set_rom(RV32EmuState *state, void *data, uint32_t size);
extern "C" void rv32emu_read(RV32EmuState *state, uint32_t addr, uint32_t size, void *p);

extern "C" int32_t rv32emu_step(RV32EmuState *state);
extern "C" int32_t rv32emu_run(RV32EmuState *state, uint32_t *breakpoints, uint32_t num_breakpoints);

extern "C" void rv32emu_print(RV32EmuState *state);


#endif
