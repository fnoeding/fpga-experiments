#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define ROM_OFFSET          0x1000
#define ROM_SIZE            (16 * 1024 * 1024)

#define RAM_OFFSET          0x10000000
#define RAM_SIZE            (32 * 1024 * 1024)

#define OPCODE_OP_IMM       0b0010011
#define OPCODE_OP           0b0110011
#define OPCODE_LOAD         0b0000011
#define OPCODE_STORE        0b0100011
#define OPCODE_BRANCH       0b1100011
#define OPCODE_CUSTOM0      0b0001011
#define OPCODE_LUI          0b0110111
#define OPCODE_AUIPC        0b0010111
#define OPCODE_JALR         0b1100111
#define OPCODE_JAL          0b1101111

#define FUNCT3_OP_ADD       0b000
#define FUNCT3_OP_SLL       0b001
#define FUNCT3_OP_SLT       0b010
#define FUNCT3_OP_SLTU      0b011
#define FUNCT3_OP_XOR       0b100
#define FUNCT3_OP_SRL       0b101
#define FUNCT3_OP_OR        0b110
#define FUNCT3_OP_AND       0b111

#define FUNCT3_LOAD_B       0b000
#define FUNCT3_LOAD_H       0b001
#define FUNCT3_LOAD_W       0b010
#define FUNCT3_LOAD_BU      0b100
#define FUNCT3_LOAD_HU      0b101

#define FUNCT3_STORE_B      0b000
#define FUNCT3_STORE_H      0b001
#define FUNCT3_STORE_W      0b010

#define FUNCT3_BRANCH_EQ    0b000
#define FUNCT3_BRANCH_NE    0b001
#define FUNCT3_BRANCH_LT    0b100
#define FUNCT3_BRANCH_GE    0b101
#define FUNCT3_BRANCH_LTU   0b110
#define FUNCT3_BRANCH_GEU   0b111

#define FUNCT3_CUSTOM_ASSERT_EQ     0b000
#define FUNCT3_CUSTOM_HLT           0b111

struct State {
    int32_t reg[32];
    uint32_t pc;

    uint8_t rom[ROM_SIZE];
    uint8_t ram[RAM_SIZE];

    uint64_t cycle;
};


class MiniAssembler {
    uint32_t *instruction;

public:
    MiniAssembler(void *memory) {
        instruction = (uint32_t*)memory;
    }

    void load_imm8u(uint32_t dst_reg, uint8_t value) {
        i(OPCODE_OP_IMM, FUNCT3_OP_ADD, dst_reg, 0, value);
    }

    void load_imm32u(uint32_t dst_reg, uint32_t value) {
        // simple wrapper around ADDI and SLL
        // LUI and ADDI would be better, but we have this from early on here. Also LUI + ADDI has a caveat due to sign extension that we would have to handle

        i(OPCODE_OP_IMM, FUNCT3_OP_ADD, dst_reg, 0, (value >> 24) & 0xFF);
        i(OPCODE_OP_IMM, FUNCT3_OP_SLL, dst_reg, dst_reg, 8);

        i(OPCODE_OP_IMM, FUNCT3_OP_ADD, dst_reg, dst_reg, (value >> 16) & 0xFF);
        i(OPCODE_OP_IMM, FUNCT3_OP_SLL, dst_reg, dst_reg, 8);

        i(OPCODE_OP_IMM, FUNCT3_OP_ADD, dst_reg, dst_reg, (value >> 8) & 0xFF);
        i(OPCODE_OP_IMM, FUNCT3_OP_SLL, dst_reg, dst_reg, 8);

        i(OPCODE_OP_IMM, FUNCT3_OP_ADD, dst_reg, dst_reg, value & 0xFF);
    }

    // TODO all load and store: is offset signed or unsigned?
    void load_mem32(uint32_t dst_reg, uint32_t addr_reg, uint32_t offset) {
        i(OPCODE_LOAD, FUNCT3_LOAD_W, dst_reg, addr_reg, offset);
    }

    void store_mem32(uint32_t src_reg, uint32_t addr_reg, uint32_t offset) {
        uint32_t offset_hi = (offset >> 5) & ((1 << 7) - 1);
        uint32_t offset_lo = offset & 0x1F;
        r(OPCODE_STORE, FUNCT3_STORE_W, offset_hi, offset_lo, addr_reg, src_reg);
    }

    void store_mem16(uint32_t src_reg, uint32_t addr_reg, uint32_t offset) {
        uint32_t offset_hi = (offset >> 5) & ((1 << 7) - 1);
        uint32_t offset_lo = offset & 0x1F;
        r(OPCODE_STORE, FUNCT3_STORE_H, offset_hi, offset_lo, addr_reg, src_reg);
    }

    void store_mem8(uint32_t src_reg, uint32_t addr_reg, uint32_t offset) {
        uint32_t offset_hi = (offset >> 5) & ((1 << 7) - 1);
        uint32_t offset_lo = offset & 0x1F;
        r(OPCODE_STORE, FUNCT3_STORE_B, offset_hi, offset_lo, addr_reg, src_reg);
    }

    void custom_assert_equal(uint32_t rs1, uint32_t rs2) {
        r(OPCODE_CUSTOM0, FUNCT3_CUSTOM_ASSERT_EQ, 0, 0, rs1, rs2);
    }

    void custom_hlt() {
        r(OPCODE_CUSTOM0, FUNCT3_CUSTOM_HLT, 0, 0, 0, 0);
    }


    void r(uint32_t opcode, uint32_t funct3, uint32_t funct7, uint32_t rd, uint32_t rs1, uint32_t rs2) {
        *instruction++ = (opcode & 0x7F)
        | ((rd & 0x1F) << 7)
        | ((funct3 &0x07) << 12)
        | ((rs1 & 0x1F) << 15)
        | ((rs2 & 0x1F) << 20)
        | ((funct7 & 0x7F) << 25);
    }

    void i(uint32_t opcode, uint32_t funct3, uint32_t rd, uint32_t rs1, uint32_t imm) {
        *instruction++ = (opcode & 0x7F)
        | ((rd & 0x1F) << 7)
        | ((funct3 &0x07) << 12)
        | ((rs1 & 0x1F) << 15)
        | ((imm & 0x0FFF) << 20);
    }

    void b(uint32_t funct3, uint32_t rs1, uint32_t rs2, int32_t signed_offset) {
        uint32_t offset = signed_offset;

        uint32_t instr = OPCODE_BRANCH
        | (funct3 << 12)
        | ((rs1 & 0x1F) << 15)
        | ((rs2 & 0x1F) << 20);

        // offset[12] --> bits (32, 31]
        instr |= ((offset >> 12) & 1) << 31;

        // offset[10:5] --> bits (31, 25]
        instr |= ((offset >> 5) & ((1 << 6) - 1)) << 25;

        // offset[4:1] --> bits (12, 8]
        instr |= ((offset >> 1) & ((1 << 4) - 1)) << 8;

        // offset[11] --> bits (8, 7]
        instr |= ((offset >> 11) & 1) << 7;

        *instruction++ = instr;
    }
};



void init(State *state) {
    for(size_t i = 0; i < 32; i++) {
        state->reg[i] = 0;
    }

    state->pc = ROM_OFFSET;

    // init rom
    for(size_t i = 0; i < ROM_SIZE; ++i) {
        state->rom[i] = 0;
    }

    // init ram
    for(size_t i = 0; i < RAM_SIZE; ++i) {
        state->ram[i] = 0xAA; // in practice this is not initialized, so set it to something other than 0
    }

    state->cycle = 0;
}


void load_demo(State *state) {
    uint32_t *instr_p = (uint32_t*)(state->rom);
    uint32_t *data_p = (uint32_t*)(state->ram);

    size_t data_idx = 0;

    MiniAssembler masm = MiniAssembler(instr_p);

    // add
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 16/*rd*/, 0/*rs*/, 0/*imm*/);                // x16 = 0
    for(size_t i = 0; i <= 10; i++) {
        masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 16/*rd*/, 16/*rs*/, i/*imm*/);           // x16 = x16 + imm          --> x16 = 0x37
    }

    // add with negative number
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 17/*rd*/, 0/*rs*/, -1/*imm*/);               // x17 = -1                 --> x17 = 0xFFFF_FFFF

    // bit shifting positive
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 1/*rd*/, 0/*rs*/, 16/*imm*/);                // x1 = 16

    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SRL, 18/*rd*/, 1/*rs*/, 2/*imm*/);                // x18 = x1 >> 2 (logical)  --> x19 = 0x04
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SRL, 19/*rd*/, 1/*rs*/, 2 | (1 << 10)/*imm*/);    // x19 = x1 >> 2 (arith.)   --> x20 = 0x04
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SLL, 20/*rd*/, 1/*rs*/, 2/*imm*/);                // x18 = x1 << 2            --> x18 = 0x40

    // bit shifting negative
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 1/*rd*/, 0/*rs*/, -16/*imm*/);               // x1 = -16

    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SLL, 21/*rd*/, 1/*rs*/, 2/*imm*/);                // x21 = x1 << 2            --> x21 = 0xFFFF_FFC0 (-64)
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SRL, 22/*rd*/, 1/*rs*/, 2/*imm*/);                // x22 = x1 >> 2 (logical)  --> x22 = 0x3FFF_FFFC
    masm.i(OPCODE_OP_IMM, FUNCT3_OP_SRL, 23/*rd*/, 1/*rs*/, 2 | (1 << 10)/*imm*/);    // x23 = x1 >> 2 (arith.)   --> x23 = 0xFFFF_FFFC (-4)

    // load from memory
    data_p[data_idx++] = 0xF00DC0FF;
    masm.load_imm32u(2, RAM_OFFSET);                                                    // x2 = RAM_OFFSET
    masm.i(OPCODE_LOAD, FUNCT3_LOAD_W, 24/*rd*/, 2/*rs*/, 0);                      // x24 = load(x1)           --> x24 = 0xF00D_C0FF
    masm.i(OPCODE_LOAD, FUNCT3_LOAD_H, 25/*rd*/, 2/*rs*/, 0);                      // x25 = load(x1)           --> x25 = 0xFFFF_C0FF
    masm.i(OPCODE_LOAD, FUNCT3_LOAD_B, 26/*rd*/, 2/*rs*/, 0);                      // x26 = load(x1)           --> x26 = 0xFFFF_FFFF
    masm.i(OPCODE_LOAD, FUNCT3_LOAD_BU, 27/*rd*/, 2/*rs*/, 0);                     // x27 = load(x1)           --> x27 = 0x0000_00FF
    masm.i(OPCODE_LOAD, FUNCT3_LOAD_HU, 28/*rd*/, 2/*rs*/, 0);                     // x28 = load(x1)           --> x28 = 0x0000_C0FF

    // store to memory
    masm.load_imm32u(2, RAM_OFFSET);                                               // x2 = $RAM_OFFSET
    masm.load_imm32u(3, 0xDEADDEAD);                                               // x3 = 0xDEAD_DEAD
    masm.store_mem32(3, 2, 4);                                                     // *(x2 + 4) = *x3
    masm.store_mem32(3, 2, 8);                                                     // *(x2 + 8) = *x3

    masm.load_mem32(29, 2, 4);                                                     // x29 = 0xDEAD_DEAD

    masm.load_imm32u(3, 0x1234);
    masm.store_mem16(3, 2, 4);
    masm.load_mem32(30, 2, 4);                                                     // x30 = 0xDEAD_1234 

    masm.load_imm32u(3, 0x56);
    masm.store_mem8(3, 2, 4);
    masm.load_mem32(31, 2, 4);                                                      // x31 = 0xDEAD_1256

    // assertions
    masm.custom_assert_equal(0, 0);                                                 // register zero is always zero, must always work
    
    masm.load_imm32u(1, 0x37);
    masm.custom_assert_equal(1, 16);

    masm.load_imm32u(1, -1);
    masm.custom_assert_equal(1, 17);

    masm.load_imm32u(1, 4);
    masm.load_imm32u(2, 0x40);
    masm.custom_assert_equal(1, 18);
    masm.custom_assert_equal(1, 19);
    masm.custom_assert_equal(2, 20);

    masm.load_imm32u(1, 0xFFFFFFC0);
    masm.load_imm32u(2, 0x3FFFFFFC);
    masm.load_imm32u(3, 0xFFFFFFFC);
    masm.custom_assert_equal(1, 21);
    masm.custom_assert_equal(2, 22);
    masm.custom_assert_equal(3, 23);

    masm.load_imm32u(1, 0xF00DC0FF);
    masm.load_imm32u(2, 0xFFFFC0FF);
    masm.load_imm32u(3, 0xFFFFFFFF);
    masm.load_imm32u(4, 0xFF);
    masm.load_imm32u(5, 0xC0FF);
    masm.custom_assert_equal(1, 24);
    masm.custom_assert_equal(2, 25);
    masm.custom_assert_equal(3, 26);
    masm.custom_assert_equal(4, 27);
    masm.custom_assert_equal(5, 28);

    masm.load_imm32u(1, 0xDEADDEAD);
    masm.load_imm32u(2, 0xDEAD1234);
    masm.load_imm32u(3, 0xDEAD1256);
    masm.custom_assert_equal(1, 29);
    masm.custom_assert_equal(2, 30);
    masm.custom_assert_equal(3, 31);

    /**************************
    end of minimal tests. registers are considered freely usable again for more complex tests
    ***************************/

    // branching
    masm.load_imm8u(1, 0);                                                          // counter
    masm.load_imm8u(2, 10);                                                         // target count
    masm.load_imm8u(3, 0);                                                          // accumulator
    masm.load_imm8u(4, 55);                                                         // expected value

    masm.i(OPCODE_OP_IMM, FUNCT3_OP_ADD, 1, 1, 1);                                  // x1 = x1 + 1
    masm.r(OPCODE_OP, FUNCT3_OP_ADD, 0, 3, 3, 1);                                   // x3 = x3 + x1
    masm.b(FUNCT3_BRANCH_LTU, 1, 2, -8);

    masm.custom_assert_equal(3, 4);

    // last instruction: halt
    masm.custom_hlt();   
}


void* _get_pointer(State *state, uint32_t addr) {
    if(addr >= ROM_OFFSET && addr < (ROM_OFFSET + ROM_SIZE)) {
        return state->rom + addr - ROM_OFFSET;
    } else if(addr >= RAM_OFFSET && addr < (RAM_OFFSET + RAM_SIZE)) {
        return state->ram + addr - RAM_OFFSET;
    } else {
        printf("out of bounds memory access\n");
        exit(1);
    }
}


void step(State *state) {
    uint32_t *instr_p = (uint32_t*)(_get_pointer(state, state->pc));
    uint32_t instr = *instr_p;

    // decode
    uint32_t opcode = instr & 0x7F; // instr[6..0]
    uint32_t funct3 = (instr >> 12) & 0x07; // instr[14..12]

    uint32_t data1 = 0xDEADDEAD;
    uint32_t data2 = 0xC0FFEEC0;
    uint32_t reg_dest = 0;

    bool branch_or_jump = false;

    switch(opcode) {
        case OPCODE_CUSTOM0:
        case OPCODE_OP: {
            uint32_t rs1 = (instr >> 15) & 0x1F;
            uint32_t rs2 = (instr >> 20) & 0x1F;
            data1 = state->reg[rs1];
            data2 = state->reg[rs2];
            reg_dest = (instr >> 7) & 0x1F;
            break;
        }
        case OPCODE_LOAD:
        case OPCODE_OP_IMM: {
            uint32_t rs1 = ((instr >> 15) & 0x1F);

            uint32_t imm = (instr >> 20) & 0x0FFF; // 12 bits
            uint32_t imm_sign = instr & (1ul << 31);
            if((opcode != OPCODE_LOAD) && imm_sign) {
                imm |= 0xFFFFF000;
            }

            // XXX refactor to be similar to OPCODE_STORE path? --> data1 as actual address, thus data2 always zero?
            data1 = state->reg[rs1];
            data2 = imm;
            reg_dest = (instr >> 7) & 0x1F;
            break;
        }
        case OPCODE_STORE:
        {
            uint32_t rs1 = (instr >> 15) & 0x1F;
            uint32_t rs2 = (instr >> 20) & 0x1F;

            uint32_t imm_high = (instr >> 25) & ((1 << 7) - 1);
            uint32_t imm_low = (instr >> 7) & 0x1F;
            uint32_t imm = (imm_high << 5) | imm_low;

            data1 = state->reg[rs1] + imm; // address
            data2 = state->reg[rs2]; // data
            reg_dest = 0; // unused
            break;
        }
        case OPCODE_BRANCH: {
            uint32_t imm_32_12 = (instr >> 31) & 1;
            if(imm_32_12) {
                imm_32_12 = (0xFFFFFFFF ^ ((1 << 12) - 1)) >> 12;
            }
            uint32_t imm_11_11 = (instr >> 7) & 1;
            uint32_t imm_10_5 = (instr >> 25) & ((1 << 6) - 1);
            uint32_t imm_4_1 = (instr >> 8) & ((1 << 4) - 1);
            uint32_t imm_0_0 = 0;
            uint32_t imm = (imm_32_12 << 12) | (imm_11_11 << 11) | (imm_10_5 << 5)  | (imm_4_1 << 1) | imm_0_0;

            uint32_t rs1 = (instr >> 15) & 0x1F;
            uint32_t rs2 = (instr >> 20) & 0x1F;

            data1 = state->reg[rs1]; // a
            data2 = state->reg[rs2]; // b
            reg_dest = imm; // offset
            break;
        }
        default: {
            printf("invalid opcode\n");
            exit(1);
        }
    }

    // execute
    switch(opcode) {
        case OPCODE_OP:
        case OPCODE_OP_IMM: {
            uint32_t result = 0;
            switch(funct3) {
                case FUNCT3_OP_ADD: {
                    // add
                    result = data1 + data2; // TODO need to support flags
                    break;
                }
                case FUNCT3_OP_SLL: {
                    // shift left (logical, i.e. treat it as unsigned)
                    result = data1 << data2;
                    break;
                }
                case FUNCT3_OP_SLT: {
                    // shifts take only up to 32 bits
                    data2 = data2 & 0b11111;

                    // set less than (signed)
                    if ((int32_t)data1 < (int32_t) data2) {
                        result = 1;
                    } else {
                        result = 0;
                    }
                    break;
                }
                case FUNCT3_OP_SLTU: {
                    // set less than unsigned
                    if (data1 < data2) {
                        result = 1;
                    } else {
                        result = 0;
                    }
                    break;
                }
                case FUNCT3_OP_XOR: {
                    // (bitwise) xor
                    result = data1 ^ data2;
                    break;
                }
                case FUNCT3_OP_SRL: {
                    // shifts take only up to 32 bits
                    data2 = data2 & 0b11111;

                    // second highest bit of imm indicates whether it's arithmetic (set) or logical (not set)
                    bool arithmetic = instr & (1 << 30);

                    // shift right logical (i.e. treat it as unsigned)
                    if(arithmetic) {
                        uint32_t sign = data1 & (1 << 31);
                        for(size_t i = 0; i < data2; i++) {
                            sign |= sign >> 1;
                        }
                        result = (data1 >> data2) | sign;
                    } else {
                        result = data1 >> data2;
                    }
                    break;
                }
                case FUNCT3_OP_OR: {
                    // (bitwise) or
                    result = data1 | data2;
                    break;
                }
                case FUNCT3_OP_AND: {
                    // (bitwise) and
                    result = data1 & data2;
                }
                default: {
                    printf("invalid funct3\n");
                    exit(1);
                }
            }

            // write back
            if(reg_dest != 0) {
                state->reg[reg_dest] = result;
            }
            break;
        }
        case OPCODE_LOAD: {
            switch(funct3) {
                case FUNCT3_LOAD_W: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1 + data2));
                    state->reg[reg_dest] = *p;
                    break;
                }
                case FUNCT3_LOAD_H: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1 + data2));
                    uint32_t val = *p;
                    uint32_t sign = val & (1 << 15);
                    val = val & 0xFFFF;
                    if(sign) {
                        val |= 0xFFFF0000;
                    }
                    state->reg[reg_dest] = val;
                    break;
                }
                case FUNCT3_LOAD_B: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1 + data2));
                    uint32_t val = *p;
                    uint32_t sign = val & (1 << 7);
                    val = val & 0xFF;
                    if(sign) {
                        val |= 0xFFFFFF00;
                    }
                    state->reg[reg_dest] = val;
                    break;
                }
            case FUNCT3_LOAD_HU: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1 + data2));
                    uint32_t val = *p;
                    val = val & 0xFFFF;
                    state->reg[reg_dest] = val;
                    break;
                }
            case FUNCT3_LOAD_BU: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1 + data2));
                    uint32_t val = *p;
                    val = val & 0xFF;
                    state->reg[reg_dest] = val;
                    break;
                }
            }
            break;
        }
        case OPCODE_STORE:
        {
            switch(funct3) {
                case FUNCT3_STORE_W: {
                    uint32_t *p = (uint32_t*)(_get_pointer(state, data1));
                    *p = data2;
                    break;
                }
                case FUNCT3_STORE_H: {
                    uint16_t *p = (uint16_t*)(_get_pointer(state, data1));
                    *p = (uint16_t)data2;
                    break;
                }
                case FUNCT3_STORE_B: {
                    uint8_t *p = (uint8_t*)(_get_pointer(state, data1));
                    *p = (uint8_t)data2;
                    break;
                }
            }
            break;
        }
        case OPCODE_BRANCH: {
            switch(funct3) {
                case FUNCT3_BRANCH_EQ: {
                    if (data1 == data2) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                case FUNCT3_BRANCH_NE: {
                    if (data1 != data2) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                case FUNCT3_BRANCH_LT: {
                    int32_t x = data1;
                    int32_t y = data2;
                    if (x < y) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                case FUNCT3_BRANCH_GE: {
                    int32_t x = data1;
                    int32_t y = data2;
                    if (x >= y) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                case FUNCT3_BRANCH_LTU: {
                    if (data1 < data2) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                case FUNCT3_BRANCH_GEU: {
                    if (data1 >= data2) {
                        state->pc += reg_dest;
                        branch_or_jump = true;
                    }
                    break;
                }
                default: {
                    printf("invalid funct3\n");
                    exit(1);
                    break;
                }
            }
            break;
        }
        case OPCODE_CUSTOM0: {
            switch(funct3) {
                case FUNCT3_CUSTOM_ASSERT_EQ: {
                    if (data1 != data2) {
                        printf("FAILED assertion: 0x%08x != 0x%08x\n", data1, data2);
                        exit(1);
                    }
                    break;
                }
                case FUNCT3_CUSTOM_HLT: {
                    printf("HLT encountered. Exiting...\n");
                    exit(0);
                    break;
                }
                default: {
                    printf("invalid funct3\n");
                    exit(1);
                    break;
                }
            }
            break;
        }
        default: {
            printf("invalid opcode\n");
            exit(1);
        }
    }

    // update state
    if (!branch_or_jump) {
        state->pc += 4;
    }
    state->cycle += 1;
}


void print(State *state) {
    printf("cycle %lu\n", state->cycle);
    printf("registers\n");
    for(size_t i = 0; i < 8; ++i) {
        printf("    x%02lu = 0x%08x", 4 * i, state->reg[4 * i]);
        printf("    x%02lu = 0x%08x", 4 * i + 1, state->reg[4 * i + 1]);
        printf("    x%02lu = 0x%08x", 4 * i + 2, state->reg[4 * i + 2]);
        printf("    x%02lu = 0x%08x\n", 4 * i + 3, state->reg[4 * i + 3]);
    }
    printf("program counter\n");
    uint32_t* instr_p = (uint32_t*)(_get_pointer(state, state->pc));
    printf("    pc      0x%08x\n", state->pc);
    printf("    instr   0x%08x\n", *instr_p);
    printf("    opcode  0x%08x\n", (*instr_p) & 0x7F);

    printf("memory\n");
    for(size_t i = 0; i < 8; ++i) {
        printf("    ");
        for(size_t j = 0; j < 8; ++j) {
            uint32_t *mem = (uint32_t*)(_get_pointer(state, RAM_OFFSET + i * 8 + j));
            printf("%08x ", *mem);
        }
        printf("\n");
    }
    printf("---\n");
}


void test_add() {
    
}

int main() {
    State *state = new State;

    init(state);
    load_demo(state);

    while(true) {
        print(state);
        printf("\n");
        step(state);
    }

    return 0;
}