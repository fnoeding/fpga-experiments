
/*
1. install toolchain
    apt install lisc64-unknown-elf-gcc

2. compile
    riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32e -nostdlib /data/life.c

*/

#ifndef FREESTANDING
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#endif


#ifndef WIDTH
#define WIDTH 80
#endif

#ifndef HEIGHT
#define HEIGHT 40
#endif

#define DELTA_UP_LEFT (0 - WIDTH - 2 - 1)
#define DELTA_UP_MIDDLE (0 - WIDTH - 2)
#define DELTA_UP_RIGHT (0 - WIDTH - 2 + 1)
#define DELTA_MIDDLE_LEFT (-1)
#define DELTA_MIDDLE_RIGHT 1
#define DELTA_DOWN_LEFT (WIDTH + 2 - 1)
#define DELTA_DOWN_MIDDLE (WIDTH + 2)
#define DELTA_DOWN_RIGHT (WIDTH + 2 + 1)


// maps are actually (WIDTH + 2) * (HEIGHT + 2) so that we have a dead border at all times
unsigned char* map0;
unsigned char* map1;
unsigned char* leds;
unsigned int generation;


void step(unsigned char* a, unsigned char* b) {
    // init new map
    for(unsigned int i = 0; i < (WIDTH + 2) * (HEIGHT + 2); ++i) {
        b[i] = 0;
    }

    // step simulation and apply Conway's Game of Life rules
    for(unsigned int y = 1; y <= HEIGHT; ++y) {
        for(unsigned int x = 1; x <= WIDTH; ++x) {
            unsigned int idx = y * (WIDTH + 2) + x;

            unsigned int alive_neighbors = 0;
            alive_neighbors += a[idx + DELTA_UP_LEFT];
            alive_neighbors += a[idx + DELTA_UP_MIDDLE];
            alive_neighbors += a[idx + DELTA_UP_RIGHT];
            alive_neighbors += a[idx + DELTA_MIDDLE_LEFT];
            alive_neighbors += a[idx + DELTA_MIDDLE_RIGHT];
            alive_neighbors += a[idx + DELTA_DOWN_LEFT];
            alive_neighbors += a[idx + DELTA_DOWN_MIDDLE];
            alive_neighbors += a[idx + DELTA_DOWN_RIGHT];

            if(a[idx]) {
                if(alive_neighbors == 2 || alive_neighbors == 3) {
                    b[idx] = 1;
                }
            } else {
                if (alive_neighbors == 3) {
                    b[idx] = 1;
                }
            }
        }
    }
}

void init_map(unsigned char *map) {
    unsigned int idx = 0;
    for(unsigned int y = 0; y < HEIGHT + 2; ++y) {
        for(unsigned int x = 0; x < WIDTH + 2; ++x) {
            map[idx++] = 0;
        }
    }

    unsigned int center_x = (WIDTH + 2) / 2;
    unsigned int center_y = (HEIGHT + 2) / 2;

    idx = center_y * (WIDTH + 2) + center_x;

    // blinker
    map[idx] = 1;
    map[idx + DELTA_UP_MIDDLE] = 1;
    map[idx + DELTA_DOWN_MIDDLE] = 1;

    // block
    idx += 10;
    map[idx] = 1;
    map[idx + DELTA_MIDDLE_RIGHT] = 1;
    map[idx + DELTA_DOWN_MIDDLE] = 1;
    map[idx + DELTA_DOWN_RIGHT] = 1;

    // glider
    idx = WIDTH * 3 + 10;
    map[idx + DELTA_MIDDLE_LEFT] = 1;
    map[idx + DELTA_DOWN_MIDDLE] = 1;
    map[idx + DELTA_DOWN_RIGHT] = 1;
    map[idx + DELTA_MIDDLE_RIGHT] = 1;
    map[idx + DELTA_UP_RIGHT] = 1;

    // pulsar
    const char *pulsar = ""
        "......................O...O.....\n"
        "..OOO...OOO...........O...O.....\n"
        "......................O...O.....\n"
        "O....O.O....O........OO...OO....\n"
        "O....O.O....O.......O.O...O.O...\n"
        "O....O.O....O....OOOOO.....OOOOO\n"
        "..OOO...OOO.....................\n"
        "................................\n"
        "..OOO...OOO.....................\n"
        "O....O.O....O....OOOOO.....OOOOO\n"
        "O....O.O....O.......O.O...O.O...\n"
        "O....O.O....O........OO...OO....\n"
        "......................O...O.....\n"
        "..OOO...OOO...........O...O.....\n"
        "......................O...O.....";
    const char *p = pulsar;
    idx = (WIDTH + 2) * 3 + 30;
    while(*p != 0) {
        if(*p == '.') {
            map[idx++] = 0;
        } else if (*p == 'O') {
            map[idx++] = 1;
        } else if (*p == '\n') {
            idx += WIDTH + 2 - 32;
        }

        ++p;
    }
}


// without a standard library this is the entry point

void run() {
    generation = 0;

    #ifdef FREESTANDING
        map0 = (unsigned char*)0x10080000;
        map1 = map0 + 0x80000;
        leds = map1 + 0x80000;
    #else
        map0 = (unsigned char*)malloc((WIDTH + 2) * (HEIGHT + 2));
        map1 = (unsigned char*)malloc((WIDTH + 2) * (HEIGHT + 2));
    #endif

    init_map(map0);

    while(1) {
        #ifdef FREESTANDING
        for(unsigned int i = 0; i < (WIDTH + 2) * (HEIGHT + 2); ++i) {
                leds[i] = map0[i];
        }

        #else
        printf("\033[2J"); // clear screen, go to 0,0
        for(unsigned int y = 1; y <= HEIGHT; y++) {
            for(unsigned int x = 1; x <= WIDTH; x++) {
                unsigned int idx = y * (WIDTH + 2) + x;

                if (map0[idx] != 0) {
                    putchar('X');
                } else {
                    putchar('-');
                }
            }
            putchar('\n');
        }
        printf("gen %d\n", generation);

        sleep(1);
        #endif

        step(map0, map1);
        generation++;

        unsigned char *p = map1;
        map1 = map0;
        map0 = p;
    }
}


#ifdef FREESTANDING

void __attribute__((section (".text.boot"))) _start() {
    run();
}

#else

int main() {
    run();
}

#endif

