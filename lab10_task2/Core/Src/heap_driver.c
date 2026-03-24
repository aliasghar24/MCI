#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define TOTAL_BLOCKS     (HEAP_SIZE / BLOCK_SIZE)


static uint8_t block_map[TOTAL_BLOCKS];

void heap_init() {
    memset(block_map, 0, sizeof(block_map));
}

void* heap_alloc(size_t request_size) {
    if (request_size == 0) {
        return NULL;
    }

    size_t required_blocks = (request_size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    size_t free_run_length = 0;

    for (size_t block_idx = 0; block_idx < TOTAL_BLOCKS; block_idx++) {
        if (block_map[block_idx] == 0) {
            free_run_length++;

            if (free_run_length == required_blocks) {
                size_t start_idx = block_idx - required_blocks + 1;

                for (size_t mark_idx = start_idx; mark_idx <= block_idx; mark_idx++) {
                    block_map[mark_idx] = 1;
                }

                return (void*)(HEAP_START_ADDR + start_idx * BLOCK_SIZE);
            }
        } else {
            free_run_length = 0;
        }
    }

    return NULL;
}

void heap_free(void* address) {
    if (address == NULL) {
        return;
    }

    uint8_t* byte_address = (uint8_t*)address;


    if (byte_address < HEAP_START_ADDR ||
        byte_address >= HEAP_START_ADDR + HEAP_SIZE) {
        return;
    }

    size_t start_block_idx = (byte_address - HEAP_START_ADDR) / BLOCK_SIZE;

    for (size_t block_idx = start_block_idx; block_idx < TOTAL_BLOCKS; block_idx++) {
        if (block_map[block_idx] == 1) {
            block_map[block_idx] = 0;
        } else {
            break;
        }
    }
}