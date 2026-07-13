#pragma once

#include <stdint.h>

#define ARB_MAX_FILES 8
#define ARB_FILENAME_LEN 16

uint8_t arb_scan_files(void);
uint8_t arb_file_count(void);
const char *arb_file_name(uint8_t index);
uint16_t arb_file_sample_count(uint8_t index);
uint8_t arb_load_file(uint8_t index, uint8_t *buffer, uint16_t max);
