#include "digipot_config.h"
#define DIGIPOT_NUM 3

const DigipotConfig default_digipot_values[DIGIPOT_NUM] = {
    {1, 0x50, 0x00, 0x28},
    {3, 0x50, 0x01, 0x78},
    {12, 0x51, 0x00, 0x9C},
};

const DigipotConfig power_saving_digipot_values[DIGIPOT_NUM] = {
    {1, 0x50, 0x00, 0x37},
    {3, 0x50, 0x01, 0xD7},
    {12, 0x51, 0x00, 0xF0},
};
// const uint8_t digipot_index_lookup_from_pin_number(uint8_t pinNumber) {

// }

const size_t digipot_default_count = sizeof(default_digipot_values) / sizeof(default_digipot_values[0]);
const size_t digipot_power_saving_count = sizeof(power_saving_digipot_values) / sizeof(power_saving_digipot_values[0]);