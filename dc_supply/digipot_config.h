#ifndef DIGIPOT_CONFIG_H
#define DIGIPOT_CONFIG_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t pin_number;
    uint8_t slave_addr;
    uint8_t register_addr;
    uint8_t code;
} DigipotConfig;

extern const DigipotConfig power_saving_digipot_values[];
extern const DigipotConfig default_digipot_values[];
extern const size_t digipot_default_count;
extern const size_t digipot_power_saving_count;
#endif