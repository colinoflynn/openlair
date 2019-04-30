#pragma once
#include <stdint.h>

#define I2C_GetStatusFlag(x) I2C_MasterGetStatusFlags(x)
#define WMADDR 0x1A


void wm8940_setup_loopback(void);
void init_wm8940(uint8_t bit_width, uint32_t sample_rate);
uint16_t wm8940_read(uint8_t reg);

void wm8940_ramp_vol(void);

void wm8940_set_vol(int8_t dB);
void wm8940_setval(uint8_t reg, uint16_t mask, uint16_t val);

int wm8940_clr(uint8_t reg, uint16_t mask);
int wm8940_set(uint8_t reg, uint16_t mask);
void set_wm_vreg(void);
int wm8940_send(uint8_t reg, uint16_t val);

int i2c_read(uint8_t *data, int len);
int i2c_send(uint8_t *data, int len);
int i2c_sendaddr_rep(int rw, uint8_t addr);
int i2c_sendaddr(int rw, uint8_t addr);

int i2c_setup(void);

