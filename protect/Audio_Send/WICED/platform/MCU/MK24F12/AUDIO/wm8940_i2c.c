#include "wm8940_i2c.h"

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_sai.h"
#include "fsl_i2c.h"
#include "fsl_smc.h"

static uint16_t wm8940_reg[] = {
     0x0000, //reset
     0x0000, /* Power 1 */
     0x0000, /* Power 2 */
     0x0000, /* Power 3 */
     0x0050, /* Interface Control */
     0x0000, /* Companding Control */
     0x0140, /* Clock Control */
     0x0000, /* Additional Controls */
     0x0000, /* GPIO Control */
     0x0002, /* Auto Increment Control */
     0x0000, /* DAC Control */
     0x00FF, /* DAC Volume */
	 0x0000, /* reserved */
	 0x0000,
     0x0100, /* ADC Control */
     0x00FF, /* ADC Volume */
     0x0000, /* Notch Filter 1 Control 1 */
     0x0000, /* Notch Filter 1 Control 2 */
     0x0000, /* Notch Filter 2 Control 1 */
     0x0000, /* Notch Filter 2 Control 2 */
     0x0000, /* Notch Filter 3 Control 1 */
     0x0000, /* Notch Filter 3 Control 2 */
     0x0000, /* Notch Filter 4 Control 1 */
     0x0000, /* Notch Filter 4 Control 2 */
     0x0032, /* DAC Limit Control 1 */
     0x0000, /* DAC Limit Control 2 */
	 
	 0x0000, /*reserved */
	 0x0000,
	 0x0000,
	 0x0000,
	 0x0000,
	 0x0000,

     0x0038, /* ALC Control 1 */
     0x000B, /* ALC Control 2 */
     0x0032, /* ALC Control 3 */
     0x0000, /* Noise Gate */
     0x0041, /* PLLN */
     0x000C, /* PLLK1 */
     0x0093, /* PLLK2 */
     0x00E9, /* PLLK3 */

	 0x0000,
	 0x0000, /*reserved*/
	 
     0x0030, /* ALC Control 4 */
	 
	 0x0000, //reserved

     0x0002, /* Input Control */
     0x0050, /* PGA Gain */

     0x0000, //reserved

     0x0000, /* ADC Boost Control */
	 
	 0x0000, //reserved

     0x0002, /* Output Control */
     0x0000, /* Speaker Mixer Control */
	 0x0000,
	 0x0000,
	 0x0000,
     0x0079, /* Speaker Volume */
	 
	 0x0000,

     0x0000, /* Mono Mixer Control */
};

void delay(uint8_t times)
{
     volatile uint32_t i = 0;
     for (; times; times--)
          for (; i < 1000U; i++);
}

#define I2CBASE I2C0

void i2c_mux(void)
{
     //Open drain selection and unlock register is not in the structure doc
     //for the SDK, but is in the examples and the header files (though behind an
     //ifdef). They may not be a thing
     CLOCK_EnableClock(kCLOCK_PortB);
     /* CLOCK_EnableClock(kCLOCK_I2c0); */
     port_pin_config_t pconf = {
          kPORT_PullUp,
          kPORT_FastSlewRate,
          kPORT_PassiveFilterDisable,
          kPORT_OpenDrainEnable, //not in structure define for doc
          kPORT_LowDriveStrength,
          kPORT_MuxAlt2, //i2c for SDA and SCL
          kPORT_UnlockRegister //this is a thing I think?
     };
     PORT_SetMultiplePinsConfig(PORTB, (1 << 2) | (1 << 3), &pconf);
}

int i2c_setup(void)
{
     i2c_master_config_t conf;
     status_t result;

     //port mux stuff
     i2c_mux();
     I2C_MasterGetDefaultConfig(&conf);

     conf.baudRate_Bps = 400000;

     //change settings here

     I2C_MasterInit(I2CBASE, &conf, CLOCK_GetFreq(I2C0_CLK_SRC));
     return 0;
}

int i2c_sendaddr(int rw, uint8_t addr)
{
     uint8_t status;
     I2C_MasterStart(I2CBASE, addr, rw ? kI2C_Read : kI2C_Write);
     delay(1);
     if (status = I2C_GetStatusFlag(I2CBASE), !(status & kI2C_IntPendingFlag)); //wait for addr to finish sending
     if (status & (kI2C_ReceiveNakFlag | kI2C_ArbitrationLostFlag)) //we got nak or lost arb
          return -1; //error
     else
          return 0; //all good
}

int i2c_sendaddr_rep(int rw, uint8_t addr)
{
     uint8_t status;
     I2C_MasterRepeatedStart(I2CBASE, addr, rw ? kI2C_Read : kI2C_Write);
     delay(1);
     if (status = I2C_GetStatusFlag(I2CBASE), !(status & kI2C_IntPendingFlag)); //wait for addr to finish sending
     if (status & (kI2C_ReceiveNakFlag | kI2C_ArbitrationLostFlag)) //we got nak or lost arb
          return -1; //error
     else
          return 0; //all good
}

//only need to send, not read
int i2c_send(uint8_t *data, int len)
{
     status_t result = kStatus_Success;
     if (result = I2C_MasterWriteBlocking(I2CBASE, data, len,
                                          kI2C_TransferNoStopFlag | kI2C_TransferNoStartFlag)) {
          if (result == kStatus_I2C_ArbitrationLost || result == kStatus_I2C_Nak) {
               I2C_MasterStop(I2CBASE);
               return - 1; //error
          }
     }

     /* if (status = I2C_GetStatusFlag(I2CBASE), status ) */
     delay(1);
     return 0;
}

//works!
int i2c_read(uint8_t *data, int len)
{
     status_t result = kStatus_Success;
     if (result = I2C_MasterReadBlocking(I2CBASE, data, len,
                                         /* kI2C_TransferNoStopFlag | kI2C_TransferNoStartFlag)) { */
                                         kI2C_TransferNoStartFlag)) {
          return - 1; //error
     }
     delay(1);
     return 0;
}

int wm8940_send(uint8_t reg, uint16_t val)
{
     int ret = 0;
     uint8_t *valrev = (uint8_t *)&val;
     if (ret = i2c_sendaddr(0, WMADDR), ret < 0) //keep trying
          return -1;
     if (ret = i2c_send(&reg, 1))
          return -2;
     if (ret = i2c_send(valrev + 1, 1)) //need to send high bit first
          return -3;
     if (ret = i2c_send(valrev, 1))
          return -4;
     if (ret = I2C_MasterStop(I2CBASE))
          return -5;
     return 0;
}

uint16_t wm8940_read(uint8_t reg)
{
     int ret;
     ret = i2c_sendaddr(0, WMADDR);
     if (ret < 0)
          return 1;
     ret = i2c_send(&reg, 1);
     if (ret < 0)
          return 2;
     ret = i2c_sendaddr_rep(1, WMADDR);
     if (ret < 0)
          return 3;
     uint16_t data;
     ret = i2c_read(&data, 2);
     if (ret < 0)
          return 4;
     ret = I2C_MasterStop(I2CBASE);
     if (ret < 0)
          return 5;
     uint16_t data_rev = (data << 8) | (data >> 8);
     return data_rev;
}

void set_wm_vreg(void)
{
     CLOCK_EnableClock(kCLOCK_PortB); //enable port e clock
     PORT_SetPinMux(PORTB, 16, kPORT_MuxAsGpio);
     gpio_pin_config_t LED = {
          kGPIO_DigitalOutput, 1
     };
     GPIO_PinInit(GPIOB, 16, &LED);
     GPIO_WritePinOutput(GPIOB, 16, 1);
}
int wm8940_set(uint8_t reg, uint16_t mask)
{
     wm8940_reg[reg] |= mask;
     wm8940_send(reg, wm8940_reg[reg]);
     return 0;
}

int wm8940_clr(uint8_t reg, uint16_t mask)
{
     wm8940_reg[reg] &= ~mask;
     wm8940_send(reg, wm8940_reg[reg]);
     return 0;
}

void wm8940_setval(uint8_t reg, uint16_t mask, uint16_t maskval)
{
     wm8940_reg[reg] |= mask;
     wm8940_reg[reg] &= ~mask | maskval;
     wm8940_send(reg, wm8940_reg[reg]);
}

void wm8940_set_vol(int8_t dB)
{
     wm8940_setval(0x36, 0x3F, dB + 57);
}

void wm8940_ramp_vol(void)
{
     wm8940_set_vol(-27);
     //delay(10);
     wm8940_set_vol(-21);
     //delay(10);
     wm8940_set_vol(-15);
     //delay(10);
     wm8940_set_vol(-13);
     //delay(10);
     wm8940_set_vol(-11);
     //delay(10);
     wm8940_set_vol(-9);
     //delay(10);
     wm8940_set_vol(-8);
     //delay(10);
     wm8940_set_vol(-7);
     //delay(10);
     wm8940_set_vol(-6);
     /* delay(10); */
     wm8940_set_vol(-5);
     /* delay(10); */
     wm8940_set_vol(-4);
     //delay(10);
     wm8940_set_vol(-3);
     //delay(10);
     wm8940_set_vol(-2);
     //delay(10);
     wm8940_set_vol(-1);
     //delay(10);
     wm8940_set_vol(-0);
     //delay(10);
}

//startup procedure as documented in the wm8940 to minimize pop
//slave = 1 for slave, slave = 0 for master
void startup_wm8940(int master)
{
     //reset internal registers
     wm8940_send(0x00, 0xFFFF);

     //VMID_OP_EN = 1, LVLSHIFT_EN = 1
     wm8940_set(1, (1 << 8) | (1 << 7));

     //DACMU = 1 -> mute DAC
     wm8940_set(10, 1 << 6);

     //CLKSEL = 0, Audio mode = slave (default)
     wm8940_clr(6, 1 << 8); //Use MCLK for device clocking
     wm8940_setval(6, 0b1, master); //WM8940 in slave or master mode

     //POB_CTRL = 1 SOFT_START = 1 //pop control stuff
     wm8940_set(0x07, (1 << 6) | (1 << 5));

     //SPKPEN = 1, SPKNEN = 1 -> activate speaker outputs
     wm8940_set(0x03, (1 << 6) | (1 << 5));

     //VMIDSEL[1:0] bits for 50k ref string impedance between AVDD and VMID
     wm8940_set(0x01, 0x0001);

     //delay
     delay(100);

     //BIASEN = 1 BUFIOEN = 1 -> enable VMID buffer and analog amp bias
     wm8940_set(0x01, (1 << 3) | (1 << 2));

     //POB_CTRL = 0 SOFT_START = 0 -> done with pop mitigation
     wm8940_clr(7, (1 << 6) | (1 << 5));

     //DACEN = 1 SPKMIXEN = 1 -> enable DAC and speaker mixer
     wm8940_set(0x03, (1 << 0) | (1 << 2));

     //DAC2SPK = 1 -> connects the DAC output to the speaker
     wm8940_set(50, 1 << 0);

     //SPKMUTE = 0 -> unmute speaker
     wm8940_clr(54, (1 << 6));

     //-45dB speaker volume
     //would normally ramp up from -57dB to 0dB
     wm8940_set_vol(-45);

     //DACMU = 0 -> unmute DAC
     wm8940_clr(0x0A, 1 << 6);

     //Enable the DAC limiter
     wm8940_set(24, 1 << 8);
}

uint8_t get_mclkdiv_wm8940(uint32_t sample_rate)
{
     switch (sample_rate) {
     case 48000:
          return 0b10; // 2
     case 32000:
          return 0b11; // 3
     case 24000:
          return 0b100; // 4
     case 16000:
          return 0b101; // 6
     case 12000:
          return 0b110; // 8
     case 8000:
          return 0b111; //12
     }
     return 0;
}

uint8_t get_bclkdiv_wm8940(uint8_t bit_width)
{
     //BCLK should be equal to FS * bit_width * 2
     switch (bit_width) {
     case 16:
          return 0b011; //BCLK = SYSCLK / 8
     case 32:
          return 0b010; //BCLK = SYSCLK / 4
     }
}

//assumes a 12MHz mclk from K24F
void setup_clock_wm8940(uint8_t bit_width, uint32_t sample_rate)
{
     //SR = 0b101
     //setup digital filters for 8kHz sample rate
     /* wm8940_set(7, (1 << 3) | (1 << 1)); */
     wm8940_setval(7, 0b111 << 1, 0b101 << 1);

     //PLLEN = 1 -> enable PLL
     //we use the PLL because we can't divide the 12MHz mclk directly
     //to get the 2.048MHz * A sysclk needed for 8kHz * A sample rate
     wm8940_set(1, 1 << 5); //enable pll

     //Feed the 12MHz MCLK directly into the pll, aka f1 = 12MHz
     wm8940_setval(36, 0x30, 0x10); //mclk -> pll
     //wm8940_setval(36, 0x30, 0x00); //mclk*2 -> pll

     //SETUP PLL RATIO
     //We need to use the PLL to get a 98.304MHz clock (f2 = 98.304MHz), which will then divide down nicely into our needed SYSCLK
     //R = f2 / f1
     //N = int(R)
     //K = int(2^24 * (R-N))

     //R = 98.304 / 12 = 8.192
     //N = 8
     //K = int(2^24 *(8.192-8)) = 0x3126E9 (This is the default K value, so this doesn't have to be changed)
     wm8940_setval(36, 0x0F, 0x08); //sets N = 8


     //Setup ADC and I2S clocks
     //The FS (aka the sample rate of the data) and the BCLK (FS * bit_width * 2) are derived from SYSCLK
     //SYSCLK has to be 256 * FS
     //SYSCLK = (f2 / 4) / MCLKDIV
     wm8940_setval(6, 0b111 << 5, get_mclkdiv_wm8940(sample_rate) << 5);

     //BCLK = SYSCLK / BCLKDIV
     wm8940_setval(6, 0b111 << 2, get_bclkdiv_wm8940(bit_width) << 2); //

     //CLKSEL = 1 -> SYSCLK = f2 / 4 / MCLKDIV
     wm8940_set(6, 1 << 8);
}

void setup_mic_wm8940(void)
{
     //sets boost for mic to 0dB
     //the automatic leveling for the microphone should overwrite this if it's turned on
     //wm8940_setval(45, 0b111111, 0b010000); //0dB gain on mic

    wm8940_setval(45, 0b111111, 0b0111111); //0dB gain on mic

     //MICP2INPPGA -> Hooks up the MICP pin to the PGA amplifier
     //Basically enables differential input
     wm8940_set(44, 1 << 0);

     //INPPGAEN = 1 -> Enables Mic input amplifier
     //There's no bypass for this, so this needs to be on for the mic to work
     wm8940_set(2, 1 << 2);

     //INPPGAMUTE = 0 -> Unmutes Mic input amplifier
     wm8940_clr(45, 1 << 6);

     //PGABOOST = 1 -> PGA amp boost = 20dB
     wm8940_set(47, 1 << 8); //20dB boost on mic input

     //BOOSTEN = 1 -> Enables Input boost stage
     wm8940_set(2, 1 << 4);

     //MICBEN = 1 -> Enables microphone bias enable at 0.9 * Vdd
     wm8940_set(1, 1 << 4);
}

//TODO: update to allow different targets for alc (Default is -6dB)
//The HP/LP/BP filters are also in the ADC section, so this could be configured here as well
void setup_adc_wm8940(int enable_alc)
{
     //ALCSEL = 1 -> Enables ALC
     wm8940_setval(32, 1 << 8, enable_alc << 8); //enable ALC
     //NGATEN = 1 -> Enables noise gate to prevent quiet noise signals from being amplified way up
     wm8940_set(35, 1 << 3); //enable noise gate

     //ADCEN = 1 -> Enables ADC
     wm8940_set(0x02, 1 << 0);

     //setup notch filter 3 for low pass fc = 3kHz
     //doesn't seem to work
     wm8940_set(22, (1 << 15) | (1 << 14)); //filter 3 on
     wm8940_set(23, (1 << 14) | (1 << 15)); //LPF mode
     wm8940_setval(23, 0x3FFF, 1697); //coefficient = 1697
}

void setup_i2s_wm8940(uint8_t bit_width)
{
     //PCM mode, 16-bit signed data, fix frame
     wm8940_set(4, (1<<7) | (3<<3));
     wm8940_clr(4, 1 << 6); //16bit signed
     wm8940_setval(4, 0b11 << 5, 0b00 << 5);
}

void init_wm8940(uint8_t bit_width, uint32_t sample_rate)
{
     //turn on vreg
     //delay
     delay(100);

     startup_wm8940(1); // start in master mode
     setup_clock_wm8940(bit_width, sample_rate); //setup PLL to generate the right FS/BCLK and sample at the right frequency
     setup_mic_wm8940();
     setup_adc_wm8940(0); //automatic level control OFF

     setup_i2s_wm8940(bit_width);
}

/* void wm8940_setup_i2s(void) */
void wm8940_setup_loopback(void)
{
     //ADC_LOOPBACK = 1
     //ADC output is fed directly into DAC input
     wm8940_set(0x05, 1 << 0);
}
