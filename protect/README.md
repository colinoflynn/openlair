# Protect #



## Datasheets / References ##



### Main Processor ###

**PN**: MK24FN1M0VLL12

**PACKAGE**: 100-LQFP

- [K24F Reference Manual](https://www.nxp.com/docs/en/reference-manual/K24P144M120SF5RM.pdf)
- [K24F Datasheet](https://www.nxp.com/docs/en/data-sheet/K24P144M120SF5.pdf)

### Low Power Processor ###

**PN**: MKL16Z128VLH4

**PACKAGE**: 64-LQFP

### Wifi SoC ###

**PN**: "Type ZX" SS6006079

### Bluetooth SoC ###

**PN**: DA14580-01

### 802.15.4 SoC ###

**PN**: EM3581

**PACKAGE**: QFN48

### Audio DAC/ADC ###

**PN**: WM8940G


# Hardware Notes #


## Mic Listening Example ##

In this example, data from the mic is read in 16-bit signed PCM format by the K24F and stored in a 120000 byte buffer. The speaker is also setup in loopback mode, meaning sound from the mic is played through the speaker.

## WM8940 Notes ##

The WM8940 is controlled over I2C. All initialization, except for enabling loopback, is done in init_wm8940() in wm8940_i2c.c. 

There's a very helpful diagram on page 11 that shows the input, ADC, DAC, and output stages, along with what the registers control.

To initialize, most of the power up steps from the WM8940 datasheet are followed, with the exception of ramping up the speaker volume. These steps are done in startup_wm8940().

To setup the various clocks needed by the WM8940, we use an input MCLK from the K24F. Details are given in wm8940_i2c.c and the WM8940 datasheet, but basically this clock is fed into a PLL which steps the 12MHz MCLK to 98.304MHz.
This clock is then divided to reach 256 * sample_rate (the same as the FS line) and used as the SYSCLK for all the internals. BCLK needs to be FS * 2 * bit_width and divided from SYSCLK.  

To setup the microphone, the MICP needs to be setup to connect to the PGA amplifier. Input needs to run through this amp. It is also recommended that the ALC (automatic level control) be enabled, which boosts the mic when it's quiet and softens it when the volume gets loud.

All of the filtering after the ADC seems to be able to be bypassed. The last notch filter (NF3) can be setup as a low pass filter, but this doesn't seem to work (frequencies upwards of 5kHz were still normal levels with a cutoff frequency of 2kHz).

Loopback is done on the WM8940. There's a direct bypass from the PGA output to the speaker, but the one used in the example feeds the ADC output into the DAC input.

The WM8940 has an I2C address of 0x07

Unfortunately, the WM8940 doesn't allow you to read from it's registers, meaning you can't verify what you write to them over I2C.

## K24F Notes ##
Unlike the rest of the SDK, the PORT functions don't set their respective clocks during initialization, meaning if you forget to enable the clock beforehand, the K24F hard faults.

### I2S ###
* I2S (specifically MCLK), need to have a high drive strength, otherwise it has trouble supplying the high frequency output.

* I2S Tx needs to be in synchronous mode so that it uses Rx's BCLK and FS (which are the ones connected to the WM8940)

* The I2S MCLK needs to be about 12MHz. There's an internal 48MHz clock that can be used for this.

## Important Nest Pins ##

### I2S ###
* RXD = Pin 77 (PTC5)
* FS = Pin 79 (PTC7)
* BCLK = Pin 81 (PTC9)
* MCLK = Pin 7 (PTE6)
* TXD = Pin 42 (PTA12)

### I2C ###
* SCL = Pin 55 (PTB2)
* SDA = Pin 56 (PTB3)

### Regulator/Amplifier Control ###
* WM8940 Regulator Control = Pin 62 (PTB16)
* MAX98502 Boost Conv Enable = Pin 69 (PTB23)
* MAX98502 Speaker Output Enable = Pin 82 (PTC10)

### Wifi / SDHC Pinout ###
* SDHC0_D0: PTE1
* SDHC0_D1: PTE0
* SDHC0_D2: PTE5
* SDHC0_D3: PTE4
* SDHC0_CMD: PTE3
* SDHC0_DCLK: PTE2
* PTC0: External REG_ON (must be set HIGH) - not necessarily internal REG_ON
* PTC6: - Unsure, seems to be low (?) - maybe IRQ
* PTB1: - Unsure, seems to be high (?) - maybe NRST

## Nest Layout Notes ##

### Sound ###
* A WM8940 does all of the ADC/DAC stuff for the mic and speaker and is located under some shielding on the bottom side of the board
* Mic input comes from a differential mic. This needs to be biased from the WM8940.
* Output from the WM8940 is fed into a MAX98502 Class D amp. The boost converter and output need to be activated by the K24F. The MAX98502 is located under shielding by the mic input on the top side of the board.
* I2S from the WM8940 seems to be little endian

