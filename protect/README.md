# Protect #


## Main Board ##


## Block Diagram ##

## Firmware Loading Process ##


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

- [Type ZX Product Page](https://wireless.murata.com/eng/type-zx.html)

From the product page, we have more useful information:

**Chipset**: Cypress (CYW43362) with SDIO interface.

- [CYW43362 Product Page](http://www.cypress.com/documentation/datasheets/cyw43362-single-chip-ieee-80211-bgn-macbasebandradio-sdio)
- [CYW43362 Datasheet](http://www.cypress.com/file/363721/download)

The last part of the firmware blob for the wifi SoC in the protect shows the options it was compiled with (this is found in the external SPI flash chip). This has the following string:

```43362a2-roml/sdio-p2p-idsup-idauth-pno Version: 5.90.230.11 CRC: f99fca16 Date: Fri 2015-04-10 18:30:54 PDT```

Note this appears to be the same firmware as in the original Amazon Dash Button based on string comparison in [Amazon Dash RE](https://github.com/dekuNukem/Amazon_Dash_Button). The verisons are slightly different but very close.

 
### Bluetooth SoC ###

**PN**: DA14580-01

- [DA14850 Datasheet](https://support.dialog-semiconductor.com/downloads/DA14580_DS_v3.1.pdf)

This device has a 84 kB ROM that holds the Bluetooth Smart protocol stack, because nobody ever writes bugs to ROM.

### 802.15.4 SoC ###

Presumably this runs the Thread stack.

**PN**: EM3581

**PACKAGE**: QFN48

### Audio DAC/ADC ###

**PN**: WM8940G

- [WM8940 Product Page](https://www.cirrus.com/products/wm8940/)
- [WM8940 Datasheet](https://statics.cirrus.com/pubs/proDatasheet/WM8940_v4.3.pdf)

This is a rather nice audio codec. It drives the speaker and microphone, and should allow for a very high-fideality audio link.