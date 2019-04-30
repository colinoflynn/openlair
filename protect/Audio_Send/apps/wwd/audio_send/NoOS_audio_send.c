/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * Canned UDP Send App
 *
 * Features demonstrated
 *  - Direct WWD driver access (bypasses WICED API)
 *  - No RTOS / No Network Stack
 *
 * This application demonstrates how to use the Cypress Wi-Fi device
 * to send pre-constructed UDP packets without the use of an RTOS or
 * Network Stack.
 *
 * The application is designed to have a minimal memory footprint
 *
 * Application Instructions
 *   1. Modify the AP_SSID/AP_PASS directives in the application code below
 *      to match your access point
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *
 *   The app connects to the AP and sends a UDP packet once per second to
 *   the IP address specified by the PKT_TARGET_IP address
 *
 * *** IMPORTANT NOTE ***
 *   In release builds all UART printing is TURNED OFF to remove
 *   printf and malloc dependency. This reduces memory usage
 *   dramatically!!
 *
 */


#include "platform/wwd_platform_interface.h"
#include "wwd_management.h"
#include "network/wwd_buffer_interface.h"
#include "network/wwd_network_interface.h"
#include "network/wwd_network_constants.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_wifi.h"
#include "wwd_poll.h"
#include <string.h>  /* for NULL */
#include "NoOS_audio_send.h"
#include "wwd_debug.h"
#include "pin_mux.h"
#include "AUDIO/hal.h"
#include "AUDIO/k24f_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include "AUDIO/wm8940_i2c.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_sai.h"
#include "fsl_i2c.h"
#include "fsl_smc.h"
#include "fsl_dac.h"
#include "MK24F12.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define AP_SSID             "wiced-test-2"
#define AP_PASS             "aabbccddee"
#define AP_SEC              WICED_SECURITY_WPA2_MIXED_PSK
#define COUNTRY             WICED_COUNTRY_CANADA
#define JOIN_TIMEOUT        (10000)                                  /* timeout for joining the wireless network in milliseconds  = 10 seconds */
#define IP_ADDR             MAKE_IPV4_ADDRESS( 192, 168, 0,  2 )
#define LOCAL_UDP_PORT      (50007)
#define PKT_TARGET_IP       MAKE_IPV4_ADDRESS( 169, 254, 122, 194 )  /* For unicast, change address e.g. MAKE_IPV4_ADDRESS( 192, 168, 1,   5 ) */
#define PKT_TARGET_UDP_PORT (50007)
#define MAX_PAYLOAD         (1300) /* bytes */
//#define PAYLOAD             "Hello!"

/* In release builds all UART printing is TURNED OFF to remove printf and malloc dependency which reduces memory usage dramatically */
#ifndef DEBUG
#undef  WPRINT_APP_INFO
#define WPRINT_APP_INFO(args)
#endif

#define WWD_ENABLE_STATS

#define SAMPLE_RATE 24000
#define BIT_WIDTH 16


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
/** @cond */
static void  resolve_dest_mac   ( uint32_t dest_ip_addr, wiced_mac_t * MAC_buffer );
static void  send_canned_packet ( char* pkt, uint16_t payload_len );
static char* setup_canned_packet( char *        pkt,
                                  uint16_t      pkt_len,
                                  uint32_t      my_ip_addr,
                                  wiced_mac_t * my_MAC,
                                  uint32_t      dest_ip_addr,
                                  wiced_mac_t * dest_MAC,
                                  uint16_t      src_udp_port,
                                  uint16_t      dest_udp_port );
void spkamp_mux(void);
void set_spkamp(uint8_t on);
void i2s_mux(void);
void i2s_setup(void);
void SAI_tx_callback(void *base, sai_handle_t *hand, status_t status, void *userdata);
void SAI_rx_callback(void *base, sai_handle_t *hand, status_t status, void *userdata);
void i2s_setup_handle(sai_handle_t *txhand, sai_handle_t *rxhand, sai_transfer_format_t *format);
int8_t wwd_thread_poll_some( void );
/******************************************************
 *               Variable Definitions
 ******************************************************/

/* AUDIO STUFF*/

int32_t txbuf[4] = {0xABCD, 0x00, 0xEF12, 0xFF};
int32_t rxbuf[30000];
int rxdone = 0;
int32_t *recbuf = 0;
int32_t *recbuf_st = 0;
int txdone = 0;
/* END AUDIO STUFF */

static wiced_mac_t   my_mac             = { {   0,   0,   0,   0,   0,   0} };
static wiced_mac_t   broadcast_mac      = { {0xff,0xff,0xff,0xff,0xff,0xff} };
static uint8_t       arp_complete       = 0;
static wiced_mac_t*  arp_mac_buffer_ptr = 0;
static uint32_t      arp_dest_ip_addr   = 0;
static uint32_t      my_ip_addr         = IP_ADDR;
static char          pkt_buffer[ ((MAX( UDP_PACKET_SIZE, MIN_IOCTL_BUFFER_SIZE )+63)&(~63))+4 ];
static const wiced_ssid_t ap_ssid =
{
    .length = sizeof(AP_SSID)-1,
    .value  = AP_SSID,
};

/** @endcond */


/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * Main function of canned UDP packet send application
 *
 * This main function initializes Wiced, joins a network,
 * requests the destination address via an ARP, then repeatedly
 * sends a UDP packet.
 */



int main( void )
{


    wiced_mac_t dest_mac = { {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} };
    char* payload_ptr = 0;
    wwd_result_t result;

    NoOS_setup_timing( );



    WPRINT_APP_INFO(("\nPlatform " PLATFORM " initialised\n"));

    /* Initialise Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    nons_buffer_init_t pkt_buff_init = { pkt_buffer, sizeof( pkt_buffer ) };
    wwd_buffer_init( &pkt_buff_init );

    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    /* Get MAC address - this needs to be done before joining a network, so that */
    /* we can check the address of any incoming packets against our MAC */
    wwd_wifi_get_mac_address( &my_mac, WWD_STA_INTERFACE );

    /* Attempt to join the Wi-Fi network */
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : " AP_SSID "   .. retrying\n"));
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));

    if ( PKT_TARGET_IP != 0xFFFFFFFF )  /* Check if the target has a broadcast address */
    {
        /* Send an ARP request to resolve the destination IP address into a MAC address */
        resolve_dest_mac( PKT_TARGET_IP, &dest_mac );
    }
    /* Setup the packet buffer with the canned packet contents. */
    payload_ptr = setup_canned_packet( pkt_buffer, sizeof( pkt_buffer ), my_ip_addr, &my_mac, PKT_TARGET_IP, &dest_mac, LOCAL_UDP_PORT, PKT_TARGET_UDP_PORT );

    /* Copy the payload into the packet */
    //memcpy( payload_ptr, PAYLOAD, sizeof( PAYLOAD ) - 1 );

    /*************** AUDIO STUFF START ************************/

    sai_handle_t txhand;
    sai_handle_t rxhand;
    sai_transfer_format_t txtfer;
    sai_transfer_t msg;
    sai_transfer_t rxmsg;
    i2c_setup();
    set_wm_vreg();

    host_rtos_delay_milliseconds(500);

    volatile uint16_t devid = wm8940_read(0x00); //read id reg
    recbuf = rxbuf;//malloc(100000);
    if (!recbuf)
         while(1);
    i2s_setup();
    i2s_setup_handle(&txhand, &rxhand, &txtfer);
    init_wm8940(BIT_WIDTH, SAMPLE_RATE);
    spkamp_mux();
    set_spkamp(1);
  //  wm8940_setup_loopback();


    /* msg.data = data; */
    /* msg.dataSize = 16; */

    msg.data = txbuf;
    msg.dataSize = 4 * 4;
    rxmsg.data = recbuf;
    /* rxmsg.dataSize = 0x25000; */
    rxmsg.dataSize = 8;
    uint32_t offset = 0;
    /* txdone = 1; */


    /*********************    AUDIO STUFF END *********************/

    int i;
    volatile int num_of_packets_tx = 0;
    /* Loop forever, repeatedly sending the UDP packet */
    while ( 1 )
    {
        /* Recieving audio data */
        SAI_TransferReceiveNonBlocking(I2S0, &rxhand, &rxmsg);
        while (!rxdone);
        rxmsg.data = recbuf + offset;

        rxdone = 0;
        txdone = 0;
        offset += 8/4;
        if (offset >= 25000){

            /* processing packet for transmission */

            for (unsigned int starting = 0; starting < 25000*4; starting += MAX_PAYLOAD) {

                for (i = 0; i < MAX_PAYLOAD; i++){
                     payload_ptr[i] = ((uint8_t *)recbuf)[i+starting];
                 }


                payload_ptr[MAX_PAYLOAD-1] = 0;

                /* Sending packet */

               send_canned_packet( pkt_buffer, MAX_PAYLOAD );
               num_of_packets_tx++;
               host_rtos_delay_milliseconds( 10 );
               while ( wwd_thread_poll_all( ) != 0 ){

               }
            }

             offset = 0;
        }
    }
}





/**
 * Resolves an IP address using ARP
 *
 * Creates and sends a ARP request packet, then waits
 * for the response.
 *
 * @Note : Currently there is no provision for timeouts/retries
 *
 * @param dest_ip_addr : the destination IP address in network endian format (little endian)
 * @param MAC_buffer   : pointer to a MAC address structure which will receive the resolved
 *                       MAC address of the destination IP
 */
static void resolve_dest_mac( uint32_t dest_ip_addr, wiced_mac_t * MAC_buffer )
{
    /* Template for ARP request packet */
    static const arp_packet_t arp_template =
    {
        .ethernet_header =
        {
            { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } },
            { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
            SWAP16( ETHER_TYPE_ARP )
        },
        .arp_message =
        {
            SWAP16( ARP_HARDWARE_TYPE_ETHERNET ),
            SWAP16( ARP_PROTOCOL_TYPE_IPV4 ),
            6,
            4,
            SWAP16( ARP_OPERATION_REQUEST ),
            { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
            0x00000000,
            { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
            0x00000000,
        }
    };

    do
    {
        int pollcount = 0;

        WPRINT_APP_INFO(("Sending ARP request\n"));
        arp_packet_t* pkt = (arp_packet_t*)pkt_buffer;
        memcpy(pkt, &arp_template, sizeof(arp_packet_t));

        /* Fill missing fields into ARP request template */
        memcpy( &pkt->ethernet_header.source,              &my_mac, 6 );
        memcpy( &pkt->arp_message.sender_hardware_address, &my_mac, 6 );
        pkt->arp_message.sender_protocol_address = my_ip_addr;
        pkt->arp_message.target_protocol_address = dest_ip_addr;

        /* Set the current size of the packet */
        host_buffer_set_size( (wiced_buffer_t)pkt,  sizeof(arp_packet_t) - sizeof(wwd_buffer_header_t) );

        /* Queue the packet in the transmit queue */
        wwd_network_send_ethernet_data( (wiced_buffer_t) &pkt->ethernet_header, WWD_STA_INTERFACE );

        /* Setup global variables to allow the ARP response processing to compare against */
        arp_mac_buffer_ptr = MAC_buffer;
        arp_dest_ip_addr = dest_ip_addr;

        /* Send and receive packets until the ARP transaction has completed */
        while ( ( arp_complete != 1 ) && ( pollcount < 10000 ) )
        {
            wwd_thread_poll_all( );
            pollcount++;
        }
        pollcount = 0;
    } while ( arp_complete != 1 );

    /* Reset global variable so incoming ARP packets won't be processed */
    arp_mac_buffer_ptr = 0;

}


/**
 * Creates a canned UDP packet
 *
 * This function creates an empty UDP packet in the given packet buffer
 *
 * @param pkt           : the packet buffer in which the UDP packet will be created
 * @param pkt_len       : the maximum length of pkt (the packet buffer)
 * @param my_ip_addr    : the local IP address in network format (little endian)
 * @param my_MAC        : the local MAC address
 * @param dest_ip_addr  : the destination IP address in network format (little endian)
 * @param dest_MAC      : the destination MAC address
 * @param src_udp_port  : the local UDP port to use
 * @param dest_udp_port : the remote UDP port at the destination
 *
 * @return A pointer to the location where the UDP payload should be copied.
 */
static char* setup_canned_packet( char *        pkt,
                                  uint16_t      pkt_len,
                                  uint32_t      my_ip_addr,
                                  wiced_mac_t *   my_MAC,
                                  uint32_t      dest_ip_addr,
                                  wiced_mac_t *   dest_MAC,
                                  uint16_t      src_udp_port,
                                  uint16_t      dest_udp_port )
{
    udp_packet_t *      udp_pkt = (udp_packet_t*) pkt;
    ethernet_header_t * eth     = &udp_pkt->ethernet_header;
    ipv4_header_t *     iphdr   = &udp_pkt->ip_header;
    udp_header_t *      udphdr  = &udp_pkt->udp_header;

    /* Clear packet */
    memset( pkt, 0, pkt_len );

    /* Setup Ethernet header */
    memcpy( &eth->source, my_MAC, 6 );
    memcpy( &eth->destination, dest_MAC, 6 );
    eth->ether_type = SWAP16( ETHER_TYPE_IPv4 );

    /* Setup IP header
     * Cannot setup total_length or checksum at this stage
     * The following fields are left zero : differentiated_services, identification
     */
    iphdr->header_length         = 5; /* IP header is 5 x 32bits long */
    iphdr->version               = 4; /* IPv4 */
    iphdr->flags_fragment_offset = 2 << 5;
    iphdr->time_to_live          = 128;
    iphdr->protocol              = 0x11; /* UDP protocol */
    iphdr->source_address        = my_ip_addr;
    iphdr->destination_address   = dest_ip_addr;

    /* Setup UDP header
     * Cannot setup udp_lengthat this stage
     * Checksum field left zero to disable UDP checksum
     */
    udphdr->source_port = SWAP16( src_udp_port );
    udphdr->dest_port   = SWAP16( dest_udp_port );

    /* Return the start address of the UDP payload, so the caller can copy their payload in. */
    return udp_pkt->data;
}


/**
 * Send the canned UDP packet
 *
 * Adds the packet length to the IP and UDP headers, then calculates the IP header
 * checksum.
 *
 * @param pkt         : the packet buffer to be sent
 * @param payload_len : the length of the UDP payload data in bytes
 *
 */

static void send_canned_packet( char* pkt, uint16_t payload_len )
{
    udp_packet_t *  udp_pkt       = (udp_packet_t*) pkt;
    ipv4_header_t * iphdr         = &udp_pkt->ip_header;
    udp_header_t *  udphdr        = &udp_pkt->udp_header;
    uint32_t     checksum_temp = 0;
    uint8_t      i;

    /* Add packet length to IP and UDP headers */
    iphdr->total_length = SWAP16( sizeof(ipv4_header_t) + sizeof(udp_header_t) + payload_len );
    udphdr->udp_length  = SWAP16(                         sizeof(udp_header_t) + payload_len );

    /* Calculate the IP header checksum */
    iphdr->header_checksum = 0;
    for ( i = 0; i < ( sizeof(ipv4_header_t) ); i += 2 )
    {
        checksum_temp += ( ( (unsigned char*) iphdr )[i] << 8 ) | ( ( (unsigned char*) iphdr )[i + 1] );
    }
    while ( checksum_temp >> 16 )
    {
        checksum_temp = ( checksum_temp & 0xFFFF ) + ( checksum_temp >> 16 );
    }
    iphdr->header_checksum = SWAP16( (uint16_t)(~checksum_temp) );


    /* Set the packet size */
    host_buffer_set_size( pkt, sizeof(ethernet_header_t) + sizeof(ipv4_header_t) + sizeof(udp_header_t) + payload_len );

    /* Send the packet */
    wwd_network_send_ethernet_data( (wiced_buffer_t) &udp_pkt->ethernet_header, WWD_STA_INTERFACE );
    wwd_thread_send_one_packet( ); /* Send packet only - do not poll all, since that would cause receives which will overwrite packet buffer contents */
}


/**
 * Processes incoming received packets
 *
 * This function is called as a callback by Wiced when a packet has been
 * received.
 * For this example application, the only kind of packet of interest is
 * an ARP response from the destination IP.
 *
 * @param p : packet buffer containing newly received data packet
 * @param interface : The interface (AP or STA) on which the packet was received.
 */
void host_network_process_ethernet_data( /*@only@*/ wiced_buffer_t p, wwd_interface_t interface )
{
    ethernet_header_t * ether_header = (ethernet_header_t *) host_buffer_get_current_piece_data_pointer( p );

    if ( ( interface != WWD_STA_INTERFACE ) ||                                   /* Check that packet came from STA interface */
         ( ether_header->ether_type != SWAP16( ETHER_TYPE_ARP ) ) ||             /* Check ethertype first as it is less costly than memcmp - Only ARP packets need to be received - all other packets are silently ignored */
         ( ( 0 != memcmp( &ether_header->destination, &my_mac, 6 ) ) &&          /* Check if the destination MAC matches ours, or the broadcast value */
           ( 0 != memcmp( &ether_header->destination, &broadcast_mac, 6 ) ) ) )

    {
        host_buffer_release( p, WWD_NETWORK_RX );
        return;
    }

    arp_message_t * arp = (arp_message_t *) &ether_header[1];

    /* Only process ARP replys - Ignore ARP requests, since we do not want anyone to send us any data */
    if ( arp->operation != SWAP16( ARP_OPERATION_REPLY ) )
    {
        host_buffer_release( p, WWD_NETWORK_RX );
        return;
    }

    /* Check if the ARP response is from our destination IP address */
    if ( arp->sender_protocol_address != arp_dest_ip_addr )
    {
        /* Not a response containing the MAC of our destination IP */
        host_buffer_release( p, WWD_NETWORK_RX );
        return;
    }

    if (arp_mac_buffer_ptr != 0)
    {
    /* We now have a MAC for our destination */
        memcpy( arp_mac_buffer_ptr, &arp->sender_hardware_address, 6 );
    }

    arp_complete = 1;
}

void spkamp_mux(void)
{
     CLOCK_EnableClock(kCLOCK_PortB); //enable port b clock
     CLOCK_EnableClock(kCLOCK_PortC); //enable port c clock

     PORT_SetPinMux(PORTB, 23, kPORT_MuxAsGpio);
     PORT_SetPinMux(PORTC, 10, kPORT_MuxAsGpio);
     gpio_pin_config_t LED = {
          kGPIO_DigitalOutput, 0
     };
     GPIO_PinInit(GPIOB, 23, &LED);
     GPIO_PinInit(GPIOC, 10, &LED);
}

void set_spkamp(uint8_t on)
{
     GPIO_WritePinOutput(GPIOB, 23, 1); //turn on boost converter
     GPIO_WritePinOutput(GPIOC, 10, 1); //turn on speaker output
}

void i2s_mux(void)
{
     CLOCK_EnableClock(kCLOCK_PortE);
     CLOCK_EnableClock(kCLOCK_PortA);
     CLOCK_EnableClock(kCLOCK_PortC);
     port_pin_config_t pconf = {
          kPORT_PullDisable,
          kPORT_FastSlewRate,
          kPORT_PassiveFilterDisable,
          kPORT_OpenDrainDisable, //not in structure define for doc
          kPORT_HighDriveStrength,
          kPORT_MuxAlt4, //i2c for SDA and SCL
          kPORT_UnlockRegister //this is a thing I think?
     };
     PORT_SetMultiplePinsConfig(PORTE, (1 << 6), &pconf);
     /* PORT_SetMultiplePinsConfig(PORTC, (1 << 8), &pconf); */
     /* pconf.mux = 4; */
     PORT_SetMultiplePinsConfig(PORTC, (1 << 7) | (1 << 9) | (1 << 5), &pconf);
     pconf.mux = kPORT_MuxAlt6;
     PORT_SetMultiplePinsConfig(PORTA, (1 << 12), &pconf);

     //can do other pins later
}

void i2s_setup(void)
{
     //doing loopback on wm8940, so just need mclk
     sai_config_t conf;
     i2s_mux();

     SAI_TxGetDefaultConfig(&conf);
     //setup options
     /* conf.masterSlave = kSAI_Master; */
     conf.masterSlave = kSAI_Slave;
     conf.syncMode = kSAI_ModeSync;
     conf.mclkSource = kSAI_MclkSourceSelect3;
     /* conf.bclkSource = kSAI_BclkSourceMclkDiv; */
     SAI_TxInit(I2S0, &conf);
     conf.syncMode = kSAI_ModeAsync;
     SAI_RxInit(I2S0, &conf);
     //setup divide on mclk
}


void SAI_tx_callback(void *base, sai_handle_t *hand, status_t status, void *userdata)
{
     if (status == kStatus_SAI_TxIdle)
          txdone = 1;
}


void SAI_rx_callback(void *base, sai_handle_t *hand, status_t status, void *userdata)
{
     if (status == kStatus_SAI_RxIdle) {
          rxdone = 1;
          /* wm8940_clr(1, 1 << 7); */
     }
}

void i2s_setup_handle(sai_handle_t *txhand, sai_handle_t *rxhand, sai_transfer_format_t *format)
{
     /* format->sampleRate_Hz = 24000U; //:)???? */

   // void (*SAI_Callback_ptr_arr[])(void*,sai_handle_t*,status_t,void*) = { SAI_tx_callback, SAI_rx_callback };
    // sai_transfer_callback_t tx_callback = &SAI_tx_callback;
    // sai_transfer_callback_t rx_callback = &SAI_rx_callback;
     format->sampleRate_Hz = SAMPLE_RATE;
     format->bitWidth = BIT_WIDTH;
     format->stereo = kSAI_MonoLeft;
     format->masterClockHz = 12000000; //6MHz
     format->channel = 0;
     format->protocol = kSAI_BusPCMA;
     format->watermark = FSL_FEATURE_SAI_FIFO_COUNT/2U;
     /* format->protocol = kSAI_BusI2S; */
     SAI_TransferTxCreateHandle(I2S0, txhand, &SAI_tx_callback , NULL);
     SAI_TransferTxSetFormat(I2S0, txhand, format, format->masterClockHz * 4, format->masterClockHz);


     SAI_TransferRxCreateHandle(I2S0, rxhand,&SAI_rx_callback, recbuf);
     SAI_TransferRxSetFormat(I2S0, rxhand, format, format->masterClockHz * 4, format->masterClockHz);
}


int8_t wwd_thread_poll_some( void ) /*@modifies internalState@*/
{
    int8_t result = 0;
    result |= wwd_thread_send_one_packet( ) ;
    return result;
}

