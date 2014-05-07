#include "wiced.h"
#include "wiced_dct.h"
#include "gedday.h"

#include <inttypes.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define RX_WAIT_TIMEOUT     50*MILLISECONDS

#define DATA_PORT           50007
#define DEBUG_PORT          50008

#define MAX_SSID_LEN        32
#define MAX_PASSPHRASE_LEN  64

#define STA_INTERFACE

#ifndef DEBUG
#define DEBUG
#endif

#ifdef STA_INTERFACE
#define ACTIVE_INTERFACE WICED_STA_INTERFACE
#else
#define ACTIVE_INTERFACE WICED_AP_INTERFACE
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct {
    char* ssid;
    char* passphrase;
} network;

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t process_received_udp_packet();
static wiced_result_t udp_printf (char* buffer);
static wiced_result_t publish_service();

wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
//static void udp_print_scan_result( wiced_scan_result_t* record );

static wiced_ip_address_t get_broadcast_address();
extern wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );
static void connection_established();

/******************************************************
 *               Variables Definitions
 ******************************************************/

#ifndef STA_INTERFACE
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  2,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  2,  1) ),
};
#endif

static platform_dct_wifi_config_t wifi_config_dct_local;

static wiced_timed_event_t process_udp_rx_event;
static wiced_udp_socket_t  udp_data_socket;
static wiced_udp_socket_t  udp_debug_socket;

static wiced_ip_address_t  ip_interface_address;
static wiced_ip_address_t  ip_interface_netmask;
static wiced_ip_address_t  ip_interface_broadcast;

static wiced_bool_t wifi_connected = WICED_FALSE;

char ip_descriptor[16];

#ifdef DEBUG
/* Debug variables */
char debug_message[200];
#endif

uint16_t prev_acc, prev_vel;

const network known_networks[] =
{
    /* Dummy network */
    { .ssid = "INSERT_SSID", .passphrase = "INSERT_PASSPHRASE" }
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    /* Initialize the device and WICED framework */
    wiced_init( );

    /* Initialize GPIO and PWM peripherals */
    /* TODO: Change dummy duty cycle to 0.0 */
    wiced_gpio_init(WICED_GPIO_8, OUTPUT_PUSH_PULL);
    wiced_gpio_init(WICED_GPIO_9, OUTPUT_PUSH_PULL);
    //wiced_pwm_init(WICED_PWM_3, 20000, 30.0);

    /* TODO: Remove (for testing purposes) */
    wiced_gpio_output_low(WICED_GPIO_8);
    wiced_gpio_output_low(WICED_GPIO_9);
    //wiced_pwm_start(WICED_PWM_3);

#ifndef STA_INTERFACE
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );
    connection_established();
#else
    wiced_wifi_scan_networks(scan_result_handler, NULL);
#endif

}

static void connection_established(){

    wifi_connected = WICED_TRUE;

    wiced_ip_get_ipv4_address(ACTIVE_INTERFACE, &ip_interface_address);
    wiced_ip_get_netmask(ACTIVE_INTERFACE, &ip_interface_netmask);
    ip_interface_broadcast = get_broadcast_address();

    /* Create UDP sockets */
    if (wiced_udp_create_socket(&udp_data_socket, DATA_PORT, ACTIVE_INTERFACE) != WICED_SUCCESS ||
        wiced_udp_create_socket(&udp_debug_socket, DEBUG_PORT, ACTIVE_INTERFACE) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP socket creation failed\r\n"));
    }

    udp_printf("Connected.");

    publish_service();
    udp_printf("Service Published.");

    /* Register a function to process received UDP packets */
    wiced_rtos_register_timed_event( &process_udp_rx_event,
                                     WICED_NETWORKING_WORKER_THREAD,
                                     &process_received_udp_packet,
                                     RX_WAIT_TIMEOUT,
                                     0
                                   );

    wiced_gpio_output_high(WICED_GPIO_8);

    udp_printf("Waiting for UDP packets ...");
}


wiced_result_t process_received_udp_packet()
{
    wiced_packet_t*           packet;
    char*                     rx_data;
    static uint16_t           rx_data_length;
    uint16_t                  available_data_length;
    static wiced_ip_address_t udp_src_ip_addr;
    static uint16_t           udp_src_port;

    /* Wait for UDP packet */
    wiced_result_t result = wiced_udp_receive(&udp_data_socket, &packet, RX_WAIT_TIMEOUT);

    if ((result == WICED_ERROR) || (result == WICED_TIMEOUT))
    {
        //udp_printf("Timeout");
    }
    else
    {
        wiced_gpio_output_low(WICED_GPIO_8);

        udp_printf("Packet received");

    	/* Get info about the received UDP packet */
        wiced_udp_packet_get_info(packet, &udp_src_ip_addr, &udp_src_port);

        /* Extract the received data from the UDP packet */
        wiced_packet_get_data(packet, 0, (uint8_t**)&rx_data, &rx_data_length, &available_data_length);

        if (rx_data_length == 0)
        {
            /* TODO: Prevents crashing */

            /* Delete the received packet, it is no longer needed */
            wiced_packet_delete(packet);

            return WICED_SUCCESS;
        }

        /* Null terminate the received data, just in case the sender didn't do this */
        rx_data[rx_data_length] = '\x0';

		WPRINT_APP_INFO ( ("UDP Rx: \"%s\" from IP %u.%u.%u.%u:%d\r\n", rx_data,
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 24 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 16 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  8 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  0 ) & 0xff ),
																        udp_src_port ) );

		if(rx_data_length >= 4 &&
		   strncmp(rx_data, "scan", 4) == 0)
		{
		    wiced_wifi_scan_networks(scan_result_handler, NULL);
		}
		else if(rx_data_length >= 8 &&
		        strncmp(rx_data, "ifconfig", 8) == 0)
		{
		    uint32_t ipv4 = GET_IPV4_ADDRESS(ip_interface_address);

		    sprintf(debug_message, "IP: %d.%d.%d.%d",(unsigned int)((ipv4 >> 24) & 0xFF),
		            (unsigned int)((ipv4 >> 16) & 0xFF),
		            (unsigned int)((ipv4 >>  8) & 0xFF),
		            (unsigned int)((ipv4 >>  0) & 0xFF));

		    udp_printf(debug_message);
		}
		else if (rx_data_length == 4)
		{
		    uint16_t curr_acc, curr_vel;

		    curr_acc = ((uint16_t *) rx_data)[0];
		    curr_vel = ((uint16_t *) rx_data)[1];

#ifdef DEBUG
	        sprintf(debug_message, "[%d bytes] RAW: %4s | Acc: %" PRIu16 " | Vel: %" PRIu16,
	                rx_data_length, rx_data, ntohs(curr_acc), ntohs(curr_vel)
	                );
		    udp_printf(debug_message);
#endif

		    prev_acc = curr_acc;
		    prev_vel = curr_vel;
		}

		/* Delete the received packet, it is no longer needed */
		wiced_packet_delete(packet);

        wiced_gpio_output_high(WICED_GPIO_8);
    }

	return WICED_SUCCESS;
}

static wiced_result_t udp_printf (char* buffer)
{
    wiced_packet_t*          packet;
    char*                    tx_data;
    uint16_t                 available_data_length;

    uint16_t buffer_length = strlen(buffer);

    /* Create the UDP packet. Memory for the tx data is automatically allocated */
    if (wiced_packet_create_udp(&udp_debug_socket, buffer_length, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP tx packet creation failed\r\n"));
        return WICED_ERROR;
    }

    /* Copy buffer into tx_data which is located inside the UDP packet */
    memcpy(tx_data, buffer, buffer_length+1);


    /* Set the end of the data portion of the packet */
    wiced_packet_set_data_end(packet, (uint8_t*)tx_data + buffer_length);

    /* Send the UDP packet */
    if (wiced_udp_send(&udp_debug_socket, &ip_interface_broadcast, DEBUG_PORT, packet) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP packet send failed\r\n"));
        wiced_packet_delete(packet);  /* Delete packet, since the send failed */
    }
    else
    {
        WPRINT_APP_INFO(("UDP Tx: \"echo: %s\"\r\n\r\n", tx_data));
    }

    /*
     * NOTE : It is not necessary to delete the packet created above, the packet
     *        will be automatically deleted *AFTER* it has been successfully sent
     */

    return WICED_SUCCESS;
}

static wiced_result_t publish_service(){

    uint32_t ipv4 = GET_IPV4_ADDRESS(ip_interface_address);

    sprintf(ip_descriptor, "%d.%d.%d.%d",(unsigned int)((ipv4 >> 24) & 0xFF),
            (unsigned int)((ipv4 >> 16) & 0xFF),
            (unsigned int)((ipv4 >>  8) & 0xFF),
            (unsigned int)((ipv4 >>  0) & 0xFF));


    gedday_init(ACTIVE_INTERFACE, ip_descriptor);
    gedday_add_service("Gedday_instance", "_murata._udp.local", DATA_PORT, 6, "Broadcom WICED = Wi-Fi for MCUs!");

    return WICED_SUCCESS;
}

wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    malloc_transfer_to_curr_thread( malloced_scan_result );

    if (wifi_connected == WICED_TRUE)
    {
        free (malloced_scan_result);
        return WICED_SUCCESS;
    }

    if (malloced_scan_result->scan_complete != WICED_TRUE)
    {
        int i;

        wiced_scan_result_t* record = &malloced_scan_result->ap_details;
        record->SSID.val[record->SSID.len] = 0; /* Ensure the SSID is null terminated */

        for (i = 0; i < sizeof(known_networks) / sizeof(network); i++)
        {
            if (strcmp((char *)record->SSID.val, known_networks[i].ssid) == 0)
            {
                wiced_result_t result;

                /* Read the Wi-Fi config from the DCT */
                result = wiced_dct_read_wifi_config_section( &wifi_config_dct_local );
                if ( result != WICED_SUCCESS )
                {
                    return result;
                }

                /* Write the new security settings into the config */
                wifi_config_dct_local.stored_ap_list[0].details.SSID.len = strlen(known_networks[i].ssid);
                strncpy((char*)&wifi_config_dct_local.stored_ap_list[0].details.SSID.val, known_networks[i].ssid, MAX_SSID_LEN);
                wifi_config_dct_local.stored_ap_list[0].details.BSSID = record->BSSID;
                wifi_config_dct_local.stored_ap_list[0].details.band = record->band;
                wifi_config_dct_local.stored_ap_list[0].details.bss_type = record->bss_type;
                wifi_config_dct_local.stored_ap_list[0].details.channel = record->channel;
                wifi_config_dct_local.stored_ap_list[0].details.security = record->security;
                memcpy((char*)wifi_config_dct_local.stored_ap_list[0].security_key, known_networks[i].passphrase, MAX_PASSPHRASE_LEN);
                wifi_config_dct_local.stored_ap_list[0].security_key_length = strlen(known_networks[i].passphrase);

                /* Write the modified config back into the DCT */
                result = wiced_dct_write_wifi_config_section( (const platform_dct_wifi_config_t*)&wifi_config_dct_local );
                if ( result != WICED_SUCCESS )
                {
                    return result;
                }

                if (wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) == WICED_SUCCESS)
                {
                    connection_established();
                }
            }
        }
    }
    else
    {

    }

    free( malloced_scan_result );

    return WICED_SUCCESS;
}

/*
static void udp_print_scan_result( wiced_scan_result_t* record )
{
    char buffer_result[300];
    sprintf(buffer_result, "%5s ", ( record->bss_type == WICED_BSS_TYPE_ADHOC ) ? "Adhoc" : "Infra" );
    sprintf(buffer_result + strlen(buffer_result), "%02X:%02X:%02X:%02X:%02X:%02X ", record->BSSID.octet[0], record->BSSID.octet[1], record->BSSID.octet[2], record->BSSID.octet[3], record->BSSID.octet[4], record->BSSID.octet[5] );
    sprintf(buffer_result + strlen(buffer_result), " %d ", record->signal_strength );
    if ( record->max_data_rate < 100000 )
    {
        sprintf(buffer_result + strlen(buffer_result), " %.1f ", (float) record->max_data_rate / 1000.0 );
    }
    else
    {
        sprintf(buffer_result + strlen(buffer_result), "%.1f ", (float) record->max_data_rate / 1000.0 );
    }
    sprintf(buffer_result + strlen(buffer_result), " %2d  ", record->channel );
    sprintf(buffer_result + strlen(buffer_result), "%-10s ", ( record->security == WICED_SECURITY_OPEN ) ? "Open" :
                                 ( record->security == WICED_SECURITY_WEP_PSK ) ? "WEP" :
                                 ( record->security == WICED_SECURITY_WPA_TKIP_PSK ) ? "WPA TKIP" :
                                 ( record->security == WICED_SECURITY_WPA_AES_PSK ) ? "WPA AES" :
                                 ( record->security == WICED_SECURITY_WPA2_AES_PSK ) ? "WPA2 AES" :
                                 ( record->security == WICED_SECURITY_WPA2_TKIP_PSK ) ? "WPA2 TKIP" :
                                 ( record->security == WICED_SECURITY_WPA2_MIXED_PSK ) ? "WPA2 Mixed" :
                                 "Unknown" );
    sprintf(buffer_result + strlen(buffer_result), " %-32s ", record->SSID.val );
    udp_printf(buffer_result);
}
*/

static wiced_ip_address_t get_broadcast_address(){

    uint32_t netmask_ipv4 = GET_IPV4_ADDRESS(ip_interface_netmask);
    uint32_t address_ipv4 = GET_IPV4_ADDRESS(ip_interface_address);
    uint32_t broadcast = ~netmask_ipv4 | address_ipv4;

    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( ip_broadcast,
                             MAKE_IPV4_ADDRESS(
                                 (unsigned int)((broadcast & 0xFF000000) >> 24),
                                 (unsigned int)((broadcast & 0x00FF0000) >> 16),
                                 (unsigned int)((broadcast & 0x0000FF00) >> 8),
                                 (unsigned int)((broadcast & 0x000000FF) >> 0)
                             ));

    return ip_broadcast;
}
