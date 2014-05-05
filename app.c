#include "wiced.h"
#include "gedday.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define RX_WAIT_TIMEOUT   1*SECONDS
#define PORTNUM           50007           // UDP port
#define DEBUG_PORT        50007
#define DELAY_BETWEEN_SCANS       (5000)
#define SEND_UDP_RESPONSE
//#define STA_INTERFACE

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
typedef struct{
    char* ssid;
    char* passphrase;
} network;
/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t process_received_udp_packet();
static wiced_result_t send_udp_response (char* buffer, uint16_t buffer_length, wiced_ip_address_t ip_addr, uint32_t port);
static wiced_result_t udp_printf (char* buffer);
static wiced_result_t publish_service();

static void wifi_scan();
wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
static void udp_print_scan_result( wiced_scan_result_t* record );


static wiced_ip_address_t get_broadcast_address();
extern wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );
static void connection_callback();

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  1,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  1,  1) ),
};

static wiced_timed_event_t process_udp_rx_event;
static wiced_udp_socket_t  udp_socket;
wiced_ip_address_t  ip_interface_address;
wiced_ip_address_t  ip_interface_netmask;
wiced_ip_address_t  ip_interface_broadcast;
char ip_descriptor[16];

static int record_count;
static wiced_time_t scan_start_time;

network* infostrada;


/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    /* Initialise the device and WICED framework */
    wiced_init( );

    infostrada = malloc(sizeof(network));
    infostrada->ssid = "InfostradaWiFi-f668f7";
    infostrada->passphrase = "timavo10d";

#ifndef STA_INTERFACE
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );
    connection_callback();
#else
    wifi_scan();
#endif
}

static void connection_callback(){

    wiced_ip_get_ipv4_address(ACTIVE_INTERFACE, &ip_interface_address);
    wiced_ip_get_netmask(ACTIVE_INTERFACE, &ip_interface_netmask);

    ip_interface_broadcast = get_broadcast_address();

   //ip_addr_broadcast ???? Check if it's the same

    /* Create UDP socket */
    if (wiced_udp_create_socket(&udp_socket, PORTNUM, ACTIVE_INTERFACE) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP socket creation failed\r\n"));
    }
    udp_printf("CONNECTED");
    uint32_t bro_v4 = GET_IPV4_ADDRESS(ip_interface_broadcast);
    sprintf(ip_descriptor, "Broadcast: %d.%d.%d.%d",(unsigned int)((bro_v4 >> 24) & 0xFF),
            (unsigned int)((bro_v4 >> 16) & 0xFF),
            (unsigned int)((bro_v4 >>  8) & 0xFF),
            (unsigned int)((bro_v4 >>  0) & 0xFF));


    udp_printf(ip_descriptor);
    publish_service();
    udp_printf("Service Published.");

    /* Register a function to process received UDP packets */
    wiced_rtos_register_timed_event( &process_udp_rx_event, WICED_NETWORKING_WORKER_THREAD, &process_received_udp_packet, 10, 0 );

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
    wiced_result_t result = wiced_udp_receive(&udp_socket, &packet, RX_WAIT_TIMEOUT );
    if ((result == WICED_ERROR) || (result == WICED_TIMEOUT))
    {
    	; //WPRINT_APP_INFO(("Timeout waiting ...\r\n"));
    }
    else
    {
    	/* Get info about the received UDP packet */
        wiced_udp_packet_get_info(packet, &udp_src_ip_addr, &udp_src_port);

        /* Extract the received data from the UDP packet */
        wiced_packet_get_data(packet, 0, (uint8_t**)&rx_data, &rx_data_length, &available_data_length);

        /* Null terminate the received data, just in case the sender didn't do this */
        rx_data[rx_data_length] = '\x0';

		WPRINT_APP_INFO ( ("UDP Rx: \"%s\" from IP %u.%u.%u.%u:%d\r\n", rx_data,
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 24 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 16 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  8 ) & 0xff ),
																        (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  0 ) & 0xff ),
																        udp_src_port ) );

#ifdef SEND_UDP_RESPONSE
		/* Echo the received data to the sender */
		send_udp_response(rx_data, rx_data_length, udp_src_ip_addr, PORTNUM);
		udp_printf(rx_data);
#endif
		if(strcmp(rx_data, "scan") == 0) wifi_scan();

		else if(strcmp(rx_data, "ifconfig") == 0){
		    uint32_t ipv4 = GET_IPV4_ADDRESS(ip_interface_address);
		    sprintf(ip_descriptor, "%d.%d.%d.%d",(unsigned int)((ipv4 >> 24) & 0xFF),
		            (unsigned int)((ipv4 >> 16) & 0xFF),
		            (unsigned int)((ipv4 >>  8) & 0xFF),
		            (unsigned int)((ipv4 >>  0) & 0xFF));
		    udp_printf(ip_descriptor);
		}

		/* Delete the received packet, it is no longer needed */
		wiced_packet_delete(packet);

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
    if (wiced_packet_create_udp(&udp_socket, buffer_length, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP tx packet creation failed\r\n"));
        return WICED_ERROR;
    }

    /* Copy buffer into tx_data which is located inside the UDP packet */
    memcpy(tx_data, buffer, buffer_length+1);


    /* Set the end of the data portion of the packet */
    wiced_packet_set_data_end(packet, (uint8_t*)tx_data + buffer_length);

    /* Send the UDP packet */
    if (wiced_udp_send(&udp_socket, &ip_interface_broadcast, DEBUG_PORT, packet) != WICED_SUCCESS)
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


static wiced_result_t send_udp_response (char* buffer, uint16_t buffer_length, wiced_ip_address_t ip_addr, uint32_t port)
{
	wiced_packet_t*          packet;
	char*                    tx_data;
	uint16_t                 available_data_length;
	const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, GET_IPV4_ADDRESS(ip_addr) );

	/* Create the UDP packet. Memory for the tx data is automatically allocated */
    if (wiced_packet_create_udp(&udp_socket, buffer_length, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("UDP tx packet creation failed\r\n"));
        return WICED_ERROR;
	}

    /* Copy buffer into tx_data which is located inside the UDP packet */
	memcpy(tx_data, buffer, buffer_length+1);


    /* Set the end of the data portion of the packet */
    wiced_packet_set_data_end(packet, (uint8_t*)tx_data + buffer_length);

    /* Send the UDP packet */
    if (wiced_udp_send(&udp_socket, &target_ip_addr, port, packet) != WICED_SUCCESS)
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
    gedday_add_service("Gedday_instance", "_murata._udp.local", PORTNUM, 6, "Broadcom WICED = Wi-Fi for MCUs!");

    return WICED_SUCCESS;
}

static void wifi_scan(){
    record_count = 0;
    udp_printf("Waiting for scan results...");
    udp_printf("Type  BSSID             RSSI  Rate Chan Security    SSID");
    udp_printf("----------------------------------------------------------------------------------------------");
    wiced_time_get_time(&scan_start_time);
    wiced_wifi_scan_networks(scan_result_handler, NULL );
}

wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    malloc_transfer_to_curr_thread( malloced_scan_result );

    if (malloced_scan_result->scan_complete != WICED_TRUE)
    {
        wiced_scan_result_t* record = &malloced_scan_result->ap_details;
        record->SSID.val[record->SSID.len] = 0; /* Ensure the SSID is null terminated */

        WPRINT_APP_INFO( ( "%3d ", record_count ) );
        udp_print_scan_result(record);
#ifdef STA_INTERFACE
        if(strcmp((char*)record->SSID.val, infostrada->ssid) == 0){
            if(wiced_wifi_join_specific(record, (uint8_t*)infostrada->passphrase, strlen(infostrada->passphrase), NULL) == WICED_SUCCESS)
                if(wiced_ip_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) == WICED_SUCCESS)
                    connection_callback();
        }
#endif
        ++record_count;
    }
    else
    {
        udp_printf("Scan complete!");
    }

    free( malloced_scan_result );

    return WICED_SUCCESS;
}

static void udp_print_scan_result( wiced_scan_result_t* record )
{
    char buffer[300];
    sprintf(buffer, "%5s ", ( record->bss_type == WICED_BSS_TYPE_ADHOC ) ? "Adhoc" : "Infra" );
    sprintf(buffer + strlen(buffer), "%02X:%02X:%02X:%02X:%02X:%02X ", record->BSSID.octet[0], record->BSSID.octet[1], record->BSSID.octet[2], record->BSSID.octet[3], record->BSSID.octet[4], record->BSSID.octet[5] );
    sprintf(buffer + strlen(buffer), " %d ", record->signal_strength );
    if ( record->max_data_rate < 100000 )
    {
        sprintf(buffer + strlen(buffer), " %.1f ", (float) record->max_data_rate / 1000.0 );
    }
    else
    {
        sprintf(buffer + strlen(buffer), "%.1f ", (float) record->max_data_rate / 1000.0 );
    }
    sprintf(buffer + strlen(buffer), " %2d  ", record->channel );
    sprintf(buffer + strlen(buffer), "%-10s ", ( record->security == WICED_SECURITY_OPEN ) ? "Open" :
                                 ( record->security == WICED_SECURITY_WEP_PSK ) ? "WEP" :
                                 ( record->security == WICED_SECURITY_WPA_TKIP_PSK ) ? "WPA TKIP" :
                                 ( record->security == WICED_SECURITY_WPA_AES_PSK ) ? "WPA AES" :
                                 ( record->security == WICED_SECURITY_WPA2_AES_PSK ) ? "WPA2 AES" :
                                 ( record->security == WICED_SECURITY_WPA2_TKIP_PSK ) ? "WPA2 TKIP" :
                                 ( record->security == WICED_SECURITY_WPA2_MIXED_PSK ) ? "WPA2 Mixed" :
                                 "Unknown" );
    sprintf(buffer + strlen(buffer), " %-32s ", record->SSID.val );
    udp_printf(buffer);
}

static wiced_ip_address_t get_broadcast_address(){

    uint32_t netmask_ipv4 = GET_IPV4_ADDRESS(ip_interface_netmask);
    uint32_t address_ipv4 = GET_IPV4_ADDRESS(ip_interface_address);
    uint32_t broadcast = ~netmask_ipv4 | address_ipv4;

    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( ip_broadcast, MAKE_IPV4_ADDRESS((unsigned int)((broadcast >> 24) & 0xFF),(unsigned int)((broadcast >> 16) & 0xFF),(unsigned int)((broadcast >>  8) & 0xFF),(unsigned int)((broadcast >>  0) & 0xFF)));
    return ip_broadcast;
}
