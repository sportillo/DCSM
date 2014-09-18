#include "wiced.h"
#include "wiced_dct.h"
#include "gedday.h"
#include "control_loop.h"

#include "stm32f1xx_platform.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <inttypes.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define RX_WAIT_TIMEOUT     50*MILLISECONDS

#define DATA_PORT           50007
#define DEBUG_PORT          50008

#define MAX_SSID_LEN        32
#define MAX_PASSPHRASE_LEN  64

//#define STA_INTERFACE

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

#define INA_L WICED_GPIO_8
#define INB_L WICED_GPIO_9
#define INA_R WICED_GPIO_10
#define INB_R WICED_GPIO_11
#define PWM_L WICED_PWM_6
#define PWM_R WICED_PWM_3
#define ENC_L WICED_GPIO_3
#define ENC_R WICED_GPIO_4

#define MSG_LEN 4
#define LOG_SERVICE_TIMEOUT 100
#define CTRL_LOOP_TIMEOUT   10
#define LOG_BUFFER_SIZE ((LOG_SERVICE_TIMEOUT / CTRL_LOOP_TIMEOUT) * 4)

#define MAX_SPEED   1.5f
#define PI_2        1.57079632679f

/******************************************************
 *                   Enumerations
 ******************************************************/

/*
SN8200x platform pin definitions ...
+--------------------------------------------------------------------------------------------------------+
| Enum ID       |Pin |   Pin Name on    |    Module     | STM32| Peripheral  |    Board     | Peripheral  |
|               | #  |      Module      |  GPIO Alias   | Port | Available   |  Connection  |   Alias     |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_1  | 5  | MICRO_WKUP       | WICED_GPIO_1  | A  0 | GPIO        | PWM_L        |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_2  | 6  | MICRO_ADC_IN1    | WICED_GPIO_2  | A  1 | GPIO        | PWM_R        |             |
|               |    |                  |               |      | TIM2_CH2    |              |             |
|               |    |                  |               |      | TIM5_CH2    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_3  | 7  | MICRO_ADC_IN2    | WICED_GPIO_3  | A  2 | ADC123_IN2  | ENC_L        |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | TIM2_CH3    |              |             |
|               |    |                  |               |      | TIM5_CH3    |              |             |
|               |    |                  |               |      | TIM9_CH1    |              |             |
|               |    |                  |               |      | USART2_TX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_4  | 9  | MICRO_ADC_IN3    | WICED_GPIO_4  | A  3 | ADC123_IN3  | ENC_R        |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | TIM2_CH4    |              |             |
|               |    |                  |               |      | TIM5_CH4    |              |             |
|               |    |                  |               |      | TIM9_CH2    |              |             |
|               |    |                  |               |      | UART2_RX    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_5  | 10 | MICRO_SPI_SSN    | WICED_GPIO_5  | A  4 | ADC12_IN4   |              |             |
|               |    |                  |               |      | DAC1_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | I2S3_WS     |              |             |
|               |    |                  |               |      | SPI1_NSS    |              |             |
|               |    |                  |               |      | SPI3_NSS    |              |             |
|               |    |                  |               |      | USART2_CK   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_6  | 11 | MICRO_SPI_SSN    | WICED_GPIO_6  | A  5 | ADC12_IN5   |              |             |
|               |    |                  |               |      | DAC2_OUT    |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_SCK    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_7  | 12 | MICRO_SPI_MOSI   | WICED_GPIO_7  | A  7 | ADC12_IN7   |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MOSI   |              |             |
|               |    |                  |               |      | TIM1_CH1N   |              |             |
|               |    |                  |               |      | TIM3_CH2    |              |             |
|               |    |                  |               |      | TIM8_CH1N   |              |             |
|               |    |                  |               |      | TIM14_CH1   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_8  | 32 | MICRO_UART_TX    | WICED_GPIO_8  | A  9 | GPIO        | INA_L        |             |
|               |    |                  |               |      | I2C3_SMBA   |              |             |
|               |    |                  |               |      | TIM1_CH2    |              |             |
|               |    |                  |               |      | USART1_TX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_9  | 33 | MICRO_UART_RX    | WICED_GPIO_9  | A 10 | GPIO        | INB_L        |             |
|               |    |                  |               |      | TIM1_CH3    |              |             |
|               |    |                  |               |      | USART1_RX   |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_10 | 34 | MICRO_JTAG_TMS   | WICED_GPIO_10 | A 11 | GPIO        | INA_R        |             |
|               |    |                  |               |      | USART1_CTS  |              |             |
|               |    |                  |               |      | USB2_DM     |              |             |
|               |    |                  |               |      | CAN_RX      |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_11 | 35 | MICRO_JTAG_TDI   | WICED_GPIO_11 | A 12 | GPIO        | INB_R        |             |
|               |    |                  |               |      | USART1_RTS  |              |             |
|               |    |                  |               |      | USB2_DP     |              |             |
|               |    |                  |               |      | CAN_TX      |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_18 | 43 | MICRO_GPIO_0     | WICED_GPIO_18 | B  6 | GPIO        |              |             |
|               |    |                  |               |      | TIM4_CH1    |              |             |
|               |    |                  |               |      | I2C1_SCL    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_19 | 44 | MICRO_GPIO_1     | WICED_GPIO_19 | B  7 | GPIO        |              |             |
|               |    |                  |               |      | TIM4_CH2    |              |             |
|               |    |                  |               |      | I2C1_SDA    |              |             |
|---------------+----+------------------+---------------+------+-------------+--------------+-------------|
| WICED_GPIO_20 | 46 | MICRO_SPI_MISO   | WICED_GPIO_20 | A  6 | ADC12_IN6   |              |             |
|               |    |                  |               |      | GPIO        |              |             |
|               |    |                  |               |      | SPI1_MISO   |              |             |
|               |    |                  |               |      | TIM1_BKIN   |              |             |
|               |    |                  |               |      | TIM3_CH1    |              |             |
|               |    |                  |               |      | TIM8_BKIN   |              |             |
|               |    |                  |               |      | TIM13_CH1   |              |             |
+---------------+----+------------------+------+---------------+-------------+--------------+-------------+
*/

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
static wiced_result_t udp_printf (char *, uint16_t);
static wiced_result_t publish_service();
static wiced_result_t control_loop();
static wiced_result_t log_service();
static wiced_result_t scan_result_handler( wiced_scan_handler_result_t* malloced_scan_result );
extern wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings );
static wiced_ip_address_t get_broadcast_address();

static void connection_established();
static void gpio_init();
static void timer_init();
static void irq_handler_l( void *arg );
static void irq_handler_r( void *arg );

//static void udp_print_scan_result( wiced_scan_result_t* record );

/******************************************************
 *               Variables Definitions
 ******************************************************/

#ifndef STA_INTERFACE
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  1,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  1,  1) ),
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

static wiced_timed_event_t   control_loop_event;
static wiced_timed_event_t   log_service_event;
static wiced_worker_thread_t control_loop_thread;
static wiced_worker_thread_t log_service_thread;

static real_T   Vd, Rd, V_l, V_r;
static real_T   T_edge_l, T_edge_r;
static uint16_t Moving_l, Moving_r;

char ip_descriptor[16];

uint32_t elapsed_r, elapsed_l;
uint16_t elapsed_count_r, elapsed_count_l;

#ifdef DEBUG
/* Debug variables */
real_T  log_buffer[LOG_BUFFER_SIZE];
uint16_t log_stored_msg = 0;
#endif

const network known_networks[] =
{
    /* Dummy network */
    { .ssid = "SSID", .passphrase = "PASSPHRASE" }
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    Vd = 0.0f;
    Rd = (real_T) MAX_uint32_T;
    T_edge_l = (real_T) MAX_uint32_T;
    T_edge_r = (real_T) MAX_uint32_T;
    Moving_l = 0;
    Moving_r = 0;
    elapsed_l = 0;
    elapsed_r = 0;
    elapsed_count_l = 0;
    elapsed_count_r = 0;

    /* Initialize the device and WICED framework */
    wiced_init();
    gpio_init();
    timer_init();
    control_loop_initialize();

    control_loop_U.Vd = Vd;
    control_loop_U.Rd = Rd;
    control_loop_U.T_edge_l = T_edge_l;
    control_loop_U.T_edge_r = T_edge_r;
    control_loop_U.Moving_l = Moving_l;
    control_loop_U.Moving_r = Moving_r;

#ifndef STA_INTERFACE
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );
    connection_established();
#else
    wiced_wifi_scan_networks(scan_result_handler, NULL);
#endif

    /* Create the loop thread with highest priority (2) */
    wiced_rtos_create_worker_thread( &control_loop_thread,
                                     2,
                                     WICED_DEFAULT_APPLICATION_STACK_SIZE,
                                     1
                                   );

#ifdef DEBUG
    /* Create the log thread with default application priority (7) */
    wiced_rtos_create_worker_thread( &log_service_thread,
                                     WICED_APPLICATION_PRIORITY,
                                     WICED_DEFAULT_APPLICATION_STACK_SIZE,
                                     1
                                   );
#endif

    /* Register the loop function to be invoked each control loop period (10ms) */
    wiced_rtos_register_timed_event( &control_loop_event,
                                     &control_loop_thread,
                                     &control_loop,
                                     CTRL_LOOP_TIMEOUT,
                                     0
                                   );

#ifdef DEBUG
    /* Register the log function to be invoked each log service period (100ms) */
    wiced_rtos_register_timed_event( &log_service_event,
                                     &log_service_thread,
                                     &log_service,
                                     LOG_SERVICE_TIMEOUT,
                                     0
                                   );
#endif
    /* Register a function to process received UDP packets */
    wiced_rtos_register_timed_event( &process_udp_rx_event,
                                     WICED_NETWORKING_WORKER_THREAD,
                                     &process_received_udp_packet,
                                     RX_WAIT_TIMEOUT,
                                     0
                                   );

}

void gpio_init(){
    /* Initialize GPIO and PWM peripherals */
    wiced_gpio_init(INA_R, OUTPUT_PUSH_PULL);               /* INB_R */
    wiced_gpio_init(INB_R, OUTPUT_PUSH_PULL);               /* INA_R */
    wiced_pwm_init(PWM_R, 20000, 0.0);                      /* PWM_R */

    wiced_gpio_init(INA_L, OUTPUT_PUSH_PULL);               /* INA_L */
    wiced_gpio_init(INB_L, OUTPUT_PUSH_PULL);               /* INB_L */
    wiced_pwm_init(PWM_L, 20000, 0.0);                      /* PWM_L */

    wiced_gpio_output_low(INA_R);
    wiced_gpio_output_low(INB_R);
    wiced_pwm_start(PWM_R);

    wiced_gpio_output_low(INA_L);
    wiced_gpio_output_low(INB_L);
    wiced_pwm_start(PWM_L);
}

void timer_init(){
    TIM_TimeBaseInitTypeDef TIM_TBInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    /**
     * RIGHT: TIM4 (us)
     * LEFT:  TIM1 (us)
     */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TBInitStructure.TIM_Period = 29999;
    TIM_TBInitStructure.TIM_Prescaler = 71;
    TIM_TBInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TBInitStructure);
    TIM_TimeBaseInit(TIM1, &TIM_TBInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t) 0xE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0xC;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0xB;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    wiced_gpio_input_irq_enable(ENC_L, IRQ_TRIGGER_RISING_EDGE, irq_handler_l, 0);
    wiced_gpio_input_irq_enable(ENC_R, IRQ_TRIGGER_RISING_EDGE, irq_handler_r, 0);

    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_irq ()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

        Moving_r = 0;
        elapsed_count_r = 0;
        elapsed_r = 0;

        TIM_SetCounter(TIM4, 0);
    }
}

void TIM1_UP_irq ()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Moving_l = 0;
        elapsed_count_l = 0;
        elapsed_l = 0;

        TIM_SetCounter(TIM1, 0);
    }
}

void irq_handler_l (void *arg)
{
    uint32_t us;
    us = TIM_GetCounter(TIM1);

    elapsed_l += us;
    elapsed_count_l ++;
    Moving_l = 1;

    TIM_SetCounter(TIM1, 0);
}

void irq_handler_r (void *arg)
{
    uint32_t us;
    us = TIM_GetCounter(TIM4);

    elapsed_r += us;
    elapsed_count_r ++;
    Moving_r = 1;

    TIM_SetCounter(TIM4, 0);
}

wiced_result_t control_loop()
{
    float PWM_l = .0f;
    float PWM_r = .0f;

    /* Compute the average period for the encoder step */
    if(elapsed_count_l != 0) {
        T_edge_l = ((real_T) elapsed_l) / elapsed_count_l;
    }

    if(elapsed_count_r != 0) {
        T_edge_r = ((real_T) elapsed_r) / elapsed_count_r;
    }

    control_loop_U.Vd = Vd;
    control_loop_U.Rd = Rd;
    control_loop_U.T_edge_l = T_edge_l;
    control_loop_U.T_edge_r = T_edge_r;
    control_loop_U.Moving_l = Moving_l;
    control_loop_U.Moving_r = Moving_r;

    elapsed_l = 0;
    elapsed_r = 0;
    elapsed_count_l = 0;
    elapsed_count_r = 0;

    control_loop_step();

    V_l = control_loop_Y.V_l;
    V_r = control_loop_Y.V_r;
    PWM_l = control_loop_Y.PWM_l;
    PWM_r = control_loop_Y.PWM_r;

    if (control_loop_Y.Dir_r > .0f)
    {
        wiced_gpio_output_low(INA_R);
        wiced_gpio_output_high(INB_R);
    }
    else
    {
        wiced_gpio_output_high(INA_R);
        wiced_gpio_output_low(INB_R);
    }

    if (control_loop_Y.Dir_l > .0f)
    {
        wiced_gpio_output_low(INA_L);
        wiced_gpio_output_high(INB_L);
    }
    else
    {
        wiced_gpio_output_high(INA_L);
        wiced_gpio_output_low(INB_L);
    }

    wiced_pwm_stop(PWM_L);
    wiced_pwm_init(PWM_L, 20000.0f, PWM_l);
    wiced_pwm_start(PWM_L);

    wiced_pwm_stop(PWM_R);
    wiced_pwm_init(PWM_R, 20000.0f, PWM_r);
    wiced_pwm_start(PWM_R);

    /* Update log buffer with current values */
    if (log_stored_msg < ((LOG_SERVICE_TIMEOUT / CTRL_LOOP_TIMEOUT) * 4) - 4)
    {
        log_buffer[log_stored_msg++] = V_l;
        log_buffer[log_stored_msg++] = V_r;
        log_buffer[log_stored_msg++] = Vd;
        log_buffer[log_stored_msg++] = Rd;
    }

    return WICED_SUCCESS;
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
        /* TODO: halt */
    }

    publish_service();

    /* Register a function to process received UDP packets */
    wiced_rtos_register_timed_event( &process_udp_rx_event,
                                     WICED_NETWORKING_WORKER_THREAD,
                                     &process_received_udp_packet,
                                     RX_WAIT_TIMEOUT,
                                     0
                                   );

}

static wiced_result_t publish_service(){

    /* Get the micro IP address */
    uint32_t ipv4 = GET_IPV4_ADDRESS(ip_interface_address);

    sprintf(ip_descriptor, "%d.%d.%d.%d",(unsigned int)((ipv4 >> 24) & 0xFF),
            (unsigned int)((ipv4 >> 16) & 0xFF),
            (unsigned int)((ipv4 >>  8) & 0xFF),
            (unsigned int)((ipv4 >>  0) & 0xFF));

    /* Init service and publish on the network */
    gedday_init(ACTIVE_INTERFACE, ip_descriptor);
    gedday_add_service("Gedday_instance", "_murata._udp.local", DATA_PORT, 6, "Broadcom WICED = Wi-Fi for MCUs!");

    return WICED_SUCCESS;
}

static wiced_result_t log_service ()
{
    char log_message[LOG_BUFFER_SIZE * sizeof(real_T)];

    /*
    sprintf(log_message, "[%d bytes] RAW: %4s | Acc: %" PRIu16 " | Vel: %" PRIu16 " | Ref. vel.: %.1f",
            rx_data_length, rx_data, acc, vel, v_ref
            );
     */

    memcpy(log_message, log_buffer, (log_stored_msg) * sizeof(real_T));
    udp_printf(log_message, (log_stored_msg) * sizeof(real_T));

    log_stored_msg = 0;

    return WICED_SUCCESS;
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

    if (result == WICED_SUCCESS)
    {
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

        /* Parse debug commands */
		if(rx_data_length >= 4 &&
		   strncmp(rx_data, "scan", 4) == 0)
		{
		    wiced_wifi_scan_networks(scan_result_handler, NULL);
		}
		else if(rx_data_length >= 8 &&
		        strncmp(rx_data, "ifconfig", 8) == 0)
		{

		}
		else if (rx_data_length == 4)
		{
		    uint16_t rad, vel;

		    /* Convert the unsigned short integer values from network byte order to host byte order */
		    rad = ntohs(((uint16_t *) rx_data)[0]);
		    vel = ntohs(((uint16_t *) rx_data)[1]);

		    /* Normalize between 0.1 and inf */
		    Rd = (rad - 32768.0f)  / 32768.0f;
            Rd = 0.1/pow(fabs(Rd), 1.2);
            Rd = (rad >= 32768) ? -Rd : Rd;
            if(fabs(Vd) <= 0.1f) {
                Rd = (real_T) MAX_uint32_T;
            }

		    /* Normalize between -MAX_SPEED and MAX_SPEED */
		    Vd = (vel & 0x7fff) / 32767.0f * MAX_SPEED;
		    if ((0x8000 & vel) == 0)
		    {
                Vd = (MAX_SPEED - Vd) * (-1.0f);
		    }
		}

		/* Delete the received packet, it is no longer needed */
		wiced_packet_delete(packet);
    }

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

    free( malloced_scan_result );

    return WICED_SUCCESS;
}

static wiced_result_t udp_printf (char* buffer, uint16_t length)
{
    wiced_packet_t*          packet;
    char*                    tx_data;
    uint16_t                 available_data_length;

    uint16_t buffer_length = length;

    /* Create the UDP packet. Memory for the tx data is automatically allocated */
    if (wiced_packet_create_udp(&udp_debug_socket, buffer_length, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Copy buffer into tx_data which is located inside the UDP packet */
    memcpy(tx_data, buffer, buffer_length+1);

    /* Set the end of the data portion of the packet */
    wiced_packet_set_data_end(packet, (uint8_t*)tx_data + buffer_length);

    /* Send the UDP packet */
    if (wiced_udp_send(&udp_debug_socket, &ip_interface_broadcast, DEBUG_PORT, packet) != WICED_SUCCESS)
    {
        wiced_packet_delete(packet);  /* Delete packet, since the send failed */
    }

    /*
     * NOTE : It is not necessary to delete the packet created above, the packet
     *        will be automatically deleted *AFTER* it has been successfully sent
     */

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
