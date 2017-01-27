/*
 * cecbridge for SolidPC
 *
 * Copyright 2016 Gerald Dachs <gda@dachsweb.de>
 * based on work from kliment
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version of 2 of the License, or (at your
 * option) any later version. See the file COPYING in the main directory of
 * this archive for more details.
 */

/* Includes ------------------------------------------------------------------*/
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <stdbool.h>

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_cec.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx_hal_conf.h"

#include "usb_device.h"

#include "atomic.h"
#include "utils.h"

#include "cecbridge.h"

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* Private function prototypes -----------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#define FIRMWARE_REVISION "1"

#define BUF_SIZE 64

#define OPCODE_GIVE_DECK_STATUS         0x1A    // give deck status
#define OPCODE_DECK_STATUS              0x1B    // deck status
#define OPCODE_REPORT_OSD_NAME          0x46    // report OSD name
#define OPCODE_SET_OSD_NAME             0x47    // set OSD name
#define OPCODE_GIVE_PHYSICAL_ADDRESS    0x83    // give physical address (and device type)
#define OPCODE_REPORT_PHYSICAL_ADDRESS  0x84    // report physical address (and device type)
#define OPCODE_REPORT_VENDOR_ID         0x8C    // report vendor id (answer is ‘not supported’)
#define OPCODE_MENU_REQUEST             0x8D    // ‘menu’ request
#define OPCODE_MENU_STATUS              0x8E    // respond to ‘menu’ request saying menu is active.
                                                // This allows some commands to be forwarded from
                                                // the remote control to the device.
#define OPCODE_GIVE_DEVICE_POWER_STATUS 0x8F    // give device power status
#define OPCODE_REPORT_POWER_STATUS      0x90    // report power status
#define OPCODE_GET_CEC_VERSION          0x9F    // CEC version
#define OPCODE_CEC_VERSION              0x9E    // give CEC version

#define HIGHER_LEVEL_PROTOCOL_FLAG      (1<<0)  // enables higher level protocol if set
#define LOGICAL_ADDRESS_DETECTION_FLAG  (1<<1)  // enables logical address detection is
#define HOST_WAKEUP_FLAG                (1<<2)  // allows wakeup of host if set and
                                                // higher level protocoll is set too

__attribute__((__section__(".persistent")))
const volatile static uint32_t flash[(sizeof(cecbridge_t) + sizeof(uint32_t)) / sizeof(uint32_t)] = { 0xdeadbeef };

static cecbridge_t cecbridge2persist = {
        0xf,        // logical address, default is broadcast
        { 0, 0 },   // bit field for masking on which logical addresses to respond
        { 0, 0 },   // physical address
        0x4,        // device type, set to playback 1
        5,          // retry count
        { 0, 0 },   // configuration bits
        "MyDevice", // OSD name
    };

static cecbridge_t cecbridge;

static int8_t host_power_state = 1;          // Host power state is on

typedef enum {
    buf_empty,      // buffer is empty
    buf_busy,       // buffer gets filled by interrupt
    buf_ready,      // buffer is ready for processing
} buf_state_t;

typedef struct {
    char buf[BUF_SIZE];
    buf_state_t buf_state;
} usb_buffer_t;

static usb_buffer_t usb_in_buffer = { "", buf_empty };

CEC_HandleTypeDef hcec;

void set_host_power_state(int8_t state)
{
    host_power_state = state;
}

int8_t get_host_power_state()
{
    return host_power_state;
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
            | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_CEC;
    PeriphClkInit.CecClockSelection = RCC_CECCLKSOURCE_HSI;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* HDMI_CEC init function */
static void init_cec(void)
{
    hcec.Instance = CEC;
    hcec.Init.SignalFreeTime = CEC_DEFAULT_SFT;
    hcec.Init.Tolerance = CEC_STANDARD_TOLERANCE;
    hcec.Init.BRERxStop = CEC_RX_STOP_ON_BRE;
    hcec.Init.BREErrorBitGen = CEC_BRE_ERRORBIT_NO_GENERATION;
    hcec.Init.LBPEErrorBitGen = CEC_LBPE_ERRORBIT_NO_GENERATION;
    hcec.Init.BroadcastMsgNoErrorBitGen =
    CEC_BROADCASTERROR_ERRORBIT_GENERATION;
    hcec.Init.SignalFreeTimeOption = CEC_SFT_START_ON_TXSOM;
    hcec.Init.ListenMode = CEC_FULL_LISTENING_MODE;
    hcec.Init.InitiatorAddress = cecbridge.logical_address;
    hcec.Init.OwnAddress = ((cecbridge.bit_field[0] << 8)
            + cecbridge.bit_field[1]) | (1 << cecbridge.logical_address);

    if (HAL_CEC_Init(&hcec) != HAL_OK)
    {
        Error_Handler();
    }
}

static void host_wakeup()
{
    for (int i = 0; i < 2; ++i)
    {
        USB->CNTR |= USB_CNTR_RESUME;
        HAL_Delay(10);
        USB->CNTR &= (uint16_t) (~(USB_CNTR_RESUME));
    }
}

uint8_t tx_usb_message(char *msg)
{
    uint8_t ret = USBD_OK;

    if (strlen(msg) > 0)
    {
        ret = CDC_Transmit_FS((uint8_t *) msg, strlen(msg) + 1);
    }

    return ret;
}

#if 0
static void log_message(char *format, ...)
{
    char *buffer = NULL;
    va_list aptr;

    va_start(aptr, format);
    vasprintf(&buffer, format, aptr);
    va_end(aptr);

    if (buffer != NULL)
    {
        char *newbuffer = NULL;

        asprintf(&newbuffer, "#%s\r\n", buffer);

        if (newbuffer != NULL)
        {
            tx_usb_message(newbuffer);
            free(newbuffer);
        }
        free(buffer);
    }
}
#endif

static void tx_cec_message(uint8_t *msg, size_t len)
{
    char response[9] = "?STA ";
    char *response_ptr = response + strlen(response);
    hcec.Init.InitiatorAddress = msg[0] >> CEC_INITIATOR_LSB_POS;
    uint8_t destAddr = msg[0] & 0xf;

    for (int i = 0; i < cecbridge.retry_count; ++i)
    {
        HAL_StatusTypeDef state  = HAL_CEC_Transmit(&hcec, destAddr, &msg[1], len - 1, 200);
        hcec.State = HAL_CEC_STATE_READY;

        if (state == HAL_OK)
        {
            break;
        }
    }

    if (hcec.ErrorCode == HAL_CEC_ERROR_NONE)
    {
        *response_ptr++ = '1';
    }
    else if (hcec.ErrorCode == HAL_CEC_ERROR_TXACKE)
    {
        *response_ptr++ = '2';
    }
    else
    {
        *response_ptr++ = '3';
    }
    strcpy(response_ptr, "\r\n");

    tx_usb_message(response);
}

static int flash_cmp(uint8_t *src, uint32_t size)
{
    uint8_t *dest = (uint8_t *)flash;

    for (int i = 0; i < size; ++i)
    {
        if (dest[i] != src[i])
        {
            return dest - src;
        }
    }
    return 0;
}

static void persist_cecbridge()
{
    uint32_t *src = (uint32_t *) &cecbridge2persist;

    if (flash_cmp((uint8_t *)src, sizeof(cecbridge_t)))
    {
        HAL_FLASH_Unlock();

        FLASH_PageErase((uint32_t)&flash);

        for (int i = 0; i < sizeof(flash) / sizeof(flash[0]); ++i)
        {
            FLASH->CR = 0;

            for (int j = 0;
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                            (uint32_t) &flash[i], (uint64_t) src[i]) != HAL_OK;
                    ++j)
            {
                FLASH->CR = 0;
                if (j > 3)
                    return;
            }
        }

        HAL_FLASH_Lock();
    }
}

static uint8_t higher_level_handler(uint8_t *cec_buffer)
{
    uint8_t buf[CEC_MAX_MSG_SIZE];
    uint8_t *buf_ptr = buf;
    uint8_t opcode = cec_buffer[1];
    uint8_t initiator = cec_buffer[0] >> 4;
    uint8_t destination = cec_buffer[0] & 0x0f;
    uint8_t handled = 0;

    if ((cecbridge.logical_address != 0x0f) 
            && (cecbridge.configuration_bits[1] & HIGHER_LEVEL_PROTOCOL_FLAG)
            && (destination == cecbridge.logical_address || destination == 0x0f))
    {
        handled = 1;

        *buf_ptr++ = cecbridge.logical_address << 4 | initiator;

        switch (opcode)
        {
        case OPCODE_GIVE_DECK_STATUS:                       // deck status
            *buf_ptr++ = OPCODE_DECK_STATUS;
            *buf_ptr++ = 0x1a;                              // Stop
            break;
        case OPCODE_REPORT_OSD_NAME:                        // report OSD name
            *buf_ptr++ = OPCODE_SET_OSD_NAME;
            memcpy(buf_ptr, cecbridge.osd_name, strlen(cecbridge.osd_name));
            buf_ptr += strlen(cecbridge.osd_name);
            break;
        case OPCODE_GIVE_PHYSICAL_ADDRESS: // report physical address (and device type)
            buf[0] = cecbridge.logical_address << 4 | 0x0f; // broadcast to all
            *buf_ptr++ = OPCODE_REPORT_PHYSICAL_ADDRESS;
            *buf_ptr++ = cecbridge.physical_address[0];
            *buf_ptr++ = cecbridge.physical_address[1];
            *buf_ptr++ = cecbridge.device_type;
            break;
        case OPCODE_REPORT_VENDOR_ID: // report vendor id (answer is ‘not supported’)
            *buf_ptr++ = 0;                                 // feature abort
            *buf_ptr++ = 0x1b;
            *buf_ptr++ = 0;
            break;
        case OPCODE_MENU_REQUEST: // respond to ‘menu’ request saying menu is active.
            *buf_ptr++ = OPCODE_MENU_STATUS;
            *buf_ptr++ = 0;                                 // active
            break;
        case OPCODE_GIVE_DEVICE_POWER_STATUS:             // report power status
            *buf_ptr++ = OPCODE_REPORT_POWER_STATUS;
            *buf_ptr++ = get_host_power_state() ? 0 : 2;
            break;
        case OPCODE_GET_CEC_VERSION:                       // report CEC version
            *buf_ptr++ = OPCODE_CEC_VERSION;
            *buf_ptr++ = 5;
            break;
        default:
            handled = 0;
        }

        if (handled)
        {
            tx_cec_message(buf, buf_ptr - buf);

            if (!get_host_power_state()
                    && (cecbridge.configuration_bits[1] & HOST_WAKEUP_FLAG))
            {
                host_wakeup();
            }
        }
    }

    return handled;
}

static void rx_cec_message()
{
    uint8_t cec_buffer[CEC_MAX_MSG_SIZE];
    char response[BUF_SIZE] = "?REC";
    char *response_ptr = response + strlen(response);

    HAL_StatusTypeDef hal_status = HAL_CEC_Receive(&hcec, cec_buffer, 200);
    hcec.State = HAL_CEC_STATE_READY;

    if (hal_status == HAL_TIMEOUT || hal_status == HAL_BUSY)
    {
        return;
    }

    if (higher_level_handler(cec_buffer))
    {
        return;
    }

    for (int i = 0; i <= hcec.RxXferSize; ++i)
    {
        *response_ptr++ = ' ';
        response_ptr = bin2hex(response_ptr, &cec_buffer[i], 1);
    }
    *response_ptr++ = ' ';

    if (hcec.ErrorCode == HAL_CEC_ERROR_NONE)
    {
        *response_ptr++ = '1';
    }
    else if (hcec.ErrorCode == HAL_CEC_ERROR_RXACKE)
    {
        *response_ptr++ = '2';
    }
    else
    {
        *response_ptr++ = '3';
        *response_ptr++ = ' ';
        response_ptr = hex_byte_pack(response_ptr, (hcec.ErrorCode >> 24) & 0xff);
        response_ptr = hex_byte_pack(response_ptr, (hcec.ErrorCode >> 16) & 0xff);
        response_ptr = hex_byte_pack(response_ptr, (hcec.ErrorCode >> 8) & 0xff);
        response_ptr = hex_byte_pack(response_ptr, hcec.ErrorCode & 0xff);
    }
    strcpy(response_ptr, "\r\n");

    tx_usb_message(response);
}

static void handle_usb_message(char *command)
{
    char response[BUF_SIZE] = "";
    char cmd = toupper(command[0]);
    char *arg_ptr = &command[1];

    switch (cmd)
    {
    case 'A': // display/update the device’s current logical address and address ‘bit-field’ (also see ‘b’)
    case 'B':   // like A, but logical address gets committed to flash
    {
        int logical_address;

        arg_ptr = skip_white_space(arg_ptr);

        if ((logical_address = hex_to_bin(*arg_ptr++)) >= 0)
        {
            uint8_t bit_field[2];

            cecbridge.logical_address = logical_address;

            arg_ptr = skip_white_space(arg_ptr);

            if (!hex2bin(bit_field, arg_ptr, 2))
            {
                memcpy(cecbridge.bit_field, bit_field, 2);
            }
            init_cec();
        }

        if (cmd == 'A')
        {
            strcpy(response, "?ADR ");
        }
        else
        {
            cecbridge2persist.logical_address = cecbridge.logical_address;
            memcpy(cecbridge2persist.bit_field, cecbridge.bit_field, 2);
            persist_cecbridge();
            strcpy(response, "?BDR ");
        }

        char *response_ptr = response + strlen(response);
        *response_ptr++ = hex_asc_lo(cecbridge.logical_address);
        *response_ptr++ = ' ';
        response_ptr = bin2hex(response_ptr, cecbridge.bit_field, 2);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'C':   // display/update the configuration bits
    {
        uint8_t configuration_bits[4];

        arg_ptr = skip_white_space(arg_ptr);

        if (!hex2bin(configuration_bits, arg_ptr, 2))
        {
            memcpy(cecbridge.configuration_bits, configuration_bits, sizeof(cecbridge.configuration_bits));
            memcpy(cecbridge2persist.configuration_bits, configuration_bits, sizeof(cecbridge.configuration_bits));
            persist_cecbridge();
        }

        strcpy(response, "?CFG ");
        char *response_ptr = bin2hex(response + strlen(response),
                cecbridge.configuration_bits, 2);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'M':   // mirror a text string back to the host
        strcpy(response, "?MIR");
        strcat(response, arg_ptr);
        strcat(response, "\r\n");
        break;
    case 'O':   // display/update the device’s OSD (on screen display) name
        if (*arg_ptr != '\0')
        {
            strncpy(cecbridge.osd_name, arg_ptr,
                    sizeof(cecbridge.osd_name) - 1);
            cecbridge.osd_name[sizeof(cecbridge.osd_name) - 1] = '\0';
            memcpy(cecbridge2persist.osd_name, cecbridge.osd_name, sizeof(cecbridge2persist.osd_name));
            persist_cecbridge();
        }

        strcpy(response, "?OSD ");
        strcat(response, cecbridge.osd_name);
        strcat(response, "\r\n");
        break;
    case 'P':  // display/update the device’s physical address and ‘device type’
    {
        uint8_t physical_address[2];

        arg_ptr = skip_white_space(arg_ptr);

        if (!hex2bin(physical_address, arg_ptr, 2))
        {
            int device_type;

            arg_ptr += 4;

            memcpy(cecbridge.physical_address, physical_address,
                    sizeof(cecbridge.physical_address));
            memcpy(cecbridge2persist.physical_address, physical_address,
                    sizeof(cecbridge.physical_address));

            arg_ptr = skip_white_space(arg_ptr);

            if ((device_type = hex_to_bin(*arg_ptr)) >= 0)
            {
                cecbridge.device_type = device_type;
                cecbridge2persist.device_type = device_type;
                persist_cecbridge();
            }
        }

        strcpy(response, "?PHY ");
        char *response_ptr = bin2hex(response + strlen(response),
                cecbridge.physical_address, 2);
        *response_ptr++ = ' ';
        *response_ptr++ = hex_asc_lo(cecbridge.device_type);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'Q':   // display/update the device’s retry count
    {
        int retry_count;

        arg_ptr = skip_white_space(arg_ptr);

        if ((retry_count = hex_to_bin(*arg_ptr)) >= 0)
        {
            cecbridge.retry_count = retry_count;
        }

        strcpy(response, "?QTY ");
        char *response_ptr = response + strlen(response);
        *response_ptr++ = hex_asc_lo(cecbridge.retry_count);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'R':   // report the device firmware revision level
        strcpy(response, "?REV " FIRMWARE_REVISION "\r\n");
        break;
    case 'X':   // transmit a CEC frame on the bus
    {
        uint8_t msg[CEC_MAX_MSG_SIZE];
        uint8_t *msg_ptr = msg;
        int len = 0;
        uint8_t nibble;

        arg_ptr = skip_white_space(arg_ptr);
        if ((nibble = hex_to_bin(*arg_ptr++)) >= 0)
        {
            *msg_ptr++ = (cecbridge.logical_address << 4) | nibble;
            ++len;

            for (; *arg_ptr; arg_ptr++)
            {
                if (!isxdigit(*arg_ptr))
                    continue;
                if (len == CEC_MAX_MSG_SIZE)
                    break;
                if ((nibble = hex_to_bin(*arg_ptr++)) >= 0)
                {
                    *msg_ptr = nibble;

                    if ((nibble = hex_to_bin(*arg_ptr)) >= 0)
                    {
                        *msg_ptr = (*msg_ptr << 4) | nibble;
                    }
                    ++msg_ptr;
                    ++len;
                }
                else
                {
                    len = 0;
                    break;
                }
            }
        }
        if (len > 0)
        {
            tx_cec_message(msg, len);
        }
        break;
    }
    default:
        // shouldn't happen
        break;
    }

    tx_usb_message(response);
}

int main(void)
{
    /* fill cecbridge with persisted data */
    uint8_t *src = (uint8_t *)&cecbridge2persist;
    uint8_t *dest = (uint8_t *)&cecbridge;

    if (flash[0] != 0xdeadbeef)
    {
        uint8_t *src = (uint8_t *)flash;
        uint8_t *dest = (uint8_t *)&cecbridge2persist;

        for (int i = 0; i < sizeof(cecbridge_t); ++i)
        {
           dest[i] = src[i];
        }
    }

    for (int i = 0; i < sizeof(cecbridge_t); ++i)
    {
       dest[i] = src[i];
    }

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_USB_DEVICE_Init();

    init_cec();

    while (1)
    {
        char buf[BUF_SIZE] = "";

        // read CEC in polling mode
        rx_cec_message();

        ATOMIC_BLOCK()
        {
            if (usb_in_buffer.buf_state == buf_ready)
            {
                strncpy(buf, usb_in_buffer.buf, BUF_SIZE);
                usb_in_buffer.buf_state = buf_empty;
            }
        }
        if (buf[0] != '\0')
        {
            handle_usb_message(buf);
        }
    }
}

/** Pinout Configuration
 */
static void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void USB_CDC_Recv_CB(uint8_t *data, uint32_t len)
{
    static char *buf_ptr = usb_in_buffer.buf;

    for (int i = 0; i < len; ++i)
    {
        if (usb_in_buffer.buf_state != buf_busy && data[i] != '!')
        {
            continue;
        }

        if (data[i] == '\r' || data[i] == '\n' || data[i] == '~')
        {
            *buf_ptr = '\0';
            buf_ptr = usb_in_buffer.buf;
            usb_in_buffer.buf_state = buf_ready;
            continue;
        }
        else if (data[i] == '!')
        {
            buf_ptr = usb_in_buffer.buf;
            usb_in_buffer.buf_state = buf_busy;
            continue;
        }

        if (buf_ptr - usb_in_buffer.buf >= sizeof(usb_in_buffer.buf) - 1)
        {
            // throwing away garbage
            buf_ptr = usb_in_buffer.buf;
            usb_in_buffer.buf_state = buf_empty;
        }
        *buf_ptr++ = data[i];
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    // currently empty
}
