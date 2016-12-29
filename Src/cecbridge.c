/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2016 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, arg_ptr,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
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

#include "usb_device.h"

#include "atomic.h"
#include "utils.h"

#include "cecbridge.h"

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* Private function prototypes -----------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

static uint8_t promiscuousMode = 0;

#define FIRMWARE_REVISION "0.0.1"

#define BUF_SIZE 64

typedef struct {
    uint8_t logical_address;
    uint8_t bit_field[2];
    uint8_t physical_address[4];
    uint8_t device_type;
    char osd_name[15];
} cecbridge_t;

static cecbridge_t cecbridge = { 0xf, {0, 0}, {0, 0, 0, 0}, 0xf, "MyDevice" };

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
    hcec.Init.OwnAddress = (cecbridge.bit_field[0] << 8) + cecbridge.bit_field[1];

    if (HAL_CEC_Init(&hcec) != HAL_OK)
    {
        Error_Handler();
    }
}

static void tx_usb_message(char *msg)
{
    if (strlen(msg) > 0)
    {
        CDC_Transmit_FS((uint8_t *) msg, strlen(msg));
    }
}

static void tx_cec_message(uint8_t *buffer, size_t len)
{
    char response[9] = "?STA ";
    char *response_ptr = response + strlen(response);

    hcec.Init.InitiatorAddress = buffer[0] >> CEC_INITIATOR_LSB_POS;
    uint8_t destAddr = buffer[0] & 0xf;

    HAL_CEC_Transmit(&hcec, destAddr, buffer, len - 1, 200);
    /* after transmission, return to stand-by mode */
    hcec.State = HAL_CEC_STATE_STANDBY_RX;

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
        for (; *arg_ptr != '\0' && isspace(*arg_ptr); ++arg_ptr)
            ;

        if (!hex2bin(&cecbridge.logical_address, arg_ptr, 1))
        {

            if (isspace(*arg_ptr))
            {
                uint8_t bit_field[2];

                for (; *arg_ptr != '\0' && isspace(*arg_ptr); ++arg_ptr)
                    ;

                if (!hex2bin(cecbridge.bit_field, arg_ptr, 4))
                {
                    memcpy(cecbridge.bit_field, bit_field, 2);
                    hcec.Init.InitiatorAddress =
                            cecbridge.logical_address;
                    hcec.Init.OwnAddress = (cecbridge.bit_field[0]
                            << 8) + cecbridge.bit_field[1];
                }
            }
        }

        if (cmd == 'A')
        {
            strcpy(response, "?ADR ");
        }
        else
        {
            // TODO: save addresses in flash
            strcpy(response, "?BDR ");
        }

        char *response_ptr = bin2hex(response + strlen(response),
                &cecbridge.logical_address, 1);
        *response_ptr++ = ' ';
        response_ptr = bin2hex(response_ptr, cecbridge.bit_field, 4);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'C':   // display/update the configuration bits
        strcpy(response, "?CFG 0000\r\n"); // currently no support for higher functions
        break;
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
        }

        strcpy(response, "?OSD ");
        strcat(response, cecbridge.osd_name);
        strcat(response, "\r\n");
        break;
    case 'P':  // display/update the device’s physical address and ‘device type’
    {
        uint8_t physical_address[4];

        for (; *arg_ptr != '\0' && isspace(*arg_ptr); ++arg_ptr)
            ;

        if (!hex2bin(physical_address, arg_ptr, 4))
        {
            if (isspace(*arg_ptr))
            {
                for (; *arg_ptr != '\0' && isspace(*arg_ptr); ++arg_ptr)
                    ;

                if (!hex2bin(&cecbridge.device_type, arg_ptr, 1))
                {
                    memcpy(cecbridge.physical_address, physical_address, sizeof(cecbridge.physical_address));
                }
            }
        }

        strcpy(response, "?PHY ");
        char *response_ptr = bin2hex(response + strlen(response),
                cecbridge.physical_address, 4);
        *response_ptr++ = ' ';
        response_ptr = bin2hex(response_ptr, &cecbridge.device_type, 1);
        strcpy(response_ptr, "\r\n");
        break;
    }
    case 'Q':   // display/update the device’s retry count
        strcpy(response, "?QTY 1\r\n"); // retries are done by the host, so currently no support for changing it
        break;
    case 'R':   // report the device firmware revision level
        strcpy(response, "?REV " FIRMWARE_REVISION "\r\n");
        break;
    case 'X':   // transmit a CEC frame on the bus
    {
        uint8_t msg[CEC_MAX_MSG_SIZE];
        uint8_t *msg_ptr = msg;
        int len = 0;

        for (; *arg_ptr; arg_ptr++)
        {
            if (!isxdigit(*arg_ptr))
                continue;
            if (len == CEC_MAX_MSG_SIZE)
                break;
            if (!hex2bin(msg_ptr++, arg_ptr++, 2))
            {
                ++len;
            }
            else
            {
                len = 0;
                break;
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

static void rx_cec_message()
{
    uint8_t cec_buffer[CEC_MAX_MSG_SIZE];
    char response[BUF_SIZE] = "?REC";
    char *response_ptr = response + strlen(response);

    HAL_StatusTypeDef hal_status = HAL_CEC_Receive(&hcec, cec_buffer,
            200);
    if (hal_status == HAL_TIMEOUT)
    {
        return;
    }
    if (promiscuousMode || (*hcec.pRxBuffPtr & 0xf) == 0xf
            || hcec.Init.OwnAddress
                    == (*hcec.pRxBuffPtr & 0xf))
    {
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
        }
        strcpy(response_ptr, "\r\n");

        tx_usb_message(response);
    }
}

int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_USB_DEVICE_Init();

    init_cec();

    memset(&hcec, 0, sizeof(hcec));

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

// ======================== unused callbacks  ===================

/**
 * @brief Tx Transfer completed callback
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *hcec)
{
    // not used, just to make the linker happy
}

/**
 * @brief Rx Transfer completed callback
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *hcec)
{
    // not used, just to make the linker happy
}

/**
 * @brief CEC error callbacks
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec)
{
    // not used, just to make the linker happy
}

