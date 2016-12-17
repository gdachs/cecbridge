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
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
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

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_cec.h"
#include "stm32f0xx_it.h"

#include "usb_device.h"

#include "cecbridge.h"

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* Private function prototypes -----------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

uint8_t promiscuousMode = 0;

void logMessage(char *format, ...)
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
            CDC_Transmit_FS((uint8_t *) newbuffer, strlen(newbuffer));
            free(newbuffer);
        }
        free(buffer);
    }
}

void sendCecError(uint8_t errCode, uint32_t cecErrorCode)
{
    char *errStr = NULL;

    asprintf(&errStr, "E02%x:02%lx:02%lx:02%lx:02%lx\r\n", errCode,
            cecErrorCode >> 24, (cecErrorCode >> 16) & 0xff,
            (cecErrorCode >> 8) & 0xff, cecErrorCode & 0xff);

    if (errStr != NULL)
    {
        CDC_Transmit_FS((uint8_t *) errStr, strlen(errStr));
        free(errStr);
    }
}

void sendError(uint8_t err)
{
    sendCecError(err, HAL_CEC_ERROR_NONE);
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

CEC_HandleTypeDef *getCecHandle(void)
{
    static CEC_HandleTypeDef cecHandle;
    static CEC_HandleTypeDef *cecHandlePtr = NULL;

    if (cecHandlePtr == NULL)
    {
        memset(&cecHandle, 0, sizeof(cecHandle));
        cecHandlePtr = &cecHandle;
    }
    return cecHandlePtr;
}

/* HDMI_CEC init function */
static void initCec(void)
{
    CEC_HandleTypeDef *cecHandle = getCecHandle();

    cecHandle->Instance = CEC;
    cecHandle->Init.SignalFreeTime = CEC_DEFAULT_SFT;
    cecHandle->Init.Tolerance = CEC_STANDARD_TOLERANCE;
    cecHandle->Init.BRERxStop = CEC_RX_STOP_ON_BRE;
    cecHandle->Init.BREErrorBitGen = CEC_BRE_ERRORBIT_NO_GENERATION;
    cecHandle->Init.LBPEErrorBitGen = CEC_LBPE_ERRORBIT_NO_GENERATION;
    cecHandle->Init.BroadcastMsgNoErrorBitGen =
    CEC_BROADCASTERROR_ERRORBIT_GENERATION;
    cecHandle->Init.SignalFreeTimeOption = CEC_SFT_START_ON_TXSOM;
    cecHandle->Init.OwnAddress = 0;
    cecHandle->Init.ListenMode = CEC_FULL_LISTENING_MODE;
    cecHandle->Init.InitiatorAddress = 0xf;

    if (HAL_CEC_Init(cecHandle) != HAL_OK)
    {
        sendCecError(CEC_ERROR, cecHandle->ErrorCode);
    }
}

static char *escapePayload(uint8_t *bufsrc, size_t len)
{
    char *result = NULL;

    if (bufsrc != NULL
            && len > 0&& (result = calloc(len * 3 + 1, sizeof(char))) != NULL)
    {
        for (int i = 0; i < len; ++i)
        {
            sprintf(&result[i * 3], "%02x:", bufsrc[i]);
        }

        result[strlen(result) - 1] = 0;
    }
    return result;
}

static uint8_t *unescapePayload(char *dataStr, size_t *len)
{
    uint8_t *result = NULL;
    *len = 0;

    if (dataStr != NULL
            && (result = calloc((strlen(dataStr) + 1) / 3, sizeof(uint8_t)))
                    != NULL)
    {
        uint8_t *resultPtr = result;

        for (int i = 0; i < strlen(dataStr); ++i)
        {
            char c = dataStr[i];

            if (!isdigit(c) && !isalpha(c) && c != ':')
            {
                return NULL;
            }

            switch ((i + 1) % 3)
            {
            case 1:
                *resultPtr = (isalpha(c) ? c - 'A' + 10 : c - '0')
                        << CEC_INITIATOR_LSB_POS;
                break;
            case 2:
                *resultPtr |= isalpha(c) ? c - 'A' + 10 : c - '0';
                break;
            default:
                ++resultPtr;
                break;
            }
            *len = resultPtr - result;
        }
    }
    return result;
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

    logMessage("begin of main loop");

    initCec();

    /* Infinite loop */
    while (1)
    {
        static uint8_t cecBuffer[64];

        HAL_CEC_Receive_IT(getCecHandle(), cecBuffer);
    }
}


/** Pinout Configuration
 */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void transmitCecCommand(uint8_t *buffer, size_t len)
{
    getCecHandle()->Init.InitiatorAddress = buffer[0] >> CEC_INITIATOR_LSB_POS;
    uint8_t destAddr = buffer[0] & 0xf;

    if (HAL_CEC_Transmit(getCecHandle(), destAddr, buffer, len - 1, 200) != HAL_OK)
    {
        sendCecError(CEC_ERROR, getCecHandle()->ErrorCode);
    }
    else
    {
        sendError(NO_ERROR);
    }
}

void handleCommand(char *command)
{
    switch (toupper(command[0]))
    {
    case 'E':
        CDC_Transmit_FS((uint8_t *) command, strlen(command));
        break;
    case 'L':
        if (isalpha(command[1]) || isdigit(command[1]))
        {
            uint8_t logicalAddress =
            isalpha(command[1]) ?
                    (toupper(command[1]) - 'A' + 10) : command[1] - '0';
            if (logicalAddress > 0)
            {
                getCecHandle()->Init.OwnAddress = 1 << (logicalAddress - 1);
                HAL_CEC_DeInit(getCecHandle());
                HAL_CEC_Init(getCecHandle());
            }
        }
        break;
    case 'P':
        promiscuousMode = command[1] == '1' ? 1 : 0;
        break;
    case 'T':
    {
        size_t transmitBufferLen = 0;
        uint8_t *transmitBuffer = unescapePayload(&command[1],
                &transmitBufferLen);
        if (transmitBuffer != NULL && transmitBufferLen > 0)
        {
            transmitCecCommand(transmitBuffer, transmitBufferLen);

            free(transmitBuffer);
        }
    }
        break;
    default:
        // shouldn't happen
        break;
    }
}

void USB_CDC_Recv_CB(uint8_t *data, uint32_t len)
{
    static char receiveBuffer[128];
    static char *receiveBufferPtr = receiveBuffer;

    for (int i = 0; i < len; ++i)
    {
        if (data[i] == '\r')
        {
            // do nothing
        }
        else if (data[i] == '\n')
        {
            // command letter at the beginning of the buffer?
            if (strchr("ELPT", toupper(receiveBuffer[0])) != NULL)
            {
                *receiveBufferPtr++ = 0;
                handleCommand(receiveBuffer);

                // reset buffer pointer to the beginning of the buffer
                receiveBufferPtr = receiveBuffer;
            }
            else
            {
                // corrupt buffer, reset
                receiveBufferPtr = receiveBuffer;
            }
        }
        else
        {
            *receiveBufferPtr++ = data[i];
        }
    }
}

/**
 * @brief Tx Transfer completed callback
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *cecHandle)
{

    /* after transmission, return to stand-by mode */
    cecHandle->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
 * @brief Rx Transfer completed callback
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *cecHandle)
{
    /* Reminder: hcec->RxXferSize is the sum of opcodes + operands
     * (0 to 14 operands max).
     * If only a header is received, hcec->RxXferSize = 0 */
    if (promiscuousMode || (*cecHandle->pRxBuffPtr & 0xf) == 0xf
            || cecHandle->Init.OwnAddress == (*cecHandle->pRxBuffPtr & 0xf))
    {
        char *escapedPayload = escapePayload(cecHandle->pRxBuffPtr,
                cecHandle->RxXferSize + 1);
        if (escapedPayload != NULL)
        {
            char *response = NULL;

            asprintf(&response, "R%s\r\n", escapedPayload);

            if (response != NULL)
            {
                CDC_Transmit_FS((uint8_t *) response, strlen(response));
                free(response);
            }
            free(escapedPayload);
        }
    }
    cecHandle->RxXferSize = 0;
    cecHandle->ErrorCode = HAL_CEC_ERROR_NONE;
    /* return to stand-by mode */
    cecHandle->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
 * @brief CEC error callbacks
 * @param cecHandle: CEC handle
 * @retval None
 */
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *cecHandle)
{
    sendCecError(CEC_ERROR, cecHandle->ErrorCode);
    cecHandle->RxXferSize = 0;
    cecHandle->ErrorCode = HAL_CEC_ERROR_NONE;
    cecHandle->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
}

