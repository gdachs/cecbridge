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

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_cec.h"
#include "stm32f0xx_it.h"

#include "usb_device.h"

#include "cecbridge.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t rxcmdbuf[128];
uint8_t rxcmdpos = 0;
uint8_t promiscuousMode = 0;

#define CEC_MAX_PAYLOAD		16
uint32_t ErrorCode = 0x0;
#if 0
uint8_t Tab_Rx[CEC_MAX_PAYLOAD]; /* Received data buffer. Max size = 16 bytes
 * header + opcode followed by up to 14 operands */
uint8_t Tab_Tx[CEC_MAX_PAYLOAD-1]; /* Transmitted data buffer.
 * header is not included in Tab_Tx.
 *  Max size = 15 bytes.
 *  one opcode followed by up to 14 operands.
 *  When payload size = 0, only the header is sent
 *  (ping operation) */
uint8_t ReceivedFrame = 0x0; /* Set when a reception occurs */
uint16_t NbOfReceivedBytes = 0x0; /* Number of received bytes in addition to the header.
 * when a ping message has been received (header
 * only), NbOfReceivedBytes = 0 */
uint8_t StartSending = 0x0; /* Set when a transmission is triggered by the user */
uint32_t TxSize = 0x0; /* Number of bytes to transmit in addition to the header.
 * In case of ping operation (only the header sent),
 * TxSize = 0 */
uint8_t DestinationAddress; /* Destination logical address */
uint8_t LogicalAddress; /* CEC IP Initiator logical address */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Error_Handler(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#if 0
static void CEC_FlushRxBuffer(void);
#endif

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void sendError(ErrorCode err)
{
    char errStr[7];

    snprintf(errStr, sizeof(errStr), "E02%x\r\n", err);
    CDC_Transmit_FS((uint8_t *) errStr, strlen(errStr));
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
        Error_Handler();
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

    initCec();

    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
    //HAL_CEC_Transmit(hcec,0xf,0,0,0);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        static uint8_t cecBuffer[64];

        HAL_CEC_Receive_IT(getCecHandle(), cecBuffer);
    }

#if 0
    if(usb_inttxlen)
    {
        CDC_Transmit_FS(intbuf,usb_inttxlen);
        usb_inttxlen=0;
    }
    HAL_CEC_Receive_IT(&hcec,(uint8_t *)&Tab_Rx);
    if (ReceivedFrame == 1)
    {
        sprintf((char *)respbuf, "R%s\r\n", escapePayload(Tab_Rx, NbOfReceivedBytes));
        if(promiscuousMode || (Tab_Rx[0]&0xf)==0xf || hcec.Init.OwnAddress==(Tab_Rx[0]&0xf))
        {
            CDC_Transmit_FS(respbuf, strlen((char *)respbuf));
        }
    }
    else if (ReceivedFrame == 2)
    {
        /*#define HAL_CEC_ERROR_NONE    (uint32_t) 0x0    // !< no error
         #define HAL_CEC_ERROR_RXOVR   CEC_ISR_RXOVR          //!< CEC Rx-Overrun
         #define HAL_CEC_ERROR_BRE     CEC_ISR_BRE            //!< CEC Rx Bit Rising Error
         #define HAL_CEC_ERROR_SBPE    CEC_ISR_SBPE           //!< CEC Rx Short Bit period Error
         #define HAL_CEC_ERROR_LBPE    CEC_ISR_LBPE           //!< CEC Rx Long Bit period Error
         #define HAL_CEC_ERROR_RXACKE  CEC_ISR_RXACKE         //!< CEC Rx Missing Acknowledge
         #define HAL_CEC_ERROR_ARBLST  CEC_ISR_ARBLST         //!< CEC Arbitration Lost
         #define HAL_CEC_ERROR_TXUDR   CEC_ISR_TXUDR          //!< CEC Tx-Buffer Underrun
         #define HAL_CEC_ERROR_TXERR   CEC_ISR_TXERR          //!< CEC Tx-Error
         #define HAL_CEC_ERROR_TXACKE  CEC_ISR_TXACKE         //!< CEC Tx Missing Acknowledge
         */
        sprintf((char *)respbuf, "F\r\n");
        CDC_Transmit_FS(respbuf, strlen((char *)respbuf));

    }
    usb_txlen=0;
    ReceivedFrame = 0;

    /* USER CODE END WHILE */
    uint8_t rxchar;
    /* USER CODE BEGIN 3 */
    while(bufrpos!=bufwpos)
    {
        rxchar=cmdbuf[bufrpos];
        if(rxchar!='\n' && rxchar!='\r' && rxcmdpos<96)
        {
            rxcmdbuf[rxcmdpos]=rxchar;
            rxcmdpos++;
        }
        else
        {
            rxcmdbuf[rxcmdpos]=0;
            rxcmdbuf[rxcmdpos+1]=0;
            rxcmdbuf[rxcmdpos+2]=0;
            if(rxcmdpos>95)
            {

            }
            //parsing command
            if(rxcmdbuf[0]=='L'||rxcmdbuf[0]=='l')
            {
                uint8_t c=fromasc(rxcmdbuf[1]);
                if(c==0xf0 && rxcmdpos>1) c=fromasc(rxcmdbuf[2]);
                if(c!=0xf0)
                {
                    hcec.Init.OwnAddress=c;
                    HAL_CEC_DeInit(&hcec);
                    HAL_CEC_Init(&hcec);
                    respbuf[0]='L';
                    respbuf[1]=toasc(c);
                    respbuf[2]='\r';
                    respbuf[3]='\n';
                    usb_txlen=4;
                }
                else
                {
                    respbuf[0]='L';
                    respbuf[1]='P';
                    respbuf[2]='\r';
                    respbuf[3]='\n';
                    usb_txlen=4;

                }
            }
            if(rxcmdbuf[0]=='P'||rxcmdbuf[0]=='p')
            {
                uint8_t c=rxcmdbuf[1];
                if(c=='0'||c=='1')
                {
                    if(c=='1')promiscuousMode=1;
                    else promiscuousMode=0;
                    respbuf[0]='L';
                    respbuf[1]=c;
                    respbuf[2]='\r';
                    respbuf[3]='\n';
                    usb_txlen=4;
                }
                else
                {
                    respbuf[0]='F';
                    respbuf[1]='P';
                    respbuf[2]='\r';
                    respbuf[3]='\n';
                    usb_txlen=4;
                }
            }
            if(rxcmdbuf[0]=='E'||rxcmdbuf[0]=='e')
            {
                respbuf[0]='E';
                respbuf[1]='C';
                respbuf[2]='E';
                respbuf[3]='C';
                respbuf[4]='B';
                respbuf[5]='V';
                respbuf[6]='1';
                respbuf[7]='\r';
                respbuf[8]='\n';
                usb_txlen=9;

            }
            if(rxcmdbuf[0]=='T'||rxcmdbuf[0]=='t')
            {
                int offset=1;
                while(rxcmdbuf[offset]==' ')
                {
                    offset++;
                }
                uint8_t sourceaddr=fromasc(rxcmdbuf[offset]);
                if(sourceaddr==0xf0)
                {
                    respbuf[1]='S';
                    goto fail;
                }
                offset++;
                uint8_t destaddr=fromasc(rxcmdbuf[offset]);
                if(destaddr==0xf0)
                {
                    respbuf[1]='D';
                    goto fail;
                }
                offset++;
                int txlen=0;
                uint8_t nextbyte;
                while(rxcmdbuf[offset]!=0 && txlen<16)
                {
                    if(rxcmdbuf[offset]!=':')
                    {
                        respbuf[1]='C';
                        goto fail;
                    }
                    if(parsebyte(rxcmdbuf[offset+1],rxcmdbuf[offset+2],&nextbyte))
                    {
                        respbuf[1]='B';
                        goto fail;
                    }
                    Tab_Tx[txlen]=nextbyte;
                    txlen++;
                    offset+=3;
                }
                hcec.Init.InitiatorAddress=sourceaddr;
                if(HAL_CEC_Transmit(&hcec,destaddr,Tab_Tx,txlen,200))
                {
                    respbuf[0]='F';
                    if(hcec.ErrorCode&HAL_CEC_ERROR_TXACKE)
                    respbuf[1]='N';
                    else
                    respbuf[1]='X';
                    respbuf[2]='\r';
                    respbuf[3]='\n';
                    usb_txlen=4;;
                }
                else
                {
                    respbuf[0]='T';
                    respbuf[1]=toasc(sourceaddr);
                    respbuf[2]=toasc(destaddr);
                    offset=3;

                    if(txlen)
                    {
                        respbuf[offset]=':';
                        offset++;
                        offset+=toascstr(txlen,Tab_Tx,respbuf+offset);
                    }
                    respbuf[offset]='\r';
                    respbuf[offset+1]='\n';
                    usb_txlen=offset+2;
                }

                goto end;
                fail:

                respbuf[0]='F';
                respbuf[2]='\r';
                respbuf[3]='\n';
                usb_txlen=4;
                end:
                ;
            }

            rxcmdpos=0;
        }
        bufrpos=(bufrpos+1)&0x7f;
        if(usb_txlen)
        {
            CDC_Transmit_FS(respbuf,usb_txlen);
            usb_txlen=0;
        }
    }

}
/* USER CODE END 3 */
#endif

}


/** Pinout Configuration
 */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE()
    ;

}

/* USER CODE BEGIN 4 */
void transmitCecCommand(uint8_t *buffer, size_t len)
{
    getCecHandle()->Init.InitiatorAddress = buffer[0] >> CEC_INITIATOR_LSB_POS;
    uint8_t destAddr = buffer[0] & 0xf;

    if (HAL_CEC_Transmit(getCecHandle(), destAddr, buffer, len - 1, 200))
    {
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
 * @param hcec: CEC handle
 * @retval None
 */
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec)
{
    ErrorCode = hcec->ErrorCode;
    hcec->RxXferSize = 0;
    hcec->ErrorCode = HAL_CEC_ERROR_NONE;
    hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

#if 0
/**
 * @brief  Reset CEC reception buffer
 * @param  None
 * @retval None
 */
static void CEC_FlushRxBuffer(void)
{
    memset(Tab_Rx, 0, sizeof(Tab_Rx));
}
#endif

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
