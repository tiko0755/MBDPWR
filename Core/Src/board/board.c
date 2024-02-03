/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : board.c
* Author             : Tiko Zhong
* Date First Issued  : 11/18/2021
* Description        : 
*                      
********************************************************************************
* History:
* Apr22,2021: V0.2
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "main.h"
#include "app_timer.h"
#include "thread_delay.h"
#include "gpioDecal.h"

#include "stm_flash.h"
#include "stm_flash_cmd.h"
#include "disk.h"
#include "led_flash.h"
#include "vRegulator.h"

#include "task.h"
#include "user_log.h"

#include "crc16.h"

#define NOUSED_PIN_INDX 255

/* import handle from main.c variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const char ABOUT[] = {"MB2160X2-1.0.0"};
const char COMMON_HELP[] = {
    "Commands:"
    "\n help()"
    "\n about()"
    "\n restart()"
    "\n reg.write(addr,val)"
    "\n reg.read(addr)"
    "\n baud.set(bHost,bBus)"
    "\n baud.get()"
    "\n"
};
char g_addrPre[4] = {0};    //addr precode
u8 g_initalDone = 0;
u32 g_errorCode;// = 0;
u32 g_tCounter = 0;
/**********************************************
*  PINs Define
**********************************************/
const PIN_T RUNNING = {SWD_LED_GPIO_Port, SWD_LED_Pin};

/**********************************************
*  static Devices
**********************************************/
// =============================================
#define RX_POOL_LEN    (MAX_CMD_LEN)
#define TX_POOL_LEN    (MAX_CMD_LEN)
#define RX_BUF_LEN      (72)

// uart device
static u8 uartRxPool[RX_POOL_LEN] = {0};
static u8 uartRxBuf[2*RX_BUF_LEN] = {0};
static u8 uartTxPool[TX_POOL_LEN] = {0};
UartDev_t console;

// rs485 device
static u8 rs485RxPool[RX_POOL_LEN] = {0};
static u8 rs485RxBuf[2*RX_BUF_LEN] = {0};
static u8 rs485TxPool[TX_POOL_LEN] = {0};
static s8 rs485AfterSend_1(UART_HandleTypeDef *huart);
static s8 rs485BeforeSend_1(void);
const PIN_T DE = {DE_GPIO_Port, DE_Pin};
const PIN_T DET = {DET_GPIO_Port, DET_Pin};
UartDev_t rs485;

// app timer 
appTmrDev_t tmr[APP_TIMER_COUNT] = {0};
u8 gTmrIndx = 0;

// commander comsumer
cmdConsumerDev_t cmdConsumer;
cmdConsumerDev_t cmdConsumer_bus;

static u8 brdCmdU8(void* d, u8* CMDu8, u8 len, void (*xprint)(const char* FORMAT_ORG, ...));
static void forwardToBus(u8* BUFF, u16 len);
static void forwardToConsole(u8* BUFF, u16 len);

dcvDev_t dpower;

/* Private function prototypes -----------------------------------------------*/
// after GPIO initial, excute this function to enable
void boardPreInit(void){
    u8 trmIdx = 0;
    // setup app timers
    for(trmIdx=0;trmIdx<APP_TIMER_COUNT;trmIdx++){
        setup_appTmr(&tmr[trmIdx], taskPolling);
    }
    thread_delay_init(&tmr[gTmrIndx++]);
    
    stmFlsh_initial(ADDR_FLASH_PAGE_31, 1);
    disk_setup(stmFlsh_read, stmFlsh_write);
    configRead();
}

void boardInit(void){
    //read board addr
    setupUartDev(&console, &huart2, &tmr[gTmrIndx++], uartTxPool, TX_POOL_LEN, uartRxPool, RX_POOL_LEN, uartRxBuf, RX_BUF_LEN, 4);
    memset(g_addrPre,0,4);
    strFormat(g_addrPre, 4, "%d.", conf.brdAddr);

    logInitial(printS);
    log("%sabout(\"%s\")", g_addrPre, ABOUT);

    // application initial
    log_raw("setup led_flash...");
    led_flash_init(&tmr[gTmrIndx++], &RUNNING, 100);     // now, it can run itself
    log_raw("ok\r\n");
    
    log_raw("setup rs485...");
    setupUartDev_2(
        &rs485,             //        Rs485Dev_t *pDev, 
        &huart1,            //        UART_HandleTypeDef* huart,
        &tmr[gTmrIndx++],   //        ppTmrDev_t* tObj,    
        rs485TxPool, TX_POOL_LEN,       //        u8* txPool, u16 txPoolLen,
        rs485RxPool, RX_POOL_LEN,       //        u8* rxPool,    u16    rxPoolLen,
        rs485RxBuf, RX_BUF_LEN, 4,      //        u8* rxDoubleBuff,    u16 rxBufLen,	u16 rxPollingTim,
        rs485BeforeSend_1,  //        s8 (*beforeSend)(void),
        rs485AfterSend_1    //        s8 (*afterSend)(UART_HandleTypeDef *huart)
    );
    log_raw("ok\r\n");
    
    log_raw("setup cmdConsumer...");
    setup_cmdConsumer(&cmdConsumer, 
        &console.rsrc.rxRB,             // command line in a ringbuffer
        fetchLineFromRingBufferU8,      // fetchLine method  
        print,                          // print out 
        forwardToBus,
        &tmr[gTmrIndx++],
        8                              // unit in ms, polling rb each interval
    );
    cmdConsumer.append(&cmdConsumer.rsrc, NULL, brdCmdU8);
    cmdConsumer.append(&cmdConsumer.rsrc, NULL, stmFlsh_Cmd);
    log_raw("ok\r\n");
    
    log_raw("setup cmdConsumer_bus...");
    setup_cmdConsumer(&cmdConsumer_bus, 
        &rs485.rsrc.rxRB,          // command line in a ringbuffer
        fetchFrameFromRingBuffer,               // fetchLine method  
        print485,                               // print out 
        forwardToConsole,
        &tmr[gTmrIndx++],
        8                     // unit in ms, polling rb each interval
    );
    cmdConsumer_bus.append(&cmdConsumer_bus.rsrc, NULL, brdCmdU8);
    cmdConsumer_bus.append(&cmdConsumer_bus.rsrc, NULL, stmFlsh_Cmd);
    log_raw("ok\r\n");
    
    // get ready, start to work
    console.StartRcv(&console.rsrc);
    rs485.StartRcv(&rs485.rsrc);
    HAL_GPIO_WritePin(DE.GPIOx, DE.GPIO_Pin, GPIO_PIN_RESET);

    dcvDevSetup(&dpower,
        "dc",               //	const char* NAME,
        &htim3,             //	TIM_HandleTypeDef* vTimHandle,
        TIM_CHANNEL_2,      //	u32 vTimChannel,
        &hadc1,             //	ADC_HandleTypeDef* vAdcHandle,
        ADC_CHANNEL_5,      //	u32 vAdcChannel,
        &htim3,             //    TIM_HandleTypeDef* iTimHandle,
        TIM_CHANNEL_1,      //	u32 iTimChannel,
        &hadc1,             //    ADC_HandleTypeDef* iAdcHandle,
        ADC_CHANNEL_4,      //	u32 iAdcChannel,
        USR_DPWR_BASE,      //	u16 ioBase,
        usrWrite,           //	s8 (*ioWrite)(u16 addr, const u8 *pDat, u16 nBytes),
        usrRead             //	s8 (*ioRead)(u16 addr, u8 *pDat, u16 nBytes)
    );

    g_initalDone = 1;

    log("%d timers have been used", gTmrIndx);
    log("initial complete, type \"help\" for more");
}

void printS(const char* STRING){
    console.Send(&console.rsrc, (const u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
    va_list ap;
    char buf[MAX_CMD_LEN] = {0};
    s16 bytes;
    //take string
    va_start(ap, FORMAT_ORG);
    bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
    va_end(ap);
    //send out
    if(bytes>0)    console.Send(&console.rsrc, (u8*)buf, bytes);
}

void printS485(const char* STRING){
    rs485.TxSendFrame(&rs485.rsrc, (const u8*)STRING, strlen(STRING));
}

void print485(const char* FORMAT_ORG, ...){
    va_list ap;
    char buf[MAX_CMD_LEN] = {0};
    s16 bytes;
    //take string
    va_start(ap, FORMAT_ORG);
    bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
    va_end(ap);
    //send out
    if(bytes>0)    rs485.TxSendFrame(&rs485.rsrc, (u8*)buf, bytes);
}

static void forwardToBus(u8* BUFF, u16 len){
    rs485.TxSendFrame(&rs485.rsrc, BUFF, len);
}

static void forwardToConsole(u8* BUFF, u16 len){
    console.Send(&console.rsrc, BUFF, len);
}

static u8 brdCmdU8(void* d, u8* CMDu8, u8 len, void (*xprint)(const char* FORMAT_ORG, ...)){
    return(brdCmd((const char*)CMDu8, xprint));
}

u8 brdCmd(const char* CMD, void (*xprint)(const char* FORMAT_ORG, ...)){
    s32 i,j;
    char uiCmd[24] = {0};
    
    if(sscanf(CMD, "%d.", &i)==1){
        j = strlen(g_addrPre);
        if(strncmp(CMD, g_addrPre, j)==0){
            CMD = &CMD[j];
        }
        else{
            return 0;
        }
    }
    
    // common
    if(strncmp(CMD, "about", strlen("about")) == 0){
        xprint("+ok@%d.about(\"%s\")\r\n", conf.brdAddr, ABOUT);
        led_flash_answer(50, 30);
        return 1;
    }
    else if(strncmp(CMD, "rs485.rb", strlen("rs485.rb")) == 0){
        xprint("+ok@%d.rs485(%d)\r\n", conf.brdAddr, RingBuffer_GetCount(&rs485.rsrc.rxRB));
        return 1;
    }
    else if(sscanf(CMD, "rs485.send %s", uiCmd)==1){
        printS485(uiCmd);
    }
    else if(strncmp(CMD, "help", strlen("help")) == 0){
        xprint("%s +ok@%d.help()\r\n", COMMON_HELP, conf.brdAddr);
        return 1;
    }
    else if(strncmp(CMD, "restart ", strlen("restart ")) == 0){
        HAL_NVIC_SystemReset();
        return 1;
    }
    else if(sscanf(CMD, "address %d", &i)==1){
        conf.brdAddr = i&0xff;
        log("<%s conf.brdAddr:%d >", __func__, conf.brdAddr);
//        configWrite();
        memset(g_addrPre,0,4);
        strFormat(g_addrPre, 4, "%d.", conf.brdAddr);
        log("<%s conf.brdAddr:%d >", __func__, conf.brdAddr);
        
        xprint("+ok@%d.address()\r\n", conf.brdAddr);
        return 1;
    }
    else if(sscanf(CMD, "reg.write %d 0x%x ", &i, &j)==2){
        if(ioWriteReg(i,j) == 0)    xprint("+ok@%d.reg.write(%d,0x%08x)\r\n", conf.brdAddr, i, j);
        else xprint("+err@%d.reg.write(%d,0x%08x)\r\n", conf.brdAddr, i, j);
        return 1;
    }
    else if(sscanf(CMD, "reg.write %d %d ", &i, &j)==2){
        if(ioWriteReg(i,j) == 0)    xprint("+ok@%d.reg.write(%d,%d)\r\n", conf.brdAddr, i, j);
        else xprint("+err@%d.reg.write(%d,%d)\r\n", conf.brdAddr, i, j);
        return 1;
    }

    else if(sscanf(CMD, "reg.read %d ", &i)==1){
        if((ioReadReg!=NULL)&&(ioReadReg(i,&j) == 0))
            xprint("+ok@%d.reg.read(%d,%d,0x%08x)\r\n", conf.brdAddr, i, j, j);
        else xprint("+err@%d.reg.read(%d)\r\n", conf.brdAddr, i);
        return 1;
    }
    return 0;
}

static s8 rs485BeforeSend_1(void){
    if(g_initalDone == 0)    return 0;
    if(HAL_GPIO_ReadPin(DET.GPIOx, DET.GPIO_Pin)==GPIO_PIN_SET){
        return -1;
    }
    HAL_GPIO_WritePin(DE.GPIOx, DE.GPIO_Pin, GPIO_PIN_SET);
    while(1){
        if(HAL_GPIO_ReadPin(DET.GPIOx, DET.GPIO_Pin)==GPIO_PIN_SET){
            break;
        }
    }
    return 0;
}

static s8 rs485AfterSend_1(UART_HandleTypeDef *huart){
    if(g_initalDone == 0)    return 0;
    if(huart->Instance == rs485.rsrc.huart->Instance){
        HAL_GPIO_WritePin(DE.GPIOx, DE.GPIO_Pin, GPIO_PIN_RESET);
        rs485.rsrc.flag |= BIT(0);
    }
    return 0;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
	if(g_initalDone == 0)	return;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){
	if(g_initalDone == 0)	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(g_initalDone == 0){  return; }
//	if(htim->Instance == htim16.Instance){
//        g_tCounter++;
//	}
//    else if(htim->Instance == htim3.Instance){
//    }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle){}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(g_initalDone==0)    return;
    rs485.rsrc.afterSend(huart);
    if(huart->Instance == console.rsrc.huart->Instance){
        console.rsrc.flag |= BIT(0);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
