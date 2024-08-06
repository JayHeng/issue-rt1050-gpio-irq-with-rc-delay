/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SW_GPIO         BOARD_USER_BUTTON_GPIO
#define EXAMPLE_SW_GPIO_PIN     BOARD_USER_BUTTON_GPIO_PIN
#define EXAMPLE_SW_IRQ          BOARD_USER_BUTTON_IRQ
#define EXAMPLE_GPIO_IRQHandler BOARD_USER_BUTTON_IRQ_HANDLER
#define EXAMPLE_SW_NAME         BOARD_USER_BUTTON_NAME

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
static void delay_1s(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define MAX_RECORD_BUFFER (0x3000)

uint32_t s_systickReloadVal = 0;
volatile uint32_t s_systickCurVal0 = 0;
volatile uint32_t s_systickLastVal0 = 0;
volatile uint32_t s_systickCurCount0 = 0;
volatile uint32_t s_systickLastCount0 = 0;
uint32_t s_systickDelta0[MAX_RECORD_BUFFER];

volatile uint32_t s_systickCurVal1 = 0;
volatile uint32_t s_systickLastVal1 = 0;
volatile uint32_t s_systickCurCount1 = 0;
volatile uint32_t s_systickLastCount1 = 0;
uint32_t s_systickDelta1[MAX_RECORD_BUFFER];

volatile uint32_t s_inputNormalPinIrqCount   = 0;
volatile uint32_t s_inputRcPinIrqCount   = 0;
volatile uint32_t s_outputPinEdgeCount = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

void SysTick_Handler(void)
{
    GPIO_PortToggle(GPIO1, 1 << 20);
    GPIO_PortToggle(GPIO1, 1 << 21);
    s_outputPinEdgeCount++;
    __DSB();
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
     /* clear the interrupt status */
    if ((GPIO1->ISR & (1U << 26)) && (GPIO1->IMR & (1U << 26)))
    {
        s_systickCurVal0 = SysTick->VAL;
        s_systickCurCount0 = s_outputPinEdgeCount;
        GPIO_PortClearInterruptFlags(GPIO1, 1U << 26);
        if (s_inputRcPinIrqCount < MAX_RECORD_BUFFER)
        {
            s_systickDelta0[s_inputRcPinIrqCount] = (s_outputPinEdgeCount - s_systickLastCount0) * s_systickReloadVal + s_systickLastVal0 - s_systickCurVal0;
            s_systickLastVal0 = s_systickCurVal0;
            s_systickLastCount0 = s_systickCurCount0;
        }
        s_inputRcPinIrqCount++;
        __DSB();
    }
    if ((GPIO1->ISR & (1U << 27)) && (GPIO1->IMR & (1U << 27)))
    {
        s_systickCurVal1 = SysTick->VAL;
        s_systickCurCount1 = s_outputPinEdgeCount;
        GPIO_PortClearInterruptFlags(GPIO1, 1U << 27);
        if (s_inputNormalPinIrqCount < MAX_RECORD_BUFFER)
        {
            s_systickDelta1[s_inputNormalPinIrqCount] = (s_outputPinEdgeCount - s_systickLastCount1) * s_systickReloadVal + s_systickLastVal1 - s_systickCurVal1;
            s_systickLastVal1 = s_systickCurVal1;
            s_systickLastCount1 = s_systickCurCount1;
        }
        s_inputNormalPinIrqCount++;
        __DSB();
    }
}

void test_gpio_irq(void)
{
    gpio_pin_config_t out_config = { kGPIO_DigitalOutput, 1, kGPIO_NoIntmode };
    //pin that toggles every ms
    // RC out - systick drive
	{
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20, 0);
		GPIO_PinInit(GPIO1, 20, &out_config);
		GPIO_PinWrite(GPIO1, 20, 0U);
	}
	{
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_GPIO1_IO21, 0);
		GPIO_PinInit(GPIO1, 21, &out_config);
		GPIO_PinWrite(GPIO1, 21, 0U);
	}
    delay_1s();

	//init receive pin
    // RC in - irq pin
	{
		gpio_pin_config_t config = { kGPIO_DigitalInput, 1, kGPIO_NoIntmode };
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26, 1);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26, 0x011030U);
		GPIO_PinInit(GPIO1, 26, &config);
		GPIO_SetPinInterruptConfig(GPIO1, 26, kGPIO_IntRisingOrFallingEdge);
		EnableIRQ(GPIO1_Combined_16_31_IRQn);
		GPIO_PortEnableInterrupts(GPIO1, 1U << 26);
	}
    // normal in
	{
		gpio_pin_config_t config = { kGPIO_DigitalInput, 1, kGPIO_NoIntmode };
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_GPIO1_IO27, 1);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_11_GPIO1_IO27, 0x011030U);
		GPIO_PinInit(GPIO1, 27, &config);
		GPIO_SetPinInterruptConfig(GPIO1, 27, kGPIO_IntRisingOrFallingEdge);
		EnableIRQ(GPIO1_Combined_16_31_IRQn);
		GPIO_PortEnableInterrupts(GPIO1, 1U << 27);
	}

    /* Update the core clock */
    SystemCoreClockUpdate();
    
    s_inputRcPinIrqCount   = 0;
    s_inputNormalPinIrqCount   = 0;

    /* Set systick reload value to generate 1ms interrupt */
    s_systickReloadVal = SystemCoreClock / 1000U;
    s_systickLastVal0 = s_systickReloadVal;
    s_systickLastVal1 = s_systickReloadVal;
    if (SysTick_Config(s_systickReloadVal))
    {
        while (1)
        {
        }
    }
    
    while(1)
    {
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* hardware initialiize, include IOMUX, Uart debug initialize */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    PRINTF("GPIO Driver example.\r\n");
    
    test_gpio_irq();
}

static void delay_1s(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 33000000; ++i)
    {
        __NOP(); /* delay */
    }
}

