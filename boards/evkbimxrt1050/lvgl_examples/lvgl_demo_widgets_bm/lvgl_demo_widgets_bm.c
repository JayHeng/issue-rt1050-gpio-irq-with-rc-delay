/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "lvgl_support.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "board.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 1 ms per tick. */
#ifndef LVGL_TICK_MS
#define LVGL_TICK_MS 1U
#endif

/* lv_task_handler is called every 5-tick. */
#ifndef LVGL_TASK_PERIOD_TICK
#define LVGL_TASK_PERIOD_TICK 5U
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile uint32_t s_tick        = 0U;
static volatile bool s_lvglTaskPending = false;

#define MAX_RECORD_BUFFER (0x8000)

uint32_t s_systickReloadVal = 0;
volatile uint32_t s_systickCurVal = 0;
volatile uint32_t s_systickLastVal = 0;
volatile uint32_t s_systickCurCount = 0;
volatile uint32_t s_systickLastCount = 0;
uint32_t s_systickDelta[MAX_RECORD_BUFFER];

volatile uint32_t s_inputNormalPinIrqCount = 0;
volatile uint32_t s_inputRcPinIrqCount   = 0;
volatile uint32_t s_outputPinEdgePreCount = 0;
volatile uint32_t s_outputPinEdgePostCount = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DEMO_SetupTick(void);
#if LV_USE_LOG
static void print_cb(const char *buf);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
AT_QUICKACCESS_SECTION_CODE(void BOARD_ReconfigFlexSpiRxBuffer(void));

/*
 * When PXP fetch images from FlexSPI flash, the default FlexSPI RX buffer
 * configuration does not meet the PXP bandwidth requirement. Reconfigure
 * here.
 */
void BOARD_ReconfigFlexSpiRxBuffer(void)
{
    uint32_t ahbcr;

    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR))
    {
        SCB_DisableICache();
    }

    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
    {
        SCB_DisableDCache();
    }

    ahbcr = FLEXSPI->AHBCR;

    /* Temporarily disable prefetching while changing the buffer settings */
    FLEXSPI->AHBCR = ahbcr & ~(FLEXSPI_AHBCR_CACHABLEEN_MASK | FLEXSPI_AHBCR_PREFETCHEN_MASK);

    /* Wait for FlexSPI idle to make sure no flash data transfer. */
    while ((FLEXSPI->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) == 0U)
    {
    }

    /* Allocate half of the prefetch buffer to the core */
    FLEXSPI->AHBRXBUFCR0[0] =
        FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID(0) | FLEXSPI_AHBRXBUFCR0_BUFSZ(0x40);

    /* Disable dedicate prefetch buffer for DMA. */
    FLEXSPI->AHBRXBUFCR0[1] =
        FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID(1) | FLEXSPI_AHBRXBUFCR0_BUFSZ(0x00);

    /* Disable dedicate prefetch buffer for DCP. */
    FLEXSPI->AHBRXBUFCR0[2] =
        FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID(2) | FLEXSPI_AHBRXBUFCR0_BUFSZ(0x00);

    /* Other half of the buffer for other masters incl. PXP */
    FLEXSPI->AHBRXBUFCR0[3] =
        FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID(3) | FLEXSPI_AHBRXBUFCR0_BUFSZ(0x40);

    FLEXSPI->AHBCR = ahbcr; /* Set AHBCR back to the original value */

    /* Enable I cache and D cache */
    SCB_EnableDCache();
    SCB_EnableICache();
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
     /* clear the interrupt status */
    if ((GPIO1->ISR & (1U << 26)) && (GPIO1->IMR & (1U << 26)))
    {
        s_systickCurVal = SysTick->VAL;
        s_systickCurCount = s_outputPinEdgePostCount;
        GPIO_PortClearInterruptFlags(GPIO1, 1U << 26);
        if (s_inputRcPinIrqCount < MAX_RECORD_BUFFER)
        {
            s_systickDelta[s_inputRcPinIrqCount] = (s_outputPinEdgePostCount - s_systickLastCount) * s_systickReloadVal + s_systickLastVal - s_systickCurVal;
            s_systickLastVal = s_systickCurVal;
            s_systickLastCount = s_systickCurCount;
        }
        s_inputRcPinIrqCount++;
        __DSB();
    }
    if ((GPIO1->ISR & (1U << 27)) && (GPIO1->IMR & (1U << 27)))
    {
        GPIO_PortClearInterruptFlags(GPIO1, 1U << 27);
        s_inputNormalPinIrqCount++;
        __DSB();
    }
}

static void delay_1s(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 33000000; ++i)
    {
        __NOP(); /* delay */
    }
}

void test_gpio_irq(void)
{
    gpio_pin_config_t out_config = { kGPIO_DigitalOutput, 1, kGPIO_NoIntmode };
    //pin that toggles every ms
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
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    /* Set the eLCDIF read_qos priority high, to make sure eLCDIF
     * can fetch data in time when PXP is used.
     */
    *((volatile uint32_t *)0x41044100) = 5;

    BOARD_ConfigMPU();
    BOARD_ReconfigFlexSpiRxBuffer();
    BOARD_InitPins();
    BOARD_InitI2C1Pins();
    BOARD_InitSemcPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("lvgl bare metal widgets demo\r\n");

    test_gpio_irq();
    delay_1s();
    s_inputRcPinIrqCount   = 0;
    s_inputNormalPinIrqCount   = 0;

    s_systickReloadVal = SystemCoreClock / (LVGL_TICK_MS * 1000U);
    s_systickLastVal = s_systickReloadVal;
    DEMO_SetupTick();

#if LV_USE_LOG
    lv_log_register_print_cb(print_cb);
#endif

    lv_port_pre_init();
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    lv_demo_widgets();

    for (;;)
    {
        while (!s_lvglTaskPending)
        {
        }
        s_lvglTaskPending = false;

        lv_task_handler();
    }
}

static void DEMO_SetupTick(void)
{
    if (0 != SysTick_Config(s_systickReloadVal))
    {
        PRINTF("Tick initialization failed\r\n");
        while (1)
            ;
    }
}

void SysTick_Handler(void)
{
    s_outputPinEdgePreCount++;
    GPIO_PortToggle(GPIO1, 1 << 20);
    GPIO_PortToggle(GPIO1, 1 << 21);
    s_outputPinEdgePostCount++;
    __DSB();

    s_tick++;
    lv_tick_inc(LVGL_TICK_MS);

    if ((s_tick % LVGL_TASK_PERIOD_TICK) == 0U)
    {
        s_lvglTaskPending = true;
    }
}

#if LV_USE_LOG
static void print_cb(const char *buf)
{
    PRINTF("\r%s\n", buf);
}
#endif
