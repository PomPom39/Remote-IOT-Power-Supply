/*
 * main_mahesh.c
 *
 *  Created on: 20-Sep-2020
 *      Author: mahesh.rd
 */


/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Driver Includes */
#include "driverlib.h"
#include "gpio.h"
#include "spi.h"


/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[32];

#define             SPI_MSG_LENGTH              30


uint32_t uiISRCount = 0;
uint8_t ucMasterTxMsg[SPI_MSG_LENGTH] = "SPIMasterTest123";
uint8_t ucMasterRxMsg[SPI_MSG_LENGTH] = {0};
uint8_t ucSlaveTxMsg[SPI_MSG_LENGTH] = "SPISlaveTest123";
uint8_t ucSlaveRxMsg[SPI_MSG_LENGTH] = {0};


TaskHandle_t Task1Handler = NULL;


const eUSCI_SPI_MasterConfig spiConfigMaster = {
    EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              /* Clock Source for SPI */
    12000000,                                                   /* Clock Source Frequency */
    1000000,                                                   /* SPI Clock Speed = 12MHz */
    EUSCI_SPI_MSB_FIRST,                                        /* Order */
    EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,      /* Clock Polarity (CPHA = 1) */
    EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,                    /* Clock IDLE High (CPOL=1) */
    EUSCI_SPI_3PIN                                              /* SPI Mode */
};

const eUSCI_SPI_SlaveConfig spiConfigSlave = {
    EUSCI_SPI_MSB_FIRST,                                        /* Order */
    EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,      /* Clock Polarity (CPHA = 1) */
    EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,                    /* Clock IDLE High (CPOL=1) */
    EUSCI_SPI_3PIN                                              /* SPI Mode */
};




static void prvConfigureClocks( void );
void vInitialSetupTask (void *pvParameter);
void vGPIOInterruptTask (void *pvParamters);

void main_RTOS( void )
{
    MAP_WDT_A_holdTimer();
    /* This sets the clock to its maximum, ie 48MHz */
    prvConfigureClocks();

    xTaskCreate(vInitialSetupTask, "SetupTask", 200, (void *)0, tskIDLE_PRIORITY, Task1Handler);
//    xTaskCreate(vGPIOInterruptTask, "GPIOIntTask", 200, (void *)0, tskIDLE_PRIORITY, Task1Handler);
    vTaskStartScheduler();



}
static void prvConfigureClocks( void )
{
    /* Set Flash wait state for high clock frequency.  Refer to data-sheet for
    more details. */
    FlashCtl_setWaitState( FLASH_BANK0, 2 );
    FlashCtl_setWaitState( FLASH_BANK1, 2 );

    /* The full demo configures the clocks for maximum frequency, whereas the
    blink demo uses a slower clock as it also uses low power features.  Maximum
    frequency also needs more voltage.

    From the data-sheet:  For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes, the maximum
    CPU operating frequency is 48 MHz and maximum input clock frequency for
    peripherals is 24 MHz. */
    PCM_setCoreVoltageLevel( PCM_VCORE1 );
    CS_setDCOCenteredFrequency( CS_DCO_FREQUENCY_3 );
    CS_initClockSignal( CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
}
/*-----------------------------------------------------------*/
#if( configCREATE_ETH_SPI_APP == 1 )

    void vApplicationTickHook( void )
    {
        /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */


    }

#endif

void vGPIOInterruptTask (void *pvParamters) {
    MAP_WDT_A_holdTimer();

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableMaster();
    while (1) {

    }
}



void vInitialSetupTask (void *pvParameter) {
        volatile uint32_t uii = 0;
        volatile uint32_t uiRet = 0;

        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        vTaskDelay(pdMS_TO_TICKS( 1000 ));
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        vTaskDelay(pdMS_TO_TICKS( 1000 ));
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

        /* Configure CLK(GPIO1_5), MOSI(GPIO1_6)  for SPI0(Master) (EUSCI_B0) */
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN5 | GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Configure MOSI(GPIO1_7) for SPI0 (EUSCI_B0) */
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Configure CLK(GPIO3_5), MOSI(GPIO3_6), MISO(GPIO3_7)  for SPI2(Slave) (EUSCI_B2) */
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Initialize B_SPI0 as Master */
        uiRet = SPI_initMaster(EUSCI_B0_BASE, &spiConfigMaster);

        /* Initialize B_SPI2 as Slave */
        SPI_initSlave(EUSCI_B2_BASE, &spiConfigSlave);

        /* Enable SPI0 and SPI2 Modules*/
        SPI_enableModule(EUSCI_B0_BASE);
        SPI_enableModule(EUSCI_B2_BASE);

        /* Enabling DMA */
        DMA_enableModule();
        DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);

        /* Assign Channels for SPI0 and SPI2, TX and RX registers */
        DMA_assignChannel(DMA_CH0_EUSCIB0TX0);
        DMA_assignChannel(DMA_CH1_EUSCIB0RX0);
        DMA_assignChannel(DMA_CH4_EUSCIB2TX0);
        DMA_assignChannel(DMA_CH5_EUSCIB2RX0);

        /* Setup TX Characteristics and buffer for DMA for Master*/
        DMA_setChannelControl(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
        DMA_setChannelTransfer(DMA_CH0_EUSCIB0TX0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   ucMasterTxMsg,
                                   (void *)MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B0_BASE),
                                   SPI_MSG_LENGTH);

        /* Setup RX Characteristics and buffer for DMA for Master*/
        DMA_setChannelControl(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);
        DMA_setChannelTransfer(DMA_CH1_EUSCIB0RX0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)MAP_SPI_getReceiveBufferAddressForDMA(EUSCI_B0_BASE),
                                   ucMasterRxMsg,
                                   SPI_MSG_LENGTH);

        /* Setup TX Characteristics and buffer for DMA for Slave*/
        DMA_setChannelControl(DMA_CH4_EUSCIB2TX0 | UDMA_PRI_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
        DMA_setChannelTransfer(DMA_CH4_EUSCIB2TX0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   ucSlaveTxMsg,
                                   (void *)MAP_SPI_getTransmitBufferAddressForDMA(EUSCI_B2_BASE),
                                   SPI_MSG_LENGTH);

        /* Setup RX Characteristics and buffer for DMA for Slave*/
        DMA_setChannelControl(DMA_CH5_EUSCIB2RX0 | UDMA_PRI_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);
        DMA_setChannelTransfer(DMA_CH5_EUSCIB2RX0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)MAP_SPI_getReceiveBufferAddressForDMA(EUSCI_B2_BASE),
                                   ucSlaveRxMsg,
                                   SPI_MSG_LENGTH);

        /* Enabling DMA Interrupt for Channel 1*/
        DMA_assignInterrupt(DMA_INT1, 1);
        DMA_clearInterruptFlag(1);

        /* Assigning/Enabling Interrupts */
        MAP_Interrupt_enableInterrupt(INT_DMA_INT1);
        DMA_enableInterrupt(INT_DMA_INT1);
//        MAP_Interrupt_enableMaster();

        /* Start DMA for SPI2 Rx */
        DMA_enableChannel(5);

        /* Start DMA for SPI2 Tx */
        DMA_enableChannel(4);

        /* Wait 50 cycles for Master to catch up to slave */
        for (uii = 0; uii < 50; uii ++);

        /* Start DMA for SPi0 Rx */
        DMA_enableChannel(1);

        /* Start DMA for SPi0 Tx */
        DMA_enableChannel(0);

        /* Polling to see if the master receive is finished */
           while (1)
           {
               if (uiISRCount > 0)
               {
                   __no_operation();
               }
           }
}

void DMA_INT1_IRQHandler(void) {
    uiISRCount++;

    /* Clear Interrupt Flag for Ch 0 and 1 DMA */
    MAP_DMA_clearInterruptFlag(0);
    MAP_DMA_clearInterruptFlag(1);

    /* Disable the interrupt to allow execution */
    MAP_Interrupt_disableInterrupt(INT_DMA_INT1);
    MAP_DMA_disableInterrupt(INT_DMA_INT1);

}

void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
}
