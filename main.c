/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Bytes of data to be transmitted */
#define DATA_LENGTH    10

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Flag to set when Rx index equals the total data transmitted */
volatile uint32_t flag = 0;

/* Array for storing the data */
char uart_buf[DATA_LENGTH] = {0};

/* Array for storing the data to be transmitted */
char tx_buf[DATA_LENGTH]= {'u','a','r','t','_','t','x','_','r','x'};

/* Array for storing the received data */
char rx_buf[DATA_LENGTH]={0};

/* UART context structure. */
cy_stc_scb_uart_context_t UART1_context;
cy_stc_scb_uart_context_t UART2_context;

/** UART status codes */
cy_en_scb_uart_status_t errorStatus;

/* Populate interrupt configuration structure */
cy_stc_sysint_t UART_SCB_IRQ_cfg =
{
    .intrSrc      = UART1_IRQ,
    .intrPriority = 2u,
};

/* Populate interrupt configuration structure */
cy_stc_sysint_t UART_SCB_IRQ_cfg2 =
{
    .intrSrc      = UART2_IRQ,
    .intrPriority = 2u,
};

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: cb_scb_uart1_handle_events
********************************************************************************
* Summary:
* This function handles callback status of UART1 RX
*
* Parameters:
*  \param event - event mode set by the caller
*
* Return:
*  int
*
*******************************************************************************/
void cb_scb_uart1_handle_events (uint32_t event)
{
    if(event == CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        /* UART 2 receive enabled with RX trigger size */
        errorStatus = Cy_SCB_UART_Receive(UART2_HW, (void *)rx_buf, (uint32_t)sizeof(rx_buf), &UART2_context);
        if(errorStatus != CY_SCB_UART_SUCCESS)
        {
            CY_ASSERT(0);
        }

        /* Start transmitting data received from UART1 to UART2 */
        Cy_SCB_UART_PutString(UART1_HW, (const char*)uart_buf);
        while(!Cy_SCB_IsTxComplete(UART1_HW));
    }
}

/*******************************************************************************
* Function Name: cb_scb_uart2_handle_events
********************************************************************************
* Summary:
* This function handles callback status of UART2 RX
*
* Parameters:
*  \param event - event mode set by the caller
*
* Return:
*  int
*
*******************************************************************************/
void cb_scb_uart2_handle_events (uint32_t event)
{
    if(event == CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        flag = 1;
    }
}

/*******************************************************************************
* Function Name: Isr_uart1_fifo
********************************************************************************
* Summary:
* This function is registered to be called when UART1 interrupt occurs.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
void Isr_uart1_fifo(void)
{
    Cy_SCB_UART_Interrupt(UART1_HW, &UART1_context);
}

/*******************************************************************************
* Function Name: Isr_uart2_fifo
********************************************************************************
* Summary:
* This function is registered to be called when UART2 interrupt occurs.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
void Isr_uart2_fifo(void)
{
    Cy_SCB_UART_Interrupt(UART2_HW, &UART2_context);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It performs the following tasks:
* 1. Initial setup of device.
* 2. Enable the UART peripheral
* 3. Send data from UART2 to UART1 for the first time
* 4. After data transfer complete, UART1 send back same data to UART2
* 5. Check if the data transmitted is equal to the data received.
*    LED is switched ON in case of successful reception.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;
    cy_en_scb_uart_status_t init_status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the UART1 */
    init_status = Cy_SCB_UART_Init(UART1_HW, &UART1_config, &UART1_context);
    if(init_status!=CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* This configures which bits from the RX interrupt
     * request register can trigger an interrupt event. */
    Cy_SCB_SetRxInterruptMask(UART1_HW, CY_SCB_UART_RX_TRIGGER);

    /* Registers a callback function that notifies that
    *  uart_callback_events occurred in the Cy_SCB_UART_Interrupt.*/
    Cy_SCB_UART_RegisterCallback(UART1_HW, (cy_cb_scb_uart_handle_events_t)cb_scb_uart1_handle_events, &UART1_context);

    /* Configuring priority and enabling NVIC IRQ
     * for the defined Service Request line number */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, Isr_uart1_fifo);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    /* Initialize the UART2 */
    init_status = Cy_SCB_UART_Init(UART2_HW, &UART2_config, &UART2_context);
    if(init_status!=CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* This configures which bits from the RX interrupt
         * request register can trigger an interrupt event. */
    Cy_SCB_SetRxInterruptMask(UART2_HW, CY_SCB_UART_RX_TRIGGER);

    /* Registers a callback function that notifies that
    *  uart_callback_events occurred in the Cy_SCB_UART_Interrupt.*/
    Cy_SCB_UART_RegisterCallback(UART2_HW, (cy_cb_scb_uart_handle_events_t)cb_scb_uart2_handle_events, &UART2_context);

    /* Configuring priority and enabling NVIC IRQ
     * for the defined Service Request line number */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg2, Isr_uart2_fifo);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg2.intrSrc);

    /* Enable the UART peripheral*/
    Cy_SCB_UART_Enable(UART1_HW);
    Cy_SCB_UART_Enable(UART2_HW);

    /* UART 1 receive enabled with RX trigger size */
    errorStatus = Cy_SCB_UART_Receive(UART1_HW, (void *)uart_buf, (uint32_t)sizeof(uart_buf), &UART1_context);
    if(errorStatus != CY_SCB_UART_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start transmitting data from UART2 to UART1 */
    Cy_SCB_UART_PutString(UART2_HW, (const char*)tx_buf);

    /* Wait of transaction to complete */
    while(!Cy_SCB_IsTxComplete(UART2_HW));

    Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {

        /* Infinite loop */
        if (flag == 1)
        {
            /* Check if every received data match with the transmitted data */
            for (int tmp = 0; tmp < DATA_LENGTH; tmp++)
            {
                /* If reception fails stays in an infinite while loop and switch off the LED */
                if (tx_buf[tmp] != rx_buf[tmp])
                {
                    Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
                    break;
                }
                /* If reception is successful turn on the LED */
                else
                {
                    Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
                }
            }

            /* Reset the flag to zero */
            flag = 0;
        }
    }
}

/* [] END OF FILE */
