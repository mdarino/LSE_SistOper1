/* Copyright 2015, Pablo Ridolfi
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*************************************************************************//**

  @file     ej_clase2.c

  @brief    EJERCICIO 4.1 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/

/**
	Ejercicio​ Clase 2
	**/


/** \addtogroup rtos_blink FreeRTOS ej_clase2
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../../rtos_i_ejercicios/ej1_1/inc/main.h"

#include "../../../rtos_i_ejercicios/ej1_1/inc/FreeRTOSConfig.h"
#include "ciaaIO.h"
#include "ciaaUART.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/*==================[macros and definitions]=================================*/
//define for the queue
    #define QUEUE1_SIZE 5
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/
//xSemaphoreHandle xSemaphore;
QueueHandle_t xQueue1;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();

    ciaaIOInit();
    ciaaUARTInit();

    ciaaWriteOutput(0, 0);


    //BOTON 1

    Chip_PININT_Init(LPC_GPIO_PIN_INT);
    //Set GPIO0[4] (SW1) -> GPIO0_IRQHandler
    Chip_SCU_GPIOIntPinSel(0,0,4);
    //falling edge IRQ 
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT,PININTCH0);
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT,PININTCH0);

    //habilitar la IRQ
    NVIC_EnableIRQ(PIN_INT0_IRQn);

    //BOTON 2

    //Set GPIO0[4] (SW1) -> GPIO0_IRQHandler
    Chip_SCU_GPIOIntPinSel(1,1,9);
    //falling edge IRQ 
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT,PININTCH1);
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT,PININTCH1);

    //habilitar la IRQ
    NVIC_EnableIRQ(PIN_INT1_IRQn);




}

void GPIO0_IRQHandler()
{

   portBASE_TYPE xSwitchRequired1;
   
   if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT)&PININTCH0)
    { 
      Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH0);   
      //xSemaphoreGiveFromISR(xSemaphore,&xSwitchRequired1);
      uint8_t aux=0;
      xQueueSendFromISR( xQueue1, ( void * ) &aux, &xSwitchRequired1);
      

      //NO USAR - xSemaphoreGive(xSemaphore);
    }
   portEND_SWITCHING_ISR(xSwitchRequired1);


}



void GPIO1_IRQHandler()
{

   portBASE_TYPE xSwitchRequired1;
   
   if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT)&PININTCH1)
    { 
      Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH1);   
      uint8_t aux=1;
      xQueueSendFromISR( xQueue1, ( void * ) &aux, &xSwitchRequired1);
      //xSemaphoreGiveFromISR(xSemaphore,&xSwitchRequired1);
      //NO USAR - xSemaphoreGive(xSemaphore);
    }
   portEND_SWITCHING_ISR(xSwitchRequired1);

}


static void task(void * a)
{
	uint8_t buf;
  while (1) {
  xQueueReceive(xQueue1,&buf,portMAX_DELAY);
    if(buf==0)
    {
      dbgPrint("BOTON IRQ 0 \n");      
    }
    else
    {
      dbgPrint("BOTON IRQ 1 \n");
    }
  

	}
}

//vApplicationTickHook

/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware();

  dbgPrint("TEST");
	// Create a semaphore
   //vSemaphoreCreateBinary(xSemaphore);
   //xSemaphoreTake(xSemaphore,portMAX_DELAY);

  //create the queue
   xQueue1 = xQueueCreate( QUEUE1_SIZE, sizeof(uint8_t) );
  
   xTaskCreate(task, (const char *)"task", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
