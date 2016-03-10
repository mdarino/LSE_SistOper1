/*Copyright 2015, Pablo Ridolfi 
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

  @file     ej5_2.c

  @brief    EJERCICIO 5.2 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/


/**

 EJERCICIO 5.2    (Spanish)

Caso   de   uso​
:   Situación   inversa   al   anterior   el   manejo   
del   evento   se   produce   en   la   interrupción   de  
algún periférico de salida. Por ejemplo un 
"transmitter empty" de una UART. 
 
Ejercicio​
:   Ídem   anterior,   pero   usando   una   cola,   
de   modo   de   poder   hacer   el   destello   de   tiempo  
variable. 

 **/




/** \addtogroup rtos FreeRTOS Ejer5.2
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../../rtos_i_ejercicios/ej1_1/inc/main.h"

#include "../../../rtos_i_ejercicios/ej1_1/inc/FreeRTOSConfig.h"
#include "ciaaIO.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*==================[macros and definitions]=================================*/
    //BUTTONS STATES:
    #define PRESS   1  ///Buttons is press
    #define RELEASE 0  ///Buttons is relaese

    #define UPDATE_BUTTON_TIME 5 ///Time in mseg, who often is checked the button's state    
    #define TIME_NOT_REBOUND   20 ///Delay to avoid the rebound on the button(in mseg)

typedef struct  STR_Button
{
    uint8_t     state;
    uint32_t    time;
    uint8_t     number;
}button_t;

  //define for the queue
    #define QUEUE1_SIZE 5
    #define READY       1
    #define NOT_READY   0
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *  @return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/
xSemaphoreHandle xSemaphore;
QueueHandle_t xQueue1;
/*==================[external data definition]===============================*/

//Global Variables
uint16_t  buttonTime=0;  ///Save the button time press in os ticks



/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();

    ciaaIOInit();

    ciaaWriteOutput(0, 0);

    //
    Chip_PININT_Init(LPC_GPIO_PIN_INT);
    //Set GPIO0[4] (SW1) -> GPIO0_IRQHandler
    Chip_SCU_GPIOIntPinSel(0,0,4);
    //falling edge IRQ 
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT,PININTCH0);
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT,PININTCH0);

    //habilitar la IRQ
    NVIC_EnableIRQ(PIN_INT0_IRQn);

}

void GPIO0_IRQHandler()
{

   portBASE_TYPE xSwitchRequired1;
   
   if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT)&PININTCH0)
    { 
      Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH0);   
      xSemaphoreGiveFromISR(xSemaphore,&xSwitchRequired1);
      //NO USAR - xSemaphoreGive(xSemaphore);
    }
   portEND_SWITCHING_ISR(xSwitchRequired1);


}



static void taskReadButton(void * a)
{
  
   button_t  button;
   
   //Init the struct
   button.number=0;
   button.state=RELEASE;
   button.time=0;
   


   while(1)
   {
        //Delay - Who often the task will be entered
        vTaskDelay(UPDATE_BUTTON_TIME/portTICK_RATE_MS); 
        
        //check if it is press
        if (!ciaaReadInput(button.number))
        {
            button.time+=UPDATE_BUTTON_TIME;
            if (button.time>2000000000)   //Set the limit
                button.time=2000000000; 
            if (button.time>TIME_NOT_REBOUND)
                button.state=PRESS; 
        }
        else
        {
            button.state=RELEASE;
            if (button.time>TIME_NOT_REBOUND)
              {
                //take the time of press in mseg
                buttonTime=button.time;
                uint32_t aux=buttonTime;
                xQueueSend( xQueue1, ( void * ) &aux, 0);
              }
            button.time=0;
            
        }

        if(button.state==PRESS)
          {  
            ciaaWriteOutput(0,1);  //Led RED ON
          }
          else
          {
            ciaaWriteOutput(0,0);  //Led RED OFF
          }  
        


   }

}



static void task(void * a)
{
  uint32_t buf;  //buffer to receive the data
  while (1) {
      if (xSemaphoreTake(xSemaphore,portMAX_DELAY))
        {
            if (xQueueReceive(xQueue1,&buf,portMAX_DELAY))
            {  
            //ciaaWriteOutput(3,1);  
            //Check if the time is zero, if not blink
                if (buf>0)
                {
                 ciaaWriteOutput(3,1);  //Led 4 ON
                 vTaskDelay(buf/ portTICK_RATE_MS);
                 ciaaWriteOutput(3,0); 
                }
            }
        }  
  }
}



/*==================[external functions definition]==========================*/

int main(void)
{
  initHardware();

  // Create a semaphore
   vSemaphoreCreateBinary(xSemaphore);
   xSemaphoreTake(xSemaphore,portMAX_DELAY);
  
//create the queue
  xQueue1 = xQueueCreate( QUEUE1_SIZE, sizeof(uint32_t) );

  if (xQueue1==NULL)
    ciaaWriteOutput(4,1); //if it is not create, turn on the red led

    //Create task to read the button
  xTaskCreate(taskReadButton, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
       
  xTaskCreate(task, (const char *)"task", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

  vTaskStartScheduler();

  while (1) {
  }
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
