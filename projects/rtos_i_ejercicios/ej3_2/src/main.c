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

  @file     ej3_2.c

  @brief    EJERCICIO 3.1 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/


/**

 EJERCICIO 3.2  (Spanish)

Caso de uso: El mensaje recibido por cola modifica de alguna manera el comportamiento de la tarea
receptora, pero la misma debe seguir trabajando (no bloquear) aunque no reciba mensajes
Ejercicio: Una tarea destella continuamente un led, con una frecuencia constante y un ciclo de
actividad que recibe de otras tareas mediante una cola. La tarea no debe bloquearse, ya que
mientras no reciba mensajes debe mantener el led titilando.

 **/




/** \addtogroup rtos FreeRTOS Ejer3.2
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../../rtos_i_ejercicios/ej1_1/inc/main.h"

#include "../../../rtos_i_ejercicios/ej1_1/inc/FreeRTOSConfig.h"
#include "ciaaIO.h"

#include "FreeRTOS.h"
//#include "semphr.h"
#include "queue.h"
#include "task.h"


/*==================[macros and definitions]=================================*/
    //BUTTONS STATES:
    #define PRESS   1  ///Buttons is press
    #define RELEASE 0  ///Buttons is relaese
    #define TIME_NOT_REBOUND  10 ///Button delay to check the buttons
    #define TICKS_BUTTON 2 ///How many TIME_NOT_REBOUND is press

  //define for the queue
    #define QUEUE1_SIZE 5
    #define READY       1
    #define NOT_READY   0


typedef struct  STR_Button
{
    uint8_t     state;
    uint16_t    time;
    uint8_t     number;
}button_t;


/*==================[internal data declaration]==============================*/
 QueueHandle_t xQueue1;
/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

//Global Variables
//uint16_t  buttonTime=0;  ///Save the button time press in os ticks


/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    //Set the system core
    SystemCoreClockUpdate();
    //Init the clock of HW - gpio
    Board_Init();
    //Init the EDU-CIAA HW
    ciaaIOInit();
    //Turn of the LED
    ciaaWriteOutput(0, 0);


}



static void taskReadButton(void * a)
{
	
   button_t  button;
   
   //Init the struct
   button.number=0;
   button.state=RELEASE;
   button.time=0;
   
   uint32_t  buttonTime=0;

   while(1)
   {
        //Delay
        vTaskDelay(TIME_NOT_REBOUND/portTICK_RATE_MS); 
        
        //check if it is press
        if (!ciaaReadInput(button.number))
        {
            button.time++;
            if (button.time>65000)
                button.time=65000;
            if (button.time>TICKS_BUTTON)
                button.state=PRESS; 

        }
        else
        {
            button.state=RELEASE;
            if (button.time>TICKS_BUTTON)
              {
                //take the time of press in mseg
                buttonTime=(button.time-TICKS_BUTTON)*TIME_NOT_REBOUND;
                xQueueSend( xQueue1, ( void * ) &buttonTime, 0);

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


static void taskBlickLed(void * a)
{
    
   uint32_t buttonTime2=0;  //buffer to receive the data
    while (1) {
        xQueueReceive(xQueue1,&buttonTime2,0);
          
        //ciaaWriteOutput(3,1);  
        //Check if the time is zero, if not blink
        if (buttonTime2>0)
          {
           ciaaToggleOutput(3);  //Led 4 ON
           vTaskDelay(buttonTime2/ portTICK_RATE_MS);
            ciaaWriteOutput(5,1);
          }
        else
          {
           ciaaToggleOutput(3);  //Led 4 ON
           vTaskDelay(500/ portTICK_RATE_MS);
           ciaaWriteOutput(5,0);
          }    
        

 
        
        }
}
/*==================[external functions definition]==========================*/

int main(void)
{
    //Start the HW
	initHardware();
  ciaaWriteOutput(3,0);

  //create the queue
  xQueue1 = xQueueCreate( QUEUE1_SIZE, sizeof(uint32_t) );

  if (xQueue1==NULL)
    ciaaWriteOutput(4,1); //if it is not create, turn on the red led



    //Create task to read the button
	xTaskCreate(taskReadButton, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
    
    //Create task to blick the led
  xTaskCreate(taskBlickLed, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);


    //Start the Scheduler
	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
