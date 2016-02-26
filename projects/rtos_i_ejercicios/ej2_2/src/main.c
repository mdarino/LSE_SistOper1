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

  @file     ej2.2.c

  @brief    EJERCICIO 2.2 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/


/**

 EJERCICIO 2.2    (Spanish)

Caso   de   uso​
:   Varias   tareas   esperando   un   evento.   Por   ejemplo,   un   cambio   de   estado   del   sistema   que  
debe ser atendido por varias tareas con prioridades diferentes. 
 
Ejercicio​
:   Instanciar   3   veces   la   tarea   que   destella   un   led,   cada   una   aplicada   a   un   color   del   led   RGB   y  
hacer   que   las   tres   tareas   generen   el   destello   cuando   se   reciba   el   semáforo.   Las   tareas   esperaran   al  
semáforo indefinidamente. 
 
El   semáforo   no   es   más   que   un   mensaje   al   scheduler,   recibido   por   cuantas   tareas   estuvieran  
esperando   por   él.   Prestar   atención   al   detalle   del   parámetro   de   tarea,   este   debe   indicar   a   cada  
instancia creada que color le corresponde. 

 **/




/** \addtogroup rtos FreeRTOS Ejer1.4
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../../rtos_i_ejercicios/ej1_1/inc/main.h"

#include "../../../rtos_i_ejercicios/ej1_1/inc/FreeRTOSConfig.h"
#include "ciaaIO.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


/*==================[macros and definitions]=================================*/
    //BUTTONS STATES:
    #define PRESS   1  ///Buttons is press
    #define RELEASE 0  ///Buttons is relaese
    #define TIME_NOT_REBOUND  10 ///Button delay to check the buttons
    #define TICKS_BUTTON 2 ///How many TIME_NOT_REBOUND is press

typedef struct  STR_Button
{
    uint8_t     state;
    uint16_t    time;
    uint8_t     number;
}button_t;


/*==================[internal data declaration]==============================*/

xSemaphoreHandle xSemaphore;

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

//Global Variables
uint16_t  buttonTime=0;  ///Save the button time press in os ticks


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



static void taskBlickLed(void * a)
{
    uint8_t *auxP = (uint8_t *)a;
    uint8_t numLedRGB;
    numLedRGB=*auxP;
   
    while (1) {
        
        if (xSemaphoreTake(xSemaphore,portMAX_DELAY))
        {
          //ciaaWriteOutput(3,1);  
          //Check if the time is zero, if not blink
          xSemaphoreGive(xSemaphore);
          ciaaToggleOutput(numLedRGB);  //Led x
          vTaskDelay(500/ portTICK_RATE_MS);
            
          }    
        

        }
        
        
}
/*==================[external functions definition]==========================*/

int main(void)
{
    //Start the HW
  uint8_t numLedRGBred;
  uint8_t numLedRGBgreen;
  uint8_t numLedRGBblue;

	initHardware();
  ciaaWriteOutput(3,0); 
  // Create a semaphore
  vSemaphoreCreateBinary(xSemaphore);
  //xSemaphoreTake(xSemaphore,portMAX_DELAY);
    //Create task to read the button
	//xTaskCreate(taskReadButton, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
    
  numLedRGBred=0; //LED RED
    //Create task to blick the led
  xTaskCreate(taskBlickLed, (const char *)"task", configMINIMAL_STACK_SIZE*2, &numLedRGBred, tskIDLE_PRIORITY+1, 0);

  numLedRGBgreen=1; //LED GREEN
    //Create task to blick the led
  xTaskCreate(taskBlickLed, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, &numLedRGBgreen, tskIDLE_PRIORITY+1, 0);

  numLedRGBblue=2; //LED BLUE
    //Create task to blick the led
  xTaskCreate(taskBlickLed, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, &numLedRGBblue, tskIDLE_PRIORITY+1, 0);

    //Start the Scheduler
	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
