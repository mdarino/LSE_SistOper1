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

  @file     ej4_2.c

  @brief    EJERCICIO 4.2 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/


/**

 EJERCICIO 4.2   (Spanish)

Caso   de   uso​
:   Dos   tareas   comparten   un   recurso,   debe   turnarse   el   acceso   al   mismo   para   que   no   lo  
corrompa. 
 
Ejercicio​
: Escribir un programa con tres tareas: 
 
∙ Una tarea que destella un led a 0,5Hz (500ms ON, 500ms OFF). 
∙ Otra   tarea   que   mide   el   tiempo   de   pulsación  
 de   un   botón.   En   cuanto   se   suelta,   lo   enviará   por  
cola a la tarea 3 que hará destellar al led durante este tiempo. 
 
Use   un   mutex   para   turnar   el   acceso   al   LED   y   
que   no   se   perturbe   ninguna   de   las   formas   de   onda.   
Lo  importante   es   no   interrumpir   el   tiempo   de   alto   en   
exhibición,   y   no   "pegarse"   al   tiempo   de   alto   de   la  
otra tarea (deje un tiempo de off como guardabanda). 

 **/




/** \addtogroup rtos FreeRTOS Ejer4.2
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

    #define UPDATE_BUTTON_TIME 5 ///Time in mseg, who often is checked the button's state    
    #define TIME_NOT_REBOUND   20 ///Delay to avoid the rebound on the button(in mseg)

typedef struct  STR_Button
{
    uint8_t     state;
    uint32_t    time;
    uint8_t     number;
}button_t;


/*==================[internal data declaration]==============================*/

xSemaphoreHandle xMutex;

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
            
            if (button.state==PRESS)
              {
                //xSemaphoreGive(xSemaphore);
                //take the time of press in mseg
                buttonTime=button.time;
              }
            button.state=RELEASE;
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


static void taskBlinkLed1(void * a)
{
  while (1) {
        
    if (buttonTime>0)
      {
        xSemaphoreTake(xMutex,portMAX_DELAY);
        ciaaWriteOutput(3,1);;  //ver tarea led1
        ciaaWriteOutput(5,1);;  //led compartido
        vTaskDelay(buttonTime/ portTICK_RATE_MS);
        xSemaphoreGive(xMutex);
        ciaaWriteOutput(3,0);
        ciaaWriteOutput(5,0);;  //led compartido
        vTaskDelay(buttonTime/ portTICK_RATE_MS);
      }  



  }
}


static void taskBlinkLed2(void * b)
{
  while (1) {
        xSemaphoreTake(xMutex,portMAX_DELAY);
        ciaaWriteOutput(4,1);;  //Led 4 ON
        ciaaWriteOutput(5,1);;  //led compartido
        vTaskDelay(500/ portTICK_RATE_MS);
        xSemaphoreGive(xMutex);
        ciaaWriteOutput(4,0);
        ciaaWriteOutput(5,0);;  //led compartido
        vTaskDelay(500/ portTICK_RATE_MS);
  }
}


/*==================[external functions definition]==========================*/

int main(void)
{
    //Start the HW
	initHardware();
   
  // Create a semaphore
  xMutex = xSemaphoreCreateMutex();  

    //Create task to read the button
	xTaskCreate(taskReadButton, (const char *)"taskReadButton", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
    
    //Create task to blick the led
  xTaskCreate(taskBlinkLed1, (const char *)"taskReadButton1", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
  xTaskCreate(taskBlinkLed2, (const char *)"taskReadButton2", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

    //Start the Scheduler
	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
