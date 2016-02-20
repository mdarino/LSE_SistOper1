/* Copyright 2015, Pablo Ridolfi
 * Copyright 2015, Marcos Darino
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

  @file     ej1_4.c

  @brief    EJERCICIO 1.2 - RTOS 1

  @author   Marcos Darino (MD)

 ******************************************************************************/


/**

 EJERCICIO 1.2    (Spanish)
	Implementar   una   tarea   que   genere   una   onda   cuadrada   (y   que   encienda   un   LED)   con  
	periodo de 1 seg y ciclos de actividad incrementándose 100 ms, 200 ms, 300 ms. 
 **/


/** \addtogroup rtos_blink FreeRTOS Ejer1.2
 ** @{ */

/*==================[inclusions]=============================================*/

#include "../../../rtos_i_ejercicios/ej1_1/inc/main.h"

#include "../../../rtos_i_ejercicios/ej1_1/inc/FreeRTOSConfig.h"
#include "ciaaIO.h"

#include "FreeRTOS.h"
#include "task.h"


/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();

    ciaaIOInit();

    ciaaWriteOutput(0, 0);
}

static void task(void * a)
{
	//Get the current Tick
	TickType_t  tickNow = xTaskGetTickCount();
	//States
	uint8_t  state=0;

	while (1) {
		
		//Jump to the different states
		switch(state)
		{
			case 0:
				ciaaWriteOutput(0,1);  //Led ON
				vTaskDelay(100/portTICK_RATE_MS); //100mseg
				state++; //jump to the next step
			break;

			case 1:
				ciaaWriteOutput(0,1);  //Led ON
				vTaskDelay(200/portTICK_RATE_MS); //200mseg
				state++; //jump to the next step
			break;

			case 2:
				ciaaWriteOutput(0,1);  //Led ON
				vTaskDelay(300/portTICK_RATE_MS); //300mseg
				state=0; //jump to the first step
			break;
		}

		ciaaWriteOutput(0,0);  //Led OFF
		//DelayUntil: Complete the second and update the tickNow
		vTaskDelayUntil(&tickNow,1000/portTICK_RATE_MS);


	}
}

/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware();

	xTaskCreate(task, (const char *)"task", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
