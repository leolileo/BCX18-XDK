/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee�s application development. 
* Fitness and suitability of the example code for any use within Licensee�s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/
/**
 *  @file        
 *
 *  Configuration header for the BME_280 file.
 *
 * ****************************************************************************/

/* header definition ******************************************************** */
#ifndef BME_280_CH_H_
#define BME_280_CH_H_

/* local interface declaration ********************************************** */

/* local type and macro definitions */
#define PED_THREESECONDDELAY  UINT32_C(3000)	 	/** one second is represented by this macro */
#define PED_TIMERBLOCKTIME  UINT32_C(0xffff) 	/** Macro used to define blocktime of a timer*/
#define PED_USEI2CPROTOCOL  UINT8_C(0)	 		/** I2c protocol is represented by this macro*/
#define PED_VALUE_ZERO 		UINT8_C(0)	 		/** default value*/
#define PED_I2C_ADDRESS     UINT8_C(0x18) 	    /** i2c address for BME280 */

/* local function prototype declarations */

/**
 * @brief Gets the raw data from BME Environmental and prints through the USB printf on serial port
 * @param[in] pvParameters Rtos task should be defined with the type void *(as argument)
 */
void bme280_getSensorValues(void *pvParameters);

/* local module global variable declarations */

/* local inline function definitions */

#endif /* BME_280_CH_H_ */

/** ************************************************************************* */
