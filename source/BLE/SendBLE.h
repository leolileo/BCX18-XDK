/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for Licensee’s application development. 
* Fitness and suitability of the example code for any use within Licensee’s applications need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
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
/* @file
 * @brief  This Module is Configuration header for SendAccelerometerDataOverUdpAndBle configurations
 *
 */
/* header definition ******************************************************** */
#ifndef XDK110_SENDACCELDATAOVERUDPANDBLE_H_
#define XDK110_SENDACCELDATAOVERUDPANDBLE_H_

#include "BCDS_Basics.h"

/* local interface declaration ********************************************** */

/* local type and macro definitions */
typedef enum returnTypes_e
{
    STATUS_FAILURE,
    STATUS_SUCCESS,
    SOCKET_ERROR,
    SEND_ERROR
} returnTypes_t;

#define WIFI_TX_FREQ                    UINT32_C(1000)             /**< Macro to represent One second time unit*/
#define BLE_TX_FREQ                     UINT32_C(1000)             /**< Macro to represent One second time unit*/
#define ZERO                            UINT32_C(0)               /**< Macro to define value zero*/
#define TIMER_AUTORELOAD_ON             UINT32_C(1)             /**< Auto reload of timer is enabled*/
#define TIMER_AUTORELOAD_OFF            UINT32_C(0)             /**< Auto reload of timer is disabled*/

/** Network configurations */
#warning Please provide WLAN related configurations, with valid SSID & WPA key and server ip address where packets are to be sent in the macros .
#define WLAN_CONNECT_WPA_SSID               "NETWORK_SSID"         /**< Macros to define WPA/WPA2 network settings */
#define WLAN_CONNECT_WPA_PASS               "NETWORK_WPA_KEY"      /**< Macros to define WPA/WPA2 network settings */
#define SL_SECURITY_TYPE                SL_SEC_TYPE_WPA          		 /**< Macro to define security type for the network*/
#define BUFFER_SIZE                     UINT8_C(3)
/** IP addressed of server side socket.Should be in long format, E.g: 0xc0a82177 == 192.168.33.119 */
#define SERVER_IP                       UINT32_C(0xC0A80195)             /**< 192.168.33.141=c0a8218D, 1.138*/
#define SERVER_PORT                     UINT16_C(6666)                   /**< Port number on which server will listen */

#define NUMBER_ZERO                     UINT8_C(0)                       /**< Zero value for BLE variables */
#define NUMBER_ONE                      UINT8_C(1)                       /**< One value for BLE variables */
#define BLETRANSMITLENGTH               UINT8_C(16)                      /**<Transmit length for BLE */
#define ACCEL_RECEIVELENGTH             UINT8_C(30)                      /**< Receive length for BLE */
#define USB_RECEIVE_BUFFER_LEN          UINT8_C(20)                      /**< The Maximum length of usb receive buffer */
#define SET_IDLE_MODE                   UINT8_C('i')                     /**< command to set the device in idle mode */
#define SET_DISCOVERABLE_MODE           UINT8_C('d')                     /**< command to set the device in discoverable mode */
#define SET_SLEEP_MODE                  UINT8_C('s')                     /**< command to set the device in sleep mode */
#define GET_DEVICE_MODE                 UINT8_C('g')                     /**< command to get the device current state */
#define BLE_RECEIVE_BUFFER_SIZE       UINT8_C(32)                      /**< Buffer size for commands received over BLE*/
#define XDK_BLE_DEVICE_NAME           "XDK_UDP_BLE"                       /**< Name of the BLE device*/
#define XDK_BLE_DEVICE_NAME_LENGTH    UINT8_C(sizeof(XDK_BLE_DEVICE_NAME))/**< length of the BLE device name*/
#define BLE_START_SYNC_TIMEOUT        UINT32_C(5000)
#define BLE_WAKEUP_SYNC_TIMEOUT       UINT32_C(5000)
#define BLE_SEND_TIMEOUT       UINT32_C(1000)
#define BLE_TRIGGER_START_CMD      UINT32_C(1)
#define BLE_TRIGGER_END_CMD      UINT32_C(0)

/* local function prototype declarations */

/* local module global variable declarations */

/* local inline function definitions */
/**
 * @brief This is a template function where the user can write his custom application.
 *
 * @param[in] CmdProcessorHandle Handle of the main commandprocessor
 *
 * @param[in] param2  Currently not used will be used in future
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2);

#endif /* XDK110_SENDACCELDATAOVERUDPANDBLE_H_ */

/** ************************************************************************* */
