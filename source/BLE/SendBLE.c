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
 * @ingroup APPS_LIST
 *
 * @defgroup SEND_ACCELDATA_OVER_UDP_AND_BLE Send Accelerometer Data UDP And BLE
 * @{
 *
 * @brief Demo application of Transmitting BMA280 Accelerometer data on WIFI/BLE(Bluetooth Low Energy)
 *
 * @details This example demonstrates how to read sensor values from the BMA280 Acceleration sensor and send
 * the data over WLAN (UDP Broadcast) and Bluetooth Low Energy via the custom Bi-Directional Service.<br>
 * Either use your Android or iOS mobile phone (see [Android](https://play.google.com/store/apps/details?id=com.macdom.ble.blescanner&hl=en)
 * or [iOS](https://itunes.apple.com/in/app/lightblue-explorer-bluetooth-low-energy/id557428110?mt=8) App Store) to connect to XDK
 * and receive the data. Send command <b>start (HEX: 0x7374617274)</b>  to XDK via Bluetooth Low Energy, so that streaming of data begins.
 * To stop the streaming send command <b>end (HEX: 656e64)</b>
 *
 * This Application enables the bi-directional service in ble and sends Accelerometer Data over ble and UDP. <br>
 * <b> Bi-Directional Service : </b>
 *
 *  Service             |  Characteristic                |         Attribute-Type             |               UUID                   |
 * ---------------------|--------------------------------|------------------------------------|--------------------------------------|
 * BidirectionalService |    NA                          |         uint8_t                    | b9e875c0-1cfa-11e6-b797-0002a5d5c51b |
 * NA                   |    Rx                          |         uint8_t X[20]              | 0c68d100-266f-11e6-b388-0002a5d5c51b |
 * NA                   |    Tx                          |         uint8_t X[20]              | 1ed9e2c0-266f-11e6-850b-0002a5d5c51b |
 *
 *
 * @file
 **/
 
/* module includes ********************************************************** */
#include "XDKAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_ACCEL_OVER_UDP_BLE

/* own header files */
#include "SendBLE.h"
#include "XdkSensorHandle.h"
#include "XdkUsbResetUtility.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "BCDS_Assert.h"
#include "BCDS_Ble.h"
#include "BCDS_BidirectionalService.h"
#include "BCDS_BlePeripheral.h"
#include "BCDS_CmdProcessor.h"
#include "simplelink.h"
#include "Serval_Ip.h"
#include "BCDS_WlanConnect.h"
#include "BCDS_NetworkConfig.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */
static xTimerHandle BleTransmitTimerHandle; /**< variable to store timer handle*/
static xTimerHandle WifiTransmitTimerHandle; /**< variable to store timer handle*/
CmdProcessor_T *AppCmdProcessorHandle; /**< Application Command Processor Handle */
static uint8_t bleTransmitStatus = NUMBER_ZERO; /**< Validate the repeated start flag */
static SemaphoreHandle_t BleStartSyncSemphr = NULL;
static SemaphoreHandle_t BleWakeUpSyncSemphr = NULL;
static SemaphoreHandle_t SendCompleteSync = NULL;

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/** The function to send start or stop message to Bidirectional DataExchange service
 */
static void BleStartEndMsgSend(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    Retcode_T bleRetval = RETCODE_OK;

    if (param2 == BLE_TRIGGER_START_CMD)
    {
        bleRetval = BidirectionalService_SendData(((uint8_t*) "X      Y      Z"), ((uint8_t) sizeof("X      Y      Z") - 1));
    }
    if (param2 == BLE_TRIGGER_END_CMD)
    {
        bleRetval = BidirectionalService_SendData(((uint8_t*) "Transfer Terminated!"), ((uint8_t) sizeof("Transfer Terminated!") - 1));
    }
    if (RETCODE_OK == bleRetval)
    {
        if (pdFALSE == xSemaphoreTake(SendCompleteSync, BLE_SEND_TIMEOUT))
        {
            bleRetval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
        }
    }
    if (RETCODE_OK != bleRetval)
    {
        printf("Not able to send response to start or stop command  on BLE  : \r\n");
    }
}

/**
 * @brief Callback function called on data reception over BLE
 *
 * @param [in]  rxBuffer : Buffer in which received data to be stored.
 *
 * @param [in]  rxDataLength : Length of received data.
 */

static void BleDataReceivedCallBack(uint8_t *rxBuffer, uint8_t rxDataLength)
{
    Retcode_T retVal = RETCODE_OK;
    uint8_t bleReceiveBuff[BLE_RECEIVE_BUFFER_SIZE];
    if (rxDataLength >= BLE_RECEIVE_BUFFER_SIZE)
    {
        printf("Data length received is invalid \n");
    }
    else
    {
        memset(bleReceiveBuff, 0, sizeof(bleReceiveBuff));
        memcpy(bleReceiveBuff, rxBuffer, rxDataLength);
        /* make sure that the received string is null-terminated */
        bleReceiveBuff[rxDataLength] = '\0';

        /* validate received data */
        if ((NUMBER_ZERO == strcmp((const char *) bleReceiveBuff, "start"))
                && (NUMBER_ZERO == bleTransmitStatus))
        {
            retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle, BleStartEndMsgSend, NULL, UINT32_C(1));
            if (RETCODE_OK != retVal)
            {
                printf("Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
            }
            /* start accelerometer data transmission timer */
            if (pdTRUE != xTimerStart(BleTransmitTimerHandle, (BLE_TX_FREQ / portTICK_RATE_MS)))
            {
                /* Assertion Reason : Failed to start software timer. Check command queue size of software timer service*/
                assert(false);
            }
            else
            {
                bleTransmitStatus = NUMBER_ONE;
            }
        }
        else if ((NUMBER_ZERO == strcmp((const char *) bleReceiveBuff, "end"))
                && (NUMBER_ONE == bleTransmitStatus))
        {

            /* stop accelerometer data transmission timer */
            if (pdTRUE != xTimerStop(BleTransmitTimerHandle, NUMBER_ZERO))
            {
                /* Assertion Reason: Failed to start software timer. Check command queue size of software timer service. */
                assert(false);
            }
            else
            {
                bleTransmitStatus = NUMBER_ZERO;
                retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle, BleStartEndMsgSend, NULL, UINT32_C(0));
                if (RETCODE_OK != retVal)
                {
                    printf("Failed to Enqueue BleStartEndMsgSend to Application Command Processor \r\n");
                }
            }

        }
    }
}

/**
 * @brief Callback function called on BLE send completion
 *
 * @param [in]  sendStatus : event to be send by BLE during communication.
 *
 */
static void BleAccelDataSentCallback(Retcode_T sendStatus)
{
    if (RETCODE_OK != sendStatus)
    {
        printf("Error in transmitting the Accel Data over BLE ERROR Code %ui  : \r\n", (unsigned int) sendStatus);
    }
    if ( xSemaphoreGive( SendCompleteSync ) != pdTRUE)
    {
        /* We would not expect this call to fail because we must have obtained the semaphore to get here. */
        Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
    }
}

/**
 *  @brief       Function to periodically send accel data over BLE. This is called in a command processor context.
 *
 * @param[in]   *param1: a generic pointer to any context data structure which will be passed to the function when it is invoked by the command processor.
 *
 * @param[in]    param2: a generic 32 bit value  which will be passed to the function when it is invoked by the command processor.
 */
static void SendAccelDataoverBle(void * param1, uint32_t param2)
{
	printf("Sending data to BLE");
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T retval;
    /* buffer for accel data receive function */
    uint8_t accelDataRec[ACCEL_RECEIVELENGTH] = { 0 };
    Accelerometer_XyzData_T getaccelData = { INT32_C(0), INT32_C(0), INT32_C(0) };
    if ((NUMBER_ONE == bleTransmitStatus))
    {
        /*Reading accel Data (X,Y,Z) */
        retval = Accelerometer_readXyzGValue(xdkAccelerometers_BMA280_Handle, &getaccelData);
        if (RETCODE_OK == retval)
        {
            sprintf((char*) accelDataRec, "%ld %ld %ld", (long int) getaccelData.xAxisData,
                    (long int) getaccelData.yAxisData, (long int) getaccelData.zAxisData);
        }
        else
        {
            printf("Reading of accel data failed and is filled with dummy value \r\n");
        }
        retval = BidirectionalService_SendData((uint8_t*) accelDataRec, (uint8_t) BLETRANSMITLENGTH);
        if (RETCODE_OK == retval)
        {
            if (pdFALSE == xSemaphoreTake(SendCompleteSync, BLE_SEND_TIMEOUT))
            {
                retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
            }
        }
        if (RETCODE_OK != retval)
        {
            printf("Sending failed over Ble \r\n");
        }
    }
}

/**
 * @brief        This is a application timer callback function used to enqueue SendAccelDataoverBle function
 *               to the command processor.
 *
 * @param[in]    pvParameters unused parameter.
 */
static void EnqueueAccelDatatoBle(void *pvParameters)
{
    BCDS_UNUSED(pvParameters);
    printf("Loading data to the queue");

    Retcode_T retVal = CmdProcessor_Enqueue(AppCmdProcessorHandle, SendAccelDataoverBle, NULL, UINT32_C(0));
    if (RETCODE_OK != retVal)
    {
        printf("Failed to Enqueue SendAccelDataoverBle to Application Command Processor \r\n");
    }
}

/**
 *  The function to register the bidirectional service
 */

static Retcode_T BiDirectionalServiceRegistryCallback(void)
{
    Retcode_T retval = RETCODE_OK;

    retval = BidirectionalService_Init(BleDataReceivedCallBack, BleAccelDataSentCallback);
    if (RETCODE_OK == retval)
    {
        retval = BidirectionalService_Register();
    }

    return (retval);
}

/**
 * @brief Callback function called on BLE event
 *
 * @param [in]  event : event to be send by BLE during communication.
 *
 * @param [in]  data : void pointer pointing to a data type based on event.
 *
 * event                                    | data type                        |
 * -----------------------------------------|----------------------------------|
 * BLE_PERIPHERAL_STARTED                   |   Retcode_T                      |
 * BLE_PERIPHERAL_SERVICES_REGISTERED       |   unused                         |
 * BLE_PERIPHERAL_SLEEP_SUCCEEDED           |   Retcode_T                      |
 * BLE_PERIPHERAL_WAKEUP_SUCCEEDED          |   Retcode_T                      |
 * BLE_PERIPHERAL_CONNECTED                 |   Ble_RemoteDeviceAddress_T      |
 * BLE_PERIPHERAL_DISCONNECTED              |   Ble_RemoteDeviceAddress_T      |
 * BLE_PERIPHERAL_ERROR                     |   Retcode_T                      |
 *
 */
static void BleEventCallBack(BlePeripheral_Event_T event, void * data)
{

    BlePeripheral_Event_T Event = event;
    BCDS_UNUSED(data);
    switch (Event)
    {
    case BLE_PERIPHERAL_SERVICES_REGISTERED:
        break;
    case BLE_PERIPHERAL_STARTED:
        printf("BLE powered ON successfully \r\n");
        if ( xSemaphoreGive( BleStartSyncSemphr ) != pdTRUE)
        {
            /* We would not expect this call to fail because we must have obtained the semaphore to get here. */
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        break;
    case BLE_PERIPHERAL_SLEEP_SUCCEEDED:
        printf("BLE successfully entered into sleep mode \r\n");
        break;
    case BLE_PERIPHERAL_WAKEUP_SUCCEEDED:
        printf("Device Wake up succceded  : \r\n");
        if ( xSemaphoreGive( BleWakeUpSyncSemphr ) != pdTRUE)
        {
            /* We would not expect this call to fail because we must have obtained the semaphore to get here. */
            Retcode_RaiseError(RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_GIVE_ERROR));
        }
        break;
    case BLE_PERIPHERAL_CONNECTED:
        printf("Device connected  : \r\n");
        break;
    case BLE_PERIPHERAL_DISCONNECTED:
        printf("Device Disconnected   : \r\n");
        if (NULL != BleTransmitTimerHandle)
        {
            /* stop Ble timer accelerometer data transmission timer */
            if (pdTRUE != xTimerStop(BleTransmitTimerHandle, NUMBER_ZERO))
            {
                /* Assertion Reason: Failed to start software timer. Check command queue size of software timer service. */
                printf("Failed to stop the BLE Timer \r\n");
                assert(false);
            }
            else
            {
                printf("BLE Timer stopped \r\n");
            }
        }
        bleTransmitStatus = NUMBER_ZERO;
        break;
    case BLE_PERIPHERAL_ERROR:
        printf("BLE Error Event  : \r\n");
        break;
    default:
        /* assertion reason : invalid status of Bluetooth Device */
        assert(false);
        break;
    }
}

/**
 *  @brief
 *      -Function to initialize the Accel Sensor,wifi network and BTLE.
 *      -Create timer task to start WiFi Connect and get IP function after one second. After that another timer
 *      -Create timer task to start BTLE state machine and discovering/connecting BTLE device
 *      -Create timer task to Start after getting IP and then send the Accel Data to WiFi and BTLE
 *
 */
Retcode_T init(void)
{
    /* return value for BLE stack configuration */
    Retcode_T retval = RETCODE_OK;
    static_assert((portTICK_RATE_MS != 0), "Tick rate MS is zero");

    BleStartSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleStartSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    BleWakeUpSyncSemphr = xSemaphoreCreateBinary();
    if (NULL == BleWakeUpSyncSemphr)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }
    SendCompleteSync = xSemaphoreCreateBinary();
    if (NULL == SendCompleteSync)
    {
        return (RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_CREATE_ERROR));
    }

    /*initialize accel sensor*/
    retval = Accelerometer_init(xdkAccelerometers_BMA280_Handle);

    if (RETCODE_OK == retval)
    {
        /* create timer task to get and transmit accel data via BLE for every one second automatically*/
        /*WifiTransmitTimerHandle = xTimerCreate((char * const ) "WifiTransmitTimerHandle", pdMS_TO_TICKS(WIFI_TX_FREQ), TIMER_AUTORELOAD_ON, NULL, EnqueueAccelDatatoWifi);

        if (NULL != WifiTransmitTimerHandle)
        {
            retval = wifiConnect();
            if (RETCODE_OK != retval)
            {
                printf("wifi Connection failed \n");
            }
        }
        else
        {
            printf("Failed to create WifiTransmitTimerHandle so wifi connection not enabled \r\n");
        }*/
        /* create timer task to get and transmit accel data via BLE for every one second automatically*/
        BleTransmitTimerHandle = xTimerCreate((char * const ) "BleTransmitTimerHandle", pdMS_TO_TICKS(BLE_TX_FREQ)
                , TIMER_AUTORELOAD_ON, NULL, EnqueueAccelDatatoBle);
        if (NULL != BleTransmitTimerHandle)
        {
            /* Initialize the BLE Interface */
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_Initialize(BleEventCallBack, BiDirectionalServiceRegistryCallback);
            }
            /* Configuring the XDK BLE Device name  */
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_SetDeviceName((uint8_t*) XDK_BLE_DEVICE_NAME);
            }
            if (RETCODE_OK == retval)
            {
                retval = BlePeripheral_Start();
            }
            /* Powering on BLE module*/
            if (RETCODE_OK == retval)
            {
                if (pdTRUE != xSemaphoreTake(BleStartSyncSemphr, BLE_START_SYNC_TIMEOUT))
                {
                    printf("Failed to Start BLE before timeout, Ble Initialization failed \n");
                    retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
                }
            }
            if (RETCODE_OK == retval)
            {
                /* Wake up BLE module*/
                retval = BlePeripheral_Wakeup();
            }
            if (RETCODE_OK == retval)
            {
                if (pdTRUE != xSemaphoreTake(BleWakeUpSyncSemphr, BLE_WAKEUP_SYNC_TIMEOUT))
                {
                    printf("Failed to Wake up BLE before timeout, Ble Initialization failed \n");
                    retval = RETCODE(RETCODE_SEVERITY_ERROR, SEMAPHORE_TIME_OUT_ERROR);
                }
            }
            if (RETCODE_OK == retval)
            {
                printf(" Ble Initialization succeded \n");
            }
            else
            {
                printf("Ble Initialization Failed \r\n");
            }
        }
        else
        {
            retval = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_FAILURE);
            printf("Failed to create BleTransmitTimerHandle so ble connection not enabled\n");
        }
    }
    else /* Accel not initialised so no further process*/
    {
        printf("Failed to run the app \r\n");
    }
    return retval;
}

/**
 * @brief This is a template function where the user can write his custom application.
 *
 */
void appInitSystem(void * CmdProcessorHandle, uint32_t param2)
{
    if (CmdProcessorHandle == NULL)
    {
        printf("Command processor handle is null \n\r");
        assert(false);
    }
    AppCmdProcessorHandle = (CmdProcessor_T *) CmdProcessorHandle;
    BCDS_UNUSED(param2);
    /*Call the Wifi and Btle module API */
    Retcode_T retVal = RETCODE_OK;
    retVal = init();
    if (RETCODE_OK == retVal)
    {
        printf("SendAccelDataOverUdpandBle App Initialization completed successfully \r\n ");
    }
    else
    {
        printf("SendAccelDataOverUdpandBle App Initialization failed \r\n ");
    }
}

/**@} */
/** ************************************************************************* */
