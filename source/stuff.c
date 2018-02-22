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
 * @defgroup HTTP_EXAMPLE_CLIENT stuff
 * @{
 *
 * @brief Demo application for communicating with the HTTP to GET and POST content on the Server and demonstrate the use of custom headers.
 *
 * @details This example shows how to use the network stack to perform an HTTP Client Request.
 *
 * The example connects to the server <a href="DEST_SERVER_HOST"></a> and establishes a connection to the server.
 *
 * To view your posts, browse to the directory name DEST_POST_PATH:
 *
 * You need to add your WLAN-Credentials in \ref stuff.h\n
 * When running the program keep the USB plugged in to the PC. You can see in the console output of the XDK-Workbench the content of the GET request.
 *
 * @file stuff.c
 **/

/* module includes ********************************************************** */

/* own header files */
#include "XDKAppInfo.h"

#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_HTTP_EXAMPLE_CLIENT

char POST_BODY [1000];
/* own header files */
#include "stuff.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "BCDS_WlanConnect.h"
#include "BCDS_NetworkConfig.h"
#include "BCDS_CmdProcessor.h"
#include "BME_280/BME_280_cc.c"
#include "BME_280/BME_280_ch.h"
#include "BME_280/BME_280_ih.h"
#include <Serval_HttpClient.h>
#include <Serval_Network.h>
#include "BCDS_ServalPal.h"
#include "BCDS_ServalPalWiFi.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "Serval_Http.h"
#if (HTTP_SECURE_ENABLE == 1)
#include <Serval_Security.h>
#include "x509.h"



#include "PostmanEchoCA.h"
#endif //#if (HTTP_SECURE_ENABLE == 1)
/* constant definitions ***************************************************** */

/* local variables ********************************************************** */
static xTaskHandle httpGetTaskHandle;
static xTaskHandle httpPostTaskHandle;
static xTimerHandle triggerHttpRequestTimerHandle;
static uint32_t httpGetPageOffset = 0;
static CmdProcessor_T CommandProcessorHandle;

#if (HTTP_SECURE_ENABLE == 1)
static EscX509_CertificateT trustedCertificates[1];

static const unsigned char caCert[] =
{   POSTMAN_ECHO_CA};

#include "cycurlib_config.h"
#include "tls_core.h"
#if (EscTls_KEY_EXCHANGE != EscTls_RSA)
#warning "Require RSA key support"
#endif
#endif  //#if (HTTP_SECURE_ENABLE == 1)
/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief This will initialize the ServalPAL module and its necessary resources.
 *
 * @return Status of initialization
 */
static Retcode_T ServalPalSetup(void)
{
    Retcode_T returnValue = RETCODE_OK;
    returnValue = CmdProcessor_Initialize(&CommandProcessorHandle, "Serval PAL", TASK_PRIORITY_SERVALPAL_CMD_PROC, TASK_STACK_SIZE_SERVALPAL_CMD_PROC, TASK_QUEUE_LEN_SERVALPAL_CMD_PROC);
    /* serval pal common init */
    if (RETCODE_OK == returnValue)
    {
        returnValue = ServalPal_Initialize(&CommandProcessorHandle);
    }
    if (RETCODE_OK == returnValue)
    {
        returnValue = ServalPalWiFi_Init();
    }
    if (RETCODE_OK == returnValue)
    {
        ServalPalWiFi_StateChangeInfo_T stateChangeInfo = { SERVALPALWIFI_OPEN, INT16_C(0) };
        returnValue = ServalPalWiFi_NotifyWiFiEvent(SERVALPALWIFI_STATE_CHANGE, &stateChangeInfo);
    }
    return returnValue;
}

#if (HTTP_SECURE_ENABLE == 1)

static Retcode_T initializeCertificates(void)
{
    EscTls_RETURN dtls_rc;

    dtls_rc = EscX509_Parse(&trustedCertificates[0], caCert, sizeof(caCert));
    if (dtls_rc != EscTlsRet_OK)
    {
        printf("Could not parse CA Certificate\r\n\r\n");
        return RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INIT_CA_REQUEST_FAILED);
    }
    return RETCODE_OK;
}

/** callback to CRT_CA Request */
static retcode_t httpsSecurityCallback(SecurityToken_T token, SecurityData_T* tokenData)
{
    assert(token.connection == TLS);
    assert(token.deviceRole == CLIENT);

    switch (token.type)
    {
        case CURRENT_TIME:
        printf("securityCallback - CURRENT_TIME request\r\n");
        return RC_DTLS_TOKEN_NOT_PROVIDED;

        case CRT_PEER_NAME:
        printf("securityCallback - CRT_PEER_NAME request\r\n");
        return RC_DTLS_TOKEN_NOT_PROVIDED;

        case CRT_CA:
        printf("securityCallback - CRT_CA request\r\n");
        return Serval_copyToOutputBuffer(&tokenData->caData.trustedCertificates, (const char*) trustedCertificates, sizeof(trustedCertificates));

        case CRT_OWN_CERTIFICATE:
        printf("securityCallback - CRT_OWN_CERTIFICATE request\r\n");
        return RC_DTLS_TOKEN_NOT_PROVIDED;

        default:
        printf("securityCallback - default case entered\r\n");
        return RC_DTLS_UNSUPPORTED_TOKEN;
    }
}

#endif /* (HTTP_SECURE_ENABLE == 1) */

/**
 * @brief Connects to the wireless network configured in stuff.h
 *
 *   @warning
 *      If the WLAN connection fails or we don't acquire an IP address, we will be stuck in this function forever.
 *      Check whether the callback "SimpleLinkWlanEventHandler" or "SimpleLinkNetAppEventHandler" hits once the
 *      sl_WlanConnect() API called, if not check for proper GPIO pin interrupt configuration or for any other issue.
 *
 * @retval     RETCODE_OK         We connected to the WLAN successfully.
 *
 */
static Retcode_T connectToWLAN(void)
{
    Retcode_T retcode;

    retcode = WlanConnect_Init();
    if (RETCODE_OK != retcode)
    {
        return retcode;
    }

    /* The order of calls is important here. WlanConnect_init initializes the CC3100 and prepares
     * its future use. Calls to NetworkConfig_ fail if WlanConnect_Init was not called beforehand.
     */
    retcode = NetworkConfig_SetIpDhcp(NULL);
    if (RETCODE_OK != retcode)
    {
        return retcode;
    }

    printf("Connecting to %s \r\n ", WLAN_SSID);
    /* Passing NULL as onConnection callback (last parameter) makes this a blocking call, i.e. the
     * WlanConnect_WPA function will return only once a connection to the WLAN has been established,
     * or if something went wrong while trying to do so. If you wanted non-blocking behavior, pass
     * a callback instead of NULL. */
    retcode = WlanConnect_WPA((WlanConnect_SSID_T) WLAN_SSID, (WlanConnect_PassPhrase_T) WLAN_PSK, NULL);
    if (RETCODE_OK != retcode)
    {
        return retcode;
    }

    NetworkConfig_IpSettings_T currentIpSettings;
    retcode = NetworkConfig_GetIpSettings(&currentIpSettings);
    if (RETCODE_OK != retcode)
    {
        return retcode;
    }
    else
    {
        uint32_t ipAddress = Basics_htonl(currentIpSettings.ipV4);

        char humanReadbleIpAddress[SERVAL_IP_ADDR_LEN] = { 0 };
        int conversionStatus = Ip_convertAddrToString(&ipAddress, humanReadbleIpAddress);
        if (conversionStatus < 0)
        {
            printf("Couldn't convert the IP address to string format \r\n");
        }
        else
        {
            printf("Connected to WPA network successfully \r\n");
            printf(" Ip address of the device %s \r\n", humanReadbleIpAddress);
        }
    }

    return retcode;
}

/**
 * @brief httpGetSentCallback is called when the HTTP request was pushed. The callerStatus
 *        passed to this function indicates whether the request was sent successfully or not.
 *
 * @param caller The callable to which this callback was assigned to
 * @param callerStatus The status indicating whether the request was sent or not
 * @return a retcode indicating the success of this callback
 */
static retcode_t httpRequestSentCallback(Callable_T* caller, retcode_t callerStatus)
{
    BCDS_UNUSED(caller);

    if (RC_OK == callerStatus)
    {
        printf("httpRequestSentCallback: HTTP request sent successfully.\r\n");
    }
    else
    {
        printf("httpRequestSentCallback: HTTP request failed to send. error=%d\r\n", callerStatus);
        printf("httpRequestSentCallback: Restarting request timer\r\n");
        xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);
    }

    return RC_OK;
}

/**
 * @brief httpGetResponseCallback is called once a response to the GET request is received.
 *
 * @param httpSession
 *               A pointer to the currently active HTTP session
 * @param httpMessage
 *               A pointer to the HTTP message we're sending/receiving a response for
 * @param status
 *               The return code of the HTTP page download
 *
 * @returns
 *               The return code of the HTTP connect
 *
 */
static retcode_t httpGetResponseCallback(HttpSession_T *httpSession, Msg_T *httpMessage, retcode_t status)
{
    BCDS_UNUSED(httpSession);

    /* No matter what we get as response, we want to make sure send another request! */
    xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);

    if (RC_OK != status)
    {
        printf("httpGetResponseCallback: error while receiving response to GET request. error=%d\r\n", status);
        return RC_OK;
    }
    if (NULL == httpMessage)
    {
        printf("httpGetResponseCallback: received NULL as HTTP message. This should not happen.\r\n");
        return RC_OK;
    }

    Http_StatusCode_T httpStatusCode = HttpMsg_getStatusCode(httpMessage);
    if (Http_StatusCode_OK != httpStatusCode)
    {
        printf("httpGetResponseCallback: received HTTP status other than 200 OK. status=%d\r\n", httpStatusCode);
    }
    else
    {
        retcode_t retcode;
        bool isLastPartOfMessage;
        uint32_t pageContentSize;
        retcode = HttpMsg_getRange(httpMessage, UINT32_C(0), &pageContentSize, &isLastPartOfMessage);
        if (RC_OK != retcode)
        {
            printf("httpGetResponseCallback: failed to get range from message. error=%d\r\n", retcode);
        }
        else
        {
            const char* responseContent;
            unsigned int responseContentLen;
            HttpMsg_getContent(httpMessage, &responseContent, &responseContentLen);
            printf("httpGetResponseCallback: successfully received a response: %.*s\r\n", responseContentLen, responseContent);

            if (isLastPartOfMessage)
            {
                /* We're done with the GET request. Let's make a POST request. */
                printf("httpGetResponseCallback: GET request is done. Triggering the POST request.\r\n");
                xTaskNotifyGive(httpPostTaskHandle);
            }
            else
            {
                /* We're not done yet downloading the page - let's make another request. */
                printf("httpGetResponseCallback: there is still more to GET. Making another request.\r\n");
                httpGetPageOffset += responseContentLen;
                xTaskNotifyGive(httpGetTaskHandle);
            }
        }
    }

    return RC_OK;
}

/**
 * @brief httpGetTask initializes and pushes HTTP GET requests when triggered through its
 *        notification. This task can trigger itself when required to download a page/response
 *        completely.
 */
static void httpGetTask(void* parameter)
{
    BCDS_UNUSED(parameter);
    retcode_t retcode;
    Retcode_T retVal;
    Msg_T* httpMessage;

    while (1)
    {
        /* We wait until it's our time to send the GET request */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Some forms of load-balancing yield a different IP address upon resolving the DNS name.
         * Thus, we should resolve the host name for each request. If we know that the service we're
         * making a request to does not exhibit such behavior, we can get the host IP address once
         * when the WLAN connection is established.
         */
        Ip_Address_T destServerAddress;
        retVal = NetworkConfig_GetIpAddress((uint8_t*) DEST_SERVER_HOST, &destServerAddress);
        if (RETCODE_OK != retVal)
        {
            printf("httpGetTask: unable to resolve hostname " DEST_SERVER_HOST " . error=%d.\r\n", retcode);
        }
        if (RETCODE_OK == retVal)
        {
#if  (HTTP_SECURE_ENABLE == 1)
            /*send Secure Request */
            retcode = HttpClient_initSecureRequest(&destServerAddress, Ip_convertIntToPort(DEST_SERVER_PORT_SECURE), &httpMessage);
#else
            /* send request */
            retcode = HttpClient_initRequest(&destServerAddress, Ip_convertIntToPort(DEST_SERVER_PORT), &httpMessage);
#endif
            if (RC_OK != retcode)
            {
                printf("httpGetTask: unable to create HTTP request. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INIT_REQUEST_FAILED);
            }
        }
        if (RETCODE_OK == retVal)
        {
            HttpMsg_setReqMethod(httpMessage, Http_Method_Get);

            HttpMsg_setContentType(httpMessage, Http_ContentType_Text_Plain);

            retcode = HttpMsg_setReqUrl(httpMessage, DEST_GET_PATH);
            if (RC_OK != retcode)
            {
                printf("httpGetTask: unable to set request URL. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SET_REQURL_FAILED);
            }
        }
        /* It's important that we set the HOST header as many services use this header for routing. Without
         * this header your HTTP request is likely to fail/result in an undesired response.
         */
        if (RETCODE_OK == retVal)
        {
            retcode = HttpMsg_setHost(httpMessage, DEST_SERVER_HOST);
            if (RC_OK != retcode)
            {
                printf("httpGetTask: unable to set HOST header. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SET_HOST_FAILED);
            }
        }
        /* We can only receive a certain amount of data per request. Thus we may have to make multiple
         * HTTP GET requests to download a page/response completely. We set the Range HTTP header to signal
         * the other end which part of the page we'd like to download.
         */
        if (RETCODE_OK == retVal)
        {
            HttpMsg_setRange(httpMessage, httpGetPageOffset, REQUEST_MAX_DOWNLOAD_SIZE);

            /* The retcode of pushRequest does not tell us if the message was indeed sent, but rather if the
             * push was initialized. To find out of the HTTP request was actually sent we have to check the
             * callerStatus in the httpGetSentCallback function.
             *
             * When we receive a response from the server, the httpGetResponseCallback function is called.
             */
            Callable_T httpRequestSentCallable;
            (void) Callable_assign(&httpRequestSentCallable, httpRequestSentCallback);
            retcode = HttpClient_pushRequest(httpMessage, &httpRequestSentCallable, httpGetResponseCallback);
            if (RC_OK != retcode)
            {
                printf("httpGetTask: unable to push the HTTP request. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_PUSH_REQUEST_FAILED);
            }
        }
        if (RETCODE_OK != retVal)
        {
            Retcode_RaiseError(retVal);
            xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);
        }
    }
}

/**
 * @brief httpPostCustomHeaderSerializer adds two custom header to an HTTP request: X-AuthToken and X-Foobar.
 *
 * @param serializationHandover the serialization context we're adding the header to.
 * @return
 */
static retcode_t httpPostCustomHeaderSerializer(OutMsgSerializationHandover_T* serializationHandover)
{
    if (serializationHandover == NULL)
    {
        printf("httpPostCustomHeaderSerializer: serializationHandover is NULL. This should never happen.\r\n");
        return RC_APP_ERROR;
    }

    retcode_t result = RC_OK;
    switch (serializationHandover->position)
    {
    case 0:
        result = TcpMsg_copyStaticContent(serializationHandover, POST_REQUEST_CUSTOM_HEADER_0, strlen(POST_REQUEST_CUSTOM_HEADER_0));
        if (result != RC_OK)
            return result;
        serializationHandover->position = 1;
        break;
    case 1:
        result = TcpMsg_copyContentAtomic(serializationHandover, POST_REQUEST_CUSTOM_HEADER_1, strlen(POST_REQUEST_CUSTOM_HEADER_1));
        if (result != RC_OK)
            return result;
        serializationHandover->position = 2;
        break;
    default:
        result = RC_OK;
    }
    return result;
}

/**
 * httpPostPayloadSerializer serialize the HTTP request body of the POST request we're sending.
 *
 * @param serializationHandover the structure to store the body in
 * @return indication whether we need more space or are done
 */
static retcode_t httpPostPayloadSerializer(OutMsgSerializationHandover_T* serializationHandover)
{
    /* This could also be a global variable which stores a previously set message. In this example we keep
     * it simple and use some constant message.
     */

	/*POST_BODY = "{ \"device\": \"XDK110\", \"ping\": \"pong\" }";*/
	bme280_getSensorValues(NULL);

	printf("INHALT IST : %s \n", POST_BODY);

	const char* httpBodyBuffer = POST_BODY;

    uint32_t offset = serializationHandover->offset;
    uint32_t bytesLeft = strlen(httpBodyBuffer) - offset;
    uint32_t bytesToCopy = serializationHandover->bufLen > bytesLeft ? bytesLeft : serializationHandover->bufLen;

    memcpy(serializationHandover->buf_ptr, httpBodyBuffer + offset, bytesToCopy);
    serializationHandover->len = bytesToCopy;

    if (bytesToCopy < bytesLeft)
    {
        return RC_MSG_FACTORY_INCOMPLETE;
    }
    else
    {
        return RC_OK;
    }
}

/**
 * @brief httpPostResponseCallback is called once a response to the POST request is received.
 *
 * @param httpSession
 *               A pointer to the currently active HTTP session
 * @param httpMessage
 *               A pointer to the HTTP message we're sending/receiving a response for
 * @param status
 *               The return code of the HTTP page download
 *
 * @returns
 *               The return code of the HTTP connect
 *
 */
static retcode_t httpPostResponseCallback(HttpSession_T *httpSession, Msg_T *httpMessage, retcode_t status)
{
    BCDS_UNUSED(httpSession);

    /* No matter what we get as response, we want to make sure send another request! */
    xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);

    if (RC_OK != status)
    {
        printf("httpPostResponseCallback: error while receiving response to POST request. error=%d\r\n", status);
        return RC_APP_ERROR;
    }
    if (NULL == httpMessage)
    {
        printf("httpPostResponseCallback: received NULL as HTTP message. This should not happen.\r\n");
        return RC_APP_ERROR;
    }

    Http_StatusCode_T httpStatusCode = HttpMsg_getStatusCode(httpMessage);
    if (Http_StatusCode_OK != httpStatusCode)
    {
        printf("httpPostResponseCallback: received HTTP status other than 200 OK. status=%d\r\n", httpStatusCode);
    }
    else
    {
        retcode_t retcode;
        bool isLastPartOfMessage;
        uint32_t pageContentSize;
        retcode = HttpMsg_getRange(httpMessage, UINT32_C(0), &pageContentSize, &isLastPartOfMessage);
        if (RC_OK != retcode)
        {
            printf("httpPostResponseCallback: failed to get range from message. error=%d\r\n", retcode);
        }
        else
        {
            const char* responseContent;
            unsigned int responseContentLen;
            HttpMsg_getContent(httpMessage, &responseContent, &responseContentLen);
            printf("httpPostResponseCallback: successfully received a response: %.*s\r\n", responseContentLen, responseContent);

            if (!isLastPartOfMessage)
            {
                /* We're not done yet downloading the page - let's make another request. */
                printf("httpPostResponseCallback: server response was too large. This example application does not support POST responses larger than %lu.\r\n", REQUEST_MAX_DOWNLOAD_SIZE);
            }

            printf("httpPostResponseCallback: POST request is done. Restarting request timer.\r\n");
            xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);
        }
    }

    return RC_OK;
}

/**
 * @brief httpPostTask initializes and pushes HTTP POST requests when triggered through its notification.
 */
static void httpPostTask(void* parameter)
{
    BCDS_UNUSED(parameter);
    retcode_t retcode;
    Retcode_T retVal;
    Msg_T* httpMessage;

    while (1)
    {
        /* We wait until it's our time to send the POST request */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Some forms of load-balancing yield a different IP address upon resolving the DNS name.
         * Thus, we should resolve the host name for each request. If we know that the service we're
         * making a request to does not exhibit such behavior, we can get the host IP address once
         * when the WLAN connection is established.
         */
        Ip_Address_T destServerAddress;
        retVal = NetworkConfig_GetIpAddress((uint8_t*) DEST_SERVER_HOST, &destServerAddress);
        if (RETCODE_OK != retVal)
        {
            printf("httpPostTask: unable to resolve hostname " DEST_SERVER_HOST ". error=%d.\r\n", (int) retVal);
        }
        if (RETCODE_OK == retVal)
        {
#if  (HTTP_SECURE_ENABLE == 1)
            /* send Secure request */
            retcode = HttpClient_initSecureRequest(&destServerAddress, Ip_convertIntToPort(DEST_SERVER_PORT_SECURE), &httpMessage);
#else
            /* send request */
            retcode = HttpClient_initRequest(&destServerAddress, Ip_convertIntToPort(DEST_SERVER_PORT), &httpMessage);
#endif
            if (RC_OK != retcode)
            {
                printf("httpPostTask: unable to create HTTP request. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INIT_REQUEST_FAILED);
            }
        }
        if (RETCODE_OK == retVal)
        {
            HttpMsg_setReqMethod(httpMessage, Http_Method_Post);

            HttpMsg_setContentType(httpMessage, Http_ContentType_App_Json);

            retcode = HttpMsg_setReqUrl(httpMessage, DEST_POST_PATH);
            if (RC_OK != retcode)
            {
                printf("httpPostTask: unable to set request URL. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SET_REQURL_FAILED);
            }
        }
        if (RETCODE_OK == retVal)
        {
            /* It's important that we set the HOST header as many services use this header for routing. Without
             * this header your HTTP request is likely to fail/result in an undesired response.
             */
            retcode = HttpMsg_setHost(httpMessage, DEST_SERVER_HOST);
            if (RC_OK != retcode)
            {
                printf("httpPostTask: unable to set HOST header. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_SET_HOST_FAILED);
            }
        }
        if (RETCODE_OK == retVal)
        {
            /* We can only receive a certain amount of data per request. For the sake of brevity we do not implement
             * multiple POST requests and expect the response to fit in REQUEST_MAX_DOWNLOAD_SIZE. See the HTTP GET
             * part for an example of how to implement the proper behavior.
             */
            HttpMsg_setRange(httpMessage, httpGetPageOffset, REQUEST_MAX_DOWNLOAD_SIZE);

            /*
             * If we want to send custom header, e.g. an authentication token, we have to provide those through a custom
             * header serialization function such as httpPostCustomHeaderSerializer.
             */
            HttpMsg_serializeCustomHeaders(httpMessage, httpPostCustomHeaderSerializer);

            /*
             * Providing the body of the HTTP request works similarly to custom headers: through a serializer. Please
             * have a closer look at its implementation to understand how it works with the buffers and serializes the
             * data one chunk at a time.
             */
            retcode = TcpMsg_prependPartFactory(httpMessage, httpPostPayloadSerializer);
            if (RC_OK != retcode)
            {
                printf("httpPostTask: unable to serialize request body. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_FAILURE);
            }
        }
        if (RETCODE_OK == retVal)
        {
            /* The retcode of pushRequest does not tell us if the message was indeed sent, but rather if the
             * push was initialized. To find out of the HTTP request was actually sent we have to check the
             * callerStatus in the httpRequestSentCallback function.
             *
             * When we receive a response from the server, the httpPostResponseCallback function is called.
             */

            Callable_T httpRequestSentCallable;
            (void) Callable_assign(&httpRequestSentCallable, httpRequestSentCallback);
            retcode = HttpClient_pushRequest(httpMessage, &httpRequestSentCallable, httpPostResponseCallback);
            if (RC_OK != retcode)
            {
                printf("httpPostTask: unable to push the HTTP request. error=%d.\r\n", retcode);
                retVal = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_PUSH_REQUEST_FAILED);
            }
        }

        if (RETCODE_OK != retVal)
        {
            Retcode_RaiseError(retVal);
            xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL / portTICK_PERIOD_MS);
        }
    }
}

/**
 * @brief triggerHttpRequestTimerCallback is called by the triggerHttpRequestTimer when it's time
 *        to trigger a new HTTP request.
 *
 * @param timer the timer which triggered this callback.
 */
static void triggerHttpRequestTimerCallback(TimerHandle_t timer)
{
    BCDS_UNUSED(timer);

    httpGetPageOffset = 0;
    xTaskNotifyGive(httpGetTaskHandle);
}

/* global functions ********************************************************** */

/**
 * appInitSystem connects to the network, initializes the networking subsystem and
 * sets up the application tasks and timer.
 *
 * This function is executed in a FreeRTOS task context by the mainCommandProcessor
 * started in the main() function.
 */
void appInitSystem(void* cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(cmdProcessorHandle);
    BCDS_UNUSED(param2);
    Retcode_T returnValue = RETCODE_OK;

    /* 1. Step: connect to WLAN and initialize networking subsystem.
     *     When writing your own application, make sure you initialize the "platform abstraction layer" (PAL)
     *     and HTTP client, in that order, after a connection to a network was established.
     */
    retcode_t rc = RC_OK;
    rc = connectToWLAN();
    if (RC_OK != rc)
    {
        printf("appInitSystem: network init/connection failed. error=%d\r\n", rc);
        return;
    }

    printf("ServalPal Setup\r\n");
    returnValue = ServalPalSetup();
    if (RETCODE_OK != returnValue)
    {
        Retcode_RaiseError(returnValue);
        printf("ServalPal Setup failed with %d \r\n", (int) returnValue);
        return;
    }

#if  (HTTP_SECURE_ENABLE == 1)
    /* Security Initialization with the Certificate  */
    Security_setCallback(httpsSecurityCallback);
    returnValue = initializeCertificates();
    if (RETCODE_OK != returnValue)
    {
        printf("Failed to initialize http security \r\n");
        Retcode_RaiseError(returnValue);
    }
#endif
    /* start client */
    rc = HttpClient_initialize();
    if (RC_OK != rc)
    {
        printf("Failed to initialize http client \r\n ");
        return;
    }
    /* 2. Step: setup application level tasks and timers.
     *     In this demo we have two tasks running (one for sending GET requests, one for POST requests).
     *     Those tasks are periodically triggered to send out their requests using a timer and the FreeRTOS task notification.
     */
    BaseType_t taskCreated;
    taskCreated = xTaskCreate(httpGetTask, "GET task", TASK_STACK_SIZE_HTTP_REQ, NULL, TASK_PRIO_HTTP_REQ, &httpGetTaskHandle);
    if (taskCreated != pdTRUE)
    {
        printf("Failed to create the GET request task\r\n");
        return;
    }

    taskCreated = xTaskCreate(httpPostTask, "POST task", TASK_STACK_SIZE_HTTP_REQ, NULL, TASK_PRIO_HTTP_REQ, &httpPostTaskHandle);
    if (taskCreated != pdTRUE)
    {
        printf("Failed to create the POST request task\r\n");
        return;
    }

    triggerHttpRequestTimerHandle = xTimerCreate("triggerRequestTimer", INTER_REQUEST_INTERVAL / portTICK_RATE_MS, pdFALSE, NULL, triggerHttpRequestTimerCallback);
    if (triggerHttpRequestTimerHandle == NULL)
    {
        printf("Failed to create the triggerRequestTimer\r\n");
        return;
    }
    BaseType_t requestTimerStarted = xTimerStart(triggerHttpRequestTimerHandle, INTER_REQUEST_INTERVAL);
    if (requestTimerStarted == pdFALSE)
    {
        printf("Failed to start the triggerRequestTimer\r\n");
    }

    printf("Connected to network. All tasks and timer started. First HTTP request will be made soon.\r\n");

    //starting the sensors
    bme_280_init();
}

/**@} */
