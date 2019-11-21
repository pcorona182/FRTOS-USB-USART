/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include "system_config/PIC32MX270F256B/FreeRTOSConfig.h"
#include "app.h"
#include "queue.h"
#include "general.h"
#include "peripheral/adcp/plib_adcp.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_RATE_MS constant. */
#define QUEUE_SEND_FREQUENCY_MS         ( 200 / portTICK_PERIOD_MS )
/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define QUEUE_LENGTH                    ( 1 )
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/
#define APP_MAKE_BUFFER_DMA_READY 
#define APP_READ_BUFFER_SIZE 8
uint8_t APP_MAKE_BUFFER_DMA_READY DefaultResponse[] = "\r\nReady";
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];




APP_DATA appData;
DRV_HANDLE myUSARTHandle;
QueueHandle_t USBDeviceTask_EventQueue_Handle;
/* The queue used by both tasks. */

const unsigned long ulValueToSend1 = 100UL;
const unsigned long ulValueToSend2 = 1000UL;

//void USBDevice_Task(void* p_arg);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler (USB_DEVICE_CDC_INDEX index , USB_DEVICE_CDC_EVENT event , void* pData, uintptr_t userData)
{
    APP_DATA* appDataObject;
    appDataObject = (APP_DATA*) userData;
    USB_CDC_CONTROL_LINE_STATE* controlLineStateData;

    switch ( event )
        {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle, &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle, &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            {
            /* This means that the host has sent some data*/
            USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE* p =  pData;
            appData.readBufferCharCount = p->length;

            appDataObject->isReadComplete = true;
            }
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
        }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
* app USB Device Layer Event Handler.
***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void* eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED* configuredEventData;

    switch ( event )
        {
        case USB_DEVICE_EVENT_RESET:

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;

            if ( configuredEventData->configurationValue == 1)
                {
                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;
                }

            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);

            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);

            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:

            break;

        default:
            break;
        }
}
/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.isConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

//void USBDevice_Task(void* p_arg)
void USBDevice_Task(void)
{
    
    
    BaseType_t errStatus;
    USB_DEVICE_CDC_RESULT readRequestResult;
    uint32_t USBDeviceTask_Event = 0;
    USB_DEVICE_CDC_TRANSFER_HANDLE COM1Read_Handle, COM1Write_Handle;
    COM1Read_Handle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    COM1Write_Handle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    switch ( appData.state )
    {
    /* Application's initial state. */
        case APP_STATE_INIT:
            {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
                {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
                break;
                }
            
            vTaskDelay(10 / portTICK_PERIOD_MS);
            break;
            }

        case APP_STATE_WAIT_FOR_CONFIGURATION:
            {
                USB_DEVICE_Attach (appData.deviceHandle);
                /* Check if the device was configured */
                if(appData.isConfigured)
                    {
                    /* If the device is configured then lets start reading */
                    appData.state = APP_STATE_LOOPFOREVER;
                    } 
            break;
            }

        case APP_STATE_LOOPFOREVER:
            {   
                /*wait for an event to occur and process, see event handler*/
//                errStatus = xQueueReceive(appData.deviceHandle,
//                                &USBDeviceTask_Event,portMAX_DELAY);

                if(APP_StateReset())
                {   USB_DEVICE_Attach (appData.deviceHandle);
                    break;
                }
                //readRequestResult = USB_Printf(appData.readTransferHandle,"Vamos Cu\n\r");
                if(appData.isReadComplete == true){
                    readRequestResult = USB_Scanf(appData.writeTransferHandle,appData.readBuffer);
                    if(USB_DEVICE_CDC_RESULT_OK != readRequestResult) {
                        //appData.state = APP_STATE_ERROR;
                    }
                    //appData.isWriteComplete = true;
                    //vTaskDelay(100 / portTICK_PERIOD_MS);
                    appData.writeBuffer = appData.readBuffer;
                    readRequestResult = USB_Printf(appData.readTransferHandle,appData.writeBuffer);
//                    if(USB_DEVICE_CDC_RESULT_OK != readRequestResult) {
//                        //appData.state = APP_STATE_ERROR;
//                    }
                    Led5_Toggle();
                    appData.isReadComplete = false;
                }
        

                
            break;
            }
        case APP_STATE_ERROR:
            USB_DEVICE_Close(appData.deviceHandle);
            appData.state = APP_STATE_INIT;
            break;
       /* The default state should never be executed. */
        default:
            {
            /* TODO: Handle error in application's state machine. */
            break;
            }
    }
}
    

void APP_Initialize ( void )
{
   /* Create the queue. */
   //xQueue = xQueueCreate( QUEUE_LENGTH , sizeof( unsigned long ) );
   
    USBDeviceTask_EventQueue_Handle = xQueueCreate(15, sizeof(uint32_t));

    /*dont proceed if queue was not created...*/
    if(USBDeviceTask_EventQueue_Handle == NULL){
        while(true){
                Led5_Toggle();
            }
    }
        
    
   appData.state = APP_STATE_INIT;

    /* Place the App state machine in its initial state. */
    //appData.state = appData.state;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 115200;
    appData.getLineCodingData.bParityType =  0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Initialize the read complete flag */
    appData.isReadComplete = true;
    appData.bufferHasBeenReadByClient = true;
    appData.readBufferCharCount = 0;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];
    myUSARTHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
}




/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

//   /* Send to the queue - causing the queue receive task2 to unblock and
//   toggle the LED.  0 is used as the block time so the sending operation
//   will not block - it shouldn't need to block as the queue should always
//   be empty at this point in the code. */
//   xQueueSend( xQueue, &ulValueToSend1, 0U );
//   /* Send to the queue - causing the queue receive task1 to unblock and
//   toggle the LED.  0 is used as the block time so the sending operation
//   will not block - it shouldn't need to block as the queue should always
//   be empty at this point in the code. */
//   xQueueSend( xQueue, &ulValueToSend2, 0U );
//
//   /* Place this task in the blocked state until it is time to run again.
//	The block time is specified in ticks, the constant used converts ticks
//   to ms.  While in the Blocked state this task will not consume any CPU
//   time. */
//   vTaskDelay(QUEUE_SEND_FREQUENCY_MS );

USBDevice_Task();
// xTaskCreate((TaskFunction_t) USBDevice_Task,
//                "USB_AttachTask",
//                1024, NULL, 1, NULL);
    
//    static bool blockAppTask = false;
//    BaseType_t errStatus;
//    if (blockAppTask == false)
//    {
//        errStatus = xTaskCreate((TaskFunction_t) USBDevice_Task,
//                "USB_AttachTask",
//                USBDEVICETASK_SIZE,
//                NULL,
//                USBDEVICETASK_PRIO,
//                NULL);
//        /*Don't proceed if Task was not created...*/
//        if(errStatus != pdTRUE)
//        {
//            while(true){
//                Led5_Toggle();
//            }
//        }

        /* The APP_Tasks() function need to exceute only once. Block it now */
        //blockAppTask = true;
//    }     

    
}
 

/*******************************************************************************
 End of File
 */
