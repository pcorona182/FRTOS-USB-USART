/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _GENERAL    /* Guard against multiple inclusion */
#define _GENERAL


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "system_config.h"
#include "system_definitions.h"

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
#define EXAMPLE_CONSTANT 0


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */
//    typedef struct _example_struct_t {
//        /* Describe structure member. */
//        int some_number;
//
//        /* Describe structure member. */
//        bool some_flag;
//
//    } example_struct_t;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    void Led5_Toggle(void){
    LATBbits.LATB2 =~ LATBbits.LATB2;
    }
    
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
     void DelayMs(long int delay)
    {
        
    }
    // *****************************************************************************
    /**
      @Function
        void UART_Put(DRV_HANDLE localUSARTHandle,char tx_byte)

      @Summary
        Envia  por el UART Seleccionado en localUSARTHandle el valor de tx_byte

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param localUSARTHandle UART Selector.
    
        @param tx_byte Data.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    void UART_Put(DRV_HANDLE localUSARTHandle,char tx_byte){
        // make sure the transmit buffer is not full before trying to write byte 
        if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(localUSARTHandle)) )
        {
            DRV_USART_WriteByte(localUSARTHandle, tx_byte);  // send modified byte
        }
    }
    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    char UART_Get(DRV_HANDLE localUSARTHandle){
        char rx_byte;       // byte received
        // if byte received in USART instance pointed to by myUSARTHandle (USART1 in this case)
        if (!DRV_USART_ReceiverBufferIsEmpty(localUSARTHandle))
        {
           rx_byte = DRV_USART_ReadByte(localUSARTHandle); // read received byte
           return rx_byte;
        }else{
            return 0;
        }
    }
    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    void UART_Printf(DRV_HANDLE localUSARTHandle, char *tx_bytes){
        size_t tx_bytes_len = strlen(tx_bytes);
        if(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(localUSARTHandle)) )
        {
            DRV_USART_Write(localUSARTHandle,tx_bytes,tx_bytes_len);  // send modified byte
        }
    }
    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    size_t UART_Scanf(DRV_HANDLE localUSARTHandle,char *rx_bytes){
        size_t rx_bytes_len = strlen(rx_bytes);
        // if byte received in USART instance pointed to by myUSARTHandle (USART1 in this case)
        if (!DRV_USART_ReceiverBufferIsEmpty(localUSARTHandle))
        {
           return DRV_USART_Read(localUSARTHandle,rx_bytes,rx_bytes_len);// read received byte

        }else{
            return 0;
        }
    }
    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    USB_DEVICE_CDC_RESULT USB_Printf(USB_DEVICE_CDC_TRANSFER_HANDLE transferHandle, uint8_t * Buffer){
        USB_DEVICE_CDC_RESULT readRequestResult = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        uint32_t numBytesWrite = strlen(Buffer);

        readRequestResult = USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &transferHandle,
                            Buffer, numBytesWrite,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);    
        return readRequestResult;
    }
    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    USB_DEVICE_CDC_RESULT USB_Scanf(USB_DEVICE_CDC_TRANSFER_HANDLE transferHandle, uint8_t * Buffer){
        USB_DEVICE_CDC_RESULT readRequestResult = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        size_t numBytesRead = 128;
        //strcat(Buffer, "\0");
        readRequestResult =  USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                            &transferHandle, Buffer,
                            numBytesRead);
        return readRequestResult;
//        if(USB_DEVICE_CDC_RESULT_OK != readRequestResult) {
//            return 0;
//        }else{
//            return 1;
//        }

    }

//switch(USBDeviceTask_Event)
//                {
//                    case USBDEVICETASK_USBPOWERED_EVENT:
//                        USB_DEVICE_Attach (appData.deviceHandle);
//                        break;
//                    case USBDEVICETASK_USBCONFIGURED_EVENT:
//                        /*USB ready, wait for user input on either com port*/
//
//                        /* Schedule a CDC read on COM1 */
//                        USB_DEVICE_CDC_Read(0, &COM1Read_Handle,
//                            com1ReadBuffer,APP_READ_BUFFER_SIZE);
//                        /* Schedule a CDC read on COM2 */
//                        USB_DEVICE_CDC_Read(1,&COM2Read_Handle,
//                            com2ReadBuffer,APP_READ_BUFFER_SIZE);
//                        break;                    
//                    case USBDEVICETASK_READDONECOM1_EVENT:
//                        /* Send the received data to COM2 */
//                        USB_DEVICE_CDC_Write(1, &COM2Write_Handle,
//                            &com1ReadBuffer, 1,
//                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//                        break;
//                    case USBDEVICETASK_READDONECOM2_EVENT:
//                        /* Send the received data to COM1 */
//                        USB_DEVICE_CDC_Write(0, &COM1Write_Handle,
//                            &com2ReadBuffer, 1,
//                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//                        break;
//                    case USBDEVICETASK_WRITEDONECOM1_EVENT:
//                        /* Schedule a CDC read on COM2 */
//                        USB_DEVICE_CDC_Read(1,&COM2Read_Handle,
//                            com2ReadBuffer,APP_READ_BUFFER_SIZE);
//                        break;
//                    case USBDEVICETASK_WRITEDONECOM2_EVENT:
//                        /* Schedule a CDC read on COM1 */
//                        USB_DEVICE_CDC_Read(0, &COM1Read_Handle,
//                            com1ReadBuffer,APP_READ_BUFFER_SIZE);
//                        break;
//                    default:
//                        break;
//                }


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
