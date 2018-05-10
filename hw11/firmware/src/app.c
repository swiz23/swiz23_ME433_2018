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

#include "app.h"
#include "ST7735.h"
#include "i2c_master_noint.h"
#include<stdio.h>

#define ADDR 0b11010110 // default is writing mode
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

APP_DATA appData;

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData) {
    APP_DATA * appData = (APP_DATA *) userData;

    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *) eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge
              back with a Zero Length packet. The HID function driver returns
              an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
              application upon receiving this Zero Length packet from Host.
              USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
              this control transfer event is complete */
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)

  Summary:
    Event callback generated by USB device layer.

  Description:
    This event handler will handle all device layer events.

  Parameters:
    None.

  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch (event) {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device got deconfigured */

            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //appData.emulateMouse = true;
            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *) eventData;
            if (configurationValue->configurationValue == 1) {
                appData.isConfigured = true;

                //BSP_LEDOff ( APP_USB_LED_1 );
                //BSP_LEDOff ( APP_USB_LED_2 );
                //BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);
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
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
}

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    //appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;
    
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1; 
    initIMU();
    LCD_init();
    LCD_clearScreen(BLACK);    
    static char message[30];
    sprintf(message,"Who Am I: ");
    draw_string(5,5, message, WHITE, BLACK);     
    sprintf(message,"AX: ");
    draw_string(5,15, message, WHITE, BLACK);
    sprintf(message,"AY: ");
    draw_string(5,25, message, WHITE, BLACK);  
//    draw_plain(63,96,5,60,WHITE);    
  
    __builtin_enable_interrupts();    
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    static uint8_t inc = 0;
    static unsigned char dat[15]; // initialize array to hold bytes from IMU
    static signed short final_data[8];
    static int cntr, xPos, yPos;
    static char message[30];   
    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if (appData.isConfigured) {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE:
         
            
            if (getIMU() == 0x69) {
                LATAINV = 0x10; // invert the fourth bit
                sprintf(message,"%d",getIMU());
                draw_string(55,5, message, WHITE, BLACK);                    
            } else {
                sprintf(message,"Error");
                draw_string(55,5, message, WHITE, BLACK);
            }                           
            I2C_read_multiple(0x20,dat,14);
            for (cntr = 0; cntr < 14; cntr+=2) {
                final_data[cntr/2] = dat[cntr] | dat[cntr+1] << 8;
            }            
            
//            draw_axes(63,96,5,60,MAGENTA,final_data[4],final_data[5],WHITE);

            sprintf(message,"%d ",final_data[4]);
            draw_string(25,15, message, WHITE, BLACK);

            sprintf(message,"%d ",final_data[5]);
            draw_string(25,25, message, WHITE, BLACK); 
            
            // mouse stuff
            
            appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
            appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED; 
            
            xPos = final_data[4]/1000;
            yPos = final_data[5]/1000;
            if (inc == 2) {
            appData.xCoordinate = (int8_t) xPos;
            appData.yCoordinate = (int8_t) yPos;
            inc = 0;
            } else {
                appData.xCoordinate = (int8_t) 0;
                appData.yCoordinate = (int8_t) 0; 
            }   
            if (!appData.isMouseReportSendBusy) {
                /* This means we can send the mouse report. The
                   isMouseReportBusy flag is updated in the HID Event Handler. */

                appData.isMouseReportSendBusy = true;

                /* Create the mouse report */

                MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                        appData.mouseButton, &mouseReport);

                if (memcmp((const void *) &mouseReportPrevious, (const void *) &mouseReport,
                        (size_t)sizeof (mouseReport)) == 0) {
                    /* Reports are same as previous report. However mouse reports
                     * can be same as previous report as the coordinate positions are relative.
                     * In that case it needs to be sent */
                    if ((appData.xCoordinate == 0) && (appData.yCoordinate == 0)) {
                        /* If the coordinate positions are 0, that means there
                         * is no relative change */
                        if (appData.idleRate == 0) {
                            appData.isMouseReportSendBusy = false;
                        } else {
                            /* Check the idle rate here. If idle rate time elapsed
                             * then the data will be sent. Idle rate resolution is
                             * 4 msec as per HID specification; possible range is
                             * between 4msec >= idlerate <= 1020 msec.
                             */
                            if (appData.setIdleTimer
                                    >= appData.idleRate * 4) {
                                /* Send REPORT as idle time has elapsed */
                                appData.isMouseReportSendBusy = true;
                            } else {
                                /* Do not send REPORT as idle time has not elapsed */
                                appData.isMouseReportSendBusy = false;
                            }
                        }
                    }

                }
                if (appData.isMouseReportSendBusy == true) {
                    /* Copy the report sent to previous */
                    memcpy((void *) &mouseReportPrevious, (const void *) &mouseReport,
                            (size_t)sizeof (mouseReport));

                    /* Send the mouse report. */
                    USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*) & mouseReport,
                            sizeof (MOUSE_REPORT));
                    appData.setIdleTimer = 0;
                }
                inc++;
            }

            break;

        case APP_STATE_ERROR:

            break;

            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void initIMU(){
    ANSELBbits.ANSB2 = 0; // make Pin 6: SDA2 pin on PIC not an analog pin
    ANSELBbits.ANSB3 = 0; // make Pin 7: SCL2 pin on PIC not an analog pin
    i2c_master_setup();
    setIMU(0x10,0b10000010); //linear acceleration
    setIMU(0x11,0b10001000); // angular acceleration
}

void setIMU(unsigned char pin, unsigned char level){
    i2c_master_start(); // start bit
    i2c_master_send(ADDR); //signify writing
    i2c_master_send(pin);
    i2c_master_send(level);
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char reg, unsigned char *data, int length) {
    int i;
    i2c_master_start(); // start bit
    i2c_master_send(ADDR); //signify writing
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(ADDR|1); //signify reading
    for (i = 0; i<length; i++) {
        data[i] = i2c_master_recv();
        if (i == (length-1)) {
            i2c_master_ack(1);           
        } else {
             i2c_master_ack(0);
        }
    }
    i2c_master_stop(); 
}

unsigned char getIMU() {
    i2c_master_start(); // start bit
    i2c_master_send(ADDR); //signify writing
    i2c_master_send(0x0F);
    i2c_master_restart();
    i2c_master_send(ADDR|1); //signify reading
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}

void draw_string(short x, short y, char* message, short c1, short c2) {
    int i = 0;
    while(message[i]) {
        draw_char(x+5*i, y, message[i], c1, c2);
        i++;
    }
}

void draw_char(short x, short y, char mess, short c1, short c2) {
    int col;
    char row = mess - 0x20;
    for (col=0; col<5; col++) {
        char pixels = ASCII[row][col];
        int i;
        for(i = 0; i < 8; i++) {
            if ((x+col) < 128 && (y+i) < 160) {
                if ((pixels >>i)&1 == 1) {
                    LCD_drawPixel(x+col,y+i,c1);
                } else {
                    LCD_drawPixel(x+col,y+i,c2);
                }
            }
        }
    }
}

void draw_plain(short x, short y, short h, short len, short c) {
    int i,j;

    for (i=0;i<len;i++) {
        for (j=0;j<h;j++){
            LCD_drawPixel(x-i,y+j-2,c);
            LCD_drawPixel(x+i,y+j-2,c);
        }
    }
    
    for (i=0;i<h;i++) {
        for (j=0;j<len;j++){
            LCD_drawPixel(x+i-2,y-j,c);
            LCD_drawPixel(x+i-2,y+j,c);
        }
    }      
}

void draw_axes(short x, short y, short h, short len, short c1, signed short accX, signed short accY, short c2) {
    short i,j;
    signed short sgnX = sign(accX);
    signed short sgnY = sign(accY);
    accX = (signed short) (accX/16000.0*len);
    accY = (signed short) (accY/16000.0*len);
    
    for (i=0;i<len;i++) {
        for (j=0;j<h;j++){
            if (i < abs(accX)){
                LCD_drawPixel(x-i*sgnX,y+j-2,c1);
            } else {
                LCD_drawPixel(x-i,y+j-2,c2);
                LCD_drawPixel(x+i,y+j-2,c2);
            }
        }
    }
    
    for (i=0;i<h;i++) {
        for (j=0;j<len;j++){
            if (j < abs(accY)){
                LCD_drawPixel(x+i-2,y-j*sgnY,c1);
            } else {
                LCD_drawPixel(x+i-2,y+j,c2);
                LCD_drawPixel(x+i-2,y-j,c2);
            }
        }
    }     
}

signed short sign(signed short val) {
    return (val>0) - (val<0);
}
/*******************************************************************************
 End of File
 */