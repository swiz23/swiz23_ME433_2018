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
#include<stdio.h>
#include "i2c_master_noint.h"
#include "ST7735.h"

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


void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    // do your TRIS and LAT commands here
//    __builtin_disable_interrupts();
//
//    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
//    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
//
//    // 0 data RAM access wait states
//    BMXCONbits.BMXWSDRM = 0x0;
//
//    // enable multi vector interrupts
//    INTCONbits.MVEC = 0x1;
//
//    // disable JTAG to get pins back
//    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
  
    initIMU();
    LCD_init();
    LCD_clearScreen(BLUE);
    
//    __builtin_enable_interrupts();
    
    draw_plain(63,96,5,60,WHITE);
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            static char message[30];
            static unsigned char dat[15]; // initialize array to hold bytes from IMU
            static signed short final_data[8];
            static int cntr;
            _CP0_SET_COUNT(0);

            if (getIMU() == 0x69) {
                LATAINV = 0x10; // invert the fourth bit
                sprintf(message,"Who Am I: %d",getIMU());
                draw_string(5,5, message, WHITE, BLUE);
            }

            I2C_read_multiple(0x20,dat,14);
            for (cntr = 0; cntr < 14; cntr+=2) {
                final_data[cntr/2] = dat[cntr] | dat[cntr+1] << 8;
            }

            draw_axes(63,96,5,60,MAGENTA,final_data[4],final_data[5],WHITE);

            sprintf(message,"Accel X: %d  ",final_data[4]);
            draw_string(5,15, message, WHITE, BLUE);

            sprintf(message,"Accel Y: %d  ",final_data[5]);
            draw_string(5,25, message, WHITE, BLUE);

            // to get the LED to blink at 20Hz, we need a delay of 
            // 48000000/2* = 6000000 ticks
            while (_CP0_GET_COUNT() < 600000){
                ;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

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
