/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here
    double k = 0.15, kl = .03;
    int error = rxVal - 319;
 
  // PI for left motor
    double e1 = 0, u1 = 0;
    static double eint1 = 0;
    static double eprev1 = 0, ed1 = 0;
    double kp1 = 1.4,ki1 = 0, kd1 = 1;
    int lc = TMR3;
    e1 = (100*k*((319-.5*abs(error))/319.0) - lc);
    eint1 = eint1 + e1;
    ed1 = e1 - eprev1;
    if (eint1 > 100){
        eint1 = 100;
    }
    u1 = kp1*e1 + ki1*eint1+kd1*ed1;
    if (u1 > 100){
        u1 = 100;
    }
    if (error < 0 ) {
        u1 = u1 - kl*(-error);
    }
     if (u1 < 0) {
        u1 = 0;
    }
    OC4RS = (int)(u1/100.0*PR2);
    eprev1 = e1;
  // PI for right motor
    double e2 = 0, u2 = 0;
    static double eint2 = 0;
    static double eprev2 = 0, ed2 = 0;
    double kp2 = 1.4,ki2 = 0, kd2 = 1;
    int rc = TMR5;
    e2 = (100*k*((319-.5*abs(error))/319.0) - rc);
    eint2 = eint2 + e2;
    ed2 = e2 - eprev2;
    if (eint2 > 100){
        eint2 = 100;
    }
    u2 = kp2*e2 + ki2*eint2 + kd2*ed2;
    if (u2 > 100){
        u2 = 100;
    }  
    
    if (error > 0) {
        u2 = u2 - kl*error;
    }
    if (u2 < 0) {
        u2 = 0;
    }
    
    OC1RS = (int)(u2/100.0*PR2);
    eprev2 = e2;
    TMR3 = 0;
    TMR5 = 0;
    
  IFS0bits.T4IF = 0; // clear interrupt flag, last line
}
/*******************************************************************************
 End of File
*/
