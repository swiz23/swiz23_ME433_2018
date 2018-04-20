#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define ADDR 0b0100000

void initExpander(void);
void setExpander(unsigned char pin, unsigned char level);
unsigned char getExpander(void);

int main() {

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
    TRISBbits.TRISB4 = 1;
    
    initExpander();
    
    __builtin_enable_interrupts();

    while(1) {
        _CP0_SET_COUNT(0);
    // to get the LED to blink at 5Hz, we need a delay of 
    // 48000000/2*.1 = 2400000 ticks
        while (_CP0_GET_COUNT() < 2400000){
            ;
        }
        LATAINV = 0x10; // invert the fourth bit
       
        if (getExpander() >= 0b10000000){
            setExpander(0x0A,0b00000000);
        } else {
            setExpander(0x0A,0b00000001);
        }
    }
}

void initExpander(){
    ANSELBbits.ANSB2 = 0; // make Pin 6: SDA2 pin on PIC not an analog pin
    ANSELBbits.ANSB3 = 0; // make Pin 7: SCL2 pin on PIC not an analog pin
    i2c_master_setup();
    setExpander(0x00,0b11110000);
    setExpander(0x0A,0b00000000);
}

void setExpander(unsigned char pin, unsigned char level){
    i2c_master_start(); // start bit
    i2c_master_send(ADDR<<1|0); //signify writing
    i2c_master_send(pin);
    i2c_master_send(level);
    i2c_master_stop();
}

unsigned char getExpander() {
    i2c_master_start(); // start bit
    i2c_master_send(ADDR<<1|0); //signify writing
    i2c_master_send(0x09);
    i2c_master_restart();
    i2c_master_send(ADDR<<1|1);
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}