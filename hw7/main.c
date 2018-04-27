#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
#include "i2c_master_noint.h"
#include "ST7735.h"

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

#define ADDR 0b11010110 // default is writing mode

void initIMU(void);
void setIMU(unsigned char pin, unsigned char level);
void I2C_read_multiple(unsigned char reg, unsigned char *data, int length);
unsigned char getIMU(void);
void draw_string(short x, short y, char* message, short c1, short c2);
void draw_char(short x, short y, char mess, short c1, short c2);
void draw_progress_bar(short x, short y, short h, short len1, short c1, short len2, short c2);

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
  
    initIMU();
    LCD_init();
    LCD_clearScreen(BLUE);
    
    char dat[15]; // initialize array to hold bytes from IMU
    signed short final_data[8];
    int cntr;
    
    char message[30];
    int cnt = 0;
    double fps = 0;
    __builtin_enable_interrupts();

    while(1) {
        _CP0_SET_COUNT(0);
        
        if (getIMU() == 0x69) {
            LATAINV = 0x10; // invert the fourth bit
        }
        
        I2C_read_multiple(0x20,dat,14);
        for (cntr = 0; cntr < 14; cntr+=2) {
            final_data[cntr] = dat[cntr] | dat[cntr+1] << 8;
        }
        
        sprintf(message,"Hello World %d  ",cnt);
        draw_string(28,32, message, WHITE, BLUE);
        draw_progress_bar(14,50,5,100,MAGENTA,cnt,WHITE);
        cnt++;
        if (cnt > 100) {
            cnt = 0;
        }
        fps = 24000000.0/_CP0_GET_COUNT();
        sprintf(message,"FPS = %.2f ",fps);
        draw_string(28,100, message, WHITE, BLUE);
        
        // to get the LED to blink at 20Hz, we need a delay of 
        // 48000000/2* = 6000000 ticks
        while (_CP0_GET_COUNT() < 600000){
            ;
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

void draw_progress_bar(short x, short y, short h, short len1, short c1, short len2, short c2) {
    int i,j;
    for (i=0;i<len1;i++) {
        for (j=0;j<h;j++){
            if (i < len2){
                LCD_drawPixel(x+i,y+j,c1);
            } else {
                LCD_drawPixel(x+i,y+j,c2);
            }
        }
    }
}