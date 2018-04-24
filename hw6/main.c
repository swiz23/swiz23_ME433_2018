#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>
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
//    TRISAbits.TRISA4 = 0;
//    LATAbits.LATA4 = 1;
//    TRISBbits.TRISB4 = 1;
    LCD_init();
    LCD_clearScreen(BLUE);
    char message[30];
    int cnt = 0;
    double fps = 0;
    __builtin_enable_interrupts();

    while(1) {
        _CP0_SET_COUNT(0);
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
        
    // to get the program to run at 10Hz we need a delay of 
    // 48000000/2*.1 = 2400000 ticks
        while (_CP0_GET_COUNT() < 2400000){
            ;
        } 
    }
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

// Since the screen is 128 pixels wide and each character takes up 5 pixels, a 
// max of 25 characters can be printed on the screen at the same time (at least
// in one row). From the fps, it takes about 1/43 of a second to display the
// 'hello world' characters and the progress bar.