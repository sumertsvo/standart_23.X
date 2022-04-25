/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC16F1823
        Driver Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

#include "mcc_generated_files/mcc.h"

#define _XTAL_FREQ 4000000

struct f_field {
    unsigned ALARM : 1;
    unsigned WORK_MODE : 1;
    unsigned FUN_OLD : 1;
    unsigned FUN_NEW : 1;
    unsigned JUMP : 1;
    unsigned MEAS : 1;
    unsigned FREE1 : 1;
    unsigned FREE2 : 1;
};

union Byte {
    unsigned char value;
    struct f_field bits;
} FLAGS;

const long int ROTATION_TIME = 10; //= 1209600;
const long int BAD_VALUE = 327677; //= 1209600;


char zumm;
char ledd;
char watt;
char rcon;

long int time_s;

void start_tone() {
    zumm = 1;
    TMR2_StartTimer();
    return;
}

void stop_tone() {
    zumm = 0;
    TMR2_StopTimer();
    return;
}

void go_close() {

    watt = 0;

    PIN_RELE_CONTROL_SetHigh();
    __delay_ms(20);
    PIN_RELE_POWER_SetHigh();
    for (char i = 0; i < 120; i++) {
        PIN_LED_Toggle();
        __delay_ms(10);
    }
    PIN_RELE_POWER_SetLow();
    __delay_ms(20);
    PIN_RELE_CONTROL_SetLow();
    return;
}

void go_close_alt() {

    watt = 0;
    FLAGS.bits.FUN_OLD = 0;

    PIN_RELE_POWER_SetHigh();
}

void go_open() {

    watt = 1;
    FLAGS.bits.FUN_OLD = 1;

    time_s = 0;
    PIN_RELE_POWER_SetHigh();
    __delay_ms(10);
    PIN_RELE_POWER_SetLow();
    return;
}

void go_open_alt() {

    watt = 1;

    PIN_RELE_POWER_SetLow();

    return;
}

void start_measure() {
    static char measures;
    unsigned res = ADC_GetConversion(PIN_WSP_STATE);
    if (res > BAD_VALUE) measures++;
    else measures = 0;
    if (measures > 2) FLAGS.bits.ALARM = 1;
    return;
}

void Sec_tick_work() {

    //  start_measure();

    time_s++;
    if (FLAGS.bits.ALARM) {//if alarm
        PIN_LED_Toggle();
        if (zumm) {//timer4switch
            zumm=1;
        } else {//timer4switch
            zumm=0;
        }
    } else {//if not alarm
        PIN_LED_Toggle();
        /*
          static char iled;
          iled++;
          if (iled > 20) {
              PIN_LED_Toggle();
              iled = 0;
         }*/
    }
    //   TMR2IF = 0;
    return;
}

void povorot() {

    if (
            time_s > ROTATION_TIME &&
            FLAGS.bits.FUN_OLD &&
            ~FLAGS.bits.ALARM &&
            FLAGS.bits.WORK_MODE
            ) {
        go_open();
        __delay_ms(5);
        go_close();
        time_s = 0;
    }
}

void fun_work() {
    if (FLAGS.bits.FUN_OLD)//fun old open?
    {
        if (PIN_FUN_STATE_GetValue() == 0) {//todo derb
            go_close();
            FLAGS.bits.FUN_OLD = 0;
        };
    } else {//fun old close
        if (PIN_FUN_STATE_GetValue() == 1) {//todo dreb
            go_open();
            FLAGS.bits.FUN_OLD = 1;
        }
    }
}

void switch_wm() {//TODO drebezg
    if (PIN_JUMP_MODE_GetValue()) {
        FLAGS.bits.JUMP = 1;
        FLAGS.bits.WORK_MODE = FLAGS.bits.JUMP;
    } else {
        FLAGS.bits.JUMP = 0;
        FLAGS.bits.WORK_MODE = FLAGS.bits.JUMP;
    }
}

void switch_zum() {
    if (zumm){
    INTCONbits.TMR0IF = 0;
    LATAbits.LATA5 = ~LATAbits.LATA5;
    }else{
        PIN_ZUMMER_SetLow();
    }
}

void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    TMR0_SetInterruptHandler(switch_zum);

    TMR2_SetInterruptHandler(Sec_tick_work);

    TMR2_StartTimer();

    while (1) {
        //   Sec_tick_work();
        //   __delay_ms(100);
        /*
       if (FLAGS.bits.ALARM) { //alarm true?        
           if (FLAGS.bits.WORK_MODE) {//work mode 1?            
               go_close_alt();
               start_tone();
           } else {//work mode 0
               go_close();
               start_tone();
           }
       } else {//alarm false
           fun_work();
           povorot();
           switch_wm();
       };
         */
        CLRWDT();
    }
}