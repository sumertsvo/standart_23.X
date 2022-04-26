#include "mcc_generated_files/mcc.h"




#define _XTAL_FREQ 4000000


//SETUP 
#define ROTATION_DAYS 14
#define LOW_WATER_RESISTANSE 20000
#define HIGH_WATER_RESISTANSE 25000
#define UP_RESISTANSE 20000
#define LOW_PIN_VOLTAGE 6000



const long int ROTATION_TIME = ROTATION_DAYS * 24 * 60 * 60; //D*H*M*S
const long int BAD_WSP_VOLTAGE = 20000; // LOW_WATER_RESISTANSE/((UP_RESISTANSE+LOW_WATER_RESISTANSE)/256);	//	????????? ???????? ADC ??? ??????????? ??????? "????" ?? ???????
const long int GOOD_WSP_VOLTAGE = 40000; //HIGH_WATER_RESISTANSE/((UP_RESISTANSE+HIGH_WATER_RESISTANSE)/256);//	????????? ???????? ADC ??? ??????????? ?????????? "????" ?? ???????



//END SETUP

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

char zumm;
char ledd;
char watt;
char rcon;


static signed char fun_counter;
static unsigned char measures;
static signed char jump_counter;
long int time_s;
unsigned result;
unsigned jresult;
unsigned fresult;

start_alarm() {
    FLAGS.bits.ALARM = 1;
    PIN_ALARM_STATE_SetHigh();
    INTCONbits.TMR0IE = 1;
}

void toggle_tone() {
    if (FLAGS.bits.ALARM) PIN_ZUMMER_TRIS = ~PIN_ZUMMER_TRIS;
    return;
}

void go_close() {

    watt = 0;

    PIN_RELE_CONTROL_SetHigh();
    __delay_ms(5);
    PIN_RELE_POWER_SetHigh();
    for (char i = 0; i < 120; i++) {
        PIN_LED_Toggle();
        __delay_ms(10);
    }
    PIN_RELE_POWER_SetLow();
    __delay_ms(5);
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
    __delay_ms(1);
    PIN_RELE_POWER_SetLow();
    return;
}

void go_open_alt() {
    watt = 1;
    PIN_RELE_POWER_SetLow();
    return;
}

void get_measure() {
  
    PIN_POWER_MEAS_SetHigh();
    unsigned res = ADC_GetConversion(PIN_WSP_STATE);
    result = res;
    PIN_POWER_MEAS_SetLow();

    if (res < BAD_WSP_VOLTAGE) measures++;
    else if (res > GOOD_WSP_VOLTAGE) measures = 0;
    if (measures > 2) start_alarm();
    return;
}

void get_fun() {
 
    PIN_POWER_MEAS_SetHigh();    
    unsigned res = ADC_GetConversion(PIN_FUN_STATE);
    PIN_POWER_MEAS_SetLow();
    fresult = res;
    if (res < LOW_PIN_VOLTAGE) fun_counter--;
    else fun_counter++;

    if (fun_counter > 10) {
        fun_counter = 10;
        FLAGS.bits.FUN_NEW = 1;
    } else if (fun_counter<-10) {
        fun_counter = -10;
        FLAGS.bits.FUN_NEW = 0;
    }
    return;
}

void get_jump() {
 
    PIN_POWER_MEAS_SetHigh();
    unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
    PIN_POWER_MEAS_SetLow();
    jresult = res;
    if (res < LOW_PIN_VOLTAGE) jump_counter--;
    else jump_counter++;

    if (jump_counter > 10) {
        jump_counter = 10;
        FLAGS.bits.JUMP = 1;
    } else if (jump_counter<-10) {
        jump_counter = -10;
        FLAGS.bits.JUMP = 0;
    }
    return;
}

void Sec_tick_work() {
    /*
    static unsigned char ds; // 1sec/10
    ds++;
    if (ds == 10) {
        ds = 0;  // */
        time_s++;

        if (FLAGS.bits.ALARM) {//if alarm
            PIN_LED_Toggle();
            toggle_tone();
        } else {//if not alarm
            get_measure();
            get_jump();
            get_fun();
            static char iled;
            iled++;
            if (iled > 2) {
                PIN_LED_Toggle();
                iled = 0;
            }
        }
 //   }
    // */
    return;
}

void povorot() {
    if (time_s > ROTATION_TIME &&
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
        if (~FLAGS.bits.FUN_NEW) {
            go_close();
            FLAGS.bits.FUN_OLD = FLAGS.bits.FUN_NEW;
        };
    } else {//fun old close
        if (FLAGS.bits.FUN_NEW) {
            go_open();
            FLAGS.bits.FUN_OLD = FLAGS.bits.FUN_NEW;
        }
    }
}

void switch_wm() {
    if (FLAGS.bits.JUMP) {
        FLAGS.bits.WORK_MODE = 1;
    } else {
        FLAGS.bits.WORK_MODE = 0;
    }
}

void switch_zum() {
    if (FLAGS.bits.ALARM) PIN_ZUMMER_Toggle();
}

void start_setup(){
    //MCC
    SYSTEM_Initialize();  // initialize the device
    INTERRUPT_GlobalInterruptEnable();    // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();    // Enable the Peripheral Interrupts
    // end MCC
    
    TMR0_SetInterruptHandler(switch_zum);
    TMR2_SetInterruptHandler(Sec_tick_work);
    TMR2_StartTimer();
    
    PIN_JUMP_STATE_ResetPullup();   
    PIN_JUMP_STATE_SetLow();
    PIN_JUMP_STATE_SetDigitalInput();
    PIN_JUMP_STATE_SetAnalogMode();
   
    
    
    PIN_FUN_STATE_SetPullup();
    PIN_FUN_STATE_SetLow();
    PIN_FUN_STATE_SetDigitalInput();
    PIN_FUN_STATE_SetAnalogMode();
    
    PIN_ALARM_STATE_SetDigitalOutput();
    PIN_ALARM_STATE_SetLow();


    INTCONbits.TMR0IE = 0;
    FLAGS.value = 0;


}

void main(void) {
    
    start_setup();
    
    while (1) {

        __delay_ms(10);

        ///*
        if (FLAGS.bits.ALARM) { //alarm true?              
            if (FLAGS.bits.WORK_MODE) {//work mode 1?            
                go_close_alt();
                start_alarm();
            } else {//work mode 0
                go_close();
                start_alarm();
            }
        } else {//alarm false          
            fun_work();
            povorot();
            switch_wm();
        };
        // */ 
        if (result+fresult+jresult+fun_counter+jump_counter > 0)
            CLRWDT();
    }
}