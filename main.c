#include "mcc_generated_files/mcc.h"




//SETUP 
#define _XTAL_FREQ 4000000
#define ROTATION_DAYS 14
#define LOW_WATER_RESISTANSE 20000
#define HIGH_WATER_RESISTANSE 25000
#define UP_RESISTANSE 20000
#define LOW_PIN_VOLTAGE 2000
#define RELE_TIME 10//120 sec
#define RELE_GAP 100 //100 msec
#define WSP_MEAS_COUNT 2
#define FUN_MEAS_COUNT 10
#define JUMP_MEAS_COUNT 10

const long int BAD_WSP_VOLTAGE = 20000; //??? LOW_WATER_RESISTANSE/((UP_RESISTANSE+LOW_WATER_RESISTANSE)/256);	//	????????? ???????? ADC ??? ??????????? ??????? "????" ?? ???????
const long int GOOD_WSP_VOLTAGE = 40000; //??? HIGH_WATER_RESISTANSE/((UP_RESISTANSE+HIGH_WATER_RESISTANSE)/256);//	????????? ???????? ADC ??? ??????????? ?????????? "????" ?? ???????
const long int ROTATION_TIME = 60; //(ROTATION_DAYS * 24 * 60 * 60); //D*H*M*S

//END SETUP

struct f_field {
    unsigned ALARM : 1;
    unsigned NORMAL_WORK_MODE : 1;
    unsigned CLOSED : 1;
    unsigned FUN_CONNECTED : 1;
    unsigned JUMP : 1;
    unsigned MEAS : 1;
    unsigned RELE_POW_WAIT : 1;
    unsigned RELE_CON_WAIT : 1;
};

union Byte {
    unsigned char value;
    struct f_field bits;
} FLAGS;

unsigned char time_pow;
static unsigned char time_con;

static signed char fun_counter;
static unsigned char measures;
static signed char jump_counter;
long int time_s;
unsigned result;
unsigned jresult;
unsigned fresult;

//served

void switch_zum() {
    PIN_ZUMMER_Toggle();
}

void toggle_tone() {
    INTCONbits.TMR0IE = ~INTCONbits.TMR0IE;
}

void beep() {
    for (char i = 0; i < 40; i++) {
        switch_zum();
        __delay_ms(1);
    }
   
}

void boop() {
    for (char i = 0; i < 50; i++) {
        switch_zum();
        __delay_us(300);
    }
   
}
//end served

void go_close() {
    time_s = 0;
    PIN_RELE_CONTROL_SetHigh();
    __delay_ms(RELE_GAP);
    PIN_RELE_POWER_SetHigh();
    time_pow = RELE_TIME;
    FLAGS.bits.RELE_POW_WAIT = 1;
    FLAGS.bits.RELE_CON_WAIT = 1;
    return;
}

void go_open() {
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
    time_pow = RELE_TIME;
    FLAGS.bits.RELE_POW_WAIT = 1;
    return;
}

void go_close_alt() {
    FLAGS.bits.CLOSED = 1;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
}

void go_open_alt() {
    FLAGS.bits.CLOSED = 0;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetLow();
    return;
}

start_alarm() {
    FLAGS.bits.ALARM = 1;
    PIN_ALARM_STATE_SetHigh();
    INTCONbits.TMR0IE = 1; //start zummer
    if (FLAGS.bits.NORMAL_WORK_MODE) {
        go_close();
    } else {
        go_close_alt();
    }
}

void get_measure() {
    PIN_POWER_MEAS_SetHigh();
    PIN_WSP_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_WSP_STATE);
    PIN_WSP_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    if (res < BAD_WSP_VOLTAGE) measures++;
    else if (res > GOOD_WSP_VOLTAGE) measures = 0;
    if (measures > WSP_MEAS_COUNT) start_alarm();
    return;
}

void get_fun() {
    PIN_POWER_MEAS_SetHigh();
    PIN_FUN_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_FUN_STATE);
    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    if (res < LOW_PIN_VOLTAGE) fun_counter--;
    else fun_counter++;

    if (fun_counter > FUN_MEAS_COUNT) {
        fun_counter = FUN_MEAS_COUNT;
        FLAGS.bits.FUN_CONNECTED = 0;
    } else if (fun_counter<-FUN_MEAS_COUNT) {
        fun_counter = -FUN_MEAS_COUNT;
        FLAGS.bits.FUN_CONNECTED = 1;
    }
    return;
}

void get_fun_full() {
    PIN_POWER_MEAS_SetHigh();
    PIN_FUN_STATE_SetAnalogMode();
    char flag = 0;
    do {
        unsigned res = ADC_GetConversion(PIN_FUN_STATE);
        if (res < LOW_PIN_VOLTAGE) fun_counter--;
        else fun_counter++;
        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            FLAGS.bits.FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            FLAGS.bits.FUN_CONNECTED = 1;
            flag = 1;
        }
    } while (flag == 0);

    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    return;
}

void get_jump() {
    PIN_JUMP_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
    PIN_JUMP_STATE_SetDigitalMode();
    if (res < LOW_PIN_VOLTAGE) jump_counter--;
    else jump_counter++;

    if (jump_counter > JUMP_MEAS_COUNT) {
        jump_counter = JUMP_MEAS_COUNT;
        FLAGS.bits.JUMP = 1;
    } else if (jump_counter<-JUMP_MEAS_COUNT) {
        jump_counter = -JUMP_MEAS_COUNT;
        FLAGS.bits.JUMP = 0;
    }
    return;
}

void get_jump_full() {
    PIN_JUMP_STATE_SetAnalogMode();
    char flag = 0;
    do {
        unsigned res = ADC_GetConversion(PIN_JUMP_STATE);

        if (res < LOW_PIN_VOLTAGE) jump_counter--;
        else jump_counter++;

        if (jump_counter > JUMP_MEAS_COUNT) {
            jump_counter = JUMP_MEAS_COUNT;
            FLAGS.bits.JUMP = 1;
            flag = 1;
        } else if (jump_counter<-JUMP_MEAS_COUNT) {
            jump_counter = -JUMP_MEAS_COUNT;
            FLAGS.bits.JUMP = 0;
            flag = 1;
        }
    } while (flag == 0);
    PIN_JUMP_STATE_SetDigitalMode();
}

void rele_tick() {
    if (FLAGS.bits.RELE_POW_WAIT) {
        if (time_pow > 0) {
            time_pow--;
        } else {
            if (FLAGS.bits.RELE_CON_WAIT) {//closing
                PIN_RELE_POWER_SetLow();
                __delay_ms(RELE_GAP);
                PIN_RELE_CONTROL_SetLow();
                FLAGS.bits.CLOSED = 1;
                FLAGS.bits.RELE_CON_WAIT = 0;
                FLAGS.bits.RELE_POW_WAIT = 0;
            } else {//opening
                PIN_RELE_POWER_SetLow();
                FLAGS.bits.CLOSED = 0;
                FLAGS.bits.RELE_POW_WAIT = 0;
            }
        }
    }
}

void sec_tick_work() {
    time_s++;
    rele_tick();
    CLRWDT();
    if (FLAGS.bits.ALARM) {//if alarm
        PIN_LED_Toggle();
        toggle_tone();
    } else {//if not alarm
        get_measure();
        static char iled;
        iled++;
        if (iled > 2) {
            PIN_LED_Toggle();
            iled = 0;
        }
    }
}

void povorot() {
    if ((time_s > ROTATION_TIME) &&
            FLAGS.bits.CLOSED == 0 &&
            FLAGS.bits.ALARM == 0 &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_close();
    }
    if ((time_s > ROTATION_TIME) &&
            FLAGS.bits.CLOSED == 0 &&
            FLAGS.bits.ALARM == 0 &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_open();
        time_s = 0;
    }

}

void fun_work() {
    {
        if (FLAGS.bits.FUN_CONNECTED && !FLAGS.bits.ALARM && FLAGS.bits.CLOSED) {
            if (FLAGS.bits.NORMAL_WORK_MODE) go_open();
            else go_open_alt();
            beep();
             __delay_ms(100);
            beep();
        };
        if (!FLAGS.bits.FUN_CONNECTED && !FLAGS.bits.CLOSED) {
            if (FLAGS.bits.NORMAL_WORK_MODE) go_close();
            else go_close_alt();
            beep();   
             __delay_ms(100);
            beep();   
             __delay_ms(100);
            beep();
        }
    }
}

void switch_wm() {
    if (FLAGS.bits.JUMP) {
        if (!FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 1;
            if (FLAGS.bits.CLOSED) go_close();
            boop();
             __delay_ms(100);
            boop();
        }
    } else {
        if (FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 0;
            if (FLAGS.bits.CLOSED) go_close_alt();
            boop();
             __delay_ms(100);
            boop();
              __delay_ms(100);
            boop();
        }
    }
}

void start_setup() {
    //MCC
    SYSTEM_Initialize(); // initialize the device
    INTERRUPT_GlobalInterruptEnable(); // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable(); // Enable the Peripheral Interrupts
    // end MCC

    TMR0_SetInterruptHandler(switch_zum);
    TMR2_SetInterruptHandler(sec_tick_work);
    TMR2_StartTimer();

    PIN_WSP_STATE_SetDigitalMode();

    PIN_JUMP_STATE_ResetPullup();
    PIN_JUMP_STATE_SetDigitalMode();
    PIN_JUMP_STATE_SetDigitalInput();


    PIN_FUN_STATE_ResetPullup();
    PIN_FUN_STATE_SetDigitalMode();
    PIN_FUN_STATE_SetDigitalInput();

    PIN_RELE_POWER_SetLow();
    PIN_RELE_CONTROL_SetLow();

    PIN_ALARM_STATE_SetDigitalOutput();
    PIN_ALARM_STATE_SetLow();


    INTCONbits.TMR0IE = 0;
    FLAGS.value = 0;

    get_fun_full();
    get_jump_full();
    time_pow = 0;
}

void main(void) {

    start_setup();
    while (1) {

        __delay_ms(50);
        if (FLAGS.bits.ALARM == 0) {

            get_fun();
            fun_work();

            get_jump();
            switch_wm();

            povorot();
        };
    }
}