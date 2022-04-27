#include "mcc_generated_files/mcc.h"

//
#define DEBUG

//SETUP 
#define _XTAL_FREQ 4000000 //??????? ??
#define ROTATION_DAYS 14 //???? ?? ???????? ?????
#define LOW_WATER_RESISTANSE 20000  //????????????? ???????
#define HIGH_WATER_RESISTANSE 25000 //
#define UP_RESISTANSE 20000 //????????????? ????????
#define LOW_PIN_VOLTAGE 2000 // "?????? ?????????? ???????"

//?????? ?? ????????
#define WSP_MEAS_COUNT 2    //?????????? ????????? ???????
#define FUN_MEAS_COUNT 10   //?????????? ????????? ?????????????
#define JUMP_MEAS_COUNT 10  //?????????? ????????? ????????


#ifdef DEBUG

#define RELE_TIME 10// sec
#define RELE_GAP 1 // sec
const long int BAD_WSP_VOLTAGE = 20000; //
const long int GOOD_WSP_VOLTAGE = 40000; //
const long int ROTATION_TIME = 60; //sec

#else

#define RELE_TIME 120 // sec
#define RELE_GAP 1 //sec
const long int BAD_WSP_VOLTAGE = LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256); //	TODO
const long int GOOD_WSP_VOLTAGE = HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256); //	TODO
const long int ROTATION_TIME = (ROTATION_DAYS * 24 * 60 * 60); //D*H*M*S

#endif

//END SETUP

//?????

struct f_field {
    unsigned ALARM : 1;
    unsigned NORMAL_WORK_MODE : 1;
    unsigned CLOSED : 1;
    unsigned _FUN_CONNECTED : 1;
    unsigned _JUMP : 1;
    unsigned RELE_POW_WAIT : 1;
    unsigned RELE_CON_WAIT : 1;
    unsigned FREE : 1;
};

union Byte {
    unsigned char value;
    struct f_field bits;
} FLAGS;

unsigned char time_pow_s; //????? ?? ???????? ???? (???)
long int time_s; //????? ?? ???????????? (???)

void switch_zum() {//???? ????????????
    PIN_ZUMMER_Toggle();
}

void toggle_tone() {//???/???? ??????
    INTCONbits.TMR0IE = ~INTCONbits.TMR0IE;
}

void beep(unsigned delay, unsigned pause, char time, char count) {//???????? ????
    for (char j = 0; j < count; j++) {
        for (char i = 0; i < time; i++) {
            switch_zum();
            __delay_us(delay);
        }
        __delay_us(pause);
    }
}

void go_close() {//?????? ???????? ??????
    time_s = 0;
    PIN_RELE_CONTROL_SetHigh();
    __delay_ms(RELE_GAP * 1000);
    PIN_RELE_POWER_SetHigh();
    time_pow_s = RELE_TIME;
    FLAGS.bits.RELE_POW_WAIT = 1;
    FLAGS.bits.RELE_CON_WAIT = 1;
    return;
}

void go_open() {//?????? ???????? ??????
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
    time_pow_s = RELE_TIME;
    FLAGS.bits.RELE_POW_WAIT = 1;
    return;
}

void go_close_alt() {//???????? ?????? 2 ?????
    FLAGS.bits.CLOSED = 1;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
}

void go_open_alt() {//???????? ?????? 2 ?????
    FLAGS.bits.CLOSED = 0;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetLow();
    return;
}

start_alarm() {//?????????? ????????
    FLAGS.bits.ALARM = 1;
    PIN_ALARM_STATE_SetHigh();
    INTCONbits.TMR0IE = 1; //??? ??????
    if (FLAGS.bits.NORMAL_WORK_MODE) {
        go_close();
    } else {
        go_close_alt();
    }
}

void get_measure() {//????????? ????????? ????????
    static unsigned char measures;
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

void get_fun() {//??????????? ????????? ????????????? (??????????? 1 ???)

    static signed char fun_counter;
    PIN_POWER_MEAS_SetHigh();
    PIN_FUN_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_FUN_STATE);
    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    if (res < LOW_PIN_VOLTAGE) fun_counter--;
    else fun_counter++;

    if (fun_counter > FUN_MEAS_COUNT) {
        fun_counter = FUN_MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 0;
    } else if (fun_counter<-FUN_MEAS_COUNT) {
        fun_counter = -FUN_MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 1;
    }
    return;
}

void get_fun_full() {//??????????? ????????? ????????????? (??????????? ??? ????)

    static signed char fun_counter;
    PIN_POWER_MEAS_SetHigh();
    PIN_FUN_STATE_SetAnalogMode();
    char flag = 0;
    do {
        unsigned res = ADC_GetConversion(PIN_FUN_STATE);
        if (res < LOW_PIN_VOLTAGE) fun_counter--;
        else fun_counter++;
        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 1;
            flag = 1;
        }
    } while (flag == 0);

    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    return;
}

void get_jump() {//??????????? ????????? ???????? (??????????? 1 ???)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
    PIN_JUMP_STATE_SetDigitalMode();


    if (res < LOW_PIN_VOLTAGE) jump_counter--;
    else jump_counter++;

    if (jump_counter > JUMP_MEAS_COUNT) {
        jump_counter = JUMP_MEAS_COUNT;
        FLAGS.bits._JUMP = 1;
    } else if (jump_counter<-JUMP_MEAS_COUNT) {
        jump_counter = -JUMP_MEAS_COUNT;
        FLAGS.bits._JUMP = 0;
    }
    return;
}

void get_jump_full() {//??????????? ????????? ????????????? (??????????? ??? ????)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetAnalogMode();
    char flag = 0;
    do {
        unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
        if (res < LOW_PIN_VOLTAGE) jump_counter--;
        else jump_counter++;

        if (jump_counter > JUMP_MEAS_COUNT) {
            jump_counter = JUMP_MEAS_COUNT;
            FLAGS.bits._JUMP = 1;
            flag = 1;
        } else if (jump_counter<-JUMP_MEAS_COUNT) {
            jump_counter = -JUMP_MEAS_COUNT;
            FLAGS.bits._JUMP = 0;
            flag = 1;
        }
    } while (flag == 0);
    PIN_JUMP_STATE_SetDigitalMode();
}

void rele_tick() {//???????? ?????? (???????? ?? ?????? ???????)
    if (FLAGS.bits.RELE_POW_WAIT) {//???? ???????? ??????? ????
        if (time_pow_s > 0) { //????? ?? ????????
            time_pow_s--;
        } else {
            if (FLAGS.bits.RELE_CON_WAIT) {//???? ???? ??????? ?? ???????????
                PIN_RELE_POWER_SetLow();
                __delay_ms(RELE_GAP * 1000);
                PIN_RELE_CONTROL_SetLow();
                FLAGS.bits.CLOSED = 1;
                FLAGS.bits.RELE_CON_WAIT = 0;
                FLAGS.bits.RELE_POW_WAIT = 0;
            } else {//???? ?? ??????? ?? ???????????
                PIN_RELE_POWER_SetLow();
                FLAGS.bits.CLOSED = 0;
                FLAGS.bits.RELE_POW_WAIT = 0;
            }
        }
    }
}

void sec_tick_work() {//?????? ?????????? ???????
    time_s++;
    rele_tick();
    CLRWDT(); // <2.1 ???
    if (FLAGS.bits.ALARM) {
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

void povorot() {//???????????
    if ((time_s > ROTATION_TIME) &&
            !FLAGS.bits.CLOSED &&
            !FLAGS.bits.ALARM &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_close();
    }
    if ((time_s > (ROTATION_TIME + RELE_TIME + RELE_GAP * 2)) && //???????? ???? ????????? ?????
            FLAGS.bits.CLOSED &&
            FLAGS.bits.ALARM == 0 &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_open();
        time_s = 0; //???????? ???????
    }

}

void fun_work() {//?????? ?????????????
    {
        if (FLAGS.bits._FUN_CONNECTED &&
                !FLAGS.bits.ALARM &&
                FLAGS.bits.CLOSED &&
                !FLAGS.bits.RELE_POW_WAIT) {
            if (FLAGS.bits.NORMAL_WORK_MODE) go_open();
            else go_open_alt();
            //???? ?????? ????
            beep(500, 100, 40, 1); //_freq pause work_time count
        };
        if (!FLAGS.bits._FUN_CONNECTED &&
                !FLAGS.bits.CLOSED &&
                !FLAGS.bits.RELE_POW_WAIT) {
            if (FLAGS.bits.NORMAL_WORK_MODE) go_close();
            else go_close_alt();
            //??? ?????? ?????
            beep(500, 100, 40, 2); //_freq pause work_time count
        }
    }
}

void switch_wm() {//????? ?????? ??????
    if (FLAGS.bits._JUMP) {
        if (!FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 1;
            if (FLAGS.bits.CLOSED) go_close();
            //??? ??????? ?????
            beep(250, 100, 40, 2); //_freq pause work_time count;
        }
    } else {
        if (FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 0;
            if (FLAGS.bits.CLOSED) go_close_alt();
            //??? ??????? ?????
            beep(250, 100, 40, 3); //_freq pause work_time count
        }
    }
}

void start_setup() {//????????? ?????????
    //MCC ?????????????
    SYSTEM_Initialize(); // initialize the device
    INTERRUPT_GlobalInterruptEnable(); // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable(); // Enable the Peripheral Interrupts
    // end MCC

    TMR0_SetInterruptHandler(switch_zum);
    TMR2_SetInterruptHandler(sec_tick_work);
    TMR2_StartTimer(); //?????? ????????? ????




    //?????????? ?????????? ?????? ????? ?? ?????? ??????????
    PIN_WSP_STATE_SetDigitalMode();
    PIN_JUMP_STATE_SetDigitalMode();
    PIN_FUN_STATE_SetDigitalMode();

    //????? ?????????
    PIN_JUMP_STATE_ResetPullup();
    PIN_JUMP_STATE_SetDigitalInput();
    PIN_FUN_STATE_ResetPullup();
    PIN_FUN_STATE_SetDigitalInput();
    INTCONbits.TMR0IE = 0; //???? ??????
    FLAGS.value = 0;
    PIN_RELE_POWER_SetLow();
    PIN_RELE_CONTROL_SetLow();
    PIN_ALARM_STATE_SetLow();
    PIN_ALARM_STATE_SetDigitalOutput();

    //???????? ???????? ??????
    get_fun_full();
    get_jump_full();
    time_pow_s = 0;

    //time_s=get from eeprom
}

void main(void) {
    
    start_setup();
    
#ifdef DEBUG
       OPTION_REG = (uint8_t)((OPTION_REG & 0xC0) | (0xD7 & 0x3F)); 
       TMR0 = 0x16;
       timer0ReloadVal= 22;
#endif
       


    while (1) {

        __delay_ms(50);

        if (!FLAGS.bits.ALARM) {
            get_fun();
            fun_work();
            get_jump();
            switch_wm();
            povorot();
#ifdef DEBUG
            switch_zum();
#endif

        };
    }
}