#include "mcc_generated_files/mcc.h"
#include "eeprom.h"
/*▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄*/



#define DEBUG_ENABLED


/*DEFAULT_SETTINGS*/
#define VERSION 1
const char SHORT_ZUMMER_DELAY = 30;
const char LONG_ZUMMER_DELAY = 130;
const char FRIMWARE_VERSION_EEPROM_ADR = 0x01;
const char AUTOROTATION_DAYS = 14; //дней до поворота крана
const unsigned LOW_WATER_RESISTANSE = 20000; //сопротивление датчика
const unsigned HIGH_WATER_RESISTANSE = 25000; //
const unsigned UP_RESISTANSE = 20000; //сопротивление делителя
//const char LOW_PIN_VOLTAGE=25; // "низкий логический уровень"
/*защита от дребезга*/
const char WSP_MEAS_COUNT = 8; //количество измерений датчика
const char FUN_MEAS_COUNT = 10; //количество измерений переключателя
const char JUMP_MEAS_COUNT = 10; //количество измерений джампера
/*задержки*/
const char RELE_POWER_WORK_DELAY = 120; // sec
const char RELE_POWER_AUTOROTATION_DELAY = 10; // sec
const char RELE_GAP = 1; //sec
const unsigned AUTOROTATION_DELAY = (AUTOROTATION_DAYS * 24 * 60 * 60); //D*H*M*S
/*voltages*/
const unsigned BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256));
const unsigned GOOD_WSP_VOLTAGE = (HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256));

/*▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄*/

/**FLAGS*/
static union {
    unsigned long value;

    struct {
        unsigned ALARM_ON : 1;
        unsigned ALARM_OFF : 1;
        unsigned FUN_HIGH : 1;
        unsigned FUN_LOW : 1;
        unsigned ALLOW_MEASURE : 1;
        unsigned ALLOW_FUN : 1;
        unsigned ALLOW_JUMP : 1;
        unsigned JUMP_LOW : 1;
        unsigned JUMP_HIGH : 1;
        unsigned OPENING : 1;
        unsigned OPENED : 1;
        unsigned CLOSING : 1;
        unsigned CLOSED : 1;
        unsigned RELE_POWER_ON : 1;
        unsigned RELE_CONTROL_ON : 1;
        unsigned WATER_TRUE : 1;
        unsigned WATER_FALSE : 1;
        unsigned TONE_ON : 1;
        unsigned TONE_OFF : 1;
        unsigned BEEP_LONG : 1;
        unsigned ZUM_BUSY : 1;
        unsigned BEEP_SHORT : 1;
        unsigned GO_CLOSE : 1;
        unsigned GO_OPEN : 1;
        unsigned NORMAL_WORK_MODE_ON : 1;
        unsigned UNIVERSAL_VORK_MODE_ON : 1;
        unsigned LED_ON : 1;
        unsigned ZUM_ON : 1;
        unsigned MEAS_ON : 1;
        unsigned AUTOROTATION_WORK : 1;
        unsigned MELODY_ON : 1;
        unsigned LAST_BEEP_LONG : 1;
    } bits;
} ff; //*FF = 0x0f9;

/*▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄*/




/*TIMES*/

/*sec_div*/
__uint24 time_rotation; //время до автоповорота (сек)
unsigned time_rele_power; //время до закрытия реле (сек)
unsigned time_rele_control;
unsigned time_rele_gap;
unsigned time_led;
unsigned time_tone; //s
unsigned time_zummer_short; //ms
unsigned time_zummer_long; //ms

/*ms_div*/
char time_meas;

/*▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄*/



/*counters*/
char beep_short_count;
char beep_long_count;
char beep_double_count;

/*█████████████████████████████████████████████████████████████████████*/
/*█████████████████████████████████████████████████████████████████████*/



#ifdef DEBUG_ENABLED

void toggle_zummer() {
    PIN_ZUMMER_SetHigh();
    PIN_ZUMMER_SetLow();
}
#endif

void start_tone() {//вкл/выкл зуммер
    ff.bits.TONE_ON = 1;
    ff.bits.TONE_OFF = 0;
}

void stop_tone() {//вкл/выкл зуммер
    ff.bits.TONE_ON = 0;
    ff.bits.TONE_OFF = 1;
}

void beep_short(char count) { //короткий писк
    start_tone();
    ff.bits.ZUM_BUSY = 1;
    time_tone = SHORT_ZUMMER_DELAY;
    ff.bits.BEEP_SHORT = 1;
    beep_short_count = count;
}

void beep_long(char count) {//короткий писк
    start_tone();
    ff.bits.ZUM_BUSY = 1;
    time_tone = 30;
    beep_long_count = count;
}

void beep_double(char count) {//TODO
    start_tone();
    ff.bits.ZUM_BUSY = 1;
    time_tone = 30;
    beep_double_count = count;
}

void go_close() {//начало закрытия кранов

    ff.bits.CLOSING = 1;
    ff.bits.OPENED = 0;
    ff.bits.OPENING = 0;

    time_rotation = 0;
    ff.bits.RELE_CONTROL_ON = 1;

    time_rele_control = RELE_GAP + RELE_POWER_WORK_DELAY + RELE_GAP;
    time_rele_power = RELE_POWER_WORK_DELAY;
    time_rele_gap = RELE_GAP;

    return;
}

void go_close_short() {//начало закрытия кранов при автоповороте

    ff.bits.CLOSING = 1;
    ff.bits.OPENED = 0;
    ff.bits.OPENING = 0;


    time_rotation = 0;

    ff.bits.RELE_CONTROL_ON = 1;

    time_rele_control = RELE_GAP + RELE_POWER_AUTOROTATION_DELAY + RELE_GAP;
    time_rele_power = RELE_POWER_AUTOROTATION_DELAY;
    time_rele_gap = RELE_GAP;

    return;
}

void go_open() {//начало открытия кранов

    ff.bits.CLOSED = 0;
    ff.bits.CLOSING = 0;
    ff.bits.OPENING = 1;


    ff.bits.RELE_CONTROL_ON = 0;
    ff.bits.RELE_POWER_ON = 1;

    time_rele_power = RELE_POWER_WORK_DELAY;

    ff.bits.AUTOROTATION_WORK = 0;
    return;
}

void go_close_alt() {//закрытие кранов 2 режим

    ff.bits.OPENED = 0;
    ff.bits.CLOSED = 1;

    ff.bits.RELE_CONTROL_ON = 0;
    ff.bits.RELE_POWER_ON = 1;

}

void go_open_alt() {//открытие кранов 2 режим

    ff.bits.CLOSED = 0;
    ff.bits.OPENED = 1;

    ff.bits.RELE_CONTROL_ON = 0;
    ff.bits.RELE_POWER_ON = 0;

}

void close() {
    if (ff.bits.NORMAL_WORK_MODE_ON) {
        go_close;
    } else if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
        go_close_alt;
    }
}

void open() {
    if (ff.bits.NORMAL_WORK_MODE_ON) {
        go_open;
    } else if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
        go_open_alt;
    }
}

void rele_tick() {//закрытие кранов (задержка на работу привода)

#ifdef DEBUG_ENABLED
    toggle_zummer();
#endif

    if (ff.bits.OPENING && ff.bits.CLOSING) {
        return;
    }


    if (ff.bits.OPENING) {
        if (time_rele_power > 0) {
            time_rele_power--;
            if (time_rele_power == 0)
                ff.bits.RELE_POWER_ON = 0;
            ff.bits.OPENED = 1;
            ff.bits.OPENING = 0;
        }
    }


    if (ff.bits.CLOSING) {

        if (time_rele_gap > 0) {
            time_rele_gap--;
        }

        if (time_rele_gap = 0) {
            time_rele_power--;
            if (time_rele_power == 0) {
                ff.bits.RELE_POWER_ON = 0;
            }
        }

        if (time_rele_control > 0) {
            time_rele_control--;
            if (time_rele_control == 0) {
                ff.bits.RELE_CONTROL_ON = 0;
                ff.bits.CLOSED = 1;
                ff.bits.CLOSING = 0;
            }
        }
    }

}

void start_alarm() {//обнаружена протечка

    ff.bits.ALARM_ON = 1;
    ff.bits.ALARM_OFF = 0;

    ff.bits.MELODY_ON = 1;


    close();
}

void get_measure() {//измерение состояния датчиков
    static unsigned char measures;
    PIN_POWER_MEAS_SetHigh();
    PIN_WSP_STATE_SetAnalogMode();
    __delay_ms(1);
    unsigned res = ADC_GetConversion(PIN_WSP_STATE);
    PIN_WSP_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    if (res < BAD_WSP_VOLTAGE) measures++;
    else if (res > GOOD_WSP_VOLTAGE) measures = 0;
    if (measures > WSP_MEAS_COUNT) start_alarm();
    return;
}

void get_fun() {//определение положения переключателя (антидребезг 1 шаг)
    static signed char fun_counter;
    PIN_POWER_MEAS_SetHigh();
    __delay_ms(1);
    //   unsigned res = ADC_GetConversion(PIN_FUN_STATE);
    PIN_FUN_STATE_SetDigitalMode();
    PIN_FUN_STATE_SetDigitalInput();
    if (PIN_FUN_STATE_GetValue()) fun_counter--;
    else fun_counter++;
    PIN_POWER_MEAS_SetLow();
    /*
    if (res < LOW_PIN_VOLTAGE) fun_counter--;
    else fun_counter++;
     * */

    if (fun_counter > FUN_MEAS_COUNT) {
        fun_counter = FUN_MEAS_COUNT;
        ff.bits.FUN_LOW = 0;
        ff.bits.FUN_HIGH = 1;
    } else if (fun_counter<-FUN_MEAS_COUNT) {
        fun_counter = -FUN_MEAS_COUNT;
        ff.bits.FUN_LOW = 1;
        ff.bits.FUN_HIGH = 0;
    }

    return;
}

void get_fun_full() {//определение положения переключателя (антидребезг все шаги)

    static signed char fun_counter;
    PIN_POWER_MEAS_SetHigh();
    PIN_FUN_STATE_SetAnalogMode();
    char flag = 0;
    do {
        if (PIN_FUN_STATE_GetValue()) fun_counter++;
        else fun_counter--;




        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            ff.bits.FUN_LOW = 0;
            ff.bits.FUN_HIGH = 1;
            flag = 1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            ff.bits.FUN_LOW = 1;
            ff.bits.FUN_HIGH = 0;
            flag = 1;
        }


    } while (flag == 0);

    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    return;
}

void get_jump() {//определение положения джампера (антидребезг 1 шаг)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetDigitalMode();
    PIN_JUMP_STATE_SetDigitalInput();
    if (PIN_JUMP_STATE_GetValue()) jump_counter++;
    else jump_counter--;
    /*
    unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
    if (res < LOW_PIN_VOLTAGE) jump_counter--;
    else jump_counter++;
     */


    if (jump_counter > JUMP_MEAS_COUNT) {
        jump_counter = JUMP_MEAS_COUNT;
        ff.bits.JUMP_LOW = 0;
        ff.bits.JUMP_HIGH = 1;
    } else if (jump_counter<-JUMP_MEAS_COUNT) {
        jump_counter = -JUMP_MEAS_COUNT;
        ff.bits.JUMP_LOW = 1;
        ff.bits.JUMP_HIGH = 0;
    }


    return;
}

void get_jump_full() {//определение положения переключателя (антидребезг все шаги)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetAnalogMode();
    char flag = 0;
    do {

        if (PIN_JUMP_STATE_GetValue()) jump_counter++;
        else jump_counter--;



        if (jump_counter > JUMP_MEAS_COUNT) {
            jump_counter = JUMP_MEAS_COUNT;
            ff.bits.JUMP_LOW = 0;
            ff.bits.JUMP_HIGH = 1;
            ;
            flag = 1;
        } else if (jump_counter<-JUMP_MEAS_COUNT) {
            jump_counter = -JUMP_MEAS_COUNT;
            ff.bits.JUMP_LOW = 1;
            ff.bits.JUMP_HIGH = 0;
            flag = 1;
        }



    } while (flag == 0);
    PIN_JUMP_STATE_SetDigitalMode();
}

void sec_tick_work() {//работа секундного таймера

    if (ff.bits.OPENED && ff.bits.NORMAL_WORK_MODE_ON) time_rotation++;
    rele_tick();

    if (ff.bits.ALARM_ON) {
        PIN_LED_Toggle();

#ifdef DEBUG_ENABLED
        toggle_zummer();
#endif
    } else if (ff.bits.ALARM_OFF) {//if not alarm

        static char iled;
        iled++;
        if (iled > 2) {
            PIN_LED_Toggle();
            iled = 0;
        }
    }
}

void autorotation_work() {//автоповорот
    if ((time_rotation > AUTOROTATION_DELAY) &&
            !ff.bits.CLOSED &&
            !ff.bits.CLOSING &&
            ff.bits.ALARM_OFF &&
            ff.bits.NORMAL_WORK_MODE_ON
            ) {
        go_close_short();
    }

    if ((time_rotation > (AUTOROTATION_DELAY + RELE_POWER_WORK_DELAY + RELE_GAP * 2)) && //закрытие идёт указанное время
            ff.bits.CLOSED &&
            ff.bits.CLOSING &&
            ff.bits.ALARM_OFF &&
            ff.bits.NORMAL_WORK_MODE_ON
            ) {
        go_open();
        time_rotation = 0; //обнуляем счетчик
    }

}

void fun_work() {//работа переключателя
    {
        if (//открытие
                ff.bits.FUN_LOW &&
                !ff.bits.FUN_HIGH &&
                ff.bits.ALARM_OFF &&
                ff.bits.CLOSED &&
                !ff.bits.RELE_POWER_ON) {
            beep_short(1);
            open();
        };
        if (//закрытие
                ff.bits.FUN_HIGH &&
                !ff.bits.FUN_LOW &&
                ff.bits.OPENED &&
                !ff.bits.RELE_POWER_ON) {
            beep_short(2);
            close();
        }
    }
}

void switch_wm() {//выбор режима работы
    if (ff.bits.JUMP_LOW) {//go_alt_mode
        if (ff.bits.NORMAL_WORK_MODE_ON) {
            ff.bits.NORMAL_WORK_MODE_ON = 0;
            ff.bits.UNIVERSAL_VORK_MODE_ON = 1;
            //   if (*FLAGS.bits.CLOSED) go_close_alt();
            //три высоких писка
            beep_long(2); //_freq pause work_time count
        }
    } else if (ff.bits.JUMP_HIGH) {//go_norm_mode
        if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
            ff.bits.NORMAL_WORK_MODE_ON = 1;
            ff.bits.UNIVERSAL_VORK_MODE_ON = 0;
            //    if (*FLAGS.bits.CLOSED) go_close();
            //два высоких писка
            beep_long(1); //_freq pause work_time count;
        }
    }
}

void ms_tick() {
    static unsigned tick_count = 0;
    tick_count++;

    if (ff.bits.BEEP_SHORT) {
        time_zummer_short--;
    }
    if (ff.bits.BEEP_LONG) {
        time_zummer_long--;
    }
    if (tick_count == 100) {
        ff.bits.ALLOW_MEASURE = 1;
        tick_count = 0;
    }

    if (tick_count == 1000) {
        sec_tick_work();
        tick_count = 0;
    }
}

/*█████████████████████████████████████████████████████████████████████*/

/*HARDWARE*/
void eeprom_set() {
    char vers = EEPROM_ReadByte(FRIMWARE_VERSION_EEPROM_ADR);
    if (vers == 0xFF) {
        EEPROM_WriteByte(FRIMWARE_VERSION_EEPROM_ADR, VERSION);
    }
    //add_eeprom_setup
}

void hardware_work() {
    PIN_ALARM_STATE_LAT = ff.bits.ALARM_ON;
    PIN_POWER_MEAS_LAT = ff.bits.MEAS_ON;
    PIN_RELE_CONTROL_LAT = ff.bits.RELE_CONTROL_ON;
    PIN_RELE_POWER_LAT = ff.bits.RELE_POWER_ON;
    PIN_LED_LAT = ff.bits.LED_ON;
    // PIN_ZUMMER_LAT = ff.bits.ZUM_ON;
    if (ff.bits.TONE_ON) {
        INTCONbits.TMR0IE = 1;
    };
    if (ff.bits.TONE_OFF) {
        INTCONbits.TMR0IE = 0;
        PIN_ZUMMER_SetLow();
    };
}

void timer0_switch() {//одно переключение
    PIN_ZUMMER_Toggle();
}

void start_setup() {//начальная настройка
    //MCC сгенерировано
    SYSTEM_Initialize(); // initialize the device
    INTERRUPT_GlobalInterruptEnable(); // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable(); // Enable the Peripheral Interrupts
    // end MCC
    // get_eeprom();
    eeprom_set();

    TMR0_SetInterruptHandler(timer0_switch);
    TMR2_SetInterruptHandler(ms_tick);
    TMR2_StartTimer(); //начать секундный счет

    //отключение аналоговых входов чтобы не мешали измерениям
    PIN_WSP_STATE_SetDigitalMode();
    PIN_JUMP_STATE_SetDigitalMode();
    PIN_FUN_STATE_SetDigitalMode();

    //сброс состояния
    PIN_JUMP_STATE_ResetPullup();
    PIN_JUMP_STATE_SetDigitalInput();
    PIN_FUN_STATE_ResetPullup();
    PIN_FUN_STATE_SetDigitalInput();
    INTCONbits.TMR0IE = 0; //выкл зуммер
    ff.value = 0;
    ff.bits.ALARM_ON = 0;
    ff.bits.ALARM_OFF = 1;
    PIN_RELE_POWER_SetLow();
    PIN_RELE_CONTROL_SetLow();
    PIN_ALARM_STATE_SetLow();
    PIN_ALARM_STATE_SetDigitalOutput();

    //проверка текущего режима
    //  get_jump_full();
    //   get_fun_full();
    time_rele_power = 0;
}

#ifdef DEBUG_ENABLED

void flag_error() {
    if (
            (ff.bits.ALARM_OFF && ff.bits.ALARM_ON) ||
            ((ff.bits.ALLOW_FUN + ff.bits.ALLOW_JUMP + ff.bits.ALLOW_MEASURE) > 1) ||
            ((ff.bits.CLOSED + ff.bits.CLOSING + ff.bits.OPENED + ff.bits.OPENING) > 1) ||
            (ff.bits.FUN_HIGH && ff.bits.FUN_LOW) ||
            (ff.bits.GO_CLOSE && ff.bits.GO_OPEN) ||
            (ff.bits.JUMP_HIGH && ff.bits.JUMP_LOW) ||
            (ff.bits.NORMAL_WORK_MODE_ON && ff.bits.UNIVERSAL_VORK_MODE_ON) ||
            (ff.bits.TONE_OFF && ff.bits.TONE_ON) ||
            (ff.bits.WATER_FALSE && ff.bits.WATER_TRUE)
            ) {
        PIN_LED_SetHigh();
        start_tone();
        TMR2_StopTimer();
        while (1) {
            CLRWDT();
        }
    }
}
#endif

/*█████████████████████████████████████████████████████████████████████*/

void main(void) {

    start_setup();

    while (1) {
        //    if 
        //        get_voltage(); //  deleted, unused
        CLRWDT();
        hardware_work();
        if (!ff.bits.ALARM_OFF) {

            get_jump();
            switch_wm();

            get_fun();
            fun_work();

            if (ff.bits.ALLOW_MEASURE) {
                get_measure();
                ff.bits.ALLOW_MEASURE = 0;
            }
            autorotation_work();

        };
    }
}