#include "mcc_generated_files/mcc.h"
#include "eeprom.h"
/*▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄*/



//#define DEBUG_ENABLED


/*DEFAULT_SETTINGS*/
#define VERSION 1
const char SHORT_ZUMMER_DELAY = 30;
const char LONG_ZUMMER_DELAY = 130;
const char FRIMWARE_VERSION_EEPROM_ADR = 0x01;
const char AUTOROTATION_DAYS = 14; //дней до поворота крана
const char MOVING_WAIT_DELAY = 2;
const unsigned LOW_WATER_RESISTANSE = 20000; //сопротивление датчика
const unsigned HIGH_WATER_RESISTANSE = 25000; //
const unsigned UP_RESISTANSE = 20000; //сопротивление делителя
/*защита от дребезга*/
const char WSP_MEAS_COUNT = 4; //количество измерений датчика
const char FUN_MEAS_COUNT = 10; //количество измерений переключателя
const char JUMP_MEAS_COUNT = 10; //количество измерений джампера
/*задержки*/
const char RELE_POWER_WORK_DELAY = 15; // sec
const char RELE_POWER_AUTOROTATION_DELAY = 5; // sec
const char RELE_GAP = 2; //sec
const char MELODY_REPEAT_DELAY = 3; // 30; //min
const unsigned AUTOROTATION_DELAY = 120;// (AUTOROTATION_DAYS * 24 * 60 * 60); //D*H*M*S
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
        unsigned SIREN : 1;
        unsigned ZUM_BUSY : 1;
        unsigned BEEP_SHORT : 1;
        unsigned GO_CLOSE : 1;
        unsigned MOVING_ALLOWED : 1;
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
unsigned time_tone; 
char sec_count = 0;
//char time_siren; //s
//char time_silent; //s
char time_melody; //minute
char time_moving_wait;
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

/*SERVICE*/

#ifdef DEBUG_ENABLED

void toggle_zummer() {
    PIN_ZUMMER_SetHigh();
    PIN_ZUMMER_SetLow();
}
#endif

void start_tone() {//вкл/выкл зуммер
    ff.bits.ZUM_BUSY = 1;
    ff.bits.TONE_ON = 1;
    ff.bits.TONE_OFF = 0;
}

void stop_tone() {//вкл/выкл зуммер
    ff.bits.ZUM_BUSY = 0;
    ff.bits.TONE_ON = 0;
    ff.bits.TONE_OFF = 1;
}

void beep_short() { //короткий писк
    if (!ff.bits.ZUM_BUSY) {
        if (beep_short_count > 0) beep_short_count--;
        time_tone = SHORT_ZUMMER_DELAY;
        ff.bits.LAST_BEEP_LONG = 0;
        start_tone();
    }
}

void beep_long() {//короткий писк
    if (!ff.bits.ZUM_BUSY) {
        if (beep_long_count > 0) beep_long_count--;
        time_tone = LONG_ZUMMER_DELAY;
        ff.bits.LAST_BEEP_LONG = 1;
        start_tone();
    }
}

void beep_double() {//TODO
    if (ff.bits.LAST_BEEP_LONG) {
        beep_short();
    } else {
        beep_long();
    }
}

void go_close() {//начало закрытия кранов

    if (!ff.bits.CLOSING && !ff.bits.CLOSED && ff.bits.MOVING_ALLOWED) {
        ff.bits.CLOSING = 1;
        ff.bits.OPENED = 0;
        ff.bits.OPENING = 0;

        ff.bits.RELE_POWER_ON = 0;
        ff.bits.RELE_CONTROL_ON = 1;

        time_rele_control = RELE_GAP + RELE_POWER_WORK_DELAY + RELE_GAP;
        time_rele_power = RELE_POWER_WORK_DELAY;
        time_rele_gap = RELE_GAP;

        time_rotation = 0;

    }
}

void go_close_short() {//начало закрытия кранов

    if (!ff.bits.CLOSING && !ff.bits.CLOSED && ff.bits.MOVING_ALLOWED) {
        ff.bits.CLOSING = 1;
        ff.bits.OPENED = 0;
        ff.bits.OPENING = 0;

        ff.bits.RELE_POWER_ON = 0;
        ff.bits.RELE_CONTROL_ON = 1;

        time_rele_control = RELE_GAP + RELE_POWER_AUTOROTATION_DELAY + RELE_GAP;
        time_rele_power = RELE_POWER_AUTOROTATION_DELAY;
        time_rele_gap = RELE_GAP;

        time_rotation = 0;

    }
}


void go_open() {//начало открытия кранов

    if (!ff.bits.OPENED && !ff.bits.OPENING && ff.bits.MOVING_ALLOWED) {
        ff.bits.OPENING = 1;
        ff.bits.CLOSED = 0;
        ff.bits.CLOSING = 0;


        ff.bits.RELE_CONTROL_ON = 0;
        ff.bits.RELE_POWER_ON = 1;

        time_rele_power = RELE_POWER_WORK_DELAY;

        ff.bits.AUTOROTATION_WORK = 0;
        return;
    }
}

void go_close_alt() {//закрытие кранов 2 режим

    if ((!ff.bits.CLOSED && ff.bits.MOVING_ALLOWED) || ff.bits.ALARM_ON) {
        ff.bits.OPENED = 0;
        ff.bits.CLOSED = 1;

        ff.bits.RELE_CONTROL_ON = 0;
        ff.bits.RELE_POWER_ON = 1;
    }
}

void go_open_alt() {//открытие кранов 2 режим
    if (!ff.bits.OPENED && ff.bits.MOVING_ALLOWED) {
        ff.bits.CLOSED = 0;
        ff.bits.OPENED = 1;

        ff.bits.RELE_CONTROL_ON = 0;
        ff.bits.RELE_POWER_ON = 0;
    }
}

void rele_off() {
    ff.bits.RELE_CONTROL_ON = 0;
    ff.bits.RELE_POWER_ON = 0;
    ff.bits.CLOSING = 0;
    ff.bits.OPENING = 0;
    ff.bits.CLOSED = 0;
    if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
        ff.bits.OPENED = 1;
    } else {
        ff.bits.OPENED = 0;
    }
    ff.bits.MOVING_ALLOWED = 0;
    time_moving_wait = MOVING_WAIT_DELAY;
}

void close() {
    if (ff.bits.OPENING) {
        rele_off();
    } else {
        if (ff.bits.NORMAL_WORK_MODE_ON) {
            go_close();
        } else if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
            go_close_alt();
        }
    }
}

void open() {
    if (ff.bits.CLOSING) {
        rele_off();
    } else {
        if (ff.bits.NORMAL_WORK_MODE_ON) {
            go_open();
        } else if (ff.bits.UNIVERSAL_VORK_MODE_ON) {
            go_open_alt();
        }
    }
}

void rele_tick() {//закрытие кранов (задержка на работу привода)

    if (ff.bits.OPENING && ff.bits.CLOSING) {
        return;
    }


    if (ff.bits.OPENING) {
        if (time_rele_power > 0) {
            time_rele_power--;
            if (time_rele_power == 0) {
                ff.bits.RELE_POWER_ON = 0;
                ff.bits.OPENED = 1;
                ff.bits.OPENING = 0;
            }
        }
    }


    if (ff.bits.CLOSING) {

        if (time_rele_gap == 0) {
            if (time_rele_power > 0) {
                ff.bits.RELE_POWER_ON = 1;
                time_rele_power--;
            } else {
                ff.bits.RELE_POWER_ON = 0;
            }
        } else {
            time_rele_gap--;
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

void start_alarm() {
    ff.bits.ALARM_ON = 1;
    ff.bits.ALARM_OFF = 0;
    ff.bits.MELODY_ON = 1;
    ff.bits.SIREN = 1;
    sec_count=0;
}

void clear_alarm() {
    ff.bits.ALARM_ON = 0;
    ff.bits.ALARM_OFF = 1;
}

void fun_work() {//работа переключателя
    {
        if (//открытие
                ff.bits.FUN_LOW &&
                !ff.bits.FUN_HIGH &&
                ff.bits.ALARM_OFF &&
                ff.bits.MOVING_ALLOWED &&
                !ff.bits.OPENED &&
                !ff.bits.OPENING) {
            beep_short_count = 1;
            open();
        };
        if (//закрытие
                ff.bits.FUN_HIGH &&
                 ff.bits.MOVING_ALLOWED &&
                !ff.bits.FUN_LOW &&
                !ff.bits.CLOSED &&
                !ff.bits.CLOSING) {
            beep_short_count = 2;
            close();
        }
    }
}

void switch_wm() {//выбор режима работы
    if (ff.bits.JUMP_LOW) {//go_alt_mode
        if (!ff.bits.UNIVERSAL_VORK_MODE_ON) {
            ff.bits.NORMAL_WORK_MODE_ON = 0;
            ff.bits.UNIVERSAL_VORK_MODE_ON = 1;
            rele_off();
            //три высоких писка
            beep_long_count = 2; //_freq pause work_time count
        }
    } else if (ff.bits.JUMP_HIGH) {//go_norm_mode
        if (!ff.bits.NORMAL_WORK_MODE_ON) {
            ff.bits.NORMAL_WORK_MODE_ON = 1;
            ff.bits.UNIVERSAL_VORK_MODE_ON = 0;
            rele_off();
            //два высоких писка
            beep_long_count = 1; //_freq pause work_time count;
        }
    }
}

void autorotation_work() {//автоповорот
    if ((time_rotation > AUTOROTATION_DELAY) &&
            !ff.bits.CLOSED &&
            !ff.bits.CLOSING &&
            ff.bits.ALARM_OFF &&
            ff.bits.MOVING_ALLOWED &&
            ff.bits.NORMAL_WORK_MODE_ON
            ) {
              go_close_short();
              beep_short_count=3;
              beep_long_count=3;
    }

    if ((time_rotation > (AUTOROTATION_DELAY + RELE_POWER_AUTOROTATION_DELAY + RELE_GAP * 2)) && //закрытие идёт указанное время
            ff.bits.CLOSED &&
            ff.bits.CLOSING &&
            ff.bits.ALARM_OFF &&
            ff.bits.MOVING_ALLOWED &&
            ff.bits.NORMAL_WORK_MODE_ON
            ) {
        go_open();
        time_rotation = 0; //обнуляем счетчик
    }

}


/*█████████████████████████████████████████████████████████████████████*/

/*TIMES*/


void minute_tick() {

    //  static unsigned minute_count = 0;

    if (time_melody > 0) {
        time_melody--;};
        
        if (time_melody == 0) {
            ff.bits.SIREN = 1;
            time_melody = MELODY_REPEAT_DELAY;
        
    };

    /*
    if (minute_count = 24 * 60 * AUTOROTATION_DAYS) {
        autorotation_work();
    }
     */

}

void sec_30_work() {
    if (ff.bits.SIREN) {
        ff.bits.SIREN = 0;
    } else {
        beep_short_count = 3;
    }
}

void sec_work() {//работа секундного таймера

#ifdef DEBUG_ENABLED
    toggle_zummer();
#endif

  

    sec_count++;
    if (!ff.bits.MOVING_ALLOWED) {
        if (time_moving_wait > 0) {
            time_moving_wait--;
        } else {
            ff.bits.MOVING_ALLOWED = 1;
        }
    }
    if (ff.bits.NORMAL_WORK_MODE_ON) {
        if (ff.bits.OPENED){
            time_rotation++;
        }
        rele_tick();
    }

    if (ff.bits.ALARM_ON) {

        if (sec_count == 30|| sec_count==60) {
            sec_30_work();
        }

        ff.bits.LED_ON = !ff.bits.LED_ON;

    } else if (ff.bits.ALARM_OFF) {

        static char iled;
        iled++;
        if (iled > 2) {
            ff.bits.LED_ON = !ff.bits.LED_ON;
            iled = 0;
        }

    }


    if (sec_count == 60) {
        minute_tick();
        sec_count = 0;
    }

}

void ms_200_work() {
    if (ff.bits.ALARM_ON) {
        if (ff.bits.SIREN) {
            beep_double();
        } else {
            if (beep_short_count > 0) {
                beep_short();
            }
        }
    } else if (ff.bits.ALARM_OFF) {


        if ((beep_short_count > 0) && (beep_long_count > 0)) {
            beep_double();
        } else {
            if (beep_short_count > 0) {
                beep_short();
            }
            if (beep_long_count > 0) {
                beep_long();
            }
        }

    }
}

void ms_100_work() {
    if (ff.bits.NORMAL_WORK_MODE_ON || ff.bits.UNIVERSAL_VORK_MODE_ON) {
        ff.bits.ALLOW_MEASURE = 1;
    } 
}

void ms_tick() {
    static unsigned ms_count = 0;
    static unsigned s_count = 0;
    ms_count++;
    if (time_tone > 0) {
        time_tone--;
        if (time_tone == 0) {
            stop_tone();
        }
    }

    ff.bits.ALLOW_FUN = 1;
    ff.bits.ALLOW_JUMP = 1;

    if (ms_count == 100) {
        ms_100_work();
        ms_200_work();
        s_count++;
        ms_count = 0;
    //PIN_ALARM_STATE_Toggle();
    }

    if (ms_count == 200) {
    //   ms_200_work();
    }

    if (s_count == 10) {
        sec_work();
        s_count = 0;
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

void zummer_switch() {//одно переключение
#ifdef DEBbUG_ENABLED
    PIN_ZUMMER_SetHigh();
    PIN_ZUMMER_SetLow();
#else
    PIN_ZUMMER_Toggle();
#endif 
}

void get_wsp() {//измерение состояния датчиков

    if (ff.bits.ALLOW_MEASURE) {

        static signed char bad_measures_counter = 0;
        PIN_POWER_MEAS_SetHigh();
        PIN_WSP_STATE_SetAnalogMode();
        __delay_ms(1);
        unsigned res = ADC_GetConversion(PIN_WSP_STATE);
        PIN_WSP_STATE_SetDigitalMode();
        PIN_POWER_MEAS_SetLow();
        if (res < BAD_WSP_VOLTAGE) {
            bad_measures_counter++;
        } else {
            if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter--;
            }
        }
        if (bad_measures_counter > WSP_MEAS_COUNT) {
            start_alarm();
            bad_measures_counter = WSP_MEAS_COUNT;
        }
        if (bad_measures_counter < -WSP_MEAS_COUNT) {
            clear_alarm();
            bad_measures_counter = -WSP_MEAS_COUNT;
        }
        ff.bits.ALLOW_MEASURE = 0;
    }
}

void get_fun() {//определение положения переключателя (антидребезг 1 шаг)

    if (ff.bits.ALLOW_FUN) {

        static signed char fun_counter;
        PIN_POWER_MEAS_SetHigh();
        __delay_ms(1);
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
        ff.bits.ALLOW_FUN = 0;
    }
}

/*
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
 */

void get_jump() {//определение положения джампера (антидребезг 1 шаг)

    static signed char jump_counter;

    if (ff.bits.ALLOW_JUMP) {

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
        ff.bits.ALLOW_JUMP = 0;
    }

}

/*
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
 */

void start_setup() {//начальная настройка
    //MCC сгенерировано
    SYSTEM_Initialize(); // initialize the device
    INTERRUPT_GlobalInterruptEnable(); // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable(); // Enable the Peripheral Interrupts
    // end MCC
    // get_eeprom();
    eeprom_set();

    TMR0_SetInterruptHandler(zummer_switch);
    TMR2_SetInterruptHandler(ms_tick);
    TMR2_StartTimer(); //начать секундный счет

    /*
    //отключение аналоговых входов чтобы не мешали измерениям
    PIN_WSP_STATE_SetDigitalMode();
    PIN_JUMP_STATE_SetDigitalMode();
    PIN_FUN_STATE_SetDigitalMode();

    //сброс состояния
    PIN_JUMP_STATE_ResetPullup();
    PIN_JUMP_STATE_SetDigitalInput();
    PIN_FUN_STATE_ResetPullup();
    PIN_FUN_STATE_SetDigitalInput();
     
    PIN_ALARM_STATE_SetDigitalOutput();
     */

    INTCONbits.TMR0IE = 0; //выкл зуммер
    ff.value = 0;

    PIN_RELE_POWER_SetLow();
    PIN_RELE_CONTROL_SetLow();
    PIN_ALARM_STATE_SetLow();
    PIN_POWER_MEAS_SetLow();
    PIN_ZUMMER_SetLow();
    PIN_LED_SetLow();

    time_rotation = 0;
    time_rele_power = 0;
    time_rele_control = 0;
    time_rele_gap = 0;
    time_tone = 0;


    // time_siren = 0;
    // time_silent = 0;
    time_melody = 0;
    time_zummer_short = 0;
    time_zummer_long = 0;

    /*ms_div*/
    time_meas = 0;
}

#ifdef DEBUG_ENABLED

void flag_error() {
    if (
            (ff.bits.ALARM_OFF && ff.bits.ALARM_ON) ||
            ((ff.bits.ALLOW_FUN + ff.bits.ALLOW_JUMP + ff.bits.ALLOW_MEASURE) > 1) ||
            ((ff.bits.CLOSED + ff.bits.CLOSING + ff.bits.OPENED + ff.bits.OPENING) > 1) ||
            (ff.bits.FUN_HIGH && ff.bits.FUN_LOW) ||
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
        CLRWDT();


        hardware_work();



        if (!ff.bits.ALARM_ON) {

            get_jump();
            switch_wm();

            get_fun();
            fun_work();

            get_wsp();

               autorotation_work();

        } else {
            close();
        };


    }
}