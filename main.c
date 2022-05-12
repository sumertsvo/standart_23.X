#include "mcc_generated_files/mcc.h"
#include "eeprom.h"

//настройка компиляции
//
#define DEBUG_ENABLED
//#define doubledd
//~настройка компиляции


//SETUP 
#define VERSION 1
#define ROTATION_DAYS 14 //дней до поворота крана
const unsigned LOW_WATER_RESISTANSE = 20000; //сопротивление датчика
const unsigned HIGH_WATER_RESISTANSE = 25000; //
const unsigned UP_RESISTANSE = 20000; //сопротивление делителя

#define LOW_PIN_VOLTAGE 25 // "низкий логический уровень"

//защита от дребезга
#define WSP_MEAS_COUNT 8    //количество измерений датчика

#ifdef doubledd
#define FUN_MEAS_COUNT 10   //количество измерений переключателя
#define JUMP_MEAS_COUNT 10  //количество измерений джампера
#else
#define MEAS_COUNT 10
#endif
//~защита от дребезга


#ifdef DEBUG_ENABLED

#define RELE_TIME 10// sec
#define RELE_TIME_SHORT 2// sec
#define RELE_GAP 2 // sec
const unsigned BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256));
const unsigned GOOD_WSP_VOLTAGE = (HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256));
#define ROTATION_TIME  120 //sec



#else

#define RELE_TIME 120 // sec
#define RELE_TIME_SHORT 10// sec
#define RELE_GAP 1 //sec
const unsigned BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256));
const unsigned GOOD_WSP_VOLTAGE = (HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256));
const unsigned ROTATION_TIME = (ROTATION_DAYS * 24 * 60 * 60); //D*H*M*S

#endif

//END SETUP

//Флаги

//______________________________________________________________________________
struct f_field {
    unsigned ALARM : 1;
    unsigned NORMAL_WORK_MODE : 1;
    unsigned _FUN_CONNECTED : 1;
    unsigned _JUMP_CONNECTED : 1;
    unsigned ALLOW_MEASURE : 1;
    unsigned RELE_POWER_WAIT : 1;
    unsigned RELE_CONTROL_WAIT : 1;
    unsigned OPENING : 1;
    unsigned OPENED : 1;
    unsigned CLOSING : 1;
    unsigned CLOSED : 1;
    unsigned BEEP_SHORT_ON : 1;
    unsigned BEEP_LONG_ON : 1;
    unsigned ZUMM_ON : 1;
    unsigned WATER_TRUE : 1;
    unsigned WATER_FALSE : 1;
    unsigned : 1;
};

static union {
    unsigned value;
    struct f_field bits;
} FF;
//______________________________________________________________________________
char FRIMWARE_VERSION_EEPROM_ADR;

//TIMES
//sec_div
__uint24 time_rotation; //время до автоповорота (сек)
unsigned time_rele_power; //время до закрытия реле (сек)
unsigned time_rele_control;
unsigned time_rele_gap;
unsigned time_led;
unsigned time_zummer; //s
unsigned time_zummer_short; //ms
unsigned time_zummer_long; //ms


//ms_div
char time_meas;
//~TIMES

char beep_short_count;
char beep_long_count;


#ifdef DEBUG_ENABLED

void toggle_zummer() {
    PIN_ZUMMER_SetHigh();
    PIN_ZUMMER_SetLow();
}
#endif



//______________________________________________________________________________
//______________________________________________________________________________
//______________________________________________________________________________


void timer0_switch() {//одно переключение
    PIN_ZUMMER_Toggle();
}

void start_tone() {//вкл/выкл зуммер
    INTCONbits.TMR0IE = 1;
    FF.bits.ZUMM_ON = 1;
}

void stop_tone() {//вкл/выкл зуммер
    INTCONbits.TMR0IE = 0;
    FF.bits.ZUMM_ON = 0;
    PIN_ZUMMER_SetLow();
}

void beep_short(char count) { //короткий писк
    start_tone();
    FF.bits.BEEP_SHORT_ON = 1;
    time_zummer_short = 30;
    beep_short_count = count;
}

void beep_long(char count) {//короткий писк
    start_tone();
    FF.bits.BEEP_LONG_ON = 1;
    time_zummer_long = 130;
    beep_long_count = count;
}

void go_close() {//начало закрытия кранов
    time_rotation = 0;
    PIN_RELE_CONTROL_SetHigh();
    FF.bits.CLOSING = 1;
    __delay_ms(RELE_GAP * 1000);
    PIN_RELE_POWER_SetHigh();
    time_rele_power = RELE_TIME;
    FF.bits.RELE_POWER_WAIT = 1;
    FF.bits.RELE_CONTROL_WAIT = 1;
    return;
}

void go_close_short() {//начало закрытия кранов при автоповороте
    time_rotation = 0;
    PIN_RELE_CONTROL_SetHigh();
    FF.bits.CLOSING = 1;
    __delay_ms(RELE_GAP * 1000);
    PIN_RELE_POWER_SetHigh();
    time_rele_power = RELE_TIME_SHORT;
    FF.bits.RELE_POWER_WAIT = 1;
    FF.bits.RELE_CONTROL_WAIT = 1;
    return;
}

void go_open() {//начало открытия кранов
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
    time_rele_power = RELE_TIME;
    FF.bits.RELE_POWER_WAIT = 1;
    return;
}

void go_close_alt() {//закрытие кранов 2 режим
    FF.bits.CLOSED = 1;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
}

void go_open_alt() {//открытие кранов 2 режим
    FF.bits.CLOSED = 0;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetLow();
    return;
}

void start_alarm() {//обнаружена протечка
    FF.bits.ALARM = 1;
    PIN_ALARM_STATE_SetHigh();
    INTCONbits.TMR0IE = 1; //вкл зуммер
    if (FF.bits.NORMAL_WORK_MODE) {
        go_close();
    } else {
        go_close_alt();
    }
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

#ifdef doubledd
    if (fun_counter > FUN_MEAS_COUNT) {
        fun_counter = FUN_MEAS_COUNT;
        FF.bits._FUN_CONNECTED = 0;
    } else if (fun_counter<-FUN_MEAS_COUNT) {
        fun_counter = -FUN_MEAS_COUNT;
        FF.bits._FUN_CONNECTED = 1;
    }
#else
    if (fun_counter > MEAS_COUNT) {
        fun_counter = MEAS_COUNT;
        FF.bits._FUN_CONNECTED = 0;
    } else if (fun_counter<-MEAS_COUNT) {
        fun_counter = -MEAS_COUNT;
        FF.bits._FUN_CONNECTED = 1;
    }
#endif
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



#ifdef doubledd 
        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            FF.bits._FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            FF.bits._FUN_CONNECTED = 1;
            flag = 1;
        }
#else
        if (fun_counter > MEAS_COUNT) {
            fun_counter = MEAS_COUNT;
            FF.bits._FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-MEAS_COUNT) {
            fun_counter = MEAS_COUNT;
            FF.bits._FUN_CONNECTED = 1;
            flag = 1;
        }
#endif

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

#ifdef doubledd
    if (jump_counter > JUMP_MEAS_COUNT) {
        jump_counter = JUMP_MEAS_COUNT;
        FF.bits._JUMP_CONNECTED = 0;
    } else if (jump_counter<-JUMP_MEAS_COUNT) {
        jump_counter = -JUMP_MEAS_COUNT;
        FF.bits._JUMP_CONNECTED = 1;
    }
#else
    if (jump_counter > MEAS_COUNT) {
        jump_counter = MEAS_COUNT;
        FF.bits._JUMP_CONNECTED = 0;
    } else if (jump_counter<-MEAS_COUNT) {
        jump_counter = -MEAS_COUNT;
        FF.bits._JUMP_CONNECTED = 1;
    }
#endif

    return;
}

void get_jump_full() {//определение положения переключателя (антидребезг все шаги)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetAnalogMode();
    char flag = 0;
    do {
        
        if (PIN_JUMP_STATE_GetValue()) jump_counter++;
        else jump_counter--;


#ifdef doubledd
        if (jump_counter > JUMP_MEAS_COUNT) {
            jump_counter = JUMP_MEAS_COUNT;
            FF.bits._JUMP_CONNECTED = 0;
            flag = 1;
        } else if (jump_counter<-JUMP_MEAS_COUNT) {
            jump_counter = -JUMP_MEAS_COUNT;
            FF.bits._JUMP_CONNECTED = 1;
            flag = 1;
        }
#else
        if (jump_counter > MEAS_COUNT) {
            jump_counter = MEAS_COUNT;
            FF.bits._JUMP_CONNECTED = 0;
            flag = 1;
        } else if (jump_counter<-MEAS_COUNT) {
            jump_counter = -MEAS_COUNT;
            FF.bits._JUMP_CONNECTED = 1;
            flag = 1;
        }
#endif


    } while (flag == 0);
    PIN_JUMP_STATE_SetDigitalMode();
}

void rele_tick() {//закрытие кранов (задержка на работу привода)
    toggle_zummer();
    if (FF.bits.RELE_POWER_WAIT) {//если работает силовое реле
        if (time_rele_power > 0) { //время до закрытия
            time_rele_power--;
        } else {
            if (FF.bits.RELE_CONTROL_WAIT) {//если реле активно то закрываемся
                PIN_RELE_POWER_SetLow();
                __delay_ms(RELE_GAP * 1000);
                PIN_RELE_CONTROL_SetLow();
                FF.bits.CLOSING = 0;
                FF.bits.CLOSED = 1;
                FF.bits.RELE_CONTROL_WAIT = 0;
                FF.bits.RELE_POWER_WAIT = 0;
            } else {//если не активно то открываемся
                PIN_RELE_POWER_SetLow();
                FF.bits.OPENING = 0;
                FF.bits.CLOSED = 0;
                FF.bits.RELE_POWER_WAIT = 0;
            }
        }
    }
}

void sec_tick_work() {//работа секундного таймера
#ifdef DEBUG_ENABLED
    //   switch_zum();
#endif
    if (!FF.bits.CLOSED) time_rotation++;
    rele_tick();

    if (FF.bits.ALARM) {
        PIN_LED_Toggle();

#ifdef DEBUG_ENABLED
        toggle_zummer();
#endif
    } else {//if not alarm

        static char iled;
        iled++;
        if (iled > 2) {
            PIN_LED_Toggle();
            iled = 0;
        }
    }
}

void povorot() {//автоповорот
    if ((time_rotation > ROTATION_TIME) &&
            !FF.bits.CLOSED &&
            !FF.bits.CLOSING &&
            !FF.bits.ALARM &&
            FF.bits.NORMAL_WORK_MODE
            ) {
        go_close_short();
    }
    if ((time_rotation > (ROTATION_TIME + RELE_TIME + RELE_GAP * 2)) && //закрытие идёт указанное время
            FF.bits.CLOSED &&
            FF.bits.CLOSING &&
            FF.bits.ALARM == 0 &&
            FF.bits.NORMAL_WORK_MODE
            ) {
        go_open();
        time_rotation = 0; //обнуляем счетчик
    }
}

void fun_work() {//работа переключателя
    {//todo 
        if (FF.bits._FUN_CONNECTED &&//открытие
                !FF.bits.ALARM &&
                FF.bits.CLOSED &&
                !FF.bits.RELE_POWER_WAIT) {
            //один низкий писк
            beep_short(1);

            if (FF.bits.NORMAL_WORK_MODE) go_open();
            else go_open_alt();
        };
        if (!FF.bits._FUN_CONNECTED &&//закрытие
                !FF.bits.CLOSED &&
                !FF.bits.RELE_POWER_WAIT) {
            //два низких писка
               beep_short(2);

            if (FF.bits.NORMAL_WORK_MODE) go_close();
            else go_close_alt();
        }
    }
}

void switch_wm() {//выбор режима работы
    if (FF.bits._JUMP_CONNECTED) {//go_alt_mode
        if (FF.bits.NORMAL_WORK_MODE) {
            FF.bits.NORMAL_WORK_MODE = 0;
            //   if (FLAGS.bits.CLOSED) go_close_alt();
            //три высоких писка
               beep_long(2); //_freq pause work_time count
        }
    } else {//go_norm_mode
        if (!FF.bits.NORMAL_WORK_MODE) {
            FF.bits.NORMAL_WORK_MODE = 1;
            //    if (FLAGS.bits.CLOSED) go_close();
            //два высоких писка
               beep_long(1); //_freq pause work_time count;
        }
    }
}

/*
void get_voltage() {
    unsigned res = ADC_GetConversion(channel_FVR);
    if (res > 46200) {
        for (char q = 0; q < 0x10; q++) {
            char buf = EEPROM_ReadByte(q);
            if (buf != FRIMWARE_VERSION_EEPROM_ADR) EEPROM_WriteByte(q, FRIMWARE_VERSION_EEPROM_ADR);
        }
        __uint24 buf = time_rotation;
        for (char q = FRIMWARE_VERSION_EEPROM_ADR; q < FRIMWARE_VERSION_EEPROM_ADR + 12; q += 4) {
            EEPROM_WriteShortLong(q, buf);
        }
    }
}
//*/
void ms_tick() {
    static unsigned tick_count = 0;
    tick_count++;

    if(FF.bits.BEEP_SHORT_ON){
        time_zummer_short--;
    }
    if(FF.bits.BEEP_LONG_ON){
        time_zummer_long--;
    }
    if (tick_count == 100) {
        FF.bits.ALLOW_MEASURE = 1;
        tick_count = 0;
    }
    
    if (tick_count == 1000) {
        sec_tick_work();
        tick_count = 0;
    }
}

/*
void get_adr() {
    char buf = 0;
    char adr[8][2] = {};

    for (unsigned char i = 0; i < 8; i++) {//цикл по EEPROM
        buf = EEPROM_ReadByte(i);
        if (buf == 0) continue;
        for (unsigned char q = 0; q < 8; q++) {//цикл по адресам если уже есть значение
            if (buf == adr[q][0]) {
                (adr[q][1])++;
                buf = 0;
            }
        }
        //блок обработки ошибки
        if (buf != 0) {
            for (unsigned char q = 0; q < 8; q++)//цикл по адресам для добавления нового
                if (adr[q][0] == 0) {
                    adr[q][0] = buf;
                    adr[q][1] = 1;
                    buf = 0;
                    break;
                }
        }
    }
    buf = 0;
    for (unsigned char i = 0; i < 8; i++) {
        if (adr[i][1] > adr[buf][1]) buf = i;
    }
    START_EEPROM_ADR = adr[buf][0];
    if (START_EEPROM_ADR == 0 || START_EEPROM_ADR == 0xFF) START_EEPROM_ADR = 0x10;
    //конец блока адреса
}

void get_time() {
    //блок значения
    char adr_error = 0;
    char buf = 0;
    __uint24 buf2 = 0;
    __uint24 times[3] = {};
    char time_count[3] = {};
    for (unsigned char i = START_EEPROM_ADR; i < START_EEPROM_ADR + 0x10; i += 4) {//цикл по EEPROM
        buf2 = EEPROM_ReadShortLong(i);

        for (char q = 0; q < 3; q++) {//цикл по значениям если уже есть значение
            if (buf2 == times[q]) {
                time_count[q]++;
                buf2 = 0;
            }
        }
        //блок обработки ошибки
        if (buf2 != 0) {
            adr_error++;
            for (unsigned char q = 0; q < 3; q++)//цикл по значениям для добавления нового
                if (times[q] == 0) {
                    times[q] = buf;
                    time_count[q] = 1;
                    buf = 0;
                    break;
                }
        }
    }
    buf = 0;
    for (unsigned char q = 0; q < 3; q++) {
        if (time_count[q] > time_count[buf]) buf = q;
    }
    time_s = times[buf];
    //смещение нового адреса
    if (adr_error > 1) START_EEPROM_ADR += 0x10;
}

void get_eeprom() {
    get_adr();
    get_time();
}
//*/
void eeprom_set() {
    char adres = EEPROM_ReadByte(FRIMWARE_VERSION_EEPROM_ADR);
    if (adres == 0xFF) {
        EEPROM_WriteByte(FRIMWARE_VERSION_EEPROM_ADR, VERSION);
    }
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
    FF.value = 0;
    FF.bits.ALARM = 0;
    PIN_RELE_POWER_SetLow();
    PIN_RELE_CONTROL_SetLow();
    PIN_ALARM_STATE_SetLow();
    PIN_ALARM_STATE_SetDigitalOutput();

    //проверка текущего режима
    //  get_jump_full();
    //   get_fun_full();
    time_rele_power = 0;
}

void main(void) {

    start_setup();

    while (1) {
        if 
        //        get_voltage(); //  deleted, unused
        if (!FF.bits.ALARM) {

            get_jump();
            switch_wm();

            get_fun();
            fun_work();

            if (FF.bits.ALLOW_MEASURE) {
                get_measure();
                FF.bits.ALLOW_MEASURE =0;
            }
            povorot();
            CLRWDT(); // 
        };
    }
}