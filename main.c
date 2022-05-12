#include "mcc_generated_files/mcc.h"
#include "eeprom.h"

//настройка компиляции
//
#define DEBUG_ENABLED
//#define doubledd
//~настройка компиляции


//SETUP 
#define ROTATION_DAYS 14 //дней до поворота крана
const unsigned LOW_WATER_RESISTANSE =20000;  //сопротивление датчика
const unsigned  HIGH_WATER_RESISTANSE =25000; //
const unsigned  UP_RESISTANSE = 20000; //сопротивление делителя
#define LOW_PIN_VOLTAGE 25 // "низкий логический уровень"

//защита от дребезга
#define WSP_MEAS_COUNT 2    //количество измерений датчика

#ifdef doubledd
#define FUN_MEAS_COUNT 10   //количество измерений переключателя
#define JUMP_MEAS_COUNT 10  //количество измерений джампера
#else
#define MEAS_COUNT 10
#endif
//~защита от дребезга


#ifdef DEBUG_ENABLED

#define RELE_TIME 10// sec
#define RELE_GAP 2 // sec
const unsigned BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256));
const unsigned GOOD_WSP_VOLTAGE = (HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256));
#define ROTATION_TIME  120 //sec



#else

#define RELE_TIME 120 // sec
#define RELE_GAP 1 //sec
const unsigned BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 256));
const unsigned GOOD_WSP_VOLTAGE = (HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 256));
const unsigned ROTATION_TIME  = (ROTATION_DAYS * 24 * 60 * 60); //D*H*M*S

#endif

//END SETUP

//Флаги

struct f_field {
    unsigned ALARM : 1;
    unsigned NORMAL_WORK_MODE : 1;
    unsigned _FUN_CONNECTED : 1;
    unsigned _JUMP_CONNECTED : 1;
    unsigned MEASURE_ON : 1;
    unsigned RELE_POWER_WAIT : 1;
    unsigned RELE_CONTROL_WAIT : 1;
    unsigned OPENING : 1;
    unsigned OPENED : 1;
    unsigned CLOSING : 1;
    unsigned CLOSED : 1;
    unsigned BEEP_ON : 1;
    unsigned BOOP_ON : 1;
    unsigned ZUMM_ON : 1;
    unsigned WATER_TRUE : 1;
    unsigned WATER_FALSE : 1;
};

static union {
    unsigned value;
    struct f_field bits;
} FLAGS;

char FRIMWARE_VERSION_EEPROM_ADR;

//TIMES
//sec_div
__uint24 time_rotation; //время до автоповорота (сек)
unsigned time_rele_power; //время до закрытия реле (сек)
unsigned time_rele_control;
unsigned time_rele_gap;
unsigned time_led;
unsigned time_zummer;
//ms_div
char time_meas;
//~TIMES



void switch_zum() {//одно переключение
    PIN_ZUMMER_SetHigh();
    PIN_ZUMMER_SetLow();
}

void toggle_tone() {//вкл/выкл зуммер
    INTCONbits.TMR0IE = ~INTCONbits.TMR0IE;
}

void beep(char time, char count) {//короткий писк
    for (char j = 0; j < count; j++) {
        for (char i = 0; i < time; i++) {
            switch_zum();
            __delay_us(300);
        }
        __delay_ms(100);
    }
}

void boop(char time, char count) {//короткий писк
    for (char j = 0; j < count; j++) {
        for (char i = 0; i < time; i++) {
            switch_zum();
            __delay_ms(1);
        }
        __delay_ms(150);
    }
}

void go_close() {//начало закрытия кранов
    time_rotation = 0;
    PIN_RELE_CONTROL_SetHigh();
    FLAGS.bits.CLOSING = 1;
    __delay_ms(RELE_GAP * 1000);
    PIN_RELE_POWER_SetHigh();
    time_rele_power = RELE_TIME;
    FLAGS.bits.RELE_POWER_WAIT = 1;
    FLAGS.bits.RELE_CONTROL_WAIT = 1;
    return;
}

void go_open() {//начало открытия кранов
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
    time_rele_power = RELE_TIME;
    FLAGS.bits.RELE_POWER_WAIT = 1;
    return;
}

void go_close_alt() {//закрытие кранов 2 режим
    FLAGS.bits.CLOSED = 1;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetHigh();
}

void go_open_alt() {//открытие кранов 2 режим
    FLAGS.bits.CLOSED = 0;
    PIN_RELE_CONTROL_SetLow();
    PIN_RELE_POWER_SetLow();
    return;
}

void start_alarm() {//обнаружена протечка
    FLAGS.bits.ALARM = 1;
    PIN_ALARM_STATE_SetHigh();
    INTCONbits.TMR0IE = 1; //вкл зуммер
    if (FLAGS.bits.NORMAL_WORK_MODE) {
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
    PIN_FUN_STATE_SetAnalogMode();
    __delay_ms(1);
    unsigned res = ADC_GetConversion(PIN_FUN_STATE);
    PIN_FUN_STATE_SetDigitalMode();
    PIN_POWER_MEAS_SetLow();
    if (res < LOW_PIN_VOLTAGE) fun_counter--;
    else fun_counter++;


#ifdef doubledd
    if (fun_counter > FUN_MEAS_COUNT) {
        fun_counter = FUN_MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 0;
    } else if (fun_counter<-FUN_MEAS_COUNT) {
        fun_counter = -FUN_MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 1;
    }
#else
    if (fun_counter > MEAS_COUNT) {
        fun_counter = MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 0;
    } else if (fun_counter<-MEAS_COUNT) {
        fun_counter = -MEAS_COUNT;
        FLAGS.bits._FUN_CONNECTED = 1;
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
        __delay_ms(1);
        unsigned res = ADC_GetConversion(PIN_FUN_STATE);
        if (res < LOW_PIN_VOLTAGE) fun_counter--;
        else fun_counter++;



#ifdef doubledd 
        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 1;
            flag = 1;
        }
#else
        if (fun_counter > MEAS_COUNT) {
            fun_counter = MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 0;
            flag = 1;
        } else if (fun_counter<-MEAS_COUNT) {
            fun_counter = MEAS_COUNT;
            FLAGS.bits._FUN_CONNECTED = 1;
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
    PIN_JUMP_STATE_SetAnalogMode();
    unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
    PIN_JUMP_STATE_SetDigitalMode();
    if (res < LOW_PIN_VOLTAGE) jump_counter--;
    else jump_counter++;

#ifdef doubledd
    if (jump_counter > JUMP_MEAS_COUNT) {
        jump_counter = JUMP_MEAS_COUNT;
        FLAGS.bits._JUMP_CONNECTED = 0;
    } else if (jump_counter<-JUMP_MEAS_COUNT) {
        jump_counter = -JUMP_MEAS_COUNT;
        FLAGS.bits._JUMP_CONNECTED = 1;
    }
#else
    if (jump_counter > MEAS_COUNT) {
        jump_counter = MEAS_COUNT;
        FLAGS.bits._JUMP_CONNECTED = 0;
    } else if (jump_counter<-MEAS_COUNT) {
        jump_counter = -MEAS_COUNT;
        FLAGS.bits._JUMP_CONNECTED = 1;
    }
#endif

    return;
}

void get_jump_full() {//определение положения переключателя (антидребезг все шаги)

    static signed char jump_counter;
    PIN_JUMP_STATE_SetAnalogMode();
    char flag = 0;
    do {
        unsigned res = ADC_GetConversion(PIN_JUMP_STATE);
        if (res < LOW_PIN_VOLTAGE) jump_counter--;
        else jump_counter++;


#ifdef doubledd
        if (jump_counter > JUMP_MEAS_COUNT) {
            jump_counter = JUMP_MEAS_COUNT;
            FLAGS.bits._JUMP_CONNECTED = 0;
            flag = 1;
        } else if (jump_counter<-JUMP_MEAS_COUNT) {
            jump_counter = -JUMP_MEAS_COUNT;
            FLAGS.bits._JUMP_CONNECTED = 1;
            flag = 1;
        }
#else
        if (jump_counter > MEAS_COUNT) {
            jump_counter = MEAS_COUNT;
            FLAGS.bits._JUMP_CONNECTED = 0;
            flag = 1;
        } else if (jump_counter<-MEAS_COUNT) {
            jump_counter = -MEAS_COUNT;
            FLAGS.bits._JUMP_CONNECTED = 1;
            flag = 1;
        }
#endif


    } while (flag == 0);
    PIN_JUMP_STATE_SetDigitalMode();
}

void rele_tick() {//закрытие кранов (задержка на работу привода)
#ifdef DEBUG_ENABLED
    switch_zum();
#endif
   
    if (FLAGS.bits.RELE_POWER_WAIT) {//если работает силовое реле
        if (time_rele_power > 0) { //время до закрытия
            time_rele_power--;
        } else {
            if (FLAGS.bits.RELE_CONTROL_WAIT) {//если реле активно то закрываемся
                PIN_RELE_POWER_SetLow();
                __delay_ms(RELE_GAP * 1000);
                PIN_RELE_CONTROL_SetLow();
                FLAGS.bits.CLOSING =0;
                FLAGS.bits.CLOSED = 1;
                FLAGS.bits.RELE_CONTROL_WAIT = 0;
                FLAGS.bits.RELE_POWER_WAIT = 0;
            } else {//если не активно то открываемся
                PIN_RELE_POWER_SetLow();
                FLAGS.bits.OPENING = 0;
                FLAGS.bits.CLOSED = 0;
                FLAGS.bits.RELE_POWER_WAIT = 0;
            }
        }
    }
}

void sec_tick_work() {//работа секундного таймера
#ifdef DEBUG_ENABLED
    //   switch_zum();
#endif
    time_rotation++;
    rele_tick();
   
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

void povorot() {//автоповорот
    if ((time_rotation > ROTATION_TIME) &&
            !FLAGS.bits.CLOSED &&
            !FLAGS.bits.CLOSING &&
            !FLAGS.bits.ALARM &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_close();
    }
    if ((time_rotation > (ROTATION_TIME + RELE_TIME + RELE_GAP * 2)) && //закрытие идёт указанное время
            FLAGS.bits.CLOSED &&
            FLAGS.bits.CLOSING &&
            FLAGS.bits.ALARM == 0 &&
            FLAGS.bits.NORMAL_WORK_MODE
            ) {
        go_open();
        time_rotation = 0; //обнуляем счетчик
    }

}

void fun_work() {//работа переключателя
    {
        if (FLAGS.bits._FUN_CONNECTED &&//открытие
                !FLAGS.bits.ALARM &&
                FLAGS.bits.CLOSED &&
                !FLAGS.bits.RELE_POWER_WAIT) {
            //один низкий писк
            beep(40, 1); 
            
            if (FLAGS.bits.NORMAL_WORK_MODE) go_open();
            else go_open_alt();
            
        };
        if (!FLAGS.bits._FUN_CONNECTED &&//закрытие
                !FLAGS.bits.CLOSED &&
                !FLAGS.bits.RELE_POWER_WAIT) {
             //два низких писка
            beep(40, 2);
            
            if (FLAGS.bits.NORMAL_WORK_MODE) go_close();
            else go_close_alt();
           
        }
    }
}

void switch_wm() {//выбор режима работы
    if (FLAGS.bits._JUMP_CONNECTED) {//go_alt_mode
        if (FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 0;
            //   if (FLAGS.bits.CLOSED) go_close_alt();
            //три высоких писка
            boop(40, 2); //_freq pause work_time count
        }
    } else {//go_norm_mode
        if (!FLAGS.bits.NORMAL_WORK_MODE) {
            FLAGS.bits.NORMAL_WORK_MODE = 1;
            //    if (FLAGS.bits.CLOSED) go_close();
            //два высоких писка
            boop(40, 1); //_freq pause work_time count;
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
void ms_tick(){
    static unsigned tick_count =0;
    tick_count++;
    
    if (tick_count == 1000){
    sec_tick_work();
    tick_count=0;
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
void start_setup() {//начальная настройка
    //MCC сгенерировано
    SYSTEM_Initialize(); // initialize the device
    INTERRUPT_GlobalInterruptEnable(); // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable(); // Enable the Peripheral Interrupts
    // end MCC
    // get_eeprom();
    TMR0_SetInterruptHandler(switch_zum);
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
    FLAGS.value = 0;
    FLAGS.bits.ALARM =0;
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
         CLRWDT(); // <2.1 сек
//        get_voltage();
        if (!FLAGS.bits.ALARM) {
            
            get_jump();
            switch_wm();
            
            get_fun();
            fun_work();
            
            povorot();
        };
    }
}