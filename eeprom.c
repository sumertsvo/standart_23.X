#include "mcc_generated_files/mcc.h"




void EEPROM_WriteByte(unsigned char addr, unsigned char dt) {
    unsigned char status;
    while (WR);
    status = GIE;
    GIE = 0;
    EEADR = addr;
    EEDATA = dt;
    WREN = 1;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    WR = 1;
    GIE = status;
    WREN = 0;
}

unsigned char EEPROM_ReadByte(unsigned char addr) {
    while (RD || WR);
    EEADR = addr;
    RD = 1;
    return EEDATA;
}

void EEPROM_WriteWord(unsigned char addr, unsigned int ucData) {
    EEPROM_WriteByte(addr, (unsigned char) ucData);
    unsigned char dt = ucData >> 8;
    EEPROM_WriteByte(addr + 1, dt);
}

unsigned int EEPROM_ReadWord(unsigned char addr) {
    unsigned int dt = EEPROM_ReadByte(addr + 1)*256;
    dt += EEPROM_ReadByte(addr);
    return dt;
}

void EEPROM_WriteShortLong(unsigned char addr,  __uint24 ucData) {
    EEPROM_WriteByte(addr, (unsigned char) ucData);
     char dt = ucData >> 8;
    EEPROM_WriteByte(addr + 1, dt);
    dt = ucData >> 16;
    EEPROM_WriteByte(addr + 2, dt);
}

__uint24 EEPROM_ReadShortLong( char addr) {
    __uint24  dt = EEPROM_ReadByte(addr + 2)*256*256;
    dt += EEPROM_ReadByte(addr + 1)*256;
    dt += EEPROM_ReadByte(addr);
    return dt;
}

void EEPROM_WriteDword(unsigned char addr, unsigned long ucData) {
    EEPROM_WriteWord(addr, (unsigned int) ucData);
    unsigned int dt = ucData >> 16;
    EEPROM_WriteWord(addr + 2, dt);
}

unsigned long EEPROM_ReadDword(unsigned char addr) {
    unsigned long dt = EEPROM_ReadWord(addr + 2)*65536;
    dt += EEPROM_ReadWord(addr);
    return dt;
}

void EEPROM_WriteString(unsigned char addr, char* str1) {
    unsigned char n;
    for (n = 0; str1[n] != '\0'; n++)
        EEPROM_WriteByte(addr + n, str1[n]);
}

void EEPROM_ReadString(unsigned char addr, char* str1, unsigned char sz) {
    unsigned char i;
    for (i = 0; i < sz; i++) str1[i] = EEPROM_ReadByte(addr + i);
    str1[i] = 0;
}