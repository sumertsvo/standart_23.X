#include <stdlib.h>

//-----------------------------------------------------

void EEPROM_WriteByte(unsigned char addr, unsigned char dt);

unsigned char EEPROM_ReadByte(unsigned char addr);

void EEPROM_WriteWord(unsigned char addr, unsigned int ucData);

unsigned int EEPROM_ReadWord(unsigned char addr);

void EEPROM_Writeint24(unsigned char addr, __uint24 ucData);

__uint24 EEPROM_Readint24(unsigned char addr);

void EEPROM_WriteDword(unsigned char addr, unsigned long ucData);

unsigned long EEPROM_ReadDword(unsigned char addr);

void EEPROM_WriteString(unsigned char addr, char* str1);

void EEPROM_ReadString(unsigned char addr, char* str1, unsigned char sz);

//-----------------------------------------------------