/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC16F1823
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
        MPLAB 	          :  MPLAB X 5.45	
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set PIN_JUMP_STATE aliases
#define PIN_JUMP_STATE_TRIS                 TRISAbits.TRISA1
#define PIN_JUMP_STATE_LAT                  LATAbits.LATA1
#define PIN_JUMP_STATE_PORT                 PORTAbits.RA1
#define PIN_JUMP_STATE_WPU                  WPUAbits.WPUA1
#define PIN_JUMP_STATE_ANS                  ANSELAbits.ANSA1
#define PIN_JUMP_STATE_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define PIN_JUMP_STATE_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define PIN_JUMP_STATE_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define PIN_JUMP_STATE_GetValue()           PORTAbits.RA1
#define PIN_JUMP_STATE_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define PIN_JUMP_STATE_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define PIN_JUMP_STATE_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define PIN_JUMP_STATE_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define PIN_JUMP_STATE_SetAnalogMode()      do { ANSELAbits.ANSA1 = 1; } while(0)
#define PIN_JUMP_STATE_SetDigitalMode()     do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set PIN_LED aliases
#define PIN_LED_TRIS                 TRISAbits.TRISA4
#define PIN_LED_LAT                  LATAbits.LATA4
#define PIN_LED_PORT                 PORTAbits.RA4
#define PIN_LED_WPU                  WPUAbits.WPUA4
#define PIN_LED_ANS                  ANSELAbits.ANSA4
#define PIN_LED_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define PIN_LED_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define PIN_LED_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define PIN_LED_GetValue()           PORTAbits.RA4
#define PIN_LED_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define PIN_LED_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define PIN_LED_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define PIN_LED_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define PIN_LED_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define PIN_LED_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set PIN_ZUMMER aliases
#define PIN_ZUMMER_TRIS                 TRISAbits.TRISA5
#define PIN_ZUMMER_LAT                  LATAbits.LATA5
#define PIN_ZUMMER_PORT                 PORTAbits.RA5
#define PIN_ZUMMER_WPU                  WPUAbits.WPUA5
#define PIN_ZUMMER_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define PIN_ZUMMER_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define PIN_ZUMMER_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define PIN_ZUMMER_GetValue()           PORTAbits.RA5
#define PIN_ZUMMER_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define PIN_ZUMMER_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define PIN_ZUMMER_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define PIN_ZUMMER_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)

// get/set PIN_ALARM_STATE aliases
#define PIN_ALARM_STATE_TRIS                 TRISCbits.TRISC0
#define PIN_ALARM_STATE_LAT                  LATCbits.LATC0
#define PIN_ALARM_STATE_PORT                 PORTCbits.RC0
#define PIN_ALARM_STATE_WPU                  WPUCbits.WPUC0
#define PIN_ALARM_STATE_ANS                  ANSELCbits.ANSC0
#define PIN_ALARM_STATE_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define PIN_ALARM_STATE_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define PIN_ALARM_STATE_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define PIN_ALARM_STATE_GetValue()           PORTCbits.RC0
#define PIN_ALARM_STATE_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define PIN_ALARM_STATE_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define PIN_ALARM_STATE_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define PIN_ALARM_STATE_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define PIN_ALARM_STATE_SetAnalogMode()      do { ANSELCbits.ANSC0 = 1; } while(0)
#define PIN_ALARM_STATE_SetDigitalMode()     do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set PIN_POWER_MEAS aliases
#define PIN_POWER_MEAS_TRIS                 TRISCbits.TRISC1
#define PIN_POWER_MEAS_LAT                  LATCbits.LATC1
#define PIN_POWER_MEAS_PORT                 PORTCbits.RC1
#define PIN_POWER_MEAS_WPU                  WPUCbits.WPUC1
#define PIN_POWER_MEAS_ANS                  ANSELCbits.ANSC1
#define PIN_POWER_MEAS_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define PIN_POWER_MEAS_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define PIN_POWER_MEAS_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define PIN_POWER_MEAS_GetValue()           PORTCbits.RC1
#define PIN_POWER_MEAS_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define PIN_POWER_MEAS_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define PIN_POWER_MEAS_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define PIN_POWER_MEAS_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define PIN_POWER_MEAS_SetAnalogMode()      do { ANSELCbits.ANSC1 = 1; } while(0)
#define PIN_POWER_MEAS_SetDigitalMode()     do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set PIN_FUN_STATE aliases
#define PIN_FUN_STATE_TRIS                 TRISCbits.TRISC2
#define PIN_FUN_STATE_LAT                  LATCbits.LATC2
#define PIN_FUN_STATE_PORT                 PORTCbits.RC2
#define PIN_FUN_STATE_WPU                  WPUCbits.WPUC2
#define PIN_FUN_STATE_ANS                  ANSELCbits.ANSC2
#define PIN_FUN_STATE_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define PIN_FUN_STATE_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define PIN_FUN_STATE_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define PIN_FUN_STATE_GetValue()           PORTCbits.RC2
#define PIN_FUN_STATE_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define PIN_FUN_STATE_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define PIN_FUN_STATE_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define PIN_FUN_STATE_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define PIN_FUN_STATE_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define PIN_FUN_STATE_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set PIN_WSP_STATE aliases
#define PIN_WSP_STATE_TRIS                 TRISCbits.TRISC3
#define PIN_WSP_STATE_LAT                  LATCbits.LATC3
#define PIN_WSP_STATE_PORT                 PORTCbits.RC3
#define PIN_WSP_STATE_WPU                  WPUCbits.WPUC3
#define PIN_WSP_STATE_ANS                  ANSELCbits.ANSC3
#define PIN_WSP_STATE_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define PIN_WSP_STATE_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define PIN_WSP_STATE_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define PIN_WSP_STATE_GetValue()           PORTCbits.RC3
#define PIN_WSP_STATE_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define PIN_WSP_STATE_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define PIN_WSP_STATE_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define PIN_WSP_STATE_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define PIN_WSP_STATE_SetAnalogMode()      do { ANSELCbits.ANSC3 = 1; } while(0)
#define PIN_WSP_STATE_SetDigitalMode()     do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set PIN_RELE_CONTROL aliases
#define PIN_RELE_CONTROL_TRIS                 TRISCbits.TRISC4
#define PIN_RELE_CONTROL_LAT                  LATCbits.LATC4
#define PIN_RELE_CONTROL_PORT                 PORTCbits.RC4
#define PIN_RELE_CONTROL_WPU                  WPUCbits.WPUC4
#define PIN_RELE_CONTROL_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define PIN_RELE_CONTROL_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define PIN_RELE_CONTROL_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define PIN_RELE_CONTROL_GetValue()           PORTCbits.RC4
#define PIN_RELE_CONTROL_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define PIN_RELE_CONTROL_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define PIN_RELE_CONTROL_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define PIN_RELE_CONTROL_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)

// get/set PIN_RELE_POWER aliases
#define PIN_RELE_POWER_TRIS                 TRISCbits.TRISC5
#define PIN_RELE_POWER_LAT                  LATCbits.LATC5
#define PIN_RELE_POWER_PORT                 PORTCbits.RC5
#define PIN_RELE_POWER_WPU                  WPUCbits.WPUC5
#define PIN_RELE_POWER_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define PIN_RELE_POWER_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define PIN_RELE_POWER_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define PIN_RELE_POWER_GetValue()           PORTCbits.RC5
#define PIN_RELE_POWER_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define PIN_RELE_POWER_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define PIN_RELE_POWER_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define PIN_RELE_POWER_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/