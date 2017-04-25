/* 
 * File:   main.c
 * Author: mtiutiu
 *
 * Created on April 24, 2017, 7:19 PM
 */

/*
 * Simple capsense application based on AN1298 from Microchip
 * http://ww1.microchip.com/downloads/en/AppNotes/01298A.pdf
 * 
 * Compiler used: SDCC
 */

#define NO_BIT_DEFINES
#include <pic12f1572.h>
#include <stdint.h>

static __code uint16_t __at(_CONFIG1) configword1 = _FOSC_INTOSC & _WDTE_OFF & _BOREN_ON & _LVP_ON;

#define USART_DEBUG
#define USART_DEBUG_BUFF_SIZE               4
#define TOUCH_SENSOR_SAMPLES                50
#define SENSOR_TOUCHED_DELTA_THRESHOLD      20

#define abs(val)    (val < 0 ? -val : val)

#ifdef USART_DEBUG
char buff[USART_DEBUG_BUFF_SIZE];

#define NUMBER_OF_DIGITS 16   /* space for NUMBER_OF_DIGITS + '\0' */

void uitoa(uint16_t value, char* string, uint16_t radix) {
    uint8_t index, i;

    index = NUMBER_OF_DIGITS;
    i = 0;

    do {
        string[--index] = '0' + (value % radix);
        if (string[index] > '9') string[index] += 'A' - ':'; /* continue with A, B,.. */
        value /= radix;
    } while (value != 0);

    do {
        string[i++] = string[index++];
    } while (index < NUMBER_OF_DIGITS);

    string[i] = 0; /* string terminator */
}

void itoa(int16_t value, char* string, int16_t radix) {
    if (value < 0 && radix == 10) {
        *string++ = '-';
        uitoa(-value, string, radix);
    } else {
        uitoa(value, string, radix);
    }
}
#endif 

static void config_internal_osc(void) {
    OSCCONbits.IRCF = 0b1110; // internal HF osc at 8MHz
    //OSCCONbits.SCS = 0;
    OSCCONbits.SPLLEN = 1; // enable PLL x 4 -> Fosc is now 32MHz
    while (!OSCSTATbits.HFIOFR); // wait until internal HF osc is ready
}

#ifdef USART_DEBUG

static void config_usart(void) {
    TXSTAbits.TXEN = 1; // enable usart tx
    TXSTAbits.SYNC = 0; // enable async mode
    RCSTAbits.SPEN = 1; // enable rx/tx port pins
    BAUDCONbits.BRG16 = 1; // 16 bit baudrate generator
    TXSTAbits.BRGH = 0; // low speed async mode
    SPBRG = 51; // 9600 bps
}
#endif

static void config_adc(void) {
    ADCON0bits.CHS = 3; // AN3 channel selected
    ADCON1bits.ADFM = 1; // right justified result format from ADC
    ADCON1bits.ADCS = 0; // ADC clock is Fosc/2
    ADCON1bits.ADPREF = 0; // ADC positive Vref is connected to Vdd
}

#ifdef USART_DEBUG

static void usart_tx_char(const char c) {
    //while(!PIR1bits.TXIF);
    while (!TXSTAbits.TRMT);
    TXREG = c;
}

static void usart_tx_string(const char* str) {
    while (*str) {
        usart_tx_char(*str++);
    }
}

static void usart_tx_line(const char* str) {
    usart_tx_string(str);
    usart_tx_string("\r\n");
}
#endif

static void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us; i++);
}

static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < (ms * 320); i++);
}

static uint16_t read_adc(void) {
    ADCON0bits.ADON = 1; // enable ADC
    delay_us(50);
    ADCON0bits.GO = 1; // start a conversion
    while (ADCON0bits.GO_NOT_DONE);

    return (ADRESL | (ADRESH << 8));
}

static uint16_t read_touch_sensor1(void) {
    // prepare RA2 as ADC sampling capacitor charger to Vdd
    TRISAbits.TRISA2 = 0; // RA2 as output
    ANSELAbits.ANSA2 = 0; // RA2 disable analog
    WPUAbits.WPUA2 = 0; // disable RA2 pull-up

    // write logic one to RA2 for next step(tie RA2 to Vdd)
    LATAbits.LATA2 = 1;

    // AN2 channel selected - 
    //  charge ADC sampling capacitor from Vdd by connecting to RA2
    ADCON0bits.CHS = 2;
    delay_us(50);

    // ground sensor line(RA4)
    TRISAbits.TRISA4 = 0; // RA4 as output
    ANSELAbits.ANSA4 = 0; // RA4 disable analog
    LATAbits.LATA4 = 0; // write logic zero to RA4(short touch sensor to GND)

    // set sensor line (RA4) as input
    TRISAbits.TRISA4 = 1; // RA4 as input
    ANSELAbits.ANSA4 = 1; // RA4 as analog input

    // AN3 channel selected - read touch sensor
    ADCON0bits.CHS = 3;
    // perform actual reading
    return read_adc();
}

static uint16_t read_touch_sensor2(void) {

    // prepare RA4 as ADC sampling capacitor charger to Vdd
    TRISAbits.TRISA4 = 0; // RA4 as output
    ANSELAbits.ANSA4 = 0; // RA4 disable analog
    WPUAbits.WPUA4 = 0; // disable RA4 pull-up

    // write logic one to RA4 for next step(tie RA4 to Vdd)
    LATAbits.LATA4 = 1;

    // AN3 channel selected - 
    //  charge ADC sampling capacitor from Vdd by connecting to RA4
    ADCON0bits.CHS = 3;
    delay_us(50);

    // ground sensor line(RA2)
    TRISAbits.TRISA2 = 0; // RA2 as output
    ANSELAbits.ANSA2 = 0; // RA2 disable analog
    LATAbits.LATA2 = 0; // write logic zero to RA2(short touch sensor to GND)

    // set sensor line (RA2) as input
    TRISAbits.TRISA2 = 1; // RA2 as input
    ANSELAbits.ANSA2 = 1; // RA2 as analog input

    // AN2 channel selected - read touch sensor
    ADCON0bits.CHS = 2;

    // perform actual reading
    return read_adc();
}

static void check_touch_sensor1(void) {
    static int16_t last_touch_avg1 = 0;
    int16_t current_touch_avg1 = 0;

    for (uint8_t i = 0; i < TOUCH_SENSOR_SAMPLES; i++) {
        current_touch_avg1 += read_touch_sensor1() / TOUCH_SENSOR_SAMPLES;
    }

    if (abs(last_touch_avg1 - current_touch_avg1) >= SENSOR_TOUCHED_DELTA_THRESHOLD) {
#ifdef USART_DEBUG
        usart_tx_char('1');
        usart_tx_line(" - touched!");
#endif
    }
    
    last_touch_avg1 = current_touch_avg1;
    
    //uitoa(abs(last_touch_avg1 - current_touch_avg1), buff, 10);
    //usart_tx_line(buff);
}

static void check_touch_sensor2(void) {
    static int16_t last_touch_avg2 = 0;
    int16_t current_touch_avg2 = 0;

    for (uint8_t i = 0; i < TOUCH_SENSOR_SAMPLES; i++) {
        current_touch_avg2 += read_touch_sensor2() / TOUCH_SENSOR_SAMPLES;
    }

    if (abs(last_touch_avg2 - current_touch_avg2) >= SENSOR_TOUCHED_DELTA_THRESHOLD) {
#ifdef USART_DEBUG
        usart_tx_char('2');
        usart_tx_line(" - touched!");
#endif
    }

    last_touch_avg2 = current_touch_avg2;
    
    //uitoa(abs(last_touch_avg2 - current_touch_avg2), buff, 10);
    //usart_tx_line(buff);
}

void main(void) {
    config_internal_osc();
#ifdef USART_DEBUG
    config_usart();
#endif
    config_adc();

    while (1) {
        check_touch_sensor1();
        check_touch_sensor2();
    }
}

