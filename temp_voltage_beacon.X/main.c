/*
 * File:   main.c
 * Author: mtiutiu
 *
 * Created on September 18, 2014, 3:11 PM
 */


// PIC12F1572 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    //  (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOREN = OFF    // Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// simple float number representation with one decimal point value
typedef struct {
    int integral;
    unsigned int floating;
} Float;

typedef unsigned char bool;

const bool TRUE = 1;
const bool FALSE = 0;

const bool DIGITAL = 0;
const bool ANALOG  = 1;

const bool OUTPUT = 0;
const bool INPUT = 1;

const bool ON = 1;
const bool OFF = 0;

const bool ADC_MODULE_OFF = TRUE;
const bool ADC_FVR_GAIN_BUFFER_OFF = TRUE;
const bool COMPARATOR_FVR_GAIN_BUFFER_OFF = TRUE;
const bool TEMPERATURE_SENSOR_OFF = TRUE;
const bool LOW_POWER_SLEEP_MODE = TRUE;

// ----------------------- LM335 TEMPERATURE SENSOR --------------------------------------------
const unsigned char TEMPERATURE_SAMPLES = 4;
const unsigned int TEMPERATURE_CORRECTION_FACTOR = 1;
const unsigned int TEMPERATURE_ADC_VREF = 3; // this should be equal to Vdd if FVR is not enabled
const unsigned int KELVIN_TO_CELSIUS_CONSTANT = 273;
const unsigned int TEMPERATURE_SENSOR_SLOPE = 10;  // sensor temperature output is K/10mv
// ------------------- END LM335 TEMPERATURE SENSOR --------------------------------------------

#define WATCHDOG_ON()   WDTCONbits.SWDTEN = TRUE
#define WATCHDOG_OFF()   WDTCONbits.SWDTEN = FALSE

#define ADC_ON()    ADCON0bits.ADON = TRUE
#define ADC_OFF()    ADCON0bits.ADON = FALSE

#define LM335_TEMPERATURE_SENSOR_ENABLE()   LATAbits.LATA5 = ON
#define LM335_TEMPERATURE_SENSOR_DISABLE()   LATAbits.LATA5 = OFF

// possible internal oscillator clock values
typedef enum {
    LF_CLOCK_31KHz_0,   // same as below: clock @31KHz
    LF_CLOCK_31KHz_1,   // Low Frequency clock @31KHz
    MF_CLOCK_31_25KHz,  // Medium Frequency clock @31.25KHz
    HF_CLOCK_31_25KHz,  // High Frequency clock @31.25KHz
    MF_CLOCK_62_5KHz,   // Medium Frequency clock @62.5KHz
    MF_CLOCK_125KHz,    // Medium Frequency clock @125KHz
    MF_CLOCK_250KHz,    // Medium Frequency clock @250KHz
    MF_CLOCK_500KHz,    // Medium Frequency clock @500KHz
    HF_CLOCK_125KHz,    // High Frequency clock @125KHz
    HF_CLOCK_250KHz,    // High Frequency clock @250KHz
    HF_CLOCK_500KHz,    // High Frequency clock @500KHz
    HF_CLOCK_1MHz,      // High Frequency clock @1MHz
    HF_CLOCK_2MHz,      // High Frequency clock @2MHz
    HF_CLOCK_4MHz,      // High Frequency clock @4MHz
    HF_CLOCK_8MHz,      // High Frequency clock @8MHz
    HF_CLOCK_16MHz,     // High Frequency clock @16MHz
    HF_CLOCK_PLL_32MHz = HF_CLOCK_8MHz  // High Frequency clock @32MHz(PLL enabled only)
} ClockSelect;

const unsigned long FOSC[] = {
    31000,      // Low Frequency clock @31KHz
    31000,      // Low Frequency clock @31KHz
    31250,      // Medium Frequency clock @31.25KHz
    31250,      // High Frequency clock @31.25KHz
    62500,      // Medium Frequency clock @62.5KHz
    12500,      // Medium Frequency clock @125KHz
    250000,     // Medium Frequency clock @250KHz
    500000,     // Medium Frequency clock @500KHz
    125000,     // High Frequency clock @125KHz
    250000,     // High Frequency clock @250KHz
    500000,     // High Frequency clock @500KHz
    1000000,    // High Frequency clock @1MHz
    2000000,    // High Frequency clock @2MHz
    4000000,    // High Frequency clock @4MHz
    8000000,    // High Frequency clock @8MHz
    16000000,   // High Frequency clock @16MHz
    32000000    // High Frequency clock @32MHz
};

// possible watchdog timer interval values
typedef enum {
    WDT_1MS,    // 1ms watchdog interval
    WDT_2MS,    // 2ms watchdog interval
    WDT_4MS,    // 4ms watchdog interval
    WDT_8MS,    // 8ms watchdog interval
    WDT_16MS,   // 16ms watchdog interval
    WDT_32MS,   // 32ms watchdog interval
    WDT_64MS,   // 64ms watchdog interval
    WDT_128MS,  // 128ms watchdog interval
    WDT_256MS,  // 256ms watchdog interval
    WDT_512MS,  // 512ms watchdog interval
    WDT_1S,     // 1s watchdog interval
    WDT_2S,     // 2s watchdog interval
    WDT_4S,     // 4s watchdog interval
    WDT_8S,     // 8s watchdog interval
    WDT_16S,    // 16s watchdog interval
    WDT_32S,    // 32s watchdog interval
    WDT_64S,    // 64s watchdog interval
    WDT_128S,   // 128s watchdog interval
    WDT_256S    // 256s watchdog interval
} WatchdogInterval;

typedef enum {
    NONE,
    ADC_MODULE,
    COMPARATOR_MODULE
} FVRModuleSelect;

typedef enum {
    FVR_GAIN_OFF,
    FVR_GAIN_1X,
    FVR_GAIN_2X,
    FVR_GAIN_4X
} FVRGainSelect;

typedef enum {
    AN0_CHANNEL,
    AN1_CHANNEL,
    AN2_CHANNEL,
    AN3_CHANNEL,
    TEMPERATURE_CHANNEL = 29,
    DAC_CHANNEL,
    FVR_CHANNEL
} ADCChannelSelect;

typedef enum  {
    FOSC_DIV_2,     // Fosc divided by 2
    FOSC_DIV_8,     // Fosc divided by 8
    FOSC_DIV_32,    // Fosc divided by 32
    FRC0,           // clock supplied from an internal RC oscillator
    FOSC_DIV_4,     // Fosc divided by 4
    FOSC_DIV_16,    // Fosc divided by 16
    FOSC_DIV_64,    // Fosc divided by 64
    FRC1            // clock supplied from an internal RC oscillator
} ADCClockSelect;

// ADC positive Vref select
typedef enum {
    VDD,                // positive Vref is connected to Vdd
    VREF_EXT_PIN = 2,   // positive Vref is connected to external Vref+ pin
    FVR                 // positive Vref is connected to internal FVR(Fixed Voltage Reference)
} ADCPVREFSelect;

// some simple float division based on our requirements(temperature readings)
void float_div(unsigned int n1, unsigned int n2, Float* result) {
    result->integral = n1 / n2;
    result->floating = abs(((n1 % n2) * 10) / n2);
}

// some simple float substraction based on our requirements(temperature readings)
void float_sub(Float* n1, unsigned int n2, Float* result) {
    if(n1->integral > n2) {
        result->integral = n1->integral - n2;
        result->floating = n1->floating;
    } else {
        result->integral = (n2 - n1->integral - 2) ^ 0xFFFF;
        result->floating = 10 - n1->floating;
    }
}

void float_add(Float* n1, unsigned int n2, Float* result) {
    result->integral = n1->integral + n2;
    result->floating = n1->floating;
}

// fixed voltage reference module gain setup
// select for which module the gain should be applied or none
void fvr_gain_setup(FVRModuleSelect module, FVRGainSelect gs) {
    if(module == ADC_MODULE) {
        FVRCONbits.ADFVR = gs;
    }

    if(module == COMPARATOR_MODULE) {
        FVRCONbits.CDAFVR = gs;
    }
}

void fvr_enable() {
    FVRCONbits.FVREN = TRUE;    // enable FVR
    while(!FVRCONbits.FVRRDY); // wait until FVR is ready
}

// the most important part which pulses the MCU - System Clock
// this function works as intended only if the MCU config words have PLL set to OFF and FOSC set to INTOSC
void system_clock_setup(ClockSelect cs, bool pll) {
    OSCCONbits.SCS1 = TRUE; // internal clock source
    OSCCONbits.SPLLEN = FALSE;
    OSCCONbits.IRCF = cs;

    if((pll == TRUE) && (cs == HF_CLOCK_PLL_32MHz)) {
        OSCCONbits.SPLLEN = TRUE;
        // we're in HF PLL clock domain
        while(!OSCSTATbits.PLLR); // wait untill PLL is ready
    }

    if((cs == HF_CLOCK_31_25KHz) || (cs >= HF_CLOCK_125KHz)) {
        // we're in HF clock domain
        while(!OSCSTATbits.HFIOFR); // wait untill HF osc is ready
    }

    if((cs == MF_CLOCK_31_25KHz) ||
       ((cs >= MF_CLOCK_62_5KHz) && (cs <=MF_CLOCK_500KHz)))
    {
        // we're in MF clock domain
        while(!OSCSTATbits.MFIOFR); // wait untill MF osc is ready
    }

    if((cs == LF_CLOCK_31KHz_0) || (cs == LF_CLOCK_31KHz_1)) {
        // we're in LF clock domain and we need also FVR to be enabled as per datasheet
        fvr_enable();
        while(!OSCSTATbits.LFIOFR); // wait untill LF osc is ready
    }
}

void ports_setup(void) {
    // ------------------ USART PORT SETUP ---------------------
    //APFCONbits.RXDTSEL = TRUE; // RX on RA5 pin
    //APFCONbits.TXCKSEL = TRUE; // TX on RA4 pin
    //TRISAbits.TRISA5 = INPUT; // RX pin as input
    //TRISAbits.TRISA4 = OUTPUT; // TX pin as output

    // if the above is not enabled, then:
    // RA1 - RX
    // RA0 - TX

    ANSELAbits.ANSA1 = DIGITAL; // RX pin as digital pin
    ANSELAbits.ANSA0 = DIGITAL; // TX pin as digital pin
    TRISAbits.TRISA1 = INPUT; // RX pin as input
    TRISAbits.TRISA0 = OUTPUT; // TX pin as output
    // ------------------ END USART PORT SETUP --------------------

    // ------------- LM335 TEMPERATURE SENSOR PORT SETUP ----------
    ANSELAbits.ANSA2 = ANALOG; // AN2 pin as analog pin
    TRISAbits.TRISA2 = INPUT; // AN2 pin as input

    TRISAbits.TRISA5 = OUTPUT; // RA5 pin as output for lm335 enable/disable
    // ------------ END LM335 TEMPERATURE SENSOR PORT SETUP -------
}

// this won't work for all the possible combinations of clock frequencies and baudrates
// so be warned...
// we use 16 bit baudrate generator as a compromise
// 2400 - 19200 baudrate range works ok
// HF_CLOCK_4MHz - HF_CLOCK_16MHz system clock range works ok
void usart_setup(unsigned int baudrate, ClockSelect cs) {
    unsigned int BRG = 0;

    // calculate baudrate generator register value
    BRG = ((FOSC[cs] / baudrate) >> 4) - 1;
    SPBRGH = (unsigned char)(BRG << 8);
    SPBRGL = (unsigned char)(BRG & 0x00FF);

    BAUDCONbits.BRG16 = TRUE; // enable 16 bit  baudrate generator
    TXSTAbits.BRGH = TRUE;    // for lower system clock frequencies
    TXSTAbits.SYNC = FALSE; // asynchronous operation
    RCSTAbits.SPEN = TRUE;  // enable serial port TX/RX pins
    TXSTAbits.TXEN = TRUE;  // enable transmitter
    RCSTAbits.CREN = TRUE;  // enable receiver
}

void adc_setup(ADCClockSelect cs, ADCPVREFSelect pvref, FVRGainSelect gs) {
    // if fixed voltage reference is selected then we have to make sure that it's enabled
    if(pvref == FVR) {
        fvr_enable();
        fvr_gain_setup(ADC_MODULE, gs);
    }

    ADCON1bits.ADCS = cs;       // ADC clock select
    ADCON1bits.ADPREF = pvref;  // ADC positive Vref select
    ADCON1bits.ADFM = TRUE;     // ADC result is right justified
    ADC_ON();     // ADC enabled
}

unsigned int read_adc(ADCChannelSelect channel) {
    ADCON0bits.CHS = channel; // select ADC channel to perform conversion on
    _delay(10000);  // give it a break
    ADCON0bits.GO_nDONE = TRUE; // start ADC conversion
    while(!PIR1bits.ADIF);      // wait for ADC conversion to finish

    return (ADRESL | (ADRESH << 8));
}

void read_lm335_temperature(ADCChannelSelect cs,
                            unsigned char samples,
                            int correction_factor,
                            unsigned int temp_adc_vref,
                            Float* temp_result)
{
    unsigned int avg_adc = 0;

    // average values read from ADC for better readings
    for(unsigned char i = 0; i < samples; i++) {
        avg_adc += read_adc(cs) / samples;
    }

    // select adc vref based on fvr status
    float_div((avg_adc * (FVRCONbits.FVREN ? (FVRCONbits.ADFVR + 1) : temp_adc_vref)),
                TEMPERATURE_SENSOR_SLOPE,
                temp_result
    );

    // temperature value resulted from voltage ADC conversion is in Kelvin - need Celsius
    float_sub(temp_result, KELVIN_TO_CELSIUS_CONSTANT, temp_result);

    // apply a small correction factor to the result
    float_add(temp_result, correction_factor, temp_result);
}

void usart_tx_byte(char byte) {
    TXSTAbits.TXEN = TRUE;  // enable transmitter
    while(!PIR1bits.TXIF); // wait for tx buffer to be empty(ready for next transmission)
    TXREG = byte;   // send character
}

bool usart_rx_available() {
    return PIR1bits.RCIF;
}

char usart_rx_byte(void) {
    char recv = 0;

    RCSTAbits.CREN = TRUE;  // enable receiver
    while(!usart_rx_available());  // wait for receive complete

    recv = RCREG; // get received char

    // if overrun detected handle it
    if(RCSTAbits.OERR) {
        RCSTAbits.CREN = FALSE;
    }

    return recv;
}

void usart_tx_line(const char* string) {
    while(*string) {
        usart_tx_byte(*string++);
    }
    usart_tx_byte('\r');
    usart_tx_byte('\n');
}

/*void usart_rx_line(char* line) {
    char recv_char = 0;

    if(line == NULL) {
        return;
    }

    while((recv_char = usart_rx_byte()) != 0 ||
            recv_char != '\r' ||
            recv_char != '\n')
    {
        *line++ = recv_char;
    }
}*/

// this works only when MCU configuration bit WDTE = SWDTEN
void go_to_sleep(WatchdogInterval interval,
                 bool adc_off,
                 bool adc_fvr_gain_buffer_off,
                 bool comparator_gain_fvr_buffer_off,
                 bool temperature_sensor_off,
                 bool low_power_sleep_mode)
{
    unsigned char adc_module_fvr_gain_buffer_previous_state;
    unsigned char comparator_module_fvr_gain_buffer_previous_state;

    if(adc_off) {
        ADC_OFF();
    }

    if(adc_fvr_gain_buffer_off) {
        adc_module_fvr_gain_buffer_previous_state = FVRCONbits.ADFVR;
        fvr_gain_setup(ADC_MODULE, FVR_GAIN_OFF);
    }

    if(comparator_gain_fvr_buffer_off) {
        comparator_module_fvr_gain_buffer_previous_state = FVRCONbits.CDAFVR;
        fvr_gain_setup(COMPARATOR_MODULE, FVR_GAIN_OFF);
    }

    if(temperature_sensor_off) {
        LM335_TEMPERATURE_SENSOR_DISABLE();
    }

    if(low_power_sleep_mode) {
        VREGCONbits.VREGPM = TRUE;
    }

    WATCHDOG_OFF(); // turn off watchdog for sleep interval setup
    WDTCONbits.WDTPS = interval;
    WATCHDOG_ON();  // start watchdog before going to sleep so that we can wake up

    SLEEP(); // go to sleep

    // restore gain buffers previous states on wake up
    fvr_gain_setup(ADC_MODULE, adc_module_fvr_gain_buffer_previous_state);
    fvr_gain_setup(COMPARATOR_MODULE, comparator_module_fvr_gain_buffer_previous_state);

    ADC_ON(); // on wake up turn ADC back on
    LM335_TEMPERATURE_SENSOR_ENABLE(); // on wake up turn LM335 back on
    _delay(10000);
}

void mcu_init() {
    // set system clock working frequency
    system_clock_setup(HF_CLOCK_1MHz, FALSE);

    // set ports direction
    ports_setup();

    // enable adc using its separate RC oscillator and FVR with 4x gain
    adc_setup(FRC0, FVR, FVR_GAIN_4X);

    // set usart baudrate based on system clock frequency
    usart_setup(9600, HF_CLOCK_1MHz);
}

void main(void) {
    static char buff[32];
    static Float temperature;

    mcu_init();

    //usart_tx_line("system ready...");

    for(;;) {
        read_lm335_temperature(
                AN2_CHANNEL,
                TEMPERATURE_SAMPLES,
                TEMPERATURE_CORRECTION_FACTOR,
                TEMPERATURE_ADC_VREF,
                &temperature
        );

        sprintf(
            buff,
            "T=%d.%d,V=0",
            temperature.integral,
            temperature.floating
        );

        usart_tx_line(buff);
        
        go_to_sleep(
                WDT_8S,
                ADC_MODULE_OFF,
                ADC_FVR_GAIN_BUFFER_OFF,
                COMPARATOR_FVR_GAIN_BUFFER_OFF,
                TEMPERATURE_SENSOR_OFF,
                LOW_POWER_SLEEP_MODE
        );
    }
}