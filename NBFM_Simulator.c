// zR open all folds    zM close all folds   zf define fold
// {{{ Documentation
// vim: ts=4 et sw=4 foldmethod=marker
//
// Inputs: 
// 1) Rotary knob
// 2) Selector knob
// 3) PTT switch
// 4) Shifted transmit switch
//  
// Outputs:
// 1) Display
// 2) Transmitter on
// 3) PLL frequency setting
// 4) CTCSS generator
//
// Memories
// 1) Shift frequency
// 2) CTCSS frequency
// 3) CTCSS enable
// 4) Mute level
// 
// #define TESTING "this is the simulation. #UNDEF for the real thing" 
// #define  DBG_LOGGING "used during testing to log debug info"

// The variable field width operator (%*s) in the printf library does not work for the ATMEGA328.
// This is the reason you will see the complex string hack in the output routines

/* How to add a menu item :

   - Add define to "defines generic"
   - Add entry to the menu datastructure in "globals"
   - Update length of the menu entry arrays (now done automatic)

   - Add entry to ProcessingHandler/Rotary Handling/Menu Scrolling
   - Add entry to ProcessingHandler/Selector Button/Select pushed during Menu browsing
   - Add entry to ProcessingHandler/Selector Button/Select pushed during Value editing

   - Add entry to BottomLinePrinter
 */
// }}}
// {{{ includes

// F_CPU used in util/delay.h
#define F_CPU       1000000UL

#include <stdio.h>
#include <string.h>

#ifdef TESTING

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#else

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#endif

// }}}
// {{{ defines generic

#define TRUE (1==1)
#define FALSE (!TRUE)

// {{{ States

// {{{ Main menu select states
#define MAINMENU            0
#define MMUTELEVEL          (MAINMENU+0)
#define MSHIFT              (MAINMENU+1)
#define MCTCSS              (MAINMENU+2)
#define MSSTART             (MAINMENU+3)
#define MSEND               (MAINMENU+4)
#define MSCAN               (MAINMENU+5)
#define MSETTINGS           (MAINMENU+6)
#define MBACK2TUNE          (MAINMENU+7)
// }}}
// {{{ Submenu states
#define SUBMENU1            (MAINMENU+8)
#define MARETURNMODE        (MAINMENU+8)
#define MAPLLREFMHZ         (MAINMENU+9)
#define MAPLLREFKHZ         (MAINMENU+10)
#define MABAUDRATE          (MAINMENU+11)
#define MAROTARYTYPE        (MAINMENU+12)
#define MAREMOTEENABLE      (MAINMENU+13)
#define MAFRONTENABLE       (MAINMENU+14)
#define MABACK2MAIN         (MAINMENU+15)
#define MAFACTORYRESET      (MAINMENU+16)
// }}}

// }}} States
// {{{ facdtory settings

#define IF                  69300UL     // in kHz
#define INITIAL_FREQUENCY   1298200UL   // in kHz    
#define INITIAL_SHIFT       -28000L     // in kHz
#define INITIAL_CTCSS       0           // in cHz
#define INITIAL_MUTELEVEL   10          // scalar
#define INITIAL_REFERENCE   13000UL     // in kHz
#define CHANNELSTEP         25          // in kHz
#define LARGESTEP           1000        // in kHz
#define BANDBOTTOM          1240000UL   // in kHz
#define BANDTOP             1300000UL   // in kHz

// }}}

#define IF                  69300UL     // in kHz
#define INITIAL_FREQUENCY   1298200UL   // in kHz    
#define INITIAL_SHIFT       -28000L     // in kHz
#define INITIAL_CTCSS       0           // in cHz (centi Hertz)
#define INITIAL_MUTELEVEL   10          // scalar
#define INITIAL_REFERENCE   13000UL     // in kHz
#define CHANNELSTEP         25          // in kHz
#define LARGESTEP           1000        // in kHz
#define BANDBOTTOM          1240000UL   // in kHz
#define BANDTOP             1300000UL   // in kHz

#define DISPLAY_WIDTH       16
#define DISPLAY_HEIGHT       2

#define MAXMUTELEVEL        32
#define MINSHIFT            -60000L
#define MAXSHIFT            60000L

#define SELECTBOUNCEDELAY   1   // mainloop cycle time = 34 ms.

#ifdef TESTING
#define YTop        10
#define XTop        21
#define DBGROW      14
#endif

// }}}
// {{{ defines for ATMEGA328 
#ifdef TESTING
#define eeprom_write_dword(a,b) 
//      port-nr      pin-nr  function
#define PB0 0       // (14) display - E
#define PB1 1       // (15) display - RS
#define PB2 2       // (16) not used
#define PB3 3       // (17) CTCSS - tone
#define PB4 4       // (18) display - D4
#define PB5 5       // (19) display - D5
#define PB6 6       // (9)  display - D6
#define PB7 7       // (10) display - D7

#define PC0 0       // (23) PLL - LE
#define PC1 1       // (24) PLL - DATA
#define PC2 2       // (25) PLL - CLK   
#define PC3 3       // (26) TXON
#define PC4 4       // (27) MUTE
#define PC5 5       // (28) ADC5 Smeter-input
#define PC6 6       // (1)  (uP /reset)
#define PC7 7       // does not exist

#define PD0 0       // (2)  input - /PTT
#define PD1 1       // (3)  input - Shift
#define PD2 2       // (4)  input - Push / Select
#define PD3 3       // (5)  input - Rotary Encoder Clock
#define PD4 4       // (6)  input - Rotary Encoder Data
#define PD5 5       // (11) not used
#define PD6 6       // (12) not used
#define PD7 7       // (13) not used

short ports[10];
#define PORTB ports[1]
#define PORTC ports[2]
#define PORTD ports[3]
#define DDRB  ports[4]
#define DDRC  ports[5]
#define DDRD  ports[6]
#define PINB  ports[7]
#define PINC  ports[7]
#define PIND  ports[7]
#endif


typedef unsigned char u08;

#define sbi(x,y) x |= _BV(y)    // set bit    - using bitwise OR operator 
#define cbi(x,y) x &= ~(_BV(y)) // clear bit  - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    // toggle bit - using bitwise XOR operator
#define is_high(x,y) (x & _BV(y) == _BV(y)) //check if the y'th bit of register 'x' is high ... test if its AND with 1 is 1

#define SHORT       1
#define LONG        2

// various
#define Smeter      PC5
#define MUTE        PC4
#define TXON        PC3

// ADF4113
#define ALE          PC2
#define ADATA        PC1
#define ACLK         PC0


// CTCSS & tone
#define Beep        PB3

#ifdef TESTING
#define PD0         0
#define PD1         1
#define PD2         2
#define PD3         3
#define PD4         4
#endif

// rotary & switches
#define PTT         PD0
#define SHIFTKEY    PD1
#define SELECTKEY   PD2
#define RAL         PD3
#define ALINEMASK   (1<<RAL)
#define RBL         PD4
#define BLINEMASK   (1<<RBL)
#define ROTARYMASK  (ALINEMASK || BLINEMASK)

// LCD
#define LCD_D7      PB7
#define LCD_D6      PB6
#define LCD_D5      PB5
#define LCD_D4      PB4
#define LCD_RS      PB1
#define LCD_E       PB0

// LCD commands (HD44780)
#define dispCLEAR   0x01
#define dispHOME    0x02
#define dispMODE    0x06 // left to right + shift cursor        (init)
#define dispONOF    0x0C // dispi-on + no cursor + no blink     (init)  
#define dispSHIF    0x18 // 2 Line+dir + n.c. + n.c.            (init)
#define dispFUNC    0x20 // 4bit  + 2line + size + n.c. + n.c.  (init)
#define dispCGRA    0x40 // <5 bit adr>                         (define custom chars)
#define dispDDRA    0x80 // <6 bit adr>                         (cursor addresssing)

// read status : RS=0, RW=0 bit 7 is busyflag
// write data  : RS=1, RW=1 <8 bit data>
// read  data  : RS=1, RW=0 <8 bit data>
// }}}
// {{{ Stubs for test code

#ifdef TESTING

unsigned char _BV(unsigned char c) { return c; }
void _delay_us(short s) {} 
void _delay_ms(short s) {} 

#endif

// }}}
// {{{ Function Prototypes

void initPLL(void);
void initLCD(void);
void initADC(void);

void deFrame(void);
void lcdStr(char *s);
void lcdStr16(char *s);
void lcdCmd(char c);
void lcdData(char c);
void lcdHome(void);
void lcdNib(char);
void lcdCursorPosition(int row, int col);

void OutputSetPLL(int32_t c);
void OutputSetVfoFrequency(int32_t SS_VfoFrequency);
void OutputSetTransmitterOn(char boolean);

void WritePersistent(int index);
int32_t ReadPersistent(int index);

#ifdef TESTING
void initPersistentStorage(void);
#endif
// }}}
// {{{ Globals

// {{{ System State variables

volatile int   IRQ_RotaryChange;        // amount of steps to take 
volatile char  IRQ_SelectorPushed;      // boolean: Pushed = true; Idle = false;
volatile char  IRQ_Ticks;

int         SS_RotaryCount;
int         SS_RotaryType;              // int: 0=click per cycle (classic), 1=click per pulse
char        SS_Tuning;                  // boolean: tuning = true, in menu = false
char        SS_Transmitting;            // boolean: TX = true, RX = false;
char        SS_Scanning;                // boolean: scanning = true;
char        SS_Muted;                   // boolean: TRUE=audio muted, FALSE=audio on 
char        SS_ShiftChange;             // int: 0=no change, 1 = actived, 2 = deactivated
char        SS_ShiftEnable;             // boolean: shifted = true;
char        SS_ReverseShift;            // boolean: swap transmit and receive frequencies = true
uint32_t    SS_CtcssFrequency;          // the tone value to inject
int8_t      SS_CtcssIndex;            
int8_t      SS_BaudrateIndex;            
uint32_t    SS_Baudrate;
int         SS_MenuState;               // state of the current user input menu
int         SS_MenuIndex;               // position in the value lists
char        SS_MuteIndicator;           // marker to display when audio is muted
uint8_t     SS_MuteLevel;               // level below which the audio will be muted
int32_t     SS_FrequencyShift;          // shift value to use during repeater shift
int32_t     SS_BaseFrequency;           // tuned with the rotary dial. All freqs are derived from this var.
int32_t     SS_VfoFrequency;            // the frequency to VFO must be tuned to 
int32_t     SS_DisplayFrequency;        // frequency to show on display
int16_t     SS_DisplaySMeter;           // value to show on display
int16_t     SS_SMeterIn;                // value read from the s-meter ADC

char        SS_Selected;
char        SS_PTT;
char        SS_TxRxIndicator;
int32_t     SS_ScanStartFrequency;      // duh...
int32_t     SS_ScanEndFrequency;        // duh...

char        SS_DirectMenuReturn;        // boolean: if true, direct return to tuning on entering a value
char        SS_FrontEnable;             // boolean to en/disable the frontpanel switches
char        SS_RemoteEnable;            // boolean to en/disable the remote control function
// int   SS_SMeterCalib;
uint32_t    SS_PllReferenceFrequency;   // frequency that is used as PLL reference

char  SS_ValueEdit;

// }}}  System State variables
// {{{ Menu
//
// Menu's
//

// level definitions
#define ML_MAIN 0
#define ML_SUB1 1

// datatype defintions
#define MD_NONE 0
#define MD_INT  1
#define MD_STR  2
#define MD_BOOL 3

struct MenuInfoStruct
{
    uint8_t level;
    uint8_t start;
    uint8_t last;
    uint8_t length;
};

struct MenuStruct
{
    char    *name;
    uint8_t level;
    uint8_t position;
    uint8_t datatype;
    char    *format;
    int32_t value;
};

// If an extra submenu would be introduced, then you would need to add the following record:
// { ML_SUB2, <index_of_first_entry>, <index_of_last_entry>, <nr_of_entries> }
// and off course we need: #define ML_SUB2  2


struct MenuInfoStruct infoMenu[] = 
{
//    level    start     end          nr of entries (length)
    { ML_MAIN, MAINMENU, MBACK2TUNE , MBACK2TUNE-MAINMENU + 1 },
    { ML_SUB1, SUBMENU1, MABACK2MAIN, MABACK2MAIN-SUBMENU1 + 1 }    // for now, skip factory reset
};

// formula for next menu item to display
// f(x) = (( x – start + Nsteps ) % length ) + start

// enter all menu items in the datastructure below

struct MenuStruct theMenu[] = 
{
    { "Mute Level"    , ML_MAIN, 0, MD_INT , "%s%2d"           , INITIAL_MUTELEVEL   },    // 00
#ifdef TESTING
    { "Shift"         , ML_MAIN, 1, MD_INT , "%s%4d.%03d MHz"  , INITIAL_SHIFT       },    // 01
    { "CTCSS"         , ML_MAIN, 2, MD_INT , "%s%3u.%1u Hz"    , INITIAL_CTCSS       },    // 02
    { "Scan Start"    , ML_MAIN, 3, MD_INT , "%s%4u.%03u MHz"  , BANDBOTTOM          },    // 03
    { "Scan End"      , ML_MAIN, 4, MD_INT , "%s%4u.%03u MHz"  , BANDTOP             },    // 04
#else
    { "Shift"         , ML_MAIN, 1, MD_INT , "%s%4ld.%03ld MHz", INITIAL_SHIFT       },    // 01
    { "CTCSS"         , ML_MAIN, 2, MD_INT , "%s%3u.%1u Hz"    , INITIAL_CTCSS       },    // 02
    { "Scan Start"    , ML_MAIN, 3, MD_INT , "%s%4lu.%03lu MHz", BANDBOTTOM          },    // 03
    { "Scan End"      , ML_MAIN, 4, MD_INT , "%s%4lu.%03lu MHz", BANDTOP             },    // 04
#endif
    { "Start Scanning", ML_MAIN, 5, MD_NONE, ""                , 0                   },    // 05 "value" unused
    { "Settings"      , ML_MAIN, 6, MD_NONE, ""                , 0                   },    // 06 "value" unused
    { "Back to tune"  , ML_MAIN, 7, MD_NONE, ""                , 0                   },    // 07 "value" unused
    { "On select go"  , ML_SUB1, 0, MD_BOOL, "%s%s"            , TRUE                },    // 08
#ifdef TESTING
    { "PLL Ref MHz"   , ML_SUB1, 1, MD_INT , "%s%4u.%03u MHz"  , INITIAL_REFERENCE   },    // 09
    { "PLL Ref kHz"   , ML_SUB1, 2, MD_INT , "%s%4u.%03u MHz"  , INITIAL_REFERENCE   },    // 10
    { "Baudrate"      , ML_SUB1, 3, MD_INT , "%s%6d"           , 9600                },    // 11
#else
    { "PLL Ref MHz"   , ML_SUB1, 1, MD_INT , "%s%4lu.%03lu MHz", INITIAL_REFERENCE   },    // 09
    { "PLL Ref kHz"   , ML_SUB1, 2, MD_INT , "%s%4lu.%03lu MHz", INITIAL_REFERENCE   },    // 10
    { "Baudrate"      , ML_SUB1, 3, MD_INT , "%s%6ld"          , 9600                },    // 11
#endif
    { "Rotary type"   , ML_SUB1, 4, MD_BOOL, "%s%s"            , FALSE               },    // 13
    { "Remote enable" , ML_SUB1, 5, MD_BOOL, "%s%s"            , FALSE               },    // 14
    { "Front enable"  , ML_SUB1, 6, MD_BOOL, "%s%s"            , TRUE                },    // 15
    { "Back to main"  , ML_SUB1, 7, MD_NONE, ""                , 0                   },    // 16 "value" unused
    { "Factory reset" , ML_SUB1, 8, MD_NONE, "%s%s"            , 0                   },    // 17 "value" unused
};

// }}}

#ifdef TESTING
clock_t now;
char  tmpFreqChanged=FALSE;
char  tmpFreqSaved=FALSE;
char  AutoTest=FALSE;        // set to true via commandline when automated testing is in order
char  dbg_logging=FALSE;     // set to true via commandline when we need debug logging
short goingUp;               // tmp global
short LoopCounter;           // tmp global
short cpos;                  // display: lineair cursor position
#endif

uint16_t currentTime;        // tmp global
uint16_t prevStepTime;       // tells scanner when it is time for the next channel
uint16_t goStep;
uint32_t lastFrequencyChange;   // keeps track of time since last tuning action
uint16_t TimerValue;         // value to program in the timer

char GClkPrev;               // used for rotary dial handling

uint16_t LowPass;            // Variable for S-meter lowpass filter
char  IntRotLines;           // remember status of rotary switch inputs


int32_t prevFreq;            // used to determine if the freq display needs updating

char  Line[DISPLAY_WIDTH+10];

// define S-meter chars
unsigned char smeter[3][8] = {
    {0b00000,0b00000,0b10000,0b10000,0b10000,0b10000,0b00000,0b00000},
    {0b00000,0b00000,0b10100,0b10100,0b10100,0b10100,0b00000,0b00000},
    {0b00000,0b00000,0b10101,0b10101,0b10101,0b10101,0b00000,0b00000}
};

//CTCSS frequencies
uint16_t CtcssTones[] = {   0, 670, 689, 693, 710, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974,
    1000,1035,1072,1109,1148,1188,1230,1273,1318,1365,1413,1462,1514,1567,1598,
    1622,1655,1679,1713,1738,1773,1799,1835,1862,1899,1928,1966,1995,2035,2065,
    2107,2181,2257,2291,2336,2418,2503,2541, -1
};
// uint8_t ctcssIndex;
uint8_t ctcssLength = (sizeof(CtcssTones)/sizeof(uint16_t))-2;

uint32_t Baudrates[] = { 1200, 2400, 4800, 9600, 19200, 38400, 76800, 115600 };
uint8_t baudrateLength = 8 - 1;

#ifdef TESTING
// short  theMainEvent;
char   GetcBuffer;
short  GetcAvail;
struct termios orig_termios;
FILE   *dbg;
FILE   *eeprom;
#endif

// input states controls
#ifdef TESTING
// char vInRotState=9;
//short S;
#endif

// }}}

// {{{ Interrupt Service Routines

// {{{ Rotary Pulse : Single line change per click

#ifndef TESTING

#define ALINE    0x02
#define IRQMASK  0x06

// {{{ documentation rotary encoder handling

//  A line caused interrupt (A line changed direction)
//   | A | B | Step
//   +---+---+-----
//   | 0 | 0 |  -1 
//   | 1 | 0 |  +1 
//   | 0 | 1 |  +1 
//   | 1 | 1 |  -1 

//  B line caused interrupt (B line changed direction)
//   | A | B | Step
//   +---+---+-----
//   | 0 | 0 |  +1 
//   | 1 | 0 |  -1 
//   | 0 | 1 |  -1 
//   | 1 | 1 |  +1 

// A polarty chagnge : Step = A xor B
// B polarty chagnge : Step = not (A xor B)

// }}}

//  Arrive here when either the A-line or the B-line changed polarity

ISR(PCINT2_vect)
{
    // only active with rotary type=1
    if (SS_RotaryType == 1)
    {
        char aLine;
        char bLine;
        char direction;
        char eval;

        // sample the PIND register value;
        char sample = PIND;

        // find out which line caused an interrupt
        char aLineChanged = (((sample ^ IntRotLines) & ALINEMASK) != 0);
        //  char bLineChanged = (((sample ^ IntRotLines) & BLINEMASK) != 0);

        // direction is TRUE  when the A line caused the interrupt;
        // direction is FALSE when the B line caused the interrupt;
        direction = aLineChanged;

        // save current value of the lines;
        IntRotLines = sample;

        aLine = (sample & ALINEMASK) != 0;
        bLine = (sample & BLINEMASK) != 0;
        eval = aLine ^ bLine;

        // eval now implements the truth table for an A line triggered interrupt
        // the B line has the exact opposite truth table, so we can simple XOR with 
        // the direction value. (see documentation at the start of this ISR)

        if (eval ^ direction)
            IRQ_RotaryChange--;
        else
            IRQ_RotaryChange++;
    }
}

#endif

// }}}
// {{{ Selector Pulse

#ifndef TESTING

ISR(INT0_vect)
{
    volatile register char sample;

    // The selector line has gone low, so the button was pushed
    IRQ_SelectorPushed = TRUE;
    sample = PIND;      //  make sure the interrupt is acknowledged and cleared
    sample = EIMSK;     //  make sure the interrupt is acknowledged and cleared
    sample++;           //  suppress compilter warnings about unused variables;
}

#endif

// }}}
// {{{ Timer
#ifndef TESTING

ISR(TIMER1_OVF_vect) 
{ 
    IRQ_Ticks++;              // increment tick for freq save timeout
    // restart timer
    TCNT1 = 65536-TimerValue;  
    if (SS_Transmitting && (SS_CtcssIndex != 0))
    {
        tbi(PORTB, Beep);       // toggle Beep port
    }
} 

#endif
// }}}

// }}}

// {{{ Initialisation

// {{{ void initIRQ(void)

void initIRQ(void)
{
#ifndef TESTING
    cli();      // Disable global interrupts

    // {{{ Rotary Dial Line A (PD3) and line B (PD4)
    // make PD3 (INT1 pin) an input
    // make PD4 (INT? pin) an input
    DDRD &= ~(1 << DDD3);     // Clear the PD3 ad PD4 in
    DDRD &= ~(1 << DDD4);     // Clear the PD3 ad PD4 in

    // enable pull-up on PD3 and PD4 
    PORTD |= (1 << PORTD3);    // turn On the Pull-up
    PORTD |= (1 << PORTD4);    // turn On the Pull-up

    PCMSK2 = 0x18;  // enable Pin Change interrupts 19 en 20
    PCICR  = 0x04;  // enable Pin Change Interrupt Enable 2

    // }}}
    // {{{ Selector (PD2)

    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    // PD2 (INT0 pin) is now an input

    PORTD |= (1 << PORTD2);    // turn on the pull-up
    // PD2 is now an input with pull-up enabled

    // External Interrupt Control Register A
    // set Interrupt Sense Control ISC00 and ISC01 to 01 for falling edge trigger
    EICRA |= (1 << ISC01);    // set INT0 to trigger on a falling edge  
    // External Interrupt Mask register
    EIMSK |= (1 << INT0);     // Turns on INT0

    // }}}
    // {{{ Timer () 

    // Setup Timer 1
    TCCR1A = 0x00;        // Normal Mode 
    TCCR1B = 0x01;        // div/1 clock, 1/F_CPU clock
    TimerValue = 5*F_CPU/SS_CtcssFrequency; // *10/2

    // Enable interrupts as needed 
    TIMSK1 |= _BV(TOIE1);   // Timer 1 overflow interrupt 

    // }}}

    sei();      // Enable global interrupts
#endif
}

// }}}
// {{{ void initLCD(void)
void initLCD(void)
{
    // allow the lcd controller to wake up
    _delay_ms(100);

    // depending on intial state:
    // ... force to state 1 or state 3
    lcdNib(0x30);
    _delay_ms(20);

    // ... and force to state 1
    lcdNib(0x30);
    _delay_ms(20);

    // 4 bits mode, 2 lines
    lcdCmd(dispFUNC); // 0x20
    _delay_ms(20);

    //lcdCmd(dispFUNC);
    //_delay_ms(10);

    // cursor shifts to right, text no shift
    lcdCmd(dispSHIF); // 0x18
    _delay_ms(40);

    // display on, no cursor, no blink
    lcdCmd(dispONOF); // 0x0C
    _delay_ms(40);

    // shift mode
    lcdCmd(dispMODE); // 0x06
    _delay_ms(40);

    // clear display
    // leave cursor at top left
    lcdCmd(dispCLEAR); // 0x01
    _delay_ms(40);


    // define custom chars
    uint8_t i,j;
    lcdCmd(dispCGRA);
    for (i=0; i<3; i++) 
    {
        for (j=0; j<8; j++) 
        {
            lcdData(smeter[i][j]);
        }
    }


    // welcome message 
    lcdHome();
    _delay_ms(40);

    //              0123456789ABCDEF
    char hello[] = "PA3BJI sw v0.5  ";
    lcdStr16(hello);
}


// }}}
// {{{ void initADC(void)

void initADC(void)
{
#ifndef TESTING
    // prescaler: devide by 128
    // ADC clock = 1MHz / 128 = 7812,5 Hz
    ADCSRA = (1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);  

    // Disable digital functions on analog input
    DIDR0 = (1<<ADC5D);

    // Choose ADC channel 5 
    ADMUX = (1<<REFS0) + 5;         
#endif
}

// }}}
// {{{ void initPLL(void)

void initPLL(void)
{
    int32_t reg;

    // clear signal lintes
    cbi(PORTC, ADATA);
    cbi(PORTC, ACLK);
    cbi(PORTC, ALE);

    // init function latch
    reg = 0x438086;
    OutputSetPLL(reg);

    // init R-counter c.q. the reference frequency
    reg = (2UL<<16) + ((SS_PllReferenceFrequency/CHANNELSTEP)<<2);
    OutputSetPLL(reg);

    // init the start up channel
    OutputSetVfoFrequency(SS_VfoFrequency);

    // init something else
    reg = 0x438082;
    OutputSetPLL(reg);
}

// }}}
// {{{ void initPORTS(void)

void initPORTS(void)
{
    // PORTB output for LCD
    DDRB = 0xff;
    PORTB = 0xff;

    // PORTC PC0-4 output, PC5 input
    DDRC = 0x1f;
    // make outputs low
    PORTC = 0x00;

    // PORTD is input with pullup
    DDRD = 0x00;
    PORTD = 0xff;
}

// }}}
// {{{ void initPersistentStorage()

#ifdef TESTING
void initPersistentStorage()
{
    int32_t zero=0;
    eeprom = fopen("eeprom.bin","r");
    if (!eeprom)
    {
        eeprom = fopen("eeprom.bin","w");
        fwrite (&zero,sizeof(int32_t),16,eeprom);
    }
    fclose(eeprom);
    eeprom = fopen("eeprom.bin","r");
}
#endif

// }}}
// {{{ void readPersistentStorage(void)

void readPersistentStorage(void)
{
#ifdef TESTING 
    uint32_t i;
    for (i=0; i<16; i++)
    {
        fread((int32_t *)&theMenu[(int)i].value,1,sizeof(int32_t), eeprom);
    }
#else
    theMenu[(int)0x00].value = eeprom_read_dword((uint32_t *) (0x00*sizeof(uint32_t)));
    theMenu[(int)0x01].value = eeprom_read_dword((uint32_t *) (0x01*sizeof(uint32_t)));
    theMenu[(int)0x02].value = eeprom_read_dword((uint32_t *) (0x02*sizeof(uint32_t)));
    theMenu[(int)0x03].value = eeprom_read_dword((uint32_t *) (0x03*sizeof(uint32_t)));
    theMenu[(int)0x04].value = eeprom_read_dword((uint32_t *) (0x04*sizeof(uint32_t)));
    theMenu[(int)0x05].value = eeprom_read_dword((uint32_t *) (0x05*sizeof(uint32_t)));
    theMenu[(int)0x06].value = eeprom_read_dword((uint32_t *) (0x06*sizeof(uint32_t)));
    theMenu[(int)0x07].value = eeprom_read_dword((uint32_t *) (0x07*sizeof(uint32_t)));
    theMenu[(int)0x08].value = eeprom_read_dword((uint32_t *) (0x08*sizeof(uint32_t)));
    theMenu[(int)0x09].value = eeprom_read_dword((uint32_t *) (0x09*sizeof(uint32_t)));
    theMenu[(int)0x0A].value = eeprom_read_dword((uint32_t *) (0x0A*sizeof(uint32_t)));
    theMenu[(int)0x0B].value = eeprom_read_dword((uint32_t *) (0x0B*sizeof(uint32_t)));
    theMenu[(int)0x0C].value = eeprom_read_dword((uint32_t *) (0x0C*sizeof(uint32_t)));
    theMenu[(int)0x0D].value = eeprom_read_dword((uint32_t *) (0x0D*sizeof(uint32_t)));
    theMenu[(int)0x0E].value = eeprom_read_dword((uint32_t *) (0x0E*sizeof(uint32_t)));
    theMenu[(int)0x0F].value = eeprom_read_dword((uint32_t *) (0x0F*sizeof(uint32_t)));
#endif

#define inbetween(v, a, b) (!((v < a) || (v > b)))

    SS_MuteLevel = (uint8_t) theMenu[MMUTELEVEL].value;
    // integrity checking
    if (!inbetween(SS_MuteLevel, 0, MAXMUTELEVEL))
    {
        SS_MuteLevel = INITIAL_MUTELEVEL;
        theMenu[MMUTELEVEL].value = (uint32_t) SS_MuteLevel;
        eeprom_write_dword((uint32_t *)(MMUTELEVEL*sizeof(uint32_t)), theMenu[MMUTELEVEL].value);
    }

    SS_FrequencyShift = theMenu[MSHIFT].value;
    // integrity checking
    if (!inbetween(SS_FrequencyShift, -60000L, 60000L))
    {
        SS_FrequencyShift = INITIAL_SHIFT;
        theMenu[MSHIFT].value = SS_FrequencyShift;
        eeprom_write_dword((uint32_t *)(MSHIFT*sizeof(uint32_t)), theMenu[MSHIFT].value);
    }

    SS_CtcssIndex = (uint8_t)theMenu[MCTCSS].value;
    // integrity checking
    if (!inbetween(SS_CtcssIndex,0,ctcssLength))
    {
        SS_CtcssIndex = 0;
        eeprom_write_dword((uint32_t *)(MCTCSS*sizeof(uint32_t)), theMenu[MCTCSS].value);
    }
    theMenu[MCTCSS].value = (uint32_t)SS_CtcssIndex;
    SS_CtcssFrequency     = (int16_t) CtcssTones[SS_CtcssIndex];

    SS_ScanStartFrequency = theMenu[MSSTART].value;
    // integrity checking
    if (!inbetween(SS_ScanStartFrequency,BANDBOTTOM, BANDTOP))
    {
        SS_ScanStartFrequency = BANDBOTTOM;
        theMenu[MSSTART].value = SS_ScanStartFrequency;
        eeprom_write_dword((uint32_t *)(MSSTART*sizeof(uint32_t)), theMenu[MSSTART].value);
    }

    SS_ScanEndFrequency = theMenu[MSEND].value;
    // integrity checking
    if (!inbetween(SS_ScanEndFrequency,BANDBOTTOM, BANDTOP))
    {
        SS_ScanEndFrequency = BANDTOP;
        theMenu[MSEND].value = SS_ScanEndFrequency;
        eeprom_write_dword((uint32_t *)(MSEND*sizeof(uint32_t)), theMenu[MSEND].value);
    }

    SS_BaseFrequency = theMenu[5].value;
    // integrity checking
    if (!inbetween(SS_BaseFrequency,BANDBOTTOM, BANDTOP))
    {
        SS_BaseFrequency = INITIAL_FREQUENCY;
        theMenu[5].value = SS_BaseFrequency;
        eeprom_write_dword((uint32_t *)(5*sizeof(uint32_t)), theMenu[5].value);
    }

    SS_BaudrateIndex = theMenu[MABAUDRATE].value;
    // integrity checking
    if (!inbetween(SS_BaudrateIndex,0,baudrateLength))
    {
        SS_BaudrateIndex = 0;
        theMenu[MABAUDRATE].value = (uint32_t)SS_BaudrateIndex;
        eeprom_write_dword((uint32_t *)(MABAUDRATE*sizeof(uint32_t)), theMenu[MABAUDRATE].value);
    }
    SS_Baudrate = (uint32_t) Baudrates[SS_BaudrateIndex];

    SS_RotaryType = theMenu[MAROTARYTYPE].value;
    // integrity checking
    if (!inbetween(SS_RotaryType,0,1))
    {
        SS_RotaryType = 0;
        theMenu[MAROTARYTYPE].value = SS_RotaryType;
        eeprom_write_dword((uint32_t *)(MAROTARYTYPE*sizeof(uint32_t)), theMenu[MAROTARYTYPE].value);
    }
     
    SS_PllReferenceFrequency = theMenu[MAPLLREFMHZ].value;
    // integrity checking
    if (!inbetween(SS_PllReferenceFrequency, 5000UL, 150000UL))
    {
        theMenu[MAPLLREFMHZ].value = (uint32_t)INITIAL_REFERENCE;
        theMenu[MAPLLREFKHZ].value = (uint32_t)INITIAL_REFERENCE;
        eeprom_write_dword((uint32_t *)(MAPLLREFMHZ*sizeof(uint32_t)), theMenu[MAPLLREFMHZ].value);
        eeprom_write_dword((uint32_t *)(MAPLLREFKHZ*sizeof(uint32_t)), theMenu[MAPLLREFKHZ].value);
    }
}

// }}}
// {{{ void initialize(void)

void initialize(void)
{
    // Some articles suggest you'd better wait a while until
    // its very sure the 5v power line is stable
    _delay_ms(100);

    // We need to know about hardware config before hw init 
#ifdef TESTING
    initPersistentStorage();
#endif
    readPersistentStorage();

    // These are actualy read from persistant storage
    // SS_PllReferenceFrequency= INITIAL_REFERENCE;
    // SS_FrequencyShift       = INITIAL_SHIFT;
    // SS_ScanStartFrequency   = BANDBOTTOM;
    // SS_ScanEndFrequency     = BANDTOP;
    // SS_MuteLevel            = INITIAL_MUTELEVEL;
    // SS_BaseFrequency        = INITIAL_FREQUENCY;

    SS_DisplayFrequency     = SS_BaseFrequency;
    SS_VfoFrequency         = SS_BaseFrequency - IF;
    SS_ShiftEnable          = FALSE;
    SS_ReverseShift         = FALSE;
    SS_Transmitting         = FALSE;
    SS_MenuState            = MAINMENU;
    SS_ValueEdit            = FALSE;
    SS_Tuning               = TRUE;
    IRQ_SelectorPushed      = FALSE;
    IRQ_RotaryChange        = 0;
    IRQ_Ticks               = 0;

#ifdef TESTING
    //    vInRotState = 9;
    GetcAvail   = FALSE;
    GetcBuffer  = 0;
    PIND |= (1<<PTT); // PTT switch not active!
#else
    initPORTS();
    initPLL();
    initLCD();
    initADC();
    initIRQ(); 
#endif
}

// }}}

// }}}
// {{{ Scaffolding test code

#ifdef TESTING

// {{{ Output Scaffolding

// {{{ ttyControl

// {{{ void ttySetCursorPosition(short row, short col)

void ttySetCursorPosition(short row, short col)
{
    if (!AutoTest) printf("\033[%d;%df",row+1, col+1);
}

// }}}
// {{{ void ttyClearScreen(void)

void ttyClearScreen(void)
{
    if (!AutoTest)
    {
        printf("\033[2J");
        printf("\033[?1h");
    }
}

// }}}

// }}}
// {{{ Display emulator

void NL(void)
{
    printf("\n"); 
    if (!AutoTest) printf ("\r");
}

// {{{ deSetCursorPosition(short row, short col)

void deSetCursorPosition(short row, short col)
{
    ttySetCursorPosition(row+YTop, col+XTop);
}

// }}}
// {{{ deTop(void)

void deTop(void)
{
    deSetCursorPosition(0,0);
}

// }}}
// {{{ deData(char c)

void deData(char c)
{
    printf("%c",c);
    //cpos++; 
    //if (cpos==DISPLAY_WIDTH)
    //    deSetCursorPosition(2,1);
}

// }}}
// {{{ deClearScreen(void)

void deClearScreen(void)
{
    short i;
    deTop();
    for (i=0; i<DISPLAY_WIDTH; i++)
    {
        deData(' ');
    }
    deSetCursorPosition(1,0); 
    for (i=0; i<DISPLAY_WIDTH; i++)
    {
        deData(' ');
    }

    for (i=0; i<DISPLAY_WIDTH; i++)
    {
        // DisplayBuffer[0][i]=' ';
        // DisplayBuffer[1][i]=' ';
    }
}

// }}}
// {{{ deCmd(char c)

void deCmd(char c)
{
    short pos;
    short row;

    if (c == dispCLEAR)
    {
        deClearScreen();
    }

    if ((c & 0xFE) == dispHOME) 
    {
        deTop();
    }

    if ((c & 0x80) == dispDDRA)
    {
        pos = c & 0x0F;
        row = (c & 0x40) ? 1 : 0;
        deSetCursorPosition(row,pos);
    }
}

// }}}
// {{{ deFrame(void)

void deLine(short w)
{
    short i;
    printf("+");
    for (i=1; i<w; i++)
        printf("-");
    printf("+\n"); NL();
}

void deFrame(void)
{
    short y;

    if (!AutoTest)
    {
        // cursor off
        printf("\033[?25l");   

        deSetCursorPosition(-1, -1);
        deLine(DISPLAY_WIDTH+1);
        for (y=0; y<DISPLAY_HEIGHT; y++)
        {
            deSetCursorPosition(y, -1);
            printf("|");
            deSetCursorPosition(y,DISPLAY_WIDTH);
            printf("|");
        }
        deSetCursorPosition(DISPLAY_HEIGHT, -1);
        deLine(DISPLAY_WIDTH+1);
    }
}

// }}}

// }}}
// {{{ char* yesno(short boolean)

char* yesno(short boolean)
{
    static char ayes[]="yes";
    static char ano[] =" no";
    // green -> light gray
    static char myes[]="\033[32myes\033[37m";
    // red -> light gray
    static char mno[] ="\033[31m no\033[37m";
    if (AutoTest)
        return (boolean ? ayes : ano); 
    else
        return (boolean ? myes : mno); 
}

// }}}

// }}}
// {{{ Input Scaffolding

#ifdef TESTING

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    tcgetattr(0, &new_termios);

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

short FHEkbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

short FHEgetchar()
{
    short r;
    char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

// non blocking getkeyboard function
// return FALSE if no character is available
char FHEgetc(void)
{
    char c;

    if (GetcAvail)
    {
        c = GetcBuffer;
        GetcBuffer=0;
    } else 
    {
        if (FHEkbhit()) 
        {
            c = FHEgetchar();
        } else 
        {
            c = FALSE;
        }
    }
    GetcAvail = FALSE;
    return c;
}

void FHEungetc(char c)
{
    GetcBuffer = c;
    GetcAvail = TRUE;
}

#endif

// }}}

#endif

// }}}

// {{{ Input functions

// {{{ int8_t InputRotaryPoller(void) (Full cycle per click)

int8_t InputRotaryPoller(void)
{
    unsigned char sample;
    unsigned char data;
    unsigned char clk;
    unsigned char UpDown;

    // read input lines
    sample = PIND;

    // isolate the signals
    data = (sample & ALINEMASK) != 0;
    clk  = (sample & BLINEMASK) != 0;

    if (clk && !GClkPrev)   // true on rising flank 
    {                    
        UpDown = data;      // sample data signal
        if (UpDown)
            IRQ_RotaryChange++;
        else
            IRQ_RotaryChange--; 
    }
    GClkPrev = clk;         // save for next iteration

    return IRQ_RotaryChange;
}

// }}}
// {{{ char InputGetPTT(void)

char InputGetPTT(void)
{
    static char prevPttActive;
    char pttActive;
    char rv;

#ifdef TESTING
    char ppa = prevPttActive;
#endif

    // sample value from PTT switch input
    pttActive = ((PIND & (1<<PTT)) == 0);
    if (prevPttActive != pttActive)
    {
        prevPttActive = pttActive;
        // if PTT activated return 1
        // if PTT released return  2
        // return 0 on no-change
        rv = (pttActive) ? 1 : 2;
    } else
        rv = 0;
#ifdef TESTING    
    if (dbg_logging) 
        fprintf(dbg, "PIND=%02x prevPTT=%02x, PTT=%02x rv=%d \n", PIND, ppa, pttActive, rv);
#endif

    //test and debug $$$
    rv = (pttActive) ? 1 : 2;
    return rv;
}

// }}}
// {{{ int  InputGetRotaryDialCount(void)

int InputGetRotaryDialCount(void)
{
#ifdef TESTING
    return 0;
#else
    register int rv;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        rv = IRQ_RotaryChange;
        IRQ_RotaryChange = 0;
    }
    return rv;
#endif
}

// }}}
// {{{ char InputGetSelectorPushed(void)

char InputGetSelectorPushed(void)
{
#ifdef TESTING
    return FALSE;
#else
    register int rv;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        rv = IRQ_SelectorPushed;
        IRQ_SelectorPushed = FALSE;
    }
    return rv;
#endif
}

// }}}
// {{{ char InputGetShiftEnable(void)
char InputGetShiftEnable(void)
{
    static char prevShiftActive;
    char shiftActive;
    char rv;

    // sample value from shift switch input
    shiftActive = ((PIND & (1<<SHIFTKEY)) == 0);
    if (prevShiftActive != shiftActive)
    {
        prevShiftActive = shiftActive;
        // if Shift activated return 1
        // if Shift released return -1
        // return 0 on no-change
        rv = (shiftActive) ? 1 : -1;
    } else
        rv = 0;

#ifdef TESTING 
    rv = FALSE;
#endif
    return rv;
}

// }}}
// {{{ short InputGetSMeter(void)

uint16_t InputGetSMeter(void)
{
#ifdef TESTING
    static int simuls;
    static int delay;
    static char rising;
    static int hoog=980;
    static int laag=980-84;

    if (SS_Tuning && (delay++ > 320))
    {
        delay = 0;
        if (rising) simuls++; else simuls--;
        if (simuls>hoog) { simuls=hoog; rising=FALSE; }
        if (simuls<laag) { simuls=laag; rising=TRUE; }
    }
    return simuls;
#else
    // set AD Start Conversion bit and AD ENable bit
    ADCSRA |= (1<<ADSC)|(1<<ADEN); 

    // wait for conversion to finish
    // this is indicated by the ADSC bit clear
    while ((ADCSRA & (1<<ADSC))!=0);

    // apparantly ADC is a 16 bit register
    return ADC;
#endif
}

// }}}

// {{{  void InputHandler(void)

char InputHandler(void)
{
    char busy = TRUE;
    if (SS_RotaryType != 1) InputRotaryPoller();

    SS_RotaryCount = InputGetRotaryDialCount();
    SS_Selected    = InputGetSelectorPushed();
    SS_ShiftChange = InputGetShiftEnable();
    SS_PTT         = InputGetPTT();
    SS_SMeterIn    = InputGetSMeter();

#ifdef TESTING
    // {{{ read keyboard for test

    // clear now, set to TRUE and process when 'e' key is pressed
    SS_Selected = FALSE;

    char theKey;
    char c;
    if ((c = FHEgetc()))
    {
        // save the key for debug purposes
        theKey = c;

        switch (c)
        {
            case 27  : // 27 is the ESCAPE character
            case 'q' : busy = FALSE;
                       theKey = c;
                       break;

            case '[' : SS_RotaryCount = -1;
                       theKey = c;
                       break;

            case ']' : SS_RotaryCount = +1;
                       theKey = c;
                       break;

                       // Clear the PTT bit (switch pulls to ground)
            case 't' : PIND &= ~(1<<PTT);
                       theKey = c;
                       break;

                       // Set the PTT bit (switch released, pull-up active)
            case 'r' : PIND |= (1<<PTT);
                       theKey = c;
                       break;

            case 's' : SS_ShiftChange = 1;
                       theKey = c;
                       break;

            case 'a' : SS_ShiftChange = 2;
                       theKey = c;
                       break;

            case 'e' : SS_Selected = TRUE;
                       theKey = c;
                       break;

            default:
                       // swallow unused input characters by 
                       // calling the non-blocking FHEgetc();
                       (void)FHEgetc();
        }
    }

    // }}}
#endif

    return busy;
}
// }}} input handling

// }}}
// {{{ void RemoteControlHandler(void)

void RemoteControlHandler(void)
{
}

// }}}
// {{{ void ProcessingHandler(void)

void ProcessingHandler(void)
{
    // {{{ // Rotary Handling

    // {{{ Tuning

    // Tuning is only allowed during receive
    if ((!SS_Transmitting) & SS_Tuning)
    {
        if (SS_RotaryCount != 0)
        {
            SS_BaseFrequency += SS_RotaryCount * CHANNELSTEP;
            SS_RotaryCount = 0;
            lastFrequencyChange = IRQ_Ticks;
#ifdef TESTING
            tmpFreqChanged = TRUE;
            tmpFreqSaved  = FALSE;
            lastFrequencyChange = clock();
#endif
        }
    } 
#ifdef TESTING
    now = clock();
    if ((now - lastFrequencyChange) > 1000000L) // 1 mil usecs ~ 2 sec
    {
        theMenu[5].value = SS_BaseFrequency;
        // position 5 is not used for regular value storage
        // eeprom_write_dword(5, theMenu[5].value);
        tmpFreqChanged = FALSE;
        tmpFreqSaved = TRUE;
    }
#else
    currentTime = IRQ_Ticks;
    if ((currentTime - lastFrequencyChange) > 200) //  ~ 2 sec
    {
        theMenu[5].value = SS_BaseFrequency;
        // position 5 is not used for regular value storage
        eeprom_update_dword((uint32_t *)(5*sizeof(uint32_t)), theMenu[5].value);
    }
#endif

    // }}}
    // {{{ Menu scrolling

    int8_t start;
    int8_t length;
    int8_t level;
    int8_t tmp;
    uint32_t kHz;

    // scrolling through menu
    if (!SS_Tuning && !SS_ValueEdit)
    {
        if (SS_RotaryCount != 0)
        { 
            level  = theMenu[SS_MenuState].level;
            start  = infoMenu[level].start;
            length = infoMenu[level].length;

            // f(x) = (( x – start + Nsteps ) % length ) + start
            tmp = SS_MenuState - start + SS_RotaryCount;
            SS_MenuState = tmp % length;
            if (SS_MenuState < 0) 
                SS_MenuState += length; // necesary because the % operator may return negative results
            SS_MenuState += start;
            SS_RotaryCount = 0;
        }
    }

    // editing values
    if (!SS_Tuning && SS_ValueEdit)
    {
        if (SS_RotaryCount != 0)
        { 
            switch (SS_MenuState)
            {
                case MMUTELEVEL :
                    SS_MuteLevel+= SS_RotaryCount;
                    if (SS_MuteLevel > MAXMUTELEVEL) SS_MuteLevel = MAXMUTELEVEL;
                    if (SS_MuteLevel < 0)            SS_MuteLevel = 0;
                    theMenu[SS_MenuState].value    = SS_MuteLevel;
                    break;

                case MSHIFT :
                    SS_FrequencyShift += (SS_RotaryCount*1000);
                    if (SS_FrequencyShift > MAXSHIFT) SS_FrequencyShift = MAXSHIFT;
                    if (SS_FrequencyShift < MINSHIFT) SS_FrequencyShift = MINSHIFT;
                    theMenu[SS_MenuState].value     = SS_FrequencyShift;                
                    break;

                case MCTCSS :
                    SS_CtcssIndex += SS_RotaryCount;
                    if (SS_CtcssIndex < 0) SS_CtcssIndex = 0;
                    if (SS_CtcssIndex > ctcssLength) SS_CtcssIndex = ctcssLength;

                    theMenu[SS_MenuState].value = (int32_t)SS_CtcssIndex;
                    SS_CtcssFrequency = CtcssTones[SS_CtcssIndex];
                    break;

                case MSSTART :
                    SS_ScanStartFrequency += (SS_RotaryCount * CHANNELSTEP);
                    if (SS_ScanStartFrequency < BANDBOTTOM) SS_ScanStartFrequency = BANDBOTTOM;
                    if (SS_ScanStartFrequency > SS_ScanEndFrequency) SS_ScanStartFrequency = SS_ScanEndFrequency;
                    theMenu[SS_MenuState].value = SS_ScanStartFrequency;
                    break;

                case MSEND :
                    SS_ScanEndFrequency += (SS_RotaryCount * CHANNELSTEP);
                    if (SS_ScanEndFrequency < SS_ScanStartFrequency) SS_ScanEndFrequency = SS_ScanStartFrequency;
                    if (SS_ScanEndFrequency > BANDTOP)    SS_ScanEndFrequency = BANDTOP;
                    theMenu[SS_MenuState].value = SS_ScanEndFrequency;
                    break;

                case MARETURNMODE :
                    SS_RotaryCount = SS_RotaryCount % 2;
                    SS_DirectMenuReturn = (SS_RotaryCount==1) ? TRUE : FALSE;
                    theMenu[SS_MenuState].value = SS_DirectMenuReturn;
                    break;
                    /*
                       case MASMCALIB :
                       SS_SMeterCalib += SS_RotaryCount;
                       break;
                     */
                    // ADF4113HV   Fref = 5 .. 150 MHz
                case MAPLLREFMHZ :
                    SS_PllReferenceFrequency += SS_RotaryCount * 1000;
                    if (SS_PllReferenceFrequency <   5000UL) SS_PllReferenceFrequency =   5000UL;
                    if (SS_PllReferenceFrequency > 150000UL) SS_PllReferenceFrequency = 150000UL;
                    theMenu[SS_MenuState  ].value = SS_PllReferenceFrequency;
                    theMenu[SS_MenuState+1].value = SS_PllReferenceFrequency;
                    break;

                case MAPLLREFKHZ :
                    kHz = SS_PllReferenceFrequency;
                    kHz += SS_RotaryCount;
                    kHz = kHz % 1000;                   // Stay in the kHz range
                    SS_PllReferenceFrequency /= 1000;   // Integer devide by 1000, followed by...
                    SS_PllReferenceFrequency *= 1000;   // multiply by 1000 to set the lower 3 digits to 0
                    SS_PllReferenceFrequency += kHz;    // Now add in the kHz value.
                    theMenu[SS_MenuState  ].value = SS_PllReferenceFrequency;
                    theMenu[SS_MenuState-1].value = SS_PllReferenceFrequency;
                    break;

                case MABAUDRATE :
                    SS_BaudrateIndex += SS_RotaryCount;
                    if (SS_BaudrateIndex < 0) SS_BaudrateIndex = 0;
                    if (SS_BaudrateIndex > baudrateLength) SS_BaudrateIndex = baudrateLength;

                    theMenu[SS_MenuState].value = (int32_t)SS_BaudrateIndex;
                    SS_Baudrate = Baudrates[SS_BaudrateIndex];
                    break;

                case MAROTARYTYPE :
                    SS_RotaryType += SS_RotaryCount;
                    if (SS_RotaryType < 0) SS_RotaryType = 0;
                    if (SS_RotaryType > 1) SS_RotaryType = 1;
                    theMenu[SS_MenuState].value = (int32_t)SS_RotaryType;
                    break;

                case MAREMOTEENABLE:
                    SS_RotaryCount = SS_RotaryCount % 2;
                    SS_RemoteEnable = (SS_RotaryCount==1) ? TRUE : FALSE;
                    theMenu[SS_MenuState].value = SS_RemoteEnable;
                    break;

                case MAFRONTENABLE :
                    SS_RotaryCount = SS_RotaryCount % 2;
                    SS_FrontEnable = (SS_RotaryCount==1) ? TRUE : FALSE;
                    theMenu[SS_MenuState].value = SS_FrontEnable;
                    break;
            }
            // swallow the rotary pulses used
            SS_RotaryCount = 0;
        } 
    }

    // }}}

    // }}}
    // {{{ // Selector Button

    // {{{ Select pushed during tuning

    if (SS_Selected && SS_Tuning && !SS_Transmitting)
    {
        SS_Selected = FALSE;            // swallow the selection action

        // handle the selector button pushed
        SS_Selected = FALSE;        // swallow the selection action
        SS_MenuState = MAINMENU;    // always start here
        SS_Tuning   = FALSE;        // rotary input now goes to menu
        SS_Scanning = FALSE;        // stop scanning on entering menu
        SS_ValueEdit= FALSE;
    }

    // }}}
    // {{{ Select pushed during Menu browing 

    if (SS_Selected && !SS_Tuning && !SS_ValueEdit) 
    {
        // here we are already in menu handling mode
        SS_Selected = FALSE;        // swallow the selection action
        switch (SS_MenuState)
        {
            case MBACK2TUNE :
                SS_Tuning = TRUE;   // switch to tuning mode
                prevFreq = 0L;      // force update of freq dispaly
                break;

            case MABACK2MAIN :
                SS_MenuState = MAINMENU;
                break;

            case MSCAN :
                SS_Tuning = TRUE;   // switch to tuning mode
                SS_Scanning = TRUE; // and go to scanning mode
                prevFreq = 0L;      // force update of freq display
                break;

            case MSETTINGS :        // Goto the first submenu
                SS_MenuState = SUBMENU1;
                break;

            default :  
                SS_ValueEdit = TRUE;
        }
    }

    // }}}
    // {{{  Select pushed during Value editing

    if (SS_Selected && !SS_Tuning && SS_ValueEdit) 
    {
        SS_Selected = FALSE;        // swallow the selection action

        // here we are already in menu handling mode AND editing values
        if (!SS_Tuning && SS_ValueEdit) 
        {
            SS_ValueEdit = FALSE;   // return from value editing
            SS_Tuning = SS_DirectMenuReturn;
            prevFreq = 0L;          // force update of freq display
            // and write value to persistent memory
            // need to write proper persistent storage routine for that
        }
        switch (SS_MenuState)
        {
            case MMUTELEVEL :
                SS_MuteLevel = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MMUTELEVEL*sizeof(uint32_t)),theMenu[MMUTELEVEL].value);
                break;

            case MSHIFT :
                SS_FrequencyShift = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MSHIFT*sizeof(uint32_t)) ,theMenu[MSHIFT].value);
                break;

            case MCTCSS:
                SS_CtcssIndex = theMenu[SS_MenuState].value;
                SS_CtcssFrequency = CtcssTones[SS_CtcssIndex];
                TimerValue = 5*F_CPU/SS_CtcssFrequency; // *10/2
                eeprom_write_dword((uint32_t *)(MCTCSS*sizeof(uint32_t)),theMenu[MCTCSS].value);
                break;

            case MSSTART :
                SS_ScanStartFrequency = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MSSTART*sizeof(uint32_t)) ,theMenu[MSSTART].value);
                break;

            case MSEND :
                SS_ScanEndFrequency= theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MSEND*sizeof(uint32_t)) ,theMenu[MSEND].value);
                break;

            case MABAUDRATE :
                SS_BaudrateIndex = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MABAUDRATE*sizeof(uint32_t)) ,theMenu[MABAUDRATE ].value);
                break;

            case MAROTARYTYPE :
                SS_RotaryType = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MAROTARYTYPE*sizeof(uint32_t)) ,theMenu[MAROTARYTYPE].value);
                // reset the rotary input system
                break;

            case MAFRONTENABLE :
                SS_FrontEnable = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MAFRONTENABLE*sizeof(uint32_t)),theMenu[MAFRONTENABLE].value);
                break;

            case MAREMOTEENABLE :
                SS_RemoteEnable = theMenu[SS_MenuState].value;
                eeprom_write_dword((uint32_t *)(MAREMOTEENABLE*sizeof(uint32_t)),theMenu[MAREMOTEENABLE].value);
                break;
        }
    }

    // }}}

    // }}} end selector button

    // {{{ // Push To Talk

    switch (SS_PTT)
    {
        // activated
        case 1  :
            SS_Transmitting = TRUE;
            // if we are scanning, then stop now!
            SS_Scanning = FALSE;
            break;

            // released
        case 2  :
            SS_Transmitting = FALSE;
            break;
    }

    SS_TxRxIndicator = (SS_Transmitting) ? 'T' : 'R';

    // }}}
    // {{{ // Shift Enable

    switch (SS_ShiftChange)
    {
        // activated
        case 1  :
            SS_ShiftEnable = TRUE;
            break;

            // released
        case 2  :
            SS_ShiftEnable = FALSE;
            break;
    }


    // }}}
    // {{{ // SMeter and Squelch 

    if (!SS_Transmitting)
    {
        //SS_DisplaySMeter = ((1024-SS_SMeterIn) - 44) >> 1;
        SS_DisplaySMeter = (980 - SS_SMeterIn) >> 1;

        // low pass s-meter signal
        SS_DisplaySMeter += LowPass;
        SS_DisplaySMeter >>= 1;
        LowPass = SS_DisplaySMeter;

        SS_Muted = (SS_MuteLevel > SS_DisplaySMeter);
    } else {
        SS_Muted = TRUE;
    }
    SS_MuteIndicator = (SS_Muted) ? 'M' : ' ';

    // }}}
    // {{{ // Scanner
#ifdef TESTING
    // stop scanning when found a busy channel
    if (!SS_Muted) SS_Scanning = FALSE;

    if (SS_Scanning)
    {   
        uint16_t StepDelay=125;
        clock_t now = clock();
        currentTime = (float)now * 1000.0F / CLOCKS_PER_SEC;
        goStep = (currentTime - prevStepTime) > StepDelay;
        if (goStep)
        {   
            prevStepTime = currentTime;
            SS_BaseFrequency =
                (SS_BaseFrequency > SS_ScanEndFrequency) ? SS_ScanStartFrequency : SS_BaseFrequency+CHANNELSTEP;
        }
    }   
#else
    // $$$ FHE TODO
#endif
    // }}}
    // {{{ // Frequency Calculations
    int32_t offset;

    SS_VfoFrequency     = SS_BaseFrequency - ((SS_Transmitting) ? 0L : IF);
    SS_DisplayFrequency = SS_BaseFrequency;

    //                shift  AND (ptt            XOR  reversShift)
    offset = (SS_ShiftEnable && (SS_Transmitting != SS_ReverseShift)) ? SS_FrequencyShift : 0L;
    SS_VfoFrequency     += offset;
    SS_DisplayFrequency += offset;

    // }}}

}

// }}} Processing
// {{{ Output functions

// {{{ void OutputSetAudioMute(char mute)

#define MUTEINDICATOR 'M'
#define BLANK         ' '

void OutputSetAudioMute(char mute)
{
    // boolean mute;
    static char prevMute;

    if (prevMute != mute)
    {
        prevMute = mute;
        if (mute) 
        {
            sbi(PORTC, MUTE);
        } else {
            cbi(PORTC, MUTE);
        }
    }
}


// }}}
// {{{ void OutputSetVfoFrequency(int32_t vfoFreq)

void OutputSetVfoFrequency(int32_t vfoFreq)
{
    static int32_t prevVfo;
    int32_t reg, frast;
    int32_t fRasterHigh, fRasterLow;

    if (prevVfo != vfoFreq)
    {
        prevVfo = vfoFreq;
        frast = vfoFreq / CHANNELSTEP;
        fRasterHigh = frast/16;
        fRasterLow  = frast%16;

        reg = ((fRasterHigh & 0x1fff)<<8) + ((fRasterLow & 0x3f)<<2) + 1;
        OutputSetPLL(reg);
    }
}

// }}}
// {{{ void OutputSetPLLReference(int32_t reference)

void OutputSetPLLReference(int32_t reference)
{   
    int32_t reg;
    // init R-counter
    reg = (2UL<<16) + ((SS_PllReferenceFrequency/CHANNELSTEP)<<2);
    OutputSetPLL(reg);
}

// }}}
// {{{ void OutputSetPLL(int32_t r)

void OutputSetPLL(int32_t r)
{
    char i;

    for (i=0; i<24; i++) 
    {
        if (r & 0x800000)
            sbi(PORTC, ADATA);
        else
            cbi(PORTC, ADATA);

        // output the int32_t word via bit banging
        _delay_us(1);
        sbi(PORTC, ACLK);
        _delay_us(1);
        cbi(PORTC, ACLK);
        r <<= 1;
    }

    // activate the latch
    _delay_us(1);
    sbi(PORTC, ALE);
    _delay_us(1);
    cbi(PORTC, ALE);
}

// }}}
// {{{ void OutputSetCtcssFreq(int ctcssFreq) 

void OutputSetCtcssFreq(int ctcssFreq)
{
    TimerValue = 5*F_CPU/SS_CtcssFrequency; // *10/2
}

// }}}
// {{{ void OutputSetTransmitterOn(char value)

void OutputSetTransmitterOn(char tx)
{
    if (tx)
    {
        sbi(PORTC, TXON);
        sbi(PORTC, MUTE);
        // mute audio amp during transmit;
        OutputSetAudioMute(tx);
    }
    else
    {    
        cbi(PORTC, TXON);
    }
}

// }}}
// {{{ void OutputSetDisplayFrequency(int32_t freq)

void OutputSetDisplayFrequency(int32_t freq)
{

    if (prevFreq != freq)
    {
        prevFreq = freq;
#ifdef TESTING
        sprintf(Line, "VFO %4u.%03u MHz", freq/1000, freq%1000);
#else
        sprintf(Line, "VFO %4lu.%03lu MHz", freq/1000, freq%1000);
#endif

#ifdef TESTING
        // set color to blue
        if (!AutoTest) printf("\033[34m");
#endif
        lcdHome();
        lcdStr16(Line);
#ifdef TESTING
        // reset color to black
        if (!AutoTest) printf("\033[30m");
#endif
    }
}

// }}}
// {{{ void OutputSetDisplaySMeter(short sValue)

void OutputSetDisplaySMeter(uint16_t sValue)
{
#ifdef TESTING
#define C0  '.'
#define C1  '\''
#define C2  '"'
#else
#define C0  0
#define C1  1
#define C2  2
#endif
#define MaxSMeter 42

    if (sValue > MaxSMeter) sValue = MaxSMeter;
    // first position is for the Mute indicator
    // last position is for the TxRx indicator
    // which leaves the display (width - 2) for the S-Meter
    int8_t n = DISPLAY_WIDTH-2;

    lcdCursorPosition(1,1); // goto second line

    // characters in the full bar are 3 lines
    while ((sValue >= 3) & (n>0))
    {
        // write character 3 (3 lines)
        lcdData(C2);
        sValue -= 3;
        n--;
    }

    // last char 0, 1 or 2 lines
    switch (sValue) 
    {
        case 2 : lcdData(C1); 
                 break;
        case 1 : lcdData(C0); 
                 break;
                 // default: // nothing to print
    }

    // clear any remaining characters to the right
    while (--n > 0) 
    {
        lcdData(' ');
    }
    /*
       sprintf(Line,"  %5d", sValue);
       lcdCursorPosition(1,1);
       lcdStr(Line);
     */
}

// }}}
// {{{ void OutputSetDisplayTxRxIndicator(char indicator)

// only write to the (slow) display if the indicator
// really should be updated

void OutputSetDisplayTxRxIndicator(char indicator)
{
    static char prevIndicator;

    if (prevIndicator != indicator)
    {
        prevIndicator = indicator;
        lcdCursorPosition(1, DISPLAY_WIDTH-1);
        lcdData(indicator);
    }
}

// }}}
// {{{ void OutputSetDisplayMuteIndicator(char indicator)

void OutputSetDisplayMuteIndicator(char indicator)
{
    lcdCursorPosition(1,0);  // bottom row, first column;
    lcdData(indicator); 
} 

// }}}

// {{{ void TopLinePrinter(uint16_t ix)

void TopLinePrinter(uint16_t ix)
{
    char *prompt = (SS_ValueEdit) ? "  " : "> ";

    lcdHome();
    sprintf(Line,"%s%-14s", prompt, theMenu[ix].name);
    lcdStr(Line);
}

// }}}
// {{{ void BottomLinePrinter(uint16_t ix)

void BottomLinePrinter(uint16_t ix)
{
    int32_t val;
    int8_t slength = -1;
    uint8_t i;
    char *prompt = (SS_ValueEdit) ? "> " : "  ";
    char *valStr;
    val = theMenu[ix].value;

    // goto bottom line
    lcdCursorPosition(1,0);

    switch (ix)
    {
        case MMUTELEVEL :
            slength = sprintf(Line, theMenu[ix].format, prompt, val);
            break;
        case MSSTART :
        case MSEND :
            slength = sprintf(Line, theMenu[ix].format, prompt, val/1000, val%1000);
            break;

        case MCTCSS :
            slength = sprintf(Line, theMenu[ix].format, prompt, CtcssTones[val]/10, CtcssTones[val]%10);
            break;

        case MABAUDRATE :
            slength = sprintf(Line, theMenu[ix].format, prompt, Baudrates[val]);
            break;

        case MAROTARYTYPE :
            if (SS_RotaryType == 1)
                valStr = "Step per pulse";
            else
                valStr = "Step per cycle";
            slength = sprintf(Line, theMenu[ix].format, prompt, valStr);
            break;

        case MARETURNMODE :
            slength = sprintf(Line, theMenu[ix].format, prompt, (SS_DirectMenuReturn) ? "to tuning" : "to menu");
            break;

        case MAFRONTENABLE :
        case MAREMOTEENABLE :
            slength = sprintf(Line, theMenu[ix].format, prompt, (val) ? "Enabled" : "Disabled");
            break;

        case MSHIFT :
        case MAPLLREFMHZ :
        case MAPLLREFKHZ :
            slength = sprintf(Line, theMenu[ix].format, prompt, val/1000, val%1000);
            break;

        default:
            slength = 0;
    }
    if (slength > -1)
    {
        for (i=slength; i<=DISPLAY_WIDTH; i++)
            Line[i] = ' ';                  // clear rest of line
        Line[i] = (uint8_t)0;               // set string terminator
    }
    lcdStr(Line);
}

// }}}

// {{{ Display control routines

// {{{ void lcdNib(char c)

void lcdNib(char nibble)
{
#if FALSE // for debugging
    unsigned char bcpy;
    bcpy = PORTB & 0x0C;    // clear high nibble
    nibble = nibble & 0xF3; // clear low nibble
    nibble = nibble + bcpy; // combine log nibble of PORTB with high nibble of NIBBLE
    // nibble now containts the content of nibble + the original 4 low bits of PORTB
#endif

    PORTB = nibble;
    sbi(PORTB, LCD_E);
    _delay_us(2);
    cbi(PORTB, LCD_E);
    _delay_us(200);
}

// }}}
// {{{ void lcdCmd(char c)

void lcdCmd(char c)
{
#ifdef TESTING
    deCmd(c);
#else
    lcdNib(c & 0xF0);
    lcdNib(c << 4);
#endif
}

// }}}

// {{{ void lcdHome(void)

void lcdHome(void)
{
#ifdef TESTING
    deTop();
#else
    lcdCmd(dispHOME);
    _delay_ms(20);
#endif
}

// }}}
// {{{ void lcdData(char c)

void lcdData(char c)
{
#ifdef TESTING
    // set color to blue
    //if (!AutoTest)
    //    printf("\033[34m");
    deData(c);
#else
    char t;

    t = c & 0xf0;   
    t |= (1<<LCD_RS);   
    lcdNib(t);

    c <<= 4;
    c |= (1<<LCD_RS);   
    lcdNib(c);
#endif
}

// }}}
// {{{ void lcdStr(char *s)

void lcdStr(char *s)
{
    char i=0;
    while ((*s) && (i<DISPLAY_WIDTH))
    {
        lcdData(*s++);
        i++;
    }
}

// }}}
// {{{ void lcdStr16(char *s)

void lcdStr16(char *s)
{
    uint8_t i;
    for (i=0; i<DISPLAY_WIDTH; i++)
        lcdData(*s++);
#ifdef TESTING
    printf("\n");
#endif
}

// }}}
// {{{ void lcdCursorPosition(int row, int column)

void lcdCursorPosition(int row, int column)
{
    // constants defined according to the datasheet
#define row1 0x80
#define row2 0xC0

    unsigned char position;

    // guarantee the inputs are in range
    row = row % DISPLAY_HEIGHT;  
    column = column % DISPLAY_WIDTH;

    position = row1 + column;   // coloumn + 0x80

    if (row == 1)
        position += (row2-row1);  // coloumn + 0xC0

    // write position to display command register
    lcdCmd(position);
}

// }}}}
// }}}

// {{{ void OutputHandler(void)

void OutputHandler(void)
{
#ifdef TESTING
    deFrame();
#endif

    OutputSetCtcssFreq(SS_CtcssFrequency);
    OutputSetAudioMute(SS_Muted);

    if (SS_Tuning)
    {
        OutputSetDisplayFrequency(SS_DisplayFrequency);
        OutputSetDisplaySMeter(SS_DisplaySMeter);
        OutputSetDisplayTxRxIndicator(SS_TxRxIndicator);
        OutputSetDisplayMuteIndicator(SS_MuteIndicator);
    } else // "not Tuning" means "in menu"
    {
        OutputSetDisplayTxRxIndicator(' ');
        OutputSetDisplayMuteIndicator(' ');

        TopLinePrinter   (SS_MenuState);
        BottomLinePrinter(SS_MenuState);
    }

    OutputSetVfoFrequency (SS_VfoFrequency);
    OutputSetTransmitterOn(SS_Transmitting);

    // {{{ testing and debugging
#ifdef TESTING

    if (TRUE)
    {
        if (!AutoTest) ttySetCursorPosition(DBGROW,0);
        if (!AutoTest) printf("\033[37m"); // light gray
        printf("\n========================= debug ========================"); 
        NL();
        printf("CTCSSindex      =      %3d  | ",SS_CtcssIndex);
        printf("SS_Tuning       =        %d",SS_Tuning);
        NL();
        //            printf("inputStateRotary= %8d\n",inputStateRotary);

        printf("CTCSSfrequency  = %8d  | ",SS_CtcssFrequency);
        printf("SS_MenuState    =     %04X",SS_MenuState ); 
        NL();

        printf("MuteLevel       = %8d  | ",SS_MuteLevel);
        printf("SS_ValueEdit    = %8d",SS_ValueEdit);
        NL();

        printf("ShiftEnable     =      %3s  | ",yesno(SS_ShiftEnable));
        //printf("menuLoopState T =     %04X",menuLoopState & TYPEMASK); 
        NL();

        //            printf("LargeEnable     =      %3s  | ",yesno(SS_LargeStepEnable));
        //            printf("\n");

        printf("FrequencyShift  = %8d  | ",SS_FrequencyShift);
        printf("BaseFrequency   = %8d",SS_BaseFrequency); 
        NL();

        printf("Transmitting    =      %3s  | ",yesno(SS_Transmitting));
        printf("VfoFrequency    = %8d",SS_VfoFrequency);
        NL();

        printf("Scanning        =      %3s  | ",yesno(SS_Scanning));
        printf("SS_PllReference = %4u.%03u", SS_PllReferenceFrequency/1000, SS_PllReferenceFrequency%1000);
        NL();

        printf("Scan Start      = %4u.%03u  | ", SS_ScanStartFrequency/1000, SS_ScanStartFrequency%1000);
        printf("Scan End        = %4u.%03u", SS_ScanEndFrequency/1000, SS_ScanEndFrequency%1000);
        NL();

        printf("SS_SMeterIn     =    %5d  | ", (int)SS_SMeterIn);
        printf("SS_DisplaySMeter=    %5d", (int)SS_DisplaySMeter);
        NL();

        printf("SS_RotaryCount  =    %5d  | ", SS_RotaryCount);
        printf("SS_Selected     =        %1d", SS_Selected);
        NL();

        printf("tmpFreqChanged  =    %5d  | ", tmpFreqChanged);
        printf("tmpFreqSaved    =    %5d ", tmpFreqSaved);
        NL();

        // printf("lastFrequencyChange= %8lu ",lastFrequencyChange);
        // printf("now                = %8lu",now);
        // NL();
        // printf("clocks per sec = %d ",CLOCKS_PER_SEC);
        // NL();

        // printf("ctcssIndex      = %8d\n",ctcssIndex);
        // printf("vInRotState     = %8d\n",vInRotState);
        // printf("keypressed  = %8X\n",theKey);
        // printf("unget charvail  = %8s\n",yesno(GetcAvail));
        printf("========================================================\n"); NL();
        if (!AutoTest) printf("\033[30m"); // black
    }
    else
    {
        deSetCursorPosition(DBGROW,1);
    }
#endif
    // }}}
}

// }}} Output handling

// }}}

// {{{ void mainLoop(void)

void mainLoop(void)
{
    char busy = TRUE;
    while (busy)
    {
        busy = InputHandler();
        RemoteControlHandler();
        ProcessingHandler();
        OutputHandler();        

        tbi(PORTB, 2); // for measuring loop timing

    }
}

// }}}
// {{{ int main(int argc, char *argv[])

int main(int argc, char *argv[])
{
    // {{{ testing

#ifdef TESTING
    if (argc>1)
    {
        if (strcmp(argv[1],"-a") == 0)
            AutoTest = TRUE;
        if (strcmp(argv[1],"-d") == 0)
            dbg_logging = TRUE;
    }

    if (!AutoTest)
    {
        ttyClearScreen();
        ttySetCursorPosition(0,0);
        printf("========= 23cm NBFM control software simulator =========\n");
        printf("\n");
        printf("q quit\n"); 
        printf("[ downward rotating         ] upward rotating\n"); 
        printf("t transmit                  r receive\n"); 
        printf("s shift on                  a shift off\n");
        printf("e menu selector button      l toggle large steps on/off\n"); 
        printf("\n");
        set_conio_terminal_mode();
    }
    if (dbg_logging) dbg = fopen("log.txt","w");
#endif

    // }}}

    initialize();
    mainLoop();

    // {{{ testing

#ifdef TESTING
    eeprom=fopen("eeprom.bin","w");
    for (int i=0; i<16; i++)
        fwrite(&theMenu[i].value, sizeof(int32_t),1, eeprom);
    fclose(eeprom);

    if (dbg_logging) fclose(dbg);
    // position cursor below lowest printed line
    if (!AutoTest) ttySetCursorPosition(27,0);
    // cursor back on
    if (!AutoTest) printf("\033[?25h");   
    // short i;
    // for (i=0x20; i<255; i++) printf("%X-%c ", i,i);
#endif

    // }}}
}

// }}}

// {{{ 



// }}}
// EOF
