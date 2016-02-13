// vim: ts=4 et sw=4 foldmethod=marker
// zR open all folds    zM close all folds   zf define fold
// {{{ Documentation
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
// }}}
// #define TESTING "this is the simulation. #UNDEF for the real thing" 
// #define  DBG_LOGGING "used during testing to log debug info"
// {{{ includes

// F_CPU used in util/delay.h
#define F_CPU       8000000UL

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
// {{{ defines

#define TRUE (1==1)
#define FALSE (!TRUE)

#define IF                  69300UL     // in kHz
#define INITIAL_FREQUENCY   1298200UL   // in kHz    
#define INITIAL_SHIFT       -28         // in MHz
#define INITIAL_CTCSS       0           // in cHz
#define INITIAL_MUTELEVEL   5           // scalar
#define INITIAL_REFERENCE   12000UL     // in kHz
#define CHANNELSTEP         25          // in kHz
#define LARGESTEP           1000        // in kHz
#define BANDBOTTOM          1240000UL   // in kHz
#define BANDTOP             1300000UL   // in kHz

#define DISPLAY_WIDTH       16
#define LINECOUNT           2

// {{{ input events
#define IDLE                -10
#define UPEVENT             1
#define DOWNEVENT           2
#define TXEVENT             3
#define RXEVENT             4
#define SELECTEVENT         5
#define SELECTBOUNCING      6
#define SELECTWAITRELEASE   7
#define SHIFTONEVENT        8
#define SHIFTOFFEVENT       9
#define LARGEEVENT          10
// }}}

// {{{ States

// Lower 6 bits are for state indicator (64 state range)
// Higher order 10 bites are for type indicator (1024 types range)

#define STATEMASK           0x002F
#define TYPEMASK            0xFFC0

#define TUNING              0
// {{{ Main menu select states
#define MAINMENU            64
#define MMUTELEVEL          (MAINMENU+0)
#define MSHIFT              (MAINMENU+1)
#define MCTCSS              (MAINMENU+2)
#define MSSTART             (MAINMENU+3)
#define MSEND               (MAINMENU+4)
#define MSCAN               (MAINMENU+5)
#define MSETTINGS           (MAINMENU+6)
#define MBACK               (MAINMENU+7)
#define MAINMENU_END        (MAINMENU+7)
// }}}
// {{{ Main menu value states
#define MAINMENU_VAL        128
#define MMUTELEVELVAR       (MAINMENU_VAL+0)
#define MSHIFTVAR           (MAINMENU_VAL+1)
#define MCTCSSVAR           (MAINMENU_VAL+2)
#define MSSTARTVAR          (MAINMENU_VAL+3)
#define MSENDVAR            (MAINMENU_VAL+4)
#define MAINMENU_VAL_END    (MAINMENU_VAL+4)
// }}}
// {{{ Submenu states
#define SUBMENU1            192
#define MARETURNMODE        (SUBMENU1+0)
#define MAREFFREQ           (SUBMENU1+1)
#define SUBMENU1_END        (SUBMENU1+1)
// }}}
// {{{ Submenu value states
#define SUBMENU1_VAL        256
#define MARETURNMODEVAR     (SUBMENU1_VAL+0)
#define MAREFFREQVAR        (SUBMENU1_VAL+1)
#define SUBMENU1_VAL_END    (SUBMENU1_VAL+1)
// }}}

// }}} States

#define MAXMUTELEVEL        32
#define MINSHIFT            -60
#define MAXSHIFT            60
#define MINCTCSS            10
#define MAXCTCSS            90

#define SELECTBOUNCEDELAY   1   // mainloop cycle time = 34 ms.

#ifdef TESTING
#define DE_WIDTH    16
#define DE_HEIGHT    2
#define YTop        10
#define XTop        21
#define DBGROW      14
#endif

// }}}
// {{{ defines for ATMEGA328 
#ifdef TESTING

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

//short tx;
//long  freq;
//short shiftSwitch;
//short shift;
//char str[50];
short ports[10];
#define PORTB ports[1]
#define PORTC ports[2]

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
#define RCLK        PD3
#define CLKMASK     (1<<RCLK)
#define RDAT        PD4
#define DATAMASK    (1<<RDAT)

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
#ifdef TESTING
// {{{ All of these should be removed

unsigned char _BV(unsigned char c) { return c; }
void _delay_us(short s) {} 
void _delay_ms(short s) {} 

// }}}
#endif
// {{{ Function Prototypes

void updateDisplay(void);
void showTrx(void);
void setFreq(long f);
void initPLL(void);
void initLCD(void);
void initADC(void);
void deFrame(void);
void lcdCmd(char c);
void lcdData(char c);
void lcdHome(void);
void lcdNib(char);
void setPLL(long c);
void setMuted(void);
void showFrequency(short r, long f);
void lcdStr(char *s);

// }}}
// {{{ Globals

#ifdef TESTING
short goingUp;               // tmp global
short LoopCounter;           // tmp global
short cpos;                  // display: lineair cursor position
#endif
short currentTime;           // tmp global
short prevStepTime;
short goStep;

char GClkPrev;               // used for rotary dial handling
char GQdPrev;                // used for rotary dial handling
char GQd;                    // used for rotary dial handling

char DisplayDirty;           // indicates the display buffer has been updated

short LowPass;               // Variable for S-meter lowpass filter

// {{{ System State variables

volatile int   SS_RotaryChange;    // amount of steps to take 
volatile char  SS_SelectorPushed;  // boolean: Pushed = true; Idle = false;

char  SS_Transmitting;          // boolean: TX = true, RX = false;
char  SS_Scanning;              // boolean: scanning = true;
char  SS_Muted;                 // boolean: TRUE=audio muted, FALSE=audio on 
char  SS_ShiftEnable;           // boolean: shifted = true;
char  SS_ReverseShift;          // boolean: swap transmit and receive frequencies = true
char  SS_CtcssIndex;            // the tone value to inject
int   SS_MenuState;             // state of the current user input menu
int   SS_MuteLevel;             // level below which the audio will be muted
long  SS_FrequencyShift;        // shift value to use during repeater shift
long  SS_BaseFrequency;         // tuned with the rotary dial. All freqs are derived from this var.
long  SS_VFOFrequency;          // the frequency to VFO must be tuned to 
long  SS_DisplayFrequency;      // frequency to show on display
int   SS_DisplaySMeter;         // value to show on display

char  SS_DirectMenuReturn;      // boolean:
long  SS_PllReferenceFrequency; // frequency that is used as PLL reference

// }}}  System State variables

char  LargeStepEnable;       // boolean: channel steps are 10x as big
short FrequencyShift;        // size of frequency shift
short MuteLevel;             // level below which audio muting is enabled
short inputSelectDebounce;   // counter for bounce delay period
char  CTCSSenable;           // boolean: CTCSS tone enabled during transmit
short CTCSSfrequency;        // frequency of CTCSS tone to use
long  DialFrequency;         // frequency as set by dial
long  CurrentFrequency;      // frequency setting sent to synthesizer
long  ScanStartFrequency;    // duh...
long  ScanEndFrequency;      // duh...
long  PllReferenceFrequency; // duh...
char  DirectMenuReturn;      // When true: exit the menu upon a value selection.
char  Line[DISPLAY_WIDTH+10];
char  DisplayBuffer[LINECOUNT][DISPLAY_WIDTH+5];

// Menu
// 
#define MENULENGTH 8
char *menuStrings[] = { 
    //234567890123456
    "Mute level",       // 0
    "Shift",            // 1
    "CTCSS",            // 2
    "Scan Start",       // 3
    "Scan End",         // 4
    "Scan",             // 5
    "Settings",         // 6
    "Back"};            // 7

long  menuValues [] =  {  
    INITIAL_MUTELEVEL,  // 0
    INITIAL_SHIFT,      // 1
    INITIAL_CTCSS,      // 2
    1280000L,           // 3
    1290000L,           // 4
    0,                  // 5
    0,                  // 6
    0};                 // 7

#define SUBMENULENGTH 2
char *subMenuStrings1[] = {
    //234567890123456
    "Select action:",   // 0
    "PLL Ref:"          // 1
};

long subMenuValues1 [] = {
    TRUE,               // 0
    INITIAL_REFERENCE   // 1
};

short menuLoopState;

// define S-meter chars
unsigned char smeter[3][8] = {
    {0b00000,0b00000,0b10000,0b10000,0b10000,0b10000,0b00000,0b00000},
    {0b00000,0b00000,0b10100,0b10100,0b10100,0b10100,0b00000,0b00000},
    {0b00000,0b00000,0b10101,0b10101,0b10101,0b10101,0b00000,0b00000}

    //CTCSS frequencies
    short CtcssTones[] = {   0, 670, 689, 693, 710, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974,
        1000,1035,1072,1109,1148,1188,1230,1273,1318,1365,1413,1462,1514,1567,1598,
        1622,1655,1679,1713,1738,1773,1799,1835,1862,1899,1928,1966,1995,2035,2065,
        2107,2181,2257,2291,2336,2418,2503,2541, -1};
    short ctcssIndex;

    short refFreqPLL[] = { 13000, 12000, 10700, -1 };
    short refFreqIndex;

#ifdef TESTING
    short  theMainEvent;
    char   GetcBuffer;
    short  GetcAvail;
    struct termios orig_termios;
#ifdef DBG_LOGGING
    FILE *dbg;
#endif
#endif

    // input states controls
#ifdef TESTING
    char vInRotState=9;
    //short S;
#endif
    char inputStateRotary;
    char inputStateSelector;
    char inputStateShift;
    char inputStatePTT;
    char inputStateSMeter;
    char inputStateLargeStep;

    // }}}

// {{{ Interrupt Service Routines

// {{{ Rotary Pulse

ISR(INT1_vect)
{
    // 
    // -+   +---+   +---+
    //  +---+   +---+   +---   Data
    //
    //    |   |   |   |   |     
    //
    // ---+   +---+   +---+
    //    +---+   +---+   +-   Clock
    //   0 1 2 3 4 5 6 7 8 9
    // =========================
    //                        +----------------------> rising edge -> frequency step
    //                        |
    //         double         |           positive
    //      edge triggerd     |        edge triggered
    //        +-------+       |           +-------+
    // DATA---| D   Q |--(Qd)-+     DATA--| D   Q |--> UpDown
    // CLK-+--|>      |         +---------|>      |
    //     |  |    /Q |         |         |    /Q |
    //     |  +-------+         |         +-------+
    //     +--------------------+
    //
    // Given a 90 degrees out of phase DATA and CLOCK signal that both have 
    // bounce during the level change, Qd will be a cleaned up clock signal.
    // Using this cleaned up CLOCK (Qd), a single frequency step per cycle
    // can be guaranteed, regardless of bouncing contacts.
    // Using the rising edge of CLOCK to sample the DATA in its stable periods 
    // will provide a clean UpDown signal.
    // On the rising Qd edge, the UpDown signal differentiates between rotate 
    // up and rotate down

    /* read the port D input values             */
    sample = PIND;

    /* isolate data signal                      */
    data = (sample & DATAMASK) != 0;
    clk  = (sample & CLKMASK)  != 0;

    /* Double edge triggered D-flipflop creates */
    /* clean clock pulses output on Qd          */

    /* Any clock change will do                 */
    /* Qd will be used to guarantee single freq */
    /* steps dispute bounce                     */

    /* The interrupt is caused by a change in   */
    /* so sample the data signal                */
    GQd = data;      

    /* Positive edge triggered D-flipflop       */ 
    /* creates  a directional value  (UpDown)   */

    /* Trigger only on rising edge of CLK       */
    if (clk && !GClkPrev) 
    {                    
        /* The data signal is stable at this moment */
        UpDown = data;
    }

    /* Trigger only on clean clock (Qd) going   */
    /* high.                                    */
    if (GQd & !GQdPrev)  
    {                    
        if (UpDown)
            SS_RotaryCount++;
        else
            SS_RotaryCount--; 
    }

    GClkPrev = clk;
    GQdPrev  = GQd;
}

// }}}
// {{{ Selector Pulse

ISR(INT0_vect)
{
    // The selector line has gone low, so the button was pushed
    SS_SelectorPushed = TRUE;
}

// }}}
// {{{ Timer



// }}}

// }}}

    // {{{ Initialisation

    // {{{ void initIRQ(void)

void initIRQ(void)
{
    // {{{ Rotary Dial Clk input (PD3)

    DDRD &= ~(1 << DDD3);     // Clear the PD3 pin
    // PD3 (INT1 pin) is now an input

    PORTD |= (1 << PORTD3);    // turn On the Pull-up
    // PD3 is now an input with pull-up enabled


    // External Interrupt Control Register A
    // set Interrupt Sense Control ISC11 and ISC10 to 01 for dual edge trigger
    EICRA |= (1 << ISC10);    // set INT1 to trigger on ANY logic change
    // External Interrupt Mask register
    EIMSK |= (1 << INT1);     // Turns on INT1

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



    // }}}

    // Enable global interrupts
    sei(); 
}

// }}}
// {{{ void initLCD(void)
void initLCD(void)
{
#ifdef TESTING
    deFrame();
    showTrx();
#endif

    // allow the lcd controller to wake up
    _delay_ms(100);

    // depending on intial state:
    // ... force to state 1 or state 3
    lcdNib(0x30);
    _delay_ms(10);

    // ... and force to state 1
    lcdNib(0x30);
    _delay_ms(10);

    // 4 bits mode, 2 lines
    lcdCmd(dispFUNC); // 0x20
    _delay_ms(10);

    //lcdCmd(dispFUNC);
    //_delay_ms(10);

    // cursor shifts to right, text no shift
    lcdCmd(dispSHIF); // 0x18
    _delay_ms(20);

    // display on, no cursor, no blink
    lcdCmd(dispONOF); // 0x0C
    _delay_ms(20);

    // shift mode
    lcdCmd(dispMODE); // 0x06
    _delay_ms(20);

    // clear display
    // leave cursor at top left
    lcdCmd(dispCLEAR); // 0x01
    _delay_ms(20);

    /*
    // define custom chars
    short i,j;
    lcdCmd(dispCGRA);
    for (i=0; i<3; i++) 
    {
    for (j=0; j<8; j++) 
    {
    lcdData(smeter[i][j]);
    }
    }
     */

    // welcome message 
    lcdHome();
    _delay_ms(20);

    //              0123456789ABCDEF
    char hello[] = "PA3BJI sw v0.5 ";
    lcdStr(hello);
}


// }}}
// {{{ void initADC(void)

void initADC(void)
{
}

// }}}
// {{{ void initPLL(void)

void initPLL(void)
{
    long reg;
    PllReferenceFrequency = 13000UL;

    cbi(PORTC, DATA);
    cbi(PORTC, CLK);
    cbi(PORTC, LE);

    // set function latch
    reg = 0x438086;
    setPLL(reg);

    // init R-counter
    reg = (2UL<<16) + ((PllReferenceFrequency/CHANNELSTEP)<<2);
    setPLL(reg);

    setFreq(CurrentFrequency);

    reg = 0x438082;
    setPLL(reg);
}

// }}}
// {{{ void initIRQjpd(void)

void initIRQjpd(void)
{
#ifdef TESTING
#else

    /*
       EIMSK |= _BV(INT1);
       EICRA |= _BV(ISC11);

    // Setup Timer 1
    TCCR1A = 0x00;				// Normal Mode 
    TCCR1B = 0x01;				// div/1 clock, 1/F_CPU clock
    // $$$ FHE TODO toneValue = 5*F_CPU/tone;	// *10/2

    // Enable interrupts as needed 
    TIMSK1 |= _BV(TOIE1);  	// Timer 1 overflow interrupt 

    // enable interrupts
    sei();
     */
#endif
}

// }}}
// {{{ void initPORTS(void)

void initPORTS(void)
{
#ifdef TESTING
#else
    // PORTB output for LCD
    DDRB = 0xff;
    PORTB = 0xff;

    // PORTC PC0-4 output, PC5 input
    DDRC = 0x1f;
    PORTC = 0x00;

    // PORTD is input with pullup
    DDRD = 0x00;
    PORTD = 0xff;
#endif
}

// }}}
// {{{ void initialize(void)

void initialize(void)
{
    DialFrequency = INITIAL_FREQUENCY;
    FrequencyShift = INITIAL_SHIFT;
    ScanStartFrequency = INITIAL_FREQUENCY;
    ScanEndFrequency = BANDTOP;
    MuteLevel = INITIAL_MUTELEVEL;
    ShiftEnable = FALSE;
    ReverseShift = FALSE;
    Transmitting = FALSE;
    menuLoopState  = TUNING;
    inputStateRotary = IDLE;
    inputStateSelector= IDLE;
    inputStateShift= IDLE;
    inputStatePTT = IDLE;
    prevStepTime = 0;

    initPORTS();
    initPLL();
    initLCD();
    initADC();
    initIRQ(); // for now: do nothing

#ifdef TESTING
    vInRotState = 9;
    GetcAvail = FALSE;
    GetcBuffer = 0;
#endif
}

// }}}

// }}}
#ifdef TESTING
// {{{ Scaffolding

// {{{ ttyControl

// {{{ void ttySetCursorPosition(short row, short col)

void ttySetCursorPosition(short row, short col)
{
    printf("\033[%d;%df",row+1, col+1);
}

// }}}
// {{{ void ttyClearScreen(void)

void ttyClearScreen(void)
{
    printf("\033[2J");
    printf("\033[?1h");
}

// }}}

// }}}

// {{{ Display emulator
// {{{ deSetCursorPosition

void deSetCursorPosition(short row, short col)
{
    cpos = row*DISPLAY_WIDTH + col;
    ttySetCursorPosition(row+YTop, col+XTop);
}

// }}}
// {{{ deTop

void deTop(void)
{
    deSetCursorPosition(0,0);
}

// }}}
// {{{ deData

void deData(char c)
{
    printf("%c",c);
    cpos++; 
    if (cpos==DISPLAY_WIDTH)
        deSetCursorPosition(2,1);
}

// }}}
// {{{ deClearScreen

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
        DisplayBuffer[0][i]=' ';
        DisplayBuffer[1][i]=' ';
    }
}

// }}}
// {{{ deCmd

void deCmd(char c)
{
    short pos;

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
        pos = c & 0x7F;
        deSetCursorPosition((pos/DISPLAY_WIDTH)+1 , (pos%DISPLAY_WIDTH)+1);
    }
}

// }}}
// {{{ deFrame()

void deLine(short w)
{
    short i;
    printf("+");
    for (i=1; i<w; i++)
        printf("-");
    printf("+");
}

void deFrame(void)
{
    short y;

    // cursor off
    printf("\033[?25l");   

    deSetCursorPosition(-1, -1);
    deLine(DE_WIDTH+1);
    for (y=0; y<DE_HEIGHT; y++)
    {
        deSetCursorPosition(y, -1);
        printf("|");
        deSetCursorPosition(y, DE_WIDTH);
        printf("|");
    }
    deSetCursorPosition(DE_HEIGHT, -1);
    deLine(DE_WIDTH+1);
}

// }}}

// }}}

// {{{ char* yesno(short boolean)

char* yesno(short boolean)
{
    // green -> light gray
    static char yes[]="\033[32myes\033[37m";
    // red -> light gray
    static char no[] ="\033[31m no\033[37m";
    return (boolean ? yes : no); 
}

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

// }}}
#endif
// {{{ input functions

#ifdef TESTING
// {{{ short bounce()

#define bounceCLK   1
#define bounceDATA  2

short bounce(short clock, short data, short bounceSelect, short newState)
{
    short rv; // return value

    static short bounceCount;

    // bounce during the first 4 cycles
    // keep stable in the last 4 cycles

    if (bounceSelect == bounceCLK)
    {
        // clock bounces : on odd bounceCount values, return inverted clock value
        rv =  (bounceCount & 0x01) ?  clock | data : (clock ^ CLKMASK) | (data);
        //                            unchanged    one's complement   unchanged
    } else // bounceDATA
    {
        // data bounces : on odd bounceCount values, return inverted data value
        rv =  (bounceCount & 0x01) ?  clock | data : (clock) | (data ^ DATAMASK);
        //                            unchanged     unchanged   one's  complement
    }

    // after 4 times stop bouncing and return the true values
    //if (bounceCount > 4)
    rv = (clock | data);

    // rv = rv & 3; 
    // S = rv;

    // rotate through 8 cycles
    bounceCount++;
    //if (bounceCount > 8) 
    {
        bounceCount=0;
        vInRotState = newState;
    }
#ifdef DBG_LOGGING
    fprintf(dbg,"%02x ",rv);
#endif
    return rv;
}

// }}}
#endif

// feb 2016
// {{{ int  InputGetRotaryDialCount(void)

int InputGetRotaryDialCount(void)
{
    register int rv;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        rv = SS_RotaryChange;
        SS_RotaryChange = 0;
    }
    return rv;
}

// }}}
// {{{ char InputGetSelectorPushed(void)

char InputGetSelectorPushed(void)
{
    register int rv;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        rv = SS_SelectorPushed;
    }
    return rv;
}

// }}}
// {{{ char InputGetShiftEnable(void)
char InputGetShiftEnable(void)
{
    char ioreg;
    // sample value from PTT switch input
    ioreg = PIND;
    // write boolean to system-state
    SS_ShiftEnable = (0 != (ioreg & (1<<SHIFTKEY)))
}

// }}}
// {{{ char InputGetPTT(void)

char InputGetPTT(void)
{
    char ioreg;
    // sample value from PTT switch input
    ioreg = PIND;
    // write boolean to system-state
    SS_Transmitting = (0 != (ioreg & (1<<PTT)))
}

// }}}
// {{{ char InputGetSMeter(void)

char InputGetSMeter(void)
{
    register short s;

    ADCSRA |= (1<<ADSC)|(1<<ADEN); 
    while ((ADCSRA & (1<<ADSC))!=0);

    // calculate Smeter
    s = (1024-ADC)-44;

    // low pass s-meter signal
    s += LowPass;
    s >>= 1;
    LowPass= s;

    SS_DispaySMeter = s;
}

// }}}
// /feb 2016

// {{{ obsolete
// {{{ void getInputRotary(void)

void getInputRotary(void)
{
    char clk;
    char data;
    char sample;
    char UpDown=0;

#ifdef TESTING
    // =========================
    static char input;
    char c;
    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case ']' :
                vInRotState = 0;
                break;

            case '[' :
                vInRotState = 4;
                break;

            default :
                FHEungetc(c);
        }
    }

#ifdef DBG_LOGGING
    fprintf(dbg," vir-%02x, ch-%02x ",vInRotState, c);
#endif
    switch (vInRotState)
    {
        // up rotation
        case 0 :
            input = bounce(CLKMASK, 0,        bounceDATA, 1);
            break;

        case 1 :
            input = bounce(0,       0,        bounceCLK , 2);
            break;

        case 2 :
            input = bounce(0,       DATAMASK, bounceDATA, 3);
            break;

        case 3 :
            input = bounce(CLKMASK, DATAMASK, bounceCLK , 9);
            break;

            // down rotation
        case 4 :
            input = bounce(CLKMASK, DATAMASK, bounceDATA, 5) ;
            break;

        case 5 :
            input = bounce(0,       DATAMASK, bounceCLK , 6);
            break;

        case 6 :
            input = bounce(0,       0,        bounceDATA, 7);
            break;

        case 7 :
            input = bounce(CLKMASK, 0,        bounceCLK , 9);
            break;

        case 9 :
            vInRotState = 9;
            break;
    }

#ifdef DBG_LOGGING
    fprintf(dbg, "i-%02x ",input);
#endif
    // 
    // -+   +---+   +---+
    //  +---+   +---+   +---   Data
    //
    //    |   |   |   |   |     
    //
    // ---+   +---+   +---+
    //    +---+   +---+   +-   Clock
    //   0 1 2 3 4 5 6 7 8 9
    // =========================
#endif
    //                        +----------------------> rising edge -> frequency step
    //                        |
    //         double         |           positive
    //      edge triggerd     |        edge triggered
    //        +-------+       |           +-------+
    // DATA---| D   Q |--(Qd)-+     DATA--| D   Q |--> UpDown
    // CLK-+--|>      |         +---------|>      |
    //     |  |    /Q |         |         |    /Q |
    //     |  +-------+         |         +-------+
    //     +--------------------+
    //
    // Given a 90 degrees out of phase DATA and CLOCK signal that both
    // have bounce during the level change, Qd will be a cleaned up clock signal.
    // Using this cleaned up CLOCK (Qd), a single frequency step per cycle can be guaranteed, 
    // regardless of bouncing contacts.
    // Using the rising edge of CLOCK to sample the DATA in its stable periods will 
    // provide a clean UpDown signal.
    // On the rising Qd edge, the UpDown signal differentiates between rotate up and rotate down

#ifdef TESTING
    sample = input;
#else
    /* read the port D input values             */
    sample = PIND;
#endif

    /* isolate clock and data signals           */
    clk  = (sample & CLKMASK ) == 0;
    data = (sample & DATAMASK) == 0;

#ifdef DBG_LOGGING
    fprintf(dbg,"c-%02x d-%02x ",clk,data);
#endif
    /* Double edge triggered D-flipflop creates */
    /* clean clock pulses output on Qd          */

    /* Any clock change will do                 */
    /* Qd will be used to guarantee single freq */
    /* steps dispute bounce                     */
    if (clk != GClkPrev) 
    {
        /* sample the data signal               */
        GQd = data;      
    }

#ifdef DBG_LOGGING
    fprintf(dbg,"qd-%02x ",GQd);
#endif

    /* Positive edge triggered D-flipflop       */ 
    /* creates  a directional value  (UpDown)   */

    /* Trigger only on rising edge (CLK)        */
    if (clk && !GClkPrev) 
    {                    
        /* The data signal is stable at this moment */
        UpDown = data;
    }

    /* Trigger only on clean clock (Qd) going   */
    /* high.                                    */
    if (GQd & !GQdPrev)  
    {                    
        inputStateRotary = (UpDown) ? UPEVENT : DOWNEVENT;
    }


#ifdef DBG_LOGGING
    fprintf(dbg,"ud-%02x isr-%02x ",UpDown, inputStateRotary);
#endif

    GClkPrev = clk;
    GQdPrev  = GQd;

    // $$$ FHE DEBUG
    if (GQd) 
        sbi(PORTB,PB2);
    else
        cbi(PORTB,PB2);
    // $$$ FHE /DEBUG

}


// }}}
// {{{ void getInputSelector(void)

void getInputSelector(void)
{
#ifdef TESTING
    // {{{ testing
    char c;
    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 'e'  :
            case 0x0D  :
                inputStateSelector = SELECTEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
    // }}}
#else

#if FALSE
    // {{{ obsolete example
    // rotary button pushed?
    if (!(PIND & (1<<ROTKEY))) 
    {
        for (c=0;;c++) 
        {
            _delay_ms(200);// $$$ FHE TODO bounce suppressor (get rid of this) 
            // wait for button released
            if ((PIND & (1<<ROTKEY))) 
            {
                _delay_ms(200);// $$$ FHE TODO delay loop (get rid of this)
                break;
            }
        }
        if (c>5)
            inputStateSelector = LARGEEVENT;
        else
            inputStateSelector = SELECTEVENT;
    }
    // }}}
#endif

    // {{{ new selector code
    if (!(PIND & (1<<ROTKEY))) 
    {
        // run this if selector is pressed    
        switch (inputStateSelector)
        {   
            // this is a new push event
            case IDLE : 
                // signal selector pushed to the proces state machine
                inputStateSelector  = SELECTEVENT;
                // setup the bounce delay
                inputSelectDebounce = SELECTBOUNCEDELAY;
                break;

                // after the first push, we may bounce for some amount of cycles
            case SELECTEVENT : 
                inputStateSelector  = SELECTBOUNCING;
                break; 

                // decrease bounce delay unil bounce time passed
            case SELECTBOUNCING : 
                inputSelectDebounce--;
                inputStateSelector  = (inputSelectDebounce==0) ? SELECTWAITRELEASE : SELECTBOUNCING; 
                break; 

                // now we need to wait until the select button has been relased again.
                // the next "else" statement will bring us to the IDLE state
            case SELECTWAITRELEASE : 
                break;

                // any other state on selector pushed is illegal, 
                // so return to idle state
            default :
                inputStateSelector = IDLE;

        }
    }  else 
        // arrive here when selector not pressed (anymore)
    {
        // if we are waiting for release
        if (inputStateSelector == SELECTWAITRELEASE)
        {
            // then set state to idle
            inputStateSelector = IDLE; 
        }
    }
    // }}}
#endif
}

// }}}
// {{{ void getInputTxRx(void)

void getInputTxRx(void)
{
#ifdef TESTING
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 't' :
                inputStatePTT = TXEVENT;
                break;

            case 'r' :
                inputStatePTT = RXEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
#else

    char sw;
    sw = PIND;
    if (Transmitting && (sw & (1<<PTT)))
        inputStatePTT = RXEVENT;

    if (!Transmitting && !(sw & (1<<PTT)))
        inputStatePTT = TXEVENT;

#endif
}

// }}}
// {{{ void getInputShift(void)

void getInputShift(void)
{
#ifdef TESTING
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 'a'  :
                inputStateShift = SHIFTOFFEVENT;
                break;

            case 's'  :
                inputStateShift = SHIFTONEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
#else
    char sw;
    sw = PIND;

    // switch shift off
    if (ShiftEnable && (sw & (1<<SHIFTKEY) )) 
        inputStateShift = SHIFTOFFEVENT;

    // switch shift on
    if (!ShiftEnable && !(sw & (1<<SHIFTKEY) )) 
        inputStateShift = SHIFTONEVENT;

#endif
}

// }}}
// {{{ void getInputSMeter(void)

void getInputSMeter(void)
{
#ifdef TESTING
    LoopCounter++;
    if ((LoopCounter%2000) == 0)
    {
        if (inputStateSMeter>26) goingUp = FALSE;
        if (inputStateSMeter<1 ) goingUp = TRUE;

        if (goingUp)
            inputStateSMeter++;
        else
            inputStateSMeter--;
    }
#else
    register short s;

    ADCSRA |= (1<<ADSC)|(1<<ADEN); 
    while ((ADCSRA & (1<<ADSC))!=0);

    // calculate Smeter
    s = (1024-ADC)-44;

    // low pass s-meter signal
    s += LowPass;
    s >>= 1;
    LowPass= s;

    inputStateSMeter = s;
#endif
}

// }}}
// {{{ void getInputLargeStep(void)
#ifdef TESTING

void getInputLargeStep(void)
{
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 'l'  :
                inputStateLargeStep= LARGEEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
}

#endif
// }}}
// }}}

// }}}

// {{{ void showItem(void)

void showItem(void)
{
    switch (menuLoopState & TYPEMASK)
    {
        case MAINMENU :
            showMenuItem();
            break;
        case MAINMENU_VAL:
            showMenuItemValue();
            break;
        case SUBMENU1 :
            showSubmenu1Item();
            break;
        case SUBMENU1_VAL:
            showSubmenu1ItemValue();
            break;
    }
}

// }}}
// {{{ short getMaxMenuIndex(void)

short getMaxMenuIndex(void)
{
    short max;
    switch (menuLoopState & TYPEMASK)
    {
        case MAINMENU : 
            max = MAINMENU_END;
            break;
        case MAINMENU_VAL : 
            max = MAINMENU_VAL_END;
            break;
        case SUBMENU1 : 
            max = SUBMENU1_END;
            break;
        case SUBMENU1_VAL : 
            max = SUBMENU1_VAL_END;
            break;

            // this value should never occur
        default :
            max = 666;
    }
    return max;
}

// }}}
// {{{ void nextMenuItem(void)

void nextMenuItem(void)
{
    short max = getMaxMenuIndex();  

    menuLoopState++;
    if ((menuLoopState & STATEMASK) > (max & STATEMASK))
        menuLoopState = max & TYPEMASK;
    inputStateRotary = IDLE;
}

// }}}
// {{{ void prevMenuItem(void)

void prevMenuItem(void)
{
    short max = getMaxMenuIndex();

    menuLoopState--;
    // STATEMASK is -1 in 6bit signed mode 
    if ((menuLoopState & STATEMASK) == STATEMASK)
        menuLoopState = max;
    inputStateRotary = IDLE;
}

// }}}
// {{{ void mainLoopObsolete(void)

void mainLoopObsolete(void)
{
    char busy = TRUE;
    while (busy)
    {
        // {{{ handle inputs

        // input stuff
        SS_RotaryCount  = InputGetRotary();
        SS_             = InputGetSelector();
        SS_PTT          = InputGetPTT();
        SS_ShiftEnable  = InputGetShift();
        SS_SMeter       = InputGetSMeter();

        // }}}
        // {{{ testing
#ifdef TESTING
        getInputLargeStep();

        char c;
        if ((c = FHEgetc()))
            busy = !((c == 'q') || (c == 27)); // not 'q' and not escape

        // swallow unused input characters
        (void)FHEgetc();
#endif
        // }}}
        // {{{ output and processing state machine

        // DEBUG $$$ FHE start of output state machine
        sbi(PORTB,PB3);

        // the state machine does all the processing
        switch (menuLoopState)
        {
            // {{{ case TUNING : and input processing

            case TUNING : // 0

                // {{{ inputStateRotary

                // suppress frequency change during transmit
                if (Transmitting) inputStateRotary = IDLE;


                switch (inputStateRotary)
                {
                    case UPEVENT :
                        DialFrequency += (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (DialFrequency > BANDTOP) DialFrequency=BANDBOTTOM;
                        setFrequency();
                        break;

                    case DOWNEVENT : 
                        DialFrequency -= (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (DialFrequency < BANDBOTTOM) DialFrequency=BANDTOP;
                        setFrequency();
                        break;
                }
#ifdef DBG_LOGGING
                fprintf(dbg, "fr-%ld \n",DialFrequency);
#endif

                inputStateRotary = IDLE;

                // }}}
                // {{{ inputStatePTT

                switch (inputStatePTT)
                {
                    case TXEVENT :
                        Transmitting = TRUE;

                        // stop scanning when PTT activated
                        Scanning = FALSE; 

                        // don't display the S-meter during transmit
                        inputStateSMeter = 0;
                        clearSMeter();

                        setTransmitter();
                        break;

                    case RXEVENT :
                        Transmitting = FALSE;
                        setTransmitter();
                        break;
                }
                inputStatePTT= IDLE;

                // }}}
                // {{{ inputStateShift

                switch (inputStateShift)
                {
                    case SHIFTONEVENT :
                        ShiftEnable = TRUE;
                        setFrequency();
                        inputStateShift= IDLE;
                        break;

                    case SHIFTOFFEVENT :
                        ShiftEnable = FALSE;
                        setFrequency();
                        inputStateShift= IDLE;
                        break;
                }

                // }}}
                // {{{ inputStateLargeStep

                switch (inputStateLargeStep)
                {
                    case LARGEEVENT :
                        LargeStepEnable = !LargeStepEnable;
                        inputStateLargeStep = IDLE;
                        break;
                }

                // }}}
                // {{{ inputStateSelector

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = MAINMENU;
                        // stop scanning when entering the menu
                        Scanning = FALSE;
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }
                break;

                // }}}

                // }}}
                // {{{ main menu item selection
                // {{{  case MMUTELEVEL :

            case MMUTELEVEL : 
            case MSHIFT     : 
            case MCTCSS     : 
            case MSSTART    : 
            case MSEND      : 
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        break;
                }

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch from item selection to value selection
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU_VAL;
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }
                switch (menuLoopState & TYPEMASK)
                {
                    case MAINMENU :
                        showMenuItem();
                        break;
                    case MAINMENU_VAL:
                        showMenuItemValue();
                        break;
                }
                break;

                // }}}
                // {{{  case MSCAN:

            case MSCAN : // 5
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        break;
                }
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = TUNING;
                        Scanning = !Scanning;
                        inputStateSelector = SELECTBOUNCING;
                        showFrequency(0, CurrentFrequency);
                        break;
                }
                showMenuItem();
                break;

                // }}}
                // {{{  case MSETTINGS :

            case MSETTINGS : // 6
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        showMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        showMenuItem();
                        break;
                }
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = MARETURNMODE;
                        inputStateSelector = SELECTBOUNCING;
                        showFrequency(0, CurrentFrequency);
                        break;
                }
                break;

                // }}}
                // {{{  case MBACK :

            case MBACK : // 7
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        showMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        showMenuItem();
                        break;
                }
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = TUNING;
                        inputStateSelector = SELECTBOUNCING;
                        showFrequency(0, CurrentFrequency);
                        break;
                }
                break;

                // }}}
                // }}}
                // {{{ main menu value selection
                // {{{  case MMUTELEVELVAR :

            case MMUTELEVELVAR : 
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        MuteLevel = (MuteLevel<MAXMUTELEVEL) ? MuteLevel+1 : MuteLevel;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = MuteLevel;
                        break;

                    case DOWNEVENT : 
                        MuteLevel = (MuteLevel>0) ? MuteLevel-1 : 0;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = MuteLevel;
                        break;
                }
                // }}}
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU;
                        inputStateSelector = SELECTBOUNCING;
                        menuLoopState = (DirectMenuReturn) ? TUNING : menuLoopState;
                        break;
                }
                // }}}
                showItem();
                // }}}
                // {{{  case MSHIFTVAR :

            case MSHIFTVAR : 
                // {{{ up/down

                // suppress frequency change during transmit
                if (Transmitting) inputStateRotary = IDLE;

                switch (inputStateRotary)
                {
                    case UPEVENT :
                        FrequencyShift = (FrequencyShift < MAXSHIFT) ? FrequencyShift+1 : MAXSHIFT;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = FrequencyShift;
                        break;

                    case DOWNEVENT : 
                        FrequencyShift = (FrequencyShift > MINSHIFT) ? FrequencyShift-1 : MINSHIFT;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = FrequencyShift;
                        break;
                }
                // }}}
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU;
                        menuValues[menuLoopState & STATEMASK] = FrequencyShift;
                        inputStateSelector = SELECTBOUNCING;
                        menuLoopState = (DirectMenuReturn) ? TUNING : menuLoopState;
                        break;
                }
                //menuValues[menuLoopState & STATEMASK] = FrequencyShift;
                // }}}
                showItem();
                break;

                // }}}
                // {{{  case MCTCSSVAR :

            case MCTCSSVAR : // 202
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        inputStateRotary = IDLE;
                        ctcssIndex++;
                        if (CtcssTones[ctcssIndex] == -1) 
                            ctcssIndex--;
                        CTCSSfrequency = CtcssTones[ctcssIndex]; 
                        menuValues[menuLoopState & STATEMASK] = CTCSSfrequency;
                        break;

                    case DOWNEVENT : 
                        inputStateRotary = IDLE;
                        ctcssIndex--;
                        if (ctcssIndex == -1)
                            ctcssIndex=0;
                        CTCSSfrequency = CtcssTones[ctcssIndex]; 
                        menuValues[menuLoopState & STATEMASK] = CTCSSfrequency;
                        break;
                }
                // }}}
                CTCSSenable    = (CTCSSfrequency != 0);
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU;
                        menuValues[menuLoopState & STATEMASK] = CTCSSfrequency;
                        inputStateSelector = SELECTBOUNCING;
                        menuLoopState = (DirectMenuReturn) ? TUNING : menuLoopState;
                        break;
                }
                // }}}
                showItem();
                setCTCSSfreq();
                break;

                // }}}
                // {{{  case MSSTARTVAR

            case MSSTARTVAR : 
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        ScanStartFrequency += (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanStartFrequency > ScanEndFrequency) ScanStartFrequency = ScanEndFrequency;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = ScanStartFrequency;
                        break;

                    case DOWNEVENT : 
                        ScanStartFrequency -= (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanStartFrequency < BANDBOTTOM) ScanStartFrequency = BANDBOTTOM;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = ScanStartFrequency;
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch back to item selection
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU;

                        menuValues[menuLoopState & STATEMASK] = ScanStartFrequency;
                        inputStateSelector = SELECTBOUNCING;
                        menuLoopState = (DirectMenuReturn) ? TUNING : menuLoopState;
                        break;
                }

                // }}}
                showItem();
                break;

                // }}}
                // {{{  case MSENDVAR

            case MSENDVAR : 
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        ScanEndFrequency += (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanEndFrequency > BANDTOP) ScanEndFrequency = BANDTOP;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = ScanEndFrequency;
                        break;

                    case DOWNEVENT : 
                        ScanEndFrequency -= (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanEndFrequency < ScanStartFrequency) ScanEndFrequency = ScanStartFrequency;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState & STATEMASK] = ScanEndFrequency;
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch back to item selection
                        menuLoopState = (menuLoopState & STATEMASK) + MAINMENU;

                        menuValues[menuLoopState & STATEMASK] = ScanEndFrequency;
                        inputStateSelector = SELECTBOUNCING;
                        menuLoopState = (DirectMenuReturn) ? TUNING : menuLoopState;
                        break;
                }

                // }}}
                showItem();
                break;

                // }}}
                // }}}
                // {{{ submenu item selection
                // {{{  case MARETURNMODE:

            case MARETURNMODE:      // 400
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        break;
                }

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch from item selection to value selection
                        menuLoopState = (menuLoopState & STATEMASK) + SUBMENU1_VAL;
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }
                showItem();
                break;

                // }}}
                // {{{  case MAREFFREQ

            case MAREFFREQ :
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        nextMenuItem();
                        break;

                    case DOWNEVENT : 
                        prevMenuItem();
                        break;
                }

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch from item selection to value selection
                        menuLoopState = (menuLoopState & STATEMASK) + SUBMENU1_VAL;
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }
                showItem();
                break;

                // }}}
                // }}}
                // {{{ submenu value selection
                // {{{  case MARETURNMODEVAR

            case MARETURNMODEVAR :
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                    case DOWNEVENT : 
                        DirectMenuReturn = !DirectMenuReturn;
                        inputStateRotary = IDLE;
                        subMenuValues1[menuLoopState & STATEMASK] = DirectMenuReturn;
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // immediate switch back to mainmenu selection
                        menuLoopState = MSETTINGS;
                        subMenuValues1[menuLoopState & STATEMASK] = DirectMenuReturn;
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }

                // }}}
                showItem();
                break;

                // }}}
                // {{{  case MAREFFREQVAR

            case MAREFFREQVAR :
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        inputStateRotary = IDLE;
                        refFreqIndex++;
                        if (refFreqPLL[refFreqIndex] == -1) 
                            refFreqIndex--;
                        subMenuValues1[menuLoopState & STATEMASK] = refFreqPLL[refFreqIndex];
                        break;

                    case DOWNEVENT : 
                        inputStateRotary = IDLE;
                        refFreqIndex--;
                        if (refFreqIndex == -1) 
                            refFreqIndex=0;
                        subMenuValues1[menuLoopState & STATEMASK] = refFreqPLL[refFreqIndex];
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // immediate switch back to mainmenu selection
                        menuLoopState = MSETTINGS;
                        subMenuValues1[menuLoopState & STATEMASK] = refFreqPLL[refFreqIndex];
                        inputStateSelector = SELECTBOUNCING;
                        break;
                }

                // }}}
                showItem();
                break;

                // }}}
                // }}}
        }
        // {{{ scanner
#ifdef TESTING
        if (Scanning)
        {   
            short StepDelay=125;
            clock_t now = clock();
            currentTime = (float)now * 1000.0F / CLOCKS_PER_SEC;
            goStep = (currentTime - prevStepTime) > StepDelay;
            if (goStep)
            {   
                prevStepTime = currentTime;
                DialFrequency =
                    (DialFrequency > ScanEndFrequency) ? ScanStartFrequency : DialFrequency+CHANNELSTEP;
                setFrequency();
            }
        }   
#else
        // $$$ FHE TODO
#endif
        // }}}
        // {{{ squelch

        if ((!Transmitting) && (menuLoopState == TUNING)) 
            showSMeter();
        if (menuLoopState == TUNING)
            showTrx();

        Muted = (inputStateSMeter < MuteLevel);
        setMuted();

        // }}}

        updateDisplay();

        // }}}


        // DEBUG $$$ FHE end of output state machine
        cbi(PORTB,PB3);
        // pulse width is output processing time
        // frequency is system main-loop time 

        // {{{ testing and debugging
#ifdef TESTING

        if (TRUE)
        {
            ttySetCursorPosition(DBGROW,1);
            printf("\033[37m"); // light gray
            printf("========================= debug ========================\r\n");
            printf("CTCSSenable     =      %3s  | ",yesno(CTCSSenable));
            printf("inputStateRotary= %8d\r\n",inputStateRotary);

            printf("CTCSSfrequency  = %8d  | ",CTCSSfrequency);
            printf("menuLoopState   =     %04X\r\n",menuLoopState);

            printf("MuteLevel       = %8d  | ",MuteLevel);
            printf("menuLoopState S = %8d\r\n",menuLoopState & STATEMASK);

            printf("ShiftEnable     =      %3s  | ",yesno(ShiftEnable));
            printf("menuLoopState T =     %04X\r\n",menuLoopState & TYPEMASK);

            printf("LargeEnable     =      %3s  | ",yesno(LargeStepEnable));
            printf("\r\n");

            printf("FrequencyShift  = %8d  |",FrequencyShift);
            printf("\r\n");

            printf("Transmitting    =      %3s  | ",yesno(Transmitting));
            printf("\r\n");

            printf("Scanning        =      %3s  | ",yesno(Scanning));
            printf("\r\n");

            printf("Scan Start      = %4lu.%03lu  | ", ScanStartFrequency/1000, ScanStartFrequency%1000);
            printf("\r\n");

            printf("Scan End        = %4lu.%03lu  | ", ScanEndFrequency/1000, ScanEndFrequency%1000);
            printf("\r\n");

            // printf("currentTime     = %8d\r\n",currentTime);
            // printf("prevStepTime    = %8d\r\n",prevStepTime);
            // printf("ctcssIndex      = %8d\r\n",ctcssIndex);
            // printf("vInRotState     = %8d\r\n",vInRotState);
            // printf("unget charbuf   = %8X\r\n",GetcBuffer);
            // printf("unget charvail  = %8s\r\n",yesno(GetcAvail));
            printf("========================================================\r\n");
            // printf("|%-*s|\r\n", DISPLAY_WIDTH, DisplayBuffer[0]);
            // printf("|%-*s|\r\n", DISPLAY_WIDTH, DisplayBuffer[1]);
            // printf("=====================\r\n");
            printf("\033[30m"); // black
        }
        else
        {
            deSetCursorPosition(DBGROW,1);
            printf("2");
        }
#endif
        // }}}
    }
}

// }}}


// {{{ void mainLoop(void)

void mainLoop(void)
{
    // input handling
    SS_Rotary      = InputGetRotaryDialCount();
    SS_Selected    = InputGetSelectorPushed();
    SS_ShiftEnable = InputGetShiftEnable();
    SS_PTT         = InputGetPTT();
    SS_SMeterIn    = InputGetSMeter();


    // output handling
    OutputSetCTCSSfreq();
    OutputSetAudioMute();
    OutputSetDisplaySMeter();
    OutputSetDisplayMuteIndicator();
    OutputSetDisplayTxRxIndicator();
    OutputSetDisplayFrequency();

    OutputSetPLLDivider();
    OutputSetTransmittorOn();

}

// }}}
// {{{ int main(int argc, char *argv[])

int main(int argc, char *argv[])
{
    // {{{ testing

#ifdef TESTING
    ttyClearScreen();
    ttySetCursorPosition(0,0);
    printf("========= 23cm NBFM control software simulator =========\n");
    printf("\n");
    printf("q quit\n"); 
    printf("[ upward rotating           ] downward rotating\n"); 
    printf("t transmit                  r receive\n"); 
    printf("s shift on                  a shift off\n");
    printf("e menu selector button      l toggle large steps on/off\n"); 
    printf("\n");
    set_conio_terminal_mode();
#ifdef DBG_LOGGING
    dbg = fopen("log.txt","w");
#endif
#endif

    // }}}

    initialize();
    mainLoop();

    // {{{ testing

#ifdef TESTING
#ifdef DBG_LOGGING
    fclose(dbg);
#endif
    // cursor back on
    printf("\033[?25h");   
    // short i;
    // for (i=0x20; i<255; i++) printf("%X-%c ", i,i);
#endif

    // }}}
}

// }}}

// {{{ 



// }}}
// EOF
