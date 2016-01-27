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
// {{{ includes

#include <stdio.h>

#ifdef TESTING
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
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

#define CLKMASK             1
#define DATAMASK            2

// {{{ input events
#define IDLE                -10
#define UPEVENT             1
#define DOWNEVENT           2
#define TXEVENT             3
#define RXEVENT             4
#define SELECTEVENT         5
#define SHIFTEVENT          6
#define LARGEEVENT          7
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
#define PB0 0
#define PB1 1
#define PB2 2
#define PB3 3
#define PB4 4
#define PB5 5
#define PB6 6
#define PB7 7

#define PC0 0
#define PC1 1
#define PC2 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PC6 6
#define PC7 7

int tx;
int freq;
int shiftSwitch;
int shift;
char str[50];
int ports[10];
#define PORTB ports[1]
#define PORTC ports[2]

#endif

#define F_CPU       1000000UL

typedef unsigned char u08;

#define sbi(x,y) x |= _BV(y) //set bit - using bitwise OR operator 
#define cbi(x,y) x &= ~(_BV(y)) //clear bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y) //toggle bit - using bitwise XOR operator
#define is_high(x,y) (x & _BV(y) == _BV(y)) //check if the y'th bit of register 'x' is high ... test if its AND with 1 is 1

#define SHORT       1
#define LONG        2

// various
#define Smeter      PC5
#define MUTE        PC4
#define TXON        PC3

// ADF4113
#define LE          PC2
#define DATA        PC1
#define CLK         PC0


// CTCSS & tone
#define Beep        PB3

// rotary & switches
#define PTT         PD0
#define SHIFTKEY    PD1
#define ROTKEY      PD2
#define ROTINT      PD3
#define ROT         PD4

// LCD
#define LCD_D7      PB7
#define LCD_D6      PB6
#define LCD_D5      PB5
#define LCD_D4      PB4
#define LCD_RS      PB1
#define LCD_E       PB0

#define dispCLEAR   0x01
#define dispHOME    0x02
#define dispMODE    0x06 // left to right + shift cursor
#define dispONOF    0x0C // dispi-on + no cursor + no blink
#define dispSHIF    0x18 // 2 Line+dir + n.c. + n.c.
#define dispFUNC    0x20 // 4bit  + 2line + size + n.c. + n.c.
#define dispCGRA    0x40 // <5 bit adr> // for custom chars
#define dispDDRA    0x80 // <6 bit adr>

// read status : RS=0, RW=0 bit 7 is busyflag
// write data  : RS=1, RW=1 <8 bit data>
// read  data  : RS=1, RW=0 <8 bit data>
// }}}

unsigned char _BV(unsigned char c) { return c; }
// {{{ All of these should be removed

void _delay_us(int s) {} 
void _delay_ms(int s) {} 

// }}}
// {{{ Globals

void showTrx(void);
void setFreq(long int f);
void initPLL(void);
void initLCD(void);
void initADC(void);
void deInit(void);
void lcdCmd(char c);
void lcdData(char c);
void setPLL(long int c);
void setMuted(void);
void showFrequency(int r, int f);

#ifdef TESTING
int goingUp;                // tmp global
int LoopCounter;            // tmp global
#endif
int currentTime;            // tmp global
int prevStepTime;
int goStep;

int Transmitting;           // boolean: TX = true, RX = false;
int Scanning;               // boolean: scanning = true;
int Muted;                  // boolean: TRUE=audio muted, FALSE=audio on 
int DialFrequency;          // frequency as set by dial
int CurrentFrequency;       // frequency setting sent to synthesizer
int ShiftEnable;            // boolean: shifted = true;
int ReverseShift;           // boolean: swap transmit and receive frequencies
int LargeStepEnable;        // boolean: channel steps are 10x as big
int FrequencyShift;         // size of frequency shift
int MuteLevel;              // level below which audio muting is enabled
int CTCSSenable;            // boolean: CTCSS tone enabled during transmit
int CTCSSfrequency;         // frequency of CTCSS tone to use
int ScanStartFrequency;     // duh...
int ScanEndFrequency;       // duh...
long PllReferenceFrequency; // duh...
int  DirectMenuReturn;       // When true: exit the menu upon a value selection.
char Line[DISPLAY_WIDTH+10];
char DisplayBuffer[LINECOUNT][DISPLAY_WIDTH+10];

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

int  menuValues [] =  {  
    INITIAL_MUTELEVEL,  // 0
    INITIAL_SHIFT,      // 1
    INITIAL_CTCSS,      // 2
    1280000,            // 3
    1290000,            // 4
    0,                  // 5
    0,                  // 6
    0};                 // 7

#define SUBMENULENGTH 2
char *subMenuStrings1[] = {
    //234567890123456
    "Select action:",   // 0
    "PLL Ref:"          // 1
};

int subMenuValues1 [] = {
    TRUE,               // 0
    INITIAL_REFERENCE   // 1
};

int menuLoopState;

// define S-meter chars
unsigned char smeter[3][8] = {
    {0b00000,0b00000,0b10000,0b10000,0b10000,0b10000,0b00000,0b00000},
    {0b00000,0b00000,0b10100,0b10100,0b10100,0b10100,0b00000,0b00000},
    {0b00000,0b00000,0b10101,0b10101,0b10101,0b10101,0b00000,0b00000}
};

// rotary
int GClkPrev;
int GQdPrev;
int input;
// /rotary

//CTCSS frequencies
int CtcssTones[] = {   0, 670, 689, 693, 710, 719, 744, 770, 797, 825, 854, 885, 915, 948, 974,
    1000,1035,1072,1109,1148,1188,1230,1273,1318,1365,1413,1462,1514,1567,1598,
    1622,1655,1679,1713,1738,1773,1799,1835,1862,1899,1928,1966,1995,2035,2065,
    2107,2181,2257,2291,2336,2418,2503,2541, -1};
int ctcssIndex;

int refFreqPLL[] = { 13000, 12000, 10700, -1 };
int refFreqIndex;

#ifdef TESTING
int theMainEvent;
char GetcBuffer;
int  GetcAvail;
struct termios orig_termios;
//FILE *dbg;
#endif

// input states controls
#ifdef TESTING
int vInRotState;
int vInputStateRotary;
int S;
#endif
int inputStateRotary;
int inputStateSelector;
int inputStateShift;
int inputStatePTT;
int inputStateSMeter;
int inputStateLargeStep;

// }}}
// {{{ Initialisation

// {{{ void initLCD(void)
void initLCD(void)
{
#ifdef TESTING
    deInit();
    showTrx();
#endif
    int i,j;

    // 4 bits mode, 2 lines
    lcdCmd(0x20);
    _delay_ms(10);

    lcdCmd(0x20);
    _delay_ms(10);

    // cursor shifts to right, text no shift
    lcdCmd(0x18);
    _delay_ms(20);

    // display on, no cursor, no blink
    lcdCmd(0x0c);
    _delay_ms(20);

    // shift mode
    lcdCmd(0x06);
    _delay_ms(20);

    // home
    lcdCmd(0x02);
    _delay_ms(20);

    // clear display
    lcdCmd(0x01);
    _delay_ms(20);

    // define custom chars
    lcdCmd(0x40);
    for (i=0; i<3; i++) {
        for (j=0; j<8; j++) {
            lcdData(smeter[i][j]);
        }
    }
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
    long int reg;
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
// {{{ void initIRQ(void)

void initIRQ(void)
{
#ifdef TESTING
#else
    EIMSK |= _BV(INT1);
    EICRA |= _BV(ISC11);

    // Setup Timer 1
    TCCR1A = 0x00;				// Normal Mode 
    TCCR1B = 0x01;				// div/1 clock, 1/F_CPU clock
    toneValue = 5*F_CPU/tone;	// *10/2

    // Enable interrupts as needed 
    TIMSK1 |= _BV(TOIE1);  	// Timer 1 overflow interrupt 

    // enable interrupts
    sei();
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
    initIRQ();

#ifdef TESTING
    vInRotState = 3;
    GetcAvail = FALSE;
    GetcBuffer = 0;
#endif
}

// }}}

// }}}
// {{{ Scaffolding

void setCursorPosition(int x, int y);

// {{{ Display emulator

// {{{ deClearScreen

void deClearScreen(void)
{
    printf("\033[2J");
    printf("\033[?1h");
}

// }}}
// {{{ deSetCursorPosition

void deSetCursorPosition(int row, int col)
{
    printf("\033[%d;%df",row,col);
}

// }}}
// {{{ deTop

void deTop(void)
{
    deSetCursorPosition(1,1);
}

// }}}
// {{{ deCmd

void deCmd(char c)
{

}

// }}}
// {{{ deData

void deData(char c)
{
}

// }}}
// {{{ deInit()

void deLineH(int w)
{
    int i;
    printf("+");
    for (i=1; i<w; i++)
        printf("-");
    printf("+");
}

void deInit(void)
{
    int y;

    // cursor off
    printf("\033[?25l");   

    deSetCursorPosition(YTop-1, XTop-1);
    deLineH(DE_WIDTH+1);
    deSetCursorPosition(YTop+DE_HEIGHT, XTop-1);
    deLineH(DE_WIDTH+1);
    for (y=0; y<DE_HEIGHT; y++)
    {
        deSetCursorPosition(YTop+y, XTop-1);
        printf("|");
        deSetCursorPosition(YTop+y, XTop+DE_WIDTH);
        printf("|");
    }
}

// }}}
// }}}

// {{{ void clearScreen(void)

void clearScreen(void)
{
    deClearScreen();
}

// }}}
// {{{ void setCursorPosition(int row, int col)

void setCursorPosition(int row, int col)
{
    deSetCursorPosition(row+YTop, col+XTop);
}

// }}}
// {{{ void top(void)

void top(void)
{
    deTop();
}

// }}}

// {{{ char* yesno(int boolean)

char* yesno(int boolean)
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

int FHEkbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int FHEgetchar()
{
    int r;
    unsigned char c;
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

// {{{ input functions

#ifdef TESTING
// {{{ int bounce()
int bounce(int clock, int data, int bounceSelect, int newState)
{
    int rv;

    static int bounceCount;

    // bounce during the first 4 cycles
    // keep stable in the last 4 cycles

    if (bounceSelect == 1)
    {
        // clock bounces
        rv =  (bounceCount & 0x01) ?  clock | data : (clock ^ CLKMASK) | (data);
        //                            unchanged    one's complenment   unchanged
    } else
    {
        // data bounces
        rv =  (bounceCount & 0x01) ?  clock | data : (clock) | (data ^ DATAMASK);
        //                            unchanged     unchanged   one's  complenment
    }

    if (bounceCount > 13)
        rv = (clock | data);

    rv = rv & 3; 
    S = rv;

    // rotate through 8 cycles
    bounceCount++;
    if (bounceCount > 31) 
    {
        bounceCount=0;
        vInRotState = newState;
        // after 15 cycles goto next state
    }
    //    if (bounceCount == 0) fprintf(dbg,"\n");
    //    fprintf(dbg,"%02X ",rv);    

    return rv;
}
// }}}
#endif
// {{{ void getInputRotary(void)


void getInputRotary(void)
{
    int clk;
    int data;
    int sample;
    int qd;
    int UpDown;

#ifdef TESTING
    // =========================

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

    switch (vInRotState)
    {
        // up rotate
        case 0 :
            input = bounce(CLKMASK, 0,        2, 1);
            break;

        case 1 :
            input = bounce(0,       0,        1, 2);
            break;

        case 2 :
            input = bounce(0,       DATAMASK, 2, 3);
            break;

        case 3 :
            input = bounce(CLKMASK, DATAMASK, 1, 9);
            break;

            // down rotate
        case 4 :
            input = bounce(CLKMASK, DATAMASK, 2, 5) ;
            break;

        case 5 :
            input = bounce(0,       DATAMASK, 1, 6);
            break;

        case 6 :
            input = bounce(0,       0,        2, 7);
            break;

        case 7 :
            input = bounce(CLKMASK, 0,        1, 9);
            break;

        case 9 :
            vInRotState = 9;
            break;
    }

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

    sample = input;
    clk = (sample & CLKMASK) == 0;
    data = (sample & DATAMASK) == 0;

    /* double edge triggered flipflop creates   */
    /* clean clock pulses (qd)                  */

    if (clk != GClkPrev)
    {
        qd = data;
    }

    /* positive edge triggered flipflop creates */
    /* directional value  (UpDown)              */

    if (clk & !GClkPrev)
    {
        UpDown = data;
    }

    /* qd clock pulse changes frequency         */
    /* on rising flank                          */
    if (qd & !GQdPrev)
    {
        if (UpDown)
            inputStateRotary = UPEVENT;
        else
            inputStateRotary = DOWNEVENT;
    }

    GClkPrev = clk;
    GQdPrev  = qd;
}


// }}}
// {{{ void getInputSelector(void)

void getInputSelector(void)
{
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
}

// }}}
// {{{ void getInputTxRx(void)

void getInputTxRx(void)
{
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
}

// }}}
// {{{ void getInputShift(void)

void getInputShift(void)
{
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 's'  :
                inputStateShift = SHIFTEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
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
    register int s;

    ADCSRA |= (1<<ADSC)|(1<<ADEN); 
    while ((ADCSRA & (1<<ADSC))!=0);

    // calculate Smeter
    s = (1024-ADC)-44;

    // low pass s-meter signal
    s += lps;
    s >>= 1;
    lps = s;

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
// {{{ output functions
// {{{ Display control routines

// {{{ void lcdNib(char c)

void lcdNib(char c)
{
    PORTB = c;
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
// {{{ void lcdData(char c)

void lcdData(char c)
{
#ifdef TESTING
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
    while (*s) 
        lcdData(*s++);
}

// }}}

// {{{ void hex2bcd(long int f)
void hex2bcd(long int f)
{
    long int x = 10000000;
    int i;
    char d;

    for (i=0; i<8; i++) {
        d = 0;
        while (TRUE) 
        {
            f -= x;
            if (f < 0) break;
            d++;
        }
        str[i] = d + 0x30;
        f += x;
        x = x/10;
    }
}


// }}}
// {{{ void display(long int f)
void display(long int f)
{
    int i;
    char c;

    hex2bcd(f);

    lcdCmd(0x80 + 4);
    _delay_us(200);

    for (i=1; i<8; i++) {
        c = str[i];
        if (i==5) {
            lcdData('.');
        }
        lcdData(c);
    }
}


// }}}
// {{{ void update()
void update()
{
    long int f;

    if (tx) {
        f = freq;
        if (shiftSwitch)
            f += shift;
        setFreq(f);
        display(f);
        lcdCmd(0xcf);
        lcdData('T');
    }
    else {
        setFreq(freq - IF);
        display(freq);
        lcdCmd(0xcf);
        lcdData('R');
    }
}


// }}}

// }}}

// {{{ void updateDisplay(void)

void updateDisplay(void)
{
    printf("\033[34m");
    setCursorPosition(0,0);
    printf("%-16s", DisplayBuffer[0]);
    setCursorPosition(1,0);
    printf("%-16s", DisplayBuffer[1]);
    printf("\033[30m");
}

// }}}

// {{{ void outputTrxBit(void) 

void outputTrxBit(void)
{
    if (Transmitting)
    {
        // set transmit bit
        sbi(PORTC, TXON);
    } else {
        // clear transmit bit
        cbi(PORTC, TXON);
    }
}

// }}}

// {{{ void setDisplay(int row, char *string)

void setDisplay(int row, char *string)
{
    strcpy(DisplayBuffer[row], string);
}

// }}}
// {{{ void setFrequency(void)

void setFrequency(void)
{
    CurrentFrequency = DialFrequency 
        + (( Transmitting && !ReverseShift && ShiftEnable) ? FrequencyShift*1000 : 0)
        + ((!Transmitting &&  ReverseShift && ShiftEnable) ? FrequencyShift*1000 : 0);
    showFrequency(0, CurrentFrequency);
}

// }}}
// {{{ void setTransmitter(void)

void setTransmitter(void)
{
    setFrequency();
    setMuted();
    outputTrxBit();
    showTrx();
}

// }}}
// {{{ void setCTCSSfreq(void)

void  setCTCSSfreq(void)
{
    //  showCTCSSfreq(1);
}

// }}}
// {{{ void setMuted(void)

void setMuted()
{
    // Mute when signal to low or when transmitting
    if (Muted || Transmitting)
    {
        // set output mute bit
        sbi(PORTC, MUTE);
        // Only show the 'M' symbol in the S-Meter during regular receive mode
        if ((menuLoopState == TUNING) && (!Transmitting))
            DisplayBuffer[1][0] = 'M';
    }
    else
        // clear output mute bit
        cbi(PORTC, MUTE);
}

// }}}

// {{{ void showFrequency(int row, int freq)

void showFrequency(int row, int fr)
{
    sprintf(Line, "VFO %4u.%03u MHz", fr/1000, fr%1000);
    setDisplay(row,Line);
}

// }}}
// {{{ void showCTCSSfreq(char *prompt)

void showCTCSSfreq(char *prompt)
{
    if (CTCSSfrequency == 0)
        sprintf(Line, "%s%-*s", prompt, DISPLAY_WIDTH-2, "off");
    else
        sprintf(Line, "%s%3d.%d Hz%*s", prompt, CTCSSfrequency/10,CTCSSfrequency%10,DISPLAY_WIDTH-10," ");
}

// }}}
// {{{ void showTrx(void)

void showTrx(void)
{
    DisplayBuffer[1][DISPLAY_WIDTH-1] = (Transmitting) ? 'T' : 'R';
}

// }}}
// {{{ void showSMeter(void)

void showSMeter(void)
{
#ifdef TESTING
    int i;
    for (i=0; i<inputStateSMeter/2; i++)
        DisplayBuffer[1][i]='"';
    DisplayBuffer[1][inputStateSMeter/2] = ((inputStateSMeter%2)==0) ? '\'' : '"';
    DisplayBuffer[1][(inputStateSMeter/2)+1] = ' ';
#else
    short n = 15;

    lcdCmd(0xc0);

    // chars in the full bar are 3 lines
    level >>= 1;
    while (level >= 3) 
    {
        lcdData(2);
        level -= 3;
        n--;
    }

    // last char 0, 1 or 2 lines
    switch (level) 
    {
        case 2: 
            lcdData(1); 
            break;

        case 1: 
            lcdData(0); 
            break;

        default: 
            lcdData(' ');
    }

    // clear any chars to the right
    while (--n > 0) 
        lcdData(' ');
#endif
}

// }}}
// {{{ void clearSMeter(void)

void clearSMeter(void)
{
    sprintf(DisplayBuffer[1],"%*s",DISPLAY_WIDTH," ");
}

// }}}

// {{{ void bottomLinePrinter(int index, char *prompt)

void bottomLinePrinter(int index, char *prompt)
{
    int val;

    if (((index & TYPEMASK) == MAINMENU) || ((index & TYPEMASK) == MAINMENU_VAL))
        val = menuValues[index & STATEMASK];
    else
        val = subMenuValues1[index & STATEMASK];

    // bottom line
    switch (index)
    {
        case MSSTART :
        case MSSTARTVAR :
        case MSEND :
        case MSENDVAR :
            sprintf(Line, "%s%4u.%03u MHz", prompt, val/1000, val%1000);
            break;

        case MCTCSS :
        case MCTCSSVAR :
            showCTCSSfreq(prompt);
            //sprintf(Line, "%s%3d.%d Hz%*s", prompt, val/10, val%10, DISPLAY_WIDTH-10, " ");
            break;

        case MBACK :
        case MSCAN :
        case MSETTINGS :
            sprintf(Line, "%*s", DISPLAY_WIDTH, " ");
            break;

        case MARETURNMODE :
        case MARETURNMODEVAR :
            sprintf(Line, "%s%-*s", prompt, DISPLAY_WIDTH-2, (DirectMenuReturn) ? "to tuning" : "to menu");
            break;

        case MAREFFREQ :
        case MAREFFREQVAR :
            sprintf(Line, "%s%2d.%03d MHz%*s", prompt, val/1000, val%1000, DISPLAY_WIDTH-14, " ");
            break;

        default : 
            sprintf(Line, "%s%-*d", prompt, DISPLAY_WIDTH-2, val);
    }
    setDisplay(1, Line);
}

// }}}
// {{{ void showMenuItem()

void showMenuItem()
{
    // top line
    switch (menuLoopState)
    {
        case MBACK :
        case MSCAN :
        case MSETTINGS :
            sprintf(Line,"> %-*s",DISPLAY_WIDTH-14, menuStrings[menuLoopState & STATEMASK]);
            break;

        default : 
            sprintf(Line,"> Set %-*s",DISPLAY_WIDTH-6, menuStrings[menuLoopState & STATEMASK]);
    }
    setDisplay(0, Line);

    // bottom line
    bottomLinePrinter(menuLoopState,"  "); 
}

// }}}
// {{{ void showMenuItemValue()

void showMenuItemValue()
{
    // top line
    sprintf(Line,"  Set %-*s", DISPLAY_WIDTH-6, menuStrings[menuLoopState & STATEMASK]);
    setDisplay(0, Line);

    // bottom line
    bottomLinePrinter(menuLoopState,"> "); 
}

// }}}
// {{{ void showSubmenu1Item(void)

void showSubmenu1Item(void)
{
    // top line
    switch (menuLoopState)
    {
        case MARETURNMODE :
        case MAREFFREQ    :
            sprintf(Line,"> %-*s",DISPLAY_WIDTH-2, subMenuStrings1[menuLoopState & STATEMASK]);
            break;
    }
    setDisplay(0, Line);

    // bottom line
    bottomLinePrinter(menuLoopState,"  "); 
}

// }}}
// {{{ void showSubmenuItemValue()

void showSubmenu1ItemValue()
{
    // top line
    sprintf(Line,"  %-*s", DISPLAY_WIDTH-2, subMenuStrings1[menuLoopState & STATEMASK]);
    setDisplay(0, Line);

    // bottom line
    bottomLinePrinter(menuLoopState,"> "); 
}

// }}}

// {{{ PLL

// {{{ void setPLL(long int r)

void setPLL(long int r)
{
    int i;

    for (i=0; i<24; i++) {
        if (r & 0x800000)
            sbi(PORTC, DATA);
        else
            cbi(PORTC, DATA);
        _delay_us(1);
        sbi(PORTC, CLK);
        _delay_us(1);
        cbi(PORTC, CLK);
        r <<= 1;
    }
    _delay_us(1);
    sbi(PORTC, LE);
    _delay_us(1);
    cbi(PORTC, LE);
}

// }}}
// {{{ void setFreq(long int f)

void setFreq(long int f)
{
    long int reg, frast, A, B;

    f = (Transmitting) ? f : f-IF;
    frast = f/CHANNELSTEP;
    B = frast/16;
    A = frast%16;

    reg = ((B & 0x1fff)<<8) + ((A & 0x3f)<<2) + 1;

    setPLL(reg);
}

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
// {{{ int getMaxMenuIndex(void)

int getMaxMenuIndex(void)
{
    int max;
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
    int max = getMaxMenuIndex();  

    menuLoopState++;
    if ((menuLoopState & STATEMASK) > (max & STATEMASK))
        menuLoopState = max & TYPEMASK;
    inputStateRotary = IDLE;
}

// }}}
// {{{ void prevMenuItem(void)

void prevMenuItem(void)
{
    int max = getMaxMenuIndex();

    menuLoopState--;
    // STATEMASK is -1 in 6bit signed mode 
    if ((menuLoopState & STATEMASK) == STATEMASK)
        menuLoopState = max;
    inputStateRotary = IDLE;
}

// }}}
// {{{ void mainLoop(void)

void mainLoop(void)
{
    int busy = TRUE;
    while (busy)
    {
        // {{{ handle inputs

        // input stuff
        getInputRotary();
        getInputSelector();
        getInputTxRx();
        getInputShift();
        getInputSMeter();

        // }}}
        // {{{ testing
#ifdef TESTING
        getInputLargeStep();

        char c;
        if ((c = FHEgetc()))
            busy = (c != 'q');

        // swallow unused input characters
        (void)FHEgetc();
#endif
        // }}}
        // {{{ output and processing state machine

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
                    case SHIFTEVENT :
                        ShiftEnable = !ShiftEnable;
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
                        inputStateSelector = IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector = IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        inputStateSelector= IDLE;
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
                        DirectMenuReturn = (DirectMenuReturn) ? FALSE : TRUE;
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
                        inputStateSelector= IDLE;
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

                        subMenuValues1[menuLoopState & STATEMASK] = DirectMenuReturn;
                        inputStateSelector= IDLE;
                        break;
                }

                // }}}
                showItem();
                break;

                // }}}
                // }}}
        }
        // {{{ scanner
        if (Scanning)
        {   
            int StepDelay=125;
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
        // }}}

        if ((!Transmitting) && (menuLoopState == TUNING)) 
            showSMeter();
        if (menuLoopState == TUNING)
            showTrx();

        Muted = (inputStateSMeter < MuteLevel);
        setMuted();
        updateDisplay();

        // }}}
        // {{{ testing and debugging
#ifdef TESTING

        if (TRUE)
        {
            deSetCursorPosition(DBGROW,1);
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
            printf("sample          = %8d\r\n",S);

            printf("FrequencyShift  = %8d  |",FrequencyShift);
            printf("\r\n");

            printf("Transmitting    =      %3s  | ",yesno(Transmitting));
            printf("\r\n");

            printf("Scanning        =      %3s  | ",yesno(Scanning));
            printf("\r\n");

            printf("Scan Start      = %4d.%03d  | ", ScanStartFrequency/1000, ScanStartFrequency%1000);
            printf("\r\n");

            printf("Scan End        = %4d.%03d  | ", ScanEndFrequency/1000, ScanEndFrequency%1000);
            printf("\r\n");

            // printf("currentTime     = %8d\r\n",currentTime);
            // printf("prevStepTime    = %8d\r\n",prevStepTime);
            // printf("ctcssIndex      = %8d\r\n",ctcssIndex);
            // printf("vInRotState     = %8d\r\n",vInRotState);
            // printf("unget charbuf   = %8X\r\n",GetcBuffer);
            // printf("unget charvail  = %8s\r\n",yesno(GetcAvail));
            printf("========================================================\r\n");
            printf("|%-16s|\r\n", DisplayBuffer[0]);
            printf("|%-16s|\r\n", DisplayBuffer[1]);
            printf("=====================\r\n");
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
// {{{ int main(int argc, char *argv[])

int main(int argc, char *argv[])
{
    // {{{ testing

#ifdef TESTING
    deClearScreen();
    deSetCursorPosition(1,1);
    printf("========= 23cm NBFM control software simulator =========\n");
    printf("\n");
    printf("q quit\n"); 
    printf("[ upward rotating           ] downward rotating\n"); 
    printf("t transmit                  r receive\n"); 
    printf("s toggle shift on/off       l toggle large steps on/off\n"); 
    printf("e menu selector button\n");
    printf("\n");
    set_conio_terminal_mode();
    // dbg = fopen("log.txt","w");
#endif

    // }}}

    initialize();
    setFrequency();
    mainLoop();

    // {{{ testing

#ifdef TESTING
    // fclose(dbg);
    // cursor back on
    printf("\033[?25h");   
    // int i;
    // for (i=0x20; i<255; i++) printf("%X-%c ", i,i);
#endif

    // }}}
}

// }}}

// {{{
// }}}
// EOF
