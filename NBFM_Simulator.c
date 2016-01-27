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
#define TESTING "this is the simulation. #UNDEF for the real thing" 
// {{{ includes

#include <stdio.h>
#include <time.h>

#ifdef TESTING
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
#define CHANNELSTEP         25          // in kHz
#define LARGESTEP           1000        // in kHz
#define BANDBOTTOM          1240000UL   // in kHz
#define BANDTOP             1300000UL   // in kHz

#define DISPLAY_WIDTH       16

#define CLKMASK             1
#define DATAMASK            2

#define IDLE                -10
#define UPEVENT             1
#define DOWNEVENT           2
#define TXEVENT             3
#define RXEVENT             4
#define SELECTEVENT         5
#define SHIFTEVENT          6
#define LARGEEVENT          7

#define MMUTELEVEL          0 
#define MSHIFT              1
#define MCTCSS              2
#define MSSTART             3
#define MSEND               4
#define MSCAN               5
#define MBACK               6
#define MOFF                100
#define MMUTELEVELVAR       (MMUTELEVEL+200)
#define MSHIFTVAR           (MSHIFT+MMUTELEVELVAR)
#define MCTCSSVAR           (MCTCSS+MMUTELEVELVAR)
#define MSSTARTVAR          (MSSTART+MMUTELEVELVAR)
#define MSENDVAR            (MSEND+MMUTELEVELVAR)

#define MAXMUTELEVEL        32
#define MINSHIFT            -60
#define MAXSHIFT            60
#define MINCTCSS            10
#define MAXCTCSS            90
/*
#define 
#define 
#define 
#define 
#define 
 */

#ifdef TESTING
#define DE_WIDTH    16
#define DE_HEIGHT    2
#define YTop        10
#define XTop        10
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
#define dispSHIF    0x18 // 2 line+dir + n.c. + n.c.
#define dispFUNC    0x20 // 4bit  + 2line + size + n.c. + n.c.
#define dispCGRA    0x40 // <5 bit adr> // for custom chars
#define dispDDRA    0x80 // <6 bit adr>

// read status : RS=0, RW=0 bit 7 is busyflag
// write data  : RS=1, RW=1 <8 bit data>
// read  data  : RS=1, RW=0 <8 bit data>
// }}}

// {{{ All of these should be removed

unsigned char _BV(unsigned char c) { return c; }
void _delay_us(int s) {} 
void _delay_ms(int s) {} 

// }}}
// {{{ Globals

void showTrx(void);
void setFreq(long int f);
void initPLL(void);
void initLCD(void);
void initADC(void);

int currentTime; // tmp global
int prevStepTime;
int goStep;

int Transmitting;           // boolean: TX = true, RX = false;
int Scanning;               // boolean: scanning = true;
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
long ReferenceFrequency;    // duh...

// Menu
// 
char *menuStrings[] =  { 
    "Mute level",       // 0
    "Shift",            // 1
    "CTCSS",            // 2
    "Scan Start",       // 3
    "Scan End",         // 4
    "Scan",             // 5
    "back"};            // 6

int  menuValues [] =  {  
    5,                  // 0
    -28,                // 1
    885,                // 2
    1280000,            // 3
    1290000,            // 4
    0,                  // 5
    0};                 // 6

#define MENULENGTH 7
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
int CtcssTones[] = {0,670,719,755,770,797,825,854,885,915,948,974,-1};
int ctcssIndex;


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
// {{{ void initialize(void)

void initialize(void)
{
    DialFrequency = INITIAL_FREQUENCY;
    FrequencyShift = -28;
    ScanStartFrequency = INITIAL_FREQUENCY;
    ScanEndFrequency = BANDTOP;
    ShiftEnable = FALSE;
    ReverseShift = FALSE;
    Transmitting = FALSE;
    menuLoopState  = MOFF;
    inputStateRotary = IDLE;
    inputStateSelector= IDLE;
    inputStateShift= IDLE;
    inputStatePTT = IDLE;
    prevStepTime = 0;

    initPLL();
    initLCD();
    initADC();

#ifdef TESTING
    vInRotState = 3;
    GetcAvail = FALSE;
    GetcBuffer = 0;
#endif
}

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
    deSetCursorPosition(YTop-1, XTop-1);
    deLineH(DE_WIDTH+2);
    deSetCursorPosition(YTop+DE_HEIGHT, XTop-1);
    deLineH(DE_WIDTH+2);
    for (y=0; y<DE_HEIGHT; y++)
    {
        deSetCursorPosition(YTop+y, XTop-1);
        printf("|");
        deSetCursorPosition(YTop+y, XTop+DE_WIDTH+1);
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
    static char yes[]="\033[32myes\033[30m";
    static char no[] ="\033[31m no\033[30m";
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
    inputStateSMeter = 10;
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
// {{{ outputs functions
// {{{ Display routines

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

// {{{ void setDisplay(int row, char *string)

void setDisplay(int row, char *string)
{
    setCursorPosition(row, 1);
    printf("%s",string);
}

// }}}

// {{{ void showFrequency(int row, int freq)

void showFrequency(int row, int fr)
{
    char line[DISPLAY_WIDTH+1];
    sprintf(line,"%4u.%03u MHz   ", fr/1000, fr%1000);
    setDisplay(row,line);
}

// }}}
// {{{ void showCTCSSfreq(int row)

void showCTCSSfreq(int row)
{
    char line[DISPLAY_WIDTH+1];
    sprintf(line,"%d.%d Hz %*s",CTCSSfrequency/10,CTCSSfrequency%10,DISPLAY_WIDTH-8," ");
    setDisplay(row, line);
}

// }}}

// {{{ void outputTrxBit(void) 

void outputTrxBit(void)
{
    // t.b.d.
}

// }}}
// {{{ void showTrx(void)

void showTrx(void)
{
    setCursorPosition(0, DISPLAY_WIDTH);
    if (Transmitting)
        printf("T");
    else
        printf("R");
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
    outputTrxBit();
    showTrx();
}

// }}}
// {{{ void setCTCSSfreq(void)

void  setCTCSSfreq(void)
{
    //  showCTCSSfreq();
}

// }}}

// {{{ void bottomLinePrinter(int index)

void bottomLinePrinter(int index)
{
    char line[DISPLAY_WIDTH+1];

    // bottom line
    switch (menuLoopState)
    {
        case MSSTART    :
        case MSSTARTVAR :
        case MSEND      :
        case MSENDVAR   : 
            showFrequency(1, menuValues[index]); 
            break;

        case MCTCSS     :
        case MCTCSSVAR  :
            showCTCSSfreq(1);
            break;

        case MBACK      :
        case MSCAN      : 
            sprintf(line,"%*s",DISPLAY_WIDTH, " ");
            setDisplay(1, line);
            break;

        default         : 
            sprintf(line,"%-*d",DISPLAY_WIDTH, menuValues[index]);
            setDisplay(1, line);
    }
}
// }}}
// {{{ void showMenuItem()

void showMenuItem()
{
    char line[DISPLAY_WIDTH+1];

    // top line
    sprintf(line,"> %-13s",menuStrings[menuLoopState]);
    setDisplay(0, line);

    // bottom line
    bottomLinePrinter(menuLoopState); 
}

// }}}
// {{{ void showMenuItemValue()

void showMenuItemValue()
{
    char line[DISPLAY_WIDTH+1];

    // top line
    sprintf(line,"Set %-11s", menuStrings[menuLoopState-MMUTELEVELVAR]);
    setDisplay(0, line);

    // bottom line
    bottomLinePrinter(menuLoopState-MMUTELEVELVAR); 
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
// {{{ void initPLL(void)

void initPLL(void)
{
    long int reg;
    ReferenceFrequency = 12000UL;

    cbi(PORTC, DATA);
    cbi(PORTC, CLK);
    cbi(PORTC, LE);

    // set function latch
    reg = 0x438086;
    setPLL(reg);

    // init R-counter
    reg = (2UL<<16) + ((ReferenceFrequency/CHANNELSTEP)<<2);
    setPLL(reg);

    setFreq(CurrentFrequency);

    reg = 0x438082;
    setPLL(reg);
}

// }}}

// }}}

void initADC(void)
{
}

// }}}

// {{{ void mainLoop(void)

void mainLoop(void)
{
    int busy = TRUE;
    initialize();
    setFrequency();
    while (busy)
    {
        // input stuff
        getInputRotary();
        getInputSelector();
        getInputTxRx();
        getInputShift();
        getInputSMeter();

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

        // the state machine does all the processing
        switch (menuLoopState)
        {
            // {{{ case MOFF :

            case MOFF : // 100

                // {{{ inputStateRotary

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
                        menuLoopState = MMUTELEVEL;
                        inputStateSelector = IDLE;
                        break;
                }
                break;
                // }}}

                // }}}
                // {{{  case MMUTELEVEL :

            case MMUTELEVEL : // 0
            case MSHIFT     : // 1
            case MCTCSS     : // 2
            case MSSTART    : // 3
            case MSEND      : // 4
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        menuLoopState++;
                        inputStateRotary = IDLE;
                        break;

                    case DOWNEVENT : 
                        menuLoopState--;
                        if (menuLoopState < 0) menuLoopState = MENULENGTH-1;
                        inputStateRotary = IDLE;
                        break;
                }

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch from item selection to value selection
                        menuLoopState += MMUTELEVELVAR;
                        inputStateSelector= IDLE;
                        break;
                }
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
                break;

                // }}}
                // {{{  case MSCAN:

            case MSCAN : // 5
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        menuLoopState++;
                        if (menuLoopState >=  MENULENGTH) 
                            menuLoopState = MMUTELEVEL;
                        inputStateRotary = IDLE;
                        break;

                    case DOWNEVENT : 
                        menuLoopState--;
                        inputStateRotary = IDLE;
                        break;
                }
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = MOFF;
                        Scanning = !Scanning;
                        inputStateSelector= IDLE;
                        showFrequency(0, CurrentFrequency);
                        break;
                }
                showMenuItem();
                break;

                // }}}
                // {{{  case MBACK :

            case MBACK : // 6
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        menuLoopState++;
                        if (menuLoopState >=  MENULENGTH) menuLoopState = MMUTELEVEL;
                        inputStateRotary = IDLE;
                        showMenuItem();
                        break;

                    case DOWNEVENT : 
                        menuLoopState--;
                        inputStateRotary = IDLE;
                        showMenuItem();
                        break;
                }
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = MOFF;
                        inputStateSelector= IDLE;
                        showFrequency(0, CurrentFrequency);
                        break;
                }
                break;

                // }}}
                // {{{  case MMUTELEVELVAR :

            case MMUTELEVELVAR : // 200
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        MuteLevel = (MuteLevel<MAXMUTELEVEL) ? MuteLevel+1 : MuteLevel;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = MuteLevel;
                        break;

                    case DOWNEVENT : 
                        MuteLevel = (MuteLevel>0) ? MuteLevel-1 : 0;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = MuteLevel;
                        break;
                }
                // }}}
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState = MMUTELEVEL;
                        inputStateSelector = IDLE;
                        break;
                }
                // }}}
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
                break;

                // }}}
                // {{{  case MSHIFTVAR :

            case MSHIFTVAR : // 201
                // {{{ up/down
                switch (inputStateRotary)
                {
                    case UPEVENT :
                        FrequencyShift = (FrequencyShift < MAXSHIFT) ? FrequencyShift+1 : MAXSHIFT;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = FrequencyShift;
                        break;

                    case DOWNEVENT : 
                        FrequencyShift = (FrequencyShift > MINSHIFT) ? FrequencyShift-1 : MINSHIFT;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = FrequencyShift;
                        break;
                }
                // }}}
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState -= MMUTELEVELVAR;
                        menuValues[menuLoopState] = FrequencyShift;
                        inputStateSelector= IDLE;
                        break;
                }
                menuValues[menuLoopState] = FrequencyShift;
                // }}}
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
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
                        menuValues[menuLoopState-MMUTELEVELVAR] = CTCSSfrequency;
                        break;

                    case DOWNEVENT : 
                        inputStateRotary = IDLE;
                        ctcssIndex--;
                        if (ctcssIndex == -1)
                            ctcssIndex=0;
                        CTCSSfrequency = CtcssTones[ctcssIndex]; 
                        menuValues[menuLoopState-MMUTELEVELVAR] = CTCSSfrequency;
                        break;
                }
                // }}}
                CTCSSenable    = (CTCSSfrequency != 0);
                // {{{ select
                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        menuLoopState -= MMUTELEVELVAR;
                        //menuValues[menuLoopState] = CTCSSfrequency;
                        inputStateSelector= IDLE;
                        break;
                }
                // }}}
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
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
                        menuValues[menuLoopState-MMUTELEVELVAR] = ScanStartFrequency;
                        break;

                    case DOWNEVENT : 
                        ScanStartFrequency -= (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanStartFrequency < BANDBOTTOM) ScanStartFrequency = BANDBOTTOM;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = ScanStartFrequency;
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch back to item selection
                        menuLoopState -= MMUTELEVELVAR;

                        menuValues[menuLoopState] = ScanStartFrequency;
                        inputStateSelector= IDLE;
                        break;
                }

                // }}}
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
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
                        menuValues[menuLoopState-MMUTELEVELVAR] = ScanEndFrequency;
                        break;

                    case DOWNEVENT : 
                        ScanEndFrequency -= (LargeStepEnable) ? LARGESTEP : CHANNELSTEP;
                        if (ScanEndFrequency < ScanStartFrequency) ScanEndFrequency = ScanStartFrequency;
                        inputStateRotary = IDLE;
                        menuValues[menuLoopState-MMUTELEVELVAR] = ScanEndFrequency;
                        break;
                }
                // }}}
                // {{{ select

                switch (inputStateSelector)
                {
                    case SELECTEVENT :
                        // switch back to item selection
                        menuLoopState -= MMUTELEVELVAR;

                        menuValues[menuLoopState] = ScanEndFrequency;
                        inputStateSelector= IDLE;
                        break;
                }

                // }}}
                if (menuLoopState<MMUTELEVELVAR) 
                    showMenuItem();
                else
                    showMenuItemValue();
                break;

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

        // {{{ testing
#ifdef TESTING

        if (TRUE)
        {
            deSetCursorPosition(DBGROW,1);
            printf("=====================\r\n");
            printf("CTCSSenable     = %3s\r\n",yesno(CTCSSenable));
            printf("CTCSSfrequency  = %3d\r\n",CTCSSfrequency);
            printf("MuteLevel       = %3d\r\n",MuteLevel);
            printf("ShiftEnable     = %3s\r\n",yesno(ShiftEnable));
            printf("LargeEnable     = %3s\r\n",yesno(LargeStepEnable));
            printf("FrequencyShift  = %3d\r\n",FrequencyShift);
            printf("Transmitting    = %3s\r\n",yesno(Transmitting));
            printf("Scanning        = %3s\r\n",yesno(Scanning));
            printf("Scan Start      = %d.%03d\r\n", ScanStartFrequency/1000, ScanStartFrequency%1000);
            printf("Scan End        = %d.%03d\r\n", ScanEndFrequency/1000, ScanEndFrequency%1000);
            printf("=====================\r\n");
            printf("inputStateRotary= %3d\r\n",inputStateRotary);
            printf("menuLoopState   = %3d\r\n",menuLoopState);
            printf("currentTime     = %3d\r\n",currentTime);
            printf("prevStepTime    = %3d\r\n",prevStepTime);
            printf("ctcssIndex      = %3d\r\n",ctcssIndex);
            printf("vInRotState     = %3d\r\n",vInRotState);
            printf("unget charbuf   = %3X\r\n",GetcBuffer);
            printf("unget charvail  = %3s\r\n",yesno(GetcAvail));
            printf("sample          = %3d\r\n",S);
            printf("=====================\r\n");
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
    printf("23cm NBFM control software simulator\n");
    printf("\n");
    printf("q quit\n"); 
    printf("[ upward rotating        ] downward rotating\n"); 
    printf("t transmit               r receive\n"); 
    printf("s toggle shift on/off    l toggle large steps on/off\n"); 
    printf("e menu selector button\n");
    printf("\n");
    set_conio_terminal_mode();
#endif

    // }}}

    // dbg = fopen("log.txt","w");
    mainLoop();
    // fclose(dbg);
}

// }}}

// {{{
// }}}
// EOF
