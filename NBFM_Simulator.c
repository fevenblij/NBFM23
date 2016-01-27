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

#define INITIAL_FREQUENCY 1298200
#define CHANNELSTEP 25

#define DISPROW     10 
#define DISPCOL     10
#define DBGROW      13

#define CLKMASK     1
#define DATAMASK    2

#define UPEVENT     1
#define DOWNEVENT   2
#define TXEVENT     3
#define RXEVENT     4
#define SELECTEVENT 5
#define SHIFTEVENT  6

#define MOFF        100
#define MSELECT     101
#define MMUTELEVEL  102
#define MSHIFT      103
#define MCTCSS      104
#define MBACK       105

#define MAXMUTELEVEL    32
#define MAXMPTR         5
#define MINSHIFT        10
#define MAXSHIFT        90
#define MINCTCSS        10
#define MAXCTCSS        90
/*
#define 
#define 
#define 
#define 
#define 
#define 
 */
// }}}

// {{{ Globals

int Transmitting;           // boolean: TX = true, RX = false;
int DialFrequency;          // frequency as set by dial
int CurrentFrequency;       // frequency setting sent to synthesizer
int ShiftEnable;            // boolean: shifted = true;
int ReverseShift;           // boolean: swap transmit and receive frequencies
int FrequencyShift;         // size of frequency shift
int MuteLevel;              // level below which audio muting is enabled
int CTCSSenable;            // boolean: CTCSS tone enabled during transmit
int CTCSSfrequency;         // frequency of CTCSS tone to use

// Menu
// 
char *menuStrings[] =  { "Mute level", "Shift", "CTCSS", "back"}; 
int   menuValues [] =  {  5          ,  -28000,  885   ,  0    };   
int menuState;
int menuIndex;

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
#endif

// }}}
// {{{ void initialize(void)

void initialize(void)
{
    DialFrequency = INITIAL_FREQUENCY;
    FrequencyShift = -28000;
    ShiftEnable = FALSE;
    ReverseShift = FALSE;
    theMainEvent = -1;  // forces the display to be printed, but does not do anything
    menuState  = MOFF;

#ifdef TESTING
    GetcAvail = FALSE;
    GetcBuffer = 0;
#endif
}

// }}}
// {{{ Scaffolding

// {{{ void clearScreen(void)

void clearScreen(void)
{
    printf("\033[2J");
    printf("\033[?1h");
}


// }}}
// {{{ void setCursorPosition(int row, int col)

void setCursorPosition(int row, int col)
{
    printf("\033[%d;%df",row,col);
}

// }}}
// {{{ void top(void)

void top(void)
{
    setCursorPosition(1,1);
}

// }}}
// {{{ char* yesno(int boolean)

char* yesno(int boolean)
{
    static char yes[]="\033[32myes\033[30m";
    static char no[] ="\033[31mno \033[30m";
    return (boolean ? yes : no); 
}

// }}}

// {{{ void Scaffold(void)

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

void Scaffold(void)
{
}

#endif

// }}}
//}}}

// {{{ input functions

// {{{ int getStatusRotary(void)

void getStatusRotary(void)
{
#ifdef TESTING
    char c;
#endif

    int clk;
    int data;
    int sample;
    int qd;
    int UpDown;

    sample = input;
    clk = (sample & CLKMASK) == 0;
    data = (sample & DATAMASK) == 0;

    /* double edge triggered flipflop creates   */
    /* clean clock pulse (qd)                   */
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
            CurrentFrequency -= CHANNELSTEP;
        else
            CurrentFrequency += CHANNELSTEP;
    }

    GClkPrev = clk;
    GQdPrev  = qd;

#ifdef TESTING
    // =========================

    if ((c = FHEgetc()) > 0)
    {
        switch (c)
        {
            case 'u' :
                theMainEvent = UPEVENT;
                break;

            case 'd' :
                theMainEvent = DOWNEVENT;
                break;

            default :
                FHEungetc(c);

        }

        // =========================
#endif
    }
}

// }}}
// {{{ int getStatusSelector(void)

void getStatusSelector(void)
{
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 'e'  :
            case '\n' :
                theMainEvent = SELECTEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
}

// }}}
// {{{ int getStatusTxRx(void)

int getStatusTxRx(void)
{
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 't' :
                theMainEvent = TXEVENT;
                break;

            case 'r' :
                theMainEvent = RXEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
    return 0;
}

// }}}
// {{{ int getStatusShift(void)

void getStatusShift(void)
{
    char c;

    if ((c = FHEgetc()))
    {
        switch (c)
        {
            case 's'  :
                theMainEvent = SHIFTEVENT;
                break;

            default :
                FHEungetc(c);
        }
    }
}

// }}}

// }}}
// {{{ outputs functions

// {{{ int setDisplay(int row, char *string)

int setDisplay(int row, char *string)
{
    setCursorPosition(DISPROW+row, DISPCOL);
    printf("%s                    ",string);
    return 0;
}

// }}}
// {{{ void showFrequency(void)

void showFrequency(void)
{
    char line1[40];
    sprintf(line1,"%u.%u MHz", CurrentFrequency/1000, CurrentFrequency%1000);
    setDisplay(0,line1);
    setDisplay(1,"                    ");
}

// }}}
// {{{ void setFrequency(void)

void setFrequency(void)
{
    CurrentFrequency = DialFrequency 
        + (( Transmitting && !ReverseShift && ShiftEnable) ? FrequencyShift : 0)
        + ((!Transmitting &&  ReverseShift && ShiftEnable) ? FrequencyShift : 0);
    showFrequency();
}

// }}}
// {{{ void setTransmitter(void)

void setTransmitter(void)
{
    setFrequency();
    // if (Transmitting) outputTransmitterBit();
}

// }}}
// {{{ void showCTCSSfreq(void)

void showCTCSSfreq(void)
{
    char line1[40];
    char line2[40];
    sprintf(line1,"%d.%d Hz ",CTCSSfrequency/10,CTCSSfrequency%10);
    setDisplay(0, line1);
    setDisplay(1, "> ");
}

// }}}
// {{{ void setCTCSSfreq(void)

void  setCTCSSfreq(void)
{
    showCTCSSfreq();
}

// }}}

// {{{ void showMenuItem()

void showMenuItem()
{
    char line[40];

    setDisplay(0, menuStrings[menuIndex]);
    sprintf(line, "%d        ",menuValues[menuIndex]); 
    setDisplay(1, line);
}

// }}}
// {{{
// }}}
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
        getStatusRotary();
        getStatusSelector();
        getStatusTxRx();
        getStatusShift();

        // the state machine does all the processing
        switch (menuState)
        {
            // {{{ case MOFF :
            case MOFF : // 100
                switch (theMainEvent)
                {
                    case UPEVENT :
                        DialFrequency += CHANNELSTEP;
                        setFrequency();
                        break;

                    case DOWNEVENT : 
                        DialFrequency -= CHANNELSTEP;
                        setFrequency();
                        break;

                    case TXEVENT :
                        Transmitting = TRUE;
                        setTransmitter();
                        break;

                    case RXEVENT :
                        Transmitting = FALSE;
                        setTransmitter();
                        break;

                    case SHIFTEVENT :
                        ShiftEnable = !ShiftEnable;
                        setFrequency();
                        break;

                    case SELECTEVENT :
                        menuState = MSELECT;
                        menuIndex = 0;
                        break;
                }
                break;
                // }}}
                // {{{ case MSELECT :

            case MSELECT : // 101
                switch (theMainEvent)
                {
                    case UPEVENT :
                        menuIndex = (menuStrings[menuIndex] != 0) ? menuIndex+1 : menuIndex;;
                        break;

                    case DOWNEVENT : 
                        menuIndex = (menuIndex>0) ? menuIndex-1 : 0;
                        break;

                    case SELECTEVENT :
                        menuState = MMUTELEVEL + menuIndex;
                }
                showMenuItem();
                break;

                // }}}
                // {{{  case MMUTELEVEL :

            case MMUTELEVEL : // 102
                switch (theMainEvent)
                {
                    case UPEVENT :
                        MuteLevel = (MuteLevel<MAXMUTELEVEL) ? MuteLevel+1 : MuteLevel;
                        break;

                    case DOWNEVENT : 
                        MuteLevel = (MuteLevel>0) ? MuteLevel-1 : 0;
                        break;

                    case SELECTEVENT :
                        menuState = MSELECT;
                        break;
                }
                // menuState - MMUTELEVEL = 0 ==> menuIndex voor mute level
                menuValues[menuState - MMUTELEVEL] = MuteLevel;
                showMenuItem();
                break;
                // }}}
                // {{{  case MSHIFT :

            case MSHIFT : // 103
                switch (theMainEvent)
                {
                    case UPEVENT :
                        FrequencyShift = (FrequencyShift<MAXSHIFT) ? FrequencyShift+1 : MAXSHIFT;
                        break;

                    case DOWNEVENT : 
                        FrequencyShift = (FrequencyShift>MINSHIFT) ? FrequencyShift-1 : MINSHIFT;
                        break;
                }
                // menuState - MMUTELEVEL = 0 ==> menuIndex voor mute level
                menuValues[menuState - MMUTELEVEL] = MuteLevel;
                showMenuItem();
                break;
                // }}}
                // {{{  case MCTCSS :

            case MCTCSS : // 104
                switch (theMainEvent)
                {
                    case UPEVENT :
                        ctcssIndex++;
                        if (CtcssTones[ctcssIndex] == -1) 
                            ctcssIndex--;
                        break;

                    case DOWNEVENT : 
                        ctcssIndex--;
                        if (ctcssIndex == -1)
                            ctcssIndex=0;
                        break;
                }
                CTCSSfrequency = CtcssTones[ctcssIndex]; 
                CTCSSenable    = (CTCSSfrequency != 0);
                setCTCSSfreq();
                // menuState - MMUTELEVEL = 0 ==> menuIndex voor mute level
                menuValues[menuState - MMUTELEVEL] = MuteLevel;
                showMenuItem();
                break;

                // }}}
                // {{{  case MBACK :

            case MBACK : // 105
                menuState = MOFF;
                break;
                // }}}
        }

        // {{{ testing
#ifdef TESTING
        char c;
        if ((c = FHEgetc()))
        {
            if (c == 'q')
            {
                busy = FALSE;
            } else
            {
                FHEungetc(c);
            }
        }   
        // swallow any unused characters
        (void)FHEgetc();

        if (theMainEvent != 0)
        {
            setCursorPosition(DBGROW,1);
            printf("=====================\r\n");
            printf("CTCSSenable     = %s\r\n",yesno(CTCSSenable));
            printf("CTCSSfrequency  = %d\r\n",CTCSSfrequency);
            printf("MuteLevel       = %d\r\n",MuteLevel);
            printf("ShiftEnable     = %s\r\n",yesno(ShiftEnable));
            printf("FrequencyShift  = %d\r\n",FrequencyShift);
            printf("Transmitting    = %s\r\n",yesno(Transmitting));
            printf("=====================\r\n");
            printf("theMainEvent    = %d\r\n",theMainEvent);
            printf("menuState       = %d\r\n",menuState);
            printf("menuIndex       = %d\r\n",menuIndex);
            printf("ctcssIndex      = %d\r\n",ctcssIndex);
            printf("charbuf         = %X\r\n",GetcBuffer);
            printf("charvail        = %s\r\n",yesno(GetcAvail));
            printf("=====================\r\n");
        }
#endif
        // }}}

        theMainEvent = 0;
    }
}

// }}}
// {{{ int main(int argc, char *argv[])

int main(int argc, char *argv[])
{
    // {{{ testing

#ifdef TESTING
    clearScreen();
    setCursorPosition(1,1);
    printf("23cm NBFM control software scaffolding simulator\n");
    printf("q quit\n");
    printf("u upward rotating \n");
    printf("d downward rotating\n");
    printf("t transmit\n");
    printf("r receive\n");
    printf("s toggle shift on/off\n");
    printf("enter: menu selector button\n");
    printf("\n");
    printf("\n");
    set_conio_terminal_mode();
#endif

    // }}}

    mainLoop();
}

// }}}

// {{{
// }}}
// EOF
