/* to include in the NBFM_Simulator.c code */
// vim: ts=4 et sw=4 foldmethod=marker


struct TestDefinition
{
    // inputs
    int rotaryChange;     // up down counter
    int selectorPushed;   // boolean True=pushed
    int shiftEnable;      // boolean True=shifted
    int reverseShift;     // boolean True=reversed
    int ptt;              // boolean True=transmitting
    int signal;           // SMeter value
    // outputs
    char  txBit;          // boolean True=transmitting
    char  muteBit;        // boolean True=muted
    char  toneBit;        // not sure yet
    char  lineTop[20];    // string
    char  lineBottom[20]; // string
};

struct TestDefinition tests[] =
{// rot, sel,   shift, rever,  ptt, signal,      tx~rx,  mute,   tone,   topline         ,   bottomline
                            //assume mutelevel = 10 
    { 0, FALSE, FALSE, FALSE, FALSE, 980-2*12,   FALSE, FALSE,  FALSE, "VFO:1298.200 MHz", "M              R" }, 
    { 1, FALSE, FALSE, FALSE, FALSE, 980-2*2 ,   FALSE,  TRUE,  FALSE, "VFO:1298.225 MHz", "M              R" }, 
    {-1, FALSE, FALSE, FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.200 MHz", "M              R" }, 
    { 4, FALSE, FALSE, FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.300 MHz", "M              R" }, 
    {-8, FALSE, FALSE, FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
    { 0, FALSE, FALSE, FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
    { 0, FALSE, FALSE, FALSE, TRUE , 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.100 MHz", "M              T" }, 
    { 0, FALSE, FALSE, FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
    { 0, FALSE, TRUE , FALSE, FALSE, 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
    { 0, FALSE, TRUE , FALSE, TRUE , 980-2*1 ,   FALSE,  TRUE,  FALSE, "VFO:1270.100 MHz", "M              T" }
};

int TotalTests;
int testNr;
FILE *testlog;

#define TESTLOG "./test_results.log"

void TEST_Initialize(void) 
{
    testNr = -1;
    TotalTests = sizeof(tests)/sizeof(struct TestDefinition);
    testlog = fopen(TESTLOG,"w");
    if (!testlog)
    {
        fprintf(stderr,"Could not create file %s\n",TESTLOG);
    }
}


int  TEST_GetNextTest(int testNr) 
{
    int rv;

    if (testNr >= TotalTests)
        rv = -1;
    else
        rv = testNr+1;
    return rv;
} 


void TEST_SetInputs(int testNr) 
{
    IRQ_RotaryChange   = tests[testNr].rotaryChange;
    IRQ_SelectorPushed = tests[testNr].selectorPushed;
    SS_ShiftEnable     = tests[testNr].shiftEnable;
    SS_PTT             = tests[testNr].ptt;
}


int  TEST_ExpectResults(int testNr)
{
    int success = TRUE;

    success &= (tests[testNr].txBit   == SS_Transmitting); 
    success &= (tests[testNr].muteBit == SS_Muted); 
    success &= (strcmp(tests[testNr].lineTop,    LineT) == 0);
    //  success &= (strcmp(tests[testNr].lineBottom, LineB) == 0);
    return success;
} 

char* TEST_yesNo(int bool)
{
    return (bool) ? "yes" : " no"; 
}


void TEST_Log(int testNr, int success) 
{
    if (success)
    {
        fprintf(testlog,"Test %3d OK\n",testNr);
    } else
    {
        fprintf(testlog,"test %3d, Expected values:\n",testNr);
        fprintf(testlog,"  txbit: %s\n",TEST_yesNo(tests[testNr].txBit));
        fprintf(testlog,"  mute : %s\n",TEST_yesNo(tests[testNr].muteBit));
        fprintf(testlog,"  top  : %s\n",tests[testNr].lineTop);
        fprintf(testlog,"  bot  : %s\n",tests[testNr].lineBottom);
        fprintf(testlog,"          Real values:\n");
        fprintf(testlog,"  txbit: %s\n",TEST_yesNo(SS_Transmitting));
        fprintf(testlog,"  mute : %s\n",TEST_yesNo(SS_Muted));
        fprintf(testlog,"  top  : %s\n",LineT);
        fprintf(testlog,"  bot  : %s\n",LineB);
    }
}

