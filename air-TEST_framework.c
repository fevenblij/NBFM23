/* to include in the NBFM_Simulator.c code */

// #include "TEST_framework.c"
/*
  static int testNr;
  if (testing)
  {
    testNr = TEST_GetNextTest(testNr);
    TEST_SetInputs(testNr); 
    results = TEST_ExpectResults(testNr);
    TEST_log(testNr, results);
  }
*/

/* Test framework contains : */

struct TestDefinition
{
// inputs
  int rotaryChange;     // up down counter
  int selectorPushed;   // boolean True=pushed
  int shiftEnable;      // boolean True=shifted
  int reverseShift;     // boolean True=reversed
  int ptt;              // boolean True=transmitting
// outputs
  char  txBit;          // boolean True=transmitting
  char  muteBit;        // boolean True=muted
  char  toneBit;        // not sure yet
  char  lineTop[20];    // string
  char  lineBottom[20]; // string
};

struct TestDefinition tests[] =
{// rot, sel, shift, rever,  ptt,    txing, mute,  tone,   topline          , bottomline
  { 0, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.200 MHz", "M              R" }, 
  { 1, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.225 MHz", "M              R" }, 
  {-1, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.200 MHz", "M              R" }, 
  { 4, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.300 MHz", "M              R" }, 
  {-8, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
  { 0, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
  { 0, FALSE, FALSE, FALSE, TRUE ,   FALSE, TRUE,  FALSE, "VFO:1298.100 MHz", "M              T" }, 
  { 0, FALSE, FALSE, FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
  { 0, FALSE, TRUE , FALSE, FALSE,   FALSE, TRUE,  FALSE, "VFO:1298.100 MHz", "M              R" }, 
  { 0, FALSE, TRUE , FALSE, TRUE ,   FALSE, TRUE,  FALSE, "VFO:1270.100 MHz", "M              T" }
};

int TotalTests;
int testNr;
FILE *testlog;

#define TESTLOG "./test_results.log"

void TEST_Initialize(void) 
{
  testNr = 0;
  TotalTests = sizeof(tests)/sizeof(struct TestDefinition);
  testlog = fopen(TESTLOG,"w");
  if (!testlog)
  {
    fprintf(stderr,"Could not create file %s\n",TESTLOG);
  }
}


int  TEST_GetNextTest(int testNr) 
{
  if (testNr == TotalTests)
    return FALSE;
  else
    return testNr++;
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
  success &= (strcmp(tests[testNr].lineTop,    &Line[0]) == 0);
  success &= (strcmp(tests[testNr].lineBottom, &Line[1]) == 0);
  return success;
} 


void TEST_Log(int testNr, int success) 
{
  if (!success)
  {
    fprintf(testlog,"Expected values:\n");
    yesno(tests[testNr].txBit);
    yesno(tests[testNr].muteBit);
    fprintf(testlog,"Real values:\n");
    yesno(SS_Transmitting);
    yesno(SS_Muted);
  } else
  {
    fprintf(testlog,"Test %3d OK\n",testNr);
  }
}

