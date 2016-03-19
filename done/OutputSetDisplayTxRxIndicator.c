// {{{ void OutputSetDisplayTxRxIndicator(char indicator)

// only write to the (slow) display if the indicator
// really should be updated

void OutputSetDisplayTxRxIndicator(char indicator)
{
  static char prevIndicator;

  if (prevIndicator != indicator)
  {
    prevIndicator = indicator;
    lcdCursorPosition(1, DISPWIDTH-1);
    lcdData(indicator);
  }
}

// }}
