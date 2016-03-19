// {{{ void initPLL(void)

void initPLL(void)
{
#ifdef TESTING
  // $$$FHE Do nothing for now
#else
  long reg;
  SS_PllReferenceFrequency = 13000UL;

  // clear signal lintes
  cbi(PORTC, ADATA);
  cbi(PORTC, ACLK);
  cbi(PORTC, ALE);

  // init function latch
  reg = 0x438086;
  setPLL(reg);

  // init R-counter c.q. the reference frequency
  reg = (2UL<<16) + ((SS_PllReferenceFrequency/CHANNELSTEP)<<2);
  setPLL(reg);

  
  // init the start up channel
  setFreq(SS_VfoFrequency);

  // init something else
  reg = 0x438082;
  setPLL(reg);
#endif
}

// }}}

