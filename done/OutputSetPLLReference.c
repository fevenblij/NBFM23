// {{{ void OutputSetPLLReference(long int reference)

void OutputSetPLLReference(long int reference)
{   
  // init R-counter
  reg = (2UL<<16) + ((SS_PllReferenceFrequency/F_RASTER)<<2);
  setPLL(reg);

}

// }}}


