// {{{ void OutputSetVfoFrequency(long int vfoFreq)

void OutputSetVfoFrequency(long int vfoFreq)
{
  static long int prevVfo;
  long int reg, frast;
  long int fRasterHigh, fRasterLow;

  if (prevVfo != vfoFreq)
  {
    prevVfo = vfoFreq;
    frast = vfoFreq / F_RASTER;
    fRasterHigh = frast/16;
    fRasterLow  = frast%16;

    reg = ((fRasterHigh & 0x1fff)<<8) + ((fRasterLow & 0x3f)<<2) + 1;
    setPLL(reg);
  }
}

// }}}

