// {{{ void OutputSetAudioMute(char mute)

#define MUTEINDICATOR 'M'
#define BLANK         ' '

void OutputSetAudioMute(char mute)
{
  // boolean mute;
  static char prevMute;

  if (prevMute != mute)
  {
    prevMute = mute;
    lcdCursorPosition(1,0);  // bottom row, first column;
    if (mute) 
    {
      sbi(PORTC, MUTE);
      lcdData(MUTEINDICATOR); 
    } else {
      cbi(PORTC, MUTE);
      lcdData(BLANK); 
    }
  }
}


// }}}

