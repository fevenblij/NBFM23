
// {{{ void OutputSetPLL(long int r)

void OutputSetPLL(long int r)
{
    char i;

    for (i=0; i<24; i++) 
    {
        if (r & 0x800000)
            sbi(PORTC, DATA);
        else
            cbi(PORTC, DATA);

        // output the long word via bit banging
        _delay_us(1);
        sbi(PORTC, CLK);
        _delay_us(1);
        cbi(PORTC, CLK);
        r <<= 1;
    }

    // activate the latch
    _delay_us(1);
    sbi(PORTC, LE);
    _delay_us(1);
    cbi(PORTC, LE);
}

// }}}
