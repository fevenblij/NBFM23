#define DISPHEIGHT   2
#define DISPWIDTH   16

// {{{ void lcdCursorPosition(int row, int column)

void lcdCursorPosition(int row, int column)
{
// constants defined according to the datasheet
#define row1 0x80
#define row2 0xC0

  unsigned char position;

  // guarantee the inputs are in range
  row = row % DISPHEIGHT;  
  column = column % DISPWIDTH;

  position = row1 + column;   // coloumn + 0x80

  if (row == 1)
    position += (row2-row1);  // coloumn + 0xC0

  // write position to display command register
  lcdCmd(position);
}

// }}}}
