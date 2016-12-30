#include "font.h"

uint16_t BACK_COLOR, POINT_COLOR;

#define SET_RS  (PORTC |= (1<<PORTC5))    // PC2
#define CLR_RS  (PORTC &= ~(1<<PORTC5))

#define SET_WR  (PORTC |= (1<<PORTC4))    // PC2
#define CLR_WR  (PORTC &= ~(1<<PORTC4))

// Pin RD en alto
//#define SET_RD  (PORTC |= (1<<PORTCx))    // PC
//#define CLR_RD  (PORTC &= ~(1<<PORTCx))

// Pin CS en bajo
#define SET_CS  (PORTC |= (1<<PORTC3))    // PC
#define CLR_CS  (PORTC &= ~(1<<PORTC3))

#define SET_REST  (PORTC |= (1<<PORTC2))  // PC0
#define CLR_REST  (PORTC &= ~(1<<PORTC2))



#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE           0x001F
#define BRED             0XF81F
#define GRED       0XFFE0
#define GBLUE      0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN        0XBC40
#define BRRED        0XFC07
#define GRAY         0X8430

#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C
#define GRAYBLUE         0X5458

#define LIGHTGREEN       0X841F
#define LGRAY        0XC618

#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12
// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
void LCD_Writ_Bus(char VH, char VL)
{
  PORTD = VH;
  CLR_WR;
  SET_WR;
  PORTD = VL;
  CLR_WR;
  SET_WR;
}


void LCD_Write_COM(char VH, char VL)
{
  CLR_RS;
  LCD_Writ_Bus(VH, VL);
}


void LCD_Write_DATA(char VH, char VL)
{
  SET_RS;
  LCD_Writ_Bus(VH, VL);
}

void Address_set(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
  LCD_Write_COM(0x00, 0x20);
  LCD_Write_DATA(x1 >> 8, x1);
  LCD_Write_COM(0x00, 0x21);
  LCD_Write_DATA(y1 >> 8, y1);
  LCD_Write_COM(0x00, 0x50);
  LCD_Write_DATA(x1 >> 8, x1);
  LCD_Write_COM(0x00, 0x52);
  LCD_Write_DATA(y1 >> 8, y1);
  LCD_Write_COM(0x00, 0x51);
  LCD_Write_DATA(x2 >> 8, x2);
  LCD_Write_COM(0x00, 0x53);
  LCD_Write_DATA(y2 >> 8, y2);
  LCD_Write_COM(0x00, 0x22);
}



void LCD_Init(void)
{

  DDRD = 0xFF; // Datos
  PORTD = 0x00;
  DDRC |= (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2);

  SET_REST;
  delay(5);
  CLR_REST;
  delay(5);
  SET_REST;
  delay(5);

  CLR_CS;
  //************* Start Initial Sequence **********//
  LCD_Write_COM(0x00, 0xE5);
  LCD_Write_DATA(0x78, 0xF0); // set SRAM internal timing
  LCD_Write_COM(0x00, 0x01);
  LCD_Write_DATA(0x01, 0x00); // set SS and SM bit
  LCD_Write_COM(0x00, 0x02);
  LCD_Write_DATA(0x07, 0x00); // set 1 line inversion
  LCD_Write_COM(0x00, 0x03);
  LCD_Write_DATA(0x10, 0x30); // set GRAM write direction and BGR=1.
  LCD_Write_COM(0x00, 0x04);
  LCD_Write_DATA(0x00, 0x00); // Resize register
  LCD_Write_COM(0x00, 0x08);
  LCD_Write_DATA(0x02, 0x07); // set the back porch and front porch
  LCD_Write_COM(0x00, 0x09);
  LCD_Write_DATA(0x00, 0x00); // set non-display area refresh cycle ISC[3:0]
  LCD_Write_COM(0x00, 0x0A);
  LCD_Write_DATA(0x00, 0x00); // FMARK function
  LCD_Write_COM(0x00, 0x0C);
  LCD_Write_DATA(0x00, 0x00); // RGB interface setting
  LCD_Write_COM(0x00, 0x0D);
  LCD_Write_DATA(0x00, 0x00); // Frame marker Position
  LCD_Write_COM(0x00, 0x0F);
  LCD_Write_DATA(0x00, 0x00); // RGB interface polarity
  //*************Power On sequence ****************//
  LCD_Write_COM(0x00, 0x10);
  LCD_Write_DATA(0x00, 0x00); // SAP, BT[3:0], AP, DSTB, SLP, STB
  LCD_Write_COM(0x00, 0x11);
  LCD_Write_DATA(0x00, 0x07); // DC1[2:0], DC0[2:0], VC[2:0]
  LCD_Write_COM(0x00, 0x12);
  LCD_Write_DATA(0x00, 0x00); // VREG1OUT voltage
  LCD_Write_COM(0x00, 0x13);
  LCD_Write_DATA(0x00, 0x00); // VDV[4:0] for VCOM amplitude
  LCD_Write_COM(0x00, 0x07);
  LCD_Write_DATA(0x00, 0x01);
  _delay_ms(50); // Dis-charge capacitor power voltage
  LCD_Write_COM(0x00, 0x10);
  LCD_Write_DATA(0x10, 0x90); // 1490//SAP, BT[3:0], AP, DSTB, SLP, STB
  LCD_Write_COM(0x00, 0x11);
  LCD_Write_DATA(0x02, 0x27); // DC1[2:0], DC0[2:0], VC[2:0]
  _delay_ms(50); // Delay 50ms
  LCD_Write_COM(0x00, 0x12);
  LCD_Write_DATA(0x00, 0x1F); //001C// Internal reference voltage= Vci;
  _delay_ms(50); // Delay 50ms
  LCD_Write_COM(0x00, 0x13);
  LCD_Write_DATA(0x15, 0x00); //0x1000//1400   Set VDV[4:0] for VCOM amplitude  1A00
  LCD_Write_COM(0x00, 0x29);
  LCD_Write_DATA(0x00, 0x27); //0x0012 //001a  Set VCM[5:0] for VCOMH  //0x0025  0034
  LCD_Write_COM(0x00, 0x2B);
  LCD_Write_DATA(0x00, 0x0D); // Set Frame Rate   000C
  _delay_ms(50); // Delay 50ms
  LCD_Write_COM(0x00, 0x20);
  LCD_Write_DATA(0x00, 0x00); // GRAM horizontal Address
  LCD_Write_COM(0x00, 0x21);
  LCD_Write_DATA(0x00, 0x00); // GRAM Vertical Address
  // ----------- Adjust the Gamma Curve ----------//
  LCD_Write_COM(0x00, 0x30);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x31);
  LCD_Write_DATA(0x07, 0x07);
  LCD_Write_COM(0x00, 0x32);
  LCD_Write_DATA(0x03, 0x07);
  LCD_Write_COM(0x00, 0x35);
  LCD_Write_DATA(0x02, 0x00);
  LCD_Write_COM(0x00, 0x36);
  LCD_Write_DATA(0x00, 0x08); //0207
  LCD_Write_COM(0x00, 0x37);
  LCD_Write_DATA(0x00, 0x04); //0306
  LCD_Write_COM(0x00, 0x38);
  LCD_Write_DATA(0x00, 0x00); //0102
  LCD_Write_COM(0x00, 0x39);
  LCD_Write_DATA(0x07, 0x07); //0707
  LCD_Write_COM(0x00, 0x3C);
  LCD_Write_DATA(0x00, 0x02); //0702
  LCD_Write_COM(0x00, 0x3D);
  LCD_Write_DATA(0x1D, 0x04); //1604

  //------------------ Set GRAM area ---------------//
  LCD_Write_COM(0x00, 0x50);
  LCD_Write_DATA(0x00, 0x00); // Horizontal GRAM Start Address
  LCD_Write_COM(0x00, 0x51);
  LCD_Write_DATA(0x00, 0xEF); // Horizontal GRAM End Address
  LCD_Write_COM(0x00, 0x52);
  LCD_Write_DATA(0x00, 0x00); // Vertical GRAM Start Address
  LCD_Write_COM(0x00, 0x53);
  LCD_Write_DATA(0x01, 0x3F); // Vertical GRAM Start Address
  LCD_Write_COM(0x00, 0x60);
  LCD_Write_DATA(0xA7, 0x00); // Gate Scan Line
  LCD_Write_COM(0x00, 0x61);
  LCD_Write_DATA(0x00, 0x01); // NDL,VLE, REV
  LCD_Write_COM(0x00, 0x6A);
  LCD_Write_DATA(0x00, 0x00); // set scrolling line
  //-------------- Partial Display Control ---------//
  LCD_Write_COM(0x00, 0x80);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x81);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x82);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x83);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x84);
  LCD_Write_DATA(0x00, 0x00);
  LCD_Write_COM(0x00, 0x85);
  LCD_Write_DATA(0x00, 0x00);
  //-------------- Panel Control -------------------//
  LCD_Write_COM(0x00, 0x90);
  LCD_Write_DATA(0x00, 0x10);
  LCD_Write_COM(0x00, 0x92);
  LCD_Write_DATA(0x06, 0x00);
  LCD_Write_COM(0x00, 0x07);
  LCD_Write_DATA(0x01, 0x33); // 262K color and display ON
  SET_CS;
  Pant(0xff, 0xff);
}


void Pant(char VH, char VL)
{
  int i, j;
  CLR_CS;
  Address_set(0, 0, 240, 320);
  for (i = 0; i < 320; i++)
  {
    for (j = 0; j < 240; j++)
    {
      LCD_Write_DATA(VH, VL);
    }

  }
  SET_CS;
}

void LCD_DrawPoint(uint16_t x, uint16_t y)
{
  Address_set(x, y, x, y);
  LCD_Write_DATA(POINT_COLOR >> 8, POINT_COLOR);
}

void draw_image(uint16_t x, uint16_t y, int r, int g, int b)
{
  Address_set(x, y, x, y);
  LCD_Write_DATA((r & 0xF8) | (g >> 5), (b >> 3) | ((g >> 2)) << 5);
}

void draw_image565(uint16_t x, uint16_t y, uint16_t data)
{
  Address_set(x, y, x, y);
  LCD_Write_DATA(data >> 8, data & 0xFF);
}

//void LCD_DrawPoint_big(uint16_t x,uint16_t y)
//{
//  LCD_Fill(x-1,y-1,x+1,y+1,POINT_COLOR);
//}



void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
  uint16_t i, j;
  Address_set(xsta, ysta, xend, yend);

  for (i = ysta; i < yend; i++)
  {
    for (j = xsta; j < xend; j++)
    {
      LCD_Write_DATA(color >> 8, color);

    }
  }

}




void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;

  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)incx = 1;
  else if (delta_x == 0)incx = 0;
  else {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)incy = 1;
  else if (delta_y == 0)incy = 0;
  else {
    incy = -1;
    delta_y = -delta_y;
  }
  if ( delta_x > delta_y)distance = delta_x;
  else distance = delta_y;
  for (t = 0; t <= distance + 1; t++ )
  {
    LCD_DrawPoint(uRow, uCol);
    xerr += delta_x ;
    yerr += delta_y ;
    if (xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  LCD_DrawLine(x1, y1, x2, y1);
  LCD_DrawLine(x1, y1, x1, y2);
  LCD_DrawLine(x1, y2, x2, y2);
  LCD_DrawLine(x2, y1, x2, y2);
}


void Draw_Circle(uint16_t x0, uint16_t y0, uint16_t r)
{
  int a, b;
  int di;
  a = 0; b = r;
  di = 3 - (r << 1);
  while (a <= b)
  {
    LCD_DrawPoint(x0 - b, y0 - a);        //3
    LCD_DrawPoint(x0 + b, y0 - a);        //0
    LCD_DrawPoint(x0 - a, y0 + b);        //1
    LCD_DrawPoint(x0 - b, y0 - a);        //7
    LCD_DrawPoint(x0 - a, y0 - b);        //2
    LCD_DrawPoint(x0 + b, y0 + a);        //4
    LCD_DrawPoint(x0 + a, y0 - b);        //5
    LCD_DrawPoint(x0 + a, y0 + b);        //6
    LCD_DrawPoint(x0 - b, y0 + a);
    a++;

    if (di < 0)di += 4 * a + 6;
    else
    {
      di += 10 + 4 * (a - b);
      b--;
    }
    LCD_DrawPoint(x0 + a, y0 + b);
  }
}





void draw_char(uint16_t x, uint16_t y, char text)
{
  text -= 32;
  int i;
  for (i = 0; i < 16; i++)
  {
#ifdef __AVR__
    char ch = pgm_read_byte(&font16x8[(int)text][i]);
#else
    char ch =                font16x8[(int)text][i];
#endif

    int j;
    for (j = 0; j < 8; j++)
    {
      if (ch & (1 << j))
      {
        LCD_DrawPoint(x + j + 2, y + i + 2);

      }
    }
  }
}


void draw_text(uint16_t x, uint16_t y, const char *text)
{
  while (*text)
  {
    draw_char(x, y, *text);
    x += 8;
    text++;
  }

}


void draw_number(uint16_t x, uint16_t y, int16_t number, uint16_t length, uint16_t pad)
{

  char s[10];
  sprintf(s, "%d", number);
  int len = strlen(s);

  if (length < len) {
    int i;
    for (i = 0; i < length; i++) {
      draw_char(x, y, '*');
      x += 6;
    }
    return;
  }
  int i;
  for (i = 0; i < length - len; i++) {
    draw_char(x, y, pad);
    x += 6;
  }
  draw_text(x, y, (char*)s);

}


int nDigits(int i)
{
  if (i < 0) i = -i;
  if (i <         10) return 1;
  if (i <        100) return 2;
  if (i <       1000) return 3;
  if (i <      10000) return 4;
  if (i <     100000) return 5;
  if (i <    1000000) return 6;
  if (i <   10000000) return 7;
  if (i <  100000000) return 8;
  if (i < 1000000000) return 9;
  return 10;
}
void setup() {

  
  LCD_Init();   // Inicializa la pantalla TFT
  Pant(0x07,0xE0);
  draw_text(10,150,"Bienvenido NoMADA TFT Touch");    // Escribimos texto en la pantalla
  draw_text(30,250,"Toque la pantalla para");
  draw_text(70,265,"continuar");
}

void loop() {
  // put your main code here, to run repeatedly:

}
