#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>

#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin
/*
 * 
 * Frecuencia de las cuerdas de la guitarra (Traste 0) 
1ra (E): 329,63 Hz 
2da (B): 246,94 Hz 
3ra (G): 196,00 Hz 
4ta (D): 146,83 Hz 11 

5ta (A): 110,00 Hz 
6ta (E): 82,41 Hz 

 */
// Original values
//#define TS_MINX 150
//#define TS_MINY 120
//#define TS_MAXX 920
//#define TS_MAXY 940

// Calibrate values
#define TS_MINX 116
#define TS_MINY 115
#define TS_MAXX 941
#define TS_MAXY 952
// Definimos la presion máxima y minima que podemos realizar sobre el panel
#define MINPRESSURE 1
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint p;
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
// Color definitions
#define BLACK   0xFFFF
#define BLUE    0xFFE0 //0x001F
#define RED     0x07FF
#define GREEN   0xF81F
#define CYAN    0xF800
#define MAGENTA 0x07E0//0xF81F
#define YELLOW  0x001F
#define WHITE   0
#define GREY    0x2108
// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define BOXSIZE 40
#define PENRADIUS 3


void showValue (uint16_t value) {

  if (value < 10)
    tft.print("00");
  if (value < 100)
    tft.print("0");
    
  tft.print(value);
  tft.print("  ");
}

void setup() {
  // put your setup code here, to run once:
  tft.reset();
  tft.begin(0x9341); // SDFP5408
  tft.fillScreen(BLACK);
  tft.setRotation(3); // Need for the Mega, please changed for your choice or rotation initial
  //tft.drawRect(90,105,1,10, RED); // Dibujamos un "boton"
;    // Especificamos el tamaño del texto

  int x,y;
  do{
     // Si la pulsación del eje X se produce entre los puntos 40 y 160
    // Y la pulsacion del eje Y se produce entre los puntos 20 y 60
    // Y la presión realizada esta entre el margen determinado
    do{
          readTouch();

      } while((p.z < MINPRESSURE )|| (p.z > MAXPRESSURE));
     //tft.fillScreen(BLACK);
       readTouch();
      tft.setTextSize(2);
     tft.setTextColor(WHITE,BLACK);
      tft.setCursor(2, 1);
    tft.print("X: ");
    showValue(p.y);
  
  tft.setCursor(100,1);
  tft.print("Y: ");
  showValue(p.x);

  delay(100);
  x = p.y;
  y = p.x;
  if(( x >= 100 && x <= 120 ) && (y >= 120 && y <= 160)) break;
  }while(1);
  tft.fillScreen(BLACK);

 /* for (int j = 0; j < 320; j += 40) {
    for (int i = 0; i < 220; i += 18) {
      tft.setCursor (j, i);
      tft.setTextSize (2);
      tft.setTextColor (WHITE, BLACK);
      tft.print (i);
      delay(25);
    }
  }
  */
  //Design Interface (lines)
    tft.fillRect(0,197,320,4,BLUE);
    tft.fillRect(0,236,320,4,BLUE);
    tft.fillRect(217,0,4,240,BLUE);
    tft.fillRect(217,98,320,4,BLUE);
     

}
int val;
void loop() {
  // put your main code here, to run repeatedly:
    int xpos = 0, ypos = 5, gap = 4, radius = 52;
    // Draw a large meter
    xpos = 320/2 - 160, ypos = 0, gap = 100, radius = 105;
    ringMeter(millis()/100,0,600, xpos,ypos,radius,"Hertz",RED2GREEN); // Draw analogue meter
}
void readTouch()
{
   p = ts.getPoint(); // Realizamos lectura de las coordenadas
    pinMode(XM, OUTPUT); 
    pinMode(YP, OUTPUT);
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
    // *** SPFD5408 change -- End
    p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());;
  
}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  
  x += r; y += r;   // Calculate coords of centre of ring
  int w = r / 3;    // Width of outer ring is 1/4 of radius 
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
  // Variable to save "value" text colour from scheme and set default
  int colour = BLUE;
 
  // Draw colour blocks every inc degrees
  for (int i = -angle+inc/2; i < angle-inc/2; i += inc) {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = RED; break; // Fixed colour
        case 1: colour = GREEN; break; // Fixed colour
        case 2: colour = BLUE; break; // Fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
        case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
        default: colour = BLUE; break; // Fixed colour
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      //text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, GREY);
    }
  }
  // Convert value to a string
  char buf[10];
  byte len = 2;
  if (value > 999) len = 4;
  else if(value > 99) len = 3;
  dtostrf(value, len, 0, buf);
  buf[len] = ' '; buf[len] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);
  colour = GREEN;
  if(value>9 && value < 99){
  tft.setTextColor(colour,BLACK);
  tft.setCursor(x-25,y-10);tft.setTextSize(5);
  tft.print(buf);
  }else if(value > 99){
  tft.setTextColor(colour,BLACK);
  tft.setCursor(x-40,y-10);tft.setTextSize(5);
  tft.print(buf);
  }
  if(value<10){
  tft.setTextColor(colour,BLACK);
  tft.setCursor(x-25,y-10);tft.setTextSize(5);
  tft.print(buf);
  }

  tft.setTextColor(CYAN,BLACK);
  
  tft.setCursor(x-25,y+70);tft.setTextSize(2);
  tft.print(units); // Units display
  
  // Calculate and return right hand side x coordinate
  return x + r;
}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits
  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}

// #########################################################################
// Return a value in range -1 to +1 for a given phase angle in degrees
// #########################################################################
float sineWave(int phase) {
  return sin(phase * 0.0174532925);
}



