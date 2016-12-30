#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF

#define __CS A3
#define __DC A1


TFT_ILI9163C display = TFT_ILI9163C(__CS,A0,__DC);

void setup() {
init_tft();
splashScreen();
}

void loop(void) {

//(BLACK,GREEN);
}


void makeText(String msg,uint16_t color,uint8_t size){
  display.setTextColor(color);  
  display.setTextSize(size);
  display.print(msg);
}
void init_tft(){
  display.begin();
  display.fillScreen(BLACK);
  display.display(true);
  display.setRotation(0);
  display.clearScreen();
  display.setCursor(0,0);
}
void splashScreen(){
display.drawRect((display.width()-1)/2 -(display.width()-1)/2, (display.height()-1)/2 -(display.height()-1)/2 , (display.height()-1), (display.height()-1), GREEN);
display.setCursor(0,37);
makeText(" HAMMURABI \n",WHITE,2);
makeText("    2.0 \n",WHITE,2);
display.setCursor(0,105);
makeText("      Powered by: \n",RED,1);
makeText("    Michael Vargas",RED ,1);
}


