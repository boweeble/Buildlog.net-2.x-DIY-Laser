/*
  Date Written: February 7, 2015
  Written By  : Scott E. Willson (boweeble@gmail.com)
  Description : Supporting library of sorts to display various components
                on the Adafruit's 2.8 TFT touch screen.
*/
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
//#include <Adafruit_STMPE610.h>
#include "Config.h"
#include "Sound.h"

#define BUTTON   0
#define TEXTBOX  1
#define CHECKBOX 2
#define OPTION   3

#define BUTTON_RADIUS 5

#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000
#define TS_MINZ 100

//=====================COLOR_PALLET==========================
#define BLACK         0x0
#define LIGHT_RED     0xFD14
#define RED           0xF800
#define DARK_RED      0x6000
#define LIGHT_ORANGE  0xFF52
#define ORANGE        0xFD00
#define DARK_ORANGE   0xFA80
#define LIGHT_YELLOW  0xFFF4
#define YELLOW        0xD7E0
#define DARK_YELLOW   0xCE40
#define LIGHT_GREEN   0xB7F6
#define GREEN         0x07E0
#define DARK_GREEN    0x0320
#define LIGHT_BLUE    0xA51F
#define BLUE          0x001F
#define DARK_BLUE     0x000C
#define PURPLE        0xF81F
#define CYAN          0x07FF
#define GREY          0x8410
#define WHITE         0xFFFF

// Draw various components on the screen such as buttons, textboxes, checkboxes and radio button options
void drawComponent(Adafruit_ILI9341 tft, int x, int y, int width, int height, int ctype, char* label, double value) {
  
  tft.setRotation(1);
  
  // What type of component shall we draw?
  switch (ctype) {
    case BUTTON:
      tft.drawRoundRect(x, y, width, height, BUTTON_RADIUS, WHITE);
      if (value > 0) {
        tft.fillRoundRect(x, y, width, height, BUTTON_RADIUS, WHITE);
        tft.setTextColor(BLACK);
      } else {
        tft.fillRoundRect(x+1, y+1, width-2, height-2, BUTTON_RADIUS, BLACK);
        tft.setTextColor(WHITE);
      }
      
      tft.setCursor(x + ((width - (strlen(label) * 12)) / 2), y + (height/2)-6);
      tft.print(label);   
      break;
      
    case TEXTBOX:
      tft.setTextColor(WHITE);
      tft.setCursor(x, y+5);
      tft.println(label);
      tft.fillRect(x+(strlen(label) * 12)+10,y, width, height, GREY);
      tft.setTextColor(BLACK);
      tft.setCursor(x+(strlen(label) * 12)+15, y+5);
      tft.println(value,1);
      break;
      
    case CHECKBOX:
      // Checkbox
      tft.setTextColor(WHITE);
      tft.setCursor(x, y+5);
      tft.println(label);
      tft.fillRect(x+(strlen(label) * 12)+10, y, width, height, BLACK);
      tft.drawRect(x+(strlen(label) * 12)+10, y, width, height, GREY);
      if(value > 0) {
        tft.drawLine(x+(strlen(label) * 12)+10, y, x+(strlen(label) * 12)+width+9, y+height-1, GREY);
        tft.drawLine(x+(strlen(label) * 12)+10, y+height-1, x+(strlen(label) * 12)+width+9, y, GREY);
      }
      break;

    case OPTION:
      // Option
      tft.fillCircle(x,y,(height/2),BLACK);
      tft.drawCircle(x,y,(height/2),GREY);
      if(value > 0) { tft.fillCircle(x,y,(height/4),GREY); }
      tft.setCursor(x+width,y-5);
      tft.setTextColor(WHITE);
      tft.print(label);
      break;
      
  }
  
}

// Which component did the user tap?
int whichComponent(Adafruit_STMPE610 ts, Adafruit_ILI9341 tft, int components[], char *labels[], int *xyz, int noCmp) {
  TS_Point coord = ts.getPoint();
  xyz[0] = 0;
  xyz[1] = 0;
  xyz[2] = 0;

  if (!ts.bufferEmpty()) {   
    //Serial.print("Start: ");
  
    // Scale using the calibration #'s
    // and rotate coordinate system
    coord.x = map(coord.x, TS_MINY, TS_MAXY, 0, tft.height());
    coord.y = map(coord.y, TS_MINX, TS_MAXX, 0, tft.width());
  
    int y = tft.height() - coord.x;
    int x = coord.y;
    int z = coord.z;
    
    if (ts.touched() && z < TS_MINZ) {
      xyz[0] = x;
      xyz[1] = y;
      xyz[2] = 1;
      // Check each button. Start with X
      for (int i = 0; i < noCmp; i++){
        int ii = (i * 5);
        int minX = components[ii];
        int maxX = minX + components[ii+2];
        int minY = components[ii+1];
        int maxY = minY + components[ii+3];
        int cType = components[ii+4];
        int labelSize = strlen(labels[i]) * 12;
        if (cType == CHECKBOX || cType == TEXTBOX) {
          maxX = minX + labelSize + 10 + components[ii+2];
        }
        if (cType == OPTION) {
          minX = components[ii] - components[ii+2];
          maxX = minX + components[ii+2] + labelSize;
        }
        //Serial.print(cType); Serial.print(": "); Serial.print(minX); Serial.print(", "); Serial.print(maxX); Serial.print(", "); Serial.print(minY); Serial.print(", "); Serial.println(maxY); 
        if (x >= minX && x <= maxX) {
          if (y >= minY && y <= maxY) {
            //Serial.print("HIT!! ");Serial.println(i);
            createSound(NOTE_D8, 125);
            return i;
          }
        }
      }
    }
  }
  ts.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
  return -1;
  
}

// Show a Keypad for numerical user input 4 digits with 1 assumed decimal (i.e. ###.#)
void showKeypad(Adafruit_ILI9341 tft, char* setting, double value, int *digits, int *btns, char *lbls[]) {
  int leftPos = 25;
  int topPos = 5;
  int bSize = 50;
   tft.fillRoundRect(leftPos,topPos,295,230,BUTTON_RADIUS, BLACK);
   tft.drawRoundRect(leftPos,topPos,295,230,BUTTON_RADIUS, WHITE);
   for (int i=0; i<13; i++) {
     int x = (i * 5);
     drawComponent(tft, btns[x], btns[x+1], btns[x+2], btns[x+3], btns[x+4], lbls[i], 0);
   }
  tft.setRotation(0);
  tft.setCursor(5, 3);
  tft.fillRect(0, 0, 240, 22, WHITE);
  tft.setTextColor(BLACK);
  tft.print(setting);
  tft.setRotation(1);
  
  tft.setTextSize(4);
  tft.setTextColor(WHITE);
  digits[0] = int(value / 100);
  digits[1] = int((value - (digits[0] * 100)) / 10);
  digits[2] = int(value - ((digits[0] * 100) + digits[1] * 10));
  digits[3] = int((value - ((digits[0] * 100) + (digits[1] * 10) + digits[2])) * 10);
  
  tft.setCursor(30, 20);
  tft.print(digits[0]);
  tft.print(digits[1]);
  tft.print(digits[2]);
  tft.print(".");
  tft.print(digits[3]);
  digits[4] = 0;
  tft.setTextSize(2);
  tft.drawFastHLine(30, 55, 20, WHITE);
}

// Update the user entered data
void updateKeypad(Adafruit_ILI9341 tft, int *digits) {
  tft.fillRect(30,20,125,40,BLACK);
  tft.setTextColor(WHITE);
  tft.setCursor(30, 20);
  tft.setTextSize(4);
  tft.print(digits[0]);
  tft.print(digits[1]);
  tft.print(digits[2]);
  tft.print(".");
  tft.print(digits[3]);
  tft.setTextSize(2);
  int x = 30+(digits[4]*24);
  if(digits[4] == 3) { x = x+24; }
  tft.drawFastHLine(x, 55, 20, WHITE);  // Show the user which location we're editing
  tft.setTextSize(2);
}

