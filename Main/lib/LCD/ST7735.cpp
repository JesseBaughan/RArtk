#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library for
#include "ST7735.h"    //Header file for all display setup
// #include "RiggerAssist.h"

#define LED_P 27
#define LED_N 26

//uninitalised pointers to SPI objects
//SPIClass * vspi = NULL;
//SPIClass * hspi = NULL;     // Default for Arduino if SPI used
// For 1.44" and 1.8" TFT with ST7735 use, use SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
//Adafruit_ST7735 tft = Adafruit_ST7735(23, OLED_CLK, TFT_DC, -1, OLED_CS);

static const int spiClk = 1000000; // 1 MHz

const int MARKERSIZE = 7;         // radius of circle encompassing the triange
const int MARKERCOLOR = ST77XX_WHITE;

// Setup for the dial
const int CX = 80;                // Centre of dial circle
const int CY = -15;
const int RADIUS = 65;            // radius of dial circle
const float RAD = PI / 180;

// array for old triangle position
int oldx1 = 0;
int oldx2 = 0;
int oldx3 = 0;
int oldy1 = 0;
int oldy2 = 0;
int oldy3 = 0;

int recvCnt = 0;

float oldVal = 0;                 // Save old dial value for animation
float offsetAngle = -999;           // Reference angle

// String buffers
char s1[4];                       // Main numeral clear buffer (include extra space for nullchar)
char s2[2];                       // Radio strength clear buffer
char s3[4];                       // Capacity clear buffer

bool LoRaActive;                  // Flag for LoRa active


//Constructor for the Display class
ST7735::ST7735() {}

void ST7735::begin() {
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(TFT_LED, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, TFT_MAX_BRIGHT);                   // LED brightness 0 to 255
  tft.initR(INITR_MINI160x80);                    // initialize a ST7735S chip, mini display
  // B10100000 in setRotation source for case 1 when using with internal mirror, else B11100000 with no internal mirror
  //  writedata(B11100000); for setRotation(1) for pentaprism
  tft.setRotation(1);                             // NOTE: Adafruit_ST7735.cpp::setRotation modified so that case 1 sets MADCTL to B11100000 (mirror X,Y and row/col exchanged). See https://github.com/sumotoy/TFT_ST7735/blob/master/TFT_ST7735.cpp
  
  //Do startup display jobs
  ShowSplashScreen();
  ShowBackgroundImg();
  DispLORAIndicator();
  PlayMarkerAnimation();
}

// Turns on or off the indicator LED
void ST7735::blueLED(bool state) {
  digitalWrite(LED_P, state);
  digitalWrite(LED_N, !state);
}

void ST7735::ShowSplashScreen() {  
  tft.fillScreen(ST77XX_BLACK); // Clear screen              
  tft.drawRGBBitmap(50, 10, splash, 57, 60);      // Draw DataHead splash
  tft.setFont(&FreeSansBold18pt7b);
  blueLED(LOW);
  delay(1500);                                    // Time to show splash
}

void ST7735::ShowBackgroundImg() {
  tft.fillScreen(ST77XX_BLACK);                   // Clear screen
  tft.fillScreen(ST77XX_BLACK);                   // Clear screen (needed twice for some reason...)
  tft.drawRGBBitmap(0, 20, black, 160, 80);       // Draw background
  drawMarker(0, MARKERCOLOR);                     // Marker at 0
}

void ST7735::DispLORAIndicator() {
  // radioCh(-1);
  if (LoRaActive) {
    radioStrength(-1);
  } else {
    radioStrength(3);                             // Not active
  }
  // capacity(78);
  // load(6.8);
}

void ST7735::PlayMarkerAnimation() {
  moveMarker(7.00);
  delay(100);
  moveMarker(-7.00);
  delay(100);
  moveMarker(0);
}

//Used for dimming of LCD based on ambient light


// Show filled box for the selected channel, used in LORA indicator
void ST7735::battBars(int battLvl) {
  tft.fillRect(0, 0, 20, 8, ST77XX_BLACK);            // clear old text
  tft.drawRect(2, 0, 20, 8, ST77XX_GREEN);             // Draw batery outline
  tft.drawRect(0, 2, 2, 4, ST77XX_GREEN);             // Draw +ve end

  if (battLvl == 3) {
    tft.fillRect(4, 2, 4, 4, ST77XX_GREEN);   // Far left bar
    tft.fillRect(10, 2, 4, 4, ST77XX_GREEN);   //middle bar
    tft.fillRect(16, 2, 4, 4, ST77XX_GREEN);   //far right bar
  }
  else if (battLvl ==2) {
    tft.fillRect(10, 2, 4, 4, ST77XX_GREEN);   
    tft.fillRect(16, 2, 4, 4, ST77XX_GREEN);  
  }
  else if (battLvl ==1) {
    tft.fillRect(16, 2, 4, 4, ST77XX_GREEN);   
  }
  // tft.fillRect(0, 6 * (battLvl - 1), 4, 4, ST77XX_CYAN);   // Selected channel
}

// Render a signal strength icon from 0 - 10, need to set new threshold values
void ST7735::radioStrength(int str) {
  int bot = ST7735_GREY;
  int mid = ST7735_GREY;
  int top = ST7735_GREY;
  if (str > 7)
    top = ST7735_GREEN;
  if (str > 4)
    mid = ST7735_GREEN;
  if (str > 2)
    bot = ST7735_GREEN;
  if (str < 0) {
    top = ST7735_BLUE;                      // No radio active, display in red (blue is actually red...)
    mid = ST7735_BLUE;
    bot = ST7735_BLUE;
  }
  drawArc(-2.7, -0.5, 154, 7, 5, top);
  drawArc(-2.5, -0.7, 154, 9, 3.5, mid);
  drawArc(-3, 3, 154, 11, 1.5, bot);
}

//What is capacity - batt percentage
void ST7735::capacity(int cap) {
  tft.setFont(&FreeSans9pt7b);
  sprintf(s2, "%i%%", cap);
  tft.fillRect(12, 0, 37, 14, ST77XX_BLACK);      // clear old text
  drawText(s2, 1, 12, 13, ST77XX_CYAN);
  tft.setFont(&FreeSansBold18pt7b);
}

//Load what?
void ST7735::load(float cap) {
  tft.setFont(&FreeSans9pt7b);
  cap = cap + 0.05;
  sprintf(s2, "%1.1f", cap);                  //TODO: Cater for 0 - 10, 10 - 100
  tft.fillRect(109, 0, 36, 14, ST77XX_BLACK);         // clear old text
  drawText(s2, 1, 109, 13, ST77XX_CYAN);
  tft.setFont(&FreeSansBold18pt7b);
}

// Low battery indicator
void ST7735::lowBatt(bool on) {
  tft.setFont();
  int color = ST77XX_BLACK;
  if (on) {
    color = ST77XX_BLUE;
  }
  drawText("LOW BATT", 1, 56, 0, color);
  tft.setFont(&FreeSansBold18pt7b);
}

// Draw the main text
void ST7735::drawText(char *text, uint16_t txtSize, uint16_t x, uint16_t y, uint16_t color) {
  tft.setTextWrap(false);
  tft.setTextSize(txtSize);
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.print(text);
}

// Animate the movement of the marker from existing value to new value
void ST7735::moveMarker(float newVal) {
  if (newVal < 8 && newVal > -8) {
    int dur = 140;                        //duration in cycles = number of possible pixel points, max 138
    float num = newVal * 8.6;              // 7 on the screen is 60 degrees and ignore negatives
    if (num != oldVal) {
      //Serial.println(String(num) + " " + String(oldVal));
      int newValPositive = abs(newVal * 10); //ABS only takes int vals so take abs as int then convert back to float
      float dispVal = (float)newValPositive / 10;          // round up
      
      if (dispVal >= 10) {
        sprintf(s1, "%1.0f", dispVal);
      } else {
        sprintf(s1, "%1.1f", dispVal);
      }

      tft.fillRect(55, 11, 49, 27, ST77XX_BLACK);         // clear old text
      drawText(s1, 1, 55, 36, ST77XX_WHITE);
      
      //Serial.println("Starting moving marker from " + String(oldVal) + " to " + String(newVal));
      for (int pos=0; pos<dur; pos++) {
        drawMarker(easeInOutQuad(pos, oldVal, num - oldVal, dur), MARKERCOLOR);
        delay(2);               // Allow persistence to stop flicker
        //TODO: Flicker at the start and end of the movement....
      }
      oldVal = num;
    }
  }
}

// full range is -60 (x = 18) to 60 (x = 136 pixels)
// Draw the marker on the dial circumference and rotate triangle based on angle
void ST7735::drawMarker(int angle, int color) {
  if (angle > 60)
    angle = 60;
  if (angle < -60)
    angle = -60;
  float radAngle = RAD * (angle + 90);
  float radAngle1 = RAD * (angle + 210);
  float radAngle2 = RAD * (angle + 330);
  float tempCos = cos(radAngle);
  float tempSin = sin(radAngle);
  int x = CX + RADIUS * tempCos;                  // Centre of triangle on dial circle curcumference
  int y = CY + RADIUS * tempSin;
  //Serial.println("x:" + String(x) + " y:" + String(y));

  int x1 = x + MARKERSIZE * tempCos;              // Rotate triange vertixes based on angle of rotation (using radius of MARKERSIZE)
  int y1 = y + MARKERSIZE * tempSin;
  //Serial.println("x1:" + String(x1) + " y1:" + String(y1));

  int x2 = x + MARKERSIZE * cos(radAngle1);
  int y2 = y + MARKERSIZE * sin(radAngle1);
  //Serial.println("x2:" + String(x2) + " y2:" + String(y2));
  
  int x3 = x + MARKERSIZE * cos(radAngle2);
  int y3 = y + MARKERSIZE * sin(radAngle2);
  //Serial.println("x3:" + String(x3) + " y3:" + String(y3));

  tft.fillTriangle(oldx1, oldy1, oldx2, oldy2, oldx3, oldy3, ST77XX_BLACK);     // Delete old value
  tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  oldx1 = x1;
  oldx2 = x2;
  oldx3 = x3;
  oldy1 = y1;
  oldy2 = y2;
  oldy3 = y3;
}

void ST7735::CalibrationInProgress() {
  sprintf(s1, "%s", "cal");
  tft.fillRect(55, 11, 49, 27, ST77XX_BLACK);         // clear old text
  drawText(s1, 1, 55, 36, ST77XX_WHITE);  
}

void ST7735::NotCalibrated() {
  sprintf(s1, "%s", "=>");
  tft.fillRect(55, 11, 49, 27, ST77XX_BLACK);         // clear old text
  drawText(s1, 1, 55, 36, ST77XX_WHITE);  
}

void ST7735::NoSignal() {
  sprintf(s1, "%s", " ---");
  tft.fillRect(55, 11, 49, 27, ST77XX_BLACK);         // clear old text
  drawText(s1, 1, 55, 36, ST77XX_WHITE);  
}

// quadratic easing in/out - acceleration until halfway, then deceleration
// MIT.BSD license for this function, terms http://robertpenner.com/easing_terms_of_use.html
// t: current time, b: beginning value, c: change in value, d: duration
float ST7735::easeInOutQuad (float t, float b, float c, float d) {
 if ((t/=d/2) < 1) return c/2*t*t + b;
 return -c/2 * ((--t)*(t-2) - 1) + b;
}

// Draw an arc for the radio reception
void ST7735::drawArc (float start_angle, float end_angle, int x, int y, int r, int color) {
 for (float i = start_angle; i < end_angle; i = i + 0.05)
 {
   tft.drawPixel(x + cos(i) * r, y + sin(i) * r, color);
   //Serial.println("x: " + String(x + cos(i) * r) + " y: " + String(y + sin(i) * r));
 }
}


