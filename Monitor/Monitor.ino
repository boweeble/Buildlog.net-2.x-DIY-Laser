/*
  Date Written: February 2015
  Written By  : Scott E. Willson (boweeble@gmail.com)
                My Github: 
                My Build : http://www.buildlog.net/forum/viewtopic.php?f=16&t=2650
                
                Pieces taken from and initial idea of Scott Shwarts
                Scott Shwarts Github: https://github.com/sshwarts/
                
  Description : This sketch was put together to monitor my BuildLog 2.x laser build
                Wiki: http://www.buildlog.net/wiki/doku.php?id=2x:2.x_laser

                There's lots more that can be done to optimize and cleaned up here but this is a quick & dirty start
                TODO: Create a class for the various components (i.e. Screen, Memory, etc...)
                Still need to accurately determine the fire time of the laser utilizing the interrups.
*/
// Laser Controller
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>

#include <SD.h>
#include <SimpleTimer.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Config.h"
#include "Screen.h"
#include "Memory.h"
//#include "Sound.h"

// useful defines
#define OFF false;
#define ON  true;

Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Main Screen Buttons
static int buttons[25] = {25, 0, 100, 46, BUTTON,
                            25, 48, 100, 46, BUTTON,
                            25, 96, 100, 46, BUTTON,
                            25, 145, 100, 46, BUTTON,
                            25, 194, 100, 46, BUTTON};
static char* buttonText[]={"PUMP", "EXHAUST", "AIR", "AUX", "ENABLE"};                            
boolean buttonState[5] = {false, false, false, false, false};

// Settings Screen Buttons
static int sCmp[40] = {215, 190, 100, 45, BUTTON,
                      30, 10, 25, 25, CHECKBOX,
                      30, 45, 75, 25, TEXTBOX,
                      30, 80, 75, 25, TEXTBOX,
                      42, 125, 24, 24, OPTION,
                      180, 125, 24, 24, OPTION,
                      42, 160, 24, 24, OPTION,
                      180, 160, 24, 24, OPTION};
static char* sCmpText[] = {"DONE", "Cover Disables :", "Min Water Flow :", "Max Inlet Temp :", "Liters", "Gallons", "Celcius", "Farenheit"};                            
static float sCmpVal[] = {0, 1, 2.0, 79.5, 1, 0, 0, 1};

// Keypad Buttons
static int keypadCmp[60] = {155, 10, 50, 50, BUTTON,
                      210, 10, 50, 50, BUTTON,
                      265, 10, 50, 50, BUTTON,
                      155, 65, 50, 50, BUTTON,
                      210, 65, 50, 50, BUTTON,
                      265, 65, 50, 50, BUTTON,
                      155, 120, 50, 50, BUTTON,
                      210, 120, 50, 50, BUTTON,
                      265, 120, 50, 50, BUTTON,
                      155, 175, 50, 50, BUTTON,
                      210, 175, 50, 50, BUTTON,
                      265, 175, 50, 50, BUTTON};
static char* keypadText[] = {"7", "8", "9", "4", "5", "6", "1", "2", "3", "0", "<-", "OK"};
static int editField = 0;
static int xyz[3] = {0,0,0};

// Global Variables
static int screenNo = 0;    // Which screen is being displayed?

boolean Firing = false;
volatile unsigned long firstPulseTime;
volatile unsigned long lastPulseTime;
volatile unsigned long numPulses;

boolean Enabled = false;
byte lastPressed = 0;
unsigned long pressedTime;
unsigned long fireTime = 0;
unsigned long accFireTime = 0;
unsigned long totalFireTime = 0;
byte lifeTime[4] = {0,0,0,0};
byte tickCount = 0;
volatile int flowSensorTicks = 0;   //measuring the rising edges of the signal for the Water Flow Sensor
float flowRate = 0;

// Used to display messages on the screen
static char* errors1[] = {"COVER OPEN,\0", "LOW WATER FLOW\0", "HIGH INLET TEMP\0", "HIGH OUTLET TMP\0", "System Monitor 1.0\0","SD CARD FAILED\0"};
static char* errors2[] = {"CLOSE TO ENABLE\0", "DANGER TO LASER\0", "DANGER TO LASER\0", "DANGER TO LASER\0", "monitoring...\0", "TO INITIALIZE\0"};

static const int ioPins[5] = {PUMP, BLOWER, AIR, AUX, ENABLE_LASER};

// Location of our Sensor information for the main screen
static const int coverLabelPos[2]     = {135, 30};
static const int flowLabelPos[2]      = {135, 52};
static const int temp1LabelPos[2]     = {135,74};
static const int temp2LabelPos[2]     = {135,96};
static const int jobTimeLabelPos[2]   = {137, 145};
static const int totalTimeLabelPos[2] = {137, 167};

float prev_vals[4] = {9999,9999,9999,9999};    // Stores the pervious sensor values so we're only updating those that change
float prevInletTemps[5] = {0, 0, 0, 0, 0};    // Used to store the previous 5 inlet temperatures (to prevent erroneous readings)
float prevOutletTemps[5] = {0, 0, 0, 0, 0};   // Used to store the previous 5 outlet temperatures (to prevent erroneous readings)
int keypadDigits[5] = {0, 0, 0, 0, 0};        // Stores user input

boolean coverDisable = true;
boolean useCelcius = false;
boolean useLiters = true;
float maxInletTemp = 80;
float minFlowRate = 0.5;

SimpleTimer timer;
File dataFile;    // SD Card

// Script setup - initialize variables, pins & libraries
void setup(void) {  

  // Setup Display
  tft.begin();          // Startup the display
  if (!ts.begin()) {    // Startup the touchscreen
    Serial.println("Unable to start touchscreen.");
  } else { 
    Serial.println("Touchscreen started."); 
  }

  tft.setRotation(1);  // 320x240 display (horizontal)
  tft.setTextSize(2);

  Serial.begin(9600);
  randomSeed(analogRead(A4));

  // Setup Output Pins
  pinMode(SPKR, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(BLOWER, OUTPUT);
  pinMode(AIR, OUTPUT);
  pinMode(AUX, OUTPUT);
  pinMode(ENABLE_LASER, OUTPUT);
  pinMode(PAUSE_SW, OUTPUT);
  
  // Setup Input Pins
  pinMode(WATER_FLOW, INPUT);
  pinMode(COVER, INPUT);
  pinMode(FIRING, INPUT);

  // INTERRUPTS
  attachInterrupt(0, rpm, RISING);     //attach interrupt int.0 to WATER_FLOW (pin2) for the MEGA
  attachInterrupt(1, firing, FALLING); //attach interrupt int.1 to FIRING (pin3) for the MEGA

  // Set all outputs to 1 which is off
  // and enable the pull-ups on the Analog inputs
  digitalWrite(PUMP, HIGH);
  digitalWrite(BLOWER, HIGH);
  digitalWrite(AIR, HIGH);
  digitalWrite(AUX, HIGH);
  digitalWrite(ENABLE_LASER, HIGH);
  digitalWrite(PAUSE_SW,HIGH);
  digitalWrite(COVER, HIGH);
  
  digitalWrite(INLET_TEMP, LOW);
  digitalWrite(OUTLET_TEMP, LOW);

  
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
  }

  // Read the card for total laser time
  // Display a picture
  bmpDraw("startup.bmp", 0, 0);
  delay(2000);

  // Open the file for storing laser running time
  dataFile = SD.open("log.dat", FILE_WRITE);
  dataFile.seek(0);  // reset to beginning

  // Read the total run time
  byte index = 0;
  while (dataFile.available()) {
    lifeTime[index] = dataFile.read();
    Serial.println( lifeTime[index]);
    index++;
  }

  totalFireTime = ( ((unsigned long)lifeTime[0] << 24) 
                   + ((unsigned long)lifeTime[1] << 16) 
                   + ((unsigned long)lifeTime[2] << 8) 
                  + ((unsigned long)lifeTime[3] ) ) ;
 
  loadSettings();
  
  Serial.print(totalFireTime); Serial.println(" total fire time...");
  //tft.fillScreen(BLACK);

  // Draw Buttons
  drawScreen();
  showSensors();
  
  // Show total for first time
  tickCount = 60;
  updateTime();
    
  // Setup a timer so we can check for buttons and handle events
  timer.setInterval(100, process);	// Ever 100 milliseconds

  // Setup a timer so we can update the sensor status
  timer.setInterval(1000, showSensors);	// Refresh the sensor readings once a second

  // Setup another one to update time display
  timer.setInterval(1000, updateTime); // Every second
  
  
}

void loop() {
  timer.run();
}

void rpm () {
   //This is the function that the interupt calls
   //This function measures the rising and falling edge of the hall effect sensors signal
   flowSensorTicks++;
}

void firing(){
  // This function is used to calculate the firing time of the laser
  unsigned long now = micros();
  if (numPulses == 1) {
    firstPulseTime = now;
  } else {
    lastPulseTime = now;
  }
  ++numPulses;
}

float readFrequency() {
 //attachInterrupt(0, isr, RISING);    // enable the interrupt
 return (numPulses < 3) ? 0 : (1000000.0 * (float)(numPulses - 2))/(float)(lastPulseTime - firstPulseTime);
 numPulses = 0;                      // reset the numPulses to start a new reading
}
void process() {
  //analogWrite(7, 50);
  
  unsigned long time;

  if (digitalRead(FIRING) && buttonState[4]) {
    time = millis();

    if (fireTime > 0) {
      accFireTime += time - fireTime;
      fireTime = time;
    } else {
      fireTime = time;
    }
  } else {
    fireTime = 0;
  }
  
  byte index = 99;
  int x = 0;
  int y = 0;
  int z = 0;
  int offset = 0;
  int noCmp = 0;
  
  // We first need to determine which user screen we are displaying...
  switch (screenNo) {
    
    // MAIN SCREEN
    case 0:
    
      noCmp = (sizeof(buttons)/sizeof(int)) / 5;
      index = whichComponent(ts, tft, buttons, buttonText, xyz, noCmp);
  
      // Stores the position of the screen that was tapped
      x = xyz[0];
      y = xyz[1];
      z = xyz[2];
      
      // No key was pressed, make sure the user didn't tap the sensors
      // if so then display the sensor settings screen
      if (x >= 128 && y <= 116 && index == 255 && z > 0) {
        screenNo = 1;
        index = 100;
        showSettingsScreen();
      }    
  
      if (index < 99 && index != lastPressed) {
  
        lastPressed = index;
  
        // Log the press
        pressedTime = millis();
        
        if(index == 4) {
          showMsg(9999,false);
          if (!buttonState[0]) {
            buttonState[0] = true;
            digitalWrite(ioPins[0], buttonState[0]);
            drawComponent(tft, buttons[0], buttons[1], buttons[2], buttons[3], buttons[4], buttonText[0], buttonState[0]);
          }
        }
  
        digitalWrite(ioPins[index], buttonState[index]);  // Toggle the output
        buttonState[index] = !buttonState[index];
        int ii = (index * 5);
        drawComponent(tft, buttons[ii], buttons[ii+1], buttons[ii+2], buttons[ii+3], buttons[ii+4], buttonText[index], buttonState[index]);
      }
      break;
    
    // SETTINGS SCREEN
    case 1:
      // Check each button. Start with X
      noCmp = (sizeof(sCmp)/sizeof(int)) / 5;
      index = whichComponent(ts, tft, sCmp, sCmpText, xyz, noCmp);
  
      switch (index) {
        case 0:
          // Done Button
          screenNo = 0;
          drawScreen();
          for(int i = 0; i<4; i++) { prev_vals[i] = 9999; }  // reset sensor values to force refresh
          showSensors();
          tickCount = 60;
          updateTime();
          break;
        
        // Cover Disable
        case 1:
          if (sCmpVal[1] == 1) { sCmpVal[1] = 0; coverDisable = false; } else { sCmpVal[1] = 1; coverDisable = true; }
          offset = (1 * 5);
          drawComponent(tft, sCmp[offset], sCmp[offset+1], sCmp[offset+2], sCmp[offset+3], sCmp[offset+4], sCmpText[1], sCmpVal[1]);
          break;

        // Requires user input - show Keypad
        case 2:
        case 3:
          editField = index;  // Save which field are we editing so we know what to update
          screenNo = 2;
          showKeypad(tft, sCmpText[index], sCmpVal[index], keypadDigits, keypadCmp, keypadText);
          break;
          
        // Litres or Gallons
        case 4:
        case 5:
          if (index == 4) { sCmpVal[4] = 1; sCmpVal[5] = 0; useLiters = true; } else { sCmpVal[4] = 0; sCmpVal[5] = 1; useLiters = false; }
          offset = (4 * 5);
          drawComponent(tft, sCmp[offset], sCmp[offset+1], sCmp[offset+2], sCmp[offset+3], sCmp[offset+4], sCmpText[4], sCmpVal[4]);
          offset = (5 * 5);
          drawComponent(tft, sCmp[offset], sCmp[offset+1], sCmp[offset+2], sCmp[offset+3], sCmp[offset+4], sCmpText[5], sCmpVal[5]);
          break;
  
        // Celsius or Farenheit
        case 6:
        case 7:
          if (index == 6) { sCmpVal[6] = 1; sCmpVal[7] = 0; useCelcius = true; } else { sCmpVal[6] = 0; sCmpVal[7] = 1; useCelcius = false; }
          offset = (6 * 5);
          drawComponent(tft, sCmp[offset], sCmp[offset+1], sCmp[offset+2], sCmp[offset+3], sCmp[offset+4], sCmpText[6], sCmpVal[6]);
          offset = (7 * 5);
          drawComponent(tft, sCmp[offset], sCmp[offset+1], sCmp[offset+2], sCmp[offset+3], sCmp[offset+4], sCmpText[7], sCmpVal[7]);
          break;
  
      }
      break;
    
    // INPUT KEYPAD 
    case 2:
      
      noCmp = (sizeof(keypadCmp)/sizeof(int)) / 5;
      index = whichComponent(ts, tft, keypadCmp, keypadText, xyz, noCmp);
      if (xyz[2] == 1) {
        offset = index * 5;
        drawComponent(tft, keypadCmp[offset], keypadCmp[offset+1], keypadCmp[offset+2], keypadCmp[offset+3], keypadCmp[offset+4], keypadText[index], 1);
        
        // Which keypad button was pressed?
        switch(index) {
          case 0:keypadDigits[keypadDigits[4]] = 7; keypadDigits[4]++; break;
          case 1:keypadDigits[keypadDigits[4]] = 8; keypadDigits[4]++; break;
          case 2:keypadDigits[keypadDigits[4]] = 9; keypadDigits[4]++; break;
          case 3:keypadDigits[keypadDigits[4]] = 4; keypadDigits[4]++; break;
          case 4:keypadDigits[keypadDigits[4]] = 5; keypadDigits[4]++; break;
          case 5:keypadDigits[keypadDigits[4]] = 6; keypadDigits[4]++; break;
          case 6:keypadDigits[keypadDigits[4]] = 1; keypadDigits[4]++; break;
          case 7:keypadDigits[keypadDigits[4]] = 2; keypadDigits[4]++; break;
          case 8:keypadDigits[keypadDigits[4]] = 3; keypadDigits[4]++; break;
          case 9:keypadDigits[keypadDigits[4]] = 0; keypadDigits[4]++; break;
          case 10:keypadDigits[4]--; break;  // The backspace key was pressed
          case 11:
            // Keypad OK button pressed, update the field we're editing with the new value
            float newVal = keypadDigits[3];
            newVal = newVal / 10;
            newVal = newVal + (keypadDigits[0] * 100 + keypadDigits[1] * 10 + keypadDigits[2]);
            sCmpVal[editField] = newVal;
            if (editField == 2) { minFlowRate = newVal; }
            if (editField == 3) { maxInletTemp = newVal; }
            
            screenNo = 1;
            showSettingsScreen();
            break;
        }
        
        // We're only using 4 digits with 1 implied decimal - take account for that
        if(keypadDigits[4] < 0) { keypadDigits[4] = 0; }
        if(keypadDigits[4] > 3) { keypadDigits[4] = 3; }
        
        // Refresh the display
        if(index != 11) {
          updateKeypad(tft, keypadDigits);
          delay(50);
          drawComponent(tft, keypadCmp[offset], keypadCmp[offset+1], keypadCmp[offset+2], keypadCmp[offset+3], keypadCmp[offset+4], keypadText[index], 0);
        }
      }
      break;
  
  }

  lastPressed = index;
  if (millis() > pressedTime + 800) { lastPressed = 99; }	// We're allowed to press twice as long as enough time went by

}

// Something is wrong, let's disable the laser!!
void disableLaser(int errorNo){
  digitalWrite(ioPins[4], buttonState[4]);  // Toggle the output
  buttonState[4] = false;
  if (screenNo == 0) {
    int ii = (4 * 5);
    drawComponent(tft, buttons[ii], buttons[ii+1], buttons[ii+2], buttons[ii+3], buttons[ii+4], buttonText[4], buttonState[4]);
    showMsg(errorNo, true);
  }
}

/* Inputs ADC count from Thermistor and outputs Temperature in Celsius or Farenheit
 *  requires: include <math.h>
 * There is a huge amount of information on the web about using thermistors with the Arduino.
 * This version utilizes the Steinhart-Hart Thermistor Equation:
 *    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
 *   for the themistor in the Vernier TMP-BTA probe:
 *    A =0.00102119 , B = 0.000222468 and C = 1.33342E-7
 *    Using these values should get agreement within 1 degree C to the same probe used with one
 *    of the Vernier interfaces
 * 
 * Schematic:
 *   [Ground] -- [thermistor] -------- | -- [15,000 ohm resistor] --[Vcc (5v)]
 *                                     |
 *                                Analog Pin 0
 For the circuit above:
 * Resistance = ( Count*RawADC /(1024-Count))
 http://playground.arduino.cc/ComponentLib/Thermistor
 http://playground.arduino.cc/ComponentLib/Thermistor2
*/
float Thermistor(int RawADC) {
  float Temp;
  Temp = log(10000.0*((1024.0/RawADC-1))); // for pull-up configuration
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  if(!useCelcius) {
    Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  }
  if (Temp < 0) { Temp = 0; }
  return Temp;
}

// Draw the Main User Screen
void drawScreen() {
  tft.fillScreen(BLACK);
  
  tft.drawRoundRect(128, 0,192, 240, 5, WHITE);  // Sensor Box
  tft.fillRoundRect(128, 0, 192, 27, 5, WHITE);  // Sensor Title Box
  tft.fillRect(129, 22, 190, 25, BLACK);         // Only want the top edges rounded
  tft.setTextColor(BLACK);
  tft.setCursor(135,3);
  tft.println("Sensors");

  tft.setTextColor(WHITE);
  tft.setCursor(coverLabelPos[0] , coverLabelPos[1]);
  tft.print("Cover : ");
  tft.setCursor(flowLabelPos[0] , flowLabelPos[1]);
  tft.print("Water : ");
  tft.setCursor(temp1LabelPos[0] , temp1LabelPos[1]);
  tft.print("Inlet : ");
  tft.setCursor(temp2LabelPos[0] , temp2LabelPos[1]);
  tft.print("Outlet: ");

  tft.setCursor(jobTimeLabelPos[0], jobTimeLabelPos[1]);
  tft.println("Job  :");
  tft.setCursor(totalTimeLabelPos[0], totalTimeLabelPos[1]);
  tft.println("Total:");
  
  tft.fillRect(128, jobTimeLabelPos[1]-26, 234, 22, WHITE);
  //tft.fillRect(128, 168, 234, 22, WHITE);
  tft.setTextColor(BLACK);
  tft.setCursor(jobTimeLabelPos[0],jobTimeLabelPos[1]-22);
  tft.println("Laser Time");

  tft.drawFastHLine(128, jobTimeLabelPos[0]+50, 234, WHITE);

  // Draw Main Control Buttons
  for (int i = 0; i < 5; i++){
      int x = (i * 5);
      drawComponent(tft, buttons[x], buttons[x+1], buttons[x+2], buttons[x+3], buttons[x+4], buttonText[i], buttonState[i]);
      //drawBtn(buttons[x], buttons[x+1], buttons[x+2], buttons[x+3], buttonText[i], buttonState[i]);
  }
  
  // Show Banner
  tft.setRotation(0);
  tft.setCursor(5, 3);
  tft.fillRect(0, 0, 240, 22, WHITE);
  tft.setTextColor(BLACK);
  tft.print(errors1[4]);
  //tft.setRotation(1);

  //tft.setRotation(2);
  //bmpDraw("cog.bmp", 1, 1);
  tft.setRotation(1);

}

/* Show the Settings Screen which allows the user
   to change how the controller will function
*/
void showSettingsScreen() {
  tft.fillScreen(BLACK);
  // Draw Settings Controls
  tft.drawRoundRect(25, 0, 295, 183, BUTTON_RADIUS, WHITE);
  int noCmp = (sizeof(sCmp)/sizeof(int)) / 5;
  for (int i = 0; i < noCmp; i++){
      int x = (i * 5);
      drawComponent(tft, sCmp[x], sCmp[x+1], sCmp[x+2], sCmp[x+3], sCmp[x+4], sCmpText[i], sCmpVal[i]);
  }
  tft.drawRoundRect(27, 109, 291, 33, 12, GREY);
  tft.drawRoundRect(27, 144, 291, 33, 12, GREY);

  // Show Banner
  tft.setRotation(0);
  tft.setCursor(5, 3);
  tft.fillRect(0, 0, 240, 22, WHITE);
  tft.setTextColor(BLACK);
  tft.print("Settings Menu");
  tft.setRotation(1);

}

// Update the sensor information to the User
void showSensors() {
  
  float freq = readFrequency();
  /*tft.fillRect(129, 190, 190, 44, BLACK);
  tft.setCursor(135,195); tft.print("FREQ: "); tft.println(freq);
  tft.setCursor(135,217);*/
  
  int errorNo = 9999;
  float inputValue;
  
  // COVER - is it opened or closed?
  inputValue = digitalRead(COVER);
  if(inputValue != prev_vals[0]) {
    if(screenNo == 0) {  // Only display the values if we're in the main screen
      tft.fillRect(coverLabelPos[0]+85, coverLabelPos[1], 95, 22, BLACK);
      tft.setCursor(coverLabelPos[0]+85, coverLabelPos[1]);
      if (inputValue) {
        tft.setTextColor(RED);
        tft.print("OPEN");
      } else {
        tft.setTextColor(GREEN);
        tft.print("CLOSED");
      }
    }
    prev_vals[0] = inputValue;
  }
  // If we're set to disable the laser on cover open let's do it.
  if(inputValue && buttonState[4] && coverDisable) { errorNo = 0; }

  /* FLOW SENSOR
     Get the coolant flow rate.  Partial code taken from:
     http://www.seeedstudio.com/wiki/G1/2_Water_Flow_sensor
  */
  flowRate = (flowSensorTicks / 5.5);  //(Pulse frequency x 60) / 5.5Q, = flow rate
  // Does the user want to display Gallons or Litres?
  if(!useLiters) {
    flowRate = flowRate * 0.26417;                  // Convert to Gallons per minute
  }
  if (flowRate < 0) { flowRate = 0; }
  if(flowRate != prev_vals[1]) {
    if(screenNo == 0) {  // Only display the values if we're in the main screen
      tft.fillRect(flowLabelPos[0]+85, flowLabelPos[1], 95, 22, BLACK);
      tft.setCursor(flowLabelPos[0]+85, flowLabelPos[1]);

      tft.setTextColor(RED);
      if(flowRate > minFlowRate * 0.8) { tft.setTextColor(YELLOW); }
      if(flowRate >= minFlowRate) { tft.setTextColor(GREEN); }
      tft.print(flowRate,1);
      if(useLiters) {
        tft.print(" LPM");
      } else {
        tft.print(" GPM");
      }
    }
    prev_vals[1] = flowRate;
  }
  // Bad Flow Disable Laser
  if(flowRate < minFlowRate && buttonState[4]) { errorNo = 1; }

  /* INLET TEMPERATURE 
   Let's calculate the average temperature over the last 5 seconds
   to help counteract any erroneous readings */
  inputValue = int(Thermistor(analogRead(INLET_TEMP)));  // read the Thermistor
  
  // Let's calculate the average temperature over the last 5 seconds
  // to help counteract any erroneous readings
  float totTemps = 0;
  float avgTemp = 0;
  
  // Shift the new temp in and the oldest temp out...
  for(int tt=1;tt<5;tt++){
    totTemps = totTemps + prevInletTemps[tt];
    prevInletTemps[tt] = prevInletTemps[tt-1];
  }
  prevInletTemps[0] = inputValue;
  totTemps = totTemps + inputValue;
  avgTemp = totTemps / 5;
  
  // Display the inlet temperature
  if(avgTemp != prev_vals[2]) {
    if(screenNo == 0) {  // Only display the values if we're in the main screen
      tft.fillRect(temp1LabelPos[0]+85, temp1LabelPos[1], 95, 22, BLACK);
      tft.setCursor(temp1LabelPos[0]+85, temp1LabelPos[1]);
      tft.setTextColor(GREEN);                                         // Start with status initially ok
      if(avgTemp > maxInletTemp * 0.95) { tft.setTextColor(YELLOW); }  // Getting close to being over temp!!
      if(avgTemp > maxInletTemp) { tft.setTextColor(RED); }            // We are over temp!!
      tft.print(avgTemp,1);
      if (useCelcius) { tft.print(" C"); } else { tft.print(" F"); }
    }
    prev_vals[2] = avgTemp;
  }
  // Disable the laser and show High Inlet Temperature
  if(avgTemp > maxInletTemp && buttonState[4]) { errorNo = 2; }

  /* OUTLET TEMPERATURE 
   Just like we did on the inlet temperature,
   let's calculate the average temperature over the last 5 seconds
   to help counteract any erroneous readings */
  inputValue = int(Thermistor(analogRead(OUTLET_TEMP)));  // read the Thermistor

  totTemps = 0;
  avgTemp = 0;
  
  // Shift the new temp in and the oldest temp out...
  for(int tt=1;tt<5;tt++){
    totTemps = totTemps + prevOutletTemps[tt];
    prevInletTemps[tt] = prevOutletTemps[tt-1];
  }
  prevOutletTemps[0] = inputValue;
  totTemps = totTemps + inputValue;
  avgTemp = totTemps / 5;

  if(avgTemp != prev_vals[3]) {
    if(screenNo == 0) {  // Only display the values if we're in the main screen
      tft.fillRect(temp2LabelPos[0]+85, temp2LabelPos[1], 95, 22, BLACK);
      tft.setCursor(temp2LabelPos[0]+85, temp2LabelPos[1]);
      tft.setTextColor(GREEN);
      if(inputValue > 73) { tft.setTextColor(YELLOW); }
      if(inputValue > 77) { tft.setTextColor(RED); }
      tft.print(inputValue,1);
      if (useCelcius) { tft.print(" C"); } else { tft.print(" F"); }
    }
    prev_vals[3] = avgTemp;
  }
  // High Inlet Temperature
  //if(avgTemp > 90 && buttonState[4]) { errorNo = 3; }

  if(errorNo < 9999) {
    if(screenNo == 0) {  // Only display the values if we're in the main screen
      disableLaser(errorNo);
    }
  }
  flowSensorTicks = 0;    // Reset the Flow Sensor Pulse Count

}

// Update our timer display
void updateTime() {
   if (accFireTime != 0 && fireTime != 0) {
     if(screenNo == 0) {  
       // Erase the old time
       tft.fillRect(jobTimeLabelPos[0]+80, jobTimeLabelPos[1], 95, 20, BLACK);
  
       tft.setCursor(jobTimeLabelPos[0]+80, jobTimeLabelPos[1]);
       tft.setTextColor(WHITE);
  
       if (accFireTime/1000 >= 60) {
         tft.print(int(accFireTime/1000/60));
         tft.print("m, "); 
         tft.print(accFireTime/1000 - (int(accFireTime/1000/60) * 60));
         tft.println("s"); 
       } else {
         tft.print(accFireTime/1000); tft.println("s");
       }
     }

     tickCount++;

   }

   // If the tickcount is more than 30 seconds, update the total and write to the card
   if (tickCount > 30) {
     // Add the accumulated fire time to our lifetime
     //totalFireTime = round(totalFireTime + (accFireTime/1000)/60);
     Serial.print(round(totalFireTime + (accFireTime/1000)/60)); Serial.println(" total");

     if(screenNo == 0) {
       // Update total time
       tft.fillRect(totalTimeLabelPos[0]+80, totalTimeLabelPos[1], 100, 20, BLACK);
       tft.setCursor(totalTimeLabelPos[0]+80, totalTimeLabelPos[1]);
      // tft.setTextColor(WHITE);
      int tmp = int(totalFireTime + (accFireTime/1000/60));
      if(tmp >= 60) {
        tft.print(int(tmp/60));
        tft.print("h, "); 
        tft.print((tmp - (int(tmp/60)*60)));
        tft.println("m"); 
        
      } else {
        tft.print(int(totalFireTime + (accFireTime/1000)/60)); tft.print("m");
      }
     }

     tickCount = 0;
     // Update the card
     // convert from an unsigned long int to a 4-byte array
    unsigned long accTotalFire = round(totalFireTime + (accFireTime/1000)/60);
    lifeTime[0] = (int)((accTotalFire >> 24) & 0xFF) ;
    lifeTime[1] = (int)((accTotalFire >> 16) & 0xFF) ;
    lifeTime[2] = (int)((accTotalFire >> 8) & 0XFF);
    lifeTime[3] = (int)((accTotalFire & 0XFF));

    //Serial.println(lifeTime[0]);
    //Serial.println(lifeTime[1]);
    //Serial.println(lifeTime[2]);
    //Serial.println(lifeTime[3]);

    // Reset position
    dataFile.seek(0);
    dataFile.write(&lifeTime[0], 4);
    dataFile.flush();

   }

}

void showMsg(int msgNo, boolean isError) {
  tft.setTextColor(BLUE);
  if(isError) { tft.setTextColor(RED); }
  tft.fillRect(129, 190, 190, 44, BLACK);
  if(msgNo != 9999) {
    tft.setCursor(135,195); tft.println(errors1[msgNo]);
    tft.setCursor(135,217); tft.println(errors2[msgNo]);
  }
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0; //startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  //Serial.println();
 // Serial.print(F("Loading image '"));
  //Serial.print(filename);
  //Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
   read32(bmpFile);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
     //Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        //Serial.print(F("Image size: "));
       // Serial.print(bmpWidth);
       // Serial.print('x');
       // Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.color565(r,g,b));
          } // end pixel
        } // end scanline
        //Serial.print(F("Loaded in "));
       // Serial.print(millis() - startTime);
        //Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  //if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void loadSettings() {
  byte checksum[6] = {83, 87, 80, 76, 76, 67};

  coverDisable = true;
  useCelcius = false;
  useLiters = true;
  maxInletTemp = 78.0;
  minFlowRate = 2.2;
  
  // Validate memory settings otherwise use the defaults  
  boolean validChecksum = true;
  for(int i = 0; i < 6; i++) {
    if (readEeprom(i) != checksum[i]) { validChecksum = false; }
  }
  
  if(validChecksum) {
    
    if(readEeprom(6) == 1) { coverDisable = true; } else { coverDisable = false; }
    useCelcius   = readEeprom(7);
    useLiters    = readEeprom(8);
    EEPROM_readAnything(9, maxInletTemp);
    EEPROM_readAnything(13, minFlowRate);
                  
  }
 
}
/*void saveSettings(boolean coverDisable, boolean useCelcius, boolean useLiters, float maxInletTemp, float minFlowRate){
  
}

void writeAddress(int address, byte val) {
  EEPROM.write(address,val);
}*/



