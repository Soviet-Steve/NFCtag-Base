#include <Arduino.h>

#define SDA_PIN PB7
#define SCL_PIN PB6
#define INT_PIN PB5
#define LED_PIN PC_13
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SerialPort SerialUSB
#define WireNFC Wire

String strSerialInput = "";
HardwareTimer *MyTim = new HardwareTimer(TIM3);
volatile bool bolSerialLapse = 0; // This var is prevent serial lockups

void fnvdSerialTimerCallback(void);
void I2C_Scan(void);
void failState(const char * s);
void fnvdInitDisplay();
void fnvdInitNfc();
void fnvdInitSerialTimer();
void fnvdExitSetup();

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ST25DVSensor.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);




void setup(){
  SerialUSB.begin(9600);
  while(!SerialUSB);

  fnvdInitDisplay();

  const char uri_write_message[] = "st.com/st25";       // Uri message to write in the tag
  const char uri_write_protocol[] = URI_ID_0x01_STRING; // Uri protocol to write in the tag
  String uri_write = String(uri_write_protocol) + String(uri_write_message);
  String uri_read;

  fnvdInitNfc();

  fnvdInitSerialTimer();

  fnvdExitSetup();
}

void loop(){
   if(SerialUSB.available()){
      display.clearDisplay();
      display.setCursor(0,0);
      strSerialInput = SerialUSB.readStringUntil('\n');
      SerialUSB.print("Input was: "); SerialUSB.println(strSerialInput);
      display.println(strSerialInput);
      if(st25dv.writeURI(URI_ID_0x03_STRING, strSerialInput.c_str(), "")){
        display.println("Write failed!");
      }
      display.display();
  }else{
     if(bolSerialLapse == 1){
       SerialUSB.println("No usb serial input");
       bolSerialLapse = 0;
     }
   }
}

/**
 * @brief Callback for USB Serial message to prevent lockups
 * 
 */
void fnvdSerialTimerCallback(void){
  bolSerialLapse = 1;
}


/**
 * @brief Just the arduino i2c scanner
 * 
 */
void I2C_Scan(void){
  Wire.begin();
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  SerialUSB.begin(9600);
  while (!SerialUSB);             // Leonardo: wait for serial monitor
  SerialUSB.println("\nI2C Scanner");
  byte error, address;
  int nDevices;
 
  SerialUSB.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      SerialUSB.print("I2C device found at address 0x");
      if (address<16)
        SerialUSB.print("0");
      SerialUSB.print(address,HEX);
      SerialUSB.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      SerialUSB.print("Unknown error at address 0x");
      if (address<16)
        SerialUSB.print("0");
      SerialUSB.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    SerialUSB.println("No I2C devices found\n");
  else
    SerialUSB.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}


/**
 * @brief Continuous fail state
 * 
 */
void failState(const char * s){
  while(1){
    SerialUSB.printf("System has failed at: %s", s); 
    delay(1000);
  }
}

/**
 * @brief Inits the display
 * 
 */
void fnvdInitDisplay(){
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    SerialUSB.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextWrap(true);
  display.println("In bootup");
  display.display();
}

/**
 * @brief Inits the nfc reader
 * 
 */
void fnvdInitNfc(){
  if(st25dv.begin(INT_PIN, LED_PIN, &WireNFC) == 0) {
    display.println("System Init done!");
  } else {
    display.println("System Init failed!");
    while(1);
  }
}

/**
 * @brief Sets up the timer for the usb serial interface
 * 
 */
void fnvdInitSerialTimer(){
  MyTim->setOverflow(1000000, MICROSEC_FORMAT);
  MyTim->attachInterrupt(fnvdSerialTimerCallback);
  MyTim->resume();
}

/**
 * @brief Finishes the setup and clears the display
 * 
 */
void fnvdExitSetup(){
  display.print("Exiting bootup");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
}