#include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include "ST25DVSensor.h"
// #include "extendedNFC.h"

// #include <Adafruit_GPS.h>


// #include <SoftwareSerial.h>

#include <SPI.h>              // include libraries
#include <TinyGPS++.h>
#include <LoRa.h>

#define SDA_PIN PB9
#define SCL_PIN PB7
#define INT_PIN PB5
#define LED_PIN PC_13
#define MOSI_PIN PA7
#define MISO_PIN PA6
#define CLK_PIN PA5
#define LORA_CS_PIN PA4
#define LORA_RST_PIN PA3
#define LORA_INT_PIN PA2


#define GPS_BAUD 9600  
#define LORA_FREQ 915E6
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SerialPort SerialUSB
#define WireNFC Wire



// String strNfcInput = "";
// String strSerialInput = "";
HardwareTimer *TransmittTim = new HardwareTimer(TIM3);
HardwareTimer *GPSTim = new HardwareTimer(TIM4);
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TinyGPSPlus gps;

// Adafruit_GPS GPS(&Serial1);

volatile bool bolSerialLapse = 0; // This var is prevent serial lockups
volatile bool bolGpsLapse = 0;

// SoftwareSerial gpsSerial(PB7, PB6);

void fnvdSerialTimerCallback(void);
// void I2C_Scan(void);
// void failState(const char * s);
// void fnvdInitDisplay();
// void fnvdInitNfc();
void fnvdInitSerialTimer();
void fnvdExitSetup();
// void fnvdSendNfc();
// void fnvdRecieveNfc();
void fnvdDisplayGps();
void fnvdInitGpsTimer();
void fnvdGpsTimerCallback();

void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
bool runEvery(unsigned long interval);





int tagID = 1;

void setup(){

  SerialUSB.begin(9600);
  // while(!SerialUSB); // Holds the system from running until the serial port has been connected
  
  // SerialUSB.println("Initing GPS");
  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  Serial1.begin(GPS_BAUD);
  fnvdInitGpsTimer();
  // SerialUSB.println("Exiting init GPS");


  // fnvdInitDisplay();  
  // for(int i = 0; i < 10000; i++){
  //   // delay(1);
  //   if(!(!SerialUSB)){
  //     display.println("Starting with Serial");
  //     break;
  //   }
  //   if(!SerialUSB && i == 150){
  //     display.println("Starting without Serial");
  //     break;
  //   }
  // }
  // LoRa


  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_INT_PIN);

  if(!LoRa.begin((const long)LORA_FREQ)){
    SerialUSB.println("LoRa init failed");
    // display.println("LoRa init failed");
    while(1);
  }else{
    SerialUSB.println("LoRa init Passed");
  }
  LoRa.setSyncWord(0xF3);

  // LoRa.onReceive(onReceive);

  // fnvdInitNfc();

  fnvdInitSerialTimer();

  // fnvdExitSetup();
  SerialUSB.println("Entering loop");
}

int counter = 0;

// void loop() {
//   // try to parse packet
//   int packetSize = LoRa.parsePacket();
//   if (packetSize) {
//     // received a packet
//     SerialUSB.print("Received packet '");

//     // read packet
//     while (LoRa.available()) {
//       String LoRaData = LoRa.readString();
//       SerialUSB.print(LoRaData); 
//     }

//     // print RSSI of packet
//     SerialUSB.print("' with RSSI ");
//     SerialUSB.println(LoRa.packetRssi());
//   }
// }

// void loop(){
//   SerialUSB.print("Sending packet: ");
//   SerialUSB.println(counter);

//   //Send LoRa packet to receiver
//   LoRa.beginPacket();
//   LoRa.print("hello ");
//   LoRa.print(counter);
//   LoRa.endPacket();
//   counter++;

//   delay(1000);
// }

// String message = "";
void loop(){
  // int packetSize = LoRa.parsePacket();
  // String message = "";
  // if (packetSize) {
    
  //   // received a packet
  //   message = "Received packet '";

  //   // read packet
  //   while (LoRa.available()) {
  //     message += LoRa.readString();
  //     // SerialUSB.print(LoRaData); 
  //   }

  //   // print RSSI of packet
  //   message += "' with RSSI ";
  //   message += LoRa.packetRssi();
  // }

  // while (Serial1.available() > 0)
  //   if (gps.encode(Serial1.read()))
  //     fnvDisplayGps();
  // gps.encode(Serial1.read());
  // SerialUSB.print(Serial1.read());
  // if(Serial1.available())
    // SerialUSB.println(Serial1.readString());
  if(bolGpsLapse){
    while(Serial1.available())
      gps.encode(Serial1.read());
    // String gpsMessage = "";
    // while(Serial1.available())
    //   gpsMessage += (char)Serial1.read();
    // gpsMessage += "\n";
    // LoRa_sendMessage(gpsMessage);
    bolGpsLapse = 0;
  }
  if(SerialUSB.available()){
    // fnvdSendNfc();
  }else{
    if(bolSerialLapse){ // Timer wait
      // SerialUSB.print(message);
      // if(message != ""){
      //   SerialUSB.print(" RSSI: ");
      //   SerialUSB.println(LoRa.packetRssi());
      // }

      fnvdDisplayGps();
      // SerialUSB.println(message);
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


// /**
//  * @brief Just the arduino i2c scanner
//  * 
//  */
// void I2C_Scan(void){
//   Wire.begin();
//   Wire.setSDA(SDA_PIN);
//   Wire.setSCL(SCL_PIN);
//   SerialUSB.begin(9600);
//   while (!SerialUSB);             // Leonardo: wait for serial monitor
//   SerialUSB.println("\nI2C Scanner");
//   byte error, address;
//   int nDevices;
 
//   SerialUSB.println("Scanning...");
 
//   nDevices = 0;
//   for(address = 1; address < 127; address++ ){
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
 
//     if (error == 0)
//     {
//       SerialUSB.print("I2C device found at address 0x");
//       if (address<16)
//         SerialUSB.print("0");
//       SerialUSB.print(address,HEX);
//       SerialUSB.println("  !");
 
//       nDevices++;
//     }
//     else if (error==4)
//     {
//       SerialUSB.print("Unknown error at address 0x");
//       if (address<16)
//         SerialUSB.print("0");
//       SerialUSB.println(address,HEX);
//     }    
//   }
//   if (nDevices == 0)
//     SerialUSB.println("No I2C devices found\n");
//   else
//     SerialUSB.println("done\n");
 
//   delay(5000);           // wait 5 seconds for next scan
// }


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

// /**
//  * @brief Inits the display
//  * 
//  */
// void fnvdInitDisplay(){
//     // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
//   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
//     SerialUSB.println(F("SSD1306 allocation failed"));
//     for(;;); // Don't proceed, loop forever
//   }

//   display.clearDisplay();
//   display.setRotation(2);
//   display.setTextSize(1);
//   display.setTextColor(WHITE);
//   display.setCursor(0, 0);
//   display.setTextWrap(true);
//   display.println("In bootup");
//   display.display();
// }

// /**
//  * @brief Inits the nfc reader
//  * 
//  */
// void fnvdInitNfc(){
//   // if(st25dv.begin(INT_PIN, LED_PIN, &WireNFC) == 0) {
//   if(nfc.begin(INT_PIN, LED_PIN, &WireNFC)){
//     SerialUSB.println("NFC Init done!");
//     // display.println("System Init done!");
//   } else {
//     SerialUSB.println("NFC Init failed!");
//     // display.println("System Init failed!");
//     while(1);
//   }
// }

/**
 * @brief Sets up the timer for the usb serial interface
 * 
 */
void fnvdInitSerialTimer(){
  TransmittTim->setOverflow(1000000, MICROSEC_FORMAT);
  TransmittTim->attachInterrupt(fnvdSerialTimerCallback);
  TransmittTim->resume();
}

// /**
//  * @brief Finishes the setup and clears the display
//  * 
//  */
// void fnvdExitSetup(){
//   display.print("Exiting bootup");
//   display.display();
//   delay(2000);
//   display.clearDisplay();
//   display.display();
// }

// /**
//  * @brief Sends a Url over nfc
//  * 
//  */
// void fnvdSendNfc(){
//   display.clearDisplay();
//   display.setCursor(0,0);
//   strSerialInput = SerialUSB.readStringUntil('\n');
//   SerialUSB.print("Input was: "); SerialUSB.println(strSerialInput);
//   display.println(strSerialInput);
//   // if(st25dv.writeURI(URI_ID_0x03_STRING, strSerialInput.c_str(), "")){
//   // int error_code = nfc.writeText(strSerialInput);
//   int error_code = nfc.writeURI(URI_ID_0x03_STRING, strSerialInput.c_str(), "");
//   if(error_code){
//     display.printf("Write failed!: %d\n", error_code);
//   }
//   display.display();
// }

// /**
//  * @brief Recieves a message over nfc
//  * 
//  */
// void fnvdRecieveNfc(){
//   String uri_read;
//   if(st25dv.readURI(&uri_read)){
//         display.clearDisplay();
//         display.setCursor(0,0);
//         display.println("Read Failed");
//         display.display();
//   }else{
//     if(strNfcInput != uri_read){
//       strNfcInput = uri_read;
//       display.clearDisplay();
//       display.setCursor(0,0);
//       display.println(uri_read);
//       SerialUSB.print("Input text was: "); SerialUSB.println(uri_read);
//       display.display();
//     }
//   }
// }

/**
 * @brief Allows the MCU to wait for lora messages via interrupt
 * 
 */
void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

/**
 * @brief Allows the lora radio to send without being affected by interrupts
 * 
 */
void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

/**
 * @brief Sends a string over lora
 * 
 * @param message Input string to be transmitted
 */
void LoRa_sendMessage(String message) {
  SerialUSB.print("Lora Message :");
  SerialUSB.println(message);
  // LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

/**
 * @brief The interrupt callback for lora in
 * 
 * @param packetSize ???
 */
void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  SerialPort.print("Node Receive: ");
  SerialPort.println(message);
}

/**
 * @brief Callback for sent message
 * 
 */
void onTxDone() {
  SerialPort.println("TxDone");
  LoRa_rxMode();
}

/**
 * @brief Defines when messages are sent
 * 
 * @param interval The interval time
 * @return bool Whether the interval is within params
 */
bool runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

/**
 * @brief Used for displaying everything on the gps on the usb
 * 
 */
void fnvdDisplayGps()
{
  String message = "";// "Location: ";
  message += tagID;
  message += ",";
  // SerialUSB.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    message += gps.location.lat();
    message += ",";
    message += gps.location.lng();
    // SerialUSB.print(gps.location.lat(), 6);
    // SerialUSB.print(F(","));
    // SerialUSB.print(gps.location.lng(), 6);
  }
  else
  {
    message += "NULL,NULL";
    // SerialUSB.print(F("INVALID"));
  }
  message += ","; // "  Date/Time: ";
  // SerialUSB.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    message += gps.date.day();
    message += "/";
    message += gps.date.month();
    message += "/";
    message += gps.date.year();
    // SerialUSB.print(gps.date.month());
    // SerialUSB.print(F("/"));
    // SerialUSB.print(gps.date.day());
    // SerialUSB.print(F("/"));
    // SerialUSB.print(gps.date.year());
  }
  else
  {
    message += "NULL";
    // SerialUSB.print(F("INVALID"));
  }
  message += ",";//" ";
  // SerialUSB.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) message += "0"; // SerialUSB.print(F("0"));
    message += gps.time.hour();
    message += ":";
    // SerialUSB.print(gps.time.hour());
    // SerialUSB.print(F(":"));
    if (gps.time.minute() < 10) message += "0"; // Serial.print(F("0"));
    message += gps.time.minute();
    message += ":";
    // SerialUSB.print(gps.time.minute());
    // SerialUSB.print(F(":"));
    if (gps.time.second() < 10) message += "0"; // SerialUSB.print(F("0"));
    message += gps.time.second();
    // message += ".";
    // // SerialUSB.print(gps.time.second());
    // // SerialUSB.print(F("."));
    // if (gps.time.centisecond() < 10) message += "0";// SerialUSB.print(F("0"));
    // message += gps.time.centisecond();
    // // SerialUSB.print(gps.time.centisecond());
  }
  else
  {
    message += "NULL";
    // SerialUSB.print(F("INVALID"));
  }
  message += ",";//" Number of Satelites: ";
  if(gps.satellites.isValid()){    
    message += gps.satellites.value();
  }else{
    message += "NULL";//"No Fix";
  }
  // message += "\n";
  // SerialUSB.println();
  LoRa_sendMessage(message);
  // SerialUSB.print(message);
}


/**
 * @brief Inits a timer for reading the gps
 * 
 */
void fnvdInitGpsTimer(){
  GPSTim->setOverflow(10, HERTZ_FORMAT);
  GPSTim->attachInterrupt(fnvdGpsTimerCallback);
  GPSTim->resume();
}

/**
 * @brief Callback for gps timer
 * 
 */
void fnvdGpsTimerCallback(){
  bolGpsLapse = 1;
}

