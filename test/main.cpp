#include <Arduino.h>

#include <SPI.h>
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

#define LORA_FREQ 915E6

void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();
bool runEvery(unsigned long interval);
void fnvdInitSerialTimer();
void fnvdSerialTimerCallback(void);

String serialBuffer;
HardwareTimer *TransmittTim = new HardwareTimer(TIM3);

volatile bool bolSerialLapse = 0; // This var is prevent serial lockups

void setup(){
    SerialUSB.begin(9600);
    while(!SerialUSB);
    serialBuffer = "";

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
    fnvdInitSerialTimer();
}

char charSerialInput = '\0';

void loop(){
    if(bolSerialLapse){
        if(SerialUSB.available()){
            charSerialInput = SerialUSB.read();
            SerialUSB.write(charSerialInput);
            serialBuffer += charSerialInput;
        } 
        // if(charSerialInput == '\b'){
        //   serialBuffer.remove(serialBuffer.length() - 1);
        //   serialBuffer.remove(serialBuffer.length() - 1);
        // }
        if(charSerialInput == '\r'){
            serialBuffer.remove(serialBuffer.indexOf('\r')); serialBuffer.remove(serialBuffer.indexOf('\n'));
            LoRa_sendMessage(serialBuffer);
            serialBuffer = "";
            SerialUSB.println();
        }
        charSerialInput = '\0';
        bolSerialLapse = 0;
    }
}

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
//   SerialUSB.print("Lora Message :");
//   SerialUSB.println(message);
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

//   SerialUSB.print("Node Receive: ");
//   SerialUSB.println(message);
}

/**
 * @brief Callback for sent message
 * 
 */
void onTxDone() {
//   SerialUSB.println("TxDone");
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
 * @brief Callback for USB Serial message to prevent lockups
 * 
 */
void fnvdSerialTimerCallback(void){
  bolSerialLapse = 1;
}

/**
 * @brief Sets up the timer for the usb serial interface
 * 
 */
void fnvdInitSerialTimer(){
  TransmittTim->setOverflow(20, HERTZ_FORMAT);
  TransmittTim->attachInterrupt(fnvdSerialTimerCallback);
  TransmittTim->resume();
}


// #include <Arduino.h>

// #include <SPI.h>              // include libraries
// #include <LoRa.h>

// #define SDA_PIN PB7
// #define SCL_PIN PB6
// #define INT_PIN PB5
// #define LED_PIN PC_13
// #define MOSI_PIN PA7
// #define MISO_PIN PA6
// #define CLK_PIN PA5
// #define LORA_CS_PIN PA4
// #define LORA_RST_PIN PA3
// #define LORA_INT_PIN PA2

// #define LORA_FREQ 915E6
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 32 // OLED display height, in pixels
// #define Serial SerialUSB
// #define WireNFC Wire

// const int inInterval = 1000;
// long inLastSendTime = 0;
// int inCounter = 0;

// byte msgCount = 0;            // count of outgoing messages
// byte localAddress = 0xBB;     // address of this device
// byte destination = 0xFF;      // destination to send to


// void sendMessage(String outgoing);
// void onReceive(int packetSize);


// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   while(!Serial);
//   Serial.println("Starting lorba");

//   SPI.begin();
//   SPI.setClockDivider(SPI_CLOCK_DIV16);
//   LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_INT_PIN);

//   if(!LoRa.begin((const long)LORA_FREQ)){
//     Serial.println("LoRa init failed");
//     while(1);
//   }
//   LoRa.setSignalBandwidth(125e3);
//   LoRa.setCodingRate4(5);
//   LoRa.setSpreadingFactor(12);
//   LoRa.setSyncWord(0xF3);
//   Serial.println("LoRa Initializing OK!");
// }

// void loop() {
//   if(millis() - inLastSendTime > inInterval){
//     String message = "HeLoRa World!";   // send a message
//     LoRa.beginPacket();
//     LoRa.print(message);
//     LoRa.endPacket();
//     // sendMessage(message);
//     Serial.println("Sending " + message);
//     inLastSendTime = millis();            // timestamp the message
//   }
//   onReceive(LoRa.parsePacket());
//   // // put your main code here, to run repeatedly:
//   // Serial.print("Sending packet: ");
//   // Serial.println(inCounter);

//   // //Send LoRa packet to receiver
//   // LoRa.beginPacket();
//   // LoRa.print("hello ");
//   // LoRa.print(inCounter);
//   // LoRa.endPacket();

//   // inCounter++;

//   // delay(1000);
// }

// /**
//  * @brief 
//  * 
//  * @param outgoing 
//  */
// void sendMessage(String outgoing) {
//   // LoRa_txMode();
//   LoRa.beginPacket();                   // start packet
//   LoRa.write(destination);              // add destination address
//   LoRa.write(localAddress);             // add sender address
//   LoRa.write(msgCount);                 // add message ID
//   LoRa.write(outgoing.length());        // add payload length
//   LoRa.print(outgoing);                 // add payload
//   LoRa.endPacket();                     // finish packet and send it
//   msgCount++;                           // increment message ID
// }


// /**
//  * @brief 
//  * 
//  * @param packetSize 
//  */
// void onReceive(int packetSize) {
//   if (packetSize == 0) return;          // if there's no packet, return

//   // read packet header bytes:
//   int recipient = LoRa.read();          // recipient address
//   byte sender = LoRa.read();            // sender address
//   byte incomingMsgId = LoRa.read();     // incoming msg ID
//   byte incomingLength = LoRa.read();    // incoming msg length

//   String incoming = "";

//   while (LoRa.available()) {
//     incoming += (char)LoRa.read();
//   }

//   if (incomingLength != incoming.length()) {   // check length for error
//     Serial.println("error: message length does not match length");
//     return;                             // skip rest of function
//   }

//   // if the recipient isn't this device or broadcast,
//   if (recipient != localAddress && recipient != 0xFF) {
//     Serial.println("This message is not for me.");
//     return;                             // skip rest of function
//   }

//   // if message is for this device, or broadcast, print details:
//   Serial.println("Received from: 0x" + String(sender, HEX));
//   Serial.println("Sent to: 0x" + String(recipient, HEX));
//   Serial.println("Message ID: " + String(incomingMsgId));
//   Serial.println("Message length: " + String(incomingLength));
//   Serial.println("Message: " + incoming);
//   Serial.println("RSSI: " + String(LoRa.packetRssi()));
//   Serial.println("Snr: " + String(LoRa.packetSnr()));
//   Serial.println();
// }
