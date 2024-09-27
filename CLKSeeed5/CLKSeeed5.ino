#include <SPI.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515_can.h>

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
#endif

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

  unsigned char frameDATA = 0;
  // TODO: Add pins for R/P, W/S, Starter Lockout
  // 2nd byte 00100000 - TFT active 
  // 2nd byte 00101010 - TFT inactive (In P/N)
  unsigned char gsGEAR = 0;
  // 4th byte 01001000 <- OFF
  // 4th byte 01001010 <- ON
  unsigned char GS51[8] = {0x00, 0x00, 0x00, 0x18, 0xA0, 0x00, 0xFF,0x60}; // See gearcalc sheet
  unsigned char GS52[8] = {0x4E, 0x20, 0x82, 0x04, 0x00, 0xFF, 0x17,0xFF}; // 2nd Byte sends messages to cluster
  
// This is loading Gearbox nominals, then manipulating bytes in program
// first byte is message to gear readout, 2nd status messages, 3rd is ATF temp
// 4th is variant coding effectively. 0x04 = RWD, 5AT, Small NAG; 5th is target/actual gear. Match these to 218, 6th is
// Loss moment. 7 & 8 are a combo
//
  unsigned char MS51[8] = {0,0,0,0,0,0,0,0};
  unsigned char ESP51[8] = {0,0,0,0,0,0,0,0};
  unsigned char buf[8] = {0,0,0,0,0,0,0,0};
  unsigned char len = 0;
  unsigned int engRPM = 0;
  unsigned int DHL = 0; // Rear Left Wheel Speed
  unsigned int DHR = 0; // Rear Left Wheel Speed
  unsigned int VSS = 0; // Avg Rear Wheel Speed
  unsigned long gearRatio = 0;
  bool tqrToggle = false;
  bool tqPar1 = false;
  bool tqPar2 = false;

void setup() {
    SERIAL.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(100);
    }
    SERIAL.println("CAN BUS Shield init ok!");
    CAN.init_Mask(0,0,0x03FF);
    CAN.init_Filt(0,0,0x0308);
    CAN.init_Mask(1,0,0x03FF);
    CAN.init_Filt(1,0,0x0208);
    
}

void canCheck() {

  if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned int canId = CAN.getCanId();
    deBuff(canId, buf);
  }
}

void deBuff(unsigned int msgId, unsigned char buff[8]){
  unsigned char i;
  if (msgId == (0x308)) {
    for (i = 0; i < 8; i++) {
      MS51[i] = buff[i];
    }
  }
  if (msgId == (0x208)) {
    for (i = 0; i < 8; i++) {
      ESP51[i] = buff[i];
    }
  }
}

void gearFind() {
  if ((gearRatio > 60) && (gearRatio < 80)){
    gsGEAR = 0x11;
    GS52[0] = 49;
    frameDATA = (0x05); 
  }
  
  else if ((gearRatio > 35) && (gearRatio < 45)){
    gsGEAR = 0x22;
    GS52[0] = 50;
    frameDATA = (0x05); 
  }
    
  else if ((gearRatio > 25) && (gearRatio < 30)){
    gsGEAR = 0x33;
    GS52[0] = 51;
    frameDATA = (0x05); 
  }
  
  else if ((gearRatio > 18) && (gearRatio < 23)){
    gsGEAR = 0x44;
    GS52[0] = 52;
    frameDATA = (0x05); 
  }
    
  else if ((gearRatio > 14) && (gearRatio < 18)){
    gsGEAR = 0x55;
    GS52[0] = 53;
    frameDATA = (0x05); 
  }
  
  else if ((gearRatio > 11) && (gearRatio < 14)){
    gsGEAR = 0x55; // Gear 5 no 2, to less confuse.
    GS52[0] = 54;
    frameDATA = (0x05); 
  }
  
  else if((engRPM > 1500) && (gearRatio > 2) && (VSS > 20)){
    GS52[0] = 70;
  }
  else {
    GS52[0] = 78;
    frameDATA = (0x06); // Neutral
  }
}

void loop() {
//  Serial.println("\n\nNew Cycle");
//  engRPM = 0;
  frameDATA = 0x06;
//  DHL = 0; // Rear Left Wheel Speed
//  DHR = 0; // Rear Left Wheel Speed
//  VSS = 0; // Avg Rear Wheel Speed
  unsigned char i;
  
//  for (i = 0; i < 8; i++) {
//      ESP51[i] = 0;
//  }
//  for (i = 0; i < 8; i++) {
//      MS51[i] = 0;
//  }
  
  canCheck(); // This is an extremely lazy way of doing things. DO NOT DO THIS.
  canCheck(); // The goal here is that out of 3 tries, hopefully we'll get entries for both buffers
  canCheck(); // NB, this is not deterministic.

// We can get lazy in the gear calcs in two ways, 1) MB are electronically limited to 250 kmh
// meaning we won't see the upper 2 bits of speed value, using bitshifts and longs as bad floats
// The more precise way to do this is with floats, but what if we didn't.
// wheel RPM = (VSS) / pi*dia * 60
// wheel rpm = (km/h)*(Rev/km)*(1/60)
// gear ratio = engrpm / ( diff ratio * wheel rpm)

  engRPM = ((MS51[1] << 8)+(MS51[2] & 0xFF));
  DHL = (((ESP51[4] & 0x3F) << 8)+(ESP51[5] & 0xFF)); // Rear Left Wheel Speed
  DHR = (((ESP51[6] & 0x3F) << 8)+(ESP51[7] & 0xFF)); // Rear Left Wheel Speed
  VSS = (DHR+DHL)/2; // Avg Rear Wheel Speed

  if ( VSS > 0){
    gearRatio = 0 + (engRPM & 0xFFFF);
    gearRatio = gearRatio * 16; 
    gearRatio = gearRatio/((VSS/16) * 26); // 413 = 16*(25.788 = (504 [Rev/km on stock tires] * 3.07/60))
    gearFind();
  }
  
// Gear ratios for the NSG-370 are: 4.46, 2.61, 1.72, 1.25, 1.00, 0.84 for first through sixth. (This is Actually Jeep NSG370 ratios, and is wrong for Xfire 4th gear)

  else {
    GS52[0] = 78;
    frameDATA = (0x06); // Neutral
  }
  
// handle incoming frames better.

  if (engRPM > 1200) {
    GS51[2] = gsGEAR;
    GS52[4] = gsGEAR;
    frameDATA = (0x05 & 0xFF); // Drive
  }
// TODO Engine On/Off Logic

  CAN.sendMsgBuf(0x230, 0, 1, frameDATA);
  CAN.sendMsgBuf(0x218, 0, 8, GS51); // still giving problems.
  CAN.sendMsgBuf(0x418, 0, 8, GS52);
//  Serial.print(frameDATA, HEX);
//  Serial.println("\n");
//  Serial.print(engRPM, DEC);
  delay(20);
}
