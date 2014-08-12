#include <Wire.h>

#define I2C_SDAPIN 4
#define I2C_SCLPIN 5

#define _SFM4100_I2C_ADDR_ 0x01 //I2C address!
#define _SFM4100_DATAREQ_ 0xF1
#define CRC_POLYNOMIAL 0x131

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
}

void loop() {
  int flowMeas;
  uint8_t flowData[2];
  
  if (!readSFM100()) {
  }
  
  //Request a measurement
  Wire.beginTransmission(_SFM4100_I2C_ADDR_);
  
  //Request data
  Wire.write(_SFM4100_DATAREQ_);
  
  //Close the request
  Wire.endTransmission();
  
  //Now read a response
  Wire.requestFrom(_SFM4100_I2C_ADDR_, 3);
  uint8_t flowData[0] = Wire.read();
  uint8_t flowData[1] = Wire.read();
  uint8_t uERR = Wire.read();
    
  //Checksum
  if (checkCRC(flowData, 0x02, uERR) {
    flowMeas = flowData[1] | (flowData[0] << 8);
  }
  else {
    Serial.println("Invalid data received from the SFM4100!");
  }
  
  //Wait to do it again
  delay(3000);
}


boolean checkCRC(uint8_t data[], uint8_t nBytes, uint8_t checkSum) {
  uint8_t crc = 0;
  uint8_t byteCount;
  
  for (byteCount=0;byteCount < nBytes;byteCount++) {
    crc ^= (data[byteCount]);
    
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      }
      else {
        crc = (crc << 1);
      }
    }
    
  }
  
  if (crc != checksum) {
    return false;
  }
  
  return true;
}

