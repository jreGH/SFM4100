#include <Wire.h>

#define I2C_SDAPIN 4
#define I2C_SCLPIN 5

#define _SFM4100_I2C_ADDR_ 0x01 //I2C address!
#define _SFM4100_DATAREQ_ 0xF1
#define CRC_POLYNOMIAL 0x131

const double H2CAL = 1.0; 
double pressure = 1.0; //put in correct value here

double H2_sccm;

char msg[120];

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
}

void loop() {
  
  if (!readSFM4100()) {
    Serial.println("Failed to read SFM4100 data.");
  }
  else {
    sprintf (msg, "H2 flow = %d.%.2d", (int)H2_sccm, (int)((H2_sccm-(int)(H2_sccm))*100));
    Serial.println(msg);
    
  }
  
  //Wait to do it again
  delay(3000);
}

boolean readSFM4100() {
  uint8_t flowData[2];
  uint8_t checkSum;
  
  //Request a measurement
  Wire.beginTransmission(_SFM4100_I2C_ADDR_);
  
  //Request data
  Wire.write(_SFM4100_DATAREQ_);
  
  //Close the request
  Wire.endTransmission();
  
  //Now read a response
  Wire.requestFrom(_SFM4100_I2C_ADDR_, 3);
  flowData[0] = Wire.read();
  flowData[1] = Wire.read();
  checkSum = Wire.read();
    
  //Do the checksum
  if (checkCRC(flowData, 0x02, checkSum)) {
    int H2int = flowData[1] | (flowData[0] << 8);
    sensirionCal ((double)H2int, pressure);
  }
  else {
    Serial.println("Invalid data received from the SFM4100!");
    return false;
  }
  
  return true;
}

void sensirionCal(double measurement, double pressure)
{
//pressure is the measured gauge pressure (psi)
//Linear fit ([slope intercept]) to the data
double p[2] = {1.1523,-283.7969};
double p7[2] = {1.4488,-213.9459};
 H2_sccm= (measurement - p[1] - (p7[1]-p[1])/7 * pressure) / (p[0] + (p7[0] - p[0])/7 * pressure);
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
  
  if (crc != checkSum) {
    return false;
  }
  
  return true;
}


