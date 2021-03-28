#include "I2Cdev.h"

#define    LIDARLite_ADDRESS       0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure         0x00          // Register to write to initiate ranging.
#define    REG_STATUS              0x01          // Status register
#define    MeasureValue            0x04          // Value to initiate ranging.
#define    RegisterHighLowB        0x8f          // Register to get both High and Low bytes in 1 call.
#define    CalibrationRegister     0x13          // The register to set for calibration
#define    CalibrationOffsetVlue   0x03          // The calibration offset... see note below.

/*
Calibration offset is a Two's Compliment Value:
For postive offset... i.e. to add a constant to the distance output... simply convert the decimal number to hex
Example... if i want to add 5 to all distance readings... the CalibrationOffsetValue would be 0x05
For negative offset... i.e. to subtract a constant from the distance output, you need to subtract the value from 256 
and convert that to hex. 
Example 1... if i want to remove 5 from all distance readings: 256-5 = 251 so the CalibrationOffsetValue would be 0xFB
Example 2... if i want to remove 12 from all distance readings: 256-12 = 244 so the CalibrationOffsetValue would be 0xF4
Use this page to get the hex values: http://www.cs.princeton.edu/courses/archive/fall07/cos109/bc.html
*/

class LIDARLite {
  private:
    uint8_t nacksCount = 0;

    void calibrate() {
     // Write 0x04 to register 0x00
      uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
      Fastwire::beginTransmission(LIDARLite_ADDRESS);
      while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
        Fastwire::write(CalibrationRegister);
        nackack = Fastwire::write(CalibrationOffsetVlue); // Write Calibration Offset Value to 0x13
        delay(1); // Wait 1 ms to prevent overpolling
      }
      Fastwire::stop();
    };
    
    int fetchRawDistance() {
     // Write 0x04 to register 0x00
      uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
      Fastwire::beginTransmission(LIDARLite_ADDRESS);
      while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
        Fastwire::write(RegisterMeasure);
        nackack = Fastwire::write(MeasureValue); // Write 0x04 to 0x00
        delay(1); // Wait 1 ms to prevent overpolling
      }
      Fastwire::stop();
      byte distanceArray[2]; // array to store distance bytes from read function
      
      // Read 2byte distance from register 0x8f
      nackack = 100; // Setup variable to hold ACK/NACK resopnses     
      while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
        nackack = Fastwire::readBuf(LIDARLite_ADDRESS << 1, RegisterHighLowB, distanceArray, 2);
        delay(1); // Wait 1 ms to prevent overpolling
      }
      
      int dist = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
      
      return dist; // give us this value
    };
    
    int emaFilter(int current_value) { 
        int alpha = 5; // This is in percentage. Should be between 0-99
        static uint16_t exponential_average = current_value;
        
        exponential_average = (alpha * (uint32_t)current_value + (100 - alpha) * (uint32_t)exponential_average) / 100;

        return exponential_average;
    };
  
  public:
    LIDARLite() {
      
    };

    void init() {
      calibrate();
    };

    int measureDistance() {
      return emaFilter(fetchRawDistance());
    };

};
