// Function for:
//   - Initializing the MPU9150/MPU6050 gyro and accelerometer
//   - Initializing AK8975A Magnetometer
//   - Calibrating the MPU9150/MPU6050 gyro and accelerometer
//   - Self test of the MPU9150/MPU6050 gyro and accelerometer to check health
//   - Calculating correct angles from x,y-values of the magnetormeter in the horizontal plane
//
// Most methods here are just sligthly adapted from: https://github.com/kriswiner/MPU-9150
// Special thanks to Kris Winer
//

void initMPU9150()
{  
 // wake up device
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  delay(200);
  
 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 011; this sets the sample rate at 1 kHz for both
 // Minimum delay time is 4.9 ms which sets the fastest rate at ~200 Hz
 /*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7 RESERVED
 */
  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9150_ADDRESS, MPU9150_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG);
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] (0xE0 = 0b11100000)
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]       (0x18 = 0b00011000)
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG);
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

  // Configure Interrupts and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9150_ADDRESS, MPU9150_INT_PIN_CFG, 0x02);  // Enable bypass 
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x00);  //Disable the MPU6050 as Master --> I2C Bus control by Arduino
}

void initAK8975A(float * destination)
{
  uint8_t rawData[3];  // x/y/z gyro register data stored here
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  delay(10);
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  delay(10);
  Serial.println(F("Magnetometer calibration values: "));
  Serial.print(F("X-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[0], 2);
  Serial.print(F("Y-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[1], 2);
  Serial.print(F("Z-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[2], 2);

}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// Special thanks for Kris Winers code example on github
void calibrateMPU9150()
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  float gyroBias[3]  = {0, 0, 0}, accelBias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x01);  
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_2, 0x00); 
  delay(200);
  
// Configure device for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9150_ADDRESS, MPU9150_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9150_ADDRESS, MPU9150_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9150_ADDRESS, MPU9150_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

 for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9150_ADDRESS, MPU9150_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers for consideration in calculation
  writeByte(MPU9150_ADDRESS, MPU9150_XG_OFFS_USRH, data[0]);
  writeByte(MPU9150_ADDRESS, MPU9150_XG_OFFS_USRL, data[1]);
  writeByte(MPU9150_ADDRESS, MPU9150_YG_OFFS_USRH, data[2]);
  writeByte(MPU9150_ADDRESS, MPU9150_YG_OFFS_USRL, data[3]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZG_OFFS_USRH, data[4]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZG_OFFS_USRL, data[5]);

// Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9150_ADDRESS, MPU9150_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, MPU9150_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers for consideration in calculation
  writeByte(MPU9150_ADDRESS, MPU9150_XA_OFFSET_H, data[0]);
  writeByte(MPU9150_ADDRESS, MPU9150_XA_OFFSET_L_TC, data[1]);
  writeByte(MPU9150_ADDRESS, MPU9150_YA_OFFSET_H, data[2]);
  writeByte(MPU9150_ADDRESS, MPU9150_YA_OFFSET_L_TC, data[3]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_H, data[4]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for display in the main program
   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
   
// Output for user feedback
   Serial.println(F("Calibrate gyro and accelerometers and load biases in bias registers...."));
   Serial.println(F("MPU9150 acceleration bias"));
   Serial.println(F("  x\t y\t z")); Serial.print("  ");
   Serial.print((int)(1000*accelBias[0])); Serial.print(" \t");
   Serial.print((int)(1000*accelBias[1])); Serial.print(" \t");
   Serial.print((int)(1000*accelBias[2])); Serial.println(" \tmg");       
   Serial.println(F("MPU9150 gyro bias"));
   Serial.println(F("  x\t y\t z")); Serial.print("  ");
   Serial.print(gyroBias[0], 1); Serial.print(" \t");
   Serial.print(gyroBias[1], 1); Serial.print(" \t");
   Serial.print(gyroBias[2], 1); Serial.println(" \tdeg/s");      
   Serial.println(F("Calibration done"));  
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Special thanks for Kris Winers code example on github
void MPU6050SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6]; 
   float SelfTestRes[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     SelfTestRes[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
   //Output
   Serial.print(F("x-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[0],1); Serial.println(F("% of factory value"));
   Serial.print(F("y-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[1],1); Serial.println(F("% of factory value"));
   Serial.print(F("z-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[2],1); Serial.println(F("% of factory value"));
   Serial.print(F("x-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[3],1); Serial.println(F("% of factory value"));
   Serial.print(F("y-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[4],1); Serial.println(F("% of factory value"));
   Serial.print(F("z-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[5],1); Serial.println(F("% of factory value"));
   
   if(SelfTestRes[0] < 1.0f && SelfTestRes[1] < 1.0f && SelfTestRes[2] < 1.0f && SelfTestRes[3] < 1.0f && SelfTestRes[4] < 1.0f && SelfTestRes[5] < 1.0f) {
      Serial.println(F("Passed Selftest successful!"));  
   }
   else{
      Serial.println(F("MPU6050 Selftest failed!"));
   }
   
}


// Function for correction of yaw angle (replaces the atan2 function for more controll)
float wrap(float x_h, float y_h){
  //TODO: Toleranz einbauen
  float microT_tol = 0.2; //within +-0.2µT the value is set to 270 or 90 deg --> no devision by or near 0 of x_h in arctan function
  float angle = 0;
  
  if(x_h < 0) angle = pi-atan(y_h/x_h);
  if(x_h > 0 && y_h < 0) angle = -atan(y_h/x_h);
  if(x_h > 0 && y_h > 0) angle = 2*pi -atan(y_h/x_h);
  //In case of small values for x_h (no devision by 0) set the angle
  if(abs(x_h) < microT_tol && y_h < 0) angle = 0.5*pi;
  if(abs(x_h) < microT_tol && y_h > 0) angle = 1.5*pi; 
  return angle;
}



