
// Adapted method for calibration of the magnetometer from: https://github.com/kriswiner/MPU-6050/wiki/Simple-and-Effective-Magnetometer-Calibration
// Use this method onyl from time to time in order to determine the values for the magBias[3]-vector
void magcalMPU9150(float * dest1)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias_raw[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0}, mag_temp[3] = {0, 0, 0};
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  //delay(4000);  //Wait for 4 second to give the used some time to take the device in his hands
  sample_count = 512;
  //Gather data...this will take about 16 seconds
  for(ii = 0; ii < sample_count; ii++) {
    wdt_reset(); 
    toggleReadMagData;
    delay(10);
    readMagData(mag_temp); // Read the mag data

    // Safe min max value by direct comaprision in each loop
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      { Serial.print(mag_temp[jj]); Serial.print(" ");}
    }
    Serial.println();
    delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
  }
  Serial.print("mag x max:"); Serial.print(mag_max[0]); Serial.print(" min: "); Serial.println(mag_min[0]);
  Serial.print("mag y max:"); Serial.print(mag_max[1]); Serial.print(" min: "); Serial.println(mag_min[1]);
  Serial.print("mag z max:"); Serial.print(mag_max[2]); Serial.print(" min: "); Serial.println(mag_min[2]);
  mag_bias_raw[0] = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
  mag_bias_raw[1] = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
  mag_bias_raw[2] = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts
  dest1[0] = (float) mag_bias_raw[0]*compRes*magCalibration[0]; // save mag biases in mG for main program
  dest1[1] = (float) mag_bias_raw[1]*compRes*magCalibration[1];
  dest1[2] = (float) mag_bias_raw[2]*compRes*magCalibration[2];
  Serial.println("Mag Calibration done! ");
}

