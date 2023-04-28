#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// variables for low pass filter

//  Alpha Z
int16_t prev_alphaZ_original;
int16_t prev_alphaZ_filtered;

int16_t prev_accele_orginal[] = {0,0,0,0,0,0};
int16_t prev_accele_filtered[] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
}

void loop() {
  int16_t accele_original[] = {0,0,0,0,0,0}; // ax, ay, az, gx, gy, gz
  mpu.getMotion6(&accele_original[0], &accele_original[1], &accele_original[2], &accele_original[3], &accele_original[4], &accele_original[5]);
  
  // low pass filter
  int16_t accele_filtered[] = {0,0,0,0,0,0};
  
  for(int i = 0; i < 6; i++){
    accele_filtered[i] = 0.969 * prev_accele_filtered[i] + 0.0155 * accele_original[i] + 0.0155 * prev_accele_orginal[i];

    // update previous values
    prev_accele_orginal[i] = accele_original[i];
    prev_accele_filtered[i] = accele_filtered[i];
  }

  // Detect row and pitch angle with accelerometer data
  float row, pitch;

  //    need futther calibration
  row = atan2(accele_filtered[0], sqrt(accele_filtered[1] * accele_filtered[1] + accele_filtered[2] * accele_filtered[2])) * 180 / PI;
  pitch = atan2(accele_filtered[1], sqrt(accele_filtered[0] * accele_filtered[0] + accele_filtered[2] * accele_filtered[2])) * 180 / PI;

  // Print angle values
  Serial.print("row = "); Serial.print(row); Serial.print("\t");
  Serial.print("pitch = "); Serial.print(pitch); Serial.print("\t");
  Serial.println();
}
