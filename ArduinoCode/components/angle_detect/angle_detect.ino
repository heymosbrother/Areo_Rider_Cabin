#include <Wire.h>
#include <MPU6050.h>

MPU6050 accelgyro;

// variables for low pass filter

//  Alpha Z
int16_t prev_alphaZ_original;
int16_t prev_alphaZ_filtered;

int16_t prev_accele_orginal[] = {0,0,0,0,0,0};
int16_t prev_accele_filtered[] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  accelgyro.initialize();
}

void loop() {
  int16_t accele_original[] = {0,0,0,0,0,0}; // ax, ay, az, gx, gy, gz
  accelgyro.getMotion6(&accele_original[0], &accele_original[1], &accele_original[2], &accele_original[3], &accele_original[4], &accele_original[5]);
  
  // low pass filter
  int16_t accele_filtered[] = {0,0,0,0,0,0};
  
  for(int i = 0; i < 6; i++){
    accele_filtered[i] = 0.969 * prev_accele_filtered[i] + 0.0155 * accele_original[i] + 0.0155 * prev_accele_orginal[i];

    // update previous values
    prev_accele_orginal[i] = accele_original[i];
    prev_accele_filtered[i] = accele_filtered[i];
  }

  // print values

  // print gz values
  // Serial.print("original ax = "); Serial.print(accele_original[0]);
  // Serial.print("\t");
  Serial.print("filtered ax = "); Serial.print(accele_filtered[0]);
  // Serial.println();
  // print ax values
  // Serial.print("original az = "); Serial.print(accele_original[2]);
  Serial.print("\t");
  Serial.print("filtered az = "); Serial.print(accele_filtered[2]);
  Serial.print("\t");
  // print ay values
  Serial.print("filtered ay = ");  Serial.print(accele_filtered[1]);
  Serial.println();
  // draw seperation line
  Serial.println("--------------------------------------------------");

}
