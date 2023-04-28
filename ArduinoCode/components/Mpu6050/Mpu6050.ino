#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// variables for low pass filter

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

  // print values

  // print gz values
  Serial.print("original Alpha Z = "); Serial.print(accele_original[5]);
  Serial.print("\t");
  Serial.print("filtered Alpha Z = "); Serial.print(accele_filtered[5]);
  Serial.println();
  // print ax values
  Serial.print("original ax = "); Serial.print(accele_original[0]);
  Serial.print("\t");
  Serial.print("filtered ax = "); Serial.print(accele_filtered[0]);
  Serial.println();
  // draw seperation line
  Serial.println("--------------------------------------------------");

}
