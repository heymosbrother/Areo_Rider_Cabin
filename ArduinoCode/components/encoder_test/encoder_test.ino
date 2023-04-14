/* This code tests the functionality of encoders */

#pragma region[Encoder parameters]
// Define two pins for encoder A, B channels

// motor1 encoder
const int ENC1A = 2; // yellow line
const int ENC1B = 13; // green line

// motor2 encoder
const int ENC2A = 3; // yellow line
const int ENC2B = 11; // green line

// records the position of the encoder
int enc1pos = 0;
int enc2pos = 0;
#pragma endregion

void setup() {
    Serial.begin(9600);

    // Initiate the pins for the encoders
    pinMode(ENC1A, INPUT);
    pinMode(ENC1B, INPUT);
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);

    // set up attachInterrupt function to call specific function immediately when pin value is changed
    attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
}

void loop() {
    // Test the encoder direction by showing A, B channel values
    // store the value generated when the encoders spin
    int a1 = digitalRead(ENC1A);
    int b1 = digitalRead(ENC1B);
    int a2 = digitalRead(ENC2A);
    int b2 = digitalRead(ENC2B);

    // print encoder 1 status
    Serial.print("Encoder 1 Channel A: ");
    Serial.print(a1);
    Serial.print(" Channel B: ");
    Serial.print(b1);
    Serial.print("  position: ");
    Serial.print(enc1pos);

    Serial.print(" \t");

    // print encoder 2 status
    Serial.print("Encoder 2 Channel A: ");
    Serial.print(a2);
    Serial.print(" Channel B: ");
    Serial.print(b2);
    Serial.print("  position: ");
    Serial.print(enc2pos);

    Serial.println();
}

// the readEncoder function
void readEncoder1() {
  int bValue = digitalRead(ENC1B);
  if (bValue > 0) {
    enc1pos--;
  }
  else{
    enc1pos++;
  }
}
void readEncoder2() {
  int bValue = digitalRead(ENC2B);
  if (bValue > 0) {
    enc2pos++;
  }
  else {
    enc2pos--;
  }
}
