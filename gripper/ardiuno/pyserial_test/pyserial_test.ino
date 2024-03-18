

// Serial read stuff
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;

// Define params
#define vacuum_on 1
#define vacuum_off 2
#define fingers_engaged 3
#define fingers_disengaged 4


void setup() {
  Serial.begin(115200);
  while (!Serial);
  clearInputBuffer();

  delay(100);
}


void loop() {
  recvWithStartEndMarker();
  parseCommands();
}

void recvWithStartEndMarker() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseCommands() {
  int c = 0;
  int c_idx = 0;
  int t_idx = 0;
  char temp[32];

  if (newData == true) {
    // Convert serial monitor value to int and cast as float
    int len = strlen(receivedChars);
    c = (float) atoi(receivedChars);
    newData = false;

    // Manage the serial input accordingly
    if (c==vacuum_on){
      vacuumOn();
    }
    else if (c==vacuum_off){
      vacuumOff();
    }
    else if (c==fingers_engaged){
      engageFingers();
    }
    else if (c==fingers_disengaged){
      disengageFingers();
    }
    else{
      Serial.println("Unknown input recieved");
    }
  }
}

void vacuumOn(){
  Serial.println("Arduino: turning vacuum on");
}

void vacuumOff(){
  Serial.println("Arduino: turning vacuum off");
}

void engageFingers(){
  Serial.println("Arduino: engaging fingers");
}

void disengageFingers(){
  Serial.println("Arduino: disengaging fingers");
}

void testInput(float c){
  Serial.println("recieved input");
}

void sendIntSerial(int x) {
  uint8_t LSB = x;
  uint8_t MSB = x >> 8;
  Serial.write(MSB);
  Serial.write(LSB);
}

void clearInputBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
