
int musclePin[2] = {   // define Arduino input locations (A0 and A1 are the inputs for the EMG shield)
  A0, A1};
char printBuff[10];    // allocate space for reading voltages

void setup(){
  Serial.begin(115200); // this number is the Baudrate, and it must match the serial setup in MATLAB
  delay(5000);          // evoke a delay to let the serial setup
}
 
void loop(){
  long startTime = micros();  //start timer
  
  sprintf(printBuff,"%d %d",          // read voltages
  analogRead(musclePin[0]),analogRead(musclePin[1])
  );
  Serial.println(printBuff);        // write the voltages to serial
  
  long stopTime = micros() - startTime; // determine how long it took to write
  if(stopTime < 1000) // force a maximum sampling rate of 1 kHz
  {
    delayMicroseconds(1000-stopTime);
  }
}
