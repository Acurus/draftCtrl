#include <Arduino.h>


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int result = myFunction(500, 500);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(result);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(result);                       // wait for a second
  Serial.println("Loop completed: " + String(result) + " ms delay");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}