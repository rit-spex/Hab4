#include <Arduino.h>

void setup() {
    // put your setup code here, to run once:
    Wire.begin(0x77);
    Serial.begin(9600);
    Wire.onRequest(getAltitude);
}

void loop() {
    // put your main code here, to run repeatedly:
    //Serial.println("whatever");
    delay(100);
}

void getAltitude(){
    float alt = 42.0;
    Wire.send(alt);
}
