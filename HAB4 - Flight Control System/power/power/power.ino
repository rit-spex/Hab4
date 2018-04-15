#define voltageThreshold 724
#define numConsecReads 10

int analogIn = A0;    // select the input pin for the potentiometer
int fetControl = 13;      // select the pin for the LED
int value = 0;  // variable to store the value coming from the sensor
int numBelow = 0;
boolean triggered = false;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);             // set up serial for debugging purposses
  pinMode(fetControl, OUTPUT); 

}

void loop() {
  // put your main code here, to run repeatedly:

  // read a0 , conver to analog with map, based on the schematic numbers, turn d13 digital on or off
  // if a0 is too low turn d13 off

  // to activate cubesat, toggle (with matt's board) ptc10 HIGH to deploy cubesat LOW to turn it off
  // cutdown is PTC 8
  
  value = analogRead(analogIn);
  //float corrected = map(value,0,1023,0.0,5.0);

  Serial.print("Value: ");
  Serial.println(value);
  
  if(!triggered){
    if(value < voltageThreshold){
      Serial.print("Value is below voltage threshhold!\n");
      numBelow++;
      if(numBelow == numConsecReads){
        digitalWrite(fetControl, LOW);
        triggered = true;
        Serial.print("Triggered\n");
      }
    }
    else{
      numBelow = 0;
      Serial.print("numbelow reset, no issue\n");
      digitalWrite(fetControl, HIGH);
    }
  }
  
  delay(1000);                     // wait a second
}
