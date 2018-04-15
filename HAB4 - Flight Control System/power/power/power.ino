///////////////////////
/// RIT SPEX HABIV
/// Power Delivery Board Software
/// April 28th, 2018
///
/// Thomas Hall
/// Daniel Mitchel
////////////////////////

#define voltageThreshold 724  // data member for 3.54 V
#define numConsecReads 10     // number of consecutive low reads to shut off the battery

int analogIn = A0;            // the input pin for the power 
int fetControl = 13;          // the pin for the fetControl
int value = 0;                // variable to store the current power reading
int numBelow = 0;             // number of reading that have been below (in a row)
boolean triggered = false;    // variable to see wether we have triggered the fetControl

void setup() {
  Serial.begin(9600);             // set up serial for debugging purposses
  pinMode(fetControl, OUTPUT);    // set the output for the fetControl
}

void loop() { 

  // read the value in 
  value = analogRead(analogIn);

  // print it for debugging purposses
  Serial.print("Value: ");
  Serial.println(value);

  // do the fetControl calculation
  if(!triggered){

    // the value is below the threshold
    if(value < voltageThreshold){
      Serial.print("Value is below voltage threshhold!\n");
      numBelow++;
      // if we have surpassed the number of low consectuive low reads we hold the fetControl pin high
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
