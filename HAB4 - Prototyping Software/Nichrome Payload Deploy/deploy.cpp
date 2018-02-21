boolean payloadReleased = false; //controls deployment

//triggers the external payload deployment switch
void deployPayload() {
      digitalWrite(RELAY_2, HIGH); // Nichrome wire deploys cubesat solar panels
      Serial.println("relay raised HIGH");
      payloadReleased = true;
}