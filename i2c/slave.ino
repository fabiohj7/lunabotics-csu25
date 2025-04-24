#include <Wire.h>

#define SLAVE_ADDR 0x20  // I2C Address of the slave device

void setup() {
  // Start the I2C communication as a slave with address 8
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveData);  // Register the receive function
  Serial.begin(9600);  // Start the Serial communication for debugging
  Serial.println("Arduino I2C Slave Ready");
}

void loop() {
  // Main loop does nothing; it just waits for I2C communication
  delay(100);
}

// This function is called when the master sends data
void receiveData(int byteCount) {
  while (Wire.available()) {
    char receivedChar = Wire.read();  // Read the incoming byte
    Serial.print("Received: ");
    Serial.println(receivedChar);
    
    // You can add additional logic to handle different data
    if (receivedChar == 'A') {
      Serial.println("Command A received");
    } else if (receivedChar == 'B') {
      Serial.println("Command B received");
    }
  }
}
