#include <SPI.h>

const uint8_t SS_SLAVE = 10; // this drives the Nano's SS (D10)

void setup() {
  Serial.begin(115200);

  pinMode(SS_SLAVE, OUTPUT);
  digitalWrite(SS_SLAVE, HIGH); // deselect slave

  // IMPORTANT on Mega: keep pin 53 as OUTPUT so SPI stays in master mode
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0)); // start conservative
  Serial.println("Mega SPI Master ready.");
}

void receivePacket12(const uint8_t *pkt) {
  digitalWrite(SS_SLAVE, LOW);  // select slave

  Serial.print("Slave responded: ");
  for (int i = 0; i < 14; i++) {
    uint8_t response = SPI.transfer(pkt[i]); 
    Serial.print(response, HEX);
    Serial.print(" ");
  }
  Serial.println();

  digitalWrite(SS_SLAVE, HIGH);
}

void loop() {
  // Example packet: Pinky angle 45.00° => raw=4500 => LSB first
  uint16_t pinky_raw = 1500;
  uint16_t ring_raw = 4500;
  uint16_t middle_raw = 4500;
  uint16_t index_raw = 4500;
  uint8_t pkt[14] = {
    (uint8_t)(pinky_raw & 0xFF), (uint8_t)(pinky_raw >> 8),
    (uint8_t)(ring_raw & 0xFF), (uint8_t)(ring_raw >> 8), 
    (uint8_t)(middle_raw & 0xFF), (uint8_t)(middle_raw >> 8), 
    (uint8_t)(index_raw & 0xFF), (uint8_t)(index_raw >> 8), 
    0,0, 0,0, 0,0
  };

  receivePacket12(pkt);
  Serial.println("Receive 14-byte packet.");
  delay(300);
}
