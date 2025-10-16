#include <nvs_flash.h>
#include <Arduino.h>

void setup() {
  nvs_flash_erase(); // erase the NVS partition and...
  nvs_flash_init(); // initialize the NVS partition.
  pinMode(10, OUTPUT);

  digitalWrite(10, HIGH);
  delay(1000);
  digitalWrite(10, LOW);
  while(true);
}

void loop() {

}