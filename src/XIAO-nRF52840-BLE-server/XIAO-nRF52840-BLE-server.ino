#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

BLEService service("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic accelerationCharacteristic("abcdef01-1234-5678-1234-56789abcdef0", BLENotify, 3 * sizeof(float));
BLEFloatCharacteristic resistanceCharacteristic("abcdef02-1234-5678-1234-56789abcdef0", BLENotify);

void setup() {
  Serial.begin(9600);
  // while (!Serial);

  if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("Xiao_Sense");//スキャン時に表示する文字の設定
  BLE.setAdvertisedService(service);//サービスUUIDを追加

  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(resistanceCharacteristic);//サービスにキャラクタリスティックを追加

  BLE.addService(service);//サービスを追加
  BLE.advertise();//アドバタイジングパケットの送信

  Serial.println("BLE device active, waiting for connections...");
}

void loop() {
  BLE.poll();
  while (BLE.connected()) {
    updateSubscribedCharacteristics();
    delay(100);
  }
}

void updateSubscribedCharacteristics() {
  if (accelerationCharacteristic.subscribed()) {
    float acceleration[3] = {
      myIMU.readFloatAccelX(),
      myIMU.readFloatAccelY(),
      myIMU.readFloatAccelZ()
    };
    char buffer[60];
    sprintf(buffer,"Accel: X=%f,Y=%f,Z=%f\n",acceleration[0],acceleration[1],acceleration[2]);
    Serial.print(buffer);
    // Serial.print("Accel: X="); Serial.print(acceleration[0]);
    // Serial.print(" Y="); Serial.print(acceleration[1]);
    // Serial.print(" Z="); Serial.println(acceleration[2]);

    accelerationCharacteristic.writeValue((byte*)acceleration, sizeof(acceleration));
  }

  if (resistanceCharacteristic.subscribed()) {
    uint16_t voltage = analogRead(A0);
    float resistance = voltage / 1024.0f;
    Serial.print("Resistance: "); Serial.println(resistance);

    resistanceCharacteristic.writeValue(resistance);
  }
}
