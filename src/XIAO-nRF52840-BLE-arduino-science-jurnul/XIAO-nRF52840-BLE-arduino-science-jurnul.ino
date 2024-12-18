#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

#define CONVERT_G_TO_MS2 9.80665f

#define SCIENCE_KIT_UUID(val) ("555a0002-" val "-467a-9538-01f0652c74e8")
BLEService service(SCIENCE_KIT_UUID("0000"));
BLECharacteristic accelerationCharacteristic(SCIENCE_KIT_UUID("0011"), BLENotify, 3 * sizeof(float));
BLEFloatCharacteristic resistanceCharacteristic(SCIENCE_KIT_UUID("0020"), BLENotify);

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
    float x = myIMU.readFloatAccelX() * CONVERT_G_TO_MS2 * -0.1;
    float y = myIMU.readFloatAccelY() * CONVERT_G_TO_MS2 * -0.1;
    float z = myIMU.readFloatAccelZ() * CONVERT_G_TO_MS2 * 0.1;
    float acceleration[3];
    acceleration[0] = x;
    acceleration[1] = y;
    acceleration[2] = z;
    // Serial.print("Accel: X="); Serial.print(acceleration[0]);
    // Serial.print(" Y="); Serial.print(acceleration[1]);
    // Serial.print(" Z="); Serial.println(acceleration[2]);
    char buffer[60];
    sprintf(buffer,"Accel: X=%f,Y=%f,Z=%f\n",x * -1,y*-1 ,z);
    Serial.print(buffer);
    accelerationCharacteristic.writeValue((byte*)acceleration, sizeof(acceleration));
  }

  if (resistanceCharacteristic.subscribed()) {
    uint16_t voltage = analogRead(A0);
    float resistance = voltage / 1024.0f;
    Serial.print("Resistance: "); Serial.println(resistance);

    resistanceCharacteristic.writeValue(resistance);
  }
}
