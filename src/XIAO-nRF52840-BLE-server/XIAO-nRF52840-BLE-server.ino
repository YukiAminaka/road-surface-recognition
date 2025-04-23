#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <Road-Surface-Recognition_inferencing.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

String status = "UNCERTAIN";

BLEService service("12345678-1234-5678-1234-56789abcdef0");
BLEStringCharacteristic accelerationCharacteristic("abcdef01-1234-5678-1234-56789abcdef0",BLERead | BLENotify,20);

void setup() {
  Serial.begin(9600);
  // while (!Serial);

  if (myIMU.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("Xiao_Sense");//スキャン時に表示する文字の設定
  BLE.setAdvertisedService(service);//サービスUUIDを追加

  service.addCharacteristic(accelerationCharacteristic);

  BLE.addService(service);//サービスを追加
  BLE.advertise();//アドバタイジングパケットの送信

  Serial.println("BLE device active, waiting for connections...");
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}


void loop() {
  // BLE.poll();
  // while (BLE.connected()) {
  //   updateSubscribedCharacteristics();
  //   delay(100);
  // }
  static unsigned long previousMillis = 0;//ここは最初の関数呼び出し時に一度だけ 0 に初期化される
  const long interval = 1900; // 2秒

  BLE.poll();

  unsigned long currentMillis = millis();
  if (BLE.connected()) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      updateSubscribedCharacteristics();
    }
    delay(100);
  }
}

void updateSubscribedCharacteristics() {
  if (accelerationCharacteristic.subscribed()) {//BLEクライアントが accelerationCharacteristic をサブスクライブしているかを確認
    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix] = myIMU.readFloatAccelX();
        buffer[ix+1] = myIMU.readFloatAccelY();
        buffer[ix+2] = myIMU.readFloatAccelZ();

        for (int i = 0; i < 3; i++) {
          if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
              buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
          }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
    #endif

    

    if (result.classification[0].value > 0.5) {
      status = "IDLE";
    }
    if (result.classification[1].value > 0.5) {
      status = "MEDIUM";
    }
    if (result.classification[2].value > 0.5) {
      status = "ROUGH";
    }
    if (result.classification[3].value > 0.5) {
      status = "SMOOTH";
    }
    if (result.classification[4].value > 0.5) {
      status = "SPRINT";
    }
    accelerationCharacteristic.writeValue(status);
  }

}
