/*
GNSS NEO-F10Nから緯度,経度,高度をBNO086から加速度を取得する
500ms周期でGNSSの値を取得する
50ms周期でIMUの値を取得する
処理はマルチタスクで行い,core1で3軸の値とGNSSの値を取得し
データが50個たまったらcore0のタスクでSDに書きこみとcat-M1でデータの送信を行う
*/

/*queue*/
#define MAX_QUE_NUM 10//キューのサイズ
#define MAX_QUE_SIZE 100//各キューの大きさ(バイト)
QueueHandle_t queue1;
QueueHandle_t queue2;
/*display*/
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

/*RTC*/
#include <PCF8563.h>
PCF8563 pcf;

/*SoftwareSerial*/
#include <SoftwareSerial.h>
SoftwareSerial softwareSerial(D0, D1);

/*gnss*/
#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS_SERIAL myGNSS;

/*HardwareSerial*/
#include <HardwareSerial.h>
HardwareSerial mySerial(0);

/*IMU*/
#include <Wire.h>
#define SAMPLE_RATE 50//IMUのサンプリングレート

/*SD*/
#include <SPI.h>
#include "SdFat.h"
SdFat SD;
#define SD_CS_PIN D2
File dataFile;

/*BLE client*/
#include "BLEUUID.h"
#include "BLEDevice.h"
// サーバーとキャラクタリスティックのUUIDを定義
static BLEUUID serviceUUID("12345678-1234-5678-1234-56789abcdef0");
static BLEUUID charUUID("abcdef01-1234-5678-1234-56789abcdef0");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;//キャラクタリスティックを保存しておく変数
static BLEAdvertisedDevice *myDevice;//探しているデバイスを保存しておく変数


uint8_t year;     
uint8_t month;   
uint8_t day;      
uint8_t hour;     
uint8_t minute;  
uint8_t second;
uint16_t millisecond;


// latitude・longitude・altitude
double latitude;
double longitude;
long altitude;

// acceleration data
String status = "";

unsigned long data_number = 0;//データのシーケンシャルナンバー
String filename;//SDカードに書き込みをする際のファイル名,setup()関数でファイル名を初期化する

//JSON
#include <ArduinoJson.h>
StaticJsonDocument<6144> doc;
JsonArray data;
JsonObject dataObject;
JsonArray ac;

bool logged = false;


void getImuTask(void *pvParameters) {//IMUの値を取得するタスク
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;  // 50ミリ秒の周期で3軸の値を更新
  TickType_t xLastWakeTime = xTaskGetTickCount();         // 最後に実行された時間を取得
  while (1) {
    if (doConnect) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
      } else {
        Serial.println("Failed to connect to the server.");
      }
      doConnect = false;
    }

    if (connected) {
      // サーバーと接続している間、必要な処理を記述
      // キャラクタリスティックから値を読み取る
      // std::string value = pRemoteCharacteristic->readValue().c_str();
      // if (value.size() >= 12) {
      //   const float *accelData = reinterpret_cast<const float *>(value.data());
      //   // Serial.printf("Acceleration - X: %.2f, Y: %.2f, Z: %.2f\n",accelData[0], accelData[1], accelData[2]);
      //   x = accelData[0];
      //   y = accelData[0];
      //   z = accelData[0];
      // } else {
      //   Serial.println("Error: Unexpected data length.");
      // }
    } else if (doScan) {
      BLEDevice::getScan()->start(0);  // 再スキャン
    }

    if(data_number % 3 == 0){
      StaticJsonDocument<96> doc;
      doc["lat"] = latitude;
      doc["lon"] = longitude;
      doc["alt"] = altitude;
      doc["status"] = status;
      char send_data[100];
      serializeJson(doc, send_data);
      xQueueSend(queue1, send_data, (TickType_t)0);
    }
    
    vTaskDelay(1000);
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);//センサの値を周期的に読み取るためvTaskDelayUntil関数を使用,xFrequency秒(50ms)ごとにwhileループが回る
  }
}

void getGnssAndWriteLogTask(void *pvParameters) {//GNSSの値を取得してSDカードに書き込みを行うタスク
  const TickType_t xFrequency = 500 / portTICK_PERIOD_MS;  // 500ミリ秒の周期で位置情報を更新してSDカードにデータを書き込み
  TickType_t xLastWakeTime = xTaskGetTickCount();         // 最後に実行された時間を取得
  while (1) {

    latitude = myGNSS.getLatitude() / 10000000.0;//緯度
    longitude = myGNSS.getLongitude() / 10000000.0;//経度
    altitude = myGNSS.getAltitude();//高度
    
    //xiaoの拡張ボードに今日の日付と時刻を表示する
    u8x8.setFont(u8x8_font_chroma48medium8_r);   // choose a suitable font
    u8x8.setCursor(0, 0);
    u8x8.print(day);
    u8x8.print("/");
    u8x8.print(month);
    u8x8.print("/");
    u8x8.print("20");
    u8x8.print(year);
    u8x8.setCursor(0, 1);
    u8x8.print(hour);
    u8x8.print(":");
    u8x8.print(minute);
    u8x8.print(":");
    u8x8.println(second);

    
    // vTaskDelayUntil(&xLastWakeTime, xFrequency);//センサの値を周期的に読み取るためvTaskDelayUntil関数を使用,xFrequency秒(500ms)ごとにwhileループが回る
    vTaskDelay(1);
  }
}

void sendDataTask(void *pvParameters)//キューからデータを取り出してM5Stamp CAT-M Moduleで送信するタスク
{
  char buf[100];

  while(1) {
    if(xQueueReceive(queue1, buf, pdMS_TO_TICKS(0))) {
      serial_send(buf);
    }
    vTaskDelay(700);
  }
  // ここにはこないが明示的にタスク終了を記載しておく。
  vTaskDelete(NULL);
}


bool sendATCommand(const char* command ,int delay_time = 500) {
  mySerial.write(command);
  delay(delay_time);
  unsigned long startTime = millis();
  String response = "";  // 応答を蓄積するための文字列バッファ
  
  while (mySerial.available()) {
    
    char c = mySerial.read();
    response += c;  // 応答をバッファに追加
    // Serial.print(c);  // デバッグのためにシリアルモニタに出力

    if (response.indexOf("ERROR") != -1) {
      Serial.println();
      return false;  // 失敗
    }
    if (response.indexOf("OK") != -1) {
      Serial.println();
      return true;  // 成功
    }
  }
  
  Serial.println();
  return true;
}


void serial_send(String data) {
  // Serial.printf("Q%d\n", uxQueueMessagesWaiting(queue));
  char command[100];
  sprintf(command,"AT+CASEND=0,%d\r\n",data.length());
  if (!sendATCommand(command)){
    Serial.println("Error: AT+CASEND");
  }

  if (!sendATCommand(data.c_str())){
    Serial.println("Error: Data could not be sent");
  }

  if (!sendATCommand("AT+CARECV=0,1200\r\n")) {
    Serial.println("Error: AT+CARECV");
  }
}

void writeSDTask(void *pvParameters){
  while(1) {
    if(logged){
      writeDataToSD();
      logged = false;
    }
    vTaskDelay(1);
  }
  // ここにはこないが明示的にタスク終了を記載しておく。
  vTaskDelete(NULL);
}



static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  // Serial.print("Notify callback for characteristic ");
  // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  // Serial.print(" of data length ");
  // Serial.println(length);

  // pDataを文字列として扱う
  std::string receivedData(reinterpret_cast<char*>(pData), length);
  status = receivedData.c_str();
  Serial.printf("status - %s\n", receivedData.c_str());
}

//MyClientCallbackクラスは、BLEクライアントがサーバーに対して行う接続・切断操作に応じた処理を実装するコールバッククラスです。
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {}//BLEクライアントがサーバーに接続した際に呼び出されます。pclient: 接続イベントを発生させたBLEクライアントへのポインタです。

  void onDisconnect(BLEClient *pclient) {//BLEクライアントがサーバーから切断された際に呼び出されます。pclient: 切断イベントを発生させたBLEクライアントへのポインタです。
    connected = false;//接続状況をfalseに
    Serial.println("Disconnected from server");
  }
};

//サーバー（BLEデバイス）への接続を試み、成功した場合はtrue、失敗した場合はfalseを返す関数
bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());//接続対象のBLEデバイスのアドレスを取得してシリアルに出力

  BLEClient *pClient = BLEDevice::createClient();//BLEクライアントを作成し、サーバーへの接続を管理します。pClient: BLEクライアントを指すポインタ。
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());//BLEクライアントにコールバックを設定します。

  // BLEデバイス(サーバー)に接続
  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  // サービスを取得
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);//指定されたサービスUUID（serviceUUID）に対応するリモートサービスを取得
  if (pRemoteService == nullptr) {//nullptrの場合はサービスが見つからなかったことを意味
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();//サービスが見つからない場合、エラーメッセージを出力し、接続を切断して終了します。
    return false;
  }
  Serial.println(" - Found our service");

  // キャラクタリスティックを取得
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);//指定されたキャラクタリスティックUUID（charUUID）に対応するリモートキャラクタリスティックを取得します。
  if (pRemoteCharacteristic == nullptr) {//nullptrの場合はキャラクタリスティックが見つからなかったことを意味します。
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // 初期値を読み取る
  if (pRemoteCharacteristic->canRead()) {//キャラクタリスティックが読み取り可能かどうかを確認します。
    String value = pRemoteCharacteristic->readValue();//readValue(): キャラクタリスティックの値を読み取ります。

    Serial.printf("Initial status - %s\n", value);
  }

  // 通知を登録
  if (pRemoteCharacteristic->canNotify()) {//キャラクタリスティックが通知可能かどうかを確認します。
    pRemoteCharacteristic->registerForNotify(notifyCallback);//通知を受け取るためのコールバック関数（notifyCallback）を登録します。通知が発生するとこの関数が呼び出されます。
  }

  connected = true;
  return true;//サーバーへの接続と設定が成功したことを示します。
}

//スキャン中にアドバタイズされた Bluetooth デバイスが見つかったときに呼び出される
//MyAdvertisedDeviceCallbacksクラスは、BLEAdvertisedDeviceCallbacksを継承している
//BLEAdvertisedDeviceCallbacks: BLEスキャン中にデバイスが見つかったときに呼び出されるイベント処理を提供するための基底クラス
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  //onResult関数はアドバタイズされたデバイスが見つかるたびに呼び出されます。
  //引数advertisedDeviceには、見つかったデバイスの情報が含まれます。
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());//デバイスの情報をシリアルに出力

    /*advertisedDevice.haveServiceUUID(): アドバタイズデバイスがサービスUUIDを持っているかどうかを確認します。
    *advertisedDevice.isAdvertisingService(serviceUUID): デバイスが指定されたserviceUUIDをアドバタイズしているかを確認します。
    */
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();//スキャンを停止します。目的のデバイスが見つかった後、不要なスキャンを止めるためです。
      myDevice = new BLEAdvertisedDevice(advertisedDevice);//見つかったデバイスの情報をmyDeviceに保存
      doConnect = true;//デバイスに接続されたフラグを設定
      doScan = true;//スキャン実行のフラグを立てる
    }
  }
};



void setup() {
  Serial.begin(115200);
  delay(10);

  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("fail to initialize sd card");
    return;
  }

  softwareSerial.begin(38400); // The NEO-F10N defaults to 38400 baud
  while (myGNSS.begin(softwareSerial) == false) // Connect to the u-blox module using softwareSerial(UART)
  {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Retrying..."));
    delay(1000);
  }
  if (myGNSS.getModuleInfo())
  {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow()); // Returns uint8_t

    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType()); // Returns HPG, SPG, SPGL1L5 etc. as (const char *)

    Serial.print(F("PROTVER: "));
    Serial.print(myGNSS.getProtocolVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getProtocolVersionLow()); // Returns uint8_t

    Serial.print(F("MOD: "));
    Serial.println(myGNSS.getModuleName()); // Returns ZED-F9P, MAX-M10S etc. as (const char *)
  }
  else
    Serial.println(F("Error: could not read module info!"));

  myGNSS.setUART1Output(COM_TYPE_UBX);               // Set the UART1 port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR

  // myGNSS.setNavigationFrequency(4);// Set output to 2 times a second
  myGNSS.setMeasurementRate(500);
  myGNSS.setVal8(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1); // Make sure the GPS L5 band is enabled (needed on the NEO-F9P)

  myGNSS.setGPSL5HealthOverride(true); // Mark L5 signals as healthy - store in RAM and BBR

  myGNSS.setLNAMode(SFE_UBLOX_LNA_MODE_NORMAL); // Set the LNA gain to normal (full). Other options: LOWGAIN, BYPASS

  myGNSS.softwareResetGNSSOnly(); // Restart the GNSS to apply the L5 health override

  mySerial.begin(115200);//softwareSerialを初期化

  Wire.begin();

  BLEDevice::init("");//BLEデバイスを初期化
  BLEScan *pBLEScan = BLEDevice::getScan();//BLEスキャンを管理するBLEScanオブジェクトを取得
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());//スキャン中に見つかったアドバタイズされたデバイスを処理するためのコールバック関数を設定
  pBLEScan->setInterval(1349);//スキャン間隔
  pBLEScan->setWindow(449);//ウィンドウ値
  pBLEScan->setActiveScan(true);//アクティブスキャンを有効にする。アクティブスキャンは、デバイスにSCAN_REQを送信して、アドバタイジング・パケットに収まりきらなかった情報をさらに取得します
  pBLEScan->start(5, false);


  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("fail to initialize sd card");
    return;
  }

  char date[30];
  /*
  信頼できるデータを得るには4つ以上の衛星が必要ということにする
  こうすることで正確な情報が得られなかった時にファイル名が不正な時刻のものになるのを防ぐ
  */
  while(myGNSS.getSIV() < 4 ){
    Serial.println("getting DateTime...");
    Serial.printf("Number of Satellites is %u\n",myGNSS.getSIV());
    delay(500);
  }
  uint16_t year = myGNSS.getYear();
  uint8_t month = myGNSS.getMonth();
  uint8_t day = myGNSS.getDay();
  uint8_t hour = myGNSS.getHour();
  uint8_t minute = myGNSS.getMinute();
  uint8_t second = myGNSS.getSecond();
  // sprintf(date,"%04u-%02u-%02u-%02u-%02u-%02u.csv",year,month,day,hour,minute,second);//SDカードに作成するファイル名を生成
  // filename = date;
  createFilename(year, month, day, hour, minute, second);

  // pcf.init();
  // pcf.stopClock();
  // pcf.setYear(year % 100);
  // pcf.setMonth(month);
  // pcf.setDay(day);
  // pcf.setHour(hour);
  // pcf.setMinut(minute);
  // pcf.setSecond(second);
  // pcf.startClock();

  u8x8.begin();//OLEDディスプレイの初期化
  u8x8.setFlipMode(1);//ディスプレイの上下を反転

  delay(100);
  if (!sendATCommand("AT+CNACT=0,0\r\n")) {
    Serial.println("Error: AT+CNACT");
  }

  if (!sendATCommand("AT+CGDCONT=1,\"IP\",\"soracom.io\"\r\n")) {
    Serial.println("Error: AT+CGDCONT=1,\"IP\",\"soracom.io\"");
  }

  if (!sendATCommand("AT+CGDCONT?\r\n")) {
    Serial.println("Error: AT+CGDCONT?");
  }

  if (!sendATCommand("AT+CFUN=1\r\n")) {
    Serial.println("Error: AT+CFUN=1");
  }

  if (!sendATCommand("AT+CGATT=1\r\n")) {
    Serial.println("Error: AT+CGATT=1");
  }

  if (!sendATCommand("AT+CGATT?\r\n")) {
    Serial.println("Error: AT+CGATT?");
  }

  if (!sendATCommand("AT+CGACT=1,1\r\n")) {
    Serial.println("Error: AT+CGACT=?");
  }

  if (!sendATCommand("AT+CGACT=?\r\n")) {
    Serial.println("Error: AT+CGACT=?");
  }


  if (!sendATCommand("AT+CPIN?\r\n")) {
    Serial.println("Error: AT+CPIN");
  }

  if (!sendATCommand("AT+CSQ\r\n")) {
    Serial.println("Error: AT+CSQ");
  }

  if (!sendATCommand("AT+CASTATE?\r\n")) {
    Serial.println("Error: AT+CASTATE?");
  }

  if (!sendATCommand("AT+COPS?\r\n")) {
    Serial.println("Error: AT+COPS?");
  }

  if (!sendATCommand("AT+CGNAPN\r\n")) {
    Serial.println("Error: AT+CGNAPN");
  }
  
  if (!sendATCommand("AT+CNCFG=0,1,\"soracom.io\" \r\n")) {
    Serial.println("Error: AT+CNCFG=0,1,\"soracom.io\" ");
  }

  if (!sendATCommand("AT+CNCFG?\r\n")) {
    Serial.println("Error: AT+CNCFG?");
  }

  if (!sendATCommand("AT+CNACT=0,1\r\n")) {
    Serial.println("Error: AT+CNACT");
  }

  if (!sendATCommand("AT+CNACT?\r\n")) {
    Serial.println("Error: AT+CNACT?");
  }
  
  if (!sendATCommand("AT+CACID=0\r\n")) {
    Serial.println("Error: AT+CACID=0");
  }

  if (!sendATCommand("AT+CACID?\r\n")) {
    Serial.println("Error: AT+CACID?");
  }

  if (!sendATCommand("AT+CASSLCFG=0,\"SSL\",0\r\n")) {
    Serial.println("Error: AT+CASSLCFG?");
  }

  if (!sendATCommand("AT+CAOPEN=0,0,\"UDP\",\"harvest.soracom.io\",8514\r\n")) {
    Serial.println("Error: AT+CAOPEN");
  }

  //キューを作成
  queue1 = xQueueCreate(MAX_QUE_NUM, MAX_QUE_SIZE);
  queue2 = xQueueCreate(75, 90);

  xTaskCreateUniversal(//ここでIMUの値を取得するタスクを生成
    getImuTask,//作成するタスク関数
    "getImuTask",//タスク名
    8192,//スタックメモリサイズ(4096or8192)
    NULL,//起動パラメータ
    2,//優先度2(数字が大きい程優先度が高い)
    NULL,//タスクハンドルのポインタ
    0//core1でタスクを処理する
  );
  xTaskCreateUniversal(//ここでgnssの値の取得とSDカードに書き込みをするタスクを生成
    getGnssAndWriteLogTask,
    "getGnssAndWriteLogTask",
    8192,
    NULL,
    2,//優先度3で行う
    NULL,
    1//core1でタスクを処理する
  );
  xTaskCreateUniversal(//データをCAT-Mで送信するタスクを生成
    sendDataTask,
    "sendDataTask",
    8192,
    NULL,
    1,//優先度1
    NULL,
    1//core0でタスクを処理する
  );
  // xTaskCreateUniversal(//データをCAT-Mで送信するタスクを生成
  //   writeSDTask,
  //   "writeSDTask",
  //   4096,
  //   NULL,
  //   0,//優先度1
  //   NULL,
  //   0//core0でタスクを処理する
  // );

  delay(100);
}

void writeDataToSD()//SDカードに書き込みを行う関数
{
  // ファイルを開く（ファイル名は年-月-日-時-分-秒.csv）
  dataFile = SD.open(filename, O_RDWR | O_CREAT | O_AT_END);

  // ファイルが開けたか確認
  if (dataFile)
  {
    // データをファイルに書き込む
    char buf[90];
    int count = uxQueueMessagesWaiting(queue2);
    for (int i = 0; i < count; i++)
    {
      xQueueReceive(queue2, buf, pdMS_TO_TICKS(0));
      dataFile.println(buf);
    }
    dataFile.close(); // ファイルを閉じる
    Serial.println("Data written to SD card.");
  }
  else
  {
    Serial.println("Error opening data file!");
  }
}

void createFilename(uint16_t year, uint8_t month, uint8_t day, uint8_t hourUTC, uint8_t minute, uint8_t second) {
    // 日本時間を計算
    uint8_t hourJST = hourUTC + 9; // UTC+9時間
    if (hourJST >= 24) {
        hourJST -= 24; // 24時間を超える場合は日付を進める
        day++;

        // 月末かどうかを判定して進める（簡易的に処理）
        uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) {
            daysInMonth[1] = 29; // 閏年の場合は2月を29日とする
        }

        if (day > daysInMonth[month - 1]) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
    }
    char date[32]; // ファイル名バッファ 
    // ファイル名を生成
    snprintf(date, sizeof(date), "%04u-%02u-%02u-%02u-%02u-%02u.csv", year, month, day, hourJST, minute, second);
    filename = date; // ファイル名のポインタを設定

    pcf.init();
    pcf.stopClock();
    pcf.setYear(year % 100);
    pcf.setMonth(month);
    pcf.setDay(day);
    pcf.setHour(hourJST);
    pcf.setMinut(minute);
    pcf.setSecond(second);
    pcf.startClock();
}


void loop() {
  delay(1);
}
