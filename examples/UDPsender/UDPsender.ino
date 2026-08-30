#include <Arduino.h>

// ATコマンドを送信し、タイムアウトまでOK/ERRORを待つ
// 戻り値: true=OK受信, false=ERROR受信またはタイムアウト
bool sendATCommand(const char* command, int timeout_ms = 3000) {
  Serial.print(">> ");
  Serial.print(command);

  Serial1.write(command);

  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < (unsigned long)timeout_ms) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
      Serial.print(c);
    }
    if (response.indexOf("ERROR") != -1) {
      Serial.println("[FAIL]");
      return false;
    }
    if (response.indexOf("OK") != -1) {
      return true;
    }
  }

  Serial.println("[TIMEOUT]");
  return false;
}

// 特定の文字列が応答に現れるまで待つ (URC待ち用)
bool waitForResponse(const char* expected, int timeout_ms = 10000) {
  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < (unsigned long)timeout_ms) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
      Serial.print(c);
    }
    if (response.indexOf(expected) != -1) {
      return true;
    }
    if (response.indexOf("ERROR") != -1) {
      return false;
    }
  }

  Serial.println("[TIMEOUT]");
  return false;
}

// UDPデータ送信
// AT+CASEND は '>' プロンプト後にデータを送る仕様
void serial_send(String data) {
  char command[64];
  sprintf(command, "AT+CASEND=0,%d\r\n", data.length());

  Serial.print(">> ");
  Serial.print(command);
  Serial1.write(command);

  // '>' プロンプト待ち
  unsigned long startTime = millis();
  bool promptReceived = false;
  while (millis() - startTime < 3000) {
    if (Serial1.available()) {
      char c = Serial1.read();
      Serial.print(c);
      if (c == '>') {
        promptReceived = true;
        delay(100);  // プロンプト後の安定待ち
        break;
      }
    }
  }

  if (!promptReceived) {
    Serial.println("[ERROR] '>' プロンプトを受信できませんでした");
    return;
  }

  // データ送信
  Serial.print(">> (data) ");
  Serial.println(data);
  Serial1.write(data.c_str());

  // 送信結果待ち
  if (!waitForResponse("OK", 5000)) {
    Serial.println("[ERROR] データ送信失敗");
    return;
  }

  Serial.println("[OK] データ送信成功");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  delay(1000);

  Serial.println("=== SIM7080G CAT-M/NB-IoT 初期化開始 ===");

  // モジュール基本動作確認
  Serial.println("--- [1/11] モジュール確認 ---");
  if (!sendATCommand("AT\r\n")) {
    Serial.println("[ERROR] モジュールが応答しません。接続を確認してください。");
    return;
  }
  Serial.println("[OK] モジュール応答確認");

  // フル機能モードに設定
  Serial.println("--- [2/11] フル機能モード設定 ---");
  if (!sendATCommand("AT+CFUN=1\r\n", 5000)) {
    Serial.println("[ERROR] AT+CFUN=1 失敗");
  } else {
    Serial.println("[OK] フル機能モード設定完了");
  }

  // SIMカード確認
  Serial.println("--- [3/11] SIMカード確認 ---");
  if (!sendATCommand("AT+CPIN?\r\n")) {
    Serial.println("[ERROR] SIMカードが検出されません");
  } else {
    Serial.println("[OK] SIMカード確認完了");
  }

  // 信号品質確認
  Serial.println("--- [4/11] 信号品質・ネットワーク確認 ---");
  sendATCommand("AT+CSQ\r\n");    // 信号品質 (0-31, 99=不明)
  sendATCommand("AT+COPS?\r\n");  // 接続オペレータ

  // PDPコンテキスト設定 (SORACOM)
  Serial.println("--- [5/11] PDPコンテキスト設定 ---");
  if (!sendATCommand("AT+CGDCONT=1,\"IP\",\"soracom.io\"\r\n")) {
    Serial.println("[ERROR] PDPコンテキスト設定失敗");
  } else {
    Serial.println("[OK] PDPコンテキスト設定完了");
  }
  sendATCommand("AT+CGDCONT?\r\n");  // 設定確認

  // ネットワーク接続 (GPRS Attach)
  Serial.println("--- [6/11] ネットワーク接続 ---");
  if (!sendATCommand("AT+CGATT=1\r\n", 10000)) {
    Serial.println("[ERROR] ネットワーク接続失敗");
  } else {
    Serial.println("[OK] ネットワーク接続完了");
  }
  sendATCommand("AT+CGATT?\r\n");  // 接続状態確認

  // アプリネットワーク設定
  Serial.println("--- [7/11] アプリネットワーク設定 ---");
  if (!sendATCommand("AT+CNCFG=0,1,\"soracom.io\"\r\n")) {
    Serial.println("[ERROR] アプリネットワーク設定失敗");
  } else {
    Serial.println("[OK] アプリネットワーク設定完了");
  }
  sendATCommand("AT+CNCFG?\r\n");  // 設定確認

  // アプリネットワーク有効化
  // AT+CNACT=0,1 は OK の後に URC "+APP PDP: 0,ACTIVE" が来る
  Serial.println("--- [8/11] ネットワーク有効化 (最大30秒) ---");
  sendATCommand("AT+CNACT=0,1\r\n", 5000);  // OKを受け取る
  Serial.println("URC '+APP PDP: 0,ACTIVE' 待ち...");
  if (!waitForResponse("+APP PDP: 0,ACTIVE", 30000)) {
    Serial.println("[ERROR] ネットワーク有効化失敗");
  } else {
    Serial.println("[OK] ネットワーク有効化完了");
  }
  sendATCommand("AT+CNACT?\r\n");  // 割り当てIPアドレス確認

  // ソケットID設定
  Serial.println("--- [9/11] ソケット設定 ---");
  if (!sendATCommand("AT+CACID=0\r\n")) {
    Serial.println("[ERROR] ソケットID設定失敗");
  } else {
    Serial.println("[OK] ソケットID設定完了");
  }

  // SSL無効 (UDPのため不要)
  sendATCommand("AT+CASSLCFG=0,\"SSL\",0\r\n");

  // 接続状態確認
  Serial.println("--- [10/11] 接続状態確認 ---");
  sendATCommand("AT+CASTATE?\r\n");

  // UDPソケットオープン (Soracom Harvest Data)
  Serial.println("--- [11/11] UDPソケット接続 (最大10秒) ---");
  if (!sendATCommand("AT+CAOPEN=0,0,\"UDP\",\"harvest.soracom.io\",8514\r\n", 10000)) {
    Serial.println("[ERROR] UDPソケット接続失敗");
  } else {
    Serial.println("[OK] UDPソケット接続完了");
  }

  Serial.println("=== 初期化完了 ===");
}

void loop() {
  String data_sample = "{ \"a\" : \"1\" }";
  serial_send(data_sample);
  delay(5000);  // 5秒ごとに送信
}
