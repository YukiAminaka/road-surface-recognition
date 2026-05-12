## 自転車への搭載を想定した路面状態および走行状態を推定する TinyML デバイス

### デバイス構成

![Image](https://github.com/user-attachments/assets/f1c53281-b94c-4690-8d0e-3cb1d4d94241)

### デバイス外観

<div style="display:flex">
  <img width="250px" src="https://github.com/user-attachments/assets/258a4af0-7bbf-4b68-979d-dcb5527a1d6b">
  <img width="250px" src="https://github.com/user-attachments/assets/07469f2d-4242-47fc-857a-863efdbeceb1">
</div>  
<img width="500px" src="https://github.com/user-attachments/assets/af5ad601-04bf-4175-9508-6003ebe9f07a">

## 各処理の周期

各処理の周期

1. BLE データの取得

   実装周期: 1000ms (1 秒)
   getRoadSurfaceCharacteristicsTask タスクで実装
   BLE 通信で走行状態および路面特性を取得
   実際のデータ取得は notifyCallback によるイベント駆動型

2. 位置情報(GNSS)の取得

   実装周期: 500ms (0.5 秒)
   getGnssTask タスクで実装
   緯度、経度、高度を取得
   RTC から時刻情報も取得してディスプレイに表示

3. データ送信(CAT-M1)

   実装周期: 700ms
   sendDataTask タスクで実装
   queue1 からデータを取り出して CAT-M1 モジュールで送信
   実際には約 2 秒に 1 回のペースでデータが送信される見込み

4. SD カードへの書き込み

   実装周期: queue2 に SAMPLE_RATE (10 個) のデータが溜まったとき
   BLE データが届くたびに queue2 に追加され、10 個溜まると logged フラグが立つ
   writeSDTask タスクが 1ms ごとにフラグをチェックして書き込み実行

   データフロー

   BLE サーバーから通知が届く (イベント駆動)
   notifyCallback が呼ばれ、現在の位置情報とステータスを JSON 化
   データを queue1 (送信用) と queue2 (SD 書き込み用) に追加
   queue2 が 10 個に達したら SD カードに一括書き込み
   700ms ごとに queue1 からデータを取り出して CAT-M1 で送信
