# 【作業指示書】XIAO ESP32-C3 ＋ ICM-20948 磁気ノイズ絶縁テスト環境構築

本手順書は、VSCode + PlatformIO を用いて XIAO ESP32-C3 にファームウェアを書き込み、
Wi-Fi 経由で地磁気センサー（ICM-20948 内蔵 AK09916）の生データおよび傾斜補正方位を
リアルタイム検証する手順を定めたものです。

> 本版はライブラリ `wollewald/ICM20948_WE`（v1.2.x）の実APIに合わせてソースコードを修正済みです。
> 修正点: `setAccRange` メソッド名 / `ICM20948_GYRO_RANGE_250` 列挙名 / `readSensor()` の追加

---

## ハードウェア結線仕様

| ICM-20948 側ピン | XIAO ESP32-C3 側ピン | 役割 |
| --- | --- | --- |
| VIN（または 3Vo） | 3V3 | 電源供給（3.3V） |
| GND | GND | グランド |
| SDA | D4（GPIO 6） | I2C データ線 |
| SCL | D5（GPIO 7） | I2C クロック線 |

> **注意事項**
> 1. 配線長は 3cm〜5cm 以内に収める。
> 2. XIAO ESP32-C3 の IPEX コネクタへ付属ロッドアンテナ（Wi-Fiアンテナ）を必ず装着する。

---

## Step 1: 開発環境の準備（Windows）

1. Visual Studio Code をインストール。
2. VSCode「拡張機能（Ctrl+Shift+X）」で「PlatformIO IDE」を検索・インストール（完了後 VSCode を再起動）。

## Step 2: プロジェクトの作成と設定

1. VSCode 左側の PlatformIO アイコン →「PIO Home」→「Open」。
2. 「+ New Project」で以下を設定して「Finish」:
   - Name: `ICM20948_Noise_Test`
   - Board: `Seeed Studio XIAO ESP32C3`
   - Framework: `Arduino`
3. プロジェクトルートの `platformio.ini` を以下に丸ごと書き換えて保存。

```ini
[env:seeed_xiao_esp32c3]
platform = espressif32
board = seeed_xiao_esp32c3
framework = arduino
monitor_speed = 115200
lib_deps =
    wollewald/ICM20948_WE @ ^1.2.9
```

> ※ USB-CDC 用の build_flags は不要（ESP32-C3 は標準で USB-Serial-JTAG が有効）。

## Step 3: ファームウェアコードの実装

`src/main.cpp` を開き、以下のコードを貼り付けて保存。

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ICM20948_WE.h>

#define ICM20948_ADDR 0x69 // Adafruit基板のデフォルトI2Cアドレス（AD0=High→0x69 / Low→0x68）

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
WebServer server(80);

// 測定値保持用
float magX = 0, magY = 0, magZ = 0, magTotal = 0;
float roll = 0, pitch = 0, heading = 0;

// WebダッシュボードHTML
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ICM-20948 磁気・姿勢モニター</title>
  <style>
    body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: #fff; padding: 20px; }
    .card { background: #2a2a2a; border-radius: 8px; padding: 15px; margin: 10px auto; max-width: 450px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    .val { font-size: 24px; font-weight: bold; color: #4CAF50; }
    .unit { font-size: 14px; color: #aaa; }
    .warn { color: #ff5252; }
    table { width: 100%; border-collapse: collapse; margin-top: 10px; }
    td { padding: 6px; border-bottom: 1px solid #444; }
  </style>
</head>
<body>
  <h2>ICM-20948 磁気ノイズ絶縁テスト</h2>

  <div class="card">
    <div>合成磁界強度 (目標: 30〜60 uT)</div>
    <div class="val" id="b_total">--</div>
    <div class="unit">マイクロテスラ (uT)</div>
  </div>

  <div class="card">
    <div>傾斜補正ヘディング角</div>
    <div class="val" id="heading">--</div>
    <div class="unit">度 (0〜360 deg)</div>
  </div>

  <div class="card">
    <table>
      <tr><td>Roll</td><td id="roll">--</td><td>deg</td></tr>
      <tr><td>Pitch</td><td id="pitch">--</td><td>deg</td></tr>
      <tr><td>Mag X</td><td id="mag_x">--</td><td>uT</td></tr>
      <tr><td>Mag Y</td><td id="mag_y">--</td><td>uT</td></tr>
      <tr><td>Mag Z</td><td id="mag_z">--</td><td>uT</td></tr>
    </table>
  </div>

  <script>
    setInterval(() => {
      fetch('/data')
        .then(res => res.json())
        .then(data => {
          document.getElementById('mag_x').innerText = data.mx.toFixed(2);
          document.getElementById('mag_y').innerText = data.my.toFixed(2);
          document.getElementById('mag_z').innerText = data.mz.toFixed(2);

          let bt = document.getElementById('b_total');
          bt.innerText = data.total.toFixed(2);
          if (data.total < 25 || data.total > 70) {
            bt.className = 'val warn';
          } else {
            bt.className = 'val';
          }

          document.getElementById('roll').innerText = data.roll.toFixed(1);
          document.getElementById('pitch').innerText = data.pitch.toFixed(1);
          document.getElementById('heading').innerText = data.heading.toFixed(1);
        });
    }, 100);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json = "{";
  json += "\"mx\":" + String(magX) + ",";
  json += "\"my\":" + String(magY) + ",";
  json += "\"mz\":" + String(magZ) + ",";
  json += "\"total\":" + String(magTotal) + ",";
  json += "\"roll\":" + String(roll) + ",";
  json += "\"pitch\":" + String(pitch) + ",";
  json += "\"heading\":" + String(heading);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7); // SDA=GPIO6(D4), SCL=GPIO7(D5)
  Wire.setClock(400000);

  delay(1000);

  if (!myIMU.init()) {
    Serial.println("ICM-20948 初期化失敗。I2C結線またはアドレス(0x68/0x69)を確認してください。");
  } else {
    Serial.println("ICM-20948 初期化成功");
  }

  // 磁気センサー初期化 (AK09916)
  if (!myIMU.initMagnetometer()) {
    Serial.println("磁気センサー(AK09916) 初期化失敗。");
  } else {
    Serial.println("磁気センサー(AK09916) 初期化成功");
  }
  myIMU.setMagOpMode(AK09916_CONT_MODE_100HZ);
  delay(20); // 初回の磁気値がゼロになるのを回避

  // 加速度・ジャイロレンジ設定
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);

  // Wi-Fi Access Point 起動
  WiFi.softAP("XIAO_IMU_TEST", "12345678");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();

  xyzFloat gVal, magVal;
  myIMU.readSensor();      // ← 必須：内部バッファを更新
  myIMU.getGValues(&gVal);
  myIMU.getMagValues(&magVal);

  magX = magVal.x;
  magY = magVal.y;
  magZ = magVal.z;
  magTotal = sqrt(magX * magX + magY * magY + magZ * magZ);

  // 加速度からロール・ピッチ算出 (rad)
  float r_rad = atan2(gVal.y, gVal.z);
  float p_rad = atan2(-gVal.x, sqrt(gVal.y * gVal.y + gVal.z * gVal.z));

  roll = r_rad * 180.0 / PI;
  pitch = p_rad * 180.0 / PI;

  // 傾斜補正ヘディング計算
  float bx = magX;
  float by = magY;
  float bz = magZ;

  float bx_comp = bx * cos(p_rad) + bz * sin(p_rad);
  float by_comp = bx * sin(r_rad) * sin(p_rad) + by * cos(r_rad) - bz * sin(r_rad) * cos(p_rad);

  float h_rad = atan2(-by_comp, bx_comp);
  heading = h_rad * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  delay(20); // 約50Hz更新
}
```
## Step 4: ビルドと書き込み

1. XIAO ESP32-C3 を PC に USB 接続。
2. VSCode 下部ステータスバーの「チェックマーク（Build）」を押し、エラーが出ないことを確認。
3. 「右矢印（Upload）」を押して書き込み。

## Step 5: テスト・検証実施手順

1. **完全バッテリー駆動への移行**:
   - XIAO ESP32-C3 の USB ケーブルを PC から抜き、モバイルバッテリー／乾電池電源に接続。
   - 金属製の机や PC 本体から最低 50cm 以上離れた木製机等で実施。
2. **Wi-Fi 接続とダッシュボード表示**:
   - PC／スマホの Wi-Fi 設定で `XIAO_IMU_TEST`（パスワード: `12345678`）に接続。
   - ブラウザで `http://192.168.4.1` を開きダッシュボードを表示。

## Step 6: 合否判定基準

| テスト項目 | 操作手順 | 合格基準 | 異常（不合格・要対策） |
| --- | --- | --- | --- |
| ① 静止時ノイズ検証 | 基板を完全静止 | 合成磁界が 30〜60 µT 内で安定し、ブレが ±0.5 µT 以内 | 100 µT 以上または激しく上下 → 近接磁石・電源リップルの混入 |
| ② 3D 回転耐性検証 | 基板を全方位（8の字）にゆっくり回転 | 合成磁界が常に 30〜60 µT を維持 | 特定の向きで 10 µT や 100 µT に跳ねる → 強いハードアイアン歪み |
| ③ 傾斜補正検証 | ヨー角固定のままロール／ピッチを ±30° 傾ける | ヘディング角の変動が ±5° 以内 | ロールだけで 30°〜90° 以上回転 → 軸の向き定義不一致 |

---

## 補足（実施時のメモ）

- 起動直後、シリアルモニタ（115200）で `ICM-20948 初期化成功` と `AP IP Address: 192.168.4.1` が表示されることを確認。
- ヘディング角はセンサー軸の向き定義と磁気キャリブレーションに依存するため、③の判定は軸マッピング確認後に実施する。