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