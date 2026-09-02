#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>

#define ICM20948_ADDR 0x69 // Adafruit基板のデフォルトI2Cアドレス（AD0=High→0x69 / Low→0x68）

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// シリアル出力間隔
#define SERIAL_INTERVAL_MS 50   // 50ms = 20Hz で出力

unsigned long lastSend = 0;

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

  // CSVヘッダ
  Serial.println("ms,mx_uT,my_uT,mz_uT,total_uT,roll_deg,pitch_deg,heading_deg");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < SERIAL_INTERVAL_MS) return;
  lastSend = now;

  xyzFloat gVal, magVal;
  myIMU.readSensor();      // 必須：内部バッファを更新
  myIMU.getGValues(&gVal);
  myIMU.getMagValues(&magVal);

  float mx = magVal.x;
  float my = magVal.y;
  float mz = magVal.z;
  float total = sqrt(mx * mx + my * my + mz * mz);

  // 加速度からロール・ピッチ算出 (rad)
  float r_rad = atan2(gVal.y, gVal.z);
  float p_rad = atan2(-gVal.x, sqrt(gVal.y * gVal.y + gVal.z * gVal.z));
  float rollDeg = r_rad * 180.0 / PI;
  float pitchDeg = p_rad * 180.0 / PI;

  // 傾斜補正ヘディング計算
  float bx_comp = mx * cos(p_rad) + mz * sin(p_rad);
  float by_comp = mx * sin(r_rad) * sin(p_rad) + my * cos(r_rad) - mz * sin(r_rad) * cos(p_rad);
  float heading = atan2(-by_comp, bx_comp) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  Serial.print(now);
  Serial.print(',');
  Serial.print(mx, 2);
  Serial.print(',');
  Serial.print(my, 2);
  Serial.print(',');
  Serial.print(mz, 2);
  Serial.print(',');
  Serial.print(total, 2);
  Serial.print(',');
  Serial.print(rollDeg, 1);
  Serial.print(',');
  Serial.print(pitchDeg, 1);
  Serial.print(',');
  Serial.println(heading, 1);
}