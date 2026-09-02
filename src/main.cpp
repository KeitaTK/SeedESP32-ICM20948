/*
 * XIAO ESP32-C3 + ICM-20948 (AK09916) 9軸生データストリーマー
 *
 * 【目的】Raspberry Pi (pypilot 中継スクリプト) へ純粋な生データを安定供給する。
 *   ・I2Cタイミングバグ / フリーズに対してフェイルセーフ(自己修復)を内蔵
 *   ・単位変換済みCSVを100Hz / USB CDC経由で連続送信
 *
 * 【出力フォーマット】(単位込み・タイムスタンプなし)
 *   seq,ax,ay,az,gx,gy,gz,mx,my,mz\n
 *   seq    : 0..65535 循環の連番(欠落検知用)
 *   ax..az : 加速度 [g]      (wollewaldライブラリは元々[g]を返す)
 *   gx..gz : ジャイロ [rad/s](ライブラリは[dps]のため π/180 で変換)
 *   mx..mz : 地磁気 [µT]     (そのまま)
 *
 * 【フェイルセーフ(要件書 §5)】
 *   §5-1 初期化失敗  -> 3回リトライ後に ESP.restart()
 *   §5-2 読み取り失敗 -> フリーズ初回のみ WHO_AM_I でバス死活確認(NACK検知)。
 *        NACK連続10回(≒100ms)、または読出し8ms超のハング連続10回で ESP.restart()
 *   §5-3 完全一致フリーズ -> 9軸全軸が前回値と完全一致を50回(≒500ms)連続で ESP.restart()
 *   §5-4 異常値持続 -> 加速度合成(0.5〜2.5g)・ジャイロ各軸(±30rad/s)・
 *        磁気合成(10〜100µT) を全て外れる状態が100回(≒1s)連続で ESP.restart()
 *
 * 【注記】wollewald/ICM20948_WE v1.2.9 の readSensor() は void でありI2Cエラー
 *   (NACK)を返さない。NACK時は内部バッファが更新されず「全軸で前回値と完全一致」
 *   という形でしか観測できないため、上記の条件付きプローブでNACKを検出する。
 */

#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <string.h>

/* ================= ハードウェア / I2Cバス ================= */
static const uint8_t  PIN_SDA        = 6;    // D4 = GPIO6
static const uint8_t  PIN_SCL        = 7;    // D5 = GPIO7
static const uint8_t  IMU_I2C_ADDR   = 0x69; // Adafruit ICM-20948 Breakout (AD0=High)
static const uint32_t I2C_CLOCK_HZ   = 400000UL;
// I2Cトランザクション1回あたりの最大ブロック時間。
// バスが物理的にハングしてもWireが無限に待たないようにする(デフォルト50ms -> 5ms)。
static const uint32_t I2C_TIMEOUT_MS = 5UL;

/* ================= サンプリング / シリアル ================= */
static const uint32_t SAMPLE_PERIOD_US = 10000UL;  // 10ms = 100Hz (非ブロッキング)
static const uint32_t SERIAL_BAUD      = 115200UL;
static const uint16_t WARMUP_SAMPLES   = 30;       // 起動直後のAK09916安定待ち(出力抑制)
static const int      SEND_GUARD_BYTES = 96;       // CDC TXリングに1行分の余裕がある時のみ送信

/* ================= フェイルセーフ閾値(要件書 §5) ================= */
static const uint8_t  INIT_RETRIES          = 3;  // §5-1
static const uint16_t NACK_RESTART_FRAMES   = 10; // §5-2 NACK連続(≒100ms)
static const uint32_t SLOW_READ_MAX_US      = 8000UL; // 正常読出しは≒1ms未満。超過=ハング疑い
static const uint16_t SLOW_READ_RESTART     = 10; // §5-2 ハング連続
static const uint16_t FROZEN_RESTART_FRAMES = 50; // §5-3 9軸完全一致(≒500ms)
static const uint16_t RANGE_RESTART_FRAMES  = 100; // §5-4 全レンジ外(≒1s)

/* §5-4 物理レンジ */
static const float ACC_MAG_MIN_G = 0.5f;
static const float ACC_MAG_MAX_G = 2.5f;
static const float GYR_MAX_RPS   = 30.0f;   // rad/s
static const float MAG_MIN_UT    = 10.0f;
static const float MAG_MAX_UT    = 100.0f;

static const float D2R = 0.017453292519943295f;  // deg -> rad

ICM20948_WE imu(IMU_I2C_ADDR);

/* 最新9軸(物理単位) [0..2]=accel[g], [3..5]=gyro[rad/s], [6..8]=mag[uT] */
static float cur[9];
static float prev[9];
static bool  havePrev = false;

static uint16_t seq = 0;          // 0..65535 循環
static uint32_t lastGoodUs = 0;   // 最後に「正常なトランザクション」が完了したmicros()

/* フェイルセーフ用カウンタ */
static uint32_t nackFrames   = 0;  // NACK確認済みフレーム連続数
static uint32_t frozenFrames = 0;  // 9軸完全一致フレーム連続数(バス生存時)
static uint32_t slowReads    = 0;  // 読出し所要時間超過の連続数
static uint32_t rangeFrames  = 0;  // 全レンジ外フレーム連続数
static bool     nackProbed   = false; // 完全一致の初回にだけWHO_AM_Iを実行
static bool     busAlive     = true;
static uint16_t warmupLeft   = WARMUP_SAMPLES;

/* ---------------------------------------------------------------
 * 診断メッセージ。'#' 始まりの行はCSVストリームを汚さない(コメント行)。
 * ------------------------------------------------------------- */
static void logMsg(const char *msg) {
    Serial.print('#');
    Serial.println(msg);
}

/* ---------------------------------------------------------------
 * 致命的エラー -> 再起動。行単位で理由を出力してから ESP.restart()。
 * ------------------------------------------------------------- */
static void fatalRestart(const char *reason) {
    Serial.flush();
    if (Serial) {
        Serial.print("#RESTART ");
        Serial.print(reason);
        Serial.print(" seq=");
        Serial.print(seq);
        Serial.println();
    }
    Serial.flush();
    delay(100);
    ESP.restart();
    while (1) { delay(10); }  // 保険
}

/* ---------------------------------------------------------------
 * 初期化(セルフテスト込み)。成功時 true。IMUとAK09916を両方検証する。
 * ------------------------------------------------------------- */
static bool initSensor() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    for (int attempt = 1; attempt <= INIT_RETRIES; attempt++) {
        logMsg("#init attempt");
        delay(100);  // 電源投入直後の立ち上がり待ち

        if (!imu.init()) {
            logMsg("#init: ICM-20948 init NG");
            continue;
        }
        if (!imu.initMagnetometer()) {  // AK09916 WHO_AM_I確認 + 100Hz連続モード設定込み
            logMsg("#init: AK09916 init NG");
            continue;
        }
        imu.setMagOpMode(AK09916_CONT_MODE_100HZ);
        imu.setAccRange(ICM20948_ACC_RANGE_2G);
        imu.setGyrRange(ICM20948_GYRO_RANGE_250);
        delay(50);

        /* 最終セルフテスト: 両センサのWHO_AM_Iが期待値か */
        if (imu.whoAmI() != 0xEA) {
            logMsg("#init: WHO_AM_I NG");
            continue;
        }
        uint16_t magId = imu.whoAmIMag();
        if (magId != ICM20948_WE::AK09916_WHO_AM_I_1 &&
            magId != ICM20948_WE::AK09916_WHO_AM_I_2) {
            logMsg("#init: MAG WHO_AM_I NG");
            continue;
        }
        logMsg("#init: OK");
        return true;
    }
    return false;
}

/* ---------------------------------------------------------------
 * 1行のCSV送信: seq,ax,ay,az,gx,gy,gz,mx,my,mz\n
 * ------------------------------------------------------------- */
static void sendCsvLine(uint16_t s, const float v[9]) {
    Serial.print(s);
    Serial.print(',');
    Serial.print(v[0], 5);  // accel [g]
    Serial.print(',');
    Serial.print(v[1], 5);
    Serial.print(',');
    Serial.print(v[2], 5);
    Serial.print(',');
    Serial.print(v[3], 5);  // gyro [rad/s]
    Serial.print(',');
    Serial.print(v[4], 5);
    Serial.print(',');
    Serial.print(v[5], 5);
    Serial.print(',');
    Serial.print(v[6], 2);  // mag [uT]
    Serial.print(',');
    Serial.print(v[7], 2);
    Serial.print(',');
    Serial.println(v[8], 2);
}


/* ---------------------------------------------------------------
 * 1サンプル周期(10ms)ごとの処理: 読み出し -> フェイルセーフ -> 送信
 * ------------------------------------------------------------- */
static void sampleAndSend(uint32_t nowUs) {
    /* ---- センサ読み出し(所要時間も計測: ハング検知用) ---- */
    uint32_t t0 = micros();
    imu.readSensor();
    uint32_t readUs = micros() - t0;

    xyzFloat gV, gyrV, magV;
    imu.getGValues(&gV);       // [g]
    imu.getGyrValues(&gyrV);   // [dps] -> rad/s へ変換
    imu.getMagValues(&magV);   // [uT]

    cur[0] = gV.x;   cur[1] = gV.y;   cur[2] = gV.z;
    cur[3] = gyrV.x * D2R; cur[4] = gyrV.y * D2R; cur[5] = gyrV.z * D2R;
    cur[6] = magV.x; cur[7] = magV.y; cur[8] = magV.z;

    /* ---- §5-2 / §5-3: 完全一致(データ更新なし)の検出 ----
     * NACK発生時、ライブラリの内部バッファは更新されないため、
     * 「9軸全軸が前回値と完全一致」という形でしか検知できない。
     * 初回フリーズ時のみ WHO_AM_I プローブで、NACK(バス死亡)と
     * ゾンビ(バス生存・センサ内部停止)を切り分ける。 */
    bool identical = havePrev;
    if (identical) {
        for (int i = 0; i < 9; i++) {
            if (cur[i] != prev[i]) { identical = false; break; }
        }
    }

    if (identical && !nackProbed) {
        nackProbed = true;
        busAlive = (imu.whoAmI() == 0xEA);
    }
    if (!identical) {
        nackProbed = false;
        busAlive = true;
    }

    if (identical) {
        if (nackProbed && !busAlive) { nackFrames++;   frozenFrames = 0; }
        else                         { frozenFrames++; nackFrames = 0; }
    } else {
        nackFrames   = 0;
        frozenFrames = 0;
    }

    /* §5-2: NACK連続10回(≒100ms)で再起動 */
    if (nackFrames >= NACK_RESTART_FRAMES) {
        fatalRestart("i2c-nack");
    }
    /* §5-3: 9軸完全一致50回(≒500ms)で再起動 */
    if (frozenFrames >= FROZEN_RESTART_FRAMES) {
        fatalRestart("9axis-frozen");
    }

    /* ---- §5-2: I2Cハング検知(読出し所要時間) ---- */
    if (readUs > SLOW_READ_MAX_US) {
        slowReads++;
        if (slowReads >= SLOW_READ_RESTART) {
            fatalRestart("i2c-hang");
        }
    } else {
        slowReads = 0;
    }

    /* ---- §5-4: 異常値の持続検知(全レンジ外の状態が1s継続) ---- */
    float accMag = sqrtf(cur[0] * cur[0] + cur[1] * cur[1] + cur[2] * cur[2]);
    bool accOut  = (accMag < ACC_MAG_MIN_G) || (accMag > ACC_MAG_MAX_G);
    bool gyrOut  = (fabsf(cur[3]) > GYR_MAX_RPS) ||
                   (fabsf(cur[4]) > GYR_MAX_RPS) ||
                   (fabsf(cur[5]) > GYR_MAX_RPS);
    float magMag = sqrtf(cur[6] * cur[6] + cur[7] * cur[7] + cur[8] * cur[8]);
    bool magOut  = (magMag < MAG_MIN_UT) || (magMag > MAG_MAX_UT);

    if (accOut && gyrOut && magOut) {
        rangeFrames++;
        if (rangeFrames >= RANGE_RESTART_FRAMES) {
            fatalRestart("range-out");
        }
    } else {
        rangeFrames = 0;
    }

    /* 「正常な読み出し」判定(ストール・NACKを除く) */
    bool readOk = (readUs <= SLOW_READ_MAX_US) && !(identical && nackProbed && !busAlive);
    if (readOk) lastGoodUs = nowUs;

    /* 総合ストールガード: 正常読み出しから100ms以上経過したら再起動(§5-2後段) */
    if (warmupLeft == 0 && (nowUs - lastGoodUs) > 100000UL) {
        fatalRestart("no-good-read");
    }

    /* ウォームアップ: 起動直後のAK09916初回安定分は送信しない(判定は毎回実施) */
    if (warmupLeft > 0) {
        warmupLeft--;
    }

    /* ---- 送信(CDC未接続 or TXリング不足時は送らず次の周期へ) ---- */
    if (warmupLeft == 0 && Serial && Serial.availableForWrite() >= SEND_GUARD_BYTES) {
        sendCsvLine(seq, cur);
        seq++;  // uint16なので 65535 の次は 0 へ戻る
    }

    /* 前回値の更新(送信有無に関わらず行い、フリーズ判定を継続) */
    memcpy(prev, cur, sizeof(prev));
    havePrev = true;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    // USB CDC は未接続でも動作させるため、接続待ちはしない。

    /* §5-1: 初期化(セルフテスト)失敗時はリトライ後に再起動 */
    if (!initSensor()) {
        logMsg("#init FAILED (3 attempts)");
        fatalRestart("init-failed");
    }

    lastGoodUs = micros();
    warmupLeft = WARMUP_SAMPLES;
    logMsg("#stream start 100Hz");
}

void loop() {
    /* 非ブロッキングな100Hz(10ms)タイマ制御 */
    static uint32_t nextUs = 0;
    uint32_t nowUs = micros();

    if (nextUs == 0) {
        nextUs = nowUs;  // 初回起動
    }
    if ((nowUs - nextUs) < SAMPLE_PERIOD_US) {
        return;
    }
    /* 大きく取りこぼした場合はスケジュールを再同期する(追い越し実行の防止) */
    nextUs += SAMPLE_PERIOD_US;
    if ((nowUs - nextUs) >= SAMPLE_PERIOD_US) {
        nextUs = nowUs;
    }

    sampleAndSend(nowUs);
}

