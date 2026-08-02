// Quark rocket -- simpler avionics, no GPS. Shares the PulseR ground station.
#include <Wire.h>
#include <SPI.h>
#include <SD_fix.h>
#include <LoRa.h>
#include <GyverBME280.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// === PINS ===
#define LED_R A1
#define LED_G A0
#define LED_B A2
#define PYRO_PIN 3
#define SD_CS 4
#define LORA_DIO0 2
#define LORA_RST 9
#define LORA_NSS 10
#define S_RAW A3

#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define BME280_ADDR 0x76

GyverBME280 bme;
MPU6050 mpu;
File logFile;

const char TEAM_ID[] = "CB34";

// === Accel calibration (run mpu6050_accel_calibrate on THIS rocket's MPU) ===
static const float ACC_BIAS[3]  = {0.0f, 0.0f, 0.0f};   // g
static const float ACC_SCALE[3] = {1.0f, 1.0f, 1.0f};   // g/g

// === Magnetometer hard-iron offsets (figure-eight calibration) ===
const int MAG_OFF[3] = {0, 0, 0};

// launch/burnout thresholds (raw LSB, ay is the thrust axis)
#define BOOST_THRESHOLD 4096    // 2.00 g
#define BURNOUT_THRESHOLD 2150  // 1.05 g
#define BOOST_CONFIRM_CNT 2
#define BURNOUT_CONFIRM_CNT 2

uint8_t init_code = 0;
unsigned long pktid = 0;

bool bmeOK, mpuOK, qmcOK, sdOK, loraOK;
bool ready = false, launched = false, hit_apogee = false, landed = false;
bool alt_threshold = false, in_boost = false, burnout_detected = false;
bool pyro1_armed = false, pyro1_state = false, pyro1_fired = false;

uint16_t pyroFireTime = 750;
float max_vel = 0, start_alt = 0, apogee = 0, voltage = 0, water_alt = 0;
float baro_alt = 0, vel = 0;
float r1 = 29800, r2 = 7490, min_fire_alt = 20.0f;
float avx = 0, avy = 0, avz = 0;                 // apparent velocity, body axes (m/s)
int   heading = 0;
int16_t magX = 0, magY = 0, magZ = 0;
int   lastAxmg = 0, lastAymg = 0, lastAzmg = 0;
int   lastTemp = 0, lastHmdt = 0;
float lastPrs = 0;
constexpr float G0 = 9.815f;

unsigned long launchTime = 0, lastLog = 0;
const unsigned long LOG_INT = 50;        // SD flush 20 Hz
const unsigned long SAMPLE_INT = 10;     // IMU 100 Hz
const unsigned long BME_INT = 50;        // baro/flight 20 Hz
const unsigned long LORA_INT = 200;      // in-flight 5 Hz

// ── LED ──
void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, !r); digitalWrite(LED_G, !g); digitalWrite(LED_B, !b);
}
static uint8_t appendL(char *b, long v) { ltoa(v, b, 10); return strlen(b); }

// ── QMC5883L (0x0D) ──
bool qmcInit() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission()) return false;
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();  // set/reset period
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();  // cont,200Hz,8G,OSR512
  return true;
}
bool qmcRead() {
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6);
  if (Wire.available() < 6) return false;
  magX = Wire.read() | (Wire.read() << 8);
  magY = Wire.read() | (Wire.read() << 8);
  magZ = Wire.read() | (Wire.read() << 8);
  return true;
}
// cheap atan2 (radians) -- avoids linking atan2f
float fastAtan2(float y, float x) {
  float ax = fabsf(x), ay = fabsf(y);
  float d = (ax > ay ? ax : ay) + 1e-10f;
  float a = (ax < ay ? ax : ay) / d, s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (ay > ax) r = 1.57079637f - r;
  if (x < 0)   r = 3.14159274f - r;
  if (y < 0)   r = -r;
  return r;
}
void updateHeading() {
  if (!qmcOK || !qmcRead()) return;
  float mx = magX - MAG_OFF[0], my = magY - MAG_OFF[1];
  float h = fastAtan2(my, mx) * 57.29578f;
  if (h < 0) h += 360.0f;
  heading = (int)h;
}

// ── LoRa telemetry (TZ 3.4.5 format, matches the PulseR ground station) ──
// TID;Time;Alt;Ax;Ay;Az; Start;Ready;Apogee;Sep;PyState;PyFired; Landing;
//   Heading;Temp;Prs;Hmdt;Volt;Init;Vel;Apg;PID; Lat;Lon;Sats;Gdist(all 0, no GPS);CS
void sendPacket(bool async) {
  char pkt[160]; uint8_t n = 0;
  { uint8_t tl = strlen(TEAM_ID); memcpy(pkt + n, TEAM_ID, tl); n += tl; } pkt[n++] = ';';
  n += appendL(pkt+n,(long)millis());          pkt[n++]=';';
  n += appendL(pkt+n,(long)(baro_alt*100.0f)); pkt[n++]=';';   // cm
  n += appendL(pkt+n,(long)lastAxmg);          pkt[n++]=';';   // mg
  n += appendL(pkt+n,(long)lastAymg);          pkt[n++]=';';
  n += appendL(pkt+n,(long)lastAzmg);          pkt[n++]=';';
  n += appendL(pkt+n,launched          ?1L:0L);pkt[n++]=';';   // Start
  n += appendL(pkt+n,ready             ?1L:0L);pkt[n++]=';';   // ready
  n += appendL(pkt+n,hit_apogee        ?1L:0L);pkt[n++]=';';   // apogee
  n += appendL(pkt+n,0L);                      pkt[n++]=';';   // sep (n/a)
  n += appendL(pkt+n,pyro1_state        ?1L:0L);pkt[n++]=';';  // pyro state
  n += appendL(pkt+n,pyro1_fired        ?1L:0L);pkt[n++]=';';  // pyro fired
  n += appendL(pkt+n,landed             ?1L:0L);pkt[n++]=';';  // Landing
  n += appendL(pkt+n,(long)heading);           pkt[n++]=';';
  n += appendL(pkt+n,(long)lastTemp);          pkt[n++]=';';
  n += appendL(pkt+n,(long)lastPrs);           pkt[n++]=';';
  n += appendL(pkt+n,(long)lastHmdt);          pkt[n++]=';';
  n += appendL(pkt+n,(long)(voltage*100.0f));  pkt[n++]=';';   // cV
  n += appendL(pkt+n,(long)init_code);         pkt[n++]=';';
  n += appendL(pkt+n,(long)(vel*100.0f));      pkt[n++]=';';   // cm/s
  n += appendL(pkt+n,(long)(apogee*100.0f));   pkt[n++]=';';   // cm
  n += appendL(pkt+n,(long)pktid);             pkt[n++]=';';
  n += appendL(pkt+n,0L);                      pkt[n++]=';';   // lat (no GPS)
  n += appendL(pkt+n,0L);                      pkt[n++]=';';   // lon
  n += appendL(pkt+n,0L);                      pkt[n++]=';';   // sats
  n += appendL(pkt+n,0L);                                      // gdist
  pkt[n]=0;
  uint8_t cs=0; for(uint8_t i=0;i<n;i++) cs+=pkt[i];
  LoRa.beginPacket(); LoRa.print(pkt); LoRa.print(';'); LoRa.print(cs);
  LoRa.endPacket(async);
  pktid++;
}

void sendAck(int cmd) {
  char ack[16]; uint8_t an = 4; memcpy(ack, "ACK;", 4);
  an += appendL(ack + an, (long)cmd); ack[an] = 0;
  uint8_t cs = 0; for (uint8_t i = 0; i < an; i++) cs += ack[i];
  LoRa.beginPacket(); LoRa.print(ack); LoRa.print(';'); LoRa.print(cs); LoRa.endPacket();
  LoRa.receive();
}

void applyCmd(int cmd, int val) {
  if (cmd == 2) {                          // pyro arm / fire
    if (val == 2 && pyro1_armed) pyro1_state = true;
    else pyro1_armed = (bool)val;
  } else if (cmd == 3) {                    // arm / idle
    ready = (bool)val; pyro1_armed = (bool)val;
    start_alt = water_alt; apogee = 0; max_vel = 0;
    avx = avy = avz = 0;
    setLED(0, 0, ready ? 1 : 0);
  } else if (cmd == 5) {                    // reset altitude reference
    start_alt = water_alt; apogee = 0; max_vel = 0;
  }
  sendAck(cmd);
}

void handleCmd(char *b) {
  char *ls = strrchr(b, ';'); if (!ls) return;
  uint8_t rx = (uint8_t)atoi(ls + 1); *ls = 0;
  uint8_t cs = 0; for (char *p = b; *p; p++) cs += *p;
  if (cs != rx) return;
  if (strncmp(b, "CMD;", 4) != 0) return;
  strtok(b, ";"); strtok(NULL, ";");        // "CMD", target
  char *sc = strtok(NULL, ";"), *sv = strtok(NULL, ";");
  if (!sc || !sv) return;
  applyCmd(atoi(sc), atoi(sv));
}

// ═══════════════ SETUP ═══════════════
void setup() {
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  pinMode(SD_CS, OUTPUT); pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);
  setLED(1, 1, 0);
  Wire.begin(); Wire.setClock(400000); SPI.begin();

  bmeOK = bme.begin(BME280_ADDR);
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setDLPFMode(MPU6050_DLPF_BW_188);
  mpuOK = mpu.testConnection();
  qmcOK = qmcInit();
  sdOK = SD.begin(SD_CS);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW); delay(10); digitalWrite(LORA_RST, HIGH); delay(50);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  loraOK = LoRa.begin(433E6);
  if (loraOK) LoRa.setTxPower(13);

  if (bmeOK && mpuOK && qmcOK && sdOK && loraOK) { init_code = 0; setLED(0, 1, 0); delay(400); }
  else {
    if (!bmeOK) init_code |= 1; if (!mpuOK) init_code |= 2; if (!qmcOK) init_code |= 4;
    if (!sdOK) init_code |= 8; if (!loraOK) init_code |= 16;
    setLED(1, 0, 0); delay(800);
  }

  if (sdOK) {
    char name[] = "F00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      name[1] = '0' + (i / 10); name[2] = '0' + (i % 10);
      if (!SD.exists(name)) { logFile = SD.open(name, FILE_WRITE); break; }
    }
    if (logFile) {
      logFile.println(F("Quark"));   // columns (integer units), parsed by index:
      // tid,time,alt_cm,ax_mg,ay_mg,az_mg,gx_cdps,gy_cdps,gz_cdps,vx_cms,vy_cms,vz_cms,
      // launched,ready,hitapg,landed,inboost,burnout,p_armed,p_state,p_fired,
      // temp_ddeg,prs_pa,hmdt,volt_cv,init,vel_cms,apg_cm,heading,logid
      logFile.flush();
    }
  }
  setLED(1, 0, 1); delay(300);

  start_alt = pressureToAltitude(bme.readPressure());
  ready = false;
  setLED(0, 0, 0);
  if (loraOK) LoRa.receive();
}

// ═══════════════ LOOP ═══════════════
void loop() {
  unsigned long now = millis();

  // ---- IMU sample + SD log @100 Hz ----
  static unsigned long tSample = 0, tPrev = 0;
  if (now - tSample >= SAMPLE_INT) {
    tSample = now;
    float sdt = (tPrev == 0) ? 0.0f : (now - tPrev) / 1000.0f;
    tPrev = now;

    int16_t alx, aly, alz, grx, gry, grz;
    mpu.getMotion6(&alx, &aly, &alz, &grx, &gry, &grz);

    float axmg = ((alx / 2048.0f - ACC_BIAS[0]) / ACC_SCALE[0]) * 1000.0f;
    float aymg = ((aly / 2048.0f - ACC_BIAS[1]) / ACC_SCALE[1]) * 1000.0f;
    float azmg = ((alz / 2048.0f - ACC_BIAS[2]) / ACC_SCALE[2]) * 1000.0f;
    float gxd = grx / 16.4f, gyd = gry / 16.4f, gzd = grz / 16.4f;
    lastAxmg = (int)axmg; lastAymg = (int)aymg; lastAzmg = (int)azmg;

    if (launched) {                       // apparent velocity (body axes), zeroed at launch
      avx += (axmg * 0.001f * G0) * sdt;
      avy += (aymg * 0.001f * G0) * sdt;
      avz += (azmg * 0.001f * G0) * sdt;
    }

    // launch detect: thrust axis (ay) over threshold, or baro climb
    if (ready && !launched && (aly > BOOST_THRESHOLD || baro_alt > 4.0f)) {
      launched = true; launchTime = now; avx = avy = avz = 0; setLED(1, 1, 0);
    }

    // burnout detection (drives pyro), on raw ay
    static uint8_t boost_cnt = 0, burnout_cnt = 0;
    if (launched && !burnout_detected) {
      if (!in_boost) {
        if (aly > BOOST_THRESHOLD) { if (++boost_cnt >= BOOST_CONFIRM_CNT) { in_boost = true; boost_cnt = burnout_cnt = 0; } }
        else boost_cnt = 0;
      } else {
        if (aly < BURNOUT_THRESHOLD) {
          if (++burnout_cnt >= BURNOUT_CONFIRM_CNT) {
            in_boost = false; burnout_detected = true; burnout_cnt = 0;
            if (pyro1_armed && alt_threshold && now - launchTime >= 750) pyro1_state = true;
            setLED(0, 1, 1);
          }
        } else burnout_cnt = 0;
      }
    }

    if (logFile) {
      static uint8_t logId = 0;
      logFile.print(TEAM_ID);                logFile.print(',');
      logFile.print(now);                    logFile.print(',');
      logFile.print((long)(baro_alt*100));   logFile.print(',');
      logFile.print(lastAxmg);               logFile.print(',');
      logFile.print(lastAymg);               logFile.print(',');
      logFile.print(lastAzmg);               logFile.print(',');
      logFile.print((long)(gxd*100));        logFile.print(',');
      logFile.print((long)(gyd*100));        logFile.print(',');
      logFile.print((long)(gzd*100));        logFile.print(',');
      logFile.print((long)(avx*100));        logFile.print(',');
      logFile.print((long)(avy*100));        logFile.print(',');
      logFile.print((long)(avz*100));        logFile.print(',');
      logFile.print(launched);               logFile.print(',');
      logFile.print(ready);                  logFile.print(',');
      logFile.print(hit_apogee);             logFile.print(',');
      logFile.print(landed);                 logFile.print(',');
      logFile.print(in_boost);               logFile.print(',');
      logFile.print(burnout_detected);       logFile.print(',');
      logFile.print(pyro1_armed);            logFile.print(',');
      logFile.print(pyro1_state);            logFile.print(',');
      logFile.print(pyro1_fired);            logFile.print(',');
      logFile.print(lastTemp);               logFile.print(',');
      logFile.print((long)lastPrs);          logFile.print(',');
      logFile.print(lastHmdt);               logFile.print(',');
      logFile.print((int)(voltage*100));     logFile.print(',');
      logFile.print((int)init_code);         logFile.print(',');
      logFile.print((long)(vel*100));        logFile.print(',');
      logFile.print((long)(apogee*100));     logFile.print(',');
      logFile.print(heading);                logFile.print(',');
      logFile.println(logId++);
      if (now - lastLog >= LOG_INT) { logFile.flush(); lastLog = now; }
    }
  }

  // ---- baro + vertical velocity + flight logic @20 Hz ----
  static unsigned long tBme = 0, landTime = 0;
  if (now - tBme >= BME_INT) {
    float dt = (now - tBme) / 1000.0f; tBme = now;
    updateHeading();
    lastPrs = bme.readPressure();
    lastTemp = (int)bme.readTemperature();
    lastHmdt = bme.readHumidity();
    water_alt = pressureToAltitude(lastPrs);
    baro_alt = water_alt - start_alt;

    static float altPrev = 0; static bool vfirst = true;
    if (vfirst) { altPrev = baro_alt; vfirst = false; }
    vel += 0.25f * ((baro_alt - altPrev) / dt - vel);
    altPrev = baro_alt;
    if (baro_alt > apogee) apogee = baro_alt;

    int v_raw = analogRead(S_RAW);
    voltage = ((v_raw * 5.0f / 1024.0f) / (r2 / (r1 + r2))) + 0.35f;

    if (launched && baro_alt > min_fire_alt) alt_threshold = true;

    if (launched && !hit_apogee && vel < -1.0f && apogee - baro_alt > 2.0f) {
      hit_apogee = true; setLED(1, 0, 0);
    }
    if (hit_apogee && !landed && fabsf(vel) < 1.5f) {
      if (landTime == 0) landTime = now;
      if (now - landTime > 3000) { landed = true; pyro1_armed = false; setLED(0, 1, 0); }
    } else landTime = 0;
  }

  // ---- pyro drive ----
  static unsigned long pyro1_fire_time = 0;
  if (pyro1_state) {
    if (pyro1_fire_time == 0) { pyro1_fire_time = now; digitalWrite(PYRO_PIN, HIGH); }
    if (now - pyro1_fire_time >= pyroFireTime) {
      digitalWrite(PYRO_PIN, LOW); pyro1_state = false; pyro1_fired = true; pyro1_fire_time = 0;
    }
  }

  // ---- LoRa: pre-launch listen for commands + slow TX; in flight TX-only ----
  static unsigned long tLora = 0;
  if (loraOK) {
    if (!launched) {
      int ps = LoRa.parsePacket();
      if (ps) {
        char rb[56]; int i = 0;
        while (LoRa.available() && i < 55) rb[i++] = (char)LoRa.read();
        rb[i] = 0; handleCmd(rb);
      }
      if (now - tLora >= 1000) { tLora = now; sendPacket(false); LoRa.receive(); }
      bool on = (now % 800) < 200;
      setLED(false, ready && on, (!ready) && on);
    } else {
      if (now - tLora >= LORA_INT) { tLora = now; sendPacket(true); }
    }
  }
}
