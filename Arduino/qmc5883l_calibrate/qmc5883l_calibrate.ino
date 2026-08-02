/*
 * qmc5883l_calibrate.ino  --  hard-iron calibration for Quark's QMC5883L (0x0D).
 *
 * Same procedure as the QMC5883P version: power the electronics as in flight,
 * and during the 30 s window slowly TUMBLE the board through every orientation
 * (figure-eights + rolls on all axes) so each axis sees its true min/max field.
 *
 * Output: MAG_OFF[3] to SD (MAGCAL.TXT) and Serial. Paste into Quark.ino.
 *
 * LED:  blue = get ready | magenta blink = ROTATE NOW | green heartbeat = done
 *       yellow = span too small | red blink = QMC not found
 * NOTE: Quark's LED pins are R=A1, G=A0 (swapped vs PulseR).
 */
#include <Wire.h>
#include <SPI.h>
#include <SD_fix.h>

#define LED_R A1
#define LED_G A0
#define LED_B A2
#define SD_CS 4
#define QMC_ADDR 0x0D
#define COLLECT_MS 30000

File f;
int16_t mn[3] = { 32767,  32767,  32767};
int16_t mx[3] = {-32768, -32768, -32768};

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, !r); digitalWrite(LED_G, !g); digitalWrite(LED_B, !b);
}
bool qmcInit() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission()) return false;
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();  // set/reset
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();  // cont,200Hz,8G
  return true;
}
bool qmcRead(int16_t *x, int16_t *y, int16_t *z) {
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6);
  if (Wire.available() < 6) return false;
  *x = Wire.read() | (Wire.read() << 8);
  *y = Wire.read() | (Wire.read() << 8);
  *z = Wire.read() | (Wire.read() << 8);
  return true;
}

void setup() {
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  setLED(0, 0, 0);
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000); SPI.begin();

  if (!qmcInit()) { for (;;) { setLED(1,0,0); delay(120); setLED(0,0,0); delay(120); } }
  SD.begin(SD_CS);
  f = SD.open("MAGCAL.TXT", FILE_WRITE);

  setLED(0, 0, 1); delay(2000);
}

void loop() {
  int16_t x, y, z;
  uint32_t t0 = millis();
  while (millis() - t0 < COLLECT_MS) {
    if (qmcRead(&x, &y, &z)) {
      int16_t v[3] = {x, y, z};
      for (uint8_t k = 0; k < 3; k++) {
        if (v[k] < mn[k]) mn[k] = v[k];
        if (v[k] > mx[k]) mx[k] = v[k];
      }
    }
    bool on = (millis() % 400) < 200;
    setLED(on, false, on);
    delay(15);
  }

  long ox = ((long)mx[0] + mn[0]) / 2;
  long oy = ((long)mx[1] + mn[1]) / 2;
  long oz = ((long)mx[2] + mn[2]) / 2;
  long sx = (long)mx[0] - mn[0], sy = (long)mx[1] - mn[1], sz = (long)mx[2] - mn[2];
  bool weak = (sx < 500 || sy < 500 || sz < 500);

  for (uint8_t pass = 0; pass < 2; pass++) {
    Print &o = (pass == 0) ? (Print &)Serial : (Print &)f;
    if (pass == 1 && !f) break;
    o.println();
    o.println(F("# --- paste into Quark.ino ---"));
    o.print(F("const int MAG_OFF[3] = {"));
    o.print(ox); o.print(F(", ")); o.print(oy); o.print(F(", ")); o.print(oz); o.println(F("};"));
    o.print(F("# span x,y,z: ")); o.print(sx); o.print(' '); o.print(sy); o.print(' '); o.println(sz);
    if (weak) o.println(F("# WARNING: small span -- rotate more fully and redo"));
  }
  if (f) { f.flush(); f.close(); }

  for (;;) {
    if (weak) { setLED(1,1,0); delay(150); setLED(0,0,0); delay(400); }
    else      { setLED(0,1,0); delay(150); setLED(0,0,0); delay(1400); }
  }
}
