// Ground Station receiver -- validated raw forwarder.
// Rocket telemetry packet (21 fields + checksum), forwarded to the GCS as-is
// with RSSI appended:
//   TID;TIME;ALT;AX;AY;AZ;HDG;STAT;SRV;TEMP;PRS;HMDT;VOLT;INIT;VEL;APG;PID;
//   LAT;LON;SATS;GDIST;RSSI
// All scaling (cm->m, cv->V, e6->deg, dm->m) is done in the Processing GCS.
// Commands from the GCS arrive on serial as "cmd,val;" and go out over LoRa
// as "CMD;FF;cmd;val;cs".  The rocket replies "ACK;cmd;cs".

#include <SPI.h>
#include <LoRa.h>

#define R 3
#define G 4
#define B 5
#define O 6

unsigned long lastPkt = 0;
char cmdBuf[32];
uint8_t cmdBufIdx = 0;

void leds(bool r, bool g, bool b, bool o) {
  digitalWrite(R, r); digitalWrite(G, g); digitalWrite(B, b); digitalWrite(O, o);
}

void sendCommand(int cmd, int val) {
  char buf[40];
  snprintf(buf, sizeof(buf), "CMD;FF;%d;%d", cmd, val);
  uint8_t cs = 0;
  for (char *p = buf; *p; p++) cs += *p;
  LoRa.beginPacket();
  LoRa.print(buf); LoRa.print(';'); LoRa.print(cs);
  LoRa.endPacket();
  Serial.print("CMD_SENT;"); Serial.print(cmd); Serial.print(';'); Serial.println(val);
  LoRa.receive();          // re-enter RX to catch the ACK
}

void setup() {
  Serial.begin(115200);
  pinMode(R, OUTPUT); pinMode(G, OUTPUT); pinMode(B, OUTPUT); pinMode(O, OUTPUT);
  leds(1, 1, 1, 1);
  Serial.println("LoRa Ground Station");
  delay(500);
  leds(0, 0, 0, 0);

  if (!LoRa.begin(433E6)) {
    Serial.println("ERR:LORA_INIT");
    leds(1, 0, 0, 0);
    while (1) ;
  }
  leds(0, 1, 0, 0);
  Serial.println("LoRa OK");
  delay(500);
  leds(0, 0, 0, 0);
  LoRa.receive();
}

void loop() {
  // ---- RECEIVE ----
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    lastPkt = millis();
    leds(0, 0, 0, 1);

    char buf[220];
    int idx = 0;
    while (LoRa.available() && idx < (int)sizeof(buf) - 1) buf[idx++] = (char)LoRa.read();
    buf[idx] = '\0';

    // checksum: last ';'-separated token
    char *lastSep = strrchr(buf, ';');
    if (!lastSep) { leds(1, 0, 0, 1); LoRa.receive(); return; }   // drop quietly
    uint8_t rxCS = (uint8_t)atoi(lastSep + 1);
    *lastSep = '\0';                         // buf now holds the payload only
    uint8_t calcCS = 0;
    for (char *p = buf; *p; p++) calcCS += *p;
    if (calcCS != rxCS) { leds(1, 0, 0, 1); LoRa.receive(); return; }   // corrupted RF, drop quietly

    // ACK from rocket?
    if (strncmp(buf, "ACK", 3) == 0) {
      Serial.println(buf);                   // e.g. "ACK;3"
      leds(0, 1, 0, 0); delay(30); leds(0, 0, 0, 0);
      LoRa.receive();
      return;
    }

    // Telemetry: forward the payload verbatim + RSSI. GCS does the parsing.
    Serial.print(buf);
    Serial.print(';');
    Serial.println(LoRa.packetRssi());

    leds(0, 0, 0, 0);
    LoRa.receive();
  }

  if (millis() - lastPkt > 600) leds(1, 0, 0, 0);   // lost-link

  // ---- SEND (non-blocking serial command "cmd,val;") ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == ';') {
      cmdBuf[cmdBufIdx] = '\0'; cmdBufIdx = 0;
      char *comma = strchr(cmdBuf, ',');
      if (comma) {
        *comma = '\0';
        sendCommand(atoi(cmdBuf), atoi(comma + 1));
      } else {
        Serial.println("ERR:BAD_CMD_FORMAT");
      }
    } else if (cmdBufIdx < (uint8_t)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdBufIdx++] = c;
    }
  }
}
