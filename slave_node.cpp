#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIG =====
const char* ssid = "--------------";
const char* password = "---------------";

const IPAddress MY_IP(192,168,18,201); // Change for each slave
const unsigned int udpPort = 4210;

const IPAddress primaryIP(192,168,18,144);

const uint8_t MY_ADDR = 2; // Change to 3 for slave #2

// ===== FRAME CONSTANTS =====
const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;
const uint8_t TYPE_DATA  = 0x01;
const uint8_t TYPE_ACK   = 0x02;

WiFiUDP udp;
uint8_t expectedSeq = 0;

uint8_t checksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i=0; i<len; i++) x ^= data[i];
  return x;
}

void sendACK(uint8_t dest, uint8_t seqno) {
  uint8_t frame[10];
  uint8_t idx = 0;

  frame[idx++] = START_BYTE;
  frame[idx++] = dest;
  frame[idx++] = MY_ADDR;
  frame[idx++] = TYPE_ACK;
  frame[idx++] = seqno;
  frame[idx++] = 0;   // Payload length = 0

  uint8_t cs = checksum(&frame[1], 5);
  frame[idx++] = cs;
  frame[idx++] = END_BYTE;

  udp.beginPacket(primaryIP, udpPort);
  udp.write(frame, idx);
  udp.endPacket();

  Serial.printf("Sent ACK seq=%d to %d\n", seqno, dest);
}

void setup() {
  Serial.begin(115200);

  WiFi.config(MY_IP, IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.begin(ssid, password);

  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\nConnected, IP: %s\n", WiFi.localIP().toString().c_str());
  udp.begin(udpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {

    uint8_t buf[256];
    udp.read(buf, sizeof(buf));

    if (packetSize < 8) return;
    if (buf[0] != START_BYTE || buf[packetSize - 1] != END_BYTE) return;

    uint8_t dest = buf[1];
    uint8_t src  = buf[2];
    uint8_t type = buf[3];
    uint8_t sseq = buf[4];
    uint8_t len  = buf[5];
    uint8_t cs   = buf[6 + len];

    uint8_t calc = checksum(&buf[1], 5 + len);
    if (cs != calc) return;
    if (dest != MY_ADDR || type != TYPE_DATA) return;

    if (sseq == expectedSeq) {
      Serial.printf("DATA received from %d seq=%d len=%d\n", src, sseq, len);

      if (len > 0) {
        char payload[64];
        memcpy(payload, &buf[6], len);
        payload[len] = '\0';
        Serial.printf("Payload: %s\n", payload);
      }

      sendACK(src, sseq);
      expectedSeq ^= 1;

    } else {
      Serial.println("Duplicate frame received -> re-ACKing");
      sendACK(src, sseq);
    }
  }

  delay(10);
}
