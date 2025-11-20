#include <WiFi.h>
#include <WiFiUdp.h>

// ======================= NETWORK CONFIG =======================
const char* ssid = "----------";
const char* password = "--------------";

const IPAddress slave1IP(192, 168, 18, 201);
const IPAddress slave2IP(192, 168, 18, 202);
const unsigned int udpPort = 4210;

// ======================= NODE ADDRESSES =======================
const uint8_t MY_ADDR = 1;
const uint8_t SLAVE1  = 2;
const uint8_t SLAVE2  = 3;

// ======================= FRAME DEFINITIONS =======================
const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;
const uint8_t TYPE_DATA  = 0x01;
const uint8_t TYPE_ACK   = 0x02;

// ======================= TIMING CONFIG =======================
const unsigned long SEND_INTERVAL_MS = 2000;
const unsigned long ACK_TIMEOUT_MS   = 1500;
const uint8_t MAX_RETRIES = 10;      // Maximum retransmission attempts
const uint8_t FRAMES_PER_NODE = 5;   // Frames per user selection

WiFiUDP udp;

// ======================= RUNTIME VARIABLES =======================
uint8_t seq = 0;
bool waitingForAck = false;
uint8_t currentDest = 0;
unsigned long txStartTime = 0;
String currentPayload;
unsigned long lastSend = 0;
uint8_t retryCount = 0;
uint8_t framesSentToNode = 0;

// ======================= UTILITY =======================
uint8_t checksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

void sendFrame(IPAddress destIP, uint8_t dest, uint8_t type, uint8_t seqno, const uint8_t *payload, uint8_t len) {
  uint8_t frame[256];
  uint8_t idx = 0;

  frame[idx++] = START_BYTE;
  frame[idx++] = dest;
  frame[idx++] = MY_ADDR;
  frame[idx++] = type;
  frame[idx++] = seqno;
  frame[idx++] = len;

  for (uint8_t i = 0; i < len; i++) frame[idx++] = payload[i];

  uint8_t cs = checksum(&frame[1], 5 + len);
  frame[idx++] = cs;
  frame[idx++] = END_BYTE;

  udp.beginPacket(destIP, udpPort);
  udp.write(frame, idx);
  udp.endPacket();

  Serial.printf("[TX] Sent frame to Node %d | Seq=%d | Len=%d | Type=%02X\n", dest, seqno, len, type);
}

bool checkForACK(uint8_t &ackSeq, uint8_t &ackSrc) {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return false;

  uint8_t buf[256];
  int len = udp.read(buf, sizeof(buf));

  if (len < 8) return false;
  if (buf[0] != START_BYTE || buf[len - 1] != END_BYTE) return false;

  uint8_t dest = buf[1];
  uint8_t src  = buf[2];
  uint8_t type = buf[3];
  uint8_t sseq = buf[4];
  uint8_t datalen = buf[5];
  uint8_t cs = buf[6 + datalen];
  uint8_t calc = checksum(&buf[1], 5 + datalen);

  if (cs != calc) {
    Serial.println("[WARN] Bad checksum, ignoring.");
    return false;
  }

  if (dest != MY_ADDR || type != TYPE_ACK) return false;

  ackSeq = sseq;
  ackSrc = src;
  return true;
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-S3 Primary Node Booting ===");

  WiFi.disconnect(true, true);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.printf("Connecting to %s", ssid);
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    if (millis() - startAttempt > 20000) {
      Serial.println("\n[ERROR] Failed to connect to WiFi.");
      while (1);
    }
  }

  Serial.printf("\nConnected! IP Address: %s\n", WiFi.localIP().toString().c_str());

  if (udp.begin(udpPort)) {
    Serial.printf("UDP started on port %d\n", udpPort);
  } else {
    Serial.println("UDP initialization failed!");
  }

  Serial.println("\n=== Node Ready ===");
  Serial.println("Enter slave node number (2 or 3) to start transmission:");
}

// ======================= LOOP =======================
void loop() {
  // ===== User Node Selection =====
  if (currentDest == 0 && Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "2") {
      currentDest = SLAVE1;
      Serial.println("Selected Node 2 (Slave 1)");
    } else if (input == "3") {
      currentDest = SLAVE2;
      Serial.println("Selected Node 3 (Slave 2)");
    } else {
      Serial.println("Invalid input. Enter 2 or 3.");
      return;
    }

    framesSentToNode = 0;
    waitingForAck = false;
    retryCount = 0;
    lastSend = millis();
  }

  if (currentDest == 0) return;

  unsigned long now = millis();

  // ===== Send new frame if idle =====
  if (!waitingForAck && framesSentToNode < FRAMES_PER_NODE && now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    IPAddress destIP = (currentDest == SLAVE1) ? slave1IP : slave2IP;

    currentPayload = "Hello Node " + String(currentDest);
    uint8_t dataBuf[64];
    uint8_t len = currentPayload.length();
    memcpy(dataBuf, currentPayload.c_str(), len);

    sendFrame(destIP, currentDest, TYPE_DATA, seq, dataBuf, len);
    waitingForAck = true;
    txStartTime = now;
    retryCount = 0;
  }

  // ===== Check for ACK =====
  if (waitingForAck) {
    uint8_t ackSeq, ackSrc;

    if (checkForACK(ackSeq, ackSrc)) {
      if (ackSrc == currentDest && ackSeq == seq) {
        Serial.printf("[RX] ACK received from Node %d | Seq=%d\n", ackSrc, ackSeq);
        seq ^= 1;
        waitingForAck = false;
        framesSentToNode++;
      }
    } 
    else if (now - txStartTime > ACK_TIMEOUT_MS) {
      retryCount++;
      if (retryCount > MAX_RETRIES) {
        Serial.printf("[ERROR] No ACK after %d retries. Stopping.\n", MAX_RETRIES);
        waitingForAck = false;
        currentDest = 0;
        Serial.println("Enter next node (2 or 3):");
        return;
      }

      Serial.printf("[WARN] ACK timeout. Retrying (%d/%d)...\n", retryCount, MAX_RETRIES);
      IPAddress destIP = (currentDest == SLAVE1) ? slave1IP : slave2IP;

      uint8_t dataBuf[64];
      uint8_t len = currentPayload.length();
      memcpy(dataBuf, currentPayload.c_str(), len);

      sendFrame(destIP, currentDest, TYPE_DATA, seq, dataBuf, len);
      txStartTime = now;
    }
  }

  // ===== Done sending =====
  if (framesSentToNode >= FRAMES_PER_NODE && !waitingForAck) {
    Serial.printf("Completed sending %d frames to Node %d.\n", FRAMES_PER_NODE, currentDest);
    currentDest = 0;
    Serial.println("Enter next node (2 or 3):");
  }

  delay(10);
}
