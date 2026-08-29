#include <ClearCore.h>
#include <Ethernet.h>

// --- Network Configuration (same as main firmware) ---
#define CC_IP_0 192
#define CC_IP_1 168
#define CC_IP_2 3
#define CC_IP_3 200

#define PLC_IP_0 192
#define PLC_IP_1 168
#define PLC_IP_2 3
#define PLC_IP_3 101

const uint16_t PLC_PORT = 1025;
byte mac[] = { 0x24, 0x15, 0x10, 0xB0, 0x00, 0x01 }; // same MAC as firmware

EthernetClient plcClient;
IPAddress plcIp(PLC_IP_0, PLC_IP_1, PLC_IP_2, PLC_IP_3);
IPAddress ccIp(CC_IP_0, CC_IP_1, CC_IP_2, CC_IP_3);

// --- MC Protocol 3E BINARY constants (matching firmware PLC_MC_ASCII=0) ---
const uint8_t MC_REQ_B0 = 0x50, MC_REQ_B1 = 0x00;
const uint8_t MC_RES_B0 = 0xD0, MC_RES_B1 = 0x00;
const uint8_t MC_NETWORK      = 0x00;
const uint8_t MC_PC            = 0xFF;
const uint8_t MC_DEST_IO_LO   = 0xFF, MC_DEST_IO_HI = 0x03;
const uint8_t MC_DEST_STATION = 0x00;
const uint16_t MC_MONITOR_TIMER = 0x0002;
const uint16_t MC_CMD_READ     = 0x0401;
const uint16_t MC_SUB_WORD     = 0x0000;
const uint8_t  MC_DEVICE_M     = 0x90;  // M register device code
const uint8_t  MC_DEVICE_Y     = 0x9D;  // Y register device code

// Response header size in binary mode = 7 bytes (before data-length field)
// Subheader(2) + Network(1) + PC(1) + DestIO(2) + DestStation(1) = 7
// Then DataLength(2) + EndCode(2) + Data...
const int RES_HEADER_BYTES = 7;

unsigned long lastPoll = 0;
String plcRxBuf = "";
int pollStep = 0;  // 0 = read M, 1 = read Y

// State
bool m30 = false, m31 = false, m32 = false;
bool plc_y0 = false, plc_y1 = false, plc_y2 = false;
bool plcConnected = false;
bool plcGotData = false;

// Blink
bool blinkLed = false;
unsigned long lastBlink = 0;

// ── Build a binary MC read-word frame ──────────────────────────
// deviceCode: 0x90 for M, 0x9D for Y
// deviceNum:  starting device number (e.g. 0 for M0)
// numWords:   how many 16-bit words to read
void buildReadFrame(uint8_t *buf, int &len,
                    uint8_t deviceCode, uint32_t deviceNum, uint16_t numWords) {
  // The "body" that goes after the data-length field:
  //   MonitorTimer(2) + Command(2) + Subcommand(2) +
  //   DeviceNum(3) + DeviceCode(1) + NumPoints(2) = 12 bytes
  uint16_t dataLen = 12;

  len = 0;
  // Subheader
  buf[len++] = MC_REQ_B0;
  buf[len++] = MC_REQ_B1;
  // Network
  buf[len++] = MC_NETWORK;
  // PC
  buf[len++] = MC_PC;
  // Dest IO (little-endian)
  buf[len++] = MC_DEST_IO_LO;
  buf[len++] = MC_DEST_IO_HI;
  // Dest Station
  buf[len++] = MC_DEST_STATION;
  // Data Length (little-endian)
  buf[len++] = dataLen & 0xFF;
  buf[len++] = (dataLen >> 8) & 0xFF;
  // Monitor Timer (little-endian)
  buf[len++] = MC_MONITOR_TIMER & 0xFF;
  buf[len++] = (MC_MONITOR_TIMER >> 8) & 0xFF;
  // Command (little-endian)
  buf[len++] = MC_CMD_READ & 0xFF;
  buf[len++] = (MC_CMD_READ >> 8) & 0xFF;
  // Subcommand (little-endian)
  buf[len++] = MC_SUB_WORD & 0xFF;
  buf[len++] = (MC_SUB_WORD >> 8) & 0xFF;
  // Device number (3 bytes, little-endian)
  buf[len++] = deviceNum & 0xFF;
  buf[len++] = (deviceNum >> 8) & 0xFF;
  buf[len++] = (deviceNum >> 16) & 0xFF;
  // Device code (1 byte)
  buf[len++] = deviceCode;
  // Number of points (little-endian)
  buf[len++] = numWords & 0xFF;
  buf[len++] = (numWords >> 8) & 0xFF;
}

// ── Parse a binary MC response ─────────────────────────────────
// Returns true if a complete response was consumed.
// On success, *words is filled with up to maxWords uint16_t values.
// *gotWords is set to the number of words actually returned.
bool parseResponse(uint16_t *words, int maxWords, int *gotWords) {
  *gotWords = 0;
  int bufLen = plcRxBuf.length();

  // Minimum: header(7) + dataLen(2) + endCode(2) = 11 bytes
  if (bufLen < 11) return false;

  // Data length is at offset 7, 2 bytes little-endian
  uint16_t dataLen = (uint8_t)plcRxBuf[7] | ((uint8_t)plcRxBuf[8] << 8);
  int totalLen = 9 + dataLen;  // 7 (header) + 2 (dataLen field) + dataLen

  if (bufLen < totalLen) return false;

  // Check subheader
  if ((uint8_t)plcRxBuf[0] != MC_RES_B0 || (uint8_t)plcRxBuf[1] != MC_RES_B1) {
    Serial.print("[ERROR] Bad subheader: ");
    Serial.print((uint8_t)plcRxBuf[0], HEX);
    Serial.print(" ");
    Serial.println((uint8_t)plcRxBuf[1], HEX);
    plcRxBuf = plcRxBuf.substring(totalLen);
    return true;
  }

  // End code at offset 9, 2 bytes little-endian
  uint16_t endCode = (uint8_t)plcRxBuf[9] | ((uint8_t)plcRxBuf[10] << 8);
  if (endCode != 0) {
    Serial.print("[ERROR] PLC end code: 0x");
    Serial.println(endCode, HEX);
    plcRxBuf = plcRxBuf.substring(totalLen);
    return true;
  }

  // Data words start at offset 11, 2 bytes each (little-endian)
  int dataBytes = dataLen - 2;  // subtract end code
  int numWords = dataBytes / 2;
  for (int i = 0; i < numWords && i < maxWords; i++) {
    int off = 11 + i * 2;
    words[i] = (uint8_t)plcRxBuf[off] | ((uint8_t)plcRxBuf[off + 1] << 8);
    (*gotWords)++;
  }

  plcRxBuf = plcRxBuf.substring(totalLen);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  Serial.println("=== ClearCore PLC Test (MC 3E BINARY) ===");

  // ClearCore IO pins for M30/M31/M32 LEDs
  ConnectorIO0.Mode(Connector::OUTPUT_DIGITAL);
  ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
  ConnectorIO2.Mode(Connector::OUTPUT_DIGITAL);

  // ClearCore IO pins for Y0/Y1/Y2 LEDs
  ConnectorIO3.Mode(Connector::OUTPUT_DIGITAL);
  ConnectorIO4.Mode(Connector::OUTPUT_DIGITAL);
  ConnectorIO5.Mode(Connector::OUTPUT_DIGITAL);

  Ethernet.begin(mac, ccIp);
  Serial.print("ClearCore IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("PLC target:   ");
  Serial.print(PLC_IP_0); Serial.print(".");
  Serial.print(PLC_IP_1); Serial.print(".");
  Serial.print(PLC_IP_2); Serial.print(".");
  Serial.print(PLC_IP_3); Serial.print(":");
  Serial.println(PLC_PORT);
}

void loop() {
  // ── Blink timer ──
  if (millis() - lastBlink > 250) {
    blinkLed = !blinkLed;
    lastBlink = millis();
  }

  // ── LED output ──
  // M sensors -> IO-0, IO-1, IO-2: solid ON when true, blink when false
  ConnectorIO0.State(m30 ? true : blinkLed);
  ConnectorIO1.State(m31 ? true : blinkLed);
  ConnectorIO2.State(m32 ? true : blinkLed);

  // Y outputs -> IO-3, IO-4, IO-5: solid ON when true, blink when false
  ConnectorIO3.State(plc_y0 ? true : blinkLed);
  ConnectorIO4.State(plc_y1 ? true : blinkLed);
  ConnectorIO5.State(plc_y2 ? true : blinkLed);

  // ── Connection ──
  if (!plcClient.connected()) {
    plcConnected = false;
    Serial.println("Connecting to PLC...");
    plcClient.stop();
    if (plcClient.connect(plcIp, PLC_PORT)) {
      Serial.println(">>> Connected to PLC successfully! <<<");
      plcConnected = true;
      pollStep = 0;
    } else {
      Serial.println("Connection FAILED. Retrying in 2s...");
      delay(2000);
      return;
    }
  }

  // ── Send poll every 300ms, alternating M and Y ──
  if (millis() - lastPoll > 300) {
    lastPoll = millis();

    uint8_t frame[32];
    int frameLen = 0;

    if (pollStep == 0) {
      // Read 3 words starting at M0 -> covers M0..M47
      buildReadFrame(frame, frameLen, MC_DEVICE_M, 0, 3);
    } else {
      // Read 1 word starting at Y0 -> covers Y0..YF
      buildReadFrame(frame, frameLen, MC_DEVICE_Y, 0, 1);
    }

    plcClient.write(frame, frameLen);
    plcClient.flush();
    plcRxBuf = "";
  }

  // ── Read incoming bytes ──
  while (plcClient.available()) {
    char c = plcClient.read();
    plcRxBuf += c;
  }

  // ── Parse response ──
  uint16_t words[4];
  int gotWords = 0;

  if (parseResponse(words, 4, &gotWords)) {
    plcGotData = true;

    if (pollStep == 0 && gotWords >= 2) {
      // words[0] = M0-M15, words[1] = M16-M31, words[2] = M32-M47
      // M30 = bit 14 of word[1], M31 = bit 15 of word[1], M32 = bit 0 of word[2]
      if (gotWords >= 3) {
        m30 = (words[1] >> 14) & 1;
        m31 = (words[1] >> 15) & 1;
        m32 = (words[2] >> 0) & 1;
      } else {
        m30 = (words[1] >> 14) & 1;
        m31 = (words[1] >> 15) & 1;
      }

      Serial.print("M word0="); Serial.print(words[0], HEX);
      if (gotWords >= 2) { Serial.print(" word1="); Serial.print(words[1], HEX); }
      if (gotWords >= 3) { Serial.print(" word2="); Serial.print(words[2], HEX); }
      Serial.print(" -> M30:"); Serial.print(m30);
      Serial.print(" M31:"); Serial.print(m31);
      Serial.print(" M32:"); Serial.println(m32);

    } else if (pollStep == 1 && gotWords >= 1) {
      plc_y0 = (words[0] >> 0) & 1;
      plc_y1 = (words[0] >> 1) & 1;
      plc_y2 = (words[0] >> 2) & 1;

      Serial.print("Y word0="); Serial.print(words[0], HEX);
      Serial.print(" -> Y0:"); Serial.print(plc_y0);
      Serial.print(" Y1:"); Serial.print(plc_y1);
      Serial.print(" Y2:"); Serial.println(plc_y2);
    }

    // Advance to next step
    pollStep = (pollStep + 1) % 2;
  }
}
