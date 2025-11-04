#include <Arduino.h>
#include <ESP32Servo.h>

/*
  Minimal Optical RX+TX (ESP32)
  -----------------------------
  RX (receiver):
    - Lean Morse with auto-polarity (BRIGHT => ΔN positive)
    - Dynamic dark edge: DARK if ΔN <= max(TH_OFF, TH_ON-1)
    - No bright-time baseline creep (prevents dash erosion)
    - Quiet during RECV (only [DECODE] at exit) + sparse [SCAN] logs

  TX (transmitter):
    - Non-blocking Morse transmitter driving 4 LEDs together (GPIO 19,21,23,22)
    - :send <text> appends to TX queue (A–Z, 0–9, space)
    - Uses UNIT_MS for timing (same as RX)
    - Optional :txinv to invert LED logic if wired active-low

  Pins:
    LDR_LEFT  -> GPIO32 (ADC1_CH4)
    LDR_RIGHT -> GPIO35 (ADC1_CH7, input-only)
    SERVO     -> GPIO25
    TX LEDs   -> GPIO19, GPIO21, GPIO23, GPIO22  (default active HIGH)
*/

// ---------------- Pins ----------------
const int LDR_LEFT_PIN  = 32;
const int LDR_RIGHT_PIN = 35;
const int SERVO_PIN     = 25;

// TX LED pins (all mirror the same signal)
const int TX_LED_PINS[4] = {19, 21, 23, 22};

// ---------------- Thresholds (Δ = baseline - raw) ----------------
int TH_ON  = 6;   // enter/consider BRIGHT when ΔN >= TH_ON
int TH_OFF = 3;   // base DARK threshold
inline int TH_DARK_EDGE(){ return max(TH_OFF, TH_ON - 1); }

// ---------------- Morse timing (single knob) ----------------
unsigned long UNIT_MS = 240;  // dot length (~5 WPM)

// Derived timing (scaled from UNIT)
inline unsigned long DEBOUNCE_MS()    { return (unsigned long)(0.55f * UNIT_MS); } // ignore short flips
inline unsigned long HOLD_BRIGHT_MS() { return (unsigned long)(0.30f * UNIT_MS); } // must stay bright
inline unsigned long HOLD_DARK_MS()   { return (unsigned long)(0.80f * UNIT_MS); } // must stay dark
inline unsigned long MIN_PULSE_MS()   { return (unsigned long)(0.60f * UNIT_MS); } // reject glints
inline unsigned long IDLE_MS()        { return 8ul * UNIT_MS; }                    // RX end-of-word/session

// Pulse/gap classification (tolerant to jitter)
char classifyOn(unsigned long onMs){
  if (onMs < MIN_PULSE_MS()) return '\0';
  return (onMs < (unsigned long)(2.5f * UNIT_MS)) ? '.' : '-';
}
uint8_t classifyGap(unsigned long offMs){                   // 0=intra, 1=letter, 2=word
  if (offMs >= (unsigned long)(6.5f * UNIT_MS)) return 2;  // ~7U
  if (offMs >= (unsigned long)(2.6f * UNIT_MS)) return 1;  // ~3U
  return 0;
}

// ---------------- Motion & timing ----------------
const unsigned SCAN_STEP_PERIOD_MS = 120;
const int ANGLE_MIN = 0, ANGLE_MAX = 180;
const int ANGLE_STEP = 1;

// ---------------- Track exit guards ----------------
const unsigned DARK_TIMEOUT_MS   = 2000;
const unsigned BRIGHT_TIMEOUT_MS = 2500;
const unsigned NOCHANGE_MS       = 3000;
const int      DELTA_EPS         = 2;

// ---------------- Logging controls ----------------
const bool LOG_SCAN_SAMPLES = true;   // keep sparse [SCAN]
const bool LOG_RECV_SAMPLES = false;  // silence RX sampling logs
const bool QUIET_RECV_TEXT  = true;   // silence [SYM]/[CHAR]/[WORD]/[SOFAR]
const bool LOG_TX           = false;  // set true to see TX transitions
const unsigned SCAN_PRINT_MS = 300;
const unsigned TRK_PRINT_MS  = 400;

// ---------------- Internals (RX) ----------------
Servo servo;
int angle = 0, dir = +1;
unsigned long lastMoveMs = 0;
unsigned long lastScanPrint = 0, lastTrkPrint = 0;

float baseL = 0, baseR = 0;        // ambient baselines (EWMA)
const float A_SCAN      = 0.15f;   // scan baseline warm-up
const float A_TRK_DARK  = 0.05f;   // follow ambient faster in dark
const float A_TRK_BRT   = 0.0f;    // NO baseline creep while bright

inline int deltaRaw(int raw, float base){ return (int)(base - raw); }

// stable ADC read (median-of-3)
inline int median3(int a,int b,int c){
  if (a > b) { int t=a; a=b; b=t; }
  if (b > c) { int t=b; b=c; c=t; }
  if (a > b) { int t=a; a=b; b=t; }
  return b;
}
int readADCstable(int pin){
  int a = analogRead(pin); delayMicroseconds(40);
  int b = analogRead(pin); delayMicroseconds(40);
  int c = analogRead(pin);
  return median3(a,b,c);
}

void warmBaseline(){
  for (int i=0;i<40;i++){
    int rl = readADCstable(LDR_LEFT_PIN);
    int rr = readADCstable(LDR_RIGHT_PIN);
    if (baseL == 0 && baseR == 0){ baseL = rl; baseR = rr; }
    baseL = A_SCAN*rl + (1-A_SCAN)*baseL;
    baseR = A_SCAN*rr + (1-A_SCAN)*baseR;
    delay(10);
  }
}

enum State { SCAN, TRACK };
State state = SCAN;
bool useLeft = true;

// TRACK timers/state
unsigned long darkStartMs   = 0; // 0 = currently bright
unsigned long brightStartMs = 0; // 0 = currently dark
unsigned long noChangeStart = 0;
int lastDeltaP = 0;              // store normalized Δ (ΔN)

// --- Auto-polarity ---
int pol = +1; // +1: ΔN = (base - raw), -1: ΔN = -(base - raw)
inline int dNorm(int raw, float base){ return pol * ((int)(base - raw)); }

// --- Morse lookup (both RX and TX) ---
struct MMap { const char* code; char ch; };
static const MMap MORSE_TABLE[] = {
  {".-",'A'},{"-...",'B'},{"-.-.",'C'},{"-..",'D'},{"." ,'E'},
  {"..-.",'F'},{"--.",'G'},{"....",'H'},{"..",'I' },{".---",'J'},
  {"-.-",'K'},{".-..",'L'},{"--",'M' },{"-.",'N' },{"---",'O'},
  {".--.",'P'},{"--.-",'Q'},{".-.",'R'},{"...",'S'},{"-", 'T'},
  {"..-",'U'},{"...-",'V'},{".--",'W'},{"-..-",'X'},{"-.--",'Y'},
  {"--..",'Z'},
  {"-----",'0'},{".----",'1'},{"..---",'2'},{"...--",'3'},{"....-",'4'},
  {".....",'5'},{"-....",'6'},{"--...",'7'},{"---..",'8'},{"----.",'9'}
};
char morseToChar(const String& code){
  for (size_t i=0;i<sizeof(MORSE_TABLE)/sizeof(MORSE_TABLE[0]);++i)
    if (code == MORSE_TABLE[i].code) return MORSE_TABLE[i].ch;
  return '?';
}
const char* charToMorse(char c){
  if (c>='a' && c<='z') c = c - 'a' + 'A';
  if (c>='A' && c<='Z'){
    for (size_t i=0;i<sizeof(MORSE_TABLE)/sizeof(MORSE_TABLE[0]);++i)
      if (MORSE_TABLE[i].ch == c) return MORSE_TABLE[i].code;
  }
  if (c>='0' && c<='9'){
    for (size_t i=0;i<sizeof(MORSE_TABLE)/sizeof(MORSE_TABLE[0]);++i)
      if (MORSE_TABLE[i].ch == c) return MORSE_TABLE[i].code;
  }
  if (c==' ') return ""; // handled as special word gap
  return nullptr;        // unsupported
}

// RX buffers
String symBuf, msgBuf;

void flushLetter(){
  if (!symBuf.length()) return;
  char c = morseToChar(symBuf);
  if (!QUIET_RECV_TEXT){
    Serial.print("[CHAR] "); Serial.print(symBuf); Serial.print(" -> "); Serial.println(c);
  }
  msgBuf += c;
  symBuf = "";
}
void flushIfAnyAndPrintDecode(const char* reason){
  flushLetter();
  if (msgBuf.length()){
    Serial.print("[DECODE] "); Serial.println(msgBuf);
  }
  Serial.print("[SCAN] "); Serial.print(reason); Serial.println(" → resume scanning");
  msgBuf = "";
}

// --- Edge detector (RX) ---
bool brightState = false;             // stable state (BRIGHT/DARK)
unsigned long lastEdgeMs = 0;         // time of last stable edge
unsigned long candBrightStart = 0;    // candidate bright hold start
unsigned long candDarkStart   = 0;    // candidate dark   hold start
unsigned long trackEnterMs    = 0;    // for idle/limit exits

// ---------------- Transmitter (non-blocking) ----------------
enum TxState { TX_IDLE, TX_ON, TX_OFF };
TxState txState = TX_IDLE;
unsigned long txUntil = 0;            // next transition time (ms)
String txQueue = "";                  // queued text to send (uppercase/space)
size_t txMsgPos = 0;                  // current char index in txQueue
const char* txCode = nullptr;         // morse code for current letter
size_t txCodePos = 0;                 // symbol index within txCode (for current letter)
bool TX_INVERT = false;               // set true if LEDs are wired active-low

void tx_leds(bool on){
  int level = on ^ TX_INVERT ? HIGH : LOW;
  for (int i=0;i<4;i++) digitalWrite(TX_LED_PINS[i], level);
}
void tx_reset(){
  txState = TX_IDLE;
  txUntil = 0;
  txMsgPos = 0;
  txCode = nullptr;
  txCodePos = 0;
  tx_leds(false);
}
inline unsigned long U(unsigned mult){ return mult * UNIT_MS; }

// Advance transmitter state machine
void updateTransmitter(unsigned long now){
  if (txState == TX_IDLE){
    if (txMsgPos >= txQueue.length()) return; // nothing queued

    // get next character
    char c = txQueue[txMsgPos];
    if (c == ' '){
      // word gap 7U
      tx_leds(false);
      txState = TX_OFF;
      txUntil = now + U(7);
      txMsgPos++;                      // consume space
      if (LOG_TX) Serial.println("[TX] word gap 7U");
      return;
    }
    txCode = charToMorse(c);
    if (!txCode || !txCode[0]){
      // unsupported or empty -> skip
      txMsgPos++;
      return;
    }
    txCodePos = 0;
    // start first symbol ON
    tx_leds(true);
    txState = TX_ON;
    txUntil = now + ((txCode[txCodePos]=='.') ? U(1) : U(3));
    if (LOG_TX) { Serial.print("[TX] ON "); Serial.println(txCode[txCodePos]=='.' ? "dot" : "dash"); }
    return;
  }

  if (now < txUntil) return; // waiting for next transition

  if (txState == TX_ON){
    // finished an ON symbol; decide next gap
    tx_leds(false);
    txCodePos++;
    if (txCode[txCodePos] != '\0'){
      // intra-symbol gap 1U
      txState = TX_OFF;
      txUntil = now + U(1);
      if (LOG_TX) Serial.println("[TX] gap intra 1U");
    } else {
      // end of letter: choose gap 3U or 7U if next is space
      unsigned long gap = U(3);
      if (txMsgPos + 1 < txQueue.length() && txQueue[txMsgPos+1] == ' '){
        gap = U(7);
        txMsgPos++; // consume the space here so we don't schedule another gap for it
        if (LOG_TX) Serial.println("[TX] gap word 7U (space consumed)");
      } else {
        if (LOG_TX) Serial.println("[TX] gap letter 3U");
      }
      txState = TX_OFF;
      txUntil = now + gap;
      txMsgPos++; // move to next character
    }
    return;
  }

  if (txState == TX_OFF){
    // if we were between symbols, resume ON of current letter; else start next char
    if (txCode && txCode[txCodePos] != '\0'){
      // resume next symbol of current letter
      tx_leds(true);
      txState = TX_ON;
      txUntil = now + ((txCode[txCodePos]=='.') ? U(1) : U(3));
      if (LOG_TX) { Serial.print("[TX] ON "); Serial.println(txCode[txCodePos]=='.' ? "dot" : "dash"); }
      return;
    } else {
      // move to next char (if any)
      if (txMsgPos >= txQueue.length()){
        txState = TX_IDLE;
        tx_leds(false);
        if (LOG_TX) Serial.println("[TX] done");
        return;
      }
      char c = txQueue[txMsgPos];
      if (c == ' '){
        // word gap 7U
        tx_leds(false);
        txState = TX_OFF;
        txUntil = now + U(7);
        txMsgPos++;
        if (LOG_TX) Serial.println("[TX] word gap 7U");
        return;
      }
      txCode = charToMorse(c);
      if (!txCode || !txCode[0]){ txMsgPos++; return; } // skip unsupported
      txCodePos = 0;
      tx_leds(true);
      txState = TX_ON;
      txUntil = now + ((txCode[txCodePos]=='.') ? U(1) : U(3));
      if (LOG_TX) { Serial.print("[TX] ON "); Serial.println(txCode[txCodePos]=='.' ? "dot" : "dash"); }
      return;
    }
  }
}

// ---------------- Setup ----------------
void handleCommand(); // fwd
void pollSerial();    // fwd

void setup(){
  Serial.begin(115200);
  delay(200);

  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  // Servo + LEDs
  servo.attach(SERVO_PIN);
  servo.write(angle);
  for (int i=0;i<4;i++){ pinMode(TX_LED_PINS[i], OUTPUT); digitalWrite(TX_LED_PINS[i], LOW); }

  warmBaseline();
  Serial.println("[INIT] RX+TX ready (4 LEDs), auto-polarity, quiet RX. Scanning...");
  delay(600);

  unsigned long now = millis();
  lastMoveMs = now;
  lastScanPrint = now;
  lastTrkPrint  = now;

  tx_reset();
}

// ---------------- Loop ----------------
void loop(){
  unsigned long now = millis();

  // 1) Handle CLI input
  pollSerial();

  // 2) Parallel TX (non-blocking)
  updateTransmitter(now);

  // 3) RX/SCAN logic
  if (state == SCAN){
    // gentle sweep
    if (now - lastMoveMs >= SCAN_STEP_PERIOD_MS){
      lastMoveMs = now;
      int next = angle + dir*ANGLE_STEP;
      if (next >= ANGLE_MAX){ next = ANGLE_MAX; dir = -1; }
      if (next <= ANGLE_MIN){ next = ANGLE_MIN; dir = +1; }
      angle = next;
      servo.write(angle);
    }

    // read both, update baselines
    int rawL = readADCstable(LDR_LEFT_PIN);
    int rawR = readADCstable(LDR_RIGHT_PIN);
    baseL = A_SCAN*rawL + (1-A_SCAN)*baseL;
    baseR = A_SCAN*rawR + (1-A_SCAN)*baseR;

    int dL0 = deltaRaw(rawL, baseL);
    int dR0 = deltaRaw(rawR, baseR);
    int aL  = abs(dL0);
    int aR  = abs(dR0);

    // sparse scan log
    if (LOG_SCAN_SAMPLES && (now - lastScanPrint >= SCAN_PRINT_MS)){
      lastScanPrint = now;
      Serial.print("[SCAN] ang="); Serial.print(angle);
      Serial.print("  L(raw/base/Δ)="); Serial.print(rawL); Serial.print("/");
      Serial.print((int)baseL); Serial.print("/"); Serial.print(dL0);
      Serial.print("  R(raw/base/Δ)="); Serial.print(rawR); Serial.print("/");
      Serial.print((int)baseR); Serial.print("/"); Serial.println(dR0);
    }

    // lock? (sign-agnostic)
    if (aL >= TH_ON || aR >= TH_ON){
      useLeft = (aL >= aR);
      state = TRACK;

      // decide polarity so that BRIGHT → ΔN positive
      int rawLock   = useLeft ? rawL   : rawR;
      float baseLock= useLeft ? baseL  : baseR;
      int dLock     = deltaRaw(rawLock, baseLock);
      pol = (dLock >= 0) ? +1 : -1;        // if bright raises raw, flip polarity
      int dLockN    = pol * dLock;
      bool bright   = (dLockN >= TH_ON);

      // init TRACK windows based on current brightness
      darkStartMs   = bright ? 0 : now;
      brightStartMs = bright ? now : 0;
      noChangeStart = now;
      lastDeltaP    = dLockN;

      // init Morse edge detector
      brightState      = bright;
      lastEdgeMs       = now;
      candBrightStart  = bright ? now : 0;
      candDarkStart    = bright ? 0   : now;
      trackEnterMs     = now;
      symBuf = ""; msgBuf = "";

      Serial.print("[LOCK] ang="); Serial.print(angle);
      Serial.print(" via "); Serial.print(useLeft ? "LEFT" : "RIGHT");
      Serial.print("  pol="); Serial.println(pol);
      return;
    }
    return;
  }

  // ---------------- TRACK ----------------
  int rawP  = readADCstable(useLeft ? LDR_LEFT_PIN : LDR_RIGHT_PIN);
  float &basePref = useLeft ? baseL : baseR;

  // normalized Δ (ΔN)
  int dN    = dNorm(rawP, basePref);

  // thresholds with dynamic dark edge
  bool rawBright = (dN >= TH_ON);
  bool rawDark   = (dN <= TH_DARK_EDGE());

  // Baseline policy: NEVER creep while bright; follow in dark only
  if (rawBright){
    // no creep while ON
  } else if (rawDark){
    basePref = A_TRK_DARK*rawP + (1-A_TRK_DARK)*basePref;
  }

  // --- windows for exits (unchanged behavior) ---
  if (rawBright){
    if (brightStartMs == 0) brightStartMs = now;
    darkStartMs = 0;
  } else {
    if (darkStartMs == 0)   darkStartMs = now;
    brightStartMs = 0;
  }

  // No-change window (use normalized Δ)
  if (abs(dN - lastDeltaP) <= DELTA_EPS){
    // continue no-change
  } else {
    noChangeStart = now;
    lastDeltaP = dN;
  }

  // --- Morse edge detection (tiny temporal hold + debounce) ---
  if (rawBright){
    candDarkStart = 0;
    if (!candBrightStart) candBrightStart = now;

    if (!brightState &&
        (now - candBrightStart >= HOLD_BRIGHT_MS()) &&
        (now - lastEdgeMs >= DEBOUNCE_MS()))
    {
      // DARK -> BRIGHT: OFF gap ended
      unsigned long offMs = now - lastEdgeMs;
      uint8_t g = classifyGap(offMs);
      if (g == 2){ flushLetter(); /* quiet */ }
      else if (g == 1){ flushLetter(); }

      brightState = true;
      lastEdgeMs  = now;
    }
  } else if (rawDark){
    candBrightStart = 0;
    if (!candDarkStart) candDarkStart = now;

    if (brightState &&
        (now - candDarkStart >= HOLD_DARK_MS()) &&
        (now - lastEdgeMs >= DEBOUNCE_MS()))
    {
      // BRIGHT -> DARK: ON pulse ended
      unsigned long onMs = now - lastEdgeMs;
      char sym = classifyOn(onMs);
      if (sym != '\0'){
        symBuf += sym; // quiet
      }
      brightState = false;
      lastEdgeMs  = now;
    }
  } else {
    // in-between zone -> reset candidates
    candBrightStart = 0;
    candDarkStart   = 0;
  }

  // --- exit conditions (and flush decode) ---
  if (darkStartMs && (now - darkStartMs >= DARK_TIMEOUT_MS)){
    flushIfAnyAndPrintDecode("Dark timeout");
    state = SCAN; return;
  }
  if (brightStartMs && (now - brightStartMs >= BRIGHT_TIMEOUT_MS)){
    flushIfAnyAndPrintDecode("Bright-stuck timeout");
    state = SCAN; return;
  }
  if (now - noChangeStart >= NOCHANGE_MS){
    if (now - lastEdgeMs >= IDLE_MS()){
      flushLetter();
    }
    flushIfAnyAndPrintDecode("No-change timeout");
    state = SCAN; return;
  }
}

// ------------- Mini CLI -------------
// :unit <ms>        -> set dot length (affects TX and RX)
// :th <on> <off>    -> set thresholds
// :send <text>      -> append text to TX queue (uppercase + spaces)
// :txinv            -> toggle TX output inversion (for active-low wiring)
String cmd;
void handleCommand(){
  if (cmd.startsWith(":unit")){
    long v = cmd.substring(5).toInt();
    if (v >= 200 && v <= 1500){ UNIT_MS = (unsigned long)v; Serial.print("[OK] UNIT_MS="); Serial.println(UNIT_MS); }
    else Serial.println("[ERR] :unit 200..1500");
  } else if (cmd.startsWith(":th")){
    int sp = cmd.indexOf(' ');
    int sp2 = (sp>0)? cmd.indexOf(' ', sp+1) : -1;
    if (sp>0 && sp2>sp){
      int onV  = cmd.substring(sp+1, sp2).toInt();
      int offV = cmd.substring(sp2+1).toInt();
      if (onV>=1 && onV<=50 && offV>=0 && offV<onV){
        TH_ON = onV; TH_OFF = offV;
        Serial.print("[OK] TH_ON="); Serial.print(TH_ON);
        Serial.print(" TH_OFF="); Serial.println(TH_OFF);
      } else Serial.println("[ERR] :th <on 1..50> <off <on>");
    } else Serial.println("[ERR] usage :th <on> <off>");
  } else if (cmd.startsWith(":send")){
    String payload = cmd.substring(6);
    // sanitize: keep A-Z, 0-9, space; uppercase
    String cleaned;
    cleaned.reserve(payload.length());
    for (size_t i=0;i<payload.length();++i){
      char c = payload[i];
      if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
      if ((c>='A' && c<='Z') || (c>='0' && c<='9') || c==' '){
        cleaned += c;
      } // else drop
    }
    if (cleaned.length()){
      txQueue += cleaned;
      Serial.print("[TX] queued: '"); Serial.print(cleaned); Serial.println("'");
    } else {
      Serial.println("[TX] nothing to queue (empty after sanitize)");
    }
  } else if (cmd.startsWith(":txinv")){
    TX_INVERT = !TX_INVERT;
    Serial.print("[TX] invert="); Serial.println(TX_INVERT ? "ON" : "OFF");
  } else {
    Serial.println("[?] commands: :unit <ms>, :th <on> <off>, :send <text>, :txinv");
  }
  cmd = "";
}
void pollSerial(){
  while (Serial.available()){
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n'){ if (cmd.length()) handleCommand(); }
    else if (cmd.length()<200) cmd += c;
  }
}

