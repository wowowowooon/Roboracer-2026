/*
  FS-iA10B PPM -> Arduino Nano USB serial monitor

  Wiring:
    Receiver PPM signal -> Nano D2
    Receiver GND        -> Nano GND

  Jetson reads this through the Nano USB serial port at 115200 baud.
  Output format is accepted by rc_receiver_control_node.py:
    1500,1500,1500,1500,1000,1000
*/

#define PPM_PIN 2
#define CHANNELS 6
#define SYNC_GAP 3000

const unsigned long PRINT_PERIOD_MS = 20;

volatile uint16_t channels[CHANNELS] = {1500, 1500, 1500, 1500, 1000, 1000};
volatile uint8_t chIndex = 0;
volatile uint32_t lastTime = 0;
unsigned long lastPrintMs = 0;

void ppmISR() {
  uint32_t now = micros();
  uint32_t width = now - lastTime;
  lastTime = now;

  if (width > SYNC_GAP) {
    chIndex = 0;
  } else if (chIndex < CHANNELS) {
    channels[chIndex] = (uint16_t)width;
    chIndex++;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
  Serial.println("FS-iA10B PPM CH monitor start");
}

void loop() {
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs < PRINT_PERIOD_MS) {
    return;
  }
  lastPrintMs = nowMs;

  uint16_t snapshot[CHANNELS];

  noInterrupts();
  for (uint8_t i = 0; i < CHANNELS; i++) {
    snapshot[i] = channels[i];
  }
  interrupts();

  for (uint8_t i = 0; i < CHANNELS; i++) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(snapshot[i]);
  }
  Serial.println();
}
