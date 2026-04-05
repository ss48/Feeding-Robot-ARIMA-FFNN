/*
 * Teensy Sensor Firmware — Feeding Robot
 *
 * Reads HX711 force sensor and HC-SR04 ultrasonic sensor,
 * sends JSON over serial to the Raspberry Pi at 115200 baud.
 *
 * Output format (one line per reading, ~20 Hz):
 *   {"force":123.45,"dist_cm":15.2}
 *
 * Wiring:
 *   HX711:
 *     DOUT -> pin 2
 *     SCK  -> pin 3
 *   HC-SR04:
 *     TRIG -> pin 4
 *     ECHO -> pin 5
 */

// ==================== HX711 PINS ====================
#define HX711_DOUT  2
#define HX711_SCK   3

// ==================== HC-SR04 PINS ====================
#define SONAR_TRIG  4
#define SONAR_ECHO  5

// ==================== CONFIG ====================
#define SERIAL_BAUD     115200
#define LOOP_PERIOD_MS  50      // 20 Hz
#define HX711_GAIN      128     // Channel A, gain 128
#define FORCE_SCALE     420.0   // Raw-to-grams calibration factor (ADJUST THIS)
#define FORCE_OFFSET    0.0     // Tare offset (set via 't' command)
#define SONAR_TIMEOUT_US 25000  // ~4.25m max range

// ==================== GLOBALS ====================
long hx711_offset = 0;
float force_scale = FORCE_SCALE;
unsigned long last_loop = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  // HX711
  pinMode(HX711_DOUT, INPUT);
  pinMode(HX711_SCK, OUTPUT);
  digitalWrite(HX711_SCK, LOW);

  // HC-SR04
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);
  digitalWrite(SONAR_TRIG, LOW);

  delay(500);  // Let HX711 stabilise

  // Auto-tare on startup (average 10 readings)
  long sum = 0;
  int count = 0;
  for (int i = 0; i < 10; i++) {
    long val = hx711_read();
    if (val != 0) {
      sum += val;
      count++;
    }
    delay(50);
  }
  if (count > 0) {
    hx711_offset = sum / count;
  }

  Serial.println("{\"status\":\"ready\"}");
}

void loop() {
  unsigned long now = millis();
  if (now - last_loop < LOOP_PERIOD_MS) return;
  last_loop = now;

  // Check for serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') {
      // Tare command
      long sum = 0;
      for (int i = 0; i < 10; i++) {
        sum += hx711_read();
        delay(20);
      }
      hx711_offset = sum / 10;
      Serial.println("{\"tare\":\"done\"}");
      return;
    }
  }

  // Read force sensor
  long raw = hx711_read();
  float force_grams = (raw - hx711_offset) / force_scale;
  float force_n = force_grams * 0.00981;  // Convert grams to Newtons

  // Read ultrasonic
  float dist_cm = sonar_read_cm();

  // Send JSON
  Serial.print("{\"force\":");
  Serial.print(force_n, 4);
  Serial.print(",\"dist_cm\":");
  if (dist_cm < 0) {
    Serial.print("null");
  } else {
    Serial.print(dist_cm, 2);
  }
  Serial.println("}");
}

// ==================== HX711 ====================
long hx711_read() {
  // Wait for data ready (DOUT goes LOW)
  unsigned long start = millis();
  while (digitalRead(HX711_DOUT) == HIGH) {
    if (millis() - start > 200) return 0;  // Timeout
  }

  long value = 0;
  // Read 24 bits
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX711_DOUT);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }

  // Set gain for next reading (1 extra pulse = gain 128, channel A)
  for (int i = 0; i < 1; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(1);
  }

  // Sign extend 24-bit to 32-bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }

  return value;
}

// ==================== HC-SR04 ====================
float sonar_read_cm() {
  // Send trigger pulse
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  // Measure echo duration
  long duration = pulseIn(SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);

  if (duration == 0) return -1.0;  // No echo / out of range

  // Speed of sound = 343 m/s = 0.0343 cm/us
  // Distance = duration * 0.0343 / 2
  float cm = duration * 0.01715;

  // Valid range: 2-400 cm
  if (cm < 2.0 || cm > 400.0) return -1.0;

  return cm;
}
