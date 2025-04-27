#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiServer.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <stdlib.h>
#include <numeric>  // for accumulate

//── DEBUG & Logging ──────────────────────────────────────────────────────────
#define DEBUG true
void logInfo(const String &msg) {
  Serial.print("[INFO] ");
  Serial.println(msg);
}
void logDebug(const String &msg) {
  if (DEBUG) {
    Serial.print("[DEBUG] ");
    Serial.println(msg);
  }
}
void logWarn(const String &msg) {
  Serial.print("[WARN] ");
  Serial.println(msg);
}

//── Configurable pins ─────────────────────────────────────────────────────────
#define PIN_ECHO 9
#define PIN_TRIGGER 10
#define PIN_WATER 8
#define LED_PIN 7    // FastLED data pin
#define NUM_LEDS 7  // configurable number of LEDs
CRGB leds[NUM_LEDS];

//── Constants ─────────────────────────────────────────────────────────────────
#define MAX_QUEUE 10
#define WINDOW_SIZE 10
#define EEPROM_CONFIG_ADDR 0

//── Config defaults ─────────────────────────────────────────────────────────
#define DEFAULT_THRESHOLD_TRIGGER 5.0f
#define DEFAULT_THRESHOLD_SAME 0.5f
#define DEFAULT_THRESHOLD_TIMEOUT 120
#define DEFAULT_FLOW_RATE 10.0f

//── Event types ───────────────────────────────────────────────────────────────
enum EventType { EVENT_MEASUREMENT,
                 EVENT_PRESENCE_TIMEOUT };
struct Event {
  EventType type;
  float value;
};

//── Activation & Session ──────────────────────────────────────────────────────
struct Activation {
  unsigned long startTime, endTime;
};
struct Session {
  unsigned long startTime;
  Activation *activations;
  int activationCount;
};

//── Application config ────────────────────────────────────────────────────────
struct Config {
  float thresholdTrigger, thresholdSame;
  int thresholdTimeout;
  float flowRate;
};

//── Runtime state ─────────────────────────────────────────────────────────────
struct State {
  Config config;
  bool presence;
  unsigned long lastTrigger;
  float distanceIdle;
  float measurementsWindow[WINDOW_SIZE];
  Session *sessions;
  int sessionCount;
} state;

//── Event queue ───────────────────────────────────────────────────────────────
static Event eventQueue[MAX_QUEUE];
static int queueStart = 0, queueEnd = 0;

//── Networking ────────────────────────────────────────────────────────────────
const char *ssid = "SSID";
const char *password = "Your password";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);
WiFiServer server(80);
int status = WL_IDLE_STATUS;

//── Timing & LED globals ─────────────────────────────────────────────────────
unsigned long appStartTime = 0;
bool redPulse = false;
unsigned long redPulseStart = 0;
const unsigned long redPulseDuration = 200;

//── Prototypes ───────────────────────────────────────────────────────────────
void dispatchEvent(const Event &e);
bool hasEvent();
Event dequeueEvent();
void processEvents();
void reducer(const Event &e);
unsigned long getCurrentTime();
float measureDistance();
void initConfig();
void setupState();
void loadConfigFromEEPROM();
void saveConfigToEEPROM();

//── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  logInfo("Starting Pet Fountain Controller");

  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_WATER, OUTPUT);
  digitalWrite(PIN_WATER, LOW);

  // FastLED init
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Load or reset config
  loadConfigFromEEPROM();
  if (state.config.thresholdTrigger <= 0 || state.config.thresholdSame <= 0 || state.config.thresholdTimeout <= 0 || state.config.flowRate <= 0) {
    initConfig();
    saveConfigToEEPROM();
    logInfo("Config reset to defaults");
  } else {
    logInfo("Loaded config from EEPROM");
  }

  setupState();

  // Wi-Fi
  while (WiFi.status() == WL_NO_MODULE) {
    logWarn("Wi-Fi module missing!");
    delay(1000);
  }
  while (status != WL_CONNECTED) {
    logInfo("Connecting to Wi-Fi...");
    status = WiFi.begin(ssid, password);
    delay(2000);
  }
  logInfo("Wi-Fi connected");

  // NTP
  timeClient.begin();
  while (!timeClient.update()) {
    delay(500);
  }
  appStartTime = timeClient.getEpochTime();
  logInfo("NTP time = " + String(appStartTime));

  // First session
  int sc = state.sessionCount;
  state.sessions = (Session *)realloc(state.sessions, (sc + 1) * sizeof(Session));
  state.sessionCount = sc + 1;
  state.sessions[sc] = { appStartTime, nullptr, 0 };
  logInfo("Started session #" + String(sc + 1));

  server.begin();
  logInfo("HTTP server started");
}

//── Main Loop ───────────────────────────────────────────────────────────────
unsigned long lastMeasurement = 0;
unsigned long sampleBuf[WINDOW_SIZE];
int sampleCount = 0;
unsigned long lastTimeoutCheck = 0;

void loop() {
  unsigned long nowMs = millis();
  unsigned long now = getCurrentTime();

  // HC-SR04 measurement every 100 ms
  if (nowMs - lastMeasurement >= 100) {
    lastMeasurement = nowMs;
    sampleBuf[sampleCount++] = measureDistance();
    if (sampleCount >= WINDOW_SIZE) {
      float sum = 0;
      float minv = sampleBuf[0], maxv = sampleBuf[0];
      bool anyNonZero = false;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        float v = sampleBuf[i];
        sum += v;
        minv = min(minv, v);
        maxv = max(maxv, v);
        if (v != 0) anyNonZero = true;
      }
      float avg = (sum - minv - maxv) / (WINDOW_SIZE - 2);
      logDebug("Measurement avg: " + String(avg, 2));
      dispatchEvent({ EVENT_MEASUREMENT, avg });
      sampleCount = 0;
    }
  }

  // Presence timeout every 60s
  if (nowMs - lastTimeoutCheck >= 60000) {
    lastTimeoutCheck = nowMs;
    if (state.presence && (now - state.lastTrigger) > state.config.thresholdTimeout) {
      dispatchEvent({ EVENT_PRESENCE_TIMEOUT, 0 });
    }
  }

  processEvents();

  // LED behavior
  if (state.presence) {
    // blue/turquoise/aqua pulsating
    uint8_t b = beatsin8(20, 128, 255);
    uint8_t h = beatsin8(10, 96, 160);
    fill_solid(leds, NUM_LEDS, CHSV(h, 255, b));
    // red pulse override
    if (redPulse && millis() - redPulseStart < redPulseDuration) {
      fill_solid(leds, NUM_LEDS, CRGB::Red);
    } else {
      redPulse = false;
    }
  } else {
    // fade off when presence lost
    fadeToBlackBy(leds, NUM_LEDS, 20);
  }
  FastLED.show();

  // HTTP handling + stats (unchanged from previous implementation)
  WiFiClient client = server.available();
  if (client) {
    // HTTP handling & stats
    WiFiClient client = server.available();
    if (client) {
      while (!client.available()) delay(1);
      String req = client.readStringUntil('\r');
      client.readStringUntil('\n');
      logDebug("HTTP request: " + req);

      // Reset Sessions
      if (req.startsWith("GET /resetSessions")) {
        for (int i = 0; i < state.sessionCount; i++) {
          free(state.sessions[i].activations);
        }
        free(state.sessions);
        state.sessions = nullptr;
        state.sessionCount = 0;

        unsigned long now2 = getCurrentTime();
        state.sessions = (Session *)malloc(sizeof(Session));
        state.sessionCount = 1;
        state.sessions[0].startTime = now2;
        state.sessions[0].activations = nullptr;
        state.sessions[0].activationCount = 0;
        logInfo("All sessions reset, new session at " + String(now2));
      }

      // Config setter
      if (req.startsWith("GET /setConfig")) {
        int qpos = req.indexOf('?');
        int end = req.indexOf(' ', qpos);
        String query = req.substring(qpos + 1, end);
        int idx = 0;
        while (idx < query.length()) {
          int amp = query.indexOf('&', idx);
          if (amp < 0) amp = query.length();
          String pair = query.substring(idx, amp);
          int eq = pair.indexOf('=');
          if (eq > 0) {
            String key = pair.substring(0, eq);
            String val = pair.substring(eq + 1);
            if (key == "thresholdTrigger") state.config.thresholdTrigger = val.toFloat();
            else if (key == "thresholdSame") state.config.thresholdSame = val.toFloat();
            else if (key == "thresholdTimeout") state.config.thresholdTimeout = val.toInt();
            else if (key == "flowRate") state.config.flowRate = val.toFloat();
          }
          idx = amp + 1;
        }
        saveConfigToEEPROM();
        logInfo("Config updated via HTTP");
      }

      // Index page
      if (req.startsWith("GET / ")) {
        unsigned long curr = getCurrentTime();
        String html = "<!DOCTYPE html><html><head><title>Infinity Bowl</title></head><body>";

        // Header & Reset button
        html += "<h1>Infinity Bowl Controller</h1>";
        html += "<form action=\"/resetSessions\" method=\"GET\">";
        html += "<button type=\"submit\">Reset Sessions</button></form>";

        // Constants
        html += "<h2>Constants</h2><ul>";
        html += "<li>PIN_ECHO: " + String(PIN_ECHO) + "</li>";
        html += "<li>PIN_TRIGGER: " + String(PIN_TRIGGER) + "</li>";
        html += "<li>PIN_WATER: " + String(PIN_WATER) + "</li>";
        html += "<li>MAX_QUEUE: " + String(MAX_QUEUE) + "</li>";
        html += "<li>WINDOW_SIZE: " + String(WINDOW_SIZE) + "</li></ul>";

        // State
        html += "<h2>State</h2><ul>";
        html += "<li>presence: " + String(state.presence ? "true" : "false") + "</li>";
        html += "<li>lastTrigger: " + String(state.lastTrigger) + "</li>";
        html += "<li>distanceIdle: " + String(state.distanceIdle, 2) + "</li>";
        html += "<li>measureWin: [";
        for (int i = 0; i < WINDOW_SIZE; i++) {
          html += String(state.measurementsWindow[i], 2);
          if (i < WINDOW_SIZE - 1) html += ",";
        }
        html += "]</li></ul>";

        // Config form
        html += "<h2>Config</h2><form action=\"/setConfig\" method=\"GET\">";
        html += "thresholdTrigger: <input name=\"thresholdTrigger\" value=\"" + String(state.config.thresholdTrigger, 2) + "\">";
        html += " thresholdSame: <input name=\"thresholdSame\" value=\"" + String(state.config.thresholdSame, 2) + "\">";
        html += " timeout: <input name=\"thresholdTimeout\" value=\"" + String(state.config.thresholdTimeout) + "\">";
        html += " flowRate: <input name=\"flowRate\" value=\"" + String(state.config.flowRate, 2) + "\">";
        html += " <button type=\"submit\">Set</button></form>";

        // Statistics (1h,6h,12h,24h,1w,since reboot)
        const int N = 6;
        unsigned long windows[N] = {
          curr - 3600UL,
          curr - 6 * 3600UL,
          curr - 12 * 3600UL,
          curr - 24 * 3600UL,
          curr - 7 * 24 * 3600UL,
          appStartTime
        };
        const char *names[N] = { "1h", "6h", "12h", "24h", "1w", "since reboot" };

        unsigned long counts[N] = { 0 }, durations[N] = { 0 };
        // Aggregate activations
        for (int i = 0; i < state.sessionCount; i++) {
          Session &s = state.sessions[i];
          for (int j = 0; j < s.activationCount; j++) {
            Activation &a = s.activations[j];
            unsigned long dur = a.endTime - a.startTime;
            for (int w = 0; w < N; w++) {
              if (a.startTime >= windows[w]) {
                counts[w]++;
                durations[w] += dur;
              }
            }
          }
        }

        // Build stats table
        html += "<h2>Statistics</h2><table border=1><tr><th>Metric</th>";
        for (int w = 0; w < N; w++) html += "<th>" + String(names[w]) + "</th>";
        html += "</tr>";

        // # triggers
        html += "<tr><td># triggers</td>";
        for (int w = 0; w < N; w++) html += "<td>" + String(counts[w]) + "</td>";
        html += "</tr>";

        // Avg presence
        html += "<tr><td>Avg presence (s)</td>";
        for (int w = 0; w < N; w++) {
          float avg = counts[w] ? (float)durations[w] / counts[w] : 0;
          html += "<td>" + String(avg, 2) + "</td>";
        }
        html += "</tr>";

        // Hydration time
        html += "<tr><td>Hydration time (s)</td>";
        for (int w = 0; w < N; w++) html += "<td>" + String(durations[w]) + "</td>";
        html += "</tr>";

        // Water dispensed
        html += "<tr><td>Water dispensed (mL)</td>";
        for (int w = 0; w < N; w++) {
          float ml = durations[w] * state.config.flowRate;
          html += "<td>" + String(ml, 2) + "</td>";
        }
        html += "</tr></table>";

        // Sessions & Activations
        html += "<h2>Sessions & Activations</h2>";
        for (int i = 0; i < state.sessionCount; i++) {
          Session &s = state.sessions[i];
          html += "<h3>Session " + String(i + 1) + ": start=" + String(s.startTime) + "</h3><ul>";
          for (int j = 0; j < s.activationCount; j++) {
            Activation &a = s.activations[j];
            unsigned long dur = a.endTime - a.startTime;
            html += "<li>Act " + String(j + 1)
                    + ": start=" + String(a.startTime)
                    + ", end=" + String(a.endTime)
                    + ", dur=" + String(dur) + "s"
                    + ", vol=" + String(dur * state.config.flowRate, 2) + "mL</li>";
          }
          html += "</ul>";
        }

        html += "</body></html>";

                client.print("HTTP/1.1 200 OK\r\n");
                client.print("Content-Type: text/html\r\n");
                client.print("Connection: close\r\n");
                client.print(html);
                logInfo("HTTP response sent");

                client.stop();
                logInfo("Client disconnected");
      }
    }
  }
}

//── Reducer ──────────────────────────────────────────────────────────────────
void reducer(const Event &e) {
  unsigned long now = getCurrentTime();

  if (e.type == EVENT_MEASUREMENT) {
    // shift window
    for (int i = WINDOW_SIZE - 1; i > 0; i--) {
      state.measurementsWindow[i] = state.measurementsWindow[i - 1];
    }
    state.measurementsWindow[0] = e.value;

    // calibrate only if non-zero
    bool stable = true, nz = false;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float v = state.measurementsWindow[i];
      if (v != 0) nz = true;
      if (fabs(v - e.value) > state.config.thresholdSame) stable = false;
    }
    if (state.distanceIdle < 0 && stable && nz) {
      state.distanceIdle = std::accumulate(state.measurementsWindow,
                                           state.measurementsWindow + WINDOW_SIZE, 0.0f)
                           / WINDOW_SIZE;
      logInfo("Calibrated idle: " + String(state.distanceIdle, 2));
      return;
    }

    // red pulse on stable distance
    if (state.presence && fabs(e.value - state.distanceIdle) <= state.config.thresholdSame) {
      redPulse = true;
      redPulseStart = millis();
    }

    // detect presence start
    if (!state.presence && e.value < state.distanceIdle - state.config.thresholdTrigger) {
      state.presence = true;
      state.lastTrigger = now;
      digitalWrite(PIN_WATER, HIGH);
    }

    // detect presence end
    if (state.presence) {
      bool gone = true;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        float v = state.measurementsWindow[i];
        if (v < state.distanceIdle - state.config.thresholdTrigger || v > state.distanceIdle + state.config.thresholdTrigger) {
          gone = false;
        }
      }
      if (gone) {
        state.presence = false;
        digitalWrite(PIN_WATER, LOW);
      }
    }
  } else {
    // timeout end
    state.presence = false;
    digitalWrite(PIN_WATER, LOW);
  }
}

//── Helper functions ─────────────────────────────────────────────────────────

unsigned long getCurrentTime() {
  timeClient.update();
  return timeClient.getEpochTime();
}

float measureDistance() {
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 30000);
  return (d * 0.034) / 2;
}

void dispatchEvent(const Event &e) {
  int nxt = (queueEnd + 1) % MAX_QUEUE;
  if (nxt != queueStart) {
    eventQueue[queueEnd] = e;
    queueEnd = nxt;
  }
}

bool hasEvent() {
  return queueStart != queueEnd;
}

Event dequeueEvent() {
  Event e = eventQueue[queueStart];
  queueStart = (queueStart + 1) % MAX_QUEUE;
  return e;
}

void processEvents() {
  while (hasEvent()) {
    Event e = dequeueEvent();
    bool prev = state.presence;
    reducer(e);
    if (state.presence != prev) {
      logInfo(String("Water ") + (state.presence ? "ON" : "OFF"));
    }
  }
}

void initConfig() {
  state.config = { DEFAULT_THRESHOLD_TRIGGER,
                   DEFAULT_THRESHOLD_SAME,
                   DEFAULT_THRESHOLD_TIMEOUT,
                   DEFAULT_FLOW_RATE };
}

void setupState() {
  initConfig();
  state.presence = false;
  state.lastTrigger = getCurrentTime();
  state.distanceIdle = -1;
  state.sessionCount = 0;
  state.sessions = nullptr;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    state.measurementsWindow[i] = 0;
  }
}

void loadConfigFromEEPROM() {
  EEPROM.get(EEPROM_CONFIG_ADDR, state.config);
}

void saveConfigToEEPROM() {
  EEPROM.put(EEPROM_CONFIG_ADDR, state.config);
}
