// ============================================================
//  GTag_Tester - one board, two instruments
//
//  1. USB PORT TESTER - probe in the port under test, measures VBUS, D+ and
//     D- voltages, line resistance, shorts, charger signatures. Powered from
//     a charger (no data on the USB-C connector).
//
//  2. PC INIT TESTER - the USB-C goes into the machine under test. The board
//     enumerates as a USB HID keyboard and, without pressing anything,
//     watches what the host does by itself: enumeration, keyboard LED state,
//     protocol handover, re-enumeration.
//
//  Mode selection:
//    - at power up the board waits a few seconds. If the host enumerated us
//      there is data on the connector, so it starts the init tester,
//      otherwise it assumes a charger and starts the port tester.
//    - BOOTSEL short press: flip the picture (port tester) / restart the
//      measurement (init tester).
//    - BOOTSEL long press: switch mode manually.
//
//  IMPORTANT: Tools -> USB Stack must be set to "Adafruit TinyUSB".
//
//  Board: RP2040-Zero, core: Raspberry Pi Pico/RP2040 (Earle Philhower)
//    LCD    -> GP8..GP11, plus the tag board for DisplayCLK
//    Buzzer -> GP7, optional
//    Probe  -> GP26 VBUS (100k/100k), GP27 D-, GP28 D+, GP29 10k reference
// ============================================================

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "gtag_lcd.h"

// ---------------- config ----------------
#define PIN_BUZZER       7
#define BUZZER_ACTIVE    0            // 0 = passive piezo (tone), 1 = active buzzer
static constexpr uint16_t BEEP_HZ = 2200;

#define PIN_VBUS    26   // ADC0, through 100k/100k divider
#define PIN_DPLUS   28   // ADC2
#define PIN_DMINUS  27   // ADC1
#define REF_PIN     29   // ADC3, 10k to GND
#define REF_OHM     10000
#define PULLUP_NOMINAL 50000.0f

static constexpr float    VCC      = 3.3f;
static constexpr bool     INK      = false;   // black ink on white paper
static constexpr float    SHORT_V  = 0.5f;    // cross-line coupling threshold
static constexpr uint32_t DRAW_MS  = 400;
static constexpr uint32_t LOG_MS   = 1500;
static constexpr uint32_t BOOT_MODE_MS = 6000;  // decide the mode by then
static constexpr uint32_t PRESS_LONG_MS = 700;  // longer than this = mode switch

static float g_pullup = PULLUP_NOMINAL;   // refined by calibration
static float g_vcc    = VCC;

// ---------------- measurement types ----------------
// Must sit before the first function: the Arduino builder inserts the
// generated prototypes just ahead of the first function body.
enum LineState : uint8_t { LN_UNKNOWN, LN_OPEN, LN_HOST_PD, LN_DEV_UP, LN_SHORT_GND };

struct Reading {
  float     vbus;
  float     vP, vN;
  float     rP, rN;
  float     rShort;
  LineState sP, sN;
  bool      shortPN;
  uint16_t  actP, actN;
};

enum Mode : uint8_t { MODE_PORT, MODE_INIT };

// ---------------- HID ----------------
static uint8_t const desc_hid_report[] = { TUD_HID_REPORT_DESC_KEYBOARD() };
Adafruit_USBD_HID usb_hid;

static void hid_report_callback(uint8_t report_id, hid_report_type_t report_type,
                                uint8_t const *buffer, uint16_t bufsize);

static volatile uint32_t led_reports = 0;
static volatile uint8_t  led_state   = 0;

// The host's keyboard LED state arrives here. No key press is involved: the
// OS and the Linux kernel send this on their own when the driver loads.
static void hid_report_callback(uint8_t report_id, hid_report_type_t report_type,
                                uint8_t const *buffer, uint16_t bufsize) {
  (void)report_id;
  if (report_type != HID_REPORT_TYPE_OUTPUT || bufsize < 1) return;
  led_state = buffer[0];
  ++led_reports;
}

// ---------------- buzzer ----------------
static void buzzer_set(bool on) {
#if BUZZER_ACTIVE
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
#else
  if (on) tone(PIN_BUZZER, BEEP_HZ);
  else    noTone(PIN_BUZZER);
#endif
}

static void buzzer_beep(int times, uint16_t ms) {
  for (int i = 0; i < times; ++i) {
    buzzer_set(true);  delay(ms);
    buzzer_set(false); delay(ms);
  }
}

static void beep_joy() {
#if BUZZER_ACTIVE
  buzzer_set(true); delay(300); buzzer_set(false);
#else
  tone(PIN_BUZZER, 1600); delay(120); noTone(PIN_BUZZER); delay(60);
  tone(PIN_BUZZER, 2400); delay(220); noTone(PIN_BUZZER);
#endif
}

static void buzzer_alarm(bool alarm) {
  static uint32_t last = 0;
  static bool phase = false;
  if (!alarm) {
    if (phase) { buzzer_set(false); phase = false; }
    return;
  }
  const uint32_t now = millis();
  if ((uint32_t)(now - last) >= 120) {
    last = now;
    phase = !phase;
    buzzer_set(phase);
  }
}

// ============================================================
//  Port tester: measurement
// ============================================================
static float adc_volts(uint8_t pin) {
  uint32_t acc = 0;
  for (int i = 0; i < 8; ++i) acc += (uint32_t)analogRead(pin);
  return (acc / 8.0f) * VCC / 4096.0f;
}

static uint16_t count_edges(uint8_t pin, uint32_t us) {
  gpio_set_function(pin, GPIO_FUNC_SIO);
  gpio_set_dir(pin, GPIO_IN);
  gpio_set_input_enabled(pin, true);

  const uint32_t t0 = micros();
  uint16_t edges = 0;
  int last = gpio_get(pin) ? 1 : 0;
  while ((uint32_t)(micros() - t0) < us) {
    const int v = gpio_get(pin) ? 1 : 0;
    if (v != last) { ++edges; last = v; }
  }

  gpio_set_input_enabled(pin, false);
  gpio_set_function(pin, GPIO_FUNC_NULL);
  return edges;
}

static float r_to_gnd(float v) {
  if (v >= VCC - 0.15f) return -1.0f;
  if (v <= 0.02f)       return 0.0f;
  return g_pullup * v / (g_vcc - v);
}

static float r_between(float v) {
  if (v < 0.1f) return -1.0f;
  const float r = g_pullup * (g_vcc / v - 2.0f);
  return (r < 0.0f) ? 0.0f : r;
}

// The pull-up test decides the state; a line that collapses under a pull-down
// was never really held high.
static LineState classify(float base_v, float pullup_v, float pulldown_v) {
  if (base_v >= 2.5f) {
    return (pulldown_v >= 2.0f) ? LN_DEV_UP : LN_OPEN;
  }
  if (pullup_v >= VCC - 0.3f) return LN_OPEN;
  if (pullup_v >= 0.35f)      return LN_HOST_PD;
  return LN_SHORT_GND;
}

static Reading measure_port() {
  Reading r{};

  gpio_set_pulls(PIN_VBUS, false, false);
  r.vbus = adc_volts(PIN_VBUS) * 2.0f;           // undo the divider

  gpio_set_pulls(PIN_DPLUS, false, false);
  gpio_set_pulls(PIN_DMINUS, false, false);
  delayMicroseconds(300);
  r.vP = adc_volts(PIN_DPLUS);
  r.vN = adc_volts(PIN_DMINUS);

  gpio_set_pulls(PIN_DPLUS, true, false);        // raise D+, hold D- down
  gpio_set_pulls(PIN_DMINUS, false, true);
  delayMicroseconds(300);
  const float uP = adc_volts(PIN_DPLUS);
  const float nUp = adc_volts(PIN_DMINUS);

  gpio_set_pulls(PIN_DPLUS, false, true);        // raise D-, hold D+ down
  gpio_set_pulls(PIN_DMINUS, true, false);
  delayMicroseconds(300);
  const float uN = adc_volts(PIN_DMINUS);
  const float pUp = adc_volts(PIN_DPLUS);

  gpio_set_pulls(PIN_DPLUS, false, false);
  gpio_set_pulls(PIN_DMINUS, false, false);

  // Smoothing keeps the display from flickering, which matters most when the
  // probe is unplugged and the lines are floating.
  static bool  have = false;
  static float sP = 0, sN = 0, sUp = 0, sUn = 0, sBus = 0, sPdn = 0, sNdn = 0;
  if (!have) {
    sP = r.vP; sN = r.vN; sUp = uP; sUn = uN; sBus = r.vbus;
    sPdn = pUp; sNdn = nUp;
    have = true;
  } else {
    const float a = 0.7f;
    sP   = sP   * a + r.vP  * (1.0f - a);
    sN   = sN   * a + r.vN  * (1.0f - a);
    sUp  = sUp  * a + uP    * (1.0f - a);
    sUn  = sUn  * a + uN    * (1.0f - a);
    sBus = sBus * a + r.vbus * (1.0f - a);
    sPdn = sPdn * a + pUp   * (1.0f - a);
    sNdn = sNdn * a + nUp   * (1.0f - a);
  }
  r.vP = sP; r.vN = sN; r.vbus = sBus;
  r.sP = classify(sP, sUp, sPdn);
  r.sN = classify(sN, sUn, sNdn);
  r.rP = r_to_gnd(sUp);
  r.rN = r_to_gnd(sUn);

  const float coupling = (nUp > pUp) ? nUp : pUp;
  r.shortPN = coupling > SHORT_V;
  r.rShort = r.shortPN ? r_between(coupling) : -1.0f;

  r.actP = count_edges(PIN_DPLUS, 5000);
  r.actN = count_edges(PIN_DMINUS, 5000);
  return r;
}

// Two known resistors let us solve for the pull-up value and the effective
// pull-up rail at once: the 10k reference on GP29 and the 100k lower divider
// leg on GP26.
static void calibrate() {
#if REF_OHM > 0
  analogRead(REF_PIN);
  analogRead(PIN_VBUS);

  gpio_set_pulls(REF_PIN, true, false);
  delayMicroseconds(500);
  const float v10 = adc_volts(REF_PIN);
  gpio_set_pulls(REF_PIN, false, false);

  gpio_set_pulls(PIN_VBUS, false, false);
  delayMicroseconds(500);
  const float v0 = adc_volts(PIN_VBUS);

  gpio_set_pulls(PIN_VBUS, true, false);
  delayMicroseconds(500);
  const float v1 = adc_volts(PIN_VBUS);
  gpio_set_pulls(PIN_VBUS, false, false);

  const float vbus = v0 * 2.0f;

  if (v10 > 0.05f) {
    float vcc = g_vcc;
    if (v0 > 0.3f) {
      const float den = 10.0f * v10 - 2.0f * v1 + vbus;   // live VBUS branch
      if (den > 0.1f) vcc = v10 * (8.0f * v1 + vbus) / den;
    } else if (v0 < 0.03f) {
      const float den = 5.0f * v10 - v1;                  // VBUS grounded
      if (den > 0.1f) vcc = 4.0f * v1 * v10 / den;
    } else {
      const float den = 10.0f * v10 - v1;                 // probe unplugged
      if (den > 0.1f) vcc = 9.0f * v1 * v10 / den;
    }
    if (vcc > 2.5f && vcc < 4.6f) {
      const float rpu = (vcc - v10) * (float)REF_OHM / v10;
      if (rpu > 8000.0f && rpu < 200000.0f) g_pullup = rpu;
      g_vcc = vcc;
    }
  }

  Serial.printf("cal: Rpu=%.0f ohm  Vcc=%.3f V  (v10=%.3f v1=%.3f vbus=%.2f)\n",
                g_pullup, g_vcc, v10, v1, vbus);
#else
  (void)REF_PIN;
  Serial.printf("pull-up assumed: %.0f ohm\n", g_pullup);
#endif
}

static const char *state_name(LineState s) {
  switch (s) {
    case LN_OPEN:      return "OPEN";
    case LN_HOST_PD:   return "HOST PULL-DOWN";
    case LN_DEV_UP:    return "DEVICE PULL-UP";
    case LN_SHORT_GND: return "SHORT TO GND";
    default:           return "UNKNOWN";
  }
}

static void fmt_ohm(char *buf, size_t n, float r) {
  if (r < 0.0f || r >= 1000000.0f) snprintf(buf, n, "OPEN");
  else if (r < 1000.0f)            snprintf(buf, n, "%.0f", r);
  else                             snprintf(buf, n, "%.0fK", r / 1000.0f);
}

static void verdict_text(const Reading &r, char *l1, size_t n1, char *l2, size_t n2) {
  l1[0] = l2[0] = 0;
  if (r.vbus < 1.0f) {
    snprintf(l1, n1, "НЕТ ПИТАНИЯ НА VBUS");
    snprintf(l2, n2, "ПРОВЕРЬ КАБЕЛЬ И ПОРТ");
  } else if (r.shortPN) {
    snprintf(l1, n1, "ЗАМЫКАНИЕ D+ И D-");
    snprintf(l2, n2, "ПРОВЕРЬ ПОРТ И КАБЕЛЬ");
  } else if (r.sP == LN_DEV_UP) {
    snprintf(l1, n1, "УСТРОЙСТВО ПОДКЛЮЧЕНО");
    snprintf(l2, n2, "СКОРОСТЬ ВЫСОКАЯ");
  } else if (r.sN == LN_DEV_UP) {
    snprintf(l1, n1, "УСТРОЙСТВО ПОДКЛЮЧЕНО");
    snprintf(l2, n2, "СКОРОСТЬ НИЗКАЯ");
  } else if (r.sP == LN_HOST_PD || r.sN == LN_HOST_PD) {
    snprintf(l1, n1, "ПОРТ ИСПРАВЕН");
    snprintf(l2, n2, "УСТРОЙСТВ НЕ НАЙДЕНО");
  } else if (r.sP == LN_OPEN && r.sN == LN_OPEN) {
    snprintf(l1, n1, "ЛИНИИ ДАННЫХ ОТКРЫТЫ");
    snprintf(l2, n2, "ПОРТ ПУСТ ИЛИ МЕРТВ");
  } else {
    snprintf(l1, n1, "ПРОВЕРЬ ЛИНИИ");
    snprintf(l2, n2, "СМОТРИ СОСТОЯНИЕ");
  }
}

// Both lines held high at once is not a valid USB state for a device: it
// happens when the two lines are tied together, which a charger does on
// purpose and a faulty port does by accident.
static bool both_lines_high(const Reading &r) {
  return r.sP == LN_DEV_UP && r.sN == LN_DEV_UP;
}

static bool looks_like_rail_short(const Reading &r) {
  return both_lines_high(r) && r.vP >= 3.0f && r.vN >= 3.0f;
}

static void print_reading(const Reading &r) {
  char rbP[10], rbN[10], rbS[10], l1[28], l2[28];
  fmt_ohm(rbP, sizeof(rbP), r.rP);
  fmt_ohm(rbN, sizeof(rbN), r.rN);
  fmt_ohm(rbS, sizeof(rbS), r.rShort);
  verdict_text(r, l1, sizeof(l1), l2, sizeof(l2));
  Serial.printf("VBUS %.2f V | D+ %.2f V (%s, R %s) | D- %.2f V (%s, R %s)\n",
                r.vbus, r.vP, state_name(r.sP), rbP, r.vN, state_name(r.sN), rbN);
  Serial.printf("short D+/D-: %s (R %s) | activity D+ %u  D- %u\n",
                r.shortPN ? "YES" : "no", rbS, r.actP, r.actN);
  Serial.printf("verdict: %s / %s\n", l1, l2);
  if (looks_like_rail_short(r))
    Serial.println("WARNING: both lines at the rail - suspect D+/D- shorted to +5V");
  else if (both_lines_high(r))
    Serial.println("short: D+ and D- are tied together (charger signature or faulty port)");
}

// ---------------- port tester screen ----------------
static void render_port(const Reading &r) {
  char rbP[10], rbN[10];
  fmt_ohm(rbP, sizeof(rbP), r.rP);
  fmt_ohm(rbN, sizeof(rbN), r.rN);

  memset(glcd_fb, 0xFF, sizeof(glcd_fb));

  static const char *const label[4] = {"VBUS", "D-", "D+", "GND"};
  char val[4][20];

  if (r.vbus < 0.5f) snprintf(val[0], sizeof(val[0]), "0.00V");
  else               snprintf(val[0], sizeof(val[0]), "%.2fV", r.vbus);

  if (r.sN == LN_OPEN)     snprintf(val[1], sizeof(val[1]), "OPEN");
  else if (r.shortPN)      snprintf(val[1], sizeof(val[1]), "%.2fV SHORT", r.vN);
  else if (r.rN >= 0)      snprintf(val[1], sizeof(val[1]), "%.2fV %s", r.vN, rbN);
  else                     snprintf(val[1], sizeof(val[1]), "%.2fV", r.vN);

  if (r.sP == LN_OPEN)     snprintf(val[2], sizeof(val[2]), "OPEN");
  else if (r.shortPN)      snprintf(val[2], sizeof(val[2]), "%.2fV SHORT", r.vP);
  else if (r.rP >= 0)      snprintf(val[2], sizeof(val[2]), "%.2fV %s", r.vP, rbP);
  else                     snprintf(val[2], sizeof(val[2]), "%.2fV", r.vP);

  snprintf(val[3], sizeof(val[3]), "USB ТЕСТЕР");

  const int scaleL = 3, scaleV = 2;
  const int boxX = 2, boxW = 80;
  const int ry[4] = {2, 34, 66, 98};
  // The rotated view lists GND first, the plain view lists VBUS first.
  static const int orderRot[4]  = {3, 2, 1, 0};
  static const int orderFlat[4] = {0, 1, 2, 3};
  const int *ord = glcd_flip ? orderRot : orderFlat;

  int idxDm = -1, idxDp = -1, idxGnd = -1;
  for (int i = 0; i < 4; ++i) {
    if (ord[i] == 1) idxDm = i;
    if (ord[i] == 2) idxDp = i;
    if (ord[i] == 3) idxGnd = i;
  }

  for (int i = 0; i < 4; ++i) {
    const int k = ord[i], yy = ry[i];
    rect(boxX, yy - 2, boxW, 25);
    const int tw = textChars(label[k]) * 6 * scaleL;
    drawText(boxX + (boxW - tw) / 2, yy, label[k], scaleL);
    drawTextCenter(169, yy + 4, val[k], scaleV);
  }

  if (r.shortPN && idxDm >= 0 && idxDp >= 0) {
    drawVArrow(244, ry[idxDm] + 10, ry[idxDp] + 10);
  } else {
    if (r.sP == LN_SHORT_GND && idxDp >= 0 && idxGnd >= 0)
      drawVArrow(244, ry[idxDp] + 10, ry[idxGnd] + 10);
    if (r.sN == LN_SHORT_GND && idxDm >= 0 && idxGnd >= 0)
      drawVArrow(236, ry[idxDm] + 10, ry[idxGnd] + 10);
  }
}

// ============================================================
//  Init tester: passive boot milestones
// ============================================================
static uint32_t it_t0    = 0;   // first enumeration
static uint32_t it_led   = 0;   // first LED state from the host
static uint32_t it_os    = 0;   // protocol handover or re-enumeration
static bool     it_os_seen = false;
static bool     it_saw_boot = false;
static bool     it_beeped   = false;

static void init_test_reset() {
  it_t0 = it_led = it_os = 0;
  led_reports = 0;
  led_state = 0;
  it_os_seen = false;
  it_saw_boot = false;
  it_beeped = false;
  if (TinyUSBDevice.mounted()) it_t0 = millis();
}

static void fmt_time(char *b, size_t n, uint32_t mark) {
  if (mark == 0 || it_t0 == 0 || mark < it_t0) { snprintf(b, n, "--"); return; }
  snprintf(b, n, "%.1fS", (mark - it_t0) / 1000.0f);
}

static void render_init() {
  memset(glcd_fb, 0xFF, sizeof(glcd_fb));

  drawTextCenter(GLCD_W / 2, 2, "ТЕСТЕР ИНИЦИАЛИЗАЦИИ ПК", 1);

  // The OS is detected by whichever host signal shows up first.
  uint32_t t_load = 0;
  if (it_os && it_led) t_load = (it_os < it_led) ? it_os : it_led;
  else                 t_load = it_os ? it_os : it_led;

  char v1[20], v2[20];
  snprintf(v1, sizeof(v1), "%s", (it_t0 != 0) ? "YES" : "NO");
  fmt_time(v2, sizeof(v2), t_load);

  static const char *const label[2] = {"INIT", "OS BOOT"};
  const char *val[2] = {v1, v2};
  const int boxY[2] = {12, 66};

  for (int i = 0; i < 2; ++i) {
    rect(2, boxY[i], 252, 48);
    drawText(8, boxY[i] + 16, label[i], 2);
    drawTextRight(250, boxY[i] + 16, val[i], 2);
  }
}

// ============================================================
//  Sketch
// ============================================================
static Mode     g_mode   = MODE_PORT;
static bool     g_locked = false;   // the user has chosen a mode by hand

// 0 = nothing, 1 = short press, 2 = long press. BOOTSEL reads the flash CS
// pad, so only sample it on demand.
static uint8_t button_event() {
  static bool     down = false;
  static uint32_t tDown = 0;
  static bool     longSent = false;
  static uint32_t lastSample = 0;

  const uint32_t ms = millis();
  // Reading BOOTSEL floats the flash chip select and disables interrupts for
  // a moment, so sample it at 50 Hz and not in a tight spin.
  if ((uint32_t)(ms - lastSample) < 20) return 0;
  lastSample = ms;

  const bool now = BOOTSEL;
  uint8_t ev = 0;

  if (now && !down) {
    down = true;
    tDown = millis();
    longSent = false;
  } else if (now && down && !longSent &&
             (uint32_t)(millis() - tDown) >= PRESS_LONG_MS) {
    longSent = true;
    ev = 2;
  } else if (!now && down) {
    down = false;
    if (!longSent && (uint32_t)(millis() - tDown) >= 40) ev = 1;   // ignore bounce
  }
  return ev;
}

static void switch_mode() {
  g_mode = (g_mode == MODE_PORT) ? MODE_INIT : MODE_PORT;
  g_locked = true;
  if (g_mode == MODE_INIT) init_test_reset();
  else                     buzzer_beep(1, 80);
  Serial.printf("mode: %s\n", (g_mode == MODE_INIT) ? "init tester" : "port tester");
}

void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  buzzer_set(false);

  Serial.begin(115200);
  Serial.println();
  Serial.println("G-Tag tester: port tester + PC init tester");

  lcd_init();

  analogReadResolution(12);
  analogRead(PIN_VBUS);
  analogRead(PIN_DPLUS);
  analogRead(PIN_DMINUS);
  calibrate();

  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_KEYBOARD);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("G-Tag Tester");
  usb_hid.setReportCallback(NULL, hid_report_callback);
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  memset(glcd_fb, 0xFF, sizeof(glcd_fb));
  lcd_send_frame(glcd_fb);
}

void loop() {
  const uint32_t now = millis();

  // ---- mode decision: charger or computer? ----
  if (!g_locked && (uint32_t)now > BOOT_MODE_MS) {
    g_mode = TinyUSBDevice.mounted() ? MODE_INIT : MODE_PORT;
    g_locked = true;
    Serial.printf("mode: %s (auto)\n",
                  (g_mode == MODE_INIT) ? "init tester" : "port tester");
  }

  // ---- one button, two gestures ----
  const uint8_t ev = button_event();
  if (ev == 2) {
    switch_mode();
  } else if (ev == 1) {
    if (g_mode == MODE_PORT) {
      glcd_flip = !glcd_flip;                     // turn the picture around
      Serial.printf("flip=%d\n", glcd_flip ? 1 : 0);
    } else {
      init_test_reset();                          // start the measurement over
      Serial.println("init measurement restarted");
    }
  }

  if (g_mode == MODE_PORT) {
    // ---- port tester ----
    static uint32_t tDraw = 0;
    if ((uint32_t)(now - tDraw) >= DRAW_MS) {
      tDraw = now;
      const Reading r = measure_port();
      render_port(r);
      lcd_send_frame(glcd_fb);
      buzzer_alarm(r.shortPN || r.sP == LN_SHORT_GND || r.sN == LN_SHORT_GND ||
                   both_lines_high(r));
      static uint8_t tick = 0;
      if (++tick >= 5) { tick = 0; print_reading(r); }
    }
  } else {
    // ---- init tester ----
    static bool prevMounted = false;
    const bool mounted = TinyUSBDevice.mounted();

    if (mounted && !prevMounted) {
      if (it_t0 == 0) {
        it_t0 = now;
        if (!it_beeped) { it_beeped = true; beep_joy(); }
        Serial.println("INIT   enumerated");
      } else if (!it_os_seen) {
        it_os_seen = true;
        it_os = now;
        Serial.printf("OS     re-enumerated   %.1f s\n", (it_os - it_t0) / 1000.0f);
      }
    }
    prevMounted = mounted;

    if (it_led == 0 && led_reports > 0) {
      it_led = now;
      Serial.printf("LED    host set lamps  %.1f s\n", (it_led - it_t0) / 1000.0f);
    }

    const bool reportProto = (usb_hid.getProtocol() == HID_PROTOCOL_REPORT);
    if (!reportProto) it_saw_boot = true;
    if (it_saw_boot && reportProto && it_t0 != 0 && !it_os_seen) {
      it_os_seen = true;
      it_os = now;
      Serial.printf("OS     report protocol %.1f s\n", (it_os - it_t0) / 1000.0f);
    }

    static uint32_t tDraw = 0;
    if ((uint32_t)(now - tDraw) >= DRAW_MS) {
      tDraw = now;
      render_init();
      lcd_send_frame(glcd_fb);
    }
  }

  static uint32_t tLog = 0;
  if ((uint32_t)(now - tLog) >= LOG_MS) {
    tLog = now;
    Serial.printf("mode=%s host=%s proto=%s led=%s reports=%lu\n",
                  (g_mode == MODE_INIT) ? "init" : "port",
                  TinyUSBDevice.mounted() ? "up" : "down",
                  (usb_hid.getProtocol() == HID_PROTOCOL_REPORT) ? "report" : "boot",
                  (led_state & KEYBOARD_LED_NUMLOCK) ? "numlock-on" : "numlock-off",
                  (unsigned long)led_reports);
  }
}
