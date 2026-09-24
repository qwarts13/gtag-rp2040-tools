// ============================================================
//  gtag_lcd.h - G-Tag 256x128 LCD driver, 4-wire transport + 5x7 font
//
//  Shared by the G-Tag tools. Transport verified word for word against a
//  logic-analyzer capture of the stock firmware (see the
//  lukdut/gtag-nrf-display project, docs/lcd-protocol-review.md).
//
//  Default wiring, change the four defines below if you moved the wires:
//    DIO=GP11  CLK=GP10  CS=GP9  RESET=GP8
//  DisplayCLK (~28.8 kHz) is supplied by the tag board and is not driven here.
//
//  Framebuffer: byte index y*32 + x/8, bit x%8, 1 = white.
// ============================================================
#pragma once
#include <Arduino.h>

#define LCD_PIN_DIO     11
#define LCD_PIN_CLK     10
#define LCD_PIN_CS       9
#define LCD_PIN_RESET    8

static constexpr uint16_t GLCD_W       = 256;
static constexpr uint16_t GLCD_H       = 128;
static constexpr uint16_t GLCD_FB_SIZE = GLCD_W * GLCD_H / 8;   // 4096
static constexpr uint8_t  GLCD_HALF_US = 2;
static constexpr bool     GLCD_INK     = false;   // black ink on white paper

static uint8_t glcd_fb[GLCD_FB_SIZE];
static bool    glcd_flip = true;                  // rotate the picture 180

// ---------------- drawing ----------------
static inline void px(int x, int y, bool white = GLCD_INK) {
  if ((unsigned)x >= (unsigned)GLCD_W || (unsigned)y >= (unsigned)GLCD_H) return;
  const int px_ = glcd_flip ? (GLCD_W - 1 - x) : x;
  const int py_ = glcd_flip ? (GLCD_H - 1 - y) : y;
  const uint16_t idx = (uint16_t)py_ * 32u + (uint16_t)(px_ >> 3);
  const uint8_t  mask = uint8_t(1u << (px_ & 7));
  if (white) glcd_fb[idx] |=  mask;
  else       glcd_fb[idx] &= uint8_t(~mask);
}

static void fillRect(int x, int y, int w, int h, bool white = GLCD_INK) {
  for (int yy = 0; yy < h; ++yy)
    for (int xx = 0; xx < w; ++xx) px(x + xx, y + yy, white);
}

static void rect(int x, int y, int w, int h, bool white = GLCD_INK) {
  for (int i = 0; i < w; ++i) { px(x + i, y, white); px(x + i, y + h - 1, white); }
  for (int i = 0; i < h; ++i) { px(x, y + i, white); px(x + w - 1, y + i, white); }
}

// Double headed vertical arrow, used to show which rows are shorted.
static void drawVArrow(int x, int y1, int y2) {
  const int top = (y1 < y2) ? y1 : y2;
  const int bot = (y1 < y2) ? y2 : y1;
  for (int y = top; y <= bot; ++y) px(x, y, GLCD_INK);
  for (int d = 0; d < 7; ++d) {
    px(x - d, top + d, GLCD_INK);
    px(x + d, top + d, GLCD_INK);
    px(x - d, bot - d, GLCD_INK);
    px(x + d, bot - d, GLCD_INK);
  }
}

// ---------------- font ----------------
// 5x7, five columns per glyph, bit 0 = top row. Latin A-Z, digits,
// punctuation and Russian Cyrillic.
static const uint8_t *glyph(uint32_t cp) {
  static const uint8_t SP[5]    = {0x00,0x00,0x00,0x00,0x00};
  static const uint8_t DOT[5]   = {0x00,0x00,0x60,0x00,0x00};
  static const uint8_t DASH[5]  = {0x08,0x08,0x08,0x08,0x08};
  static const uint8_t PLUS[5]  = {0x08,0x08,0x3E,0x08,0x08};
  static const uint8_t COL[5]   = {0x00,0x36,0x36,0x00,0x00};
  static const uint8_t SL[5]    = {0x20,0x10,0x08,0x04,0x02};
  static const uint8_t PCT[5]   = {0x23,0x13,0x08,0x64,0x62};
  /* Latin */
  static const uint8_t A[5]     = {0x7E,0x11,0x11,0x11,0x7E};
  static const uint8_t B[5]     = {0x7F,0x49,0x49,0x49,0x36};
  static const uint8_t C[5]     = {0x3E,0x41,0x41,0x41,0x22};
  static const uint8_t D[5]     = {0x7F,0x41,0x41,0x22,0x1C};
  static const uint8_t E[5]     = {0x7F,0x49,0x49,0x49,0x41};
  static const uint8_t F[5]     = {0x7F,0x09,0x09,0x09,0x01};
  static const uint8_t G[5]     = {0x3E,0x41,0x49,0x49,0x7A};
  static const uint8_t H[5]     = {0x7F,0x08,0x08,0x08,0x7F};
  static const uint8_t I[5]     = {0x00,0x41,0x7F,0x41,0x00};
  static const uint8_t J[5]     = {0x38,0x40,0x41,0x3F,0x01};
  static const uint8_t K[5]     = {0x7F,0x08,0x14,0x22,0x41};
  static const uint8_t L[5]     = {0x7F,0x40,0x40,0x40,0x40};
  static const uint8_t M[5]     = {0x7F,0x02,0x0C,0x02,0x7F};
  static const uint8_t N[5]     = {0x7F,0x02,0x0C,0x10,0x7F};
  static const uint8_t O[5]     = {0x3E,0x41,0x41,0x41,0x3E};
  static const uint8_t P[5]     = {0x7F,0x09,0x09,0x09,0x06};
  static const uint8_t Q[5]     = {0x3E,0x41,0x51,0x21,0x5E};
  static const uint8_t R[5]     = {0x7F,0x09,0x19,0x29,0x46};
  static const uint8_t S[5]     = {0x46,0x49,0x49,0x49,0x31};
  static const uint8_t T[5]     = {0x01,0x01,0x7F,0x01,0x01};
  static const uint8_t U[5]     = {0x3F,0x40,0x40,0x40,0x3F};
  static const uint8_t V[5]     = {0x1F,0x20,0x40,0x20,0x1F};
  static const uint8_t W[5]     = {0x3F,0x40,0x38,0x40,0x3F};
  static const uint8_t X[5]     = {0x63,0x14,0x08,0x14,0x63};
  static const uint8_t Y[5]     = {0x07,0x08,0x70,0x08,0x07};
  static const uint8_t Z[5]     = {0x61,0x51,0x49,0x45,0x43};
  /* Cyrillic letters without a Latin twin */
  static const uint8_t CY_B[5]  = {0x7F,0x49,0x49,0x49,0x30};
  static const uint8_t CY_G[5]  = {0x7F,0x01,0x01,0x01,0x01};
  static const uint8_t CY_D[5]  = {0x70,0x1F,0x11,0x1F,0x70};
  static const uint8_t CY_ZH[5] = {0x63,0x14,0x7F,0x14,0x63};
  static const uint8_t CY_Z[5]  = {0x41,0x49,0x49,0x49,0x3E};
  static const uint8_t CY_I[5]  = {0x7F,0x10,0x08,0x04,0x7F};  // И (diagonal rises)
  static const uint8_t CY_L[5]  = {0x7E,0x07,0x01,0x01,0x7F};
  static const uint8_t CY_P[5]  = {0x7F,0x01,0x01,0x01,0x7F};
  static const uint8_t CY_U[5]  = {0x03,0x04,0x08,0x10,0x7F};  // У (stem on the right)
  static const uint8_t CY_F[5]  = {0x3E,0x41,0x7F,0x41,0x3E};
  static const uint8_t CY_TS[5] = {0x3F,0x20,0x20,0x60,0x3F};
  static const uint8_t CY_CH[5] = {0x0F,0x08,0x08,0x08,0x7F};
  static const uint8_t CY_SH[5] = {0x7F,0x40,0x7F,0x40,0x7F};
  static const uint8_t CY_SC[5] = {0x3F,0x20,0x3F,0x60,0x3F};
  static const uint8_t CY_HRD[5]= {0x01,0x01,0x7F,0x49,0x30};
  static const uint8_t CY_Y[5]  = {0x7F,0x48,0x48,0x7F,0x30};
  static const uint8_t CY_SFT[5]= {0x7F,0x48,0x48,0x48,0x30};
  static const uint8_t CY_E[5]  = {0x3E,0x41,0x41,0x49,0x08};
  static const uint8_t CY_YU[5] = {0x7F,0x08,0x7F,0x41,0x3E};
  static const uint8_t CY_YA[5] = {0x46,0x29,0x19,0x09,0x7F};  // Я (single right wall)
  static const uint8_t G_0[5]   = {0x3E,0x51,0x49,0x45,0x3E};
  static const uint8_t G_1[5]   = {0x00,0x42,0x7F,0x40,0x00};
  static const uint8_t G_2[5]   = {0x42,0x61,0x51,0x49,0x46};
  static const uint8_t G_3[5]   = {0x21,0x41,0x45,0x4B,0x31};
  static const uint8_t G_4[5]   = {0x18,0x14,0x12,0x7F,0x10};
  static const uint8_t G_5[5]   = {0x27,0x45,0x45,0x45,0x39};
  static const uint8_t G_6[5]   = {0x3C,0x4A,0x49,0x49,0x30};
  static const uint8_t G_7[5]   = {0x01,0x71,0x09,0x05,0x03};
  static const uint8_t G_8[5]   = {0x36,0x49,0x49,0x49,0x36};
  static const uint8_t G_9[5]   = {0x06,0x49,0x49,0x29,0x1E};

  if (cp >= 0x0430 && cp <= 0x044F) cp -= 0x20;   // lower case reuses upper
  if (cp == 0x0451) cp = 0x0415;                  // ё
  if (cp == 0x0419) cp = 0x0418;                  // Й -> И
  if (cp == 0x0401) cp = 0x0415;                  // Ё -> Е

  switch (cp) {
    case ' ': return SP;   case '.': return DOT;  case '-': return DASH;
    case '+': return PLUS; case ':': return COL;  case '/': return SL;
    case '%': return PCT;
    case '0': return G_0;  case '1': return G_1;  case '2': return G_2;
    case '3': return G_3;  case '4': return G_4;  case '5': return G_5;
    case '6': return G_6;  case '7': return G_7;  case '8': return G_8;
    case '9': return G_9;
    case 'A': case 0x0410: return A;
    case 'B': case 0x0412: return B;
    case 'C': case 0x0421: return C;
    case 'D': return D;
    case 'E': case 0x0415: return E;
    case 'F': return F;
    case 'G': return G;
    case 'H': case 0x041D: return H;
    case 'I': return I;
    case 'J': return J;
    case 'K': case 0x041A: return K;
    case 'L': return L;
    case 'M': case 0x041C: return M;
    case 'N': return N;
    case 'O': case 0x041E: return O;
    case 'P': case 0x0420: return P;
    case 'Q': return Q;
    case 'R': return R;
    case 'S': return S;
    case 'T': case 0x0422: return T;
    case 'U': return U;
    case 'V': return V;
    case 'W': return W;
    case 'X': case 0x0425: return X;
    case 'Y': return Y;
    case 'Z': return Z;
    case 0x0411: return CY_B;
    case 0x0413: return CY_G;
    case 0x0414: return CY_D;
    case 0x0416: return CY_ZH;
    case 0x0417: return CY_Z;
    case 0x0418: return CY_I;
    case 0x041B: return CY_L;
    case 0x041F: return CY_P;
    case 0x0423: return CY_U;
    case 0x0424: return CY_F;
    case 0x0426: return CY_TS;
    case 0x0427: return CY_CH;
    case 0x0428: return CY_SH;
    case 0x0429: return CY_SC;
    case 0x042A: return CY_HRD;
    case 0x042B: return CY_Y;
    case 0x042C: return CY_SFT;
    case 0x042D: return CY_E;
    case 0x042E: return CY_YU;
    case 0x042F: return CY_YA;
    default:  return nullptr;
  }
}

static void drawGlyph(int x, int y, uint32_t cp, int scale) {
  const uint8_t *g = glyph(cp);
  if (!g) return;
  for (int cx = 0; cx < 5; ++cx)
    for (int cy = 0; cy < 7; ++cy)
      if (g[cx] & (1u << cy)) fillRect(x + cx * scale, y + cy * scale, scale, scale);
}

// Minimal UTF-8 decoder so Russian string literals work.
static void drawText(int x, int y, const char *s, int scale = 1) {
  const uint8_t *p = (const uint8_t *)s;
  while (*p) {
    uint32_t cp = *p++;
    if (cp >= 0xC0) {
      if ((cp & 0xE0) == 0xC0 && (*p & 0xC0) == 0x80) {
        cp = ((cp & 0x1Fu) << 6) | (*p++ & 0x3Fu);
      } else if ((cp & 0xF0) == 0xE0 && (p[0] & 0xC0) == 0x80 && (p[1] & 0xC0) == 0x80) {
        cp = ((cp & 0x0Fu) << 12) | ((uint32_t)(p[0] & 0x3Fu) << 6) | (p[1] & 0x3Fu);
        p += 2;
      }
    }
    if (cp != ' ') drawGlyph(x, y, cp, scale);
    x += 6 * scale;
  }
}

static int textChars(const char *s) {
  int n = 0;
  const uint8_t *p = (const uint8_t *)s;
  while (*p) { if ((*p++ & 0xC0) != 0x80) ++n; }
  return n;
}

static void drawTextRight(int rightX, int y, const char *s, int scale = 1) {
  drawText(rightX - textChars(s) * 6 * scale, y, s, scale);
}

static void drawTextCenter(int centerX, int y, const char *s, int scale = 1) {
  drawText(centerX - textChars(s) * 6 * scale / 2, y, s, scale);
}

// ---------------- transport ----------------
static void lcd_word(bool is_data, uint8_t byte) {
  digitalWrite(LCD_PIN_CLK, LOW);
  digitalWrite(LCD_PIN_CS, LOW);
  delayMicroseconds(GLCD_HALF_US);

  const uint16_t value = uint16_t(is_data ? 0x100u : 0u) | byte;
  for (int bit = 8; bit >= 0; --bit) {
    digitalWrite(LCD_PIN_DIO, ((value >> bit) & 1u) ? HIGH : LOW);
    delayMicroseconds(GLCD_HALF_US);
    digitalWrite(LCD_PIN_CLK, HIGH);
    delayMicroseconds(GLCD_HALF_US);
    digitalWrite(LCD_PIN_CLK, LOW);
  }

  delayMicroseconds(GLCD_HALF_US);
  digitalWrite(LCD_PIN_CS, HIGH);
  delayMicroseconds(GLCD_HALF_US);
}

static void lcd_address(uint16_t address) {
  lcd_word(false, 0x2A);
  lcd_word(true, uint8_t(address >> 8));
  lcd_word(true, uint8_t(address & 0xFF));
}

static void lcd_send_frame(const uint8_t *f) {
  lcd_address(0);
  lcd_word(false, 0x2C);
  for (uint16_t i = 0; i < GLCD_FB_SIZE; ++i) lcd_word(true, f[i]);
  digitalWrite(LCD_PIN_DIO, LOW);
}

static void lcd_init() {
  pinMode(LCD_PIN_DIO, OUTPUT);
  pinMode(LCD_PIN_CLK, OUTPUT);
  pinMode(LCD_PIN_CS, OUTPUT);
  pinMode(LCD_PIN_RESET, OUTPUT);
  digitalWrite(LCD_PIN_CS, HIGH);
  digitalWrite(LCD_PIN_CLK, LOW);
  digitalWrite(LCD_PIN_DIO, LOW);

  digitalWrite(LCD_PIN_RESET, LOW);
  delayMicroseconds(50);
  digitalWrite(LCD_PIN_RESET, HIGH);
  delay(50);

  lcd_word(false, 0x11);
  for (uint16_t base = 0; base < GLCD_FB_SIZE; base += 256) {
    lcd_address(base);
    lcd_word(false, 0x2C);
    for (unsigned j = 0; j < 256; ++j) lcd_word(true, 0xFF);
  }
  digitalWrite(LCD_PIN_DIO, LOW);
  delay(10);

  lcd_word(false, 0x4C);
  lcd_word(true, 0x0C); lcd_word(true, 0x00);
  lcd_word(true, 0x00); lcd_word(true, 0x00);
  delay(4);

  lcd_word(false, 0x4D);
  lcd_word(true, 0xFF); lcd_word(true, 0x00); lcd_word(true, 0x7F);
  delay(1);

  lcd_word(false, 0x4E);
  lcd_word(true, 0x60);
  delay(500);
}
