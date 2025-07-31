#include <FastLED.h>

// LED 스트립 설정
#define LED_PIN     2        // ESP32의 GPIO 2번 핀
#define NUM_LEDS    30       // LED 개수 (실제 개수에 맞게 조정)
#define BRIGHTNESS  64       // 밝기 (0-255)
#define LED_TYPE    WS2812B  // LED 타입
#define COLOR_ORDER GRB      // 색상 순서

CRGB leds[NUM_LEDS];

// 전역 변수
uint8_t gHue = 0;

void setup() {
  Serial.begin(115200);
  
  // FastLED 초기화
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  Serial.println("WS2812 LED Strip initialized!");
}

void loop() {
  // 밝기 조절 예제들
  
  // 방법 1: 전체 밝기 설정 후 색상 적용
  setBrightness(255);  // 최대 밝기
  setSolidColor(255, 0, 0);    // 빨간색
  delay(1000);
  
  setBrightness(128);  // 50% 밝기
  setSolidColor(255, 0, 0);    // 빨간색 (어두워짐)
  delay(1000);
  
  setBrightness(64);   // 25% 밝기
  setSolidColor(255, 0, 0);    // 빨간색 (더 어두워짐)
  delay(1000);
  
  // 방법 2: 색상과 함께 밝기 직접 설정
  setSolidColorWithBrightness(0, 255, 0, 255);  // 녹색 최대 밝기
  delay(1000);
  
  setSolidColorWithBrightness(0, 255, 0, 128);  // 녹색 50% 밝기
  delay(1000);
  
  setSolidColorWithBrightness(0, 255, 0, 32);   // 녹색 12.5% 밝기
  delay(1000);
  
  // 밝기 페이드 효과
  fadeInOut(0, 0, 255);  // 파란색으로 페이드 인/아웃
  
  turnOffAll();
  delay(1000);
}

// 단색으로 모든 LED 설정하는 함수
void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

// 밝기를 포함한 단색 설정 함수
void setSolidColorWithBrightness(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  // 밝기 값을 0-255에서 0-100 퍼센트로 변환
  float brightnessScale = brightness / 255.0;
  
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r * brightnessScale, g * brightnessScale, b * brightnessScale);
  }
  FastLED.show();
}

// 전체 밝기 설정 함수
void setBrightness(uint8_t brightness) {
  FastLED.setBrightness(brightness);
  FastLED.show();
}

// 특정 LED만 색상 설정하는 함수
void setPixelColor(int pixel, uint8_t r, uint8_t g, uint8_t b) {
  if(pixel >= 0 && pixel < NUM_LEDS) {
    leds[pixel] = CRGB(r, g, b);
    FastLED.show();
  }
}

// LED 모두 끄는 함수
void turnOffAll() {
  setSolidColor(0, 0, 0);
}

// 밝기 페이드 인/아웃 효과
void fadeInOut(uint8_t r, uint8_t g, uint8_t b) {
  // 페이드 인 (밝아지기)
  for(int brightness = 0; brightness <= 255; brightness += 5) {
    setSolidColorWithBrightness(r, g, b, brightness);
    delay(30);
  }
  
  delay(500);  // 잠시 대기
  
  // 페이드 아웃 (어두워지기)
  for(int brightness = 255; brightness >= 0; brightness -= 5) {
    setSolidColorWithBrightness(r, g, b, brightness);
    delay(30);
  }
}

// 호흡등 효과 (사인파 이용)
void breathingEffect(uint8_t r, uint8_t g, uint8_t b, int cycles) {
  for(int cycle = 0; cycle < cycles; cycle++) {
    for(int i = 0; i < 360; i += 2) {
      float brightness = (sin(radians(i)) + 1.0) / 2.0 * 255;
      setSolidColorWithBrightness(r, g, b, (uint8_t)brightness);
      delay(10);
    }
  }
}

// 무지개 효과
void rainbow() {
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
  FastLED.show();
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

// 무지개 + 반짝임
void rainbowWithGlitter() {
  rainbow();
  addGlitter(80);
}

void addGlitter(fract8 chanceOfGlitter) {
  if(random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

// 색종이 효과
void confetti() {
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
  FastLED.show();
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

// 사인파 효과
void sinelon() {
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS-1);
  leds[pos] += CHSV(gHue, 255, 192);
  FastLED.show();
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

// BPM 효과
void bpm() {
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
  FastLED.show();
  EVERY_N_MILLISECONDS(20) { gHue++; }
}

// 저글링 효과
void juggle() {
  fadeToBlackBy(leds, NUM_LEDS, 20);
  byte dothue = 0;
  for(int i = 0; i < 8; i++) {
    leds[beatsin16(i+7, 0, NUM_LEDS-1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  FastLED.show();
}