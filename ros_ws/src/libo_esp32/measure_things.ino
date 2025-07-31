/*
   HX711 로드셀 실사용 코드
   캘리브레이션된 값을 EEPROM에서 자동으로 불러와서 무게 측정
*/

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//핀 설정:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 18;  //mcu > HX711 sck pin

//HX711 객체 생성:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

void setup() {
  Serial.begin(115200); 
  delay(10);
  Serial.println();
  Serial.println("=== HX711 로드셀 시작 ===");

  LoadCell.begin();
  
  // 안정화 시간 설정
  unsigned long stabilizingtime = 2000;
  boolean _tare = true; // 시작 시 자동 영점 조정
  LoadCell.start(stabilizingtime, _tare);
  
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("❌ 오류: HX711 연결을 확인하세요!");
    while (1);
  }
  else {
    // EEPROM에서 저장된 캘리브레이션 값 불러오기
#if defined(ESP8266)|| defined(ESP32)
    EEPROM.begin(512);
#endif
    float calibrationValue; 
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    
    if (calibrationValue == 0 || isnan(calibrationValue)) {
      Serial.println("⚠️  경고: EEPROM에 캘리브레이션 값이 없습니다!");
      Serial.println("   먼저 캘리브레이션을 진행하세요.");
      calibrationValue = 1.0; // 기본값
    } else {
      Serial.print("✅ EEPROM에서 캘리브레이션 값 로드: ");
      Serial.println(calibrationValue);
    }
    
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("✅ 로드셀 준비 완료!");
    Serial.println();
    Serial.println("📋 사용법:");
    Serial.println("   t: 영점 조정 (Tare)");  
    Serial.println("   r: 캘리브레이션 다시 하기");
    Serial.println("   c: 캘리브레이션 값 수동 변경");
    Serial.println("=========================");
  }
  
  while (!LoadCell.update());
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 200; // 200ms마다 출력 (초당 5회)

  // 새로운 데이터 확인
  if (LoadCell.update()) newDataReady = true;

  // 무게 값 출력
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float weight = LoadCell.getData();
      
      // 무게 값 포맷팅하여 출력
      Serial.print("🔍 무게: ");
      if (abs(weight) < 0.1) {
        Serial.println("0.0 g");
      } else {
        Serial.print(weight, 1);
        Serial.println(" g");
      }
      
      newDataReady = 0;
      t = millis();
    }
  }

  // 시리얼 명령어 처리
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    
    if (inByte == 't') {
      Serial.println("🔄 영점 조정 중...");
      LoadCell.tareNoDelay();
    }
    else if (inByte == 'r') {
      Serial.println("🔧 캘리브레이션 모드로 전환합니다.");
      Serial.println("   캘리브레이션 코드를 업로드하세요.");
    }
    else if (inByte == 'c') {
      changeCalFactor();
    }
  }

  // 영점 조정 완료 확인
  if (LoadCell.getTareStatus() == true) {
    Serial.println("✅ 영점 조정 완료!");
  }
}

// 캘리브레이션 값 수동 변경 함수
void changeCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  
  Serial.println("=========================");
  Serial.print("현재 캘리브레이션 값: ");
  Serial.println(oldCalibrationValue);
  Serial.println("새로운 값을 입력하세요 (예: 696.0):");
  
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("새로운 캘리브레이션 값: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  
  _resume = false;
  Serial.print("EEPROM에 저장하시겠습니까? (y/n): ");
  
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        Serial.println("✅ 값이 EEPROM에 저장되었습니다!");
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("❌ 값이 저장되지 않았습니다.");
        _resume = true;
      }
    }
  }
  Serial.println("=========================");
}