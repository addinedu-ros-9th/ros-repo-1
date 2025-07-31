/*
   ESP32 EEPROM 완전 초기화 코드
   모든 EEPROM 메모리를 0xFF (255)로 초기화
*/

#if defined(ESP32)
#include <EEPROM.h>
#endif

const int EEPROM_SIZE = 512; // ESP32 EEPROM 크기 (최대 4096까지 가능)

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== ESP32 EEPROM 완전 초기화 ===");
  
  // 사용자 확인
  Serial.println("⚠️  경고: 모든 EEPROM 데이터가 삭제됩니다!");
  Serial.println("   - HX711 캘리브레이션 값");
  Serial.println("   - 기타 저장된 모든 설정값");
  Serial.println();
  Serial.println("계속하려면 시리얼 모니터에 'YES' 입력:");
  
  // 사용자 입력 대기
  String userInput = "";
  while (userInput != "YES") {
    if (Serial.available() > 0) {
      userInput = Serial.readString();
      userInput.trim(); // 공백 제거
      
      if (userInput == "YES") {
        break;
      } else if (userInput == "NO" || userInput == "N") {
        Serial.println("❌ 초기화가 취소되었습니다.");
        Serial.println("프로그램을 종료합니다.");
        while(1); // 무한 대기
      } else {
        Serial.println("'YES' 또는 'NO'를 입력하세요:");
      }
    }
  }
  
  Serial.println();
  Serial.println("🔄 EEPROM 초기화 시작...");

#if defined(ESP32)
  EEPROM.begin(EEPROM_SIZE); // ESP32 EEPROM 초기화
#endif

  // 진행률 표시를 위한 변수
  int progressStep = EEPROM_SIZE / 10; // 10% 단위로 표시
  
  // 모든 EEPROM 주소를 0xFF로 초기화
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0xFF); // 0xFF = 255 (초기화된 상태)
    
    // 진행률 표시
    if (i % progressStep == 0) {
      int progress = (i * 100) / EEPROM_SIZE;
      Serial.print("진행률: ");
      Serial.print(progress);
      Serial.println("%");
    }
  }

#if defined(ESP32)
  EEPROM.commit(); // ESP32에서는 commit() 필요
#endif

  Serial.println("진행률: 100%");
  Serial.println();
  Serial.println("✅ EEPROM 초기화 완료!");
  
  // 초기화 확인
  Serial.println();
  Serial.println("=== 초기화 확인 ===");
  
  bool allCleared = true;
  for (int i = 0; i < 20; i++) { // 처음 20바이트만 확인
    byte value = EEPROM.read(i);
    Serial.print("주소 ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(value);
    
    if (value != 0xFF) {
      allCleared = false;
    }
  }
  
  if (allCleared) {
    Serial.println("✅ 초기화 성공! 모든 값이 255(0xFF)로 설정됨");
  } else {
    Serial.println("⚠️  일부 값이 제대로 초기화되지 않았습니다.");
  }
  
  Serial.println();
  Serial.println("=== HX711 캘리브레이션 값 확인 ===");
  
  float calibrationValue;
  EEPROM.get(0, calibrationValue); // 주소 0에서 float 값 읽기
  
  if (isnan(calibrationValue)) {
    Serial.println("✅ HX711 캘리브레이션 값이 성공적으로 삭제됨");
  } else {
    Serial.print("⚠️  캘리브레이션 값이 남아있음: ");
    Serial.println(calibrationValue);
  }
  
  Serial.println();
  Serial.println("🔄 이제 HX711 캘리브레이션을 다시 진행하세요!");
  Serial.println("프로그램 종료.");
}

void loop() {
  // 아무것도 하지 않음
}