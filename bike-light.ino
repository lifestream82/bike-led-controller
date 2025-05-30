#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_sleep.h>

// ESP32-C3 Super Mini 핀 설정
const int LED_PIN = 0;          // IRLZ44N 게이트 연결 핀 (GPIO 0)
const int BUTTON_PIN = 2;       // 토글 버튼 핀 (GPIO 2, 웨이크업 핀)
const int STATUS_LED = 8;       // 내장 RGB LED (GPIO 8)

// BLE 서비스 및 캐릭터리스틱 UUID
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define LED_CHAR_UUID       "87654321-4321-4321-4321-abcdef123456"
#define STATUS_CHAR_UUID    "11111111-2222-3333-4444-555555555555"

// BLE 객체
BLEServer* pServer = nullptr;
BLECharacteristic* pLedCharacteristic = nullptr;
BLECharacteristic* pStatusCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// 상태 변수
bool ledState = false;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// 절전 모드 관련
unsigned long lastActivityTime = 0;
const unsigned long SLEEP_TIMEOUT = 30 * 60 * 1000; // 30분 (밀리초)
bool isFirstBoot = true;

// 상태 LED 제어 관련
unsigned long lastBlinkTime = 0;
bool blinkState = false;
const unsigned long BLINK_INTERVAL = 2000; // 2초 간격 (절전을 위해 늘림)
const unsigned long FAST_BLINK_INTERVAL = 500; // 빠른 깜빡임

// RTC 메모리에 저장할 변수 (절전 모드에서도 유지)
RTC_DATA_ATTR bool rtc_ledState = false;
RTC_DATA_ATTR bool rtc_isFirstBoot = true;

// BLE 서버 콜백
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      updateActivity();
      Serial.println("BLE 클라이언트 연결됨");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE 클라이언트 연결 해제됨");
    }
};

// BLE 캐릭터리스틱 콜백
class LedCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      updateActivity();

      if (value.length() > 0) {
        char command = value[0];
        
        switch(command) {
          case '1': // LED 켜기
            ledState = true;
            updateLED();
            Serial.println("BLE: LED 켜기");
            break;
          case '0': // LED 끄기
            ledState = false;
            updateLED();
            Serial.println("BLE: LED 끄기");
            break;
          case 'T': // 토글
          case 't':
            ledState = !ledState;
            updateLED();
            Serial.println("BLE: LED 토글 - " + String(ledState ? "ON" : "OFF"));
            break;
          case 'S': // 상태 요청
          case 's':
            sendStatus();
            break;
          case 'Z': // 절전 모드
          case 'z':
            Serial.println("BLE: 수동 절전 모드 진입");
            delay(1000);
            enterSleepMode();
            break;
        }
        
        // 상태 자동 전송
        sendStatus();
      }
    }
};

void setup() {
  Serial.begin(115200);
  
  // 전력 최적화
  setCpuFrequencyMhz(80); // CPU 주파수 낮추기
  
  // 웨이크업 원인 확인
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("버튼으로 웨이크업됨");
    isFirstBoot = false;
    ledState = rtc_ledState;
  } else if (rtc_isFirstBoot) {
    Serial.println("첫 부팅 - 버튼 대기 모드로 진입");
    enterButtonWaitMode();
  } else {
    Serial.println("전원 리셋 또는 기타 웨이크업");
    isFirstBoot = false;
    ledState = rtc_ledState;
  }
  
  // 핀 모드 설정
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  
  // 상태 복원
  updateLED();
  
  // 활동 시간 초기화
  lastActivityTime = millis();
  
  // BLE 초기화
  setupBLE();
  
  // 버튼 웨이크업 설정
  esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  
  Serial.println("BLE LED 컨트롤러 시작됨");
}

void loop() {
  // BLE 연결 상태 처리
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // BLE 스택에 시간 주기
    pServer->startAdvertising();
    Serial.println("BLE 광고 재시작");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // 상태 LED 업데이트 (절전을 위해 빈도 줄임)
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate >= 500) { // 500ms마다
    updateStatusLED();
    lastStatusUpdate = millis();
  }
  
  // 버튼 상태 확인
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      if (currentButtonState == LOW) {
        ledState = !ledState;
        updateLED();
        updateActivity();
        sendStatus(); // BLE로 상태 전송
        Serial.println("버튼으로 LED 상태 변경: " + String(ledState ? "ON" : "OFF"));
      }
    }
  }
  
  lastButtonState = reading;
  
  // 절전 모드 확인 (LED가 꺼져있고 30분 비활성 시에만)
  if (!ledState && (millis() - lastActivityTime > SLEEP_TIMEOUT)) {
    Serial.println("LED 꺼짐 + 30분 비활성으로 절전 모드 진입");
    enterSleepMode();
  }
  
  // 정기적 상태 전송 (연결된 경우)
  static unsigned long lastStatusSend = 0;
  if (deviceConnected && (millis() - lastStatusSend >= 5000)) { // 5초마다
    sendStatus();
    lastStatusSend = millis();
  }
  
  // CPU 부하 줄이기
  delay(50);
}

void setupBLE() {
  // BLE 장치 초기화
  BLEDevice::init("Bike LED Controller");
  
  // BLE 서버 생성
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // BLE 서비스 생성
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // LED 제어 캐릭터리스틱 (쓰기 가능)
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pLedCharacteristic->setCallbacks(new LedCallbacks());
  
  // 상태 알림 캐릭터리스틱 (알림 가능)
  pStatusCharacteristic = pService->createCharacteristic(
                         STATUS_CHAR_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_NOTIFY
                       );
  pStatusCharacteristic->addDescriptor(new BLE2902());
  
  // 서비스 시작
  pService->start();
  
  // 광고 시작
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE 서비스 시작됨 - 연결 대기 중...");
}

void updateActivity() {
  lastActivityTime = millis();
}

void updateLED() {
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  // RTC 메모리에 상태 저장
  rtc_ledState = ledState;
  rtc_isFirstBoot = false;
}

void updateStatusLED() {
  unsigned long currentTime = millis();
  unsigned long timeUntilSleep = 0;
  
  if (!ledState) {
    unsigned long elapsed = currentTime - lastActivityTime;
    if (elapsed < SLEEP_TIMEOUT) {
      timeUntilSleep = SLEEP_TIMEOUT - elapsed;
    }
  }
  
  // LED가 켜져있으면 절전 모드로 들어가지 않음
  if (ledState) {
    // LED 켜진 상태 - 천천히 깜빡임
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      digitalWrite(STATUS_LED, blinkState ? HIGH : LOW);
      lastBlinkTime = currentTime;
    }
  } else {
    // 절전 모드 5분 전부터 빠르게 깜빡임
    if (timeUntilSleep <= 5 * 60 * 1000) { // 5분 이내
      if (currentTime - lastBlinkTime >= FAST_BLINK_INTERVAL) {
        blinkState = !blinkState;
        digitalWrite(STATUS_LED, blinkState ? HIGH : LOW);
        lastBlinkTime = currentTime;
      }
    } else {
      // 정상 작동 중 - 천천히 깜빡임
      if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
        blinkState = !blinkState;
        digitalWrite(STATUS_LED, blinkState ? HIGH : LOW);
        lastBlinkTime = currentTime;
      }
    }
  }
}

void sendStatus() {
  if (deviceConnected && pStatusCharacteristic) {
    unsigned long timeUntilSleep = 0;
    
    if (!ledState) {
      unsigned long elapsed = millis() - lastActivityTime;
      if (elapsed < SLEEP_TIMEOUT) {
        timeUntilSleep = SLEEP_TIMEOUT - elapsed;
      }
    }
    
    // JSON 형태로 상태 전송
    String status = "{\"led\":" + String(ledState ? "true" : "false") + 
                   ",\"timeUntilSleep\":" + String(timeUntilSleep / 1000) + 
                   ",\"connected\":true}";
    
    pStatusCharacteristic->setValue(status.c_str());
    pStatusCharacteristic->notify();
  }
}

void enterButtonWaitMode() {
  Serial.println("최대 절전 모드 - 버튼 대기 중...");
  
  // 모든 출력 끄기
  digitalWrite(LED_PIN, LOW);
  
  // 상태 LED로 절전 모드 표시
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }
  
  // BLE 끄기
  BLEDevice::deinit(true);
  
  // 버튼 웨이크업 설정
  esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  
  // RTC 메모리 초기화
  rtc_ledState = false;
  rtc_isFirstBoot = false;
  
  delay(100);
  esp_deep_sleep_start();
}

void enterSleepMode() {
  // 현재 상태를 RTC 메모리에 저장
  rtc_ledState = ledState;
  rtc_isFirstBoot = false;
  
  Serial.println("절전 모드 진입 - LED 상태 저장됨: " + String(ledState ? "ON" : "OFF"));
  
  // 클라이언트에게 절전 알림
  if (deviceConnected) {
    String sleepMsg = "{\"led\":" + String(ledState ? "true" : "false") + 
                     ",\"sleeping\":true}";
    pStatusCharacteristic->setValue(sleepMsg.c_str());
    pStatusCharacteristic->notify();
    delay(100);
  }
  
  // 상태 LED로 절전 모드 진입 표시
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // BLE 끄기
  BLEDevice::deinit(true);
  
  delay(100);
  esp_deep_sleep_start();
}
