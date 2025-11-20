#include <bluefruit.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <nrf.h>

#define LED2_PIN     3
#define LIS3DH_CS1   9
#define LIS3DH_CS2   10

// 데이터 구조체 정의 (X, Y, Z 축 모두 포함)
struct __attribute__((packed)) SensorData {
  uint32_t timestamp;
  int16_t  x1, y1, z1;
  int16_t  x2, y2, z2;
};

// --- 원형 버퍼 설정 ---
const int BUFFER_CAPACITY = 1060;
const int TRANSMIT_CHUNK_SIZE = 50; // 한 번에 보내는 데이터 양이 많아져서 청크 사이즈를 줄임

SensorData dataBuffer[BUFFER_CAPACITY];
volatile int head = 0;
volatile int tail = 0;

// 타이밍 기준
unsigned long baseMicros;

// 센서 객체
Adafruit_LIS3DH lis1(LIS3DH_CS1, &SPI, 8000000);
Adafruit_LIS3DH lis2(LIS3DH_CS2, &SPI, 8000000);

// --- 최적화된 저수준 읽기/쓰기 함수 ---
void readAllAxesFast(int16_t* all_values) {
    // LIS3DH 데이터시트 참조:
    // 0x80(읽기) | 0x40(주소 자동 증가) | 0x28(OUT_X_L 레지스터 주소)
    const uint8_t READ_ALL_AXES_CMD = 0xE8; 
    uint8_t buffer[6]; // X, Y, Z 축 데이터를 받을 버퍼
    
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    // 각 센서별로 6바이트씩 읽기
    for (int i = 0; i < 2; i++) {
        uint8_t cs_pin;
        if (i == 0) cs_pin = LIS3DH_CS1;
        else cs_pin = LIS3DH_CS2;

        digitalWrite(cs_pin, LOW);
        SPI.transfer(READ_ALL_AXES_CMD);
        SPI.transfer(buffer, 6); // 한 번에 6바이트 수신
        digitalWrite(cs_pin, HIGH);

        // 수신된 데이터를 int16_t 값으로 변환하여 저장
        all_values[i*3 + 0] = (int16_t)((buffer[1] << 8) | buffer[0]); // X
        all_values[i*3 + 1] = (int16_t)((buffer[3] << 8) | buffer[2]); // Y
        all_values[i*3 + 2] = (int16_t)((buffer[5] << 8) | buffer[4]); // Z
    }

    SPI.endTransaction();
}
void writeLIS3DHRegister(uint8_t cs_pin, uint8_t reg, uint8_t value) {
  uint8_t write_cmd = reg & 0x7F;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_pin, LOW); SPI.transfer(write_cmd); SPI.transfer(value); digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
}

void setupTimer2() {
  NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->PRESCALER = 4;
  // --- 핵심 변경: 샘플링 속도를 1kHz로 조정 ---
  NRF_TIMER2->CC[0]     = 200; // 1000µs -> 1000 Hz
  NRF_TIMER2->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER2->INTENSET  = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER2_IRQn, 1);
  NVIC_EnableIRQ(TIMER2_IRQn);
}

extern "C" void TIMER2_IRQHandler() {
  if (NRF_TIMER2->EVENTS_COMPARE[0]) {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    
    int next_head = (head + 1) % BUFFER_CAPACITY;
    if (next_head == tail) { return; }

    int16_t axes_values[6]; // 5 sensors * 3 axes
    readAllAxesFast(axes_values);

    dataBuffer[head].timestamp = micros() - baseMicros;
    dataBuffer[head].x1 = axes_values[0]; dataBuffer[head].y1 = axes_values[1]; dataBuffer[head].z1 = axes_values[2];
    dataBuffer[head].x2 = axes_values[3]; dataBuffer[head].y2 = axes_values[4]; dataBuffer[head].z2 = axes_values[5];
    
    head = next_head;
  }
}

void setup() {
  Serial.begin(2000000);
  pinMode(LED2_PIN, OUTPUT);

  Serial.println("Ready. Waiting for start signal...");
  while (Serial.available() == 0) { delay(1); }
  char cmd = Serial.read();
  Serial.print("Received: ");  // 추가
  Serial.println(cmd);  // 추가
  
  Serial.println("Initializing sensors...");
  SPI.begin();
  if (!lis1.begin() || !lis2.begin()) {
    Serial.println("Sensor init failed!");
    while (1) yield();
  }
  
  // 센서 기본 설정 및 BDU 비활성화 (이전과 동일)
  lis1.setRange(LIS3DH_RANGE_2_G); lis2.setRange(LIS3DH_RANGE_2_G); 
  lis1.setPerformanceMode(LIS3DH_MODE_LOW_POWER); lis2.setPerformanceMode(LIS3DH_MODE_LOW_POWER); 
  lis1.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); lis2.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ); 
  //writeLIS3DHRegister(LIS3DH_CS1, LIS3DH_REG_CTRL4, 0x08); writeLIS3DHRegister(LIS3DH_CS2, LIS3DH_REG_CTRL4, 0x08); writeLIS3DHRegister(LIS3DH_CS3, LIS3DH_REG_CTRL4, 0x08); writeLIS3DHRegister(LIS3DH_CS4, LIS3DH_REG_CTRL4, 0x08); writeLIS3DHRegister(LIS3DH_CS5, LIS3DH_REG_CTRL4, 0x08);
  
  setupTimer2();

  Serial.println("Ready. Waiting for start signal from MATLAB...");
  while (Serial.available() == 0) { delay(1); }
  Serial.read();

  head = 0; tail = 0;
  baseMicros = micros();
  NRF_TIMER2->TASKS_START = 1;
  Serial.println("Start signal received. Sampling begins now at 1kHz.");
}

void loop() {
  int current_head = head;
  int available_samples = (current_head - tail + BUFFER_CAPACITY) % BUFFER_CAPACITY;

  if (available_samples >= TRANSMIT_CHUNK_SIZE) {
    digitalWrite(LED2_PIN, HIGH);
    if (tail + TRANSMIT_CHUNK_SIZE > BUFFER_CAPACITY) {
        int first_chunk_size = BUFFER_CAPACITY - tail;
        int second_chunk_size = TRANSMIT_CHUNK_SIZE - first_chunk_size;
        Serial.write((uint8_t*)&dataBuffer[tail], first_chunk_size * sizeof(SensorData));
        Serial.write((uint8_t*)&dataBuffer[0], second_chunk_size * sizeof(SensorData));
    } else {
        Serial.write((uint8_t*)&dataBuffer[tail], TRANSMIT_CHUNK_SIZE * sizeof(SensorData));
    }
    tail = (tail + TRANSMIT_CHUNK_SIZE) % BUFFER_CAPACITY;
    digitalWrite(LED2_PIN, LOW);
  }
}
