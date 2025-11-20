#include <bluefruit.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <nrf.h>

#define LIS3DH_CS1   10

BLEUart bleuart;

Adafruit_LIS3DH lis1(LIS3DH_CS1, &SPI, 8000000);

static const uint32_t SAMPLE_HZ = 3000;

uint64_t baseMicros = 0;


struct AccelSample {
  uint32_t timestamp_us; 
  int16_t z;
} __attribute__((packed));

static const int SAMPLE_SIZE_BYTES = sizeof(AccelSample);

static const uint16_t ZBUF_CAP = 1024;
volatile uint16_t z_head = 0, z_tail = 0;
AccelSample zbuf[ZBUF_CAP];

inline void z_push_isr(uint32_t ts, int16_t z){
  uint16_t nh = z_head + 1;
  if (nh >= ZBUF_CAP) nh = 0;
  if (nh == z_tail) return; 
  zbuf[z_head].timestamp_us = ts;
  zbuf[z_head].z = z;
  z_head = nh;
}

int z_pop_block(AccelSample* out, int max_n){
  int n = 0;
  while (n < max_n) {
    uint16_t t = z_tail;
    if (t == z_head) break; // empty
    out[n++] = zbuf[t]; // 구조체 복사
    t++; if (t >= ZBUF_CAP) t = 0;
    z_tail = t;
  }
  return n;
}

static inline int16_t readZ_LIS3DH() {
  const uint8_t READ_Z_CMD = 0xEC; // 0x80|0x40|0x2C
  uint8_t zl, zh;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LIS3DH_CS1, LOW);
  SPI.transfer(READ_Z_CMD);
  zl = SPI.transfer(0);
  zh = SPI.transfer(0);
  digitalWrite(LIS3DH_CS1, HIGH);
  SPI.endTransaction();
  return (int16_t)((zh << 8) | zl);
}

void setupTimer2(uint32_t hz){
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->PRESCALER = 4; 
  NRF_TIMER2->CC[0] = 333;
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER2_IRQn, 3);
  NVIC_EnableIRQ(TIMER2_IRQn);
}

extern "C" void TIMER2_IRQHandler(){
  if (NRF_TIMER2->EVENTS_COMPARE[0]) {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    
    unsigned long current_timestamp_us = micros() - baseMicros;
    int16_t z = readZ_LIS3DH();
    
    z_push_isr(current_timestamp_us, z); 
  }
}

void startAdv(){
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup(){
  Serial.begin(921600);

  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ACC-Z1-US");
  Bluefruit.Periph.setConnInterval(6, 12);
  bleuart.begin();

  startAdv();

  SPI.begin();
  pinMode(LIS3DH_CS1, OUTPUT);
  digitalWrite(LIS3DH_CS1, HIGH);
  if (!lis1.begin()) {
    Serial.println("LIS3DH init failed");
    while(1) yield();
  }
  lis1.setRange(LIS3DH_RANGE_2_G);
  lis1.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
  lis1.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

  setupTimer2(SAMPLE_HZ);
  
  baseMicros = micros();
  NRF_TIMER2->TASKS_START = 1;
}


void loop(){
  const int mtu = 247; // 가정
  const int payload = mtu - 3; // 244 바이트

  // 패킷 당 최대 샘플 수 (10바이트/샘플)
  const int max_samples_per_packet = payload / SAMPLE_SIZE_BYTES; // 24

  if (Bluefruit.connected() && bleuart.notifyEnabled()){
    
    AccelSample tmp[max_samples_per_packet];
    int n = z_pop_block(tmp, max_samples_per_packet);
    
    if (n > 0){
      int bytes_to_send = n * SAMPLE_SIZE_BYTES;
      // tmp 버퍼의 데이터를 직접 전송
      bleuart.write((uint8_t*)tmp, bytes_to_send);
    }
  }
}