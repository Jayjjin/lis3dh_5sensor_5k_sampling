#include <bluefruit.h>

BLEClientDis  clientDis;
BLEClientUart clientUart;

struct AccelSample {
  uint32_t timestamp_us; // 4바이트
  int16_t z;             // 2바이트
} __attribute__((packed));

static const int SAMPLE_SIZE_BYTES = sizeof(AccelSample); // 6바이트

static uint8_t g_rx_buffer[1024];
static int g_rx_buffer_len = 0;

void bleuart_rx_callback(BLEClientUart& uart_svc){
  // 1. BLE에서 받은 데이터를 임시 버퍼에 추가
  if (g_rx_buffer_len < (int)sizeof(g_rx_buffer)) {
     int n = uart_svc.read(g_rx_buffer + g_rx_buffer_len, 
                            sizeof(g_rx_buffer) - g_rx_buffer_len);
     if (n > 0) {
       g_rx_buffer_len += n;
     }
  } else {
     g_rx_buffer_len = 0; 
     return;
  }

  int consumed_bytes = 0;
  while (g_rx_buffer_len - consumed_bytes >= SAMPLE_SIZE_BYTES) {
    Serial.write(g_rx_buffer + consumed_bytes, SAMPLE_SIZE_BYTES);
    consumed_bytes += SAMPLE_SIZE_BYTES;
  }

  if (consumed_bytes > 0) {
    int remaining_bytes = g_rx_buffer_len - consumed_bytes;
    if (remaining_bytes > 0) {
      memmove(g_rx_buffer, g_rx_buffer + consumed_bytes, remaining_bytes);
    }
    g_rx_buffer_len = remaining_bytes;
  }
}

void connect_callback(uint16_t conn_handle){
  if ( clientUart.discover(conn_handle) ) {
    clientUart.setRxCallback(bleuart_rx_callback);
    clientUart.enableTXD(); 
  } else {
    Bluefruit.disconnect(conn_handle); 
  }
}

void disconnect_callback(uint16_t, uint8_t reason){
  g_rx_buffer_len = 0; 
}

void scan_callback(ble_gap_evt_adv_report_t* report){
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) ) {
    Bluefruit.Central.connect(report); 
  }
}

void setup(){
  Serial.begin(921600);

  Bluefruit.begin(0, 1);
  Bluefruit.setName("Z1-Central");
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  clientDis.begin();
  clientUart.begin();

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.filterUuid(clientUart.uuid); 
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.setInterval(160, 80); 
  Bluefruit.Scanner.start(0); 
}

void loop(){
}
