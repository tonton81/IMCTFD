#include "IMCTFD.h"
//                       SO SI  SCK CS INT SPI_speed
IMCTFD myFD = IMCTFD(SPI1, 5, 21, 32, 31, 0, 2000000);

#define LED_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(1000); /* give serial some time to start */
  myFD.setBaudRate(500000, 8); // 500kbps arbitration, 4Mbps data
  //setup fifo channels
  myFD.configureFIFO(FIFO1, FIFO_RX, PLSIZE_7, DEPTH_12, PRIORITY_1, TIMESTAMP_ON);
  myFD.configureFIFO(FIFO2, FIFO_TX, PLSIZE_7, DEPTH_12, PRIORITY_1, TIMESTAMP_ON);
  // enable interrupt for RX
  myFD.enableFIFOInterrupt(FIFO1);
  // initialize everything
  myFD.setFIFOFilter(FILTER_0, FIFO1, 0x1, 0x2, 0x05 ); /* lets accept frames 0x1, 0x2, and 0x5 */
  myFD.begin();
  // show config
  delay(1000);
  myFD.currentConfig();
  myFD.currentFilters();

  delay(2000);
  // enable callback
  myFD.onReceive(myCB);
}
#include "TeensyThreads.h"

void loop() {
  delay(500);
  digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
  CANFD_message_t msg;
  msg.len = 21; /* length of data, automatic DLC will choose 24 for you */
  msg.fdf = 1; /* FD frame */
  msg.id = 0x789;
  for ( uint16_t i = 0; i < 21; i++ ) msg.buf[i] = i + 1;
  myFD.write(msg);
}

void myCB(const CANFD_message_t &msg) {
  Serial.println("==========FIFO CALLBACK ===========");
  Serial.print("FIFO");
  Serial.println(msg.fifo);
  Serial.print("from Filter_");
  Serial.println(msg.filthit);
  Serial.print("ID: 0x"); Serial.println(msg.id, HEX);
  Serial.print("TimeStamp: "); Serial.println(msg.timestamp);
  Serial.print("Packet type: "); Serial.print(msg.fdf);
  Serial.print(",   Len: "); Serial.println(msg.len);    Serial.print("Output: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i]); Serial.print(" ");
  } Serial.println();
}
