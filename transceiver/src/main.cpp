
#include <SoftwareSerial.h>  //standart arduino lib
#include <simple_crc.h>

#define E32_TTL_1W 1  //for library correct device select
#define FREQUENCY_433
//#define LoRa_E32_DEBUG true  //how to activate?
#include "LoRa_E32.h"  //v1.5.13 install from Arduino IDE EByte LoRa E32 library by Renzo

// структура для хранения данных
struct StoredData {
  int16_t recvChannel; //read channel
  int16_t sendChannel; //send channel
  int16_t transceiverId;  
} storedData;  // переменная, с которой мы работаем в программе

#include <EEManager.h>         // подключаем либу EEManager
EEManager memory(storedData);  // передаём нашу переменную (фактически её адрес)



//==========HARDWARE CONFIGURATION==================================

//    debug mode send data to serial in readable format
//    instead of sending it to bt
#define DEBUG_MODE false     //change "false" to "true" to enable
#define HAS_BT_MODULE true  //есть BT модуль, сразу отправляет команду задать имя устройства

//SPECIAL PINS
#define LED_SEND_PIN 16
#define LED_RECV_PIN 10
#define LED_LOWBAT_PIN 2

//Light up warning about low battery on percent lower then
#define LOWBAT_WARNING_PERCENT 20
#define DEVICE_ID_LENGTH 12  // The same value for all devices, +1 for trailzero

char DEVICE_ID[DEVICE_ID_LENGTH] = "LoRa-TR1-";
#define TRANS_DEVICE_ID 1  // last part from DEVICE_ID, min value 1, max value 15 (4bits)
#define DEF_RECV_CHANNEL 23
#define DEF_SEND_CHANNEL 20


// ================================================================
// SETTINGS
// ================================================================

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__HARDSERIAL


// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial1
#define REMOTEXY_SERIAL_SPEED 9600


#include <RemoteXY.h>

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 78 bytes
  { 255,8,0,1,0,71,0,19,0,0,0,0,31,1,106,200,1,1,5,0,
  7,5,12,34,15,85,64,2,26,7,4,39,35,11,84,64,2,26,7,4,
  53,35,11,86,64,2,26,73,87,12,14,76,12,128,19,2,26,0,0,0,
  0,0,0,200,66,0,0,0,0,7,4,75,50,12,85,64,2,26 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int16_t name_id_xy; // -32768 .. +32767
  int16_t channel_recv_xy; // -32768 .. +32767
  int16_t channel_send_xy; // -32768 .. +32767
  int16_t sendcount_xy; // -32768 .. +32767

    // output variables
  int8_t volt_xy; // from 0 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

// ========== LoRa section ==========
SoftwareSerial lora_ss(9, 3);           //e32 tx, e32 rx
LoRa_E32 e32ttl100(&lora_ss, 4, 5, 6);  // e32 TX, e32 RX, auxPim, moPin, m1Pin
//LoRa_E32 e32ttl100(9, 3);//, 4, 5, 6);  // e32 TX e32 RX

SoftwareSerial lora_ss_sender(8, 7);                     //e32 tx, e32 rx
LoRa_E32 e32ttl100_sender(&lora_ss_sender, 20, 14, 15);  // e32 TX, e32 RX, auxPim, moPin, m1Pin

// data read by pin to have stable value
#pragma pack(push, 1)
struct AVG_VALUE {
  uint16_t values[8];
  uint8_t position;
  uint16_t average;
  bool ready;  
};
#pragma pack(pop)

void writeValueToAverage(AVG_VALUE &avg, uint16_t value)
{
  avg.values[avg.position++] = value;
  if (avg.position >= 8) {
    avg.position = 0;
    avg.ready = true;
  }  
  uint32_t sum = avg.values[0];
  uint16_t *ptr = &avg.values[1];
  uint16_t *end_ptr = &avg.values[8]; // illegal pointer for condition
  // sum up
  while (ptr != end_ptr) {
    sum += *ptr;
    ++ptr;
  }
  
  avg.average = sum >> 3;
}

AVG_VALUE batPinAverage;


//data to send recieve by LoRa
#pragma pack(push, 1)
struct DATA {
  //DeviceID
  char id[DEVICE_ID_LENGTH];
  int32_t lat;     //degree mul by 1 000 000
  int32_t lon;     //degree mul by 1 000 000
  int16_t alt;     //meters
  uint16_t speed;  //km/h mul by 10
  int16_t course;  //degrees
  uint8_t sat;     //sat counts

  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  uint8_t battery;  //0-100 level min-max
  uint8_t flags;    //flags, FLAG_OLDDATA etc
  union {
    uint8_t values;
    struct {
      unsigned int t1 : 4;  //scaled 4-bit range value 
      unsigned int t2 : 4;  //scaled 4-bit range value
    } both;
  } trans_battery;
  union {
    uint8_t values;
    struct {
      unsigned int t1 : 4;
      unsigned int t2 : 4;
    } both;
  } trans_id;
  uint8_t reserved[9];  //reserved on future feature for compatibility with older models
  uint8_t crc;          //crc of all data except this last byte
};
#pragma pack(pop)

DATA loraDataPacketReceived;

//#define UART_READOUT_BUFFER 32
//static uint8_t uart_buffer[UART_READOUT_BUFFER];

bool remoteXYEnabled = false;

void setupBLE();
bool isDipSetupReadOn();
void deviceGreetings(bool remoteXY);
uint8_t readBatteryLevel();
uint8_t calcCheckSum(const uint8_t *data, uint16_t length);
void writeBadCrcToBLE();
void printParameters(struct Configuration configuration);
bool listenToLORA();
void sendLoRaData();
uint8_t checkBatteryLed();
bool checkCrc();
void setupLoRa();
String sendCommand(const String command);

void setup() {
  memset(&loraDataPacketReceived, 0, sizeof(loraDataPacketReceived));
  batPinAverage.ready = false;
  batPinAverage.position = 0;


  pinMode(LED_SEND_PIN, OUTPUT);
  pinMode(LED_RECV_PIN, OUTPUT);
  pinMode(LED_LOWBAT_PIN, OUTPUT);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A3, INPUT);

  Serial1.begin(9600);
  Serial.begin(9600);

  storedData.recvChannel = DEF_RECV_CHANNEL;
  storedData.sendChannel = DEF_SEND_CHANNEL;
  storedData.transceiverId = TRANS_DEVICE_ID;
  memory.begin(0, 'c');

  delay(2000);
  Serial.println();
  Serial.println(F("-"));
  delay(50);

#if HAS_BT_MODULE

  setupBLE();

  if (isDipSetupReadOn()) {
    remoteXYEnabled = true;
    RemoteXY_Init();
    RemoteXY.name_id_xy = storedData.transceiverId;
    RemoteXY.channel_recv_xy = storedData.recvChannel;
    RemoteXY.channel_send_xy = storedData.sendChannel;
    RemoteXY.sendcount_xy = 0;
    deviceGreetings(true);
    delay(300);
    return;
  }

#endif
  deviceGreetings(false);

  setupLoRa();

#if DEBUG_MODE
  Serial.print(F("Data size:"));
  Serial.println((int)sizeof(loraDataPacketReceived));
  Serial.print(F("Recv channel:"));
  Serial.println(storedData.recvChannel);
  Serial.print(F("Send channel:"));
  Serial.println(storedData.sendChannel);
  Serial.println(F("All initializations completed."));

#endif
}


#if HAS_BT_MODULE

void setupBLE() {
#if DEBUG_MODE

  Serial.println(F("BLE init"));

#endif

  //if (true) return;

  analogWrite(A1, 255);
  delay(100);
  analogWrite(A1, 0);
  delay(100);

  String setNameCommand = "AT+NAME";
  String expectedReply = "+NAME:";
  expectedReply = expectedReply + DEVICE_ID + storedData.transceiverId;
  String reply = sendCommand(setNameCommand);
#if DEBUG_MODE

  Serial.print(F("[BLE: expected:"));
  Serial.println(expectedReply);
  Serial.print(F("[BLE: reply   :"));
  Serial.println(reply);

#endif
  if (reply != expectedReply) {
    sendCommand(setNameCommand + DEVICE_ID + storedData.transceiverId);
  }
  analogWrite(A1, 255);
}

String sendCommand(const String command) {
#if DEBUG_MODE

  Serial.print(F("BLEC:"));
  Serial.println(command);

#endif
  Serial1.print(command);
  Serial1.print(F("\r\n"));
  Serial1.flush();
  delay(200);  //wait some time
  char reply[200];
  memset(reply, 0, sizeof(reply));
  int i = 0;
  while (Serial1.available() > 0 && (i < 200 - 1)) {
    reply[i] = Serial1.read();
    i += 1;
  }
  if (i > 1) i = i - 2;
  reply[i] = '\0';  //end the string

#if DEBUG_MODE

  Serial.print(F("BLER:"));
  Serial.println(reply);

#endif
  return reply;
}

#endif

void loop() {
  uint8_t batpercent = checkBatteryLed();
  if (remoteXYEnabled) {
    RemoteXY.volt_xy = batpercent;
    //storeData.rotationDir = LOW;
    //storeData.stepsPerHundredSeconds = STEPS_PER_HUNDRED_SECONDS_INITIAL;
    //memset(&RemoteXY.name_xy, 0, sizeof(RemoteXY.name_xy));
    //strcpy(RemoteXY.name_xy, DEVICE_ID);

    // проверяем что что-то поменяли и обновляем
    if (RemoteXY.channel_send_xy != storedData.sendChannel) {
      storedData.sendChannel = RemoteXY.channel_send_xy;
      memory.update();
#if DEBUG_MODE

      Serial.print(F("New send chan:"));
      Serial.println(storedData.sendChannel);

#endif
    }
    if (RemoteXY.channel_recv_xy != storedData.recvChannel) {
      storedData.recvChannel = RemoteXY.channel_recv_xy;
      memory.update();
#if DEBUG_MODE

      Serial.print(F("New recv chan:"));
      Serial.println(storedData.recvChannel);

#endif
    }
    if (RemoteXY.name_id_xy != storedData.transceiverId) {
      if ((RemoteXY.name_id_xy > 0) && (RemoteXY.name_id_xy < 16)) {
        storedData.transceiverId = RemoteXY.name_id_xy;
        memory.update();
#if DEBUG_MODE

        Serial.print(F("New transId:"));
        Serial.println(storedData.transceiverId);

#endif
      }
    }
  
    RemoteXY.sendcount_xy = 1111;

    //save in mem when timeout changes
    memory.tick();
    //sync data with app
    RemoteXY_Handler();
    return;
  }

  delay(5);
  memset(&loraDataPacketReceived, 0, sizeof(loraDataPacketReceived));
  lora_ss.listen();
  if (listenToLORA()) {

    if (checkCrc()) {
      digitalWrite(LED_SEND_PIN, HIGH);
      sendLoRaData();
      digitalWrite(LED_SEND_PIN, LOW);
    } else {
      writeBadCrcToBLE();
    }
  };
}

void blinkError() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_SEND_PIN, HIGH);
    digitalWrite(LED_RECV_PIN, HIGH);
    delay(100);
    digitalWrite(LED_SEND_PIN, LOW);
    digitalWrite(LED_RECV_PIN, LOW);
    delay(100);
  }
}

void blinkLoraOk() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_SEND_PIN, HIGH);
    digitalWrite(LED_RECV_PIN, LOW);
    delay(100);
    digitalWrite(LED_RECV_PIN, HIGH);
    digitalWrite(LED_SEND_PIN, LOW);
    delay(100);
  }
  digitalWrite(LED_RECV_PIN, LOW);
}

void setupLoRa() {
#if DEBUG_MODE

  Serial.println(F("[setupLoRa]"));

#endif

  delay(50);

  lora_ss.begin(9600);
  lora_ss.listen();
  delay(50);

  e32ttl100.begin();

  delay(100);

  Serial.println("RECIEVER CONF:");
  ResponseStructContainer c;
  c = e32ttl100.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  if (c.status.code != E32_SUCCESS) {
    blinkError();

    Serial.println(c.status.getResponseDescription());
    c.close();
    return;
  }
  c.close();

  printParameters(configuration);

  if (configuration.CHAN != storedData.recvChannel) {
    configuration.CHAN = storedData.recvChannel;
    //configuration.OPTION.transmissionPower = POWER_30;
    Serial.println("Set recv: ");
    //e32ttl100.setMode(MODE_3_PROGRAM);
    ResponseStatus rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code != E32_SUCCESS) {
      blinkError();

      Serial.println(c.status.getResponseDescription());
      c.close();
      return;
    }

    Serial.println(rs.getResponseDescription());
    Serial.println(rs.code);

    delay(100);
    Serial.println("New recv config: ");
    c = e32ttl100.getConfiguration();
    configuration = *(Configuration *)c.data;
    c.close();
    printParameters(configuration);
  }

  e32ttl100.setMode(MODE_0_NORMAL);
  //lora_ss.readBytes((uint8_t *)&uart_buffer, UART_READOUT_BUFFER);

  Serial.println("SENDER CONF:");
  lora_ss_sender.begin(9600);
  lora_ss_sender.listen();
  delay(50);
  e32ttl100_sender.begin();

  c = e32ttl100_sender.getConfiguration();
  configuration = *(Configuration *)c.data;
  if (c.status.code != E32_SUCCESS) {
    blinkError();

    Serial.println(c.status.getResponseDescription());
    c.close();
    return;
  }
  c.close();

  printParameters(configuration);

  blinkLoraOk();

  //   setup device
  delay(200);

  if (configuration.CHAN != storedData.sendChannel) {
    configuration.CHAN = storedData.sendChannel;
    //configuration.OPTION.transmissionPower = POWER_30;
    Serial.println("Set sender: ");
    //e32ttl100.setMode(MODE_3_PROGRAM);
    ResponseStatus rs = e32ttl100_sender.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);

    if (rs.code != E32_SUCCESS) {
      blinkError();

      Serial.println(c.status.getResponseDescription());
      c.close();
      return;
    }

    Serial.println(rs.getResponseDescription());
    Serial.println(rs.code);

    delay(100);
    Serial.println("New sender config: ");
    c = e32ttl100_sender.getConfiguration();
    configuration = *(Configuration *)c.data;
    c.close();
    printParameters(configuration);
  }

  e32ttl100_sender.setMode(MODE_0_NORMAL);
  //lora_ss_sender.readBytes((uint8_t *)&uart_buffer, UART_READOUT_BUFFER);

#if DEBUG_MODE

#if READ_LORA_MODE

  Serial.println(F("[setupLoRa] read LoRa ON"));

#else

  Serial.println(F("[setupLoRa] read LoRa OFF"));

#endif
  Serial.println(F("[setupLoRa] done"));

#endif
  lora_ss.listen();

  blinkLoraOk();
}

// ====================== READ LORA ==============================

bool checkCrc() {
  uint8_t crc = calcCheckSum((uint8_t *)&loraDataPacketReceived, sizeof(loraDataPacketReceived) - 1);  //except last byte
  return crc == loraDataPacketReceived.crc;
}

void sendLoRaData() {
  lora_ss_sender.listen();
  delay(20);
  if (loraDataPacketReceived.trans_id.both.t1 == 0) {
    loraDataPacketReceived.trans_battery.both.t1 = (readBatteryLevel() * (uint16_t)255) / 1600;
    loraDataPacketReceived.trans_id.both.t1 = storedData.transceiverId;
  } else if (loraDataPacketReceived.trans_id.both.t2 == 0) {
    loraDataPacketReceived.trans_battery.both.t2 = (readBatteryLevel() * (uint16_t)255) / 1600;
    loraDataPacketReceived.trans_id.both.t2 = storedData.transceiverId;
  }
  loraDataPacketReceived.crc = calcCheckSum((uint8_t *)&loraDataPacketReceived, sizeof(loraDataPacketReceived) - 1);  //except last byte

#if DEBUG_MODE

  Serial.println();
  Serial.print(F("* "));
  Serial.print(loraDataPacketReceived.id);
  Serial.print(F("/"));
  Serial.print(loraDataPacketReceived.lat, 6);
  Serial.print(F("/"));
  Serial.print(loraDataPacketReceived.lon, 6);
  Serial.print(F(" - "));
  Serial.print(loraDataPacketReceived.year);
  Serial.print(F("-"));
  Serial.print(loraDataPacketReceived.month);
  Serial.print(F("-"));
  Serial.print(loraDataPacketReceived.day);
  Serial.print(F(" "));
  Serial.print(loraDataPacketReceived.hour);
  Serial.print(F(":"));
  Serial.print(loraDataPacketReceived.minute);
  Serial.print(F(" (UTC)"));
  Serial.print(F(" trans id1:"));
  Serial.print(loraDataPacketReceived.trans_id.both.t1);
  Serial.print(F(" trans bat1:"));
  Serial.print(loraDataPacketReceived.trans_battery.both.t1);
  Serial.print(F(" trans id2:"));
  Serial.print(loraDataPacketReceived.trans_id.both.t2);
  Serial.print(F(" trans bat2:"));
  Serial.print(loraDataPacketReceived.trans_battery.both.t2);
  Serial.print(F(" flags:"));
  Serial.print(loraDataPacketReceived.flags);
  Serial.print(F(" crc:"));
  Serial.print(loraDataPacketReceived.crc);
  Serial.println(F(" .. *"));
  Serial.println(F("[LoRa: sending..]"));

#endif

  ResponseStatus rs = e32ttl100_sender.sendMessage(&loraDataPacketReceived, sizeof(loraDataPacketReceived));
  if (rs.code != E32_SUCCESS) {
#if DEBUG_MODE

    Serial.println(F("[LoRa: send error]"));
    Serial.println(rs.getResponseDescription());

#endif
    blinkError();

  } else {
#if DEBUG_MODE

    Serial.println(F("[LoRa: sent OK]"));

#endif
  }
  lora_ss.listen();
  delay(20);
}


bool listenToLORA() {
  if (e32ttl100.available() > 1) {
    digitalWrite(LED_RECV_PIN, HIGH);

    ResponseStructContainer rsc = e32ttl100.receiveMessage(sizeof(DATA));
    if (rsc.status.code != 1) {
#if DEBUG_MODE  // dump out what was just received

      Serial.println();
      Serial.println(F("[LoRa: New data wih error.]"));
      Serial.println(rsc.status.getResponseDescription());

#endif
      rsc.close();
      digitalWrite(LED_RECV_PIN, LOW);
      return false;
    } else {
      loraDataPacketReceived = *(DATA *)rsc.data;
      rsc.close();
    }
    digitalWrite(LED_RECV_PIN, LOW);

#if DEBUG_MODE  // dump out what was just received

    Serial.println();
    Serial.println(F("[LoRa: New data..]"));
    Serial.print(F("ID: "));
    Serial.print(loraDataPacketReceived.id);
    Serial.print(F(", lat: "));
    Serial.print(loraDataPacketReceived.lat, 6);
    Serial.print(F(", lon: "));
    Serial.print(loraDataPacketReceived.lon, 6);
    Serial.print(F(", sat: "));
    Serial.print(loraDataPacketReceived.sat);
    Serial.print(F(", bat: "));
    Serial.print(loraDataPacketReceived.battery);
    Serial.print(F(", date: "));
    Serial.print(loraDataPacketReceived.year);
    Serial.print(F("/"));
    Serial.print(loraDataPacketReceived.month);
    Serial.print(F("/"));
    Serial.print(loraDataPacketReceived.day);

    Serial.print(F(", time: "));
    Serial.print(loraDataPacketReceived.hour);
    Serial.print(F(":"));
    Serial.print(loraDataPacketReceived.minute);
    Serial.print(F(":"));
    Serial.print(loraDataPacketReceived.second);
    Serial.println(F(" (UTC)"));

#endif
    return true;
  }
  return false;
}

void writeBadCrcToBLE() {
  //if (true) return;
#if DEBUG_MODE
  Serial.println(F("[BAD crc]"));
#endif
}

void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte  : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans        : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup       : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup       : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC          : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower        : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}


uint8_t checkBatteryLed() {
  uint8_t batPercent = readBatteryLevel();
  if (batPercent < LOWBAT_WARNING_PERCENT) {
    digitalWrite(LED_LOWBAT_PIN, HIGH);
  } else {
    digitalWrite(LED_LOWBAT_PIN, LOW);
  }
  return batPercent;
}


const static float BATTERY_LEVEL_MULTIPLIER = 4.2 * (10.0 + 10.0) / 12.9;

// return 0 to 100 values
uint8_t readBatteryLevel() {
#if DEBUG_MODE

  //Serial.println(BATTERY_LEVEL_MULTIPLIER);

#endif
  uint16_t batRef = analogRead(A3);
  writeValueToAverage(batPinAverage, batRef);
  if (!batPinAverage.ready) return 100; //safe to tell battery is OK

#if DEBUG_MODE

  //Serial.println(ar);

#endif
  uint16_t volts = batPinAverage.average * BATTERY_LEVEL_MULTIPLIER;  
#if DEBUG_MODE

  //Serial.println(volts);

#endif
  uint8_t capacity;
  if (volts > 4200) 
    capacity = 100;
  else if (volts > 3870)
    capacity = map(volts, 4200, 3870, 100, 77);
  else if (volts > 3750)
    capacity = map(volts, 3870, 3750, 77, 54);
  else if (volts > 3680)
    capacity = map(volts, 3750, 3680, 54, 31);
  else if (volts > 3400)
    capacity = map(volts, 3680, 3400, 31, 8);
  else if (volts > 2600)
    capacity = map(volts, 3400, 2600, 8, 0);
  else 
    capacity = 0;

  return capacity;
}

bool isDipSetupReadOn() {
  uint16_t ar = analogRead(A0);
#if DEBUG_MODE

  //Serial.print("Dip1 Lora:");
  //Serial.println(ar);

#endif

  return ar < 100;
}

void deviceGreetings(bool remoteXY) {
  if (remoteXY) {
    digitalWrite(LED_LOWBAT_PIN, HIGH);
    digitalWrite(LED_RECV_PIN, HIGH);
    digitalWrite(LED_SEND_PIN, HIGH);
    delay(300);
    digitalWrite(LED_RECV_PIN, LOW);
    digitalWrite(LED_LOWBAT_PIN, LOW);
    digitalWrite(LED_SEND_PIN, LOW);
    return;
  }
  digitalWrite(LED_LOWBAT_PIN, HIGH);
  delay(300);
  digitalWrite(LED_LOWBAT_PIN, LOW);
  digitalWrite(LED_RECV_PIN, HIGH);
  delay(300);
  digitalWrite(LED_RECV_PIN, LOW);
  digitalWrite(LED_SEND_PIN, HIGH);
  delay(300);
  digitalWrite(LED_SEND_PIN, LOW);
  checkBatteryLed();
#if DEBUG_MODE

  Serial.println();
  for (int i = 0; i < 30; i++) {
    Serial.print(F("-"));
    delay(100);
  }
  Serial.println();
  Serial.print(F("Transmitter "));
  Serial.print(DEVICE_ID);
  Serial.println(F(" started.."));

#else

  delay(500);

#endif
}