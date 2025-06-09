
#include <SoftwareSerial.h>  //standart arduino lib
#include <TinyGPSPlus.h>     //v1.0.3 install from Arduino IDE
// Docs: http://arduiniana.org/libraries/tinygpsplus/

#define E32_TTL_1W 1  //for library correct device select
#define FREQUENCY_433
//#define LoRa_E32_DEBUG true  //how to activate?
#include "LoRa_E32.h"  //v1.5.13 install from Arduino IDE EByte LoRa E32 library by Renzo

#include <simple_crc.h> 

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// можете включить вывод отладочной информации в Serial на 115200
//#define REMOTEXY__DEBUGLOG

// определение режима соединения и подключение библиотеки RemoteXY
#define REMOTEXY_MODE__HARDSERIAL

// настройки соединения
#define REMOTEXY_SERIAL Serial1
#define REMOTEXY_SERIAL_SPEED 9600

#include <RemoteXY.h>

// конфигурация интерфейса RemoteXY
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 79 bytes
  { 255,19,0,1,0,72,0,19,0,0,0,0,31,1,106,200,1,1,5,0,
  7,4,12,75,14,68,64,2,26,13,7,4,36,33,20,86,64,2,26,7,
  26,62,47,18,86,64,2,26,73,85,12,16,94,12,128,9,2,26,0,0,
  0,0,0,0,200,66,0,0,0,0,7,52,86,27,21,86,64,2,26 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  char name_xy[13]; // string UTF8 end zero
  int16_t channel_xy; // -32768 .. +32767
  int16_t period_xy; // -32768 .. +32767
  int16_t sendsec_xy; // -32768 .. +32767

    // output variables
  int8_t volt_xy; // from 0 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)

//==========HARDWARE CONFIGURATION==================================
//    debug mode send data to serial in readable format
//    instead of sending it to bt
#define DEBUG_MODE false      //change "false" to "true" to enable
#define READ_GPS_MODE true   //слушает GPS и пишет в LoRa
#define READ_LORA_MODE true  //слушает пакеты LORA и пишет в Serial (там может висеть BT)
#define HAS_BT_MODULE true   //есть BT модуль, сразу отправляет команду задать имя устройства

//SPECIAL PINS
//-------tracker v<12
//#define LED_GPS_PIN 16
//#define LED_LORA_PIN 10
//-------tracker v>=12
#define LED_GPS_PIN 10
#define LED_LORA_PIN 16

#define LED_LOWBAT_PIN 2

//Light up warning about low battery on percent lower then
#define LOWBAT_WARNING_PERCENT 20


// ================================================================
// SETTINGS
// ================================================================
#define SEND_GPS_INTERVAL 10000         // milliseconds, interval to send gps data
#define SEND_GPS_BETWEEN_TIME_SYNC 20   // gps send count between new send time align
#define READ_GPS_DATA_INTERVAL 2800     // milliseconds, GPS readout data time
#define LORA_READ_CHECK_INTERVAL 30000  // milliseconds

#define DEVICE_ID_LENGTH 12  // The same value for all devices, including for trailzero
#define DEVICE_ID_BASE "PB9"
#define DEVICE_ID_CHANNEL_STR "23"
#define DEVICE_ID_CHANNEL_INT 23
#define DEVICE_ID_SERIAL "006"  //UPLOAD ACTUAL!
//Sending on that OFFSET second after zero seconds
#define SEND_GPS_OFFSET 6       //UPLOAD ACTUAL!

//P
//A - продукт: A - Arduino 328 3.3 pro mini + GPS + LoRa + BT
//B - продукт: B - Arduino 328 3.3 pro micro + GPS + LoRa + BT

//9 - ревизия платы - версия с RemoteXY и последующие

//3 символа фичи по коду - GLD (GPS, LoRa, Debug)
//23 - канал Lora, 0 - если не применимо или канал 0.
//001 - серийный номер - 3 символа

//PA1GLX23001 как пример


// структура для хранения данных
struct StoredData {
  int16_t channel;
  int16_t period;
  int16_t sendsec;
} storedData;  // переменная, с которой мы работаем в программе

#include <EEManager.h>         // подключаем либу EEManager
EEManager memory(storedData);  // передаём нашу переменную (фактически её адрес)



#if READ_GPS_MODE
#define DEVICE_ID_MODE1 "G"
#else
#define DEVICE_ID_MODE1 "X"
#endif
#if READ_LORA_MODE
#define DEVICE_ID_MODE2 "L"
#else
#define DEVICE_ID_MODE2 "X"
#endif
#if DEBUG_MODE
#define DEVICE_ID_MODE3 "D"
#else
#define DEVICE_ID_MODE3 "P"
#endif


char DEVICE_ID[DEVICE_ID_LENGTH] = "";

// ========== GPS section ==========
#if READ_GPS_MODE
TinyGPSPlus gps;
SoftwareSerial gps_ss(8, 7);
#endif

// ========== LoRa section ==========
SoftwareSerial lora_ss(9, 3);

LoRa_E32 e32ttl100(&lora_ss, 4, 6, 5);  // e32 TX, e32 RX, auxPim, moPin, m1Pin
//LoRa_E32 e32ttl100(9, 3);//, 4, 5, 6);  // e32 TX e32 RX


//data to blink led by simple method tick and simple state set
#define LED_BLINK_DURATION 100
#pragma pack(push, 1)
struct LED_BLINK {
  uint32_t lastTimeBlink;
  uint16_t okBlinkPeriod;
  uint16_t searchBlinkPeriod;
  uint8_t ledPin;
  bool isStateOK;
  bool _isLedHigh;
  bool _isLedActive;
};
#pragma pack(pop)

LED_BLINK gpsLedBlink;
LED_BLINK lowbatLedBlink;

// Struct to hold values and know average
struct AVG_8_VALUE {
  uint16_t values[8];
  uint8_t position;
  uint16_t average;
  bool ready;  
};

/// @brief Add value to average
/// @param avg average struct 
/// @param value value to add
void writeValueToAverage8(AVG_8_VALUE &avg, uint16_t value)
{
  avg.values[avg.position++] = value;
  if (avg.position >= 8) {
    avg.position = 0;
    avg.ready = true;
  }  
  uint32_t sum = avg.values[0];
  uint16_t *ptr = &avg.values[1];
  uint16_t *end_ptr = &avg.values[8]; // illegal pointer for while condition
  while (ptr != end_ptr) {
    sum += *ptr++;
  }
  avg.average = sum >> 3;
}

AVG_8_VALUE batPinAverage;


const uint8_t FLAG_OLDDATA = 0b00000001;

uint16_t eventId;
uint32_t incomingPacketNumber;
unsigned long _nextGPSCheckTime;
unsigned long _lastTrackerInfoTime;
#define OLD_DATA_SEND_PERIOD_MS 300000 
unsigned long nextOldDataSendTime; 
bool wasDataSentOnce;  //to be able repeat data send 
int16_t sendAttemptBeforeAlign = 0;
bool remoteXYEnabled = false;


//data to send recieve by LoRa
#pragma pack(push, 1)
struct DATA {
  //DeviceID, last char is zero
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
  uint16_t eventId;
  uint8_t reserved[7];  //reserved on future feature for compatibility with older models
  uint8_t crc;          //crc of all data except this last byte
};
#pragma pack(pop)

uint8_t _sep1;  //separate structs
DATA loraDataPacketToSend;
uint8_t _sep2;  //separate structs
DATA loraDataPacketReceived;
uint8_t _sep3;  //separate structs

#define UART_READOUT_BUFFER 32
static uint8_t uart_buffer[UART_READOUT_BUFFER];

void printParameters(struct Configuration configuration);
void writeTrackerInfoToBLE(uint8_t batpercent);
String sendCommand(const String command);
void sendPackageToBluetooth(DATA *package);
void writeNewPositionToBLE();
bool checkRecievedPacketCrc();
bool isDipGPSReadOn();
bool isDipLoraReadOn();
uint8_t readBatteryLevel();
uint8_t checkBatteryLed();
bool listenToLORA();
void logBadCrcToBLE();
void sendLoRaData();
bool readLocationAndSendLoRa();
void setupBLE();
void setupLoRa();
void setupGPS();
uint8_t checkBatteryLed();
void deviceGreetings(bool remoteXY);
void printGPSInfoToSerial();

#if DEBUG_MODE

void serialPrintSpace() {
  Serial.print(F(" "));
}
void serialPrintArrow() {
  Serial.print(F(" -> "));
}
void serialPrintZero() {
  Serial.print(F("0"));
}
void serialPrintComma() {
  Serial.print(F(","));
}
void serialPrintColon() {
  Serial.print(F(":"));
}
void serialPrintSlash() {
  Serial.print(F("/"));
}
#endif
void serial1PrintZero() {
  Serial1.print(F("0"));
}
void serial1PrintDash() {
  Serial1.print(F("-"));  
}
void serial1PrintCRLF() {
  Serial1.print(F("\r\n"));  
}

void tickLedBlink(LED_BLINK &ledBlink) {
  if (!ledBlink._isLedActive) {
    if (ledBlink._isLedHigh) {
      ledBlink._isLedHigh = false;
      digitalWrite(ledBlink.ledPin, LOW);
    }
    return;
  }

  uint32_t t_millis = millis();
  uint32_t timeSpent = t_millis - ledBlink.lastTimeBlink;
  if (ledBlink.isStateOK) {
    if (ledBlink._isLedHigh) {
      if (timeSpent > LED_BLINK_DURATION) {
        ledBlink._isLedHigh = false;
        digitalWrite(ledBlink.ledPin, LOW);
      }
    } else {
      if (timeSpent > ledBlink.okBlinkPeriod) {
        ledBlink._isLedHigh = true;
        ledBlink.lastTimeBlink = t_millis;
        digitalWrite(ledBlink.ledPin, HIGH);
      }
    }
  } else {
    if (ledBlink._isLedHigh) {
      if (timeSpent > LED_BLINK_DURATION) {
        ledBlink._isLedHigh = false;
        digitalWrite(ledBlink.ledPin, LOW);
      }
    } else {
      if (timeSpent > ledBlink.searchBlinkPeriod) {
        ledBlink._isLedHigh = true;
        ledBlink.lastTimeBlink = t_millis;
        digitalWrite(ledBlink.ledPin, HIGH);
      }
    }
  }
}

void setup() {
  eventId = 0;
  incomingPacketNumber = 0;
  batPinAverage.ready = false;
  batPinAverage.position = 0;
  pinMode(LED_GPS_PIN, OUTPUT);
  pinMode(LED_LORA_PIN, OUTPUT);
  pinMode(LED_LOWBAT_PIN, OUTPUT);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A3, INPUT);

  //clear packet data
  memset(&loraDataPacketReceived, 0, sizeof(loraDataPacketReceived));
  memset(&loraDataPacketToSend, 0, sizeof(loraDataPacketToSend));
  
  //fill device name
  memset(DEVICE_ID, 0, sizeof(DEVICE_ID));
  strcat(DEVICE_ID, DEVICE_ID_BASE);
  strcat(DEVICE_ID, DEVICE_ID_MODE1);
  strcat(DEVICE_ID, DEVICE_ID_MODE2);
  strcat(DEVICE_ID, DEVICE_ID_MODE3);
  strcat(DEVICE_ID, DEVICE_ID_CHANNEL_STR);
  strcat(DEVICE_ID, DEVICE_ID_SERIAL);
  strcpy(loraDataPacketToSend.id, DEVICE_ID);

  storedData.channel = DEVICE_ID_CHANNEL_INT;
  storedData.period = SEND_GPS_INTERVAL / 1000;
  storedData.sendsec = SEND_GPS_OFFSET;
  memory.begin(0, 'b');

  if (!isDipLoraReadOn() && !isDipGPSReadOn()) {
    remoteXYEnabled = true;
    RemoteXY_Init();
    memset(&RemoteXY.name_xy, 0, sizeof(RemoteXY.name_xy));
    strcpy(RemoteXY.name_xy, DEVICE_ID);
    RemoteXY.channel_xy = storedData.channel;
    RemoteXY.period_xy = storedData.period;
    RemoteXY.sendsec_xy = storedData.sendsec;
    deviceGreetings(true);
    delay(300);
    return;
  }

  Serial.begin(9600);
  Serial1.begin(9600);

  deviceGreetings(false);

  gpsLedBlink.lastTimeBlink = millis();
  gpsLedBlink.okBlinkPeriod = 3000;
  gpsLedBlink.searchBlinkPeriod = 1000;
  gpsLedBlink.ledPin = LED_GPS_PIN;
  gpsLedBlink.isStateOK = false;
  gpsLedBlink._isLedHigh = false;
  gpsLedBlink._isLedActive = false;

  lowbatLedBlink.lastTimeBlink = millis();
  lowbatLedBlink.okBlinkPeriod = 60000;
  lowbatLedBlink.searchBlinkPeriod = 2000;
  lowbatLedBlink.ledPin = LED_LOWBAT_PIN;
  lowbatLedBlink.isStateOK = false;
  lowbatLedBlink._isLedHigh = false;
  lowbatLedBlink._isLedActive = true;

#if HAS_BT_MODULE

  setupBLE();

#endif
  setupLoRa();
  setupGPS();
#if READ_LORA_MODE

  lora_ss.listen();  //switch to lora software serial
  delay(10);
  lora_ss.readBytes(uart_buffer, UART_READOUT_BUFFER);

#endif

_nextGPSCheckTime = millis();
nextOldDataSendTime = _nextGPSCheckTime + OLD_DATA_SEND_PERIOD_MS;
wasDataSentOnce = false;

#if DEBUG_MODE

  Serial.print(F("GPS P-D:"));
  Serial.println(storedData.period);
  Serial.print(F("CH:"));
  Serial.println(storedData.channel);
  Serial.print(F("SendSec:"));
  Serial.println(storedData.sendsec);

  //Serial.print(F("DS:")); //last time it is 48 byte packet
  //Serial.println((uint8_t)sizeof(loraDataPacketToSend));
  Serial.print(F("Build "));
  Serial.println(__DATE__);
  Serial.println(F("Running.."));

#endif
}

void loop() {
  uint8_t batpercent = checkBatteryLed();
  if (remoteXYEnabled) {
    RemoteXY.volt_xy = batpercent;

    // проверяем что что-то поменяли и обновляем
    if (RemoteXY.channel_xy != storedData.channel) {
      storedData.channel = RemoteXY.channel_xy;
      memory.update();
#if DEBUG_MODE

      //Serial.print(F("New ch:"));
      //Serial.println(storedData.channel);

#endif
    }
    if (RemoteXY.period_xy != storedData.period) {
      storedData.period = RemoteXY.period_xy;
      memory.update();
#if DEBUG_MODE

      //Serial.print(F("New pe:"));
      //Serial.println(storedData.period);

#endif
    }
    if (RemoteXY.sendsec_xy != storedData.sendsec) {
      storedData.sendsec = RemoteXY.sendsec_xy;
      memory.update();
#if DEBUG_MODE

      //Serial.print(F("New ss:"));
      //Serial.println(storedData.sendsec);

#endif
    }

    //save in mem when timeout changes
    memory.tick();
    //sync data with app
    RemoteXY_Handler();
    return;
  }
  delay(10);
  //put millis in args
  tickLedBlink(gpsLedBlink);
  tickLedBlink(lowbatLedBlink);

#if READ_GPS_MODE

  if (millis() >= _nextGPSCheckTime) {
    _nextGPSCheckTime += storedData.period * 1000;
    if (isDipGPSReadOn()) {
      gpsLedBlink._isLedActive = true;
      gps_ss.listen();         //switch to gps software serial
      if (readLocationAndSendLoRa()) { // send location to other trackers
         nextOldDataSendTime = _nextGPSCheckTime + OLD_DATA_SEND_PERIOD_MS;
      }
      if (lora_ss.listen())    //switch to lora software serial if not yet
         lora_ss.readBytes(uart_buffer, UART_READOUT_BUFFER);
      if (millis() >= nextOldDataSendTime) {
        if (wasDataSentOnce) {
          nextOldDataSendTime += OLD_DATA_SEND_PERIOD_MS;
          loraDataPacketToSend.flags |= FLAG_OLDDATA;  //set flag OldData
          sendLoRaData();
        }
      }  
    } else {
      gpsLedBlink._isLedActive = false;
    }
  }

#endif

#if READ_LORA_MODE

  if (isDipLoraReadOn() && listenToLORA()) {
    digitalWrite(LED_LORA_PIN, HIGH);
    incomingPacketNumber++;

    if (checkRecievedPacketCrc())
      writeNewPositionToBLE();
    else
      logBadCrcToBLE();

    digitalWrite(LED_LORA_PIN, LOW);
  };

  //let the user know device status
  if ((millis() - _lastTrackerInfoTime) >= LORA_READ_CHECK_INTERVAL) {
    writeTrackerInfoToBLE(batpercent);
    _lastTrackerInfoTime = millis();
  }

#endif
}

#if HAS_BT_MODULE

void setupBLE() {
#if DEBUG_MODE

  //Serial.println(F("BLE init"));

#endif

  //if (true) return;

  analogWrite(A2, 255);
  delay(100);
  analogWrite(A2, 0);
  delay(100);

  String setNameCommand = "AT+NAME";
  //sendCommand(setNameCommand);
  String expectedReply = "+NAME:";
  expectedReply = expectedReply + DEVICE_ID;
  String reply = sendCommand(setNameCommand);
#if DEBUG_MODE

  //Serial.print(F("BLE: expected:"));
  //Serial.println(expectedReply);
  Serial.print(F("BLE: reply   :"));
  Serial.println(reply);

#endif
  if (reply != expectedReply) {
    sendCommand(setNameCommand + DEVICE_ID);
  }
  analogWrite(A2, 255);
}

String sendCommand(const String command) {
#if DEBUG_MODE

  Serial.print(F("BLEC:"));
  Serial.println(command);

#endif
  Serial1.print(command);
  serial1PrintCRLF();
  Serial1.flush();
  delay(200);  //wait some time
  char reply[200];
  memset(reply, 0, sizeof(reply));
  int i = 0;
  while (Serial1.available() > 0 && (i < 200 - 1)) {
    reply[i] = Serial1.read();
    ++i;
  }
  if (i > 1) i = i - 2;
  reply[i] = '\0';  //end the string

#if DEBUG_MODE

  //Serial.print(F("BLER:"));
  //Serial.println(reply);

#endif
  return reply;
}

#endif

void setupGPS() {

#if READ_GPS_MODE

  gps_ss.begin(9600);

#if DEBUG_MODE

  Serial.println(F("GPS read: ON"));

#endif

#else

#if DEBUG_MODE

  Serial.println(F("[GPS read: OFF"));

#endif


#endif

}

void setupLoRa() {
#if DEBUG_MODE

  Serial.println(F("setupLoRa"));

#endif

  delay(100);

  lora_ss.begin(9600);
  //lora_ss.listen();
  e32ttl100.begin();
  //e32ttl100.resetModule();
  delay(50);

  lora_ss.readBytes(uart_buffer, UART_READOUT_BUFFER);
  ResponseStructContainer c;
  c = e32ttl100.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  if (c.status.code != E32_SUCCESS) {
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_GPS_PIN, HIGH);
      digitalWrite(LED_LORA_PIN, HIGH);
      delay(100);
      digitalWrite(LED_GPS_PIN, LOW);
      digitalWrite(LED_LORA_PIN, LOW);
      delay(100);
    }

    Serial.println(c.status.getResponseDescription());
    c.close();
    return;
  }
  c.close();

  printParameters(configuration);
  //   setup device
  delay(100);

  if (configuration.CHAN != storedData.channel) {
    configuration.CHAN = storedData.channel;
    //configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
    //configuration.OPTION.transmissionPower = POWER_30;
    Serial.println(F("Set: "));
    //e32ttl100.setMode(MODE_3_PROGRAM);
    ResponseStatus rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    Serial.println(rs.getResponseDescription());
    Serial.println(rs.code);

    //e32ttl100.setMode(MODE_0_NORMAL);
    //e32ttl100.resetModule();
    //delay(100);
    //printParameters(configuration);
    delay(100);
    Serial.println(F("New: "));
    c = e32ttl100.getConfiguration();
    configuration = *(Configuration *)c.data;
    c.close();
    printParameters(configuration);
  }

  e32ttl100.setMode(MODE_0_NORMAL);


#if DEBUG_MODE

#if READ_LORA_MODE

  Serial.println(F("r LoRa ON"));

#else

  Serial.println(F("r LoRa OFF"));

#endif

#endif
}

// ====================== READ GPS and SEND TO LORA ==============================
bool readLocationAndSendLoRa() {
#if READ_GPS_MODE

  bool newData = false;  //new sat data (but location may be not available)

  // For three seconds we try to read and parse GPS data

  tickLedBlink(gpsLedBlink);
  tickLedBlink(lowbatLedBlink);
  //read last peace of GPS data, may be corrupted
  gps_ss.readBytes(uart_buffer, UART_READOUT_BUFFER);
  //now read GPS data to parse
  int8_t lastSecondValue = -61;  //impossible value means not set
  unsigned long lastSecondValueMillis = millis();
  for (unsigned long start = millis(); millis() - start < READ_GPS_DATA_INTERVAL;) {
    while (gps_ss.available() > 0) {
      int rb = gps_ss.read();
#if DEBUG_MODE
      //Serial.print((char)rb); //uncomment to see GPS output in log
#endif
      if (gps.encode(rb)) {
        TinyGPSTime time = gps.time;
        if (time.isValid() && !(time.second() == 0 && time.minute() == 0 && time.hour() == 0)) {
          int8_t currentSecondValue = time.second();
          if (lastSecondValue != currentSecondValue) {
            lastSecondValueMillis = millis();
            lastSecondValue = currentSecondValue;
          }
          newData = true;
        }
      }
      tickLedBlink(gpsLedBlink);
      tickLedBlink(lowbatLedBlink);
    }
    tickLedBlink(gpsLedBlink);
    tickLedBlink(lowbatLedBlink);
  }

  if (newData) {
    sendAttemptBeforeAlign--;
    if (sendAttemptBeforeAlign <= 0 && gps.time.isValid()) {
      sendAttemptBeforeAlign = SEND_GPS_BETWEEN_TIME_SYNC;
      int8_t secondsOffset = lastSecondValue % 10;
      int8_t delaySeconds = storedData.sendsec - secondsOffset;
      if (delaySeconds < 0) {
        delaySeconds += 10;
      }
      unsigned long waitUntil = lastSecondValueMillis + ((uint32_t)delaySeconds) * 1000 - READ_GPS_DATA_INTERVAL;

#if DEBUG_MODE

      //Serial.print(" sO:");
      //Serial.print(secondsOffset);
      //Serial.print(" d for s:");
      //Serial.println(delaySeconds);

#endif
      
      _nextGPSCheckTime = waitUntil;
      digitalWrite(LED_GPS_PIN, LOW);
      delay(20);
      digitalWrite(LED_GPS_PIN, HIGH);
      delay(50);
      digitalWrite(LED_GPS_PIN, LOW);
      delay(50);
      digitalWrite(LED_GPS_PIN, HIGH);
      delay(50);
      digitalWrite(LED_GPS_PIN, LOW);

#if DEBUG_MODE
      //int64_t diffMs = (int64_t)_nextGPSCheckTime - (int64_t)millis();
      //int32_t diff32 = (int32_t)diffMs;
      //Serial.print(F("Next ms: "));
      //Serial.println(diff32);

#endif
      // }
    }
#if DEBUG_MODE

    printGPSInfoToSerial();

#endif
  }

  if (newData && (gps.satellites.value() > 2) && gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    gpsLedBlink.isStateOK = true;
    
    // update data to send
    loraDataPacketToSend.lat = (int32_t)(gps.location.lat() * 1000000);
    loraDataPacketToSend.lon = (int32_t)(gps.location.lng() * 1000000);
    loraDataPacketToSend.alt = (int16_t)(gps.altitude.meters());
    loraDataPacketToSend.speed = (uint16_t)(gps.speed.kmph() * 10);
    loraDataPacketToSend.course = (int16_t)(gps.course.deg());

    loraDataPacketToSend.sat = gps.satellites.value();
    TinyGPSDate date = gps.date;
    loraDataPacketToSend.year = date.year();
    loraDataPacketToSend.month = date.month();
    loraDataPacketToSend.day = date.day();
    TinyGPSTime time = gps.time;
    loraDataPacketToSend.hour = time.hour();
    loraDataPacketToSend.minute = time.minute();
    loraDataPacketToSend.second = time.second();

    loraDataPacketToSend.flags &= ~FLAG_OLDDATA;  //clear flag OldData
    wasDataSentOnce = true; 
    sendLoRaData();
    return true;
  } else if (newData && gps.time.isValid()) {
    gpsLedBlink.isStateOK = false;
#if DEBUG_MODE

    //Serial.print(F("GPS:time only:"));
    //TinyGPSTime t = gps.time;
    //Serial.print(t.hour());
    //serialPrintColon();
    //Serial.print(t.minute());
    //serialPrintColon();
    //Serial.println(t.second());

#endif
   
  } else {
    gpsLedBlink.isStateOK = false;
#if DEBUG_MODE

    Serial.println(F("GPS: No valid"));

#endif    
  }

#if DEBUG_MODE
  if (gps.charsProcessed() < 10) {

    Serial.println();
    Serial.println(F("GPS: check Wiring!"));
  }
#endif
#endif
  return false;
}

bool checkRecievedPacketCrc() {
  uint8_t crc = calcCheckSum((uint8_t *)&loraDataPacketReceived, sizeof(loraDataPacketReceived) - 1);  //except last byte
  return crc == loraDataPacketReceived.crc;
}
uint8_t readBatteryLevel();

void sendLoRaData() {
  digitalWrite(LED_LORA_PIN, HIGH);

  loraDataPacketToSend.eventId = ++eventId;
  loraDataPacketToSend.battery = readBatteryLevel();
  loraDataPacketToSend.crc = calcCheckSum((uint8_t *)&loraDataPacketToSend, sizeof(loraDataPacketToSend) - 1);  //except last byte, it is where crc stored

  lora_ss.listen();
  //delay(10);

#if DEBUG_MODE

  Serial.println();
  Serial.print((uint8_t)sizeof(loraDataPacketToSend));
  Serial.print(F("b * "));
  Serial.print(loraDataPacketToSend.id);
  serialPrintSlash();
  Serial.print(loraDataPacketToSend.lat);
  serialPrintSlash();
  Serial.print(loraDataPacketToSend.lon);
  Serial.print(F(" - "));
  Serial.print(loraDataPacketToSend.year);
  Serial.print(F("-"));
  Serial.print(loraDataPacketToSend.month);
  Serial.print(F("-"));
  Serial.print(loraDataPacketToSend.day);
  serialPrintSpace();
  Serial.print(loraDataPacketToSend.hour);
  serialPrintColon();
  Serial.print(loraDataPacketToSend.minute);
  serialPrintColon();
  Serial.println(loraDataPacketToSend.second);
  Serial.print(F(" b:"));
  Serial.print(loraDataPacketToSend.battery);
  Serial.print(F(" f:"));
  Serial.print(loraDataPacketToSend.flags);
  //Serial.print(F(" crc:"));
  //Serial.print(loraDataPacketToSend.crc);
  Serial.println(F(" .. *"));
  Serial.println(F("LoRa: sending.."));

#endif

  ResponseStatus rs = e32ttl100.sendMessage(&loraDataPacketToSend, sizeof(loraDataPacketToSend));
  if (rs.code != E32_SUCCESS) {
#if DEBUG_MODE

    Serial.print(F("LoRa: send error: "));
    Serial.println(rs.getResponseDescription());

#endif
    digitalWrite(LED_GPS_PIN, LOW);
    digitalWrite(LED_LORA_PIN, LOW);
    delay(100);
    digitalWrite(LED_GPS_PIN, HIGH);
    digitalWrite(LED_LORA_PIN, HIGH);
    delay(100);
    digitalWrite(LED_GPS_PIN, LOW);
    digitalWrite(LED_LORA_PIN, LOW);
    delay(100);
    digitalWrite(LED_GPS_PIN, HIGH);
    digitalWrite(LED_LORA_PIN, HIGH);
    delay(100);
    digitalWrite(LED_GPS_PIN, LOW);
    digitalWrite(LED_LORA_PIN, LOW);

  } else {
#if DEBUG_MODE

    Serial.println(F("LoRa: sent OK"));

#endif
  }

  digitalWrite(LED_LORA_PIN, LOW);
}


bool listenToLORA() {
  if (e32ttl100.available() > 1) {
    ResponseStructContainer rsc = e32ttl100.receiveMessage(sizeof(DATA));
    if (rsc.status.code != 1) {
#if DEBUG_MODE  // dump out what was just received

      Serial.println();
      Serial.print(F("LoRa: Get data error: "));
      Serial.println(rsc.status.getResponseDescription());

#endif
      digitalWrite(LED_LORA_PIN, LOW);
      delay(50);
      digitalWrite(LED_GPS_PIN, HIGH);
      delay(50);
      rsc.close();
      return false;
    } else {
      loraDataPacketReceived = *(DATA *)rsc.data;
      rsc.close();
    }

#if DEBUG_MODE  // dump out what was just received

    Serial.println();
    Serial.println(F("LoRa: New data"));
    Serial.print(F("ID: "));
    Serial.print(loraDataPacketReceived.id);
    /*
    Serial.print(F(", lat: "));
    Serial.print(loraDataPacketReceived.lat, 6);
    Serial.print(F(", lon: "));
    Serial.print(loraDataPacketReceived.lon, 6);
    Serial.print(F(", satellites: "));
    Serial.print(loraDataPacketReceived.sat);

    Serial.print(F(", date: "));
    Serial.print(loraDataPacketReceived.year);
    serialPrintSlash();
    Serial.print(loraDataPacketReceived.month);
    serialPrintSlash();
    Serial.print(loraDataPacketReceived.day);
    */
    Serial.print(F(", time: "));
    Serial.print(loraDataPacketReceived.hour);
    serialPrintColon();
    Serial.print(loraDataPacketReceived.minute);
    serialPrintColon();
    Serial.println(loraDataPacketReceived.second);

#endif
    return true;
  }
  return false;
}

void writeTrackerInfoToBLE(uint8_t batpercent) {
  //if (true) return;
#if DEBUG_MODE
  Serial.print(F("Ble tr info. b="));
  Serial.println(batpercent);
#endif

  Serial1.print(F("AGLoRaM"));  
  Serial1.print(F("&ver=2.0"));     // BLE protocol version
  Serial1.print(F("&dn="));       // connected tracker's name
  Serial1.print(DEVICE_ID);
  Serial1.print(F("&db="));    
  Serial1.print(batpercent);  
  serial1PrintCRLF();
  Serial1.flush();
}

void logBadCrcToBLE() {
  //if (true) return;
#if DEBUG_MODE
  Serial.print(F("BAD crc:"));
  Serial.println(loraDataPacketReceived.id);
#endif
}

void writeNewPositionToBLE() {
#if DEBUG_MODE
  Serial.println(F("BLE: Send pckg"));
#endif

  Serial1.print(F("AGLoRaN&ver=2.0"));
  
  sendPackageToBluetooth(&loraDataPacketReceived);
}
//AGLoRa-info&ver=2.0&myId=GLA-22&memorySize=100&memoryIndex=1|
//AGLoRa-newpoint&id=28&name=GLA-01&lat=60.000976&lon=30.247301&timestamp=2023-06-07T15:21:00Z&sat=9|

void sendPackageToBluetooth(DATA *package) {

  package->id[DEVICE_ID_LENGTH-1] = 0; //code safety reason
  Serial1.print(F("&n="));   //other tracker's name
  Serial1.print(package->id);  //NAME_LENGTH bytes

  Serial1.print(F("&a="));                   // cordinates
  Serial1.print(package->lat / 1000000.0, 6);  // latitude
  Serial1.print(F("&o="));                   
  Serial1.print(package->lon / 1000000.0, 6);  // longitute

  Serial1.print(F("&h="));                
  Serial1.print(package->alt);              // alt
  Serial1.print(F("&s="));                
  Serial1.print(package->speed / 10.0, 1);  // speed
  Serial1.print(F("&c="));                
  Serial1.print(package->course);           // crs
  Serial1.print(F("&b="));                
  Serial1.print(package->battery);          // bat
  if (package->trans_id.both.t1 > 0) {
    Serial1.print(F("&t_id1="));              
    Serial1.print(package->trans_id.both.t1);    // trans bat
    Serial1.print(F("&t_bat1="));              
    Serial1.print(package->trans_battery.both.t1);    // trans bat
  }
  if (package->trans_id.both.t2 > 0) {
    Serial1.print(F("&t_id2="));              
    Serial1.print(package->trans_id.both.t2);    // trans bat
    Serial1.print(F("&t_bat2="));              
    Serial1.print(package->trans_battery.both.t2);    // trans bat
  }  
  //Date and time format: 2023-08-17T10:21:00Z
  //  Serial.print(F("&t=2023-08-17T10:21:00Z"));  
  Serial1.print(F("&t="));  
  Serial1.print(package->year);     // year

  serial1PrintDash();
  if (package->month < 10) {
    serial1PrintZero();          // month
  }
  Serial1.print(package->month);  // month
  
  serial1PrintDash();  
  if (package->day < 10) {
    serial1PrintZero();        // day
  }
  Serial1.print(package->day);  // day
  
  Serial1.print(F("T"));  
  if (package->hour < 10) {
    serial1PrintZero();         // hour
  }
  Serial1.print(package->hour);  // hour
  
  Serial1.print(F(":"));  // time separator

  if (package->minute < 10) {
    serial1PrintZero();           // minute
  }
  Serial1.print(package->minute);  // minute
  
  Serial1.print(F(":"));  // time separator

  if (package->second < 10) {
    serial1PrintZero();           // second
  }
  Serial1.print(package->second);  // second
  
  Serial1.print(F("Z"));  // UTC

  // Sensors and additional data
  Serial1.print(F("&sat="));    
  Serial1.print(package->sat);  // satellites  1 byte
  Serial1.print(F("&db="));    
  Serial1.print(readBatteryLevel());  // reader battery level
  Serial1.print(F("&dn="));    
  Serial1.print(DEVICE_ID);  // reader name
  Serial1.print(F("&u="));    
  Serial1.print(package->flags);  
  Serial1.print(F("&e_id="));    
  Serial1.print(package->eventId);  
  Serial1.print(F("&ipn="));    
  Serial1.print(incomingPacketNumber);  
  serial1PrintCRLF();
  Serial1.flush();
}


void printGPSInfoToSerial() {
#if READ_GPS_MODE

#if DEBUG_MODE

  Serial.println();
  Serial.print(F("GPS: upd Sat="));
  Serial.print(gps.satellites.value());
  Serial.print(F(" sabAlign: "));
  Serial.println(sendAttemptBeforeAlign);

  Serial.print(F("Loc: "));
  TinyGPSLocation location = gps.location;
  if (gps.location.isValid()) {
    Serial.print(location.lat(), 6);
    serialPrintComma();
    Serial.print(location.lng(), 6);
  } else {
    Serial.print(F("? "));
    Serial.print(location.lat(), 6);
    serialPrintComma();
    Serial.print(location.lat(), 6);
    serialPrintComma();
    Serial.print(location.lng(), 6);
  }

  Serial.print(F(" Dt: "));
  TinyGPSDate date = gps.date;
  if (gps.date.isValid()) {
    Serial.print(date.month());
    serialPrintSlash();
    Serial.print(date.day());
    serialPrintSlash();
    Serial.print(date.year());
  } else {
    Serial.print(F("? "));
  }

  serialPrintSpace();
  TinyGPSTime time = gps.time;
  if (time.isValid()) {
    if (time.hour() < 10) serialPrintZero();
    Serial.print(time.hour());
    serialPrintColon();
    if (time.minute() < 10) serialPrintZero();
    Serial.print(time.minute());
    serialPrintColon();
    if (time.second() < 10) serialPrintZero();
    Serial.print(time.second());
    Serial.print(F("."));
    if (time.centisecond() < 10) serialPrintZero();
    Serial.println(time.centisecond());
  } else {
    Serial.println(F("?"));
  }

#endif
#endif
}


void printParameters(struct Configuration configuration) {
#if DEBUG_MODE
  
  Serial.print(F("HEAD: "));
  Serial.print(configuration.HEAD, BIN);
  serialPrintSpace();
  Serial.print(configuration.HEAD, DEC);
  serialPrintSpace();
  Serial.println(configuration.HEAD, HEX);
  serialPrintSpace();
  Serial.print(F("AddH: "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL: "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan: "));
  Serial.print(configuration.CHAN, DEC);
  serialPrintArrow();
  Serial.println(configuration.getChannelDescription());
  serialPrintSpace();
  Serial.print(F("SpdParityBit: "));
  Serial.print(configuration.SPED.uartParity, BIN);
  serialPrintArrow();
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpdUARTDatte: "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  serialPrintArrow();
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpdAirDataRate: "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  serialPrintArrow();
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptTrans: "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  serialPrintArrow();
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptPullup: "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  serialPrintArrow();
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptWakeup: "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  serialPrintArrow();
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptFEC: "));
  Serial.print(configuration.OPTION.fec, BIN);
  serialPrintArrow();
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptPower: "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  serialPrintArrow();
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println(F("-"));
#endif
}

uint8_t checkBatteryLed() {
  uint8_t batPercent = readBatteryLevel();
  lowbatLedBlink.isStateOK = batPercent >= LOWBAT_WARNING_PERCENT; 
  return batPercent;
}


const static float BATTERY_LEVEL_MULTIPLIER = 4.2 * (10.0 + 10.0) / 12.9;

/// @brief reads battery level
/// @return 0 to 100
uint8_t readBatteryLevel() {
#if DEBUG_MODE

  //Serial.println(BATTERY_LEVEL_MULTIPLIER);

#endif
  uint16_t batRef = analogRead(A3);
  writeValueToAverage8(batPinAverage, batRef);
  if (!batPinAverage.ready) return 100; //safe to tell battery is OK

#if DEBUG_MODE

  //Serial.println(ar);

#endif
  uint16_t volts = batPinAverage.average * BATTERY_LEVEL_MULTIPLIER;  // несовсем корректно, так как 5 вольт ровно не бывает.
#if DEBUG_MODE

  //----!!!-- when USB is connected reading is not valid --!!!----
  //Serial.println(volts);

#endif
  uint8_t capacity;
  if (volts > 4200) {
    capacity = 100;
  } else if (volts > 3870)
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

//returns true if lora is enabled
bool isDipLoraReadOn() {
  uint16_t ar = analogRead(A0);
#if DEBUG_MODE

  //Serial.print("Dip1 Lora:");
  //Serial.println(ar);

#endif

  return ar > 100;
}


//returns true if gps is enabled
bool isDipGPSReadOn() {
  uint16_t ar = analogRead(A1);
#if DEBUG_MODE

  //Serial.print("Dip2 gps:");
  //Serial.println(ar);

#endif

  return ar > 100;
}

void deviceGreetings(bool remoteXY) {
  if (remoteXY) {
    digitalWrite(LED_LOWBAT_PIN, HIGH);
    digitalWrite(LED_LORA_PIN, HIGH);
    digitalWrite(LED_GPS_PIN, HIGH);
    delay(300);
    digitalWrite(LED_LORA_PIN, LOW);
    digitalWrite(LED_LOWBAT_PIN, LOW);
    digitalWrite(LED_GPS_PIN, LOW);
    return;
  }
  digitalWrite(LED_LOWBAT_PIN, HIGH);
  delay(200);
  digitalWrite(LED_LOWBAT_PIN, LOW);
  digitalWrite(LED_LORA_PIN, HIGH);
  delay(200);
  digitalWrite(LED_LORA_PIN, LOW);
  digitalWrite(LED_GPS_PIN, HIGH);
  delay(200);
  digitalWrite(LED_GPS_PIN, LOW);
  checkBatteryLed();
#if DEBUG_MODE

  Serial.println();
  for (int i = 0; i < 30; i++) {
    Serial.print(F("-"));
    delay(100);
  }
  Serial.println();
  Serial.print(DEVICE_ID);
  Serial.println(F(" started.."));

#else

  delay(100);

#endif
}

/*
current format https://github.com/Udj13/AGLoRa/wiki/AGLoRa-BLE-protocol
To save memory on Arduino, the LoRa RADAR app (https://github.com/Udj13/AGLoRa/wiki/Lora-Radar-app) 
supports shortened versions of parameter names. You can use them if you want to speed up data transmission and save memory.

Here is the parameter mapping table:

AGLoRa-memory = AGLoRaM

AGLoRa-point = AGLoRaP

AGLoRa-newpoint = AGLoRaN

dev_name = dn

dev_bat = db

name = n

lat = a

lon = o

timestamp = t

speed = s

course = c

height = h

battery = b

status = u



*/