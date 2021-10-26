#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <OneWire.h>

typedef struct  {
  char PhoneNum[17];
  uint8_t SAlarm;
  uint8_t RAlarm;
  uint8_t PAlarm;
  uint8_t IAlarm;
  unsigned short addr;
} Phone;

char PhoneCharArray[17]; //При изменении заменить и тут RingPhoneNumber.toCharArray(PhoneCharArray, 17);
char PhoneNum[17];
uint8_t SAlarmOffset = 19;
uint8_t RAlarmOffset = 20;
uint8_t PAlarmOffset = 21;
uint8_t IAlarmOffset = 22;

Phone data[10] = {
  {"", 0, 0, 0, 0, 100}, //Только с первого номера можно изменять другие номера.
  {"", 0, 0, 0, 0, 130},
  {"", 0, 0, 0, 0, 160},
  {"", 0, 0, 0, 0, 190},
  {"", 0, 0, 0, 0, 220},
  {"", 0, 0, 0, 0, 250},
  {"", 0, 0, 0, 0, 280},
  {"", 0, 0, 0, 0, 310},
  {"", 0, 0, 0, 0, 340},
  {"", 0, 0, 0, 0, 370}
};

SoftwareSerial mySerial(8, 9); //На этих пинах подключен модем RX, TX
#define ModemResetPin 6 //Номер выхода подключенного пину перезагрузки модема.
#define ExtPowerPin 7 //Номер входа подключенного к делителю напряжения внешнего источника питания для проверки наличия внешнего питания. +5В-R1(2,2К)-.-R2(3,3K)-GND.
#define RLed 10 //Красный светодиод
#define BLed 11 //Синий светодиод
#define GLed 12 //Зеленый светодиод
int8_t ExtPowerState = 0;
int8_t ExtPowerFlag = 2;

OneWire ds(5);
float CurrentTemperature; // Глобальная переменная для хранения значение температуры с датчика DS18B20
unsigned long previousTemperatureTimeMillis = 0; // Переменная для хранения времени последнего считывания с датчика
int8_t TempCheckTime; // Определяем периодичность проверок в минутах

int8_t LowTemperature;
int8_t HighTemperature;
int8_t AlarmTemperatureFlag = 0;

unsigned long previousBlinkTimeMillis = 0;
int8_t BlinkTimeSec = 2;

int8_t RLedState = LOW;
int8_t GLedState = LOW;
int8_t BLedState = LOW;

unsigned long previousWatchPowerTimeMillis = 0;
unsigned long previousNetCheckTimeMillis = 0;
uint8_t ActivateWatchPowerTime = 0;
uint8_t WatchPowerTimeSmsSended = 0;
uint8_t WatchPowerTime = 0; //Задержка времени перед отправкой смс при отключении сети питания
uint16_t NetCheckTime; //Временной интервал для проверки регистрации в сети оператора и уровня сигнала, указывается в минутах
uint16_t RingTime; //Длительность тревожных вызовов
int8_t DevTestOn = 0; //Тестовый режим
int8_t LedOn = 1; //Светодиод включен?
int8_t SensorReadErrorCount = 1;
int8_t WatchPower = 1; //Мониторинг внешнего питания:  0 - выключен, 1 - включен когда поставлено на охрану, 2 - включен всегда.

uint8_t SuperUser = 0;
uint8_t SMSCommand = 0;
uint8_t ConsoleCommand = 0;
int8_t ConsoleEvent = 0;

uint8_t ch = 0;

String BalanceNumber = "#100#"; //Команда запроса баланса. #100# - ответ латинскими буквами, *100# - ответ русскими буквами. Работает только с латинскими буквами. В данном случае команда запроса баланса Beeline(Россия).
uint8_t BalanceStringLen = 22; //Число символов от начала строки которые нужно переслать в смс сообщении при получении USSD ответа о балансе. Убираем спам от оператора при запросе баланса, особо критично для модема M590, обрезаем всё лишнее.
uint8_t BalanceLenOffset = 18;
uint8_t BalanceSettingsOffset = 19;
char BalanceNum[17];

int8_t guard = 1; //Охрана: 1 - включена, 0 - выключена. Если включена, будут отправляться смс при срабатывании датчиков

String val = "";
String NotCheckRingPhone = "";
String RingPhone = "";

int8_t ModemID = 0;

uint8_t CallFlag = 0;
uint8_t EndCallFlag = 0;
uint8_t CallNum = 0;
uint8_t Call = 0;
unsigned long previousCallTimeMillis = 0;

const String Marker = " -> ";
const String Warn = "Alarm! ";
String SendSmsText = "";

uint8_t PrepareAlarmFromEvent;
uint8_t CREG1;
uint8_t CREG2;

uint8_t CSQ1;
uint8_t CSQ2;
uint8_t NetCheckQueue = 0;

String LastEvent = "Guard O";

void setup() {

  Serial.begin(9600);
  Serial.println( F ("GSM Temperature Alarm 2018.12.03-01"));
  lineprint();
  delay(2500);
  randomSeed(analogRead(0));

  //EEPROM.update(0, 255); //Uncomment to reset system config;
  eepromconfig();
  //EEPROM.update(49, 255); //Uncomment to reset balance query config;
  eeprombalancenum();
  //EEPROM.update(99, 255); //Uncomment to reset phones config;
  eepromphonememory();

  pinMode(RLed, OUTPUT);
  pinMode(GLed, OUTPUT);
  pinMode(BLed, OUTPUT);
  digitalWrite(RLed, HIGH);
  digitalWrite(GLed, HIGH);
  digitalWrite(BLed, HIGH);

  pinMode(ModemResetPin, OUTPUT);
  digitalWrite(ModemResetPin, HIGH);

  InitModem();
  if (guard == 1) LastEvent += "N";
  if (guard == 0) LastEvent += "FF";
  LastEvent += F(" at boot.");
  Serial.println(LastEvent);
  Serial.println();
  GetTemperature();
}

void loop() {
  ExternalPowerMon();
  NetStatus(); //Проверка сети каждые n минут
  //Led();
  WatchTemp();
  DelayPowerAlarm();
  Detect();
  AlarmCall();

  if (mySerial.available()) {  //Если GSM модуль что-то послал нам, то
    while (mySerial.available()) {  //сохраняем входную строку в переменную val
      ch = mySerial.read();
      val += char(ch);
      delay(10);
    }
    uint8_t l = 1;
    if (ModemID == 1 || ModemID == 3) l = 1;
    if (ModemID == 2) l = 2;
    if (val.indexOf( F ("RING")) > -1) {  //Если звонок обнаружен, то проверяем номер
      String NotCheckRingPhone = val.substring(String(val.indexOf("\"")).toInt() + l, String(val.indexOf(F(","))).toInt() - 1);
      Serial.print( F ("Ring from: +"));
      Serial.println(NotCheckRingPhone);
      eepromfirstphone(NotCheckRingPhone);
      if (eepromcheckphone(NotCheckRingPhone) == 1) {
        endcall();
        MasterRing();
      } else {
        endcall();
      }
    } else if (val.indexOf( F ("+CMT:")) > -1) {  //Если пришло смс, то проверяем номер
      Serial.println(val);
      if (ModemID == 1 || ModemID == 3) NotCheckRingPhone = val.substring(String(val.indexOf("+CMT: \"")).toInt() + 8, String(val.indexOf(F(",,"))).toInt() - 1);
      if (ModemID == 2) NotCheckRingPhone = val.substring(String(val.indexOf("+CMT: \"")).toInt() + 8, String(val.indexOf(F(",\""))).toInt() - 1);
      Serial.print( F ("SMS from: +"));
      Serial.println(NotCheckRingPhone);
      if (eepromcheckphone(NotCheckRingPhone) == 1) {
        if (eepromchecksuphone(NotCheckRingPhone) == 1) SuperUser = 1;
        SMSCommand = 1;
        MasterCommands();
      }
    } else if (val.indexOf( F ("+CUSD:")) > -1) {  //Если пришел USSD ответ
      Serial.print( F ("USSD query: "));
      //Serial.println(RingPhone);
      Serial.println(val);
      if (val.indexOf("\"") > -1) {
        SendSmsText = val.substring(String(val.indexOf("\"")).toInt() + 1, String(val.indexOf("\"")).toInt() + BalanceStringLen);
        if (ConsoleEvent == 0)sms("+" + RingPhone);
        ConsoleEvent = 0;
        delay(1000);
      }
      mySerial.println( F ("AT+CUSD=0"));//Orig
      delay(200);
      //    } else
    } else if (val.indexOf( F ("+CREG:")) > -1) {  //Если пришла инфа о регистрации в сети
      Serial.println(val);
      if (val.indexOf(": ") > -1) {
        //CREG1 = (val.substring(String(val.indexOf(": ")).toInt() + 2, String(val.indexOf(",")).toInt())).toInt();
        //Serial.println(CREG1);
        CREG2 = (val.substring(String(val.indexOf(",")).toInt() + 1)).toInt();
        Serial.println(CREG2);
        if (CREG2 != 1) RebootModem(); //Если модем не в домашней сети перезагрузить модем
      }
    } else if (val.indexOf( F ("+CSQ:")) > -1) {  //Если пришел уровень сигнала сети
      Serial.println(val);
      if (val.indexOf(": ") > -1) {
        CSQ1 = (val.substring(String(val.indexOf(": ")).toInt() + 2, String(val.indexOf(",")).toInt())).toInt();
        Serial.println(CSQ1);
        //CSQ2 = (val.substring(String(val.indexOf(",")).toInt() + 1)).toInt();
        //Serial.println(CSQ2);
        if (CSQ1 == 99) RebootModem(); //Если нет сети перезагрузить модем
      }
    }
    if (DevTestOn == 1) Serial.println(val);  //Печатаем в монитор порта пришедшую строку
    val = "";
  }
  if (Serial.available()) {  //Если в мониторе порта ввели что-то
    while (Serial.available()) {  //Сохраняем строку в переменную val
      ch = Serial.read();
      val += char(ch);
      delay(20);
    }
    if (DevTestOn == 1) mySerial.println(val);  //передача всех команд набранных в мониторе порта в GSM модуль если включен тестовый режим
    SuperUser = 1;
    ConsoleCommand = 1;
    MasterCommands();
    val = "";  //Очищаем
  }
}
/*
  int8_t
  int8_t GLedState = LOW;
  int8_t BLedState = LOW;
*/

// ----- Проверка превышения температуры
void Detect() {
  unsigned long currentBlinkTimeMillis = millis();
  if (SensorReadErrorCount == 0) {
    digitalWrite(GLed, LOW);
    digitalWrite(RLed, LOW);
    digitalWrite(BLed, LOW);
    if (AlarmTemperatureFlag == 0) {
      AlarmTemperatureFlag = 3;
      SendSmsText = Warn + (F ("\nCheck temperature sensor!"));
      SendSmsText += (F ("\nCurrent temperature: "));
      SendSmsText += CurrentTemperature;
      SmsAlarm(1);
    }
    //Serial.println(F("Sensor Error"));
  }
  else if (CurrentTemperature > HighTemperature) {
    digitalWrite(BLed, HIGH);
    digitalWrite(GLed, HIGH);
    if (guard == 1) {
      digitalWrite(RLed, LOW);
    } else {
      if ((currentBlinkTimeMillis - previousBlinkTimeMillis) / 1000 > BlinkTimeSec) {
        previousBlinkTimeMillis = currentBlinkTimeMillis;
        if (RLedState == LOW)
          RLedState = HIGH;
        else
          RLedState = LOW;
        digitalWrite(RLed, RLedState);
      }
    }
    if (AlarmTemperatureFlag == 0) {
      AlarmTemperatureFlag = 1;
      SendSmsText = Warn + (F ("\nHigh temperature: "));
      SendSmsText += CurrentTemperature;
      SmsAlarm(1);
    }
  }
  else if (CurrentTemperature < LowTemperature) {
    digitalWrite(GLed, HIGH);
    digitalWrite(RLed, HIGH);
    if (guard == 1) {
      digitalWrite(BLed, LOW);
    } else {
      if ((currentBlinkTimeMillis - previousBlinkTimeMillis) / 1000 > BlinkTimeSec) {
        previousBlinkTimeMillis = currentBlinkTimeMillis;
        if (BLedState == LOW)
          BLedState = HIGH;
        else
          BLedState = LOW;
        digitalWrite(BLed, BLedState);
      }
    }
    if (AlarmTemperatureFlag == 0) {
      AlarmTemperatureFlag = 2;
      SendSmsText = Warn + (F ("\nLow temperature: "));
      SendSmsText += CurrentTemperature;
      SmsAlarm(1);
    }
  }
  else {
    digitalWrite(BLed, HIGH);
    digitalWrite(RLed, HIGH);
    if (guard == 1) {
      if (LedOn == 1) {
        digitalWrite(GLed, LOW);
      } else {
        digitalWrite(GLed, HIGH);
      }
    } else {
      if ((currentBlinkTimeMillis - previousBlinkTimeMillis) / 1000 > BlinkTimeSec) {
        previousBlinkTimeMillis = currentBlinkTimeMillis;
        if (GLedState == LOW)
          GLedState = HIGH;
        else
          GLedState = LOW;
        digitalWrite(GLed, GLedState);
      }
    }
    AlarmTemperatureFlag = 0;
  }
  /*  Serial.print(previousBlinkTimeMillis);
    Serial.print(" -> ");
    Serial.print(currentBlinkTimeMillis);
    Serial.print(" -> ");
    Serial.print((currentBlinkTimeMillis - previousBlinkTimeMillis) / 1000);
    Serial.print(" -> ");
    Serial.println(BlinkTimeSec);
    //delay (1000);
  */
}

void WatchTemp() {
  unsigned long currentTemperatureTimeMillis = millis();
  if ((currentTemperatureTimeMillis - previousTemperatureTimeMillis) / 1000 > TempCheckTime * 60) {
    previousTemperatureTimeMillis = currentTemperatureTimeMillis;
    GetTemperature();
  }
}

void GetTemperature() {
  byte TemperatureData[2];
  ds.reset();
  ds.write(0xCC);
  ds.write(0x44);
  delay(1000);
  ds.reset();
  ds.write(0xCC);
  ds.write(0xBE);
  TemperatureData[0] = ds.read();
  TemperatureData[1] = ds.read();
  CurrentTemperature =  ((TemperatureData[1] << 8) | TemperatureData[0]) * 0.0625;
  Serial.print(F("Temperature: "));
  Serial.println(CurrentTemperature);
  if (CurrentTemperature == 0.00) {
    SensorReadErrorCount = --SensorReadErrorCount;
    //Serial.println(SensorReadErrorCount);
    if (SensorReadErrorCount < 0) SensorReadErrorCount = 0;
  } else {
    SensorReadErrorCount = 2;
  }
  //Serial.println(SensorReadErrorCount);
}


// ----- Soft Reset
void(* Reset) (void) = 0;

// ----- Предварительная обработка GuardOn/GuardOff
void MasterRing() {
  if (guard == 1) {
    GuardOff(0);
  }
  else if (guard == 0) {
    GuardOn(0);
  }
}

// ----- Проверка доступности сети
void NetStatus() {
  unsigned long currentNetCheckTimeMillis = millis();
  if (NetCheckTime > 0) {
    if ((currentNetCheckTimeMillis - previousNetCheckTimeMillis) / 1000 > NetCheckTime * 60) {
      previousNetCheckTimeMillis = currentNetCheckTimeMillis;
      if (NetCheckQueue == 0) {
        netlevel(ConsoleCommand);
        NetCheckQueue = 1;
      }
      else {
        netreg(ConsoleCommand);
        NetCheckQueue = 0;
      }
    }
  }
}

// ----- Изменение времени NetCheckTime
void eepromupdateNetCheckTime(uint8_t SmsNetCheckTime) {
  EEPROM.update(15, SmsNetCheckTime);
  NetCheckTime = SmsNetCheckTime;
  Serial.print( F ("NetCheckTime is: "));
  Serial.println(EEPROM.read(15));
  lineprint();
  LedOK();
}

// ----- Изменение времени NetCheckTime
void eepromupdateTempCheckTime(uint8_t SmsTempCheckTime) {
  EEPROM.update(18, SmsTempCheckTime);
  TempCheckTime = SmsTempCheckTime;
  Serial.print( F ("TempCheckTime is: "));
  Serial.println(EEPROM.read(18));
  lineprint();
  LedOK();
}

// ----- Обработка команд
void MasterCommands() {
  if (ConsoleCommand == 1) Serial.println(val);
  val.replace("\r", "");
  val.replace("\n", "");
  val.toLowerCase();
  if (SuperUser == 1) {
    if ((val.indexOf( F ("addphone")) > -1)) {
      addphone(val, 1);
    }
    if ((val.indexOf( F ("deletephone")) > -1)) {
      deletephone(val);
    }
    if ((val.indexOf( F ("balancenum")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsBalanceNum = SmsText.substring(0, String(SmsText.indexOf("l")).toInt());
      String SmsBalanceLen = val.substring(String(val.lastIndexOf("l")).toInt() + 1);
      eepromupdatebalancenum(SmsBalanceNum, SmsBalanceLen.toInt());
    }
    if ((val.indexOf( F ("watchpowertime")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsWatchPowerTime = SmsText;
      if (SmsWatchPowerTime.toInt() < 0 || SmsWatchPowerTime.toInt() > 255 ) SmsWatchPowerTime = String(WatchPowerTime);
      eepromupdatewatchpowertime(SmsWatchPowerTime.toInt());
    }
    if ((val.indexOf( F ("ringtime")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsRingTime = SmsText;
      if (SmsRingTime.toInt() < 10 || SmsRingTime.toInt() > 255 ) SmsRingTime = String(RingTime);
      eepromupdateringtime(SmsRingTime.toInt());
    }
    if ((val.indexOf( F ("lowtemp")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsLowTemp = SmsText;
      if (SmsLowTemp.toInt() < -50 || SmsLowTemp.toInt() > 125 || SmsLowTemp.toInt() >= HighTemperature) SmsLowTemp = String(LowTemperature);
      eepromupdatelowtemp(SmsLowTemp.toInt());
    }
    if ((val.indexOf( F ("hightemp")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsHighTemp = SmsText;
      if (SmsHighTemp.toInt() < -50 || SmsHighTemp.toInt() > 125 || SmsHighTemp.toInt() <= LowTemperature) SmsHighTemp = String(HighTemperature);
      eepromupdatehightemp(SmsHighTemp.toInt());
    }
    if ((val.indexOf( F ("netchecktime")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsNetCheckTime = SmsText;
      if (SmsNetCheckTime.toInt() < 0 || SmsNetCheckTime.toInt() > 60 ) SmsNetCheckTime = String(NetCheckTime);
      eepromupdateNetCheckTime(SmsNetCheckTime.toInt());
    }
    if ((val.indexOf( F ("tempchecktime")) > -1)) {
      String SmsText = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      String SmsTempCheckTime = SmsText;
      if (SmsTempCheckTime.toInt() < 1 || SmsTempCheckTime.toInt() > 60 ) SmsTempCheckTime = String(TempCheckTime);
      eepromupdateTempCheckTime(SmsTempCheckTime.toInt());
    }
    if ((val.indexOf( F ("fullreset")) > -1)) {
      EEPROM.update(0, 255);
      eepromconfig();
      EEPROM.update(99, 255);
      eepromphonememory();
      EEPROM.update(30 + BalanceSettingsOffset, 255);
      eeprombalancenum();
      LedOK();
      Reset();
    }
    if ((val.indexOf( F ("resetphone")) > -1)) {
      EEPROM.update(99, 255);
      eepromphonememory();
      LedOK();
      Reset();
    }
    if ((val.indexOf( F ("resetconfig")) > -1)) {
      EEPROM.update(0, 255);
      eepromconfig();
      LedOK();
      Reset();
    }
    if ((val.indexOf( F ("watchpowero")) > -1)) {
      SendSmsText =  F ("Watch the power O");
      if ((val.indexOf( F ("n1")) > -1)) {
        WatchPower = 1;
        Serial.print( F ("1"));
        SendSmsText +=   "N";
      }
      else if ((val.indexOf( F ("n2")) > -1)) {
        WatchPower = 2;
        Serial.print( F ("2"));
        SendSmsText +=   "N";
      }
      else {
        WatchPower = 0;
        Serial.print( F ("3"));
        SendSmsText +=  "FF";
      }
      EEPROM.update(19, WatchPower);
      if (SMSCommand == 1) sms("+" + RingPhone);
    }
    if ((val.indexOf( F ("editmainphone")) > -1)) {
      addphone(val, 2);
    }
    if ((val.indexOf( F ("listconfig")) > -1)) {
      eepromconfig();
      eeprombalancenum();
    }
    if ((val.indexOf( F ("memtest")) > -1)) {
      memtest();
    }
    if ((val.indexOf( F ("listphone")) > -1)) {
      listphone();
    }
    if ((val.indexOf( F ("reboot")) > -1)) {
      delay(500);
      Reset();
    }
    if ((val.indexOf( F ("modemid")) > -1)) {
      String Text = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
      if (Text.toInt() < 0 || Text.toInt() > 10 ) Text = ModemID;
      setupmodemid(Text.toInt());
    }
    SuperUser = 0;
  }
  if ((val.indexOf( F ("info")) > -1)) {
    GetTemperature();
    SendSmsText = (F("Current temperature: "));
    SendSmsText += (CurrentTemperature);
    if (CurrentTemperature < -55 || CurrentTemperature > 125) SendSmsText += (F("\nSensor error!"));
    SendSmsText += (F("\nLow: "));
    SendSmsText += (LowTemperature);
    SendSmsText += (F("\nHigh: "));
    SendSmsText += (HighTemperature);
    SendSmsText += (F("\nTempCheckTime: "));
    SendSmsText += (TempCheckTime);
    SendSmsText += (F(" min"));
    SendSmsText += (F("\n"));
    SendSmsText += LastEvent;
    if (SMSCommand == 1) sms("+" + RingPhone);
    if (ConsoleCommand == 1) Serial.println(SendSmsText);
  }
  if ((val.indexOf( F ("money")) > -1)) {
    balance(ConsoleCommand);
  }
  if ((val.indexOf( F ("netstatus")) > -1)) {
    if (NetCheckQueue == 0) {
      netlevel(ConsoleCommand);
      NetCheckQueue = 1;
    }
    else {
      netreg(ConsoleCommand);
      NetCheckQueue = 0;
    }
  }
  if ((val.indexOf( F ("guardo")) > -1)) {
    if ((val.indexOf("n") > -1)) {
      GuardOn(ConsoleCommand);
    }
    else {
      GuardOff(ConsoleCommand);
    }
  }
  if ((val.indexOf( F ("testo")) > -1)) {
    if ((val.indexOf("n") > -1)) {
      DevTestOn = 1;
    }
    else {
      DevTestOn = 0;
    }
    EEPROM.update(12, DevTestOn);
    InitModem();
  }
  if ((val.indexOf( F ("ledo")) > -1)) {
    if ((val.indexOf("n") > -1)) {
      LedOn = 1;
    }
    else {
      LedOn = 0;
    }
    EEPROM.update(13, LedOn);
  }
  if ((val.indexOf( F ("clearsms")) > -1)) {
    clearsms();
  }
  if ((val.indexOf( F ("rebootmodem")) > -1)) {
    RebootModem();
  }
  SMSCommand = 0;
  ConsoleCommand = 0;
}

// ----- Обработка GuardOff
void GuardOff(uint8_t from_console) {
  guard = 0;
  EEPROM.update(14, guard); //!!! Guard Off. вынести
  LastEvent = F ("Guard OFF.");
  if (from_console == 1) ConsoleEvent = 1;
  guardeventprepare();
  Serial.println(LastEvent);
  SendSmsText = LastEvent;
  delay(1000);
  for (uint8_t i = 0; i < 10; i++) {
    if (EEPROM.read(data[i].addr + IAlarmOffset) == 1) {
      delay(1000);
      sms("+" + String(EEPROM.get(data[i].addr, PhoneNum)));
      delay(1000);
    }
  }
  ConsoleEvent = 0;
}

// ----- Обработка GuardOn
void GuardOn(uint8_t from_console) {
  guard = 1;
  AlarmTemperatureFlag = 0;
  EEPROM.update(14, guard);
  LastEvent = F ("Guard ON.");
  if (from_console == 1) ConsoleEvent = 1;
  guardeventprepare();
  Serial.println(LastEvent);
  SendSmsText = LastEvent;
  delay(1000);
  for (uint8_t i = 0; i < 10; i++) {
    if (EEPROM.read(data[i].addr + IAlarmOffset) == 1) {
      delay(1000);
      sms("+" + String(EEPROM.get(data[i].addr, PhoneNum)));
      delay(1000);
    }
  }
  ConsoleEvent = 0;
}

void guardeventprepare() {
  if (ConsoleEvent == 1) LastEvent += F (" Console");
  else {
    LastEvent += F (" Phone +");
    LastEvent += RingPhone;
  }
}

// ----- Изменение длительности тревожного звонка
void eepromupdateringtime(uint8_t SmsRingTime) {
  EEPROM.update(24, SmsRingTime);
  RingTime = SmsRingTime;
  Serial.print( F ("RingTime is: "));
  Serial.println(EEPROM.read(24));
  lineprint();
  LedOK();
}

// ----- Изменение нижней границы температуры
void eepromupdatelowtemp(uint8_t SmsLowTemp) {
  EEPROM.update(20, SmsLowTemp);
  LowTemperature = SmsLowTemp;
  Serial.print( F ("LowTemp is: "));
  Serial.println(EEPROM.read(20));
  lineprint();
  LedOK();
}

// ----- Изменение нижней границы температуры
void eepromupdatehightemp(uint8_t SmsHighTemp) {
  EEPROM.update(21, SmsHighTemp);
  HighTemperature = SmsHighTemp;
  Serial.print( F ("HighTemp is: "));
  Serial.println(EEPROM.read(21));
  lineprint();
  LedOK();
}
// ----- Инициализация номера запроса баланса
void eeprombalancenum() {
  if (EEPROM.read(30 + BalanceSettingsOffset) != 1) {
    Serial.println( F ("Delete balance data."));
    for (uint8_t i = 30 ; i < (BalanceSettingsOffset + 1) ; i++) {
      EEPROM.update(i, 255);
      delay(10);
    }
    Serial.println( F ("Load default balance data."));
    lineprint();
    BalanceNumber = "#100#";
    BalanceStringLen = 22;
    BalanceNumber.toCharArray(BalanceNum, 17);
    EEPROM.put(30, BalanceNum);
    EEPROM.update(30 + BalanceLenOffset, BalanceStringLen);
    EEPROM.update(30 + BalanceSettingsOffset, 1);
  }
  if (EEPROM.read(30 + BalanceSettingsOffset) == 1) {
    BalanceNumber = EEPROM.get(30, BalanceNum);
    BalanceStringLen = EEPROM.read(30 + BalanceLenOffset);
    Serial.print( F ("Balance number: "));
    Serial.println(BalanceNumber);
    Serial.print( F ("Length to return: "));
    Serial.println(BalanceStringLen);
    lineprint();
  }
}

// ----- Изменение номера запроса баланса
void eepromupdatebalancenum(String BalanceNumber, uint8_t BalanceStringLen) {
  BalanceNumber.toCharArray(BalanceNum, 17);
  EEPROM.put(30, BalanceNum);
  EEPROM.update(30 + BalanceLenOffset, BalanceStringLen);
  EEPROM.update(30 + BalanceSettingsOffset, 1);
  eeprombalancenum();
  Serial.println();
  lineprint();
  LedOK();
}

// ----- Изменение времени WatchPowerTime
void eepromupdatewatchpowertime(uint8_t SmsWatchPowerTime) {
  EEPROM.update(23, SmsWatchPowerTime);
  WatchPowerTime = SmsWatchPowerTime;
  Serial.print( F ("WatchPowerTime is: "));
  Serial.println(EEPROM.read(23));
  lineprint();
  LedOK();
}

// ----- Инициализация основного номера
void eepromfirstphone(String RingPhoneNumber) {
  if (String(EEPROM.get(data[0].addr, PhoneNum)) == "") {
    RingPhoneNumber.toCharArray(PhoneCharArray, 17);
    EEPROM.put(data[0].addr, PhoneCharArray);
    //EEPROM.update(data[0].addr + ManagementOffset, 1);
    EEPROM.update(data[0].addr + SAlarmOffset, 1);
    EEPROM.update(data[0].addr + RAlarmOffset, 1);
    EEPROM.update(data[0].addr + PAlarmOffset, 1);
    EEPROM.update(data[0].addr + IAlarmOffset, 1);
    Serial.print( F ("Primary phone: +"));
    Serial.println(RingPhoneNumber);
    lineprint();
    LedOK();
    endcall();
    delay(1000);
    Reset();
  }
}

// ----- Удаление номера
void eepromdeletephone(String DeletePhoneNumber) {
  uint8_t error = 1;
  for (uint8_t i = 1 ; i < 10 ; i++) {
    if (String(EEPROM.get(data[i].addr, PhoneNum)) == DeletePhoneNumber) {
      char PhoneCharArray[17] = "";
      EEPROM.put(data[i].addr, PhoneCharArray);
      EEPROM.update(data[i].addr + SAlarmOffset, 0);
      EEPROM.update(data[i].addr + RAlarmOffset, 0);
      EEPROM.update(data[i].addr + PAlarmOffset, 0);
      EEPROM.update(data[i].addr + IAlarmOffset, 0);
      Serial.print( F ("Delete phone: +"));
      Serial.println(DeletePhoneNumber);
      lineprint();
      listphone();
      LedOK();
      error = 0;
    }
  }
  if (error == 1) {
    Serial.print( F ("Error deleting phone: +"));
    Serial.println(DeletePhoneNumber);
    lineprint();
    LedERROR();
  }
}

// ----- Добавление номера
//void eepromaddphone(String AddPhoneNumber, uint8_t Cell, uint8_t M, uint8_t S, uint8_t R, uint8_t P, uint8_t I ) {
void eepromaddphone(String AddPhoneNumber, uint8_t Cell, uint8_t S, uint8_t R, uint8_t P, uint8_t I ) {
  uint8_t createnumber = 1;
  Cell -= 1;

  if (String(EEPROM.get(data[0].addr, PhoneNum)) == "") {
    Serial.println( F ("Error, not set main phone."));
    lineprint();
    LedERROR();
    createnumber = 0;
  }
  if (Cell <= 0 || Cell > 10)  {
    Serial.println( F ("Error, wrong cell."));
    lineprint();
    LedERROR();
    createnumber = 0;
  }
  if (createnumber == 1) {
    //Защита задвоения номеров, если необходимо изменить права, нужно перезписать номер на произвольный, и заново ввеси новый с нужными правами.
    for (uint8_t i = 0 ; i < 10 ; i++) {
      if (String(EEPROM.get(data[i].addr, PhoneNum)) == AddPhoneNumber) {
        Serial.print( F ("Phone already exists in the cell: "));
        Serial.println(i + 1);
        lineprint();
        if (i != Cell) {
          S = 0;
          P = 0;
          I = 0;
        }
      }
    }
  }
  if (createnumber == 1) {
    AddPhoneNumber.toCharArray(PhoneCharArray, 17);
    EEPROM.put(data[Cell].addr, PhoneCharArray);
    EEPROM.update(data[Cell].addr + SAlarmOffset, S);
    EEPROM.update(data[Cell].addr + RAlarmOffset, R);
    EEPROM.update(data[Cell].addr + PAlarmOffset, P);
    EEPROM.update(data[Cell].addr + IAlarmOffset, I);
    Serial.println( F ("Phone added."));
    lineprint();
    LedOK();
  }
}

// ----- Инициализация памяти номеров
void eepromphonememory() {
  if (EEPROM.read(99) != 1) {        //Сюда код начальной инициализации памяти телефонов.
    Serial.println( F ("Deleting phone data."));
    for (unsigned short int i = 100 ; i < 400 ; i++) {
      EEPROM.update(i, 255);
      delay(10);
    }
    Serial.println( F ("Load default phone data."));
    lineprint();
    for ( uint8_t i = 0; i < 10; i++ ) {
      EEPROM.put(data[i].addr, data[i].PhoneNum);
      EEPROM.update(data[i].addr + SAlarmOffset, data[i].SAlarm);
      EEPROM.update(data[i].addr + RAlarmOffset, data[i].RAlarm);
      EEPROM.update(data[i].addr + PAlarmOffset, data[i].PAlarm);
      EEPROM.update(data[i].addr + IAlarmOffset, data[i].IAlarm);
    }
    EEPROM.update(99, 1);
  }
  //Ниже только для отладки, пока не вижу смысла читать данные в массив.
  if (EEPROM.read(99) == 1) {
    //Сюда код чтения из памяти телефонных номеров.
    listphone();
  }
}

// ----- Вывод в консоль номеров
void listphone() {
  Serial.println ( F ("Read array phone:"));
  Serial.print (F("№"));
  Serial.print (Marker + F("Phone"));
  Serial.print (Marker + F("s"));
  Serial.print (Marker + F("r"));
  Serial.print (Marker + F("p"));
  Serial.println (Marker + F("i"));

  for ( uint8_t i = 0; i < 10; i++ ) {
    EEPROM.get(data[i].addr, PhoneNum);
    Serial.print((i + 1) + Marker + PhoneNum + Marker + EEPROM.read(data[i].addr + SAlarmOffset));
    Serial.println(Marker + EEPROM.read(data[i].addr + RAlarmOffset) + Marker + EEPROM.read(data[i].addr + PAlarmOffset) + Marker + EEPROM.read(data[i].addr + IAlarmOffset));
  }
  lineprint();
}

// ----- Инициализация конфигурации устройства
void eepromconfig() {
  if (EEPROM.read(0) != 1) {
    Serial.println( F ("Deleting eepromconfig data."));
    for (uint8_t i = 0 ; i < 30 ; i++) {
      EEPROM.update(i, 255);
      delay(10);
    }
    Serial.println( F ("Load default eepromconfig data."));
    lineprint();
    EEPROM.update(12, 0);  //DevTestOn = 0;
    EEPROM.update(13, 1);  //LedOn = 1;
    EEPROM.update(14, 1); //Мониторинг температуры: 1 - включен, 0 - выключен. Если включен, будут отправляться смс при переходе диаппазонов датчика
    EEPROM.update(15, 0); //Проверка регистрации модема в сети оператора. Устанавливается в минутах, не менее 1 и не более 60. Если 0 - функция отключена
    EEPROM.update(18, 10); //Интервал опроса датчика температуры в минутах
    EEPROM.update(19, 1); //WatchExtPower мониторинг сети питания: 1 - включен, 0 - выключен.
    EEPROM.update(20, 10); //Нижний предел температуры
    EEPROM.update(21, 70); //Верхний предел температуры
    EEPROM.update(23, 0); //WatchPowerTime Задержка времени перед отправкой смс при отключении сети питания
    EEPROM.update(24, 40); //RingTime Длительность тревожного голосового вызова.
    EEPROM.update(25, 0); //ModemID. 0 - Автоопределение модема, 1 - M590, 2 - SIM800l, 3 - A6_Mini.
    EEPROM.update(0, 1);
  }
  if (EEPROM.read(0) == 1) {
    Serial.println( F ("Read eepromconfig:"));
    lineprint();

    RingTime = EEPROM.read(24);
    Serial.print( F ("RingTime: "));
    Serial.println(RingTime);

    DevTestOn = EEPROM.read(12);  //DevTestOn = 0;
    Serial.print( F ("Test: "));//DevTestOn
    Serial.println(DevTestOn);

    LedOn = EEPROM.read(13);  //LedOn = 1;
    Serial.print( F ("Led: "));
    Serial.println(LedOn);

    guard = EEPROM.read(14); //Мониторинг температуры: 1 - включен, 0 - выключен. Если включен, будут отправляться смс при переходе диаппазонов датчика
    Serial.print( F ("Guard: "));
    Serial.println(guard);

    NetCheckTime = EEPROM.read(15); //NetCheckTime в минутах, не менее 1 и не более 60.
    Serial.print( F ("NetCheckTime: "));
    Serial.println(NetCheckTime);

    TempCheckTime = EEPROM.read(18); //TempCheckTime в минутах, не менее 1 и не более 60.
    Serial.print( F ("TempCheckTime: "));
    Serial.println(TempCheckTime);

    LowTemperature = EEPROM.read(20); //Нижняя граница температуры до -50
    Serial.print( F ("LowTemp: "));
    Serial.println(LowTemperature);

    HighTemperature = EEPROM.read(21); //Верхняя граница температуры до 125
    Serial.print( F ("HighTemp: "));
    Serial.println(HighTemperature);

    WatchPower = EEPROM.read(19);
    WatchPowerTime = EEPROM.read(23);
    Serial.println( F ("WatchPower"));
    Serial.print( F (" Mode: "));
    Serial.println(WatchPower);
    Serial.print( F (" Time: "));
    Serial.println(WatchPowerTime);

    ModemID = EEPROM.read(25);
    Serial.print( F ("ModemID: "));
    if (ModemID == 0) Serial.println(F("Autodetect"));
    else
      Serial.println(ModemID);
    lineprint();
  }
}

// ----- Инициализация модема
void InitModem() {
  delay(2000);  //Время на инициализацию модуля
  Serial.begin(9600);  //Скорость порта
  mySerial.begin(9600);
  for ( uint8_t i = 0; i < 10; i++ ) {
    mySerial.println( F ("AT"));
    delay(200);
  }

  mySerial.println( F ("ATI"));
  delay(10);
  if (mySerial.available()) {  //Если GSM модуль что-то послал нам, то
    while (mySerial.available()) {  //сохраняем входную строку в переменную val
      ch = mySerial.read();
      val += char(ch);
      delay(20);
    }
    if (ModemID == 0) {
      if (val.indexOf( F ("M590")) > -1) ModemID = 1;
      if (val.indexOf( F ("SIM800")) > -1) ModemID = 2;
      if (val.indexOf( F ("A6")) > -1) ModemID = 3;
      Serial.print(F("Detected ModemID: "));
      Serial.println(ModemID);
    }
    val = "";
  }

  mySerial.println( F ("AT+CLIP=1")); //Включаем АОН
  delay(100);
  mySerial.println( F ("AT+CMGF=1")); //Режим кодировки СМС - обычный (для англ.)
  delay(100);
  mySerial.println( F ("AT+CSCS=\"GSM\"")); //Режим кодировки текста
  delay(100);
  mySerial.println( F ("AT+CNMI=2,2")); //Отображение смс в терминале сразу после приема (без этого сообщения молча падают в память)
  delay(100);
  if (DevTestOn == 1) {
    mySerial.println( F ("ATE1"));
  } else {
    mySerial.println( F ("ATE0"));
  }
}

// ----- Новая строка в консоли
void lineprint() {
  Serial.println();
}

// ----- Завершение вызова
void endcall() {
  delay(2000);
  mySerial.println( F ("AT+CHUP"));
  EndCallFlag = 0;
  //delay(2000);
}

// ----- Проверка внешнего питания
void ExternalPowerMon() {
  //Serial.println(WatchExtPower);
  if ((WatchPower == 1 && guard == 1) || WatchPower == 2) {
    //Считываем значения с входа питания
    ExtPowerState = digitalRead(ExtPowerPin);
    if (ExtPowerState == LOW && ExtPowerFlag == 0) {
      ExtPowerFlag = 1;
      delay(100);
      SendSmsText = F ("Power supply disconnected ");
      PrepareWatchPowerTimeAlarm(2);
    }
    if (ExtPowerState == HIGH && ExtPowerFlag == 1) {
      ActivateWatchPowerTime = 0;
      ExtPowerFlag = 0;
      delay(100);
      if (WatchPowerTimeSmsSended == 1) {
        SendSmsText = F ("Power supply restored");
        SmsAlarm(2);
        WatchPowerTimeSmsSended = 0;
      }
    }
    if (ExtPowerState == HIGH && ExtPowerFlag == 2) {
      ExtPowerFlag = 0;
      delay(100);
    }
  }
}

// ----- Проверка номера SuperUser
uint8_t eepromchecksuphone(String CheckPhoneSUNumber) {
  if (String(EEPROM.get(data[0].addr, PhoneNum)) == CheckPhoneSUNumber) return 1;
  return 0;
}

// ----- Проверка номера
uint8_t eepromcheckphone(String CheckPhoneMNumber) {
  for (uint8_t i = 0; i < 10; i++) {
    if (String(EEPROM.get(data[i].addr, PhoneNum)) == CheckPhoneMNumber) {
      RingPhone = CheckPhoneMNumber;
      return 1;
      break;
    }
  }
  return 0;
}

// ----- Обработка команд добавления телефона
void addphone (String val, uint8_t from_event) {
  String Text = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
  //uint8_t M = 0;
  uint8_t S = 0;
  uint8_t R = 0;
  uint8_t P = 0;
  uint8_t I = 0;
  //if (Text.indexOf("m") > -1) M = 1;
  if (Text.indexOf("s") > -1) S = 1;
  if (Text.indexOf("r") > -1) R = 1;
  if (Text.indexOf("p") > -1) P = 1;
  if (Text.indexOf("i") > -1) I = 1;
  //Text.replace("m", "");
  Text.replace("s", "");
  Text.replace("r", "");
  Text.replace("p", "");
  Text.replace("i", "");
  Text.replace("(sp)", "");
  String Cell = Text.substring(0, String(Text.indexOf("+")).toInt());
  String AddPhoneNumber = Text.substring(String(Text.lastIndexOf("+")).toInt() + 1);
  if (from_event == 1) eepromaddphone(AddPhoneNumber, Cell.toInt(), S, R, P, I);
  if (from_event == 2) editmainphone(S, R, P, I);
}

// ----- Обработка команд удаления телефона
void deletephone (String val) {
  String Text = val.substring(String(val.lastIndexOf(":")).toInt() + 1);
  Text.replace( "(sp)", "");
  String SmsDeletePhoneNumber = Text.substring(String(Text.lastIndexOf("+")).toInt() + 1);
  eepromdeletephone(SmsDeletePhoneNumber);
}

// ----- Изменение параметров основного номера
void editmainphone (uint8_t S, uint8_t R, uint8_t P, uint8_t I) {
  EEPROM.update(data[0].addr + SAlarmOffset, S);
  EEPROM.update(data[0].addr + RAlarmOffset, R);
  EEPROM.update(data[0].addr + PAlarmOffset, P);
  EEPROM.update(data[0].addr + IAlarmOffset, I);
  Serial.println( F ("Phone settings changed."));
  lineprint();
  LedOK();
  //listphone();
}

// ----- Тестирование памяти EEPROM
void memtest() {
  uint8_t x;
  x = random(2, 255);
  Serial.print( F ("EEPROM: "));
  Serial.print(EEPROM.length());
  Serial.println( F ("B"));
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, x);
    delay(5);
  }
  uint16_t y;
  y = 0;
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
    if (EEPROM.read(i) != x) {
      //lineprint();
      Serial.print( F ("Failed cell: "));
      Serial.println(i + Marker + EEPROM.read(i));
      y++;
    }
    delay(5);
  }
  Serial.print( F ("Test "));
  if (y == 0) {
    Serial.println( F ("passed."));
    lineprint();
    delay(500);
    Reset();
  }
  if (y != 0) Serial.println( F ("failed."));
}

// ----- Принудительная установка ID модема.
void setupmodemid (uint8_t id) {
  Serial.print( F ("Previous ID: "));
  Serial.println(ModemID);
  EEPROM.update(25, id);
  ModemID = id;
  Serial.print( F ("Current ID: "));
  Serial.print(ModemID);
  Serial.print( F (". Modem: "));
  if (id == 0) {
    Serial.println( F ("Autodetect."));
    Serial.println( F ("Reboot."));
    delay(500);
    Reset();
  } else if (id == 1) {
    Serial.println( F ("M590e."));
  } else if (id == 2) {
    Serial.println( F ("SIM800L."));
  } else if (id == 3) {
    Serial.println( F ("A6."));
  } else {
    Serial.println( F ("Not use."));
  }
  lineprint();
}

// ----- Команды запроса баланса
void balance(uint8_t from_console) {
  if (from_console == 1) ConsoleEvent = 1;
  //delay(1000);
  mySerial.print(F("AT+CUSD=1,\""));
  mySerial.print(BalanceNumber);
  mySerial.println("\",15");
}

// ----- Запрос уровня сигнала сети
void netlevel(uint8_t from_console) {
  if (from_console == 1) ConsoleEvent = 1;
  //delay(1000);
  mySerial.println("AT+CSQ");
  delay(1000);
}

// ----- Запрос регистрации в сети оператора
void netreg(uint8_t from_console) {
  if (from_console == 1) ConsoleEvent = 1;
  //delay(1000);
  mySerial.println("AT+CREG?");
  delay(1000);
}

// ----- Удаление SMS
void clearsms() {
  mySerial.print ( F ("AT+CMGD="));
  if (ModemID == 1) mySerial.println( F ("0,4"));
  if (ModemID == 2 || ModemID == 3) mySerial.println( F ("4"));
  //delay(2000);
  Serial.println( F ("SMS removed"));
  LedOK();
}

// ----- Световая индикация успешного завершения команды
void LedOK() {
  digitalWrite(BLed, HIGH);
  delay(100);
  digitalWrite(BLed, LOW);
  delay(100);
  digitalWrite(BLed, HIGH);
}

// ----- Световая индикация завершения команды с ошибкой
void LedERROR() {
  for (uint8_t i = 0 ; i < 2 ; i++) {
    LedOK();
  }
}

void RebootModem() {
  digitalWrite(ModemResetPin, LOW);
  delay(500);
  digitalWrite(ModemResetPin, HIGH);
  delay(10000);
  Serial.println(F("RebootModem"));
  InitModem();

}

void PrepareWatchPowerTimeAlarm(uint8_t from_event) {
  if (ActivateWatchPowerTime == 0) previousWatchPowerTimeMillis = millis();
  ActivateWatchPowerTime = 1;
  //  PrepareAlarmText = text;
  //  PrepareAlarmSMSForce = sms_force;
  PrepareAlarmFromEvent = from_event;
}

// ----- Отправка тревожной смс и инициализация тревожного звонка
void SmsAlarm(uint8_t from_event) {
  if (guard == 1) {
    //    Serial.println(text);
    for (uint8_t i = 0; i < 10; i++) {
      if (from_event == 1) {
        if (EEPROM.read(data[i].addr + SAlarmOffset) == 1) {
          sms("+" + String(EEPROM.get(data[i].addr, PhoneNum)));
          delay(1000);
        }
      }
      if (from_event == 2) {
        if (EEPROM.read(data[i].addr + PAlarmOffset) == 1) {
          sms("+" + String(EEPROM.get(data[i].addr, PhoneNum)));
          delay(1000);
        }
      }
    }
    if (from_event == 1) {
      CallFlag = 1;
      Call = 1;
    }
  }
}

void DelayPowerAlarm() {
  if (ActivateWatchPowerTime == 1) {
    unsigned long currentWatchPowerTimeMillis = millis();
    if ((currentWatchPowerTimeMillis - previousWatchPowerTimeMillis) / 1000 > WatchPowerTime * 60) {
      if (WatchPowerTime != 0) {
        SendSmsText += WatchPowerTime;
        SendSmsText += F(" minutes ago");
      }
      //SendSmsText = PrepareAlarmText;
      //SmsAlarm(PrepareAlarmSMSForce, PrepareAlarmFromEvent);
      SmsAlarm(PrepareAlarmFromEvent);
      WatchPowerTimeSmsSended = 1;
      ActivateWatchPowerTime = 0;
    }
  }
}

// ----- Обработка тревожных звонков
void AlarmCall() {
  if (EndCallFlag == 1) endcall();
  if (Call == 1) {
    if (EEPROM.read(data[CallNum].addr + RAlarmOffset) == 1) {
      previousCallTimeMillis =  millis();
      delay(5000);
      Serial.print(F("Alarm Ring: +"));
      Serial.println(EEPROM.get(data[CallNum].addr, PhoneNum));
      mySerial.print( F ("ATD+"));
      mySerial.print(EEPROM.get(data[CallNum].addr, PhoneNum));
      mySerial.println( F (";"));
      Call = 0;
    }
  }

  if (CallFlag == 1) {
    unsigned long currentCallTimeMillis = millis();
    if ((currentCallTimeMillis - previousCallTimeMillis) / 1000 > RingTime) {
      EndCallFlag = 1;
      CallNum++;
      Call = 1;
    }
  }

  if ((CallNum > 10) || (guard == 0 && CallFlag == 1)) {
    CallFlag = 0;
    EndCallFlag = 1;
    CallNum = 0;
    Call = 0;
  }
}

// ----- Отправка SMS - сообщений
void sms(String phone) {  //Процедура отправки СМС
  delay(1000);
  mySerial.print( F ("AT+CMGS=\""));
  mySerial.print(phone);
  mySerial.println( F ("\""));
  delay(500);
  mySerial.print(SendSmsText);
  delay(500);
  mySerial.print((char)26);
  delay(500);
  Serial.print("Sent! ");
  Serial.print(phone);
  Serial.print(Marker);
  Serial.println(SendSmsText);
  delay(5000);
}


