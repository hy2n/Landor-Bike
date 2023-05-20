/*
 .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------.  .----------------. 
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |   ______     | || |  _______     | || |     ____     | || |     _____    | || |  _________   | || |     ______   | || |  _________   | |
| |  |_   __ \   | || | |_   __ \    | || |   .'    `.   | || |    |_   _|   | || | |_   ___  |  | || |   .' ___  |  | || | |  _   _  |  | |
| |    | |__) |  | || |   | |__) |   | || |  /  .--.  \  | || |      | |     | || |   | |_  \_|  | || |  / .'   \_|  | || | |_/ | | \_|  | |
| |    |  ___/   | || |   |  __ /    | || |  | |    | |  | || |   _  | |     | || |   |  _|  _   | || |  | |         | || |     | |      | |
| |   _| |_      | || |  _| |  \ \_  | || |  \  `--'  /  | || |  | |_' |     | || |  _| |___/ |  | || |  \ `.___.'\  | || |    _| |_     | |
| |  |_____|     | || | |____| |___| | || |   `.____.'   | || |  `.___.'     | || | |_________|  | || |   `._____.'  | || |   |_____|    | |
| |              | || |              | || |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 

 .----------------.  .----------------.  .-----------------. .----------------.  .----------------.  .----------------. 
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |   _____      | || |      __      | || | ____  _____  | || |  ________    | || |     ____     | || |  _______     | |
| |  |_   _|     | || |     /  \     | || ||_   \|_   _| | || | |_   ___ `.  | || |   .'    `.   | || | |_   __ \    | |
| |    | |       | || |    / /\ \    | || |  |   \ | |   | || |   | |   `. \ | || |  /  .--.  \  | || |   | |__) |   | |
| |    | |   _   | || |   / ____ \   | || |  | |\ \| |   | || |   | |    | | | || |  | |    | |  | || |   |  __ /    | |
| |   _| |__/ |  | || | _/ /    \ \_ | || | _| |_\   |_  | || |  _| |___.' / | || |  \  `--'  /  | || |  _| |  \ \_  | |
| |  |________|  | || ||____|  |____|| || ||_____|\____| | || | |________.'  | || |   `.____.'   | || | |____| |___| | |
| |              | || |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 

 - PROJECT LANDOR by DONGHYUN
 - 2023 DONGHYUN SOFTWARE DEVLOPERS.Co.,LTD all rights reserved.

 - LANDOR.ino

 - Software Verson : 3.2 Beta
*/

/*
 - STATICS CONNECTION
 
 #릴레이
 33 - 자전거 버튼 배선 / 34 - 보안경보 배선 / 32 - UX배선

 #온습도
 Digital 11 연결

 #전압센서
 Analog 14 연결

 #GPS
 A12, A13 연결
*/

#include <math.h>
#include "DHT.h"
#define DHTTYPE DHT11
DHT dht(11, DHTTYPE);
float t, h, hic;
int temp_old = 20;
int newData = 0;

//SmartAllet
int SmartAllet_lowbat = 0;
int SmartAllet_lowbat_more = 0;
int SmartAllet_lowbat30 = 0;
int SmartAllet_SecurityAllet = 0;
int SmartAllet_toolowtemp = 0;
int SmartAllet_toohightemp = 0;
int Voice = 0;
int gyroerror = 0;

//GYRO MODULE SETUP
#include <SparkFun_ADXL345.h>
ADXL345 adxl = ADXL345();
int a, b, c;                       //계산 임시 변수
int x, y, z;                       //측정 임시 변수
int motionlevel[3] = { 0, 0, 0 };  //경고레벨 (0 , 1 , 2 , 3) 감지
int lock_x;                        //현 값
int lock_y;
int lock_z;
int lock_x_m = 0;  //기존 값
int lock_y_m = 0;
int lock_z_m = 0;
int lock_x_r = 0;  //차이값
int lock_y_r = 0;
int lock_z_r = 0;
int motion_warn = 0;  //보안모션경고단계
int motion_warn_r = 0;
int speaker = 0;    //스피커 경고횟수(버그예방위해 초기1번만작동)
int skip_load = 0;  //값 무시횟수


//ADAS
int adas = 0;  //OFF

//ADAS 기준
//1.기본경고(일반)
//2.긴급경고

//기본경고 - 전후방 물체감지,감지시 기준에따라 작동
//긴급경고 - 속도 15kmh 이상일때의 전후방 장애물 감지 or 전후방센서의 너무 가까운 감지

int adas_current_level = 0;
int adas_info_crt = 0;    //ADAS센싱정보상태 (오류값 제거)
int adas_Current_XA = 0;  //ADAS센서 위험강도 total
int adas_Currt_IL = 0;    //ADAS현위험감도
int adas_warning = 0;     //기본경고일경우 경고음을 1번만 재생(기본으로 돌아갈시 초기화)

//NFC MODULE SETUP
#include <EasyMFRC522.h>
#include <RfidDictionaryView.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#define SS_PIN 53s
#define RST_PIN 5
MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
byte nuidPICC[4];

//GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = A12, TXPin = A13;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(A12, A13);
//BatteryMeter
float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value = 0;

//SmartBatManager
int bat_pst_current = 100;  //현 배터리 상태
int bat_pst_past1 = 0;      //과거 1값
int bat_pst_past2 = 0;      //과거 2값
int bat_pst_past3 = 0;      //과거 3값
int bat_pst_past4 = 0;      //과거 4값
int bat_pst_past0 = 0;      //과거 0값
int bat_pst_csa = 100;      //배터리 총 평균 %
int bat_save_sc = 0;
int bat_val_torque = 0;
int bat_val_power = 0;

int vsound = 0; //가속 확인
int vsound_stop = 0; //감속 확인

//배터리는 떨어지기만 하지 일반적인 상황에서는 오르지 않기때문에 올리지 않음.

//SmartDistance
int Smartdistance = 50;  //자전거의 최대 주행거리는 50이므로 기본값 50

//Display
SoftwareSerial display(A10, A11);
int speed = 0;

//Speaker
#include <DFMiniMp3.h>
int plays;
int AdditionalSoundCount = 1;
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
DfMp3 dfmp3(Serial3);
class Mp3Notify {
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
    if (source & DfMp3_PlaySources_Sd) {
      Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb) {
      Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash) {
      Serial.print("Flash, ");
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode) {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track) {
    Serial.print("Play finished for #");
    Serial.println(track);
    dfmp3.playMp3FolderTrack(track);  // sd:/mp3/0001.mp3, sd:/mp3/0002.mp3, sd:/mp3/0003.mp3
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
  }
};


//Supersonic Sensor
long duration, distance;
int b1, b2, b3, b4;
int sensor = 0;

void setup() {

  //Serial
  Serial.begin(115200);
  delay(100);
  dht.begin();
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);
  digitalWrite(34, HIGH);  //보안릴레이는 즉시 연결해제 (소리방지)

  //Batterymeter
  pinMode(A14, INPUT);

  pinMode(A13, OUTPUT);
  pinMode(A12, INPUT);
  //NFC ENGINE
  SPI.begin();                    // SPI통신 시작
  rfid.PCD_Init();                // NFC보드 초기화
  for (byte i = 0; i < 6; i++) {  //NFC ID 값 초기화
    key.keyByte[i] = 0xFF;
  }

  //Gyro ENGINE
  adxl.powerOn();           //자이로 센서 설정
  adxl.setRangeSetting(8);  //센서 민감도

  /* ADAS (Advaced Driver Assist System) Port
  후방 - E:42 T:43 // E:2 T:1
  전방 - E:7 T:6 (LED위) // E:12 T:13 (브레이크밑)
  */
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);  //ECHO
  pinMode(13, OUTPUT);
  pinMode(12, INPUT);  //ECHO
  pinMode(42, OUTPUT);
  pinMode(43, INPUT);  //ECHO
  pinMode(26, OUTPUT);
  pinMode(28, INPUT);  //ECHO
  //Switch
  digitalWrite(33, LOW);
  digitalWrite(32, HIGH);
  digitalWrite(34, HIGH);
  Serial.println(TinyGPSPlus::libraryVersion());
  //Speaker
  dfmp3.begin();
  dfmp3.setVolume(25);
  uint16_t count = dfmp3.getTotalTrackCount(DfMp3_PlaySource_Sd);
  dfmp3.playMp3FolderTrack(8);
  digitalWrite(32, LOW);
  display.begin(4800);
  delay(3000);
  display.print("page bootsc");
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(50);
  while (!rfid.PICC_IsNewCardPresent()) {
    //카드 대기
  }
  dfmp3.playMp3FolderTrack(8);
  display.print("page lock");
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(2000);
  digitalWrite(32, HIGH);
  display.end();
  skip_load = 5;  //1초동안의 값 무시
}

void loop() {
  //자전거 보안코드
  delay(150);
  adxl.readAccel(&x, &y, &z);
  SaveAccl();
  MeasureAccl(lock_x, lock_x_m, 0);  //차이 값 저장
  lock_x_r = c;
  MeasureAccl(lock_y, lock_y_m, 1);
  lock_y_r = c;
  MeasureAccl(lock_z, lock_z_m, 2);
  lock_z_r = c;

  lock_x_m = lock_x;  //차이 비교를 위해 예전 값 저장
  lock_y_m = lock_y;
  lock_z_m = lock_z;
  if (((lock_x == lock_y) == lock_z) && (lock_x > 0)) {
    lock_x = lock_x - 1;
    lock_y = lock_y - 1;
    lock_z = lock_z - 1;
  }
  if ((skip_load > 0)||(gyroerror == 1)) { 
    skip_load = skip_load - 1;
    delay(200);  //skip_load값이 0보다 크다면, 값을 처리하지 않고 넘긴다. (버그방지)
  } else {
    if (motionlevel[0] > 0 || motionlevel[1] > 0 || motionlevel[2] > 0) {
      motion_warn = motion_warn + 1;  //보안경고수준 증가
      motion_warn_r = 1;              //보안경고수준 유지
    }
    if ((motionlevel[0] > 0 || motionlevel[1] > 0 || motionlevel[2] > 0) && (motion_warn > 6)) {  //2단계 8+
      digitalWrite(34, LOW);
      analogWrite(A15, 255);
      delay(1000);
      analogWrite(A15, 0);
      digitalWrite(34, HIGH);
      delay(1000);
      digitalWrite(34, LOW);
      analogWrite(A15, 255);
      delay(1000);
      analogWrite(A15, 0);
      digitalWrite(34, HIGH);
      skip_load = 1;
    } else if ((motionlevel[0] > 0 || motionlevel[1] > 0 || motionlevel[2] > 0) && (motion_warn < 8) && (motion_warn > 4)) {  //1단계 5,6
      SmartAllet_SecurityAllet = 1;                                                                                           //보안시스템 실행여부 기록(보이스아웃팟)
      if (speaker == 0) {
        dfmp3.playMp3FolderTrack(6);
        skip_load = 5;  //값 일시정지 0.2s
        speaker = 1;
      }
      analogWrite(A15, 255);
      delay(200);
      analogWrite(A15, 0);
      delay(2000);
    } else if ((motionlevel[0] > 0 || motionlevel[1] > 0 || motionlevel[2] > 0) && (motion_warn == 1)) {
      skip_load = 10;
    } else {
      if (motion_warn > 0) {
        if (motion_warn_r > (-50)) {  //최근에 보안경고수준이 올랐다면 잠깐동안은 내리지 않기
          motion_warn_r = motion_warn_r - 1;
          delay(100);
        } else {
          motion_warn = motion_warn - 1;
          delay(200);
        }
      } else {  //모션단계 가장 낮음
        speaker = 0;
      }
    }
  }

  //NFC 카드가 인식됄까지 반복
  if (!rfid.PICC_IsNewCardPresent())
    return;  //NFC 없으면 다시 loop
  if (!rfid.PICC_ReadCardSerial())
    return;                                                       //NFC 정보가 비정상이면 다시 loop
    
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);  //NFC 받아오기
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI && piccType != MFRC522::PICC_TYPE_MIFARE_1K && piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    return;  //NFC 오류라면 다시 loop
  }

  c = 2;  //계산변수초기화
  CheckRFID(236, 123, 183);
  CheckRFID(247, 191, 59);  //Watch
  if (c == 1) {

  } else {
    Serial.println(F("[NFC] KEY INCORRECT "));
    NFCmoduleReset();
    return;
  }
  NFCmoduleReset();
  //자전거 버튼선 릴레이 키기
  digitalWrite(33, HIGH);
  digitalWrite(32, LOW);
  temp();
  float hic = t;
  if (hic == 0) {
    hic = temp_old;
  } else {
    temp_old = hic;
  }
  int hic_print = round(hic);
  Serial.print("Temp : ");
  Serial.print(hic_print);
  Serial.println(" *C ");
  //SmartAllet
  if ((bat_pst_current < 31) && ((bat_pst_current > 15) && (SmartAllet_lowbat30 == 0))) {  //배터리 부족경고
    SmartAllet_lowbat30 = 1;                                                               //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(12);
    delay(3000);
  } else if ((bat_pst_current < 15) && ((bat_pst_current > 9) && (SmartAllet_lowbat == 0))) {  //배터리 부족
    SmartAllet_lowbat = 1;                                                                     //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(13);
  } else if ((bat_pst_current < 10) && (SmartAllet_lowbat_more == 0)) {  //배터리 매우부족
    SmartAllet_lowbat_more = 1;                                          //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(14);
  } else if ((hic_print < 10) && (SmartAllet_toolowtemp == 0)) {  //온도 너무낮음
    SmartAllet_toolowtemp = 1;                                    //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(15);
  } else if ((hic_print > 32) && (SmartAllet_toohightemp == 0)) {  //온도 너무높음
    SmartAllet_toohightemp = 1;                                    //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(16);
  } else if (SmartAllet_SecurityAllet == 1) {  //보안시스템 작동
    SmartAllet_SecurityAllet = 0;              //SmartAllet 실행여부기록
    dfmp3.playMp3FolderTrack(11);
  } else {  //경고가 없다면
    Voice = random(1, 6);
    if (Voice == 1) {
      dfmp3.playMp3FolderTrack(17);
    } else if (Voice == 2) {
      dfmp3.playMp3FolderTrack(18);
    } else if (Voice == 3) {
      dfmp3.playMp3FolderTrack(19);
    } else if (Voice == 4) {
      dfmp3.playMp3FolderTrack(20);
    } else if (Voice == 5) {
      dfmp3.playMp3FolderTrack(21);
    } else {
      dfmp3.playMp3FolderTrack(22);
    }
  }
  delay(5000);
  dfmp3.playMp3FolderTrack(2);
  display.begin(4800);
  ss.begin(9600);
  delay(7000);
  DisplaySetup();
  adas = 0;  //OFF
  dfmp3.playMp3FolderTrack(1);

  //자전거 잠금해제

  while (1) {
    //Speed
    delay(50);
    int i;
    /*for (unsigned long start = millis(); millis() - start < 1000;) {
      while (ss.available()) {
        if (gps.encode(ss.read())) {
          newData = 1;
        }
      }
    }
    */
    /*
    Serial.print("GPS STATUS: ");
    Serial.println(gps.speed.isValid());
    if (newData == 1) {
      newData = 0;
      if (gps.speed.isValid() > 0) {
        display.print("n0.val=");
        display.print(round(gps.speed.kmph()));
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
        delay(50);
        display.print("p7.pic=11");
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
      } else { //가상 속도 표시
        display.print("n0.val=");
        display.print(round(bat_val_torque*(0.3)));
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
        delay(50);
        display.print("p7.pic=11");
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
      }
      delay(5);
    }
    */
    display.print("n0.val=");
    display.print(round(bat_val_torque * (0.25)));
    display.write(0xff);
    display.write(0xff);
    display.write(0xff);

    delay(50);
    display.print("p7.pic=11");
    display.write(0xff);
    display.write(0xff);
    display.write(0xff);

    //BAT
    value = analogRead(A14);
    vout = ((value * 5.0) / 1024.0);  //전압값을 계산해주는 공식입니다.

    vin = vout / (R2 / (R1 + R2));
    Serial.print("VIN: ");  //입력전압

    Serial.println(vin);

    //SMARTBAT
    float bat = 0;
    bat = (vin * 3.9);

    float bat_r = 54.1 - bat;
    Serial.print("BATTERY LEVEL: ");
    float bat_level = (bat_r / (15.6 / 100));
    float bat_level_k = 100 - bat_level;
    Serial.print(bat_level_k);

    //TEMP
    temp();
    float hic_org = t;

    if (hic == 0) {
      hic_org = temp_old;
    } else {
      temp_old = hic_org;
    }

    if (hic_org == 0) {
      
    }
    else {
      float  hic = hic_org;
    }

    int hic_print = round(hic);
    Serial.print("Temp : ");
    Serial.print(hic_print);
    Serial.println(" *C ");


    //BATallet
    if ((bat_pst_current < 31)) {
      display.print("p1.pic=4");
      display.write(0xff);
      display.write(0xff);
      display.write(0xff);
      delay(5);
    }
    if ((bat_pst_current < 31) && ((bat_pst_current > 15) && (SmartAllet_lowbat30 == 0))) {  //배터리 부족경고
      SmartAllet_lowbat30 = 1;                                                               //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(12);
      delay(3000);
    } else if ((bat_pst_current < 15) && ((bat_pst_current > 9) && (SmartAllet_lowbat == 0))) {  //배터리 부족
      SmartAllet_lowbat = 1;                                                                     //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(13);
      delay(4000);
    } else if ((bat_pst_current < 10) && (SmartAllet_lowbat_more == 0)) {  //배터리 매우부족
      SmartAllet_lowbat_more = 1;                                          //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(14);
      delay(4000);
    }
    if ((hic_print < 10) && (SmartAllet_toolowtemp == 0) && (hic_print != 0)) {  //온도 너무낮음
      SmartAllet_toolowtemp = 1;                                                 //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(15);
      delay(4000);
    } else if ((hic_print > 30) && (SmartAllet_toohightemp == 0)) {  //온도 너무높음
      SmartAllet_toohightemp = 1;                                    //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(16);
      delay(4000);
    } else if (SmartAllet_SecurityAllet == 1) {  //보안시스템 작동
      SmartAllet_SecurityAllet = 0;              //SmartAllet 실행여부기록
      dfmp3.playMp3FolderTrack(11);
      delay(6000);
    }

    //bat_pst_current; //현 배터리 상태
    if (bat_save_sc == 0) {
      bat_pst_past1 = bat_level_k;
    } else if (bat_save_sc == 1) {
      bat_pst_past2 = bat_level_k;
    } else if (bat_save_sc == 2) {
      bat_pst_past3 = bat_level_k;
    } else if (bat_save_sc == 3) {
      bat_pst_past4 = bat_level_k;
    } else if (bat_save_sc == 4) {
      bat_pst_past0 = bat_level_k;
      bat_save_sc = 0;
      bat_pst_csa = (((bat_pst_past1 + bat_pst_past2) + (bat_pst_past3 + bat_pst_past4) + bat_pst_past0)) / 5;
      Serial.print("BATTERY VOLTAGE A1: ");
      Serial.println(bat_pst_csa);
      if (bat_pst_csa < bat_pst_current) {
        if ((bat_pst_current - bat_pst_csa) > 30) {
          bat_pst_current = bat_pst_current - 30;
        } else {
          bat_pst_current = bat_pst_current - 1;
        }
      }
    }
    bat_save_sc = bat_save_sc + 1;
    if (bat_level_k < bat_pst_current) {
      if ((bat_pst_current - bat_level_k) > 1) {
        if (bat_val_torque > 95) {
          
        } else {
          bat_val_torque = bat_val_torque + 10;
        }
      }
    } else {
      if (bat_val_torque > 1) {
        bat_val_torque = bat_val_torque - 2;
      }
    }
    display.print("j3.val=");
    display.print(bat_val_torque);
    display.write(0xff);
    display.write(0xff);
    display.write(0xff);
    delay(10);

    if (bat_level_k < bat_pst_current) {  //W계산
      if ((bat_pst_current - bat_level_k) > 1) {
        if (bat_val_torque > 95) {

        } else {
          bat_val_torque = bat_val_torque + 10;
        }
      }
    } else {
      if (bat_val_torque > 1) {
        bat_val_torque = bat_val_torque - 2;
      }
    }
    if (bat_val_torque > 70) {
      vsound_stop = 0; //가속상태
    }
    if (bat_val_torque < 5) {
      vsound = 0; //정지상태임을 나타냄
    }

    display.print("j0.val=");
    display.print(bat_val_torque);
    display.write(0xff);
    display.write(0xff);
    display.write(0xff);
    delay(10);
    if ((vsound == 0)&&(bat_val_torque > 20)) {//가속시 실행
      vsound = 1;
      dfmp3.playMp3FolderTrack(24);
      AdditionalSoundCount = 20;
    }
    else if ((vsound == 1)&&(vsound_stop == 0)&&(bat_val_torque < 20)) {//감속시 실행
      vsound_stop = 1;
      dfmp3.playMp3FolderTrack(23);
      AdditionalSoundCount = 20;
    }
    if ((AdditionalSoundCount > 50)) {
      dfmp3.playMp3FolderTrack(1);
      delay(50);
      AdditionalSoundCount = 0;
    }
    AdditionalSoundCount = AdditionalSoundCount + 1;

    Serial.print("BATTERY VOLTAGE: ");
    Serial.println(bat);

    Serial.print("BATTERY VOLTAGE AG: ");
    Serial.println(bat_pst_current);

    if (sensor == 0) {  //센서 버튼 연결돼지 않았을때
      delay(200);
      Serial.println("ADAS OFF");
      if ((sensor == 0) && (adas == 1)) {  //센서 버튼이 연결해제됐다면
        display.print("p3.pic=6");         //ADAS경고등 on
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
        delay(50);
        dfmp3.playMp3FolderTrack(5);
        adas = 0;
        int adas_info_crt = 0;    //ADAS센싱정보상태 (오류값을 제거)
        int adas_Current_XA = 0;  //ADAS센서 위험강도 total
        int adas_Currt_IL = 0;    //ADAS현위험감도
        int adas_warning = 0;     //기본경고일경우 경고음을 1번만 재생(기본으로 돌아갈시 초기화)
      }
    } else if ((sensor == 1) && (adas == 0)) {  //센서 버튼이 연결됐다면
      display.print("p3.pic=11");               //ADAS경고등 해제
      display.write(0xff);
      display.write(0xff);
      display.write(0xff);
      delay(50);
      dfmp3.playMp3FolderTrack(4);
      delay(2000);
      dfmp3.playMp3FolderTrack(1);
      adas = 1;
    }
    //Supersonic(6, 7);
    b1 = c;
    //if ((adas == 0) && (sensor == 0)) {  //최적화를 위해 ADAS가 꺼져있고 센서값이 없다면 하나의 센서만 읽고 나머지는 패스.
    if (1) {
    } else {
      delay(10);
      Supersonic(13, 12);
      b2 = c;
      Supersonic(42, 43);
      b3 = c;
      Supersonic(26, 28);
      b4 = c;

      if (b2 < 10) {
        b2 = 500;
      }
      if (b3 < 10) {
        b3 = 500;
      }
      if (b4 < 10) {
        b4 = 500;
      }
      if (b1 < 10) {
        b1 = 500;
      }
      Serial.print("S1:");
      Serial.println(b1);
      Serial.print("S2:");
      Serial.println(b2);
      Serial.print("S3:");
      Serial.println(b3);
      Serial.print("S4:");
      Serial.println(b4);
    }
    

    if ((sensor == 1) && (adas == 1)) {                        //ADAS모듈이 정상작동중이라면
      if ((b1 < 30) || (b2 < 30) || (b3 < 30) || (b4 < 30)) {  //ADAS1 긴급경고(주변물체 너무가까움)
        adas_info_crt = adas_info_crt + 1;
        if (adas_info_crt < 3) {
          //오류값으로 ADAS 긴급경보 발동방지를 위해 최초1회는 무시
        } else {
          display.print("page dwarn");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          dfmp3.playMp3FolderTrack(9);
          delay(6000);
          display.print("page page0");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
          DisplaySetup();              //기본으로 돌아감
          display.print("p3.pic=11");  //ADAS경고등 해제
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
        }
      } else if ((b1 < 60) || (b2 < 60) || (b3 < 60) || (b4 < 60) || (adas_info_crt > 2)) {  //기본경고
        adas_info_crt = adas_info_crt + 1;
        display.print("p10.pic=15");
        display.write(0xff);
        display.write(0xff);
        display.write(0xff);
        delay(50);
        if (((b1 < 60) || (b2 < 60)) && (b3 > 60) || (b4 > 60)) {  // 전/후 중 어디에서 물체가 감지돼는지 확인 ( b1, b2) 후면으로 예상.
          display.print("p11.pic=16");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
        } else if (((b1 > 60) || (b2 > 60)) && (b3 < 60) || (b4 < 60)) {  // 전/후 중 어디에서 물체가 감지돼는지 확인 ( b3, b4) 전면으로 예상
          display.print("p12.pic=16");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
        } else {  //전 / 후 물체 동시감지
          display.print("p12.pic=16");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
          display.print("p11.pic=16");
          display.write(0xff);
          display.write(0xff);
          display.write(0xff);
          delay(50);
        }
        dfmp3.playMp3FolderTrack(10);
        delay(3000);
        DisplaySetup();  //기본으로 돌아감
      } else {
        if (adas_info_crt > 0) {
          adas_info_crt = adas_info_crt - 1;
        }
      }
    }


    display.print("n2.val=");  //온도
    display.print(hic_print);
    display.write(0xff);
    display.write(0xff);
    display.write(0xff);
    delay(10);
    display.print("n3.val=");        //배터리 잔량 %
    display.print(bat_pst_current);  //round(bat_level_k)
    display.write(0xFF);
    display.write(0xFF);
    display.write(0xFF);
    delay(10);
    display.print("n4.val=");  // (배터리 전압)
    display.print(round(bat));
    display.write(0xFF);
    display.write(0xFF);
    display.write(0xFF);
    delay(10);
    //CheckMaxOperatingDistance
    Smartdistance = bat_pst_current * 0.5;
    int SmartdistanceBar = Smartdistance * 2;
    display.print("j2.val=");  //주행거리 잔여 (바)
    display.print(SmartdistanceBar);
    display.write(0xFF);
    display.write(0xFF);
    display.write(0xFF);
    delay(10);
    display.print("n1.val=");  //주행거리 잔여
    display.print(Smartdistance);
    display.write(0xFF);
    display.write(0xFF);
    display.write(0xFF);
    delay(10);


    if (rfid.PICC_IsNewCardPresent()) {  //RFID 다시 입력됀다면 다시 잠금모드로 전환
      if (rfid.PICC_ReadCardSerial())
        NFCmoduleReset();

      //자전거 잠금 코드
      display.print("page lock");
      display.write(0xff);
      display.write(0xff);
      display.write(0xff);
      dfmp3.playMp3FolderTrack(8);
      digitalWrite(33, LOW);
      delay(5000);
      digitalWrite(32, HIGH);
      digitalWrite(34, HIGH);
      delay(50);
      int adas_info_crt = 0;    //ADAS센싱정보상태 (오류값을 제거)
      int adas_Current_XA = 0;  //ADAS센서 위험강도 total
      int adas_Currt_IL = 0;    //ADAS현위험감도
      int adas_warning = 0;     //기본경고일경우 경고음을 1번만 재생(기본으로 돌아갈시 초기화)
      display.end();
      ss.end();  //GPS
      //보안모듈 초기화
      a, b, c = 0;         //계산 임시 변수
      x, y, z = 0;         //측정 임시 변수
      motionlevel[3] = 0;  //경고레벨 (0 , 1 , 2 , 3) 감지
      motionlevel[2] = 0;  //경고레벨 (0 , 1 , 2 , 3) 감지
      motionlevel[1] = 0;  //경고레벨 (0 , 1 , 2 , 3) 감지
      motionlevel[0] = 0;  //경고레벨 (0 , 1 , 2 , 3) 감지
      lock_x = 0;          //현 값
      lock_y = 0;
      lock_z = 0;
      lock_x_m = 0;  //기존 값
      lock_y_m = 0;
      lock_z_m = 0;
      lock_x_r = 0;  //차이값
      lock_y_r = 0;
      lock_z_r = 0;
      motion_warn = 0;  //보안모션경고단계
      motion_warn_r = 0;
      speaker = 0;    //스피커 경고횟수(버그예방위해 초기1번만작동)
      skip_load = 5;  //1초동안의 값 무시
      SmartAllet_lowbat = 0;
      SmartAllet_lowbat_more = 0;
      SmartAllet_lowbat30 = 0;
      SmartAllet_SecurityAllet = 0;
      SmartAllet_toolowtemp = 0;
      SmartAllet_toohightemp = 0;
      Voice = 0;

      Serial.println(F("[Gibke] Gbike LOCKED"));

      delay(5000);
      return;
    }
  }
}

//함수
void NFCmoduleReset() {
  for (byte i = 0; i < 6; i++) {  //초기 RFID 입력값 초기화
    key.keyByte[i] = 0xFF;
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}
void SaveAccl() {
  lock_x = x;
  lock_y = y;
  lock_z = z;
  Serial.print("[GYRO] XYZ : ");
  Serial.print(lock_x_r);
  Serial.print(",");
  Serial.print(lock_y_r);
  Serial.print(",");
  Serial.print(lock_z_r);
  Serial.print(", MOTIONLEVEL : ");
  Serial.print(motion_warn);
  Serial.println();
}
void MeasureAccl(int a, int b, int d) {
  if (a > b) {  //현 값이 과거 값보다 크다면
    c = a - b;
  } else {  //과거 값이 현 값보다 크다면
    c = b - a;
  }
  if (c < 0) c = c * -1;

  if (c > 110) { //비정상 값 차단
    gyroerror = 1;
  }
  if (c > 3) {
    motionlevel[d] = 1;
    if (c > 5) {
      motionlevel[d] = 2;
      if (c > 10) {
        motionlevel[d] = 3;
      }
    }
  } else {
    motionlevel[d] = 0;
  }

  return c;
}
void CheckRFID(int a1, int a2, int a3) {
  if (rfid.uid.uidByte[0] == a1) {  //저장됀 NFC값과 비교
    if (rfid.uid.uidByte[1] == a2) {
      if (rfid.uid.uidByte[2] == a3) {
        c = 1;
      }
    }
  } else {
    if (c == 1) {
      //Do nothing
    } else {
      c = 2;
    }
  }
}
void printHex(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}
void printDec(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}
void Supersonic(int a, int b) {
  digitalWrite(a, LOW);
  delayMicroseconds(2);
  digitalWrite(a, HIGH);
  delayMicroseconds(10);
  digitalWrite(a, LOW);
  duration = pulseIn(b, HIGH);
  distance = duration * 17 / 1000;
  if (distance == 0) {  //꺼져있음
    sensor = 0;
    c = -1;
  } else if (distance > 550) {  //범위초과
    sensor = 1;
    c = 0;
  } else {  //정상
    sensor = 1;
    c = distance;
  }
}
void temp() {
  h = dht.readHumidity();     // 습도를 측정합니다.
  t = dht.readTemperature();  // 온도를 측정합니다.
  hic = t;
}
void DisplaySetup() {
  display.print("p1.pic=11");  //배터리경고등 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p2.pic=11");  //타이어압
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p4.pic=11");  //상향등 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p5.pic=11");  //빙판길 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p6.pic=11");  //과열 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p10.pic=13");  //장애물1 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p11.pic=13");  //장애물후방 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p12.pic=13");  //장애물전방 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(10);
  display.print("p7.pic=11");  //전자오류경고등 해제
  display.write(0xff);
  display.write(0xff);
  display.write(0xff);
  delay(100);
}
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate& d, TinyGPSTime& t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char* str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}


//FUCK!!
