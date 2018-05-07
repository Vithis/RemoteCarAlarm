#include <SoftwareSerial.h>
#include <OneWire.h>                //DALLAS 18B20 naudoja OneWire
#include <DallasTemperature.h>
#include<Wire.h>                    //
#include "MPU6050.h"               // bibliotekos reikalingos
#include "I2Cdev.h"               // GY521 akselerometro kalibravimui
#include "Wire.h"                // ir reikšmems gauti
#define ONE_WIRE_BUS 3
#define trigPin 10
#define echoPin 11
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

SoftwareSerial GPRS(8, 7); // GSM modulio pin'ai. 8 RX, 7 TX

enum _parseState {           // parseState deklaruoja visas busenas, per kurias Arduino pereis bandydamas atpažinti žinute ir kas yra joje.

  PS_DETECT_MSG_TYPE,        // Pradine funkcija, nuolat laukia žinutes. Gavusi, tikrina, kur ji yra išsaugota.

  PS_IGNORING_COMMAND_ECHO,  // Programoje nereikia skaityti eiluciu, kurios prasideda tekstu "AT+...", todel pamates AT+, bufferis peršoka i kita eilute.

  PS_READ_CMTI_STORAGE_TYPE, // Mato +CMTI: "SM", 1 . Po SM pamato kableli, peršoka i sekanti state.
  PS_READ_CMTI_ID,           // Bufferis pasiima ID reikšme 1.

  PS_READ_CMGR_STATUS,       //
  PS_READ_CMGR_NUMBER,       // Ieško kableliu, kol daeina iki PS_READ_CMGR_CONTENT
  PS_READ_CMGR_SOMETHING,    // PS_READ_CMGR_CONTENT parodo gautos žinutes teksta
  PS_READ_CMGR_DATE,         //
  PS_READ_CMGR_CONTENT       //

};

long duration, distance;

byte state = PS_DETECT_MSG_TYPE; // Pradine busena

int stateC = 0;                 // bus veliau, bet cia deklaruojama pradine modulio busena.
// t.y. po kalibracijos signalizacija bus išjungta
char buffer[80];  // buffer 81 nariu masyvas, kuris skaitys viska iš AT komandu.
byte pos = 0;

int lastReceivedSMSId = 0;
boolean validSender = true; // Apsaugos sistema. Kontroliuoti signalizacija gali tik duoti tel. numeriai
boolean bardaciokas = false;

void resetBuffer() {
  memset(buffer, 0, sizeof(buffer)); // Funkcija, kuri išvalo bufferi
  pos = 0;
}

int buffersize = 1000;  // reguliuoja akselometro kalibracijos tiksluma. Daugiau = tiksliau.
int acel_deadzone = 8;  // akselometro paklaida. Mažiau = geriau
int giro_deadzone = 1;   //giroskopo paklaida. Mažiau = geriau

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, stateB = 0; //variables reikalingi kalibracijai
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset; // irgi.

MPU6050 accelgyro(0x68); // akselometro adresas

const int MPU_addr = 0x68;
int16_t GyroX, GyroY, GyroZ, ax, ay, az, gx, gy, gz; // giroskopo ir akselometro reikšmems priskirti intai
int pinled = 13; // LEMPUUTE!

//šita funkcija ištraukiau iš sketchbook, žinau tik tiek, kad ji kalibruoja
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) {
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); 
  }
}
//šita irgi
void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}

void setup()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(pinled, OUTPUT); // deklaruoju, kad D13 pinas duos energijos lemputei
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);     // pažadina akselometra
  Wire.endTransmission(true);

  GPRS.begin(9600);
  Serial.begin(9600);
  sensors.begin();
  GPRS.println("AT+CMGF=1"); // gsm modulyje ijungiam text mode
  delay(1000);

  accelgyro.initialize();
  delay(1000);

  Serial.println(accelgyro.testConnection() ? "Prisijunge" : "Neprisijunge"); // konsoleje rodo ar pavyko prisijungti prie akselometro
  delay(1000);
  // nuima offsetus
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  //praeina kalibracija
  if (stateB == 0) {

    meansensors();
    stateB++;
    delay(1000);
  }

  if (stateB == 1) {

    calibration();
    stateB++;
    delay(1000);
  }

  if (stateB == 2) {
    meansensors();
    Serial.println("\nBAIGTA!");
    //done
  }

  //mano sim korteleje telpa 15 sms žinuciu, todel butu protinga kaskart ijungus arduino jas visas ištrinti ir
  for (int i = 1; i <= 15; i++) {  // atlaisvinti atminti
    GPRS.print("AT+CMGD=");
    GPRS.println(i);
    delay(200);

    while (GPRS.available())
      Serial.write(GPRS.read());

  }
  //rašo man, kad arduino pasiruošes darbui
  GPRS.println("AT+CMGS=\"+37067198850\"");

  delay(500);

  Serial.print("Ready");
  GPRS.print("Ready!");

  GPRS.write( 0x1a ); // išsiunciant sms reikia spausti ctrl+z, bet arduino konsole to neleidžia, todel idedam ctrl+z adresa

}

void loop()
{

  onoff(); //pagrindine funkcija kurios funkcija yra funkcijoje

}
void parseATText(byte b) {

  buffer[pos++] = b;

  if ( pos >= sizeof(buffer) )
    resetBuffer();

  switch (state) {

    case PS_DETECT_MSG_TYPE:
      {
        if ( b == '\n' )
          resetBuffer();
        else {
          if ( pos == 3 && strcmp(buffer, "AT+") == 0 ) {
            state = PS_IGNORING_COMMAND_ECHO;
          }
          else if ( pos == 6 ) {

            if ( strcmp(buffer, "+CMTI:") == 0 ) {
              Serial.println("Received CMTI");
              state = PS_READ_CMTI_STORAGE_TYPE;
            }
            else if ( strcmp(buffer, "+CMGR:") == 0 ) {
              Serial.println("Received CMGR");
              state = PS_READ_CMGR_STATUS;
            }
            resetBuffer();
          }
        }
      }
      break;

    case PS_IGNORING_COMMAND_ECHO:
      {
        if ( b == '\n' ) {
          state = PS_DETECT_MSG_TYPE;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMTI_STORAGE_TYPE:
      {
        if ( b == ',' ) {
          Serial.print("SMS storage is ");
          Serial.println(buffer);
          state = PS_READ_CMTI_ID;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMTI_ID:
      {
        if ( b == '\n' ) {
          lastReceivedSMSId = atoi(buffer);
          Serial.print("SMS id is ");
          Serial.println(lastReceivedSMSId);

          GPRS.print("AT+CMGR=");
          GPRS.println(lastReceivedSMSId);
        

          state = PS_DETECT_MSG_TYPE;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMGR_STATUS:
      {
        if ( b == ',' ) {
          Serial.print("CMGR status: ");
          Serial.println(buffer);
          state = PS_READ_CMGR_NUMBER;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMGR_NUMBER:
      {
        if ( b == ',' ) {
          Serial.print("CMGR number: ");
          Serial.println(buffer);

          validSender = false;                                   //APSAUGOS SISTEMA
          if ( strcmp(buffer, "\"+37067198850\",") == 0 ) {       //TIKRINA TEL. NR
            validSender = true;
            Serial.print("\nvalindSender=true\n");
            state = PS_READ_CMGR_SOMETHING;
            resetBuffer();
          }
          else {
            Serial.print("valindSender = false");
            state = PS_DETECT_MSG_TYPE;
          }
        }
      }
      break;

    case PS_READ_CMGR_SOMETHING:
      {
        if ( b == ',' ) {
          Serial.print("CMGR something: ");
          Serial.println(buffer);
          state = PS_READ_CMGR_DATE;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMGR_DATE:
      {
        if ( b == '\n' ) {
          Serial.print("CMGR date: ");
          Serial.println(buffer);
          state = PS_READ_CMGR_CONTENT;
          resetBuffer();
        }
      }
      break;

    case PS_READ_CMGR_CONTENT:
      {
        if ( b == '\n' ) {
          Serial.print("CMGR content: ");
          Serial.print(buffer);

          parseSMSContent();

          GPRS.print("AT+CMGD=");
          GPRS.println(lastReceivedSMSId);

          state = PS_DETECT_MSG_TYPE;
          resetBuffer();
        }
      }
      break;
  }
}

void parseSMSContent() {

  char* ptr = buffer;

  while ( strlen(ptr) >= 2 ) {

    if ( ptr[0] == 't' or ptr[0] == 'T' ) {

      sensors.requestTemperatures();
      GPRS.println("AT+CMGS=\"+37067198850\"");

      delay(500);
      Serial.print("Temperatura yra ");
      Serial.println(sensors.getTempCByIndex(0));
      Serial.print(" laipsniai celsijaus.");
      GPRS.print("Temperatura yra ");
      GPRS.println(sensors.getTempCByIndex(0));
      GPRS.print(" laipsniai celsijaus.");
      GPRS.write( 0x1a ); // ctrl+Z character

      delay(500);

    }

    if ( ptr[0] == 'o' && ptr[1] == 'n' || ptr[0] == 'O' && ptr[1] == 'n' ) {

      stateC = 1;

      digitalWrite(pinled, HIGH);

      GPRS.println("AT+CMGS=\"+37067198850\"");

      delay(500);
      Serial.print("Apsaugos sistema ijungta.");
      GPRS.print("Apsaugos sistema ijungta.");
      GPRS.write( 0x1a ); 

      delay(500);

    }

    if ( ptr[0] == 'o' && ptr[1] == 'f' && ptr[2] == 'f' || ptr[0] == 'O' && ptr[1] == 'f' && ptr[2] == 'f' ) {

      stateC = 0;

      GPRS.println("AT+CMGS=\"+37067198850\"");

      delay(500);
      Serial.print("Apsaugos sistema isjungta.");
      GPRS.print("Apsaugos sistema isjungta.");
      GPRS.write( 0x1a ); // ctrl+Z character

      delay(500);

    }

    ptr += 2;
  }
}

void signalizacija() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); 
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  if ( GyroX < -1000 || GyroX > 1000 || GyroY < -1000 || GyroY > 1000 || GyroZ < 14000 || GyroZ > 19000 ) {

    digitalWrite(pinled, LOW);

    GPRS.println("AT+CMGS=\"+37067198850\"");

    delay(500);

    Serial.print("\n!!!SIGNALIZACIJA!!!");
    GPRS.print("!!!SIGNALIZACIJA!!!");

    GPRS.write( 0x1a ); // ctrl+Z character

  }
  else {
    digitalWrite(pinled, LOW);
  }

}

void onoff() {

  if (stateC == 0) {

    digitalWrite(pinled, LOW);
    while (GPRS.available()) {
      parseATText(GPRS.read());

    };
  }
  if (stateC == 1) {

    digitalWrite(pinled, HIGH);
    if (bardaciokas == false) {
      ultra();
    }
    signalizacija();

    while (GPRS.available()) {
      parseATText(GPRS.read());

    }
  };
}

void ultra() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  if (distance >= 20 && bardaciokas == false) {

    GPRS.println("AT+CMGS=\"+37067198850\"");
    delay(500);
    Serial.print("\n!!!ISILAUZTA I DAIKTADEZE!!!");
    GPRS.print("!!!ISILAUZTA I DAIKTADEZE!!!");
    GPRS.write( 0x1a ); 

    bardaciokas = true;
    delay(500);

  }
}


