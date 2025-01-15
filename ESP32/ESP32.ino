#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <mutex>
#include <HardwareSerial.h>
#include <camera.h>
#include <fifo.h>

std::mutex wifiMtx;
TaskHandle_t taskCamera;

const char* ssid = "ESP32";
const char* password = "arduino142857";

char CAMERA_CONTROL1, CAMERA_CONTROL2, MOVEMENT_CONTROL;

#define CAMERA_ENABLED 7
#define CAMERA_RESOLUTION1 6
#define CAMERA_RESOLUTION2 5
#define SATURATION_SIGN 4
#define SATURATION_VALUE1 3
#define SATURATION_VALUE2 2

#define BRIGHTNESS_SIGN 7
#define BRIGHTNESS_VALUE1 6
#define BRIGHTNESS_VALUE2 5
#define CONTRAST_SIGN 4
#define CONTRAST_VALUE1 3
#define CONTRAST_VALUE2 2

#define MOVEMENT_ENABLED 7
#define MOVEMENT_IDLE 6
#define MOVEMENT_SELECT 5
#define MOVEMENT_FORWARD 4
#define MOVEMENT_ROTATION 3

#define D0 13
#define D1 12
#define D2 14
#define D3 27
#define D4 26
#define D5 25
#define D6 35
#define D7 34

#define PIXEL_SIZE 2
//volatile int xResolution = 320, yResolution = 240;

#define RECEIVED_ACKNOWLEDGE 6
#define RECEIVED_UPDATE 12
#define RECEIVED_DISCONNECT 255
#define STATUS_CONFIRM 1

#define MAX_BYTES_SEND 1025
#define BYTE_SET_START 0b00010000
char frameByteSet[MAX_BYTES_SEND + 1];
char acknowledeAvailable = 0;

unsigned long startWaitTime;
#define TIME_TO_WAIT_FOR_ANSWER 3000
#define START_DELAY 3000
#define PET_WATCHDOG0_DELAY 1000
#define PET_WATCHDOG0_2_DELAY 10
#define PET_WATCHDOG1_DELAY 20

TaskHandle_t taskImageReadAndSend;

WiFiServer server(10000); 
static WiFiClient client;
boolean alreadyConnected = false;

void send_wifi(const char *_data)
{
  std::lock_guard<std::mutex> lck(wifiMtx);
  client.println(_data);
}

void updateCoreControl(char _con1, char _con2, char _con3)
{
  char lastMovControl = MOVEMENT_CONTROL;
  CAMERA_CONTROL1 = _con1;
  CAMERA_CONTROL2 = _con2;
  MOVEMENT_CONTROL = _con3;
  if(lastMovControl != MOVEMENT_CONTROL)
  {
    Serial2.println(MOVEMENT_CONTROL);
  }
}

unsigned char readByte()
{
  unsigned char b;

  digitalWrite(RCK, 1);

  b = digitalRead(D0) | (digitalRead(D1) << 1) | (digitalRead(D2) << 2) | (digitalRead(D3) << 3) |
      (digitalRead(D4) << 4) | (digitalRead(D5) << 5) | (digitalRead(D6) << 6) | (digitalRead(D7) << 7);

  digitalWrite(RCK, 0);

  return b;
}

void readFrame(int xRes, int yRes, int bytes)
{
  fifoReset();

  int bytesLoaded = 1, messageLength;
  frameByteSet[0] = BYTE_SET_START;
  frameByteSet[MAX_BYTES_SEND] = 0;
  unsigned char fr1, fr2;
  char answer[3];
  for(int y = 0; y<yRes; y++)
  {
    for(int x = 0; x<xRes; x++)
    {
      fr1 = readByte();
      if(fr1 == 0)
        fr1 = 1;
      frameByteSet[bytesLoaded++] = char(fr1);
      fr2 = readByte();
      if(fr2 == 0)
        fr2 = 1;
      frameByteSet[bytesLoaded++] = char(fr2);
      if(bytesLoaded % MAX_BYTES_SEND == 0)
      {
        send_wifi(frameByteSet);
        delay(PET_WATCHDOG0_2_DELAY);
        bytesLoaded = 1;
        startWaitTime = millis();
        while(!acknowledeAvailable && millis() < TIME_TO_WAIT_FOR_ANSWER + startWaitTime);
        if(millis() >= TIME_TO_WAIT_FOR_ANSWER + startWaitTime)
        {
          y = 1000;
          x = 1000;
          Serial.println("Time expired!");
          CAMERA_CONTROL1 &= !(1 << CAMERA_ENABLED);
          client.stop();
        }else
        {
          acknowledeAvailable = 0;
        }
      }
    }
  }
  if(bytesLoaded > 1)
  {
    send_wifi(frameByteSet);
    delay(10);
    startWaitTime = millis();
    while(!acknowledeAvailable && millis() < TIME_TO_WAIT_FOR_ANSWER + startWaitTime);
    if(millis() >= TIME_TO_WAIT_FOR_ANSWER + startWaitTime)
    {
      Serial.println("Time expired!");
      CAMERA_CONTROL1 &= !(1 << CAMERA_ENABLED);
      client.stop();
    }else
    {
      acknowledeAvailable = 0;
    }
  }
}

void captureAndSendFrame()
{
  while(!digitalRead(VSY));
  while(digitalRead(VSY));
  prepareFrameCapture();
  startFrameCapture();
  while(!digitalRead(VSY));
  stopFrameCapture();
  readFrame(xResolution, yResolution, PIXEL_SIZE);
}

void cameraHandler(void *pvParameters)
{
  for(;;)
  {
    if((CAMERA_CONTROL1 & (1 << CAMERA_ENABLED)))
    {
      captureAndSendFrame();
    }
    delay(PET_WATCHDOG0_DELAY);
    
    int cmrRes = (1 & (CAMERA_CONTROL1 >> CAMERA_RESOLUTION1)) + (1 & (CAMERA_CONTROL1 >> CAMERA_RESOLUTION2));
    cameraSetResolution(cmrRes);
    int cmrSat = ((1 & (CAMERA_CONTROL1 >> SATURATION_SIGN)) * (-2) + 1) * ((1 & (CAMERA_CONTROL1 >> SATURATION_VALUE1)) * 2 + (1 & (CAMERA_CONTROL1 >> SATURATION_VALUE2)));
    cameraSetSaturation(cmrSat);
    int cmrBri = ((1 & (CAMERA_CONTROL2 >> BRIGHTNESS_SIGN)) * (-2) + 1) * ((1 & (CAMERA_CONTROL2 >> BRIGHTNESS_VALUE1)) * 2 + (1 & (CAMERA_CONTROL2 >> BRIGHTNESS_VALUE2)));
    cameraSetSaturation(cmrBri);
    int cmrCntr = ((1 & (CAMERA_CONTROL2 >> CONTRAST_SIGN)) * (-2) + 1) * ((1 & (CAMERA_CONTROL2 >> CONTRAST_VALUE1)) * 2 + (1 & (CAMERA_CONTROL2 >> CONTRAST_VALUE2)));
    cameraSetSaturation(cmrCntr);

    delay(PET_WATCHDOG0_DELAY);
  }
}

void setup()
{
  Wire.begin();

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(VSY, INPUT);

  pinMode(RCK, OUTPUT);
  pinMode(WR, OUTPUT);
  pinMode(RRST, OUTPUT);
  pinMode(WRST, OUTPUT);

  digitalWrite(WR, 1);
  digitalWrite(WRST, 1);
  digitalWrite(RRST, 1);

  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);

  delay(START_DELAY);
  cameraInit();

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();

  delay(START_DELAY);
  xTaskCreatePinnedToCore(
                    cameraHandler,
                    "TaskCamera",
                    4096,
                    NULL,
                    1,
                    &taskCamera,
                    0);  
}

void loop()
{
  if (!client)
    client = server.available();
  if (client)
  {       
    if (!alreadyConnected)
    {
      client.flush();
      alreadyConnected = true;
    }
    
    int messageLength;
    char receivedData[3], statusMessage[2];

    if(client.available())
    {
      do{
        client.readBytes((char*)&receivedData, sizeof(char));
        if(receivedData[0] == RECEIVED_UPDATE)
        {
          client.readBytes((char*)&receivedData, 3*sizeof(char));
          updateCoreControl(receivedData[0], receivedData[1], receivedData[2]);
          statusMessage[0] = STATUS_CONFIRM;
          statusMessage[1] = 0;
          send_wifi(statusMessage);
        }else if(receivedData[0] == RECEIVED_ACKNOWLEDGE)
        {
          acknowledeAvailable = 1;
        }else if(receivedData[0] == RECEIVED_DISCONNECT)
        {
          client.stop();
          updateCoreControl(0x01, 0x01, 0x01);
        }
      }while(client.available());
    }
  }
  delay(PET_WATCHDOG1_DELAY);
}
