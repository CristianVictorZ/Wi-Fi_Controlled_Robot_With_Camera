#ifndef FIFO_H_
#define FIFO_H_

#define SIOC 22
#define SIOD 21
#define VSY 32
#define RCK 4
#define WR 33
#define RRST 19
#define WRST 18

void prepareFrameCapture()
{
  digitalWrite(WRST, 0);
  delay(1);
  digitalWrite(WRST, 1);
}

void startFrameCapture()
{
  digitalWrite(WR, 1);
}

void stopFrameCapture()
{
  digitalWrite(WR, 0);
}

void fifoReset()
{
  digitalWrite(RRST, 0);
  delay(1);
  digitalWrite(RCK, 0);
  delay(1);
  digitalWrite(RCK, 1);
  delay(1);
  digitalWrite(RRST, 1);
}

#endif