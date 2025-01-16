#ifndef CAMERA_H_
#define CAMERA_H_

#define OV7670_ADDR 0x21
#define CLKRC 0x11
#define COM3 0x0c
#define COM7 0x12
#define COM11 0x3b
#define COM14 0x3e
#define COM15 0x40
#define TSLB 0x3a
#define SCALING_XSC 0x70
#define SCALING_YSC 0x71
#define SCALING_DCWCTR 0x72
#define SCALING_PCLK_DIV 0x73
#define SCALING_PCLK_DELAY 0xA2
#define HSTART 0x17
#define HSTOP 0x18
#define HREF 0x32
#define VSTART 0x19
#define VSTOP 0x1A
#define VREF 0x03
#define REG_MVFP 0x1e
#define BRIGHT 0x55
#define CONTRAS 0x56
#define MTX1 0x4f
#define MTX2 0x50
#define MTX3 0x51
#define MTX4 0x52
#define MTX5 0x53
#define MTX6 0x54
#define MTXS 0x58
#define AWBCTR0 0x6f
#define COM8 0x13

extern volatile int xResolution = 320, yResolution = 240;

void write_i2c(unsigned char addr, unsigned char reg, unsigned char data)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void frameControl(int hStart, int hStop, int vStart, int vStop)
{
  write_i2c(OV7670_ADDR, HSTART, hStart >> 3);
  write_i2c(OV7670_ADDR, HSTOP,  hStop >> 3);
  write_i2c(OV7670_ADDR, HREF, ((hStop & 0b111) << 3) | (hStart & 0b111));

  write_i2c(OV7670_ADDR, VSTART, vStart >> 2);
  write_i2c(OV7670_ADDR, VSTOP, vStop >> 2);
  write_i2c(OV7670_ADDR, VREF, ((vStop & 0b11) << 2) | (vStart & 0b11));
}

void cameraSetSaturation(int saturationValue)
{
  char sign;
  if(saturationValue < 0)
    sign = -1;
  else if(saturationValue > 0)
    sign = 1;
  else
    sign = 0;

  write_i2c(OV7670_ADDR, MTX1, 0x80 + sign * 14 + sign * 12 * saturationValue);
  write_i2c(OV7670_ADDR, MTX2, 0x80 + sign * 14 + sign * 12 * saturationValue);
  write_i2c(OV7670_ADDR, MTX3, 0x00);
  write_i2c(OV7670_ADDR, MTX4, 0x22 + sign * 4 + sign * 3 * saturationValue); 
  write_i2c(OV7670_ADDR, MTX5, 0x5e + sign * 10 + sign * 9 * saturationValue);
  write_i2c(OV7670_ADDR, MTX6, 0x80 + sign * 14 + sign * 12 * saturationValue);
  write_i2c(OV7670_ADDR, MTXS, 0x9e);
}

void cameraSetBrightness(int brightnessValue)
{
  if(brightnessValue < 0)
    write_i2c(OV7670_ADDR, BRIGHT, 0x80 - (brightnessValue * 0x18));
  else
    write_i2c(OV7670_ADDR, BRIGHT, brightnessValue * 0x18);
}

void cameraSetContrast(int contrastValue)
{
  if(contrastValue < 0)
    write_i2c(OV7670_ADDR, CONTRAS, 0x40 - (contrastValue * 0x08));
  else
    write_i2c(OV7670_ADDR, CONTRAS, 0x40 + (contrastValue * 0x10));
}

void cameraSetResolution(int resolutionValue)
{
  switch(resolutionValue)
  {
    case 0:

      xResolution = 320;
      yResolution = 240;

      write_i2c(OV7670_ADDR, COM3, 0x04);
      write_i2c(OV7670_ADDR, COM14, 0x19);
      write_i2c(OV7670_ADDR, SCALING_XSC, 0x3a);
      write_i2c(OV7670_ADDR, SCALING_YSC, 0x35);
      write_i2c(OV7670_ADDR, SCALING_DCWCTR, 0x11);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DIV, 0xf1);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DELAY, 0x02);

      break;
    case 1:

      xResolution = 160;
      yResolution = 120;

      write_i2c(OV7670_ADDR, COM3, 0x04);
      write_i2c(OV7670_ADDR, COM14, 0x1a);
      write_i2c(OV7670_ADDR, SCALING_XSC, 0x3a);
      write_i2c(OV7670_ADDR, SCALING_YSC, 0x35);
      write_i2c(OV7670_ADDR, SCALING_DCWCTR, 0x22);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DIV, 0xf2);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DELAY, 0x02);

      break;
    case 2:

      xResolution = 80;
      yResolution = 60;

      write_i2c(OV7670_ADDR, COM3, 0x04);
      write_i2c(OV7670_ADDR, COM14, 0x1b);
      write_i2c(OV7670_ADDR, SCALING_XSC, 0x3a);
      write_i2c(OV7670_ADDR, SCALING_YSC, 0x35);
      write_i2c(OV7670_ADDR, SCALING_DCWCTR, 0x33);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DIV, 0xf3);
      write_i2c(OV7670_ADDR, SCALING_PCLK_DELAY, 0x02);

      break;
    default:
      break;
  }
}

void cameraInit()
{
  Serial.println("Camera Init!");
  
  write_i2c(OV7670_ADDR, COM7, 0b10000000);

  write_i2c(OV7670_ADDR, CLKRC, 0b10000000);
  write_i2c(OV7670_ADDR, COM11, 0b1000);
  write_i2c(OV7670_ADDR, TSLB, 0b1000);

  write_i2c(OV7670_ADDR, COM7, 0b100);
  write_i2c(OV7670_ADDR, COM15, 0b11010000);

  frameControl(196, 52, 8, 488);//???
    
  write_i2c(OV7670_ADDR, REG_MVFP, 0x2b);

  cameraSetResolution(0);
  
  write_i2c(OV7670_ADDR, 0xb0, 0x84);//???
  
  cameraSetSaturation(0);

  write_i2c(OV7670_ADDR, COM8, 0xe7);
  write_i2c(OV7670_ADDR, AWBCTR0, 0x9f);
}

#endif
