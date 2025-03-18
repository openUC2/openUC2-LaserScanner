#pragma once

#define IS_XIAO 1
#include <esp_attr.h>
#include <vector>

#ifdef IS_XIAO
/*
D0: 1
D1: 2
D2: 3
D3: 4
D4: 5
D5: 6
D6: 43
D7: 44
D8: 7
D9: 8
D10: 9
*/
#define PIN_NUM_MISO      -1
#define PIN_NUM_SCK       GPIO_NUM_8        // D9
#define PIN_NUM_SDI       GPIO_NUM_7        // D8
#define PIN_NUM_CS        GPIO_NUM_9        // D10
#define PIN_NUM_LDAC      GPIO_NUM_6        // D7
#define PIN_NUM_LASER     GPIO_NUM_43
#define PIN_NUM_TRIG_PIXEL GPIO_NUM_2       // D1
#define PIN_NUM_TRIG_LINE  GPIO_NUM_3       // D2
#define PIN_NUM_TRIG_FRAME GPIO_NUM_4       // D3
#else
static const char *TAG = "Renderer";
#define PIN_NUM_MISO      -1
#define PIN_NUM_SCK       GPIO_NUM_14
#define PIN_NUM_SDI       GPIO_NUM_12
#define PIN_NUM_CS        GPIO_NUM_13
#define PIN_NUM_LDAC      GPIO_NUM_27
#define PIN_NUM_LASER     GPIO_NUM_26
#define PIN_NUM_TRIG_PIXEL GPIO_NUM_23
#define PIN_NUM_TRIG_LINE  GPIO_NUM_22
#define PIN_NUM_TRIG_FRAME GPIO_NUM_21
#endif

typedef struct spi_device_t *spi_device_handle_t;

class SPIRenderer
{
private:
  spi_device_handle_t spi;
  void draw(bool *stopFlag = nullptr);
  int nX;
  int nY;
  int tPixelDwelltime;
  int X_MIN;
  int X_MAX;
  int Y_MIN;
  int Y_MAX;
  int STEP;
  int nFrames;

public:
  SPIRenderer(int xmin, int xmax,
              int ymin, int ymax,
              int step,
              int tPixelDwelltime,
              int nFramesI);
  void start(bool *stopFlag = nullptr);
  void setParameters(int xmin, int xmax,
                     int ymin, int ymax,
                     int step,
                     int tPixelDwelltime,
                     int nFramesI);
};
