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
#define PIN_NUM_MISO -1
#define PIN_NUM_SCK GPIO_NUM_8        // D9
#define PIN_NUM_SDI GPIO_NUM_7        // D8
#define PIN_NUM_CS GPIO_NUM_9         // D10
#define PIN_NUM_LDAC GPIO_NUM_6      // D7
#define PIN_NUM_LASER GPIO_NUM_43     //
#define PIN_NUM_TRIG_PIXEL GPIO_NUM_2 // D1
#define PIN_NUM_TRIG_LINE GPIO_NUM_3  // D2
#define PIN_NUM_TRIG_FRAME GPIO_NUM_4 // D3
#else
static const char *TAG = "Renderer";
#define PIN_NUM_MISO -1
#define PIN_NUM_SCK GPIO_NUM_14
#define PIN_NUM_SDI GPIO_NUM_12   // 26
#define PIN_NUM_CS GPIO_NUM_13    // 27
#define PIN_NUM_LDAC GPIO_NUM_27  // GPIO_NUM_33
#define PIN_NUM_LASER GPIO_NUM_26 //
#define PIN_NUM_TRIG_PIXEL GPIO_NUM_23
#define PIN_NUM_TRIG_LINE GPIO_NUM_22x
#define PIN_NUM_TRIG_FRAME GPIO_NUM_21
#endif


typedef struct spi_device_t *spi_device_handle_t; ///< Handle for a device on a SPI bus

class SPIRenderer
{
private:
  spi_device_handle_t spi;
  void  draw();
  int nX;
  int nY;
  int tPixelDwelltime;
  int X_MIN = 0;
  int X_MAX = 2048;
  int Y_MIN = 0;
  int Y_MAX = 2048;
  int STEP = 64; // Adjust based on your desired resolution
  int nFrames = 1;
  

public:
  SPIRenderer(int xmin, int xmax, int ymin, int ymax, int step, int tPixelDwelltime, int nFramesI);
  void start();
  void setParameters(int xmin, int xmax, int ymin, int ymax, int step, int tPixelDwelltime, int nFramesI);

};
