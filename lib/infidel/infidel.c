#include <stdint.h>
typedef struct __using("jm_i2c.spin2") Bus;

// I2C addresses for Infidel sensor
#define INFIDELADD 43

static Bus i2c;

float infidelin = 0;

void setupInfidel(int sck, int sda, int address) {
  i2c.stop();
  i2c.setup(sck, sda, 100, 0);
}

float readInfidel() {
  uint8_t b[2];
  b[0] = i2c.read(0);
  b[1] = i2c.read(1);
  infidelin = (((float) b[0]) * 256 + b[1]) / 1000;
  return infidelin;
}
