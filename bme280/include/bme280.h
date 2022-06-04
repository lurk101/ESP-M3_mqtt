#ifndef _BME280_H_
#define _BME280_H_

#include <stdint.h>

int getTempHumPress(int32_t* temp, int32_t* hum, int32_t* press);

#endif
