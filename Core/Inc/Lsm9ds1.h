/**
* @author Christoph Kolhoff
* @file Lsm9ds1.h
*/

#ifndef INC_LSM9DS1_H_
#define INC_LSM9DS1_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_gpio.h"

class Lsm9ds1 {
public:
	Lsm9ds1();
	virtual ~Lsm9ds1();

        int init(SPI_HandleTypeDef *handle, GPIO_TypeDef *cs_port, uint16_t cs_pin);

private:
        SPI_HandleTypeDef *_handleImu = new SPI_HandleTypeDef;
        GPIO_TypeDef *_cs_port = new GPIO_TypeDef;
        uint16_t _cs_pin = 0;
};

#endif /* INC_LSM9DS1_H_ */
