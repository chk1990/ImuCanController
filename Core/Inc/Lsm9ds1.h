/**
 * @author Christoph Kolhoff
 * @file Lsm9ds1.h
 */

#ifndef INC_LSM9DS1_H_
#define INC_LSM9DS1_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

class Lsm9ds1 {
public:
	Lsm9ds1();
	virtual ~Lsm9ds1();

	HAL_StatusTypeDef init(I2C_HandleTypeDef *handle, uint16_t addrImu, uint16_t addrMag);
	uint8_t whoAmI(void);

private:
	I2C_HandleTypeDef *_handleImu = new I2C_HandleTypeDef;
	uint16_t _addrImu = 0xD4; // Address to write to
	uint16_t _addrMag = 0x38; // Address to write to
};

#endif /* INC_LSM9DS1_H_ */
