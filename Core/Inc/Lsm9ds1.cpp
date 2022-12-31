/**
 * @author Christoph Kolhoff
 * @file Lsm9ds1.cpp
 */

#include <Lsm9ds1.h>

/**
 * @brief Constructor
 */
Lsm9ds1::Lsm9ds1() {
	//

}

/**
 * @brief Destructor
 */
Lsm9ds1::~Lsm9ds1() {
	//
}

/**
 * @brief Initialize the I2C interface to be used
 * @param[in] handle Handle for I2C interface
 * @param[in] addrImu Address of the IMU - bit 0 = 0
 * @param[in] addrMag Address of the Magnetometer - bit 0 = 0
 * @return -1 on failure; 0 on success
 */
int8_t
Lsm9ds1::init(I2C_HandleTypeDef *handle, uint8_t addrImu, uint8_t addrMag)
{
	int8_t ret = -1;

	_handleImu = handle;

	ret = 0;

	return ret;
}

/**
 * @brief Return the value of a build-in register for testing
 * @return 104 on success
 */
uint8_t
Lsm9ds1::whoAmI(void)
{
	uint8_t dataRx = 0;

	return dataRx;
}
