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
 * @return HAL status
 */
HAL_StatusTypeDef
Lsm9ds1::init(I2C_HandleTypeDef *handle, uint16_t addrImu, uint16_t addrMag)
{
	_handleImu = handle;
	_addrImu = addrImu;
	_addrMag = addrMag;

	return HAL_OK;
}

/**
 * @brief Return the value of a build-in register for testing
 * @return 104 on success
 */
uint8_t
Lsm9ds1::whoAmI(void)
{
	uint8_t dataRx = 0;

	//HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_handleImu, _addrImu, &dataRx, 1, HAL_MAX_DELAY);
	//HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(_handleImu, _addrImu, &dataRx, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(_handleImu, _addrImu, 0x0F, 1, &dataRx, 1, HAL_MAX_DELAY);

	return dataRx;
}
