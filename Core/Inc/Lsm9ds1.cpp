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
 * @brief Initialize the SPI interface to be used
 * @param[in] handle Handle for SPI interface
 * @param[in] cs_port Port
 * @param[in] cs_pin Pin
 * @return -1 on failure; 0 on success
 */
int
Lsm9ds1::init(SPI_HandleTypeDef *handle, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
        int ret = -1;

        _handleImu = handle;
        _cs_port = cs_port;
        _cs_pin = cs_pin;

        ret = 0;

        return ret;
}
