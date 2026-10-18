#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stddef.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  I2C_BUS_OK = 0,
  I2C_BUS_INVALID_ARGUMENT,
  I2C_BUS_NOT_INITIALIZED,
  I2C_BUS_BUSY,
  I2C_BUS_TIMEOUT,
  I2C_BUS_NACK,
  I2C_BUS_ARBITRATION_LOST,
  I2C_BUS_ERROR
} i2c_bus_status_t;

typedef enum {
  I2C_BUS_REGISTER_8BIT = 8,
  I2C_BUS_REGISTER_16BIT = 16
} i2c_bus_register_size_t;

typedef struct {
  I2C_HandleTypeDef *handle;
  uint32_t timeout_ms;
} i2c_bus_t;

typedef struct {
  i2c_bus_t *bus;
  uint8_t address_7bit;
} i2c_device_t;

/**
 * Attach an already HAL-initialized, 7-bit-addressed I2C controller.
 * GPIOs, clocks, filters and timing are owned by CubeMX/board code.
 * timeout_ms must be > 0 and < HAL_MAX_DELAY. No hardware is touched here.
 * Objects must outlive all transfers; initialization failure clears *bus.
 */
i2c_bus_status_t i2c_bus_init(i2c_bus_t *bus, I2C_HandleTypeDef *handle,
                              uint32_t timeout_ms);

/**
 * All operations below block and make one HAL call, without automatic retries.
 * Call from one task/main loop with an advancing HAL tick, never from an ISR.
 * Concurrent users must serialize the entire logical operation externally.
 * Addresses are UNSHIFTED 7-bit values, 0x08..0x77. Only this backend shifts them.
 * Buffer lengths are 1..UINT16_MAX bytes; caller owns buffers until return.
 * On failed reads, buffer contents are unspecified and must not be consumed.
 * HAL's own finite bus-busy waits may exceed the requested transfer timeout.
 */
i2c_bus_status_t i2c_bus_probe(const i2c_device_t *device);
i2c_bus_status_t i2c_bus_write(const i2c_device_t *device,
                               const uint8_t *data, size_t length);
i2c_bus_status_t i2c_bus_read(const i2c_device_t *device,
                              uint8_t *data, size_t length);
i2c_bus_status_t i2c_bus_reg_write(const i2c_device_t *device,
                                   uint16_t reg,
                                   i2c_bus_register_size_t register_size,
                                   const uint8_t *data, size_t length);
/** Read using HAL_I2C_Mem_Read: register pointer + repeated START + data. */
i2c_bus_status_t i2c_bus_reg_read(const i2c_device_t *device,
                                  uint16_t reg,
                                  i2c_bus_register_size_t register_size,
                                  uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
#endif
