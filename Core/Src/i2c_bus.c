/*
 * Blocking I2C transport wrapper for STM32H7 HAL
 *
 * Callers provide an initialized HAL handle and unshifted 7-bit addresses
 * Board code owns GPIO, clock and peripheral configuration
 * Transfers use finite timeouts and are not automatically retried
 * Intended for one caller at a time, outside interrupt handlers
 */

#include "i2c_bus.h"


// Check bus descriptor configuration is correct
static i2c_bus_status_t validate_bus(const i2c_bus_t *bus)
{
  if ((bus == NULL) || (bus->handle == NULL) ||
      (bus->handle->Instance == NULL)) {
    return I2C_BUS_NOT_INITIALIZED;
  }
  if ((bus->timeout_ms == 0U) || (bus->timeout_ms == HAL_MAX_DELAY) ||
      (bus->handle->Init.AddressingMode != I2C_ADDRESSINGMODE_7BIT)) {
    return I2C_BUS_INVALID_ARGUMENT;
  }
  return I2C_BUS_OK;
}

// Validate device address and corresponding bus descriptor
static i2c_bus_status_t validate_device(const i2c_device_t *device)
{
  if ((device == NULL) || (device->address_7bit < 0x08U) ||
      (device->address_7bit > 0x77U)) {
    return I2C_BUS_INVALID_ARGUMENT;
  }
  return validate_bus(device->bus);
}

// Validate target device & buffer addresses, and transfer length 
static i2c_bus_status_t validate_transfer(
	const i2c_device_t *device,
	const void *data, 
  size_t length)
{
  if ((data == NULL) || (length == 0U) || (length > UINT16_MAX)) {
    return I2C_BUS_INVALID_ARGUMENT;
  }
  return validate_device(device);
}

// bitshift and typecast device address to HAL format
static uint16_t hal_address(const i2c_device_t *device)
{
  return (uint16_t)((uint16_t)device->address_7bit << 1U);
}

// HAL error code translation
static i2c_bus_status_t map_status(
  I2C_HandleTypeDef *handle,
  HAL_StatusTypeDef result)
{
  uint32_t error;
  if (result == HAL_OK) return I2C_BUS_OK;
  /* Do not let error bits left by an older transfer override HAL_BUSY. */
  if (result == HAL_BUSY) return I2C_BUS_BUSY;
  if (result == HAL_TIMEOUT) return I2C_BUS_TIMEOUT;
  error = HAL_I2C_GetError(handle);
  if ((error & HAL_I2C_ERROR_TIMEOUT) != 0U) return I2C_BUS_TIMEOUT;
  if ((error & HAL_I2C_ERROR_ARLO) != 0U) return I2C_BUS_ARBITRATION_LOST;
  if ((error & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR)) != 0U)
    return I2C_BUS_ERROR;
  if ((error & HAL_I2C_ERROR_AF) != 0U) return I2C_BUS_NACK;
  return I2C_BUS_ERROR;
}

// Validate register-address width and translate to HAL
static i2c_bus_status_t register_size_to_hal(
    uint16_t reg,
    i2c_bus_register_size_t size,
    uint16_t *hal_size)
{
  if ((size == I2C_BUS_REGISTER_8BIT) && (reg <= UINT8_MAX)) {
    *hal_size = I2C_MEMADD_SIZE_8BIT;
    return I2C_BUS_OK;
  }
  if (size == I2C_BUS_REGISTER_16BIT) {
    *hal_size = I2C_MEMADD_SIZE_16BIT;
    return I2C_BUS_OK;
  }
  return I2C_BUS_INVALID_ARGUMENT;
}

// Attach an initialized HAL controller; clear the descriptor on failure
i2c_bus_status_t i2c_bus_init(
    i2c_bus_t *bus, 
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms)
{
  i2c_bus_status_t result;
  if (bus == NULL) return I2C_BUS_INVALID_ARGUMENT;
  *bus = (i2c_bus_t){0};
  if ((handle == NULL) || (handle->Instance == NULL) ||
      (timeout_ms == 0U) || (timeout_ms == HAL_MAX_DELAY)) {
    return I2C_BUS_INVALID_ARGUMENT;
  }
  bus->handle = handle;
  bus->timeout_ms = timeout_ms;
  result = validate_bus(bus);
  if (result == I2C_BUS_OK) {
    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(handle);
    if (state == HAL_I2C_STATE_RESET) result = I2C_BUS_NOT_INITIALIZED;
    else if (state != HAL_I2C_STATE_READY) result = I2C_BUS_BUSY;
  }
  if (result != I2C_BUS_OK) *bus = (i2c_bus_t){0};
  return result;
}

// Check whether a device acknowledges its address using one attempt
i2c_bus_status_t i2c_bus_probe(const i2c_device_t *device)
{
  i2c_bus_status_t result = validate_device(device);
  if (result != I2C_BUS_OK) return result;
  /* This HAL reports exhausted NACK probe trials as TIMEOUT, not always AF. */
  return map_status(
      device->bus->handle,
      HAL_I2C_IsDeviceReady(
        device->bus->handle, 
        hal_address(device), 
        1U,
        device->bus->timeout_ms
        )
      );
}

// Transmit raw bytes without register-address prefix
i2c_bus_status_t i2c_bus_write(
    const i2c_device_t *device,
    const uint8_t *data, 
    size_t length)
{
  i2c_bus_status_t result = validate_transfer(device, data, length);
  if (result != I2C_BUS_OK) return result;
  /* HAL's blocking transmit takes a mutable pointer but does not modify TX data. */
  return map_status(
    device->bus->handle,
    HAL_I2C_Master_Transmit(
      device->bus->handle, 
      hal_address(device),
      (uint8_t *)data, 
      (uint16_t)length, 
      device->bus->timeout_ms)
    );
}

// Receive raw bytes without selecting a device register
i2c_bus_status_t i2c_bus_read(
  const i2c_device_t *device,
  uint8_t *data, 
  size_t length)
{
  i2c_bus_status_t result = validate_transfer(device, data, length);
  if (result != I2C_BUS_OK) return result;
  return map_status(
    device->bus->handle,
    HAL_I2C_Master_Receive(
      device->bus->handle, 
      hal_address(device), 
      data,
      (uint16_t)length, 
      device->bus->timeout_ms)
    );
}

// Write a register-address prefix followed by the caller's payload
i2c_bus_status_t i2c_bus_reg_write(
  const i2c_device_t *device,
  uint16_t reg,
  i2c_bus_register_size_t register_size,
  const uint8_t *data, size_t length)
{
  uint16_t hal_size;
  i2c_bus_status_t result = validate_transfer(device, data, length);
  if (result != I2C_BUS_OK) return result;
  result = register_size_to_hal(reg, register_size, &hal_size);
  if (result != I2C_BUS_OK) return result;
  return map_status(
    device->bus->handle,
    HAL_I2C_Mem_Write(
      device->bus->handle, 
      hal_address(device), 
      reg, 
      hal_size,
      (uint8_t *)data, 
      (uint16_t)length, 
      device->bus->timeout_ms)
    );
}

// Read a register's payload using a repeated START
i2c_bus_status_t i2c_bus_reg_read(
  const i2c_device_t *device,
  uint16_t reg,
  i2c_bus_register_size_t register_size,
  uint8_t *data, 
  size_t length)
{
  uint16_t hal_size;
  i2c_bus_status_t result = validate_transfer(device, data, length);
  if (result != I2C_BUS_OK) return result;
  result = register_size_to_hal(reg, register_size, &hal_size);
  if (result != I2C_BUS_OK) return result;
  return map_status(
    device->bus->handle,
    HAL_I2C_Mem_Read(
      device->bus->handle, 
      hal_address(device), 
      reg, 
      hal_size,
      data, 
      (uint16_t)length, 
      device->bus->timeout_ms)
    );
}
