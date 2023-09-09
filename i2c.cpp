/*
 * Copyright (c) 2023 Michał Łubiński <mlubinski@coffeetocode.pl>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "i2c.h"

#include <memory>

namespace {
  /**
   * Deleter used by @ref i2c_command_ptr.
   *
   * It calls `i2c_cmd_link_delete` function from the ESP-IDF.
   */
  constexpr auto i2c_cmd_deleter = [](i2c_cmd_handle_t cmd) {
    i2c_cmd_link_delete(cmd);
  };

  /**
   * Typedef used to conveniently wrap `i2c_cmd_handle_t` with `unique_ptr` to automate
   * I2C command list deallocation.
   *
   * @see i2c_cmd_deleter
   */
  using i2c_command_ptr = std::unique_ptr<void, decltype(i2c_cmd_deleter)>;
}

/**
 * @brief Initializes I2C master instance with given parameters.
 *
 * @param i2cPort ESP32 I2C port number
 * @param sdaPin ESP32 GPIO pin used as SDA pin
 * @param sclPin ESP32 GPIO pin used as SCL pin
 * @param clockSpeed I2C clock speed
 *
 * @return ESP_OK if initialization is successfully finished, ESP_FAIL otherwise
 */
esp_err_t I2C::Master::initialize(const i2c_port_t i2cPort,
                                  const gpio_num_t sdaPin,
                                  const gpio_num_t sclPin,
                                  const uint32_t clockSpeed)
{
  const i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = sdaPin,
    .scl_io_num = sclPin,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master = {
      .clk_speed = clockSpeed,
    },
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
  };
  port = i2cPort;

  if (i2c_param_config(port, &config) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

/**
 * @brief Resets I2C master instance.
 */
void I2C::Master::reset()
{
  i2c_driver_delete(port);
  port = I2C_NUM_MAX;
}

/**
 * @brief Read bytes from specified device and register.
 *
 * @param deviceAddress I2C address of a device
 * @param registerAddress address of a register to be read
 * @param buffer span where received bytes are going to be stored
 * @param timeout read operation timeout in milliseconds
 *
 * @return ESP_OK if bytes are successfully read, ESP_FAIL otherwise
 */
esp_err_t I2C::Master::readBytes(const uint8_t deviceAddress,
                                 const uint8_t registerAddress,
                                 const std::span<uint8_t>& buffer,
                                 const uint32_t timeout) const
{
  i2c_command_ptr command(i2c_cmd_link_create(), i2c_cmd_deleter);

  if (command == nullptr) {
    return ESP_FAIL;
  }

  if (i2c_master_start(command.get()) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(command.get(),
                            (deviceAddress << 1) | I2C_MASTER_WRITE,
                            true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(command.get(),
                            registerAddress,
                            true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_start(command.get()) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(command.get(),
                            (deviceAddress << 1) | I2C_MASTER_READ,
                            true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_read(command.get(),
                      buffer.data(),
                      buffer.size(),
                      I2C_MASTER_LAST_NACK) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_stop(command.get()) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_cmd_begin(port,
                           command.get(),
                           pdMS_TO_TICKS(timeout)) != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

/**
 * @brief Write bytes to specified device and register.
 *
 * @param deviceAddress I2C address of a device
 * @param registerAddress address of a register where bytes are going to be written
 * @param data span with data to be written
 * @param timeout write operation timeout in milliseconds
 *
 * @return ESP_OK if bytes are successfully written, ESP_FAIL otherwise
 */
esp_err_t I2C::Master::writeBytes(const uint8_t deviceAddress,
                                  const uint8_t registerAddress,
                                  const std::span<const uint8_t>& data,
                                  const uint32_t timeout) const
{
  i2c_command_ptr command(i2c_cmd_link_create(), i2c_cmd_deleter);

  if (command == nullptr) {
    return ESP_FAIL;
  }

  if (i2c_master_start(command.get()) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(command.get(),
                            (deviceAddress << 1) | I2C_MASTER_WRITE,
                            true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(command.get(),
                            registerAddress,
                            true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_write(command.get(),
                       data.data(),
                       data.size(),
                       true) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_stop(command.get()) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_master_cmd_begin(port,
                           command.get(),
                           pdMS_TO_TICKS(timeout)) != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}
