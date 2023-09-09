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

#ifndef COFFEETOCODE_ESP32_I2C
#define COFFEETOCODE_ESP32_I2C

#include <driver/i2c.h>

#include <span>

namespace I2C {
  /**
   * Default configuration options.
   */
  namespace Defaults {
    static constexpr uint32_t ClockSpeed = 100000;
    static constexpr uint32_t Timeout = 1000;
  }

  /**
   * @brief Simple C++ wrapper around I2C master implementation from ESP-IDF framework.
   *
   * Supports reading to and writing from a specific device address, under specific register address.
   */
  class Master
  {
  public:
    Master() = default;
    ~Master() = default;
    Master(const Master& master) = delete;
    Master(const Master&& master) = delete;

    [[nodiscard]] esp_err_t initialize(i2c_port_t i2cPort,
                                       gpio_num_t sdaPin,
                                       gpio_num_t sclPin,
                                       uint32_t clockSpeed = I2C::Defaults::ClockSpeed);
    void reset();

    [[nodiscard]] esp_err_t readBytes(uint8_t deviceAddress,
                                      uint8_t registerAddress,
                                      const std::span<uint8_t>& buffer,
                                      uint32_t timeout = I2C::Defaults::Timeout) const;
    [[nodiscard]] esp_err_t writeBytes(uint8_t deviceAddress,
                                       uint8_t registerAddress,
                                       const std::span<const uint8_t>& data,
                                       uint32_t timeout = I2C::Defaults::Timeout) const;

  private:
    i2c_port_t port{ I2C_NUM_MAX };
  };
}

#endif // COFFEETOCODE_ESP32_I2C