# esp-i2c-cpp

Simple C++ wrapper around ESP-IDF I2C driver.

It currently supports master mode only.

# Requirements

Wrapper uses few modern C++ features (`std::unique_ptr`, `std::span`, `[[nodiscard]]`) so it requires ESP-IDF with
fairly recent
version of GCC compiler. This module was developed with ESP-IDF 5.0.2 which included GCC 11.2.

# How to use this repository

It is intended to be used as additional component in ESP-IDF based projects. It can be either cloned as a git submodule or copied to the target project repository.

Next, you should set `EXTRA_COMPONENT_DIRS` variable in your primary `CMakeLists.txt`. For example, if you cloned the
code into `external/esp-i2c-cpp` subdirectory:

```
set(EXTRA_COMPONENT_DIRS "external") # ESP-IDF searches for components recursively
```

Finally, in the `CMakeLists.txt` of component using this wrapper, you should set it via `REQUIRED`:

```cmake
idf_component_register(
        SRCS "YourSource.cpp"
        INCLUDE_DIRS "."
        REQUIRES esp-i2c-cpp
)
```

After clearing cache and re-running cmake, it should be picked up. At this point, `i2c.h` header file can be included
and classes from `I2C` namespace can be used in your project source code.

# TODOs

* Add support for slave mode
