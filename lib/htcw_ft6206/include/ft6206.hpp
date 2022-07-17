#pragma once
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
class ft6206 {
    TwoWire& m_i2c;
};
}