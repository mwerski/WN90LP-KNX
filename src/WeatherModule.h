#pragma once
#include "OpenKNX.h"

#define WX_FIRMWARE_VERSION "0.1.0"

class WeatherModule : public OpenKNX::Module
{
  public:
    const std::string name() override;
    const std::string version() override;
    void setup(bool configured) override;
    void loop(bool configured) override;
};

extern WeatherModule openknxWeatherModule;
