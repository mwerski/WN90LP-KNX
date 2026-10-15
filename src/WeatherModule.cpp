#include "WeatherModule.h"

WeatherModule openknxWeatherModule;

const std::string WeatherModule::name()
{
    return "Weather";
}

const std::string WeatherModule::version()
{
    return WX_FIRMWARE_VERSION;
}

void WeatherModule::setup(bool configured)
{
    logInfoP("Setup (konfiguriert: %u)", configured);
}

void WeatherModule::loop(bool configured)
{
}
