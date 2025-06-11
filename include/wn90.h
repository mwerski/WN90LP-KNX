
#pragma region KNX Callbacks

// receiving date/time from the bus, setting localtime
void callback_dateTime (GroupObject& go);

// measurements gathered by wn90
void callback_Temperature (GroupObject& go);
void callback_Humidity (GroupObject& go);
void callback_WindSpeed (GroupObject& go);
void callback_GustSpeed (GroupObject& go);
void callback_WindDirection (GroupObject& go);
void callback_Pressure (GroupObject& go);
void callback_RainFall (GroupObject& go);
void callback_RainCounter (GroupObject& go);
void callback_UVindex (GroupObject& go);
void callback_Brightness (GroupObject& go);

// measurements gathered by other sensors (1wire, sds011, ...)
void callback_Temperature1 (GroupObject& go);
void callback_pm25 (GroupObject& go);
void callback_pm10 (GroupObject& go);

// these callbacks refer to calculated measurements
void callback_dewPoint (GroupObject& go);
void callback_frostPoint (GroupObject& go);
void callback_pm25_normalized (GroupObject& go);
void callback_pm10_normalized (GroupObject& go);
void callback_PressureTrend1 (GroupObject& go);
void callback_PressureTrend3 (GroupObject& go);

#pragma endregion
