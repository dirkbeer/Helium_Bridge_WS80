#include "WeatherStationData.h"

#include <Arduino.h>

void WeatherStationData::FormatData()
{
    sprintf(data, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d", id, status , (int)(battery * 100), (int)(solar * 10), rssi, (int)(minWindSpeed * 10 * 3.6),(int)(avgWindSpeed * 10 * 3.6), (int)(maxWindSpeed * 10 * 3.6), minWindDirection, avgWindDirection, maxWindDirection, (int)(temp * 10), (int)(pressure * 10), (int)(humidity),light,(int)(uv_index * 10),sensor_rssi, sensor_battery, sensor_readings, sensor_valid_readings, (int)(case_temp * 10));
}

const char* WeatherStationData::ToJson() {
    sprintf(jsonbuffer, "{\"stationId\":%d, \"battery\":%.2f, \"solar\":%.2f, \"rssi\": %d, \"minWindSpeed\": %.2f, \"avgWindSpeed\": %.2f, \"maxWindSpeed\": %.2f, \"minWindDirection\": %d, \"avgWindDirection\": %d, \"maxWindDirection\": %d, \"temp\":%.2f, \"pressure\": %.1f, \"humidity\": %.1f, \"lux\": %d}", id, battery, solar, rssi, minWindSpeed, avgWindSpeed, maxWindSpeed, minWindDirection, avgWindDirection, maxWindDirection, temp, pressure, humidity, light);
    return jsonbuffer;
}