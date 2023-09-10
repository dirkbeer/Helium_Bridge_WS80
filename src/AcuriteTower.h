
#ifndef WS80_INCLUDED
#define WS80_INCLUDED

#include "WeatherStationData.h"


#define STATUS_OK                   0
#define STATUS_CPU_RESTART          1
#define STATUS_SEND_FAILED          2
#define STATUS_SOCKET_RECONNECT     3
#define STATUS_GPRS_RECONNECT       4
#define STATUS_MODEM_RESTART        5
#define STATUS_SENSOR_NOT_FOUND     6
#define STATUS_NO_SENSOR_DATA       7
#define STATUS_SENSOR_RESET         8

#define MAX_WS80_READINGS 35

struct WS80_Reading {
    public:
        int id;
        int light;
        int battery_mv;
        float temp;
        int humidity;
        float wind_avg;
        float wind_max;
        float uv_index;
        float wind_dir_x;
        float wind_dir_y;
        int rssi;
};

class WS80
{   
    private:
        int _id;
        bool Init();
        bool Decode(unsigned char *b);        
        bool ReadDataPacket();        

        WS80_Reading _readings[MAX_WS80_READINGS];

        // float _wind_dir_x = 1.0;
        // float _wind_dir_y = 0.0;

        int _num_readings;
        int _last_rssi;

    public:

        bool Setup(int id);

        void Loop();        

        bool CheckForData(WeatherStationData *wsData);

        bool WaitForData(WeatherStationData *wsData, long timout = 15000);
        
        void SetStationData(WeatherStationData *wsData);

        unsigned long LastPacketReceivedMillis();
};

#endif
