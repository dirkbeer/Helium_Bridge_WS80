
#ifndef WS_DATA_INCLUDED
#define WS_DATA_INCLUDED

class WeatherStationData
{
    private:
        char data[150];
        char jsonbuffer[1000];

    public:
        int id;
        short status;
        float battery;
        float case_temp;
        float solar;
        int   rssi;         // Signal quality of cell modem
        float minWindSpeed = 0.0;
        float avgWindSpeed = 0.0;
        float maxWindSpeed = 0.0;
        int   minWindDirection = 0;
        int   avgWindDirection = 0;
        int   maxWindDirection = 0;
        float temp = 0.0;
        float pressure = 0.0;
        float humidity = 0.0;
        int   light = 0;
        float uv_index = 0;
        int   sensor_battery = 0;
        int   sensor_rssi = 0;      // Signal quality of rf based sensors
        int   sensor_readings = 0;  // Number of sensor readings during send interval
        int   sensor_valid_readings = 0;  // Number of valid sensor readings during send interval

        void FormatData();
        const char* GetData() { return data; }
        const char* ToJson();    
};

#endif