#include "WS80.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define SDCARD_MOSI     15
#define SDCARD_MISO     2
#define SDCARD_SCLK     14
#define SDCARD_CS       13

#define GDO0_PIN        25
#define GDO2_PIN        4

bool volatile packetWaiting = false;

unsigned long last_packet_received = 0;

void packetReceived()
{
    packetWaiting = true;
    last_packet_received = millis();
}

void combSort(float readings[], size_t size)
{
    uint8_t i, j, top, switched = 0, gap = size;

    while (1 < gap || switched)
    {
        gap = (size_t)((float)gap / 1.3f);
        if (gap < 11)
        {
            if (9 <= gap)
            {
                gap = 11;
            }
            else if (!gap)
            {
                gap = 1;
            }
        }
        for (switched = 0, top = size - gap, i = 0; i < top; ++i)
        {
            j = i + gap;
            if (readings[i] > readings[j])
            {
                auto tmp = readings[i];
                readings[i] = readings[j];
                readings[j] = tmp;
                switched = 1;
            } // swap
        }
    }
}

int toDegrees(float radians)
{
    return (int)((radians * 4068.0) / 71.0);
}

float toRadians(int degrees)
{
     return ((float)degrees * 71.0) / 4068.0;
}

int vectorToDegree(float x, float y)
{
    float tmp = toDegrees(atan2(y, x));
    return tmp < 0 ? 360 + tmp : tmp;
}


size_t findAverageMinMax(float readings[], size_t size, float& average, float& min, float& max)
{
    average = 0;

    size_t count = 0;
    for (int i = 0; i < size; i++)
    {
        auto val = readings[i];
        average += val;
        if (val > max)
        {
            max = val;
        }
        if (val < min)
        {
            min = val;
        }
        count++;
    }

    if (count > 0)
    {
        average /= (float)count;
    }

    return count;
}


size_t findFilteredAverageMinMax(float readings[], size_t size, float& average, float& min, float& max)
{
    int q1_i = size * .25;
    int q3_i = size * .75;

    auto q1 = readings[q1_i];
    auto q3 = readings[q3_i];

    auto iqr = q3 - q1;

    auto outlier_step = 1.5 * iqr;

    average = 0;

    size_t count = 0;
    for (int i = 0; i < size; i++)
    {
        auto val = readings[i];
        if (val < q1 - outlier_step || val > q3 + outlier_step)
        {
            Serial.print("discarded: ");
            Serial.println(val);
            continue;
        }
        average += val;
        if (val > max)
        {
            max = val;
        }
        if (val < min)
        {
            min = val;
        }
        count++;
    }

    if (count > 0)
    {
        average /= (float)count;
    }

    return count;
}

float filteredAverage(const char *description, float currentAverage, float newReading, float factor, float maxDeviation, float deviationFactor = 1.0) 
{
    if (newReading > currentAverage + maxDeviation) {
        newReading = currentAverage + (maxDeviation * deviationFactor);
        Serial.print(description);
        Serial.println("Max bounds");
    }
    else if (newReading < currentAverage - maxDeviation)
    {
        newReading = currentAverage - (maxDeviation * deviationFactor);
        Serial.print(description);
        Serial.println("Min bounds");
    }
    return factor * newReading + (1-factor) * currentAverage;
}

float average(const char *description, float currentAverage, float newReading, float factor) 
{
    return factor * newReading + (1-factor) * currentAverage;
}

uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}

unsigned long WS80::LastPacketReceivedMillis()
{
    return last_packet_received;
}

bool WS80::Setup(int id)
{
    _id = id;

    pinMode(GDO0_PIN, INPUT);
    pinMode(GDO2_PIN, INPUT);

    _num_readings = 0;

    Init();

    return true;
}

bool WS80::Init()
{
    ELECHOUSE_cc1101.setSpiPin(SDCARD_SCLK,SDCARD_MISO,SDCARD_MOSI,SDCARD_CS);

    if (ELECHOUSE_cc1101.getCC1101())
    { // Check the CC1101 Spi connection.
        Serial.println("CC1101 Connection OK");
    }
    else
    {
        Serial.println("CC1101 Connection Error");
        return false;
    }
/*
        .name        = "Acurite 592TXR Temp/Humidity, 592TX Temp, 5n1 Weather Station, 6045 Lightning, 899 Rain, 3N1, Atlas",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 220,  // short pulse is 220 us + 392 us gap
        .long_width  = 408,  // long pulse is 408 us + 204 us gap
        .sync_width  = 620,  // sync pulse is 620 us + 596 us gap
        .gap_limit   = 500,  // longest data gap is 392 us, sync gap is 596 us
        .reset_limit = 4000, // packet gap is 2192 us
        .decode_fn   = &acurite_txr_callback,
        .fields      = acurite_txr_output_fields,
*/
    ELECHOUSE_cc1101.Init();              // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setCCMode(1);       // set config for internal transmission mode.
    ELECHOUSE_cc1101.setModulation(2);  // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(433.92);   // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setSyncMode(2);  // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setCrc(1);      // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.

/*
        .name        = "Fine Offset Electronics WS80 weather station",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 58,
        .long_width  = 58,
        .reset_limit = 1500,
        .decode_fn   = &fineoffset_ws80_decode,
        .fields      = output_fields,

    ELECHOUSE_cc1101.Init();                  // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setModulation(0);        // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(915.0);           // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setDeviation(47.6);      // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setCCMode(1);            // set config for internal transmission mode.
    ELECHOUSE_cc1101.setRxBW(450);            // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    ELECHOUSE_cc1101.setDRate(17.241);        // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setSyncWord(0x2D, 0xD4); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
    ELECHOUSE_cc1101.setAdrChk(0);            // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    ELECHOUSE_cc1101.setAddr(0);              // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
    ELECHOUSE_cc1101.setWhiteData(0);         // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    ELECHOUSE_cc1101.setPktFormat(0);         // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    ELECHOUSE_cc1101.setFEC(0);               // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setPRE(0);               // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
    ELECHOUSE_cc1101.setPQT(0);               // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
    ELECHOUSE_cc1101.setAppendStatus(0);      // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

    ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x07);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, 0x02);  // 16/16 sync word bits detected
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x04); // Fixed packet length CRC enabled
    ELECHOUSE_cc1101.setPacketLength(32);                // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
*/
    ELECHOUSE_cc1101.SetRx(); // set Receive on

    // attachInterrupt(digitalPinToInterrupt(GDO0_PIN), packetReceived, RISING);

    return true;
}

bool WS80::Decode(unsigned char *b)
{

    //Verify checksum and CRC
    uint8_t crc = crc8(b, 17, 0x31, 0x00);
    uint8_t chk = add_bytes(b, 17);
    if (crc != 0 || chk != b[17])
    {
        Serial.println("CRC Check Failed");
        // Serial.print("CRC Check Failed: crc=");
        // Serial.print(crc);
        // Serial.print(", chk=");
        // Serial.print(chk);
        // Serial.print(", b=");
        // Serial.println(b[17]);
        return false;
    }

    int id = (b[1] << 16) | (b[2] << 8) | (b[3]);

    // if (id != _id) {
    //     return false;
    // }

    int light = (b[4] << 8) | (b[5]);
    // float light_wm2 = light_raw * 0.078925f; // W/m2
    int battery_mv = (b[6] * 20); // mV
    // int battery_lvl = _battery_mv < 1500 ? 0 : (_battery_mv - 1500.0) / 16.0; // 1.5V-3.0V is 0-100
    // int flags       = b[7]; // to find the wind msb
    int temp_raw = ((b[7] & 0x03) << 8) | (b[8]);
    float temp = (temp_raw - 400) * 0.1f;
    int humidity = (b[9]);
    float wind_avg = (((b[7] & 0x10) << 4) | (b[10])) * 0.1f;
    int wind_dir = ((b[7] & 0x20) << 3) | (b[11]);
    float wind_max = (((b[7] & 0x40) << 2) | (b[12])) * 0.1f;
    float uv_index = (b[13]) * 0.1f;

    if (_num_readings >= MAX_WS80_READINGS)
    {
        _num_readings = 0;
    }

    if (wind_avg > (wind_max + .1)) 
    {
        return false;
    }

    float factor = .7;

    WS80_Reading &reading = _readings[_num_readings];

    reading.id = id;
    reading.battery_mv = battery_mv;
    reading.temp = temp;
    reading.humidity = humidity;
    reading.wind_avg = wind_avg;
    reading.wind_max = wind_max;    
    reading.uv_index = uv_index;
    reading.light = light;
    reading.rssi = ELECHOUSE_cc1101.getRssi();    
    reading.wind_dir_x = cos(toRadians(wind_dir));
    reading.wind_dir_y = sin(toRadians(wind_dir));

    // Running filtered average of the wind direction vector
    // _wind_dir_x = average("Wind Dir X:", _wind_dir_x, cos(toRadians(wind_dir)), factor);
    // _wind_dir_y = average("Wind Dir Y:", _wind_dir_y, sin(toRadians(wind_dir)), factor);

    // Serial.print("Id: ");
    // Serial.println(reading.id);

    // Serial.print("Temp: ");
    // Serial.println(reading.temp);

    // Serial.print("Humidity: ");
    // Serial.println(reading.humidity);

    // Serial.print("Wind Dir: ");
    // Serial.println(wind_dir);

    // Serial.print("Avg Wind: ");
    // Serial.println(reading.wind_avg);

    // Serial.print("Max Wind: ");
    // Serial.println(reading.wind_max);

    // Serial.print("Light: ");
    // Serial.println(reading.light);

    // Serial.print("UV Index: ");
    // Serial.println(reading.uv_index);

    // Serial.print("Battery: ");
    // Serial.println(reading.battery_mv);

    _num_readings++;

    return true;
}

byte buffer[256] = {0};

unsigned long _lastTime = 0;

void WS80::Loop()
{
    // if (packetWaiting) {
    //     Serial.print("CC1101: Data Packet Waiting");
    //     ReadDataPacket();
    //     packetWaiting = false;
    // }
}

bool WS80::ReadDataPacket()
{
    // int len = 0;
    int len = ELECHOUSE_cc1101.ReceiveData(buffer);

    Serial.print("CC1101: Packet:");
    Serial.print(_last_rssi);
    Serial.print(",");
    Serial.println(len);

    if (len == 128)
    {        
        Serial.print("CC1101: Reading Data: ");
        Serial.println(_last_rssi);
        // Serial.print(",");
        // Serial.println(len);

        if (len > 18)
        {
            len = 18;
        }

        byte decode_buffer[19];

        decode_buffer[0] = 0x80;

        // char hex_buffer[3];

        for (int i = 0; i < len; i++)
        {
            decode_buffer[i + 1] = buffer[i];
            // sprintf (hex_buffer, "%02x", buffer[i]);
            // Serial.print(hex_buffer);
        }

        if (!Decode(decode_buffer))
        {
            return false;
        }

        Serial.println();

        return true;
    }

    return false;
}

unsigned long start_wait;
unsigned int tries = 0;

bool WS80::CheckForData(WeatherStationData *wsData)
{
    if (ReadDataPacket())
    {
        return true;
    }
    // wsData->status = STATUS_SENSOR_DATA_INVALID;
    return false;
}

bool WS80::WaitForData(WeatherStationData *wsData, long timout)
{
    unsigned long now = millis();
    start_wait = now;
    tries = 0;

    ELECHOUSE_cc1101.SetRx(); // Set Receive on

    if (!ELECHOUSE_cc1101.getCC1101())
    {
        wsData->status = STATUS_SENSOR_NOT_FOUND;
        Serial.println("CC1101 Not Found");
        return false;
    }

    while (millis() - start_wait < timout)
    {
        if (ELECHOUSE_cc1101.CheckRxFifo(0))
        {
            _last_rssi = ELECHOUSE_cc1101.getRssi();
            // Serial.println("CC1101 Packet Received");
            if (ReadDataPacket())
            {
                // Serial.println(_num_readings);
                ELECHOUSE_cc1101.goSleep();
                return true;
            }
            else
            {
                // wsData->status = STATUS_SENSOR_DATA_INVALID;
                ELECHOUSE_cc1101.goSleep();
                return false;
            }
        }
        delay(100);
    }

    ELECHOUSE_cc1101.Reset();
    Init();
    wsData->status = STATUS_SENSOR_RESET;
    return 0;
}

void WS80::SetStationData(WeatherStationData *wsData)
{
    int light = 0;
    int battery_mv = 0;
    float temp = 0;
    float humidity = 0;
    float wind_dir = 0;
    float wind_avg = 0;
    float wind_max = 0;
    float wind_min = 50;
    float uv_index = 0;
    float wind_dir_x = 0;
    float wind_dir_y = 0;
    size_t valid_reading_count = _num_readings;

    Serial.print("Setting Station Data: ");
    Serial.println(_num_readings);

    if (_num_readings > 1)
    {
        float temps[MAX_WS80_READINGS];
        float wind_maxes[MAX_WS80_READINGS];
        float wind_averages[MAX_WS80_READINGS];
        float humidities[MAX_WS80_READINGS];
        float wind_x[MAX_WS80_READINGS];
        float wind_y[MAX_WS80_READINGS];

        for (int i = 0; i < _num_readings; i++)
        {
            WS80_Reading &reading = _readings[i];

            // Serial.print("wind_max: ");
            // Serial.println(reading.wind_max);

            temps[i] = reading.temp;
            wind_averages[i] = reading.wind_avg;
            wind_maxes[i] = reading.wind_max;
            humidities[i] = (float)reading.humidity;
            wind_x[i] = reading.wind_dir_x;
            wind_y[i] = reading.wind_dir_y;
        }

        // combSort(temps, _num_readings);
        // combSort(wind_averages, _num_readings);
        // combSort(wind_maxes, _num_readings);
        // combSort(humidities, _num_readings);

        WS80_Reading &reading = _readings[_num_readings-1];

        size_t filtered_count = 0;
        float dummy1 = 0;
        float dummy2 = 0;            

        light = reading.light;
        uv_index = reading.uv_index;
        light = reading.light;
        battery_mv = reading.battery_mv;        
        // wind_dir = vectorToDegree(_wind_dir_x, _wind_dir_y);

        // filtered_count = findFilteredAverageMinMax(temps,_num_readings, temp, dummy1, dummy2);
        // valid_reading_count = filtered_count < valid_reading_count ? filtered_count : valid_reading_count;

        // wind_min = 100;
        // filtered_count = findFilteredAverageMinMax(wind_averages,_num_readings, wind_avg, wind_min, dummy1);
        // valid_reading_count = filtered_count < valid_reading_count ? filtered_count : valid_reading_count;
        // wind_min = wind_min == 100 ? 0 : wind_min;

        // filtered_count = findFilteredAverageMinMax(wind_maxes,_num_readings, wind_max, dummy1, dummy2);
        // valid_reading_count = filtered_count < valid_reading_count ? filtered_count : valid_reading_count;

        // filtered_count = findFilteredAverageMinMax(humidities,_num_readings, humidity, dummy1, dummy2);    
        // valid_reading_count = filtered_count < valid_reading_count ? filtered_count : valid_reading_count;

        findAverageMinMax(temps,_num_readings, temp, dummy1, dummy2);
        findAverageMinMax(wind_averages,_num_readings, wind_avg, wind_min, dummy1);
        findAverageMinMax(wind_maxes,_num_readings, wind_max, dummy1, wind_max);
        findAverageMinMax(humidities,_num_readings, humidity, dummy1, dummy2);
        findAverageMinMax(wind_x,_num_readings, wind_dir_x, dummy1, dummy2);
        findAverageMinMax(wind_y,_num_readings, wind_dir_y, dummy1, dummy2);

        wind_dir = vectorToDegree(wind_dir_x, wind_dir_y);
    }
    else 
    {
        wsData->status = STATUS_NO_SENSOR_DATA;
    }

    wsData->avgWindDirection = wind_dir;
    wsData->minWindDirection = 0;
    wsData->maxWindDirection = 0;
    wsData->maxWindSpeed = wind_max;
    wsData->avgWindSpeed = wind_avg;
    wsData->minWindSpeed = wind_min;
    wsData->light = light;
    wsData->humidity = humidity;
    wsData->temp = temp;
    wsData->uv_index = uv_index;
    wsData->sensor_battery = battery_mv;
    wsData->sensor_rssi = _last_rssi * -1;
    wsData->sensor_readings = _num_readings;
    wsData->sensor_valid_readings = valid_reading_count;

    _num_readings = 0;
}
