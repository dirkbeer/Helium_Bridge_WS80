function Decoder(bytes, port, uplink_info) {

/*
  The uplink_info variable is an OPTIONAL third parameter that provides the following:

  uplink_info = {
    type: "join",
    uuid: <UUIDv4>,
    id: <device id>,
    name: <device name>,
    dev_eui: <dev_eui>,
    app_eui: <app_eui>,
    metadata: {...},
    fcnt: <integer>,
    reported_at: <timestamp>,
    port: <integer>,
    devaddr: <devaddr>,
    hotspots: {...},
    hold_time: <integer>
  }
*/
    if (uplink_info) {
        // do something with uplink_info fields
    }

    const data = String.fromCharCode.apply(null, bytes);
    const parts = data.split("|").map(Number);

    const decoded = {
        id: parts[0],
        status: parts[1],
        battery: parts[2] / 100,
        solar: parts[3] / 10,
        rssi: parts[4],
        minWindSpeed: parts[5] / (10 * 3.6),
        avgWindSpeed: parts[6] / (10 * 3.6),
        maxWindSpeed: parts[7] / (10 * 3.6),
        minWindDirection: parts[8],
        avgWindDirection: parts[9],
        maxWindDirection: parts[10],
        temp: parts[11] / 10,
        pressure: parts[12] / 10,
        humidity: parts[13],
        light: parts[14],
        uv_index: parts[15] / 10,
        sensor_rssi: -parts[16],
        sensor_battery: parts[17],
        sensor_readings: parts[18],
        sensor_valid_readings: parts[19],
        case_temp: parts[20] / 10
    };

    return decoded;
}
