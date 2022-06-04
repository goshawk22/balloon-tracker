// Decoder for MaxPlastix mappers
//
// 11 Byte payload: 
// 3 Lat, 3 Long, 2 Altitude (m), 1 Speed (km/hr), 1 Battery, 1 Sats.
// Accuracy is a dummy value required by some Integrations.
// Battery is 1/100 of a volt, offset by 2v for a range of 2.00 to 4.56 volts.
//
function Decoder(bytes, port) {
  var decoded = {};

  // All formats carry a lat & lon reading:
  var latitude = ((bytes[0] << 16) >>> 0) + ((bytes[1] << 8) >>> 0) + bytes[2];
  latitude = (latitude / 16777215.0 * 180) - 90;

  var longitude = ((bytes[3] << 16) >>> 0) + ((bytes[4] << 8) >>> 0) + bytes[5];
  longitude = (longitude / 16777215.0 * 360) - 180;

  location = latitude + ", " + longitude;

  switch (port) {
    case 2: // GPS!
      decoded.latitude = latitude;
      decoded.longitude = longitude;
      decoded.location = location;

      var altValue = ((bytes[6] << 8) >>> 0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if (sign)
        decoded.altitude = 0xFFFF0000 | altValue;
      else
        decoded.altitude = altValue;

      decoded.speed = parseFloat((((bytes[8])) / 1.609).toFixed(2));
      decoded.battery = parseFloat((bytes[9] / 100 + 2).toFixed(2));
      decoded.sats = bytes[10];
      decoded.uptime = (bytes[11] << 8) + bytes[12];
      decoded.minutes_lost = (bytes[13] << 8) + bytes[14];
      decoded.temperature = ((bytes[15] << 8) + bytes[16]) / 100.0
      decoded.pressure = (bytes[17] << 16) + (bytes[18] << 8) + bytes[19];
      decoded.humidity = ((bytes[20] << 8) + bytes[21]) / 100.0;
      decoded.uv = (bytes[22] << 8) + bytes[23];
      
      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      decoded.status = "GPS";
      break;
    case 5: // System status
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.battery = parseFloat((bytes[6] / 100 + 2).toFixed(2));
      decoded.uptime = bytes[7];
      decoded.status = "BOOTED";
      break;
    case 6: // Lost GPS
      decoded.last_latitude = latitude;
      decoded.last_longitude = longitude;
      decoded.last_location = location;

      var altValue = ((bytes[6] << 8) >>> 0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if (sign)
        decoded.altitude = 0xFFFF0000 | altValue;
      else
        decoded.altitude = altValue;

      decoded.speed = parseFloat((((bytes[8])) / 1.609).toFixed(2));
      decoded.battery = parseFloat((bytes[9] / 100 + 2).toFixed(2));
      decoded.sats = bytes[10];
      decoded.uptime = (bytes[11] << 8) + bytes[12];
      decoded.minutes_lost = (bytes[13] << 8) + bytes[14];
      decoded.temperature = ((bytes[15] << 8) + bytes[16]) / 100.0
      decoded.pressure = (bytes[17] << 16) + (bytes[18] << 8) + bytes[19];
      decoded.humidity = ((bytes[20] << 8) + bytes[21]) / 100.0;
      decoded.uv = (bytes[22] << 8) + bytes[23];

      decoded.accuracy = 2.5; // Bogus Accuracy required by Cargo/Mapper integration
      decoded.status = "LOST GPS";
      break;
  }

  return decoded;
}