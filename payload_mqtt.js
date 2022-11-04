function decodeUplink(input) {
    return {
      data: {
        SoilMoisture: input.bytes[0],
        SoilTemperature: input.bytes[1],
        AirTemperature: input.bytes[2],
        AirPressure: ((input.bytes[3] << 24) + (input.bytes[4] << 16) + (input.bytes[5] << 8) + (input.bytes[6]))/ 100,
        AirHumidity: input.bytes[7],
        GasResistance: ((input.bytes[8] << 24) + (input.bytes[9] << 16) + (input.bytes[10] << 8) + (input.bytes[11]))/ 1000,
      },
      warnings: [],
      errors: []
    };
  }