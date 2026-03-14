# ESPHome BNO085 External Component

This component adds BNO085 IMU support to ESPHome as an external component.

## Status

- Supported transport: I2C
- Tested address default: `0x4A`
- Framework requirement: Arduino (uses Adafruit BNO08x library)

## Import from GitHub

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/timmchugh11/Chinese-Diesel-Heater---ESPHome
      ref: main
    components: [ bno085 ]
```

## Basic YAML

```yaml
i2c:
  sda: 21
  scl: 22

bno085:
  id: imu
  address: 0x4A
  update_interval: 100ms

  roll:
    name: "BNO085 Roll"
  pitch:
    name: "BNO085 Pitch"
  yaw:
    name: "BNO085 Yaw"
```

## Full Sensor Options

- `roll`, `pitch`, `yaw`
- `accel_x`, `accel_y`, `accel_z`
- `gyro_x`, `gyro_y`, `gyro_z`

Optional timing settings:

- `rotation_vector_interval` (default `50ms`)
- `accelerometer_interval` (default `50ms`)
- `gyroscope_interval` (default `50ms`)
