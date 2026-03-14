# ESPHome BNO085 External Component

This component adds BNO085 IMU support to ESPHome as an external component.

## Status

- Supported transport: I2C
- Native SH2 driver (no Arduino runtime dependency)
- Works with ESP-IDF (ESP32)
- Default address: `0x4A` (some boards use `0x4B`)

## Import from GitHub

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/timmchugh11/bno085---ESPHome
      ref: main
    components: [ bno085 ]
```

For stable builds, pin `ref` to a commit SHA instead of `main`.

## Basic YAML

```yaml
i2c:
  sda: 8
  scl: 9
  scan: true

bno085:
  id: imu
  address: 0x4A
  update_interval: 1s
  rotation_vector_interval: 1s
  accelerometer_interval: 1s
  gyroscope_interval: 1s

  roll:
    name: "BNO085 Roll"
  pitch:
    name: "BNO085 Pitch"
  yaw:
    name: "BNO085 Yaw"

  accel_x:
    name: "BNO085 Accel X"
  accel_y:
    name: "BNO085 Accel Y"
  accel_z:
    name: "BNO085 Accel Z"

  gyro_x:
    name: "BNO085 Gyro X"
  gyro_y:
    name: "BNO085 Gyro Y"
  gyro_z:
    name: "BNO085 Gyro Z"
```

## Full Sensor Options

- `roll`, `pitch`, `yaw`
- `accel_x`, `accel_y`, `accel_z`
- `gyro_x`, `gyro_y`, `gyro_z`

Optional timing settings:

- `rotation_vector_interval` (default `50ms`)
- `accelerometer_interval` (default `50ms`)
- `gyroscope_interval` (default `50ms`)

## Address Detection

Enable `i2c.scan: true` and check logs at boot.

Expected BNO08x addresses are usually:

- `0x4A`
- `0x4B`

If neither appears in the scan, check wiring, pull-ups, power, and ground.

## Notes

- Component implementation uses vendored Hillcrest SH2 sources in this repository.
- No PlatformIO Adafruit library install is required for this component.
