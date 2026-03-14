import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    UNIT_METER_PER_SECOND_SQUARED,
)

CODEOWNERS = ["@timmchugh11"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

bno085_ns = cg.esphome_ns.namespace("bno085")
BNO085Component = bno085_ns.class_(
    "BNO085Component", cg.PollingComponent, i2c.I2CDevice
)

CONF_ROLL = "roll"
CONF_PITCH = "pitch"
CONF_YAW = "yaw"
CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_ROTATION_VECTOR_INTERVAL = "rotation_vector_interval"
CONF_ACCELEROMETER_INTERVAL = "accelerometer_interval"
CONF_GYROSCOPE_INTERVAL = "gyroscope_interval"

ANGLE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

ACCEL_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)

GYRO_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="deg/s",
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BNO085Component),
            cv.Optional(CONF_ROLL): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_PITCH): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_YAW): ANGLE_SENSOR_SCHEMA,
            cv.Optional(CONF_ACCEL_X): ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_ACCEL_Y): ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_ACCEL_Z): ACCEL_SENSOR_SCHEMA,
            cv.Optional(CONF_GYRO_X): GYRO_SENSOR_SCHEMA,
            cv.Optional(CONF_GYRO_Y): GYRO_SENSOR_SCHEMA,
            cv.Optional(CONF_GYRO_Z): GYRO_SENSOR_SCHEMA,
            cv.Optional(
                CONF_ROTATION_VECTOR_INTERVAL, default="50ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_ACCELEROMETER_INTERVAL, default="50ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_GYROSCOPE_INTERVAL, default="50ms"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.polling_component_schema("100ms"))
    .extend(i2c.i2c_device_schema(0x4A))
)


async def to_code(config):
    cg.add_library("adafruit/Adafruit BNO08x", "^1.2.5")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_ROLL in config:
        sens = await sensor.new_sensor(config[CONF_ROLL])
        cg.add(var.set_roll_sensor(sens))
    if CONF_PITCH in config:
        sens = await sensor.new_sensor(config[CONF_PITCH])
        cg.add(var.set_pitch_sensor(sens))
    if CONF_YAW in config:
        sens = await sensor.new_sensor(config[CONF_YAW])
        cg.add(var.set_yaw_sensor(sens))

    if CONF_ACCEL_X in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_X])
        cg.add(var.set_accel_x_sensor(sens))
    if CONF_ACCEL_Y in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_Y])
        cg.add(var.set_accel_y_sensor(sens))
    if CONF_ACCEL_Z in config:
        sens = await sensor.new_sensor(config[CONF_ACCEL_Z])
        cg.add(var.set_accel_z_sensor(sens))

    if CONF_GYRO_X in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_X])
        cg.add(var.set_gyro_x_sensor(sens))
    if CONF_GYRO_Y in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_Y])
        cg.add(var.set_gyro_y_sensor(sens))
    if CONF_GYRO_Z in config:
        sens = await sensor.new_sensor(config[CONF_GYRO_Z])
        cg.add(var.set_gyro_z_sensor(sens))

    cg.add(
        var.set_rotation_vector_interval_ms(
            config[CONF_ROTATION_VECTOR_INTERVAL].total_milliseconds
        )
    )
    cg.add(
        var.set_accelerometer_interval_ms(
            config[CONF_ACCELEROMETER_INTERVAL].total_milliseconds
        )
    )
    cg.add(
        var.set_gyroscope_interval_ms(
            config[CONF_GYROSCOPE_INTERVAL].total_milliseconds
        )
    )
