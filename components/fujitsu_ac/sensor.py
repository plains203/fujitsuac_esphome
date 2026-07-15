import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import CONF_FUJITSU_AC_ID, FujitsuAC

DEPENDENCIES = ["fujitsu_ac"]

CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_INDOOR_TEMPERATURE = "indoor_temperature"

TEMP_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
    accuracy_decimals=1,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_FUJITSU_AC_ID): cv.use_id(FujitsuAC),
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): TEMP_SCHEMA,
        cv.Optional(CONF_INDOOR_TEMPERATURE): TEMP_SCHEMA,
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_FUJITSU_AC_ID])
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(hub.set_outdoor_sensor(sens))
    if CONF_INDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_TEMPERATURE])
        cg.add(hub.set_indoor_sensor(sens))
