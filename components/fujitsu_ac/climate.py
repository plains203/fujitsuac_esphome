import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate

from . import CONF_FUJITSU_AC_ID, FujitsuAC, fujitsu_ac_ns

DEPENDENCIES = ["fujitsu_ac"]

FujitsuClimate = fujitsu_ac_ns.class_("FujitsuClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.climate_schema(FujitsuClimate).extend(
    {
        cv.GenerateID(CONF_FUJITSU_AC_ID): cv.use_id(FujitsuAC),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_FUJITSU_AC_ID])
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    cg.add(hub.set_climate(var))
