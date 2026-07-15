import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import CONF_FUJITSU_AC_ID, Feature, FujitsuAC, fujitsu_ac_ns

DEPENDENCIES = ["fujitsu_ac"]

FujitsuSwitch = fujitsu_ac_ns.class_(
    "FujitsuSwitch", switch.Switch, cg.Parented.template(FujitsuAC)
)

# config key -> (Feature enum member, icon)
FEATURES = {
    "powerful": (Feature.POWERFUL, "mdi:rocket-launch"),
    "economy": (Feature.ECONOMY, "mdi:leaf"),
    "energy_saving_fan": (Feature.ENERGY_SAVING_FAN, "mdi:fan-chevron-down"),
    "outdoor_unit_low_noise": (Feature.OUTDOOR_LOW_NOISE, "mdi:volume-low"),
    "coil_dry": (Feature.COIL_DRY, "mdi:air-humidifier-off"),
    "human_sensor": (Feature.HUMAN_SENSOR, "mdi:motion-sensor"),
    "minimum_heat": (Feature.MINIMUM_HEAT, "mdi:snowflake-melt"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_FUJITSU_AC_ID): cv.use_id(FujitsuAC),
        **{
            cv.Optional(key): switch.switch_schema(
                FujitsuSwitch, icon=icon, default_restore_mode="DISABLED"
            )
            for key, (_, icon) in FEATURES.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_FUJITSU_AC_ID])
    for key, (feature, _) in FEATURES.items():
        if key not in config:
            continue
        var = await switch.new_switch(config[key])
        await cg.register_parented(var, config[CONF_FUJITSU_AC_ID])
        cg.add(var.set_feature(feature))
        cg.add(hub.register_switch(var))
