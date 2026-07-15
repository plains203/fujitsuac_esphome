import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import CONF_FUJITSU_AC_ID, FujitsuAC, fujitsu_ac_ns

DEPENDENCIES = ["fujitsu_ac"]

CONF_VERTICAL_AIRFLOW = "vertical_airflow"
CONF_HORIZONTAL_AIRFLOW = "horizontal_airflow"

FujitsuSelect = fujitsu_ac_ns.class_(
    "FujitsuSelect", select.Select, cg.Parented.template(FujitsuAC)
)

AIRFLOW_OPTIONS = [
    "Position 1",
    "Position 2",
    "Position 3",
    "Position 4",
    "Position 5",
    "Position 6",
    "Swing",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_FUJITSU_AC_ID): cv.use_id(FujitsuAC),
        cv.Optional(CONF_VERTICAL_AIRFLOW): select.select_schema(
            FujitsuSelect, icon="mdi:arrow-up-down"
        ),
        cv.Optional(CONF_HORIZONTAL_AIRFLOW): select.select_schema(
            FujitsuSelect, icon="mdi:arrow-left-right"
        ),
    }
)


async def _make_select(hub, hub_id, conf, horizontal):
    var = await select.new_select(conf, options=AIRFLOW_OPTIONS)
    await cg.register_parented(var, hub_id)
    cg.add(var.set_horizontal(horizontal))
    if horizontal:
        cg.add(hub.set_horizontal_select(var))
    else:
        cg.add(hub.set_vertical_select(var))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_FUJITSU_AC_ID])
    if CONF_VERTICAL_AIRFLOW in config:
        await _make_select(hub, config[CONF_FUJITSU_AC_ID], config[CONF_VERTICAL_AIRFLOW], False)
    if CONF_HORIZONTAL_AIRFLOW in config:
        await _make_select(hub, config[CONF_FUJITSU_AC_ID], config[CONF_HORIZONTAL_AIRFLOW], True)
