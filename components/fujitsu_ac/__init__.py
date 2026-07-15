import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@james"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_FUJITSU_AC_ID = "fujitsu_ac_id"
CONF_ENABLE_MINIMUM_HEAT_CONTROL = "enable_minimum_heat_control"

fujitsu_ac_ns = cg.esphome_ns.namespace("fujitsu_ac")
FujitsuAC = fujitsu_ac_ns.class_("FujitsuAC", cg.Component, uart.UARTDevice)
Feature = fujitsu_ac_ns.enum("Feature", is_class=True)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FujitsuAC),
            cv.Optional(CONF_ENABLE_MINIMUM_HEAT_CONTROL, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "fujitsu_ac",
    require_tx=True,
    require_rx=True,
    baud_rate=9600,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_allow_minimum_heat_write(config[CONF_ENABLE_MINIMUM_HEAT_CONTROL]))
