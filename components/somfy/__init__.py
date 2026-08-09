import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import remote_receiver, remote_transmitter
from esphome.const import CONF_ID, CONF_TYPE, PLATFORM_ESP32
from esphome.core import ID

CODEOWNERS = ["@LeonardPitzu"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["button"]
MULTI_CONF = True

DOMAIN = "somfy"

somfy_ns = cg.esphome_ns.namespace("somfy")
SomfyRtsHub = somfy_ns.class_("SomfyRtsHub", cg.Component)
SomfyIohcHub = somfy_ns.class_("SomfyIohcHub", cg.Component)
Cc1101IohcRadio = somfy_ns.class_("Cc1101IohcRadio")
Sx126xIohcRadio = somfy_ns.class_("Sx126xIohcRadio")

CONF_REMOTE_TRANSMITTER = "remote_transmitter"
CONF_REMOTE_RECEIVER = "remote_receiver"
CONF_CC1101_ID = "cc1101_id"
CONF_SX126X_ID = "sx126x_id"

TYPE_RTS = "rts"
TYPE_IOHC = "iohc"

RTS_HUB_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyRtsHub),
        cv.Required(CONF_REMOTE_TRANSMITTER): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
        cv.Optional(CONF_REMOTE_RECEIVER): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def validate_iohc_radio(config):
    """Exactly one of cc1101_id / sx126x_id selects the radio backend."""
    has_cc1101 = CONF_CC1101_ID in config
    has_sx126x = CONF_SX126X_ID in config
    if has_cc1101 == has_sx126x:
        raise cv.Invalid(
            f"Exactly one of '{CONF_CC1101_ID}' or '{CONF_SX126X_ID}' is required"
        )
    return config


IOHC_HUB_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SomfyIohcHub),
            cv.Optional(CONF_CC1101_ID): cv.use_id(cg.Component),
            cv.Optional(CONF_SX126X_ID): cv.use_id(cg.Component),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_iohc_radio,
)

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            TYPE_RTS: RTS_HUB_SCHEMA,
            TYPE_IOHC: IOHC_HUB_SCHEMA,
        },
    ),
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    typ = config[CONF_TYPE]

    if typ == TYPE_RTS:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)
        cg.add_define("USE_SOMFY_RTS")

        tx = await cg.get_variable(config[CONF_REMOTE_TRANSMITTER])
        cg.add(var.set_remote_transmitter(tx))

        if CONF_REMOTE_RECEIVER in config:
            rx = await cg.get_variable(config[CONF_REMOTE_RECEIVER])
            cg.add(var.set_remote_receiver(rx))
            cg.add_define("USE_SOMFY_COVER_RX")

    elif typ == TYPE_IOHC:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)

        if CONF_CC1101_ID in config:
            cg.add_define("USE_SOMFY_IOHC_CC1101")
            radio_id = ID(f"{config[CONF_ID]}_radio", is_declaration=True, type=Cc1101IohcRadio)
            radio = cg.new_Pvariable(radio_id)
            cc1101 = await cg.get_variable(config[CONF_CC1101_ID])
            cg.add(radio.set_cc1101(cc1101))
        else:
            cg.add_define("USE_SOMFY_IOHC_SX126X")
            radio_id = ID(f"{config[CONF_ID]}_radio", is_declaration=True, type=Sx126xIohcRadio)
            radio = cg.new_Pvariable(radio_id)
            sx126x = await cg.get_variable(config[CONF_SX126X_ID])
            cg.add(radio.set_sx126x(sx126x))

        cg.add(var.set_radio(radio))
        cg.add_define("USE_SOMFY_IOHC")
