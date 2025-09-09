import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.core import CORE

CODEOWNERS = ["@esphome/core"]

CONF_IMPLEMENTATION = "implementation"
CONF_BORDER_ROUTER_ADDRESS = "border_router_address"
IMPLEMENTATION_LWIP_TCP = "lwip_tcp"
IMPLEMENTATION_LWIP_SOCKETS = "lwip_sockets"
IMPLEMENTATION_BSD_SOCKETS = "bsd_sockets"
IMPLEMENTATION_MESHMESH_8266 = "meshmesh_esp8266"
IMPLEMENTATION_MESHMESH_ESP32 = "meshmesh_esp32"
IMPLEMENTATION_BORDER_ROUTER = "border_router"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.SplitDefault(
            CONF_IMPLEMENTATION,
            esp8266=IMPLEMENTATION_LWIP_TCP,
            esp32=IMPLEMENTATION_BSD_SOCKETS,
            rp2040=IMPLEMENTATION_LWIP_TCP,
            bk72xx=IMPLEMENTATION_LWIP_SOCKETS,
            ln882x=IMPLEMENTATION_LWIP_SOCKETS,
            rtl87xx=IMPLEMENTATION_LWIP_SOCKETS,
            host=IMPLEMENTATION_BSD_SOCKETS,
        ): cv.one_of(
            IMPLEMENTATION_LWIP_TCP,
            IMPLEMENTATION_LWIP_SOCKETS,
            IMPLEMENTATION_BSD_SOCKETS,
            IMPLEMENTATION_MESHMESH_8266,
            IMPLEMENTATION_MESHMESH_ESP32,
            IMPLEMENTATION_BORDER_ROUTER,
            lower=True,
            space="_",
        ),
        cv.Optional(CONF_BORDER_ROUTER_ADDRESS): cv.positive_int,
    }
)


async def to_code(config):
    impl = config[CONF_IMPLEMENTATION]
    if impl == IMPLEMENTATION_LWIP_TCP:
        cg.add_define("USE_SOCKET_IMPL_LWIP_TCP")
    elif impl == IMPLEMENTATION_LWIP_SOCKETS:
        cg.add_define("USE_SOCKET_IMPL_LWIP_SOCKETS")
        cg.add_define("USE_SOCKET_SELECT_SUPPORT")
    elif impl == IMPLEMENTATION_BSD_SOCKETS:
        cg.add_define("USE_SOCKET_IMPL_BSD_SOCKETS")
        cg.add_define("USE_SOCKET_SELECT_SUPPORT")
    elif impl == IMPLEMENTATION_MESHMESH_8266:
        cg.add_define("USE_SOCKET_IMPL_MESHMESH_8266")
    elif impl == IMPLEMENTATION_MESHMESH_ESP32:
        cg.add_define("USE_SOCKET_IMPL_MESHMESH_8266")
    elif impl == IMPLEMENTATION_BORDER_ROUTER:
        cg.add_define("USE_SOCKET_IMPL_BORDER_ROUTER")
        if CONF_BORDER_ROUTER_ADDRESS in config:
            address = config[CONF_BORDER_ROUTER_ADDRESS]
            cg.add_define("USE_BORDER_ROUTER_ADDRESS", address)
        else:
            raise cv.Invalid(
                "The 'border_router_address' must be provided when using the 'border_router' socket implementation."
            )

    if CONF_BORDER_ROUTER_ADDRESS in config and impl != IMPLEMENTATION_BORDER_ROUTER:
        raise cv.Invalid(
            "The 'border_router_address' can only be used with the 'border_router' socket implementation."
        )


def FILTER_SOURCE_FILES() -> list[str]:
    """Return list of socket implementation files that aren't selected by the user."""
    impl = CORE.config["socket"][CONF_IMPLEMENTATION]

    # Build list of files to exclude based on selected implementation
    excluded = []
    if impl != IMPLEMENTATION_LWIP_TCP:
        excluded.append("lwip_raw_tcp_impl.cpp")
    if impl != IMPLEMENTATION_BSD_SOCKETS:
        excluded.append("bsd_sockets_impl.cpp")
    if impl != IMPLEMENTATION_LWIP_SOCKETS:
        excluded.append("lwip_sockets_impl.cpp")
    if impl not in (IMPLEMENTATION_MESHMESH_8266, IMPLEMENTATION_MESHMESH_ESP32):
        excluded.append("meshmesh_raw_tcp_impl.cpp")
    if impl != IMPLEMENTATION_BORDER_ROUTER:
        excluded.append("border_router_client_impl.cpp")
    return excluded
