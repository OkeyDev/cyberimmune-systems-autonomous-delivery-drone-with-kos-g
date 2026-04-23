project (ServerConnector)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SERVER)
    set (SERVER_CONNECTOR_SRC "src/server_connector_online.cpp")
else()
    set (SERVER_CONNECTOR_SRC "src/server_connector_offline.cpp")
endif()

add_executable (ServerConnector "src/main.cpp" "src/server_connector.cpp"
    ${SERVER_CONNECTOR_SRC} "../shared/src/ipc_messages_logger.cpp")
add_dependencies (ServerConnector SERVER_CONNECTOR_EDL)

target_compile_definitions (ServerConnector PRIVATE ENTITY_NAME="Server Connector")
target_compile_definitions (ServerConnector PRIVATE BOARD_ID="${BOARD_ID}")
target_compile_definitions (ServerConnector PRIVATE PARTNER_ID="${PARTNER_ID}")
if (SIMULATION)
    target_compile_definitions (ServerConnector PRIVATE WLAN_INTERFACE="en0")
else()
    target_compile_definitions (ServerConnector PRIVATE WLAN_INTERFACE="wl0")
endif()

if (SERVER)
    target_compile_definitions (ServerConnector PRIVATE 
        SERVER_IP="${SERVER_IP}" 
        MQTT_IP="${MQTT_IP}" 
        MQTT_USERNAME="${MQTT_USERNAME}"
        MQTT_PASSWORD="${MQTT_PASSWORD}"
    )
    bind_nk_targets (ServerConnector vfs::client ${wpa_CLIENT_LIB} ${mosquitto_CXX_LIB} SERVER_CONNECTOR_EDL)
    set_target_properties (ServerConnector PROPERTIES vfs::entity_REPLACEMENT "")
else ()
    target_compile_definitions (ServerConnector PRIVATE NO_SERVER)
    bind_nk_targets (ServerConnector SERVER_CONNECTOR_EDL)
endif()

if (INSPECTOR_ROLE)
    target_compile_definitions (ServerConnector PRIVATE IS_INSPECTOR=TRUE)
endif()