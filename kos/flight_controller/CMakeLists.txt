project (FlightController)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

add_executable (FlightController "src/flight_controller.cpp" "../shared/src/ipc_messages_autopilot_connector.cpp"
    "../shared/src/ipc_messages_credential_manager.cpp" "../shared/src/ipc_messages_navigation_system.cpp"
    "../shared/src/ipc_messages_periphery_controller.cpp" "../shared/src/ipc_messages_server_connector.cpp"
    "../shared/src/ipc_messages_logger.cpp" "src/drone_defender_system.cpp")
add_dependencies (FlightController FLIGHT_CONTROLLER_EDL)

if (INSPECTOR_ROLE)
    target_sources (FlightController PUBLIC "src/main_inspector.cpp")
else ()
    target_sources (FlightController PUBLIC "src/main_deliverer.cpp")
endif()

target_compile_definitions (FlightController PRIVATE ENTITY_NAME="Flight Controller")
target_compile_definitions (FlightController PRIVATE PARTNER_ID="${PARTNER_ID}")
bind_nk_targets (FlightController MbedTLS::mbedtls ${CMAKE_THREAD_LIBS_INIT} FLIGHT_CONTROLLER_EDL)
