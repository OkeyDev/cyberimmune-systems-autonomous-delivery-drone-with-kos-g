project (PeripheryController)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (PERIPHERY_CONTROLLER_SRC "src/periphery_controller_simulator.cpp")
else()
    set (PERIPHERY_CONTROLLER_SRC "src/periphery_controller_real.cpp")
endif()

add_executable (PeripheryController "src/main.cpp" "src/periphery_controller.cpp"
    ${PERIPHERY_CONTROLLER_SRC} "../shared/src/ipc_messages_logger.cpp"
    "../shared/src/ipc_messages_server_connector.cpp")
add_dependencies (PeripheryController PERIPHERY_CONTROLLER_EDL)

target_compile_definitions (PeripheryController PRIVATE ENTITY_NAME="Periphery Controller")

if (SIMULATION)
    target_sources (PeripheryController PUBLIC "../shared/src/ipc_messages_navigation_system.cpp")
    target_compile_definitions (PeripheryController PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    bind_nk_targets (PeripheryController vfs::client PERIPHERY_CONTROLLER_EDL)
else ()
    bind_nk_targets (PeripheryController bsp::client gpio::client PERIPHERY_CONTROLLER_EDL)
endif()
bind_nk_targets (PeripheryController ${CMAKE_THREAD_LIBS_INIT} PERIPHERY_CONTROLLER_EDL)

if (INSPECTOR_ROLE)
    target_compile_definitions (PeripheryController PRIVATE IS_INSPECTOR=TRUE)
endif()