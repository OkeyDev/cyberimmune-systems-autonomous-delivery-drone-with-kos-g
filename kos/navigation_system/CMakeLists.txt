project (NavigationSystem)

include (platform/nk)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

if (SIMULATION)
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_simulator.cpp")
else ()
    set (NAVIGATION_SYSTEM_SRC "src/navigation_system_real.cpp")
endif ()

add_executable (NavigationSystem "src/main.cpp" "src/navigation_system.cpp"
    ${NAVIGATION_SYSTEM_SRC} "../shared/src/ipc_messages_logger.cpp"
    "../shared/src/ipc_messages_server_connector.cpp")
add_dependencies (NavigationSystem NAVIGATION_SYSTEM_EDL)

target_compile_definitions (NavigationSystem PRIVATE ENTITY_NAME="Navigation System")

if (SIMULATION)
    target_compile_definitions (NavigationSystem PRIVATE SIMULATOR_IP="${SIMULATOR_IP}")
    bind_nk_targets (NavigationSystem vfs::client NAVIGATION_SYSTEM_EDL)
else ()
    target_compile_definitions (NavigationSystem PRIVATE COORD_SRC=${COORD_SRC} ALT_SRC=${ALT_SRC})
    bind_nk_targets (NavigationSystem bsp::client uart::client i2c::process_interface NAVIGATION_SYSTEM_EDL)
endif ()
bind_nk_targets (NavigationSystem ${CMAKE_THREAD_LIBS_INIT} NAVIGATION_SYSTEM_EDL)

if (INSPECTOR_ROLE)
    target_compile_definitions (NavigationSystem PRIVATE IS_INSPECTOR=TRUE)
endif()