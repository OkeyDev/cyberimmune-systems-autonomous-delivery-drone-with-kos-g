cmake_minimum_required (VERSION 3.12)

project (DroneController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include (platform)
initialize_platform (FORCE_STATIC)
if (PLATFORM_SUPPORTS_DYNAMIC_LINKING)
    message (FATAL_ERROR "The solution only supports static libraries")
endif ()

include (platform/nk)
include (platform/install)

set (RESOURCES ${CMAKE_SOURCE_DIR}/resources)

# Add a package importing components for building an audit program and
# connecting to it.
find_package (klog REQUIRED)
include_directories (${klog_INCLUDE})

# Add the package that imports klog_storage_entity.
find_package (klog_storage REQUIRED)
include_directories (${klog_storage_INCLUDE})

find_package (vfs REQUIRED)
include_directories (${vfs_INCLUDE})
find_package (precompiled_vfs REQUIRED)
include_directories (${precompiled_vfs_INCLUDE})

if (SIMULATION OR SERVER)
    find_package (rump REQUIRED COMPONENTS DHCPCD_ENTITY)
    include_directories (${rump_INCLUDE})
endif ()

if (NOT SIMULATION)
    if (SERVER)
        find_package (wpa REQUIRED COMPONENTS SUPPLICANT_ENTITY)
        include_directories (${wpa_INCLUDE})
        find_package (dnet_executables REQUIRED)
        find_package (dnet_brcm_fmac REQUIRED)
    endif ()
endif ()

if (SERVER)
    find_package (mosquitto REQUIRED COMPONENTS CXX_LIB)
    include_directories (${mosquitto_INCLUDE})
endif ()

if (NOT SIMULATION)
    find_package (bsp REQUIRED)
    include_directories (${bsp_INCLUDE})
    find_package (gpio REQUIRED COMPONENTS CLIENT_LIB ENTITY)
    include_directories (${gpio_INCLUDE})
    find_package (uart REQUIRED)
    include_directories (${uart_INCLUDE})
    find_package (i2c REQUIRED)
    include_directories (${i2c_INCLUDE})
    find_package (camera REQUIRED)
    include_directories (${camera_INCLUDE})
endif ()

find_package (MbedTLS REQUIRED COMPONENTS)
include_directories (${MbedTLS_INCLUDE})

set (fmt_USE_STATIC ON)
set (fdn_USE_STATIC ON)
find_package (logrr_cpp REQUIRED)

if (UNIT_TESTS)
    add_nk_edl (UNIT_TESTS_EDL  "${CMAKE_SOURCE_DIR}/resources/UnitTests.edl"
                NK_MODULE       "drone_controller"
                LANG            "C")
else ()
    add_nk_idl (LOGGER_IDL "${RESOURCES}/LoggerInterface.idl"
                NK_MODULE   "drone_controller"
                LANG        "CXX")

    add_nk_idl (AUTOPILOT_CONNECTOR_IDL "${RESOURCES}/AutopilotConnectorInterface.idl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (AUTOPILOT_CONNECTOR_IDL LOGGER_IDL)
    bind_nk_targets (AUTOPILOT_CONNECTOR_IDL LOGGER_IDL)

    add_nk_idl (CREDENTIAL_MANAGER_IDL  "${RESOURCES}/CredentialManagerInterface.idl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (CREDENTIAL_MANAGER_IDL LOGGER_IDL)
    bind_nk_targets (CREDENTIAL_MANAGER_IDL LOGGER_IDL)

    add_nk_idl (NAVIGATION_SYSTEM_IDL   "${RESOURCES}/NavigationSystemInterface.idl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (NAVIGATION_SYSTEM_IDL LOGGER_IDL)
    bind_nk_targets (NAVIGATION_SYSTEM_IDL LOGGER_IDL)

    add_nk_idl (PERIPHERY_CONTROLLER_IDL    "${RESOURCES}/PeripheryControllerInterface.idl"
                NK_MODULE                   "drone_controller"
                LANG                        "CXX")

    add_dependencies (PERIPHERY_CONTROLLER_IDL LOGGER_IDL)
    bind_nk_targets (PERIPHERY_CONTROLLER_IDL LOGGER_IDL)

    add_nk_idl (SERVER_CONNECTOR_IDL    "${RESOURCES}/ServerConnectorInterface.idl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (SERVER_CONNECTOR_IDL LOGGER_IDL)
    bind_nk_targets (SERVER_CONNECTOR_IDL LOGGER_IDL)

    add_nk_edl (LOGGER_EDL  "${RESOURCES}/Logger.edl"
                NK_MODULE   "drone_controller"
                LANG        "CXX")

    add_dependencies (LOGGER_EDL LOGGER_IDL)
    bind_nk_targets (LOGGER_EDL LOGGER_IDL)
    add_dependencies (LOGGER_EDL SERVER_CONNECTOR_IDL)
    bind_nk_targets (LOGGER_EDL SERVER_CONNECTOR_IDL)

    add_nk_edl (AUTOPILOT_CONNECTOR_EDL "${RESOURCES}/AutopilotConnector.edl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (AUTOPILOT_CONNECTOR_EDL AUTOPILOT_CONNECTOR_IDL)
    bind_nk_targets (AUTOPILOT_CONNECTOR_EDL AUTOPILOT_CONNECTOR_IDL)

    add_nk_edl (CREDENTIAL_MANAGER_EDL  "${RESOURCES}/CredentialManager.edl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (CREDENTIAL_MANAGER_EDL CREDENTIAL_MANAGER_IDL)
    bind_nk_targets (CREDENTIAL_MANAGER_EDL CREDENTIAL_MANAGER_IDL)

    add_nk_edl (NAVIGATION_SYSTEM_EDL   "${RESOURCES}/NavigationSystem.edl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (NAVIGATION_SYSTEM_EDL NAVIGATION_SYSTEM_IDL)
    bind_nk_targets (NAVIGATION_SYSTEM_EDL NAVIGATION_SYSTEM_IDL)

    add_nk_edl (PERIPHERY_CONTROLLER_EDL    "${RESOURCES}/PeripheryController.edl"
                NK_MODULE                   "drone_controller"
                LANG                        "CXX")

    add_dependencies (PERIPHERY_CONTROLLER_EDL PERIPHERY_CONTROLLER_IDL)
    bind_nk_targets (PERIPHERY_CONTROLLER_EDL PERIPHERY_CONTROLLER_IDL)

    add_nk_edl (SERVER_CONNECTOR_EDL    "${RESOURCES}/ServerConnector.edl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (SERVER_CONNECTOR_EDL SERVER_CONNECTOR_IDL)
    bind_nk_targets (SERVER_CONNECTOR_EDL SERVER_CONNECTOR_IDL)

    nk_build_edl_files (klog_edl_files NK_MODULE "drone_controller" EDL "resources/KlogEntity.edl")
    nk_build_edl_files (klog_storage_edl_files NK_MODULE "drone_controller" EDL "resources/KlogStorageEntity.edl" NK_FLAGS "--extended-errors")

    add_nk_edl (FLIGHT_CONTROLLER_EDL   "${RESOURCES}/FlightController.edl"
                NK_MODULE               "drone_controller"
                LANG                    "CXX")

    add_dependencies (FLIGHT_CONTROLLER_EDL AUTOPILOT_CONNECTOR_IDL)
    bind_nk_targets (FLIGHT_CONTROLLER_EDL AUTOPILOT_CONNECTOR_IDL)
    add_dependencies (FLIGHT_CONTROLLER_EDL CREDENTIAL_MANAGER_IDL)
    bind_nk_targets (FLIGHT_CONTROLLER_EDL CREDENTIAL_MANAGER_IDL)
    add_dependencies (FLIGHT_CONTROLLER_EDL NAVIGATION_SYSTEM_IDL)
    bind_nk_targets (FLIGHT_CONTROLLER_EDL NAVIGATION_SYSTEM_IDL)
    add_dependencies (FLIGHT_CONTROLLER_EDL PERIPHERY_CONTROLLER_IDL)
    bind_nk_targets (FLIGHT_CONTROLLER_EDL PERIPHERY_CONTROLLER_IDL)
    add_dependencies (FLIGHT_CONTROLLER_EDL SERVER_CONNECTOR_IDL)
    bind_nk_targets (FLIGHT_CONTROLLER_EDL SERVER_CONNECTOR_IDL)
endif ()

add_compile_options (-Wall -Wextra -Wconversion
                     -fPIE -D_FORTIFY_SOURCE=2 -O2
                     -fstack-protector-strong
                     -mbranch-protection=standard
                     -Wsign-conversion -Wformat=2 -Wformat-security -Werror=format-security
                     -fsanitize=undefined -fsanitize-undefined-trap-on-error)

add_link_options (LINKER:-z,noexecstack
                  LINKER:-z,separate-code
                  LINKER:-z,now)

if (UNIT_TESTS)
    add_subdirectory (unit_tests)
    add_subdirectory (einit/unit_tests)
else ()
    add_subdirectory (logger)
    add_subdirectory (autopilot_connector)
    add_subdirectory (credential_manager)
    add_subdirectory (navigation_system)
    add_subdirectory (periphery_controller)
    add_subdirectory (server_connector)
    add_subdirectory (flight_controller)
    add_subdirectory (klog_storage)
    if (PAL_TESTS)
        add_subdirectory (einit/pal_tests)
    else ()
        add_subdirectory (einit/drone_controller)
    endif ()
endif ()
