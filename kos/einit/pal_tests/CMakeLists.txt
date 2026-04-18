project (einit)

include (platform/image)
include (test-generator/add_kss_pal_qemu_tests)

project_header_default ("STANDARD_GNU_14:YES" "STRICT_WARNINGS:YES")

set (ENTITIES
        AutopilotConnector
        CredentialManager
        FlightController
        NavigationSystem
        PeripheryController
        ServerConnector
        Logger
        precompiled_vfs::VfsRamFs
        precompiled_vfs::VfsNet
        rump::dhcpcd_entity
        ${wpa_SUPPLICANT_ENTITY})


# Generate PAL tests.
set_target_properties (precompiled_vfs::VfsRamFs
PROPERTIES
    EXTRA_ENV "
        VFS_FILESYSTEM_BACKEND: server:kl.VfsRamFs"
    EXTRA_ARGS "
        - -f
        - fstab"
)

add_kss_pal_qemu_tests (
    PSL_FILES "../drone_controller/src/pal_tests.psl.in"
    DEPENDS
        LOGGER_EDL
        AUTOPILOT_CONNECTOR_EDL
        CREDENTIAL_MANAGER_EDL
        NAVIGATION_SYSTEM_EDL
        PERIPHERY_CONTROLLER_EDL
        SERVER_CONNECTOR_EDL
        FLIGHT_CONTROLLER_EDL
        klog_edl_files
        klog_storage_edl_files
    ENTITIES ${ENTITIES}
    CUSTOM_VFS
        precompiled_vfs::VfsRamFs
    ENV_VARS
        VFS_FILESYSTEM_BACKEND=client:kl.VfsRamFs
    )

add_custom_target (pal-test0 DEPENDS kos-qemu-image-PalTest0-sim)
