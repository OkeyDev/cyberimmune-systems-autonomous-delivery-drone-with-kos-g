project (einit)

include (platform/image)

find_package (logrr_server REQUIRED)
find_package (logrr_fs_output_channel REQUIRED)
include (logrr_server/configure)
include (logrr_fs_output_channel/configure)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:YES")

find_package(kss REQUIRED)

nk_build_edl_files (klog_edl_files
                    NK_MODULE "drone_controller"
                    EDL "../../resources/KlogEntity.edl")

add_executable (KlogEntityHw "../../klog/src/klog_entity.c")
target_link_libraries (KlogEntityHw ${klog_SYSTEM_AUDIT_LIB})
add_dependencies (KlogEntityHw klog_edl_files)

add_executable (KlogEntityQemu "../../klog/src/klog_entity.c")
target_link_libraries (KlogEntityQemu ${klog_SYSTEM_AUDIT_LIB})
add_dependencies (KlogEntityQemu klog_edl_files)

find_program (NTPD_ENTITY
                NAMES "ntpd" REQUIRED
                NO_CMAKE_PATH
                NO_CMAKE_ENVIRONMENT_PATH
                NO_SYSTEM_ENVIRONMENT_PATH)

set (ENTITIES
        AutopilotConnector
        CredentialManager
        FlightController
        NavigationSystem
        PeripheryController
        ServerConnector
        Logger
        precompiled_vfs::VfsNet
        ${NTPD_ENTITY}
        KlogStorageEntity
        logrr_server::LogrrServer
        logrr_fs_output_channel::LogrrFsOutputChannel)
set (FILES)

set (LOG_LEVEL INFO)
create_logrr_server (ENTITY_NAME "LogrrServer"
                    VFS_CONN_ID "kl.VfsRamFs"
                    OUTPUT_CHANNELS "logrr.LogrrFsOutputChannel"
                    LOG_LEVEL INFO
                    RECV_CORE_LOGS
                    WRITE_TO_CORE ON)
create_logrr_fs_output_channel (ENTITY_NAME "LogrrFsOutputChannel"
                                SERVER_TARGET "logrr_server::LogrrServer"
                                VFS_CONN_ID "kl.VfsRamFs"
                                LOG_DIR "/logs")
connect_to_logrr_server (SERVER_TARGET logrr_server::LogrrServer ENTITIES FlightController)

project_header_default ("STANDARD_GNU_11:YES" "STRICT_WARNINGS:NO")

set_target_properties (precompiled_vfs::VfsRamFs PROPERTIES
    EXTRA_ARGS "
    - -l
    - devfs /dev devfs 0
    - -l
    - romfs /etc romfs ro")

set (DHCPCD_ENV "
    VFS_FILESYSTEM_BACKEND: client{fs->net}:kl.VfsRamFs
    VFS_NETWORK_BACKEND: client:kl.VfsNet")

set (DHCPCD_ARGS "\
    - '-4'
    - '-f'
    - /etc/dhcpcd.conf")

if (SIMULATION OR SERVER)
    set_target_properties (precompiled_vfs::VfsNet PROPERTIES
        EXTRA_ARGS "
        - -l
        - devfs /dev devfs 0
        - -l
        - romfs /etc romfs ro")
    set_target_properties (precompiled_vfs::VfsRamFs PROPERTIES
        EXTRA_ARGS "\
        - -l
        - devfs /dev devfs rw
        - -l
        - ramfs /tmp ramfs rw
        - -l
        - romfs /etc romfs ro
        - -l
        - ramfs /var ramfs rw
        - -l
        - ramfs /logs ramfs rw")

    set_target_properties (rump::dhcpcd_entity PROPERTIES
        DEPENDS_ON_ENTITY "precompiled_vfs::VfsRamFs;precompiled_vfs::VfsNet"
        EXTRA_ENV ${DHCPCD_ENV}
        EXTRA_ARGS ${DHCPCD_ARGS})

    file(COPY_FILE ${CMAKE_SOURCE_DIR}/resources/hdd/etc/ntp.conf.in ${CMAKE_SOURCE_DIR}/resources/hdd/etc/ntp.conf)
    file(APPEND ${CMAKE_SOURCE_DIR}/resources/hdd/etc/ntp.conf "server ${NTP_IP} minpoll 3 maxpoll 9 iburst trust")

    list(APPEND ENTITIES precompiled_vfs::VfsRamFs precompiled_vfs::VfsNet rump::dhcpcd_entity)
    list(APPEND FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/dhcpcd.conf
		    ${CMAKE_SOURCE_DIR}/resources/hdd/etc/ntp.conf
	        ${CMAKE_SOURCE_DIR}/resources/hdd/etc/hosts
		)
endif ()

if (SERVER)
    if (NOT SIMULATION)
        set_target_properties (rump::dhcpcd_entity PROPERTIES
            DEPENDS_ON_ENTITY ""
            EXTRA_CONNECTIONS "\
        - target: kl.VfsNet
          id: kl.VfsNet"
            EXTRA_ENV "\
            VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
            VFS_NETWORK_BACKEND: client:kl.VfsNet"
            EXTRA_ARGS ${DHCPCD_ARGS})
        set_target_properties (precompiled_vfs::VfsNet PROPERTIES
        DEPENDS_ON_ENTITY "${entropy_ENTITY};dnet_brcm_fmac::executable;dnet_executables::whole;${dnet_ENTITY}")
        set_target_properties (${wpa_SUPPLICANT_ENTITY} PROPERTIES
            DEPENDS_ON_ENTITY ""
            EXTRA_CONNECTIONS "\
        - target: kl.VfsNet
          id: kl.VfsNet"
            EXTRA_ENV "\
            VFS_FILESYSTEM_BACKEND: client:kl.VfsNet
            VFS_NETWORK_BACKEND: client:kl.VfsNet"
            EXTRA_ARGS "\
            - '-iwl0'
            - '-c/etc/wpa_supplicant.conf'")
        list(APPEND ENTITIES ${wpa_SUPPLICANT_ENTITY})
        list(APPEND FILES ${CMAKE_SOURCE_DIR}/resources/hdd/etc/wpa_supplicant.conf)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.bin ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.raspberrypi,4-model-b.txt ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt)
        add_custom_command (OUTPUT ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob
                    COMMAND cp /lib/firmware/brcm/brcmfmac43455-sdio.clm_blob ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob)
        set (FIRMWARE_FILES ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.bin
                    ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.txt
                    ${CMAKE_BINARY_DIR}/brcmfmac43455-sdio.clm_blob)
    endif()
endif ()

if (SIMULATION)
    set (FS_IMAGE mmc0.img)
    if(EXISTS "${CMAKE_BINARY_DIR}/einit/drone_controller/${FS_IMAGE}")
      add_custom_target (${FS_IMAGE})
    else()
      add_custom_target (${FS_IMAGE} COMMAND ${KL_SDK_ROOT_PATH}/common/prepare_hdd_img.sh -s 128 -f fat32 -img ${FS_IMAGE})
    endif()
    list(APPEND ENTITIES ${FS_IMAGE})
endif ()

if (SIMULATION)
    if (SERVER)
        set (CONNECITONS_FILE "src/sim_online.yaml.in")
        set (SECURITY_FILE "src/sim_online.psl.in")
    else ()
        set (CONNECITONS_FILE "src/sim_offline.yaml.in")
        set (SECURITY_FILE "src/sim_offline.psl.in")
    endif ()
else ()
    if (SERVER)
        set (CONNECITONS_FILE "src/real_online.yaml.in")
        set (SECURITY_FILE "src/real_online.psl.in")
    else ()
        set (CONNECITONS_FILE "src/real_offline.yaml.in")
        set (SECURITY_FILE "src/real_offline.psl.in")
    endif ()
endif()

if (SIMULATION)
    set (INIT_KlogEntity_PATH "KlogEntityQemu")
    set (QEMU_FLAGS "-drive file=${FS_IMAGE},if=sd,format=raw")
    set (QEMU_ENV "QEMU_AUDIO_DRV=none")
    build_kos_qemu_image (kos-qemu-image
                            EINIT_ENTITY            EinitQemu
                            CONNECTIONS_CFG         ${CONNECITONS_FILE}
                            SECURITY_PSL            ${SECURITY_FILE}
                            KLOG_ENTITY             KlogEntityQemu
                            GINGER_ENABLE           TRUE
                            GINGER_DEFINITIONS      ${GINGER_DEFINITIONS}
                            QEMU_FLAGS              "${QEMU_FLAGS}"
                            QEMU_DEPENDENCIES       "${FS_IMAGE}"
                            IMAGE_FILES             ${ENTITIES} ${FILES})
    target_link_libraries (KlogEntityQemu kssaudit)
else ()
    set (INIT_KlogEntity_PATH "KlogEntityHw")
    build_kos_hw_image (kos-image
                        EINIT_ENTITY EinitHw
                        CONNECTIONS_CFG ${CONNECITONS_FILE}
                        SECURITY_PSL ${SECURITY_FILE}
                        KLOG_ENTITY KlogEntityHw
                        IMAGE_FILES ${ENTITIES} ${FIRMWARE_FILES} ${FILES})
    target_link_libraries (KlogEntityHw kssaudit)
# SD card image with KasperskyOS for the target hardware platform.
    include (${KL_SDK_ROOT_PATH}/common/build-sd-image.cmake)
    build_sd_image (sd-image
                        KOS_IMAGE_TARGET kos-image
                        PATH_PARTITION1_IMAGE ${IMAGE_FS})
endif()
