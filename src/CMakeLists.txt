
# From freertos/*
if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
    return()
endif()

include(FreeRTOS_Kernel_import.cmake)
# ------- 

set(TARGET_NAME hello_freertos1)
add_executable(${TARGET_NAME}
    hello_freertos.c
    )
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include/
    )
target_link_libraries(${TARGET_NAME} PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_stdlib
    )
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(${TARGET_NAME} PRIVATE
        pico_cyw43_arch_none
    )
endif()
target_compile_definitions(${TARGET_NAME} PRIVATE
    configNUMBER_OF_CORES=2
    )
pico_add_extra_outputs(${TARGET_NAME})
