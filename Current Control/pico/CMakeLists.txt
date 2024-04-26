cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(hermes_vi)


pico_sdk_init()


add_executable(${PROJECT_NAME}
	./src/main.c
	./src/current.c
	./src/hermes.c
	./src/receiver.c
	./src/control.c
	./src/pwm_gen.c
	./src/debug.c
	)

target_link_libraries(hermes_vi pico_stdlib hardware_adc pico_multicore pico_sync hardware_pwm)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)