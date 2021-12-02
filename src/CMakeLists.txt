# Include source file(s) to project
add_library(haw_nrf905 STATIC
    nRF905.c
)

# Links all required dependencies
target_link_libraries(haw_nrf905 
    pico_stdlib
    hardware_spi
)

# Link include directory / Path to header file
target_include_directories(haw_nrf905 PUBLIC ../include)