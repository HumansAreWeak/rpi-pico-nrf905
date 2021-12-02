## RX Example

# Create an executable
add_executable(nrf905_rx_example 
    nrf905_rx_example.c
)

# Link all required libraries
target_link_libraries(nrf905_rx_example 
    pico_stdlib
    haw_nrf905
)

# Enable printf output for USB communication
pico_enable_stdio_usb(nrf905_rx_example 1)
pico_enable_stdio_uart(nrf905_rx_example 0)

# Add the .uf2 output to the build directory
pico_add_extra_outputs(nrf905_rx_example)

## TX Example

# Create an executable
add_executable(nrf905_tx_example 
    nrf905_tx_example.c
)

# Link all required libraries
target_link_libraries(nrf905_tx_example 
    pico_stdlib
    hardware_i2c 
    haw_nrf905
)

# Enable printf output for USB communication
pico_enable_stdio_usb(nrf905_tx_example 1)
pico_enable_stdio_uart(nrf905_tx_example 0)

# Add the .uf2 output to the build directory
pico_add_extra_outputs(nrf905_tx_example)