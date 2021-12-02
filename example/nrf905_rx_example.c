#include <pico/stdlib.h>
#include <stdio.h>
#include "haw/nRF905.h"

#define PIN_MISO 4
#define PIN_MOSI 3
#define PIN_SCK 2
#define PIN_CS 5

#define THIS_DEVICE 0xBAADF00D
#define OTHER_DEVICE 0xDEADBEEF

int main()
{
    stdio_init_all();

    // Configured SPI0 at 0.5MHz
    spi_init(spi0, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    nrf905_t nrf905_rx = nrf905_init(spi_default, PIN_CS, 6, 8, 11, 10, 9, true);
    nrf905_default_config(&nrf905_rx);

    nrf905_set_listen_address(&nrf905_rx, THIS_DEVICE);

    nrf905_receive_data(&nrf905_rx);

    uint pings = 0;
    uint invalids = 0;

    uint8_t data[12];

    while (1)
    {
        printf("Waiting for ping...\n");

        nrf905_get_config_registers(&nrf905_rx, data);

        printf("Config Register\n");
        for (uint8_t i = 0; i < 10; i++)
        {
            printf("Register %d: %d\n", i, data[i]);
        }

        while (nrf905_data_ready(&nrf905_rx))
        {
            uint8_t buffer[NRF905_MAX_PAYLOAD_SIZE];
            nrf905_read(&nrf905_rx, buffer, sizeof(buffer));

            printf("Got ping, next sending reply...\n");
        }

        sleep_ms(200);
    }

    return 0;
}