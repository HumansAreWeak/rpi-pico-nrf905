#include <pico/stdlib.h>
#include <stdio.h>
#include "haw/nRF905.h"

#define PIN_MISO 4
#define PIN_MOSI 3
#define PIN_SCK 2
#define PIN_CS 5

#define THIS_DEVICE 0xDEADBEEF
#define OTHER_DEVICE 0xBAADF00D

int main()
{
    stdio_init_all();

    // Configured SPI0 at 0.5MHz
    spi_init(spi_default, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    nrf905_t nrf905_tx = nrf905_init(spi_default, PIN_CS, 6, 8, 11, 10, 9, true);
    nrf905_default_config(&nrf905_tx);

    nrf905_set_listen_address(&nrf905_tx, THIS_DEVICE);

    uint counter = 0;
    uint sent = 0;
    uint replies = 0;
    uint timeouts = 0;
    uint invalids = 0;

    uint8_t debug[12];

    while (1)
    {
        char data[NRF905_MAX_PAYLOAD_SIZE] = {0};

        printf("Sending Data: ");

        for (uint8_t i = 0; i < 12; i++)
        {
            debug[i] = 0;
        }

        nrf905_get_config_registers(&nrf905_tx, debug);

        printf("Config Register\n");
        for (uint8_t i = 0; i < 10; i++)
        {
            printf("Register %d: %d\n", i, debug[i]);
        }
        printf("CD: %d\n", debug[10]);
        printf("DR: %d\n", debug[11]);

        nrf905_send_data(&nrf905_tx, OTHER_DEVICE, &data, sizeof(data), NRF905_NEXTMODE_TX);

        nrf905_get_config_registers(&nrf905_tx, data);

        printf("Config Register\n");
        for (uint8_t i = 0; i < 10; i++)
        {
            printf("Register %d: %d\n", i, data[i]);
        }
        printf("CD: %d\n", data[10]);
        printf("DR: %d\n", data[11]);

        printf("Data sent, waiting for reply...\n");

        sleep_ms(1000);
    }

    return 0;
}