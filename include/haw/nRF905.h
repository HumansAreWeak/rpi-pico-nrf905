#ifndef NRF905_RX_H_
#define NRF905_RX_H_

#include "pico/stdlib.h"
#include "hardware/spi.h"

#define NRF905_MAX_PAYLOAD_SIZE 32

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef SPI_INFORMATION_S_
#define SPI_INFORMATION_S_
    struct spi_information
    {
        spi_inst_t *instance;
        uint8_t chip_select_pin;
    };
#endif

    enum NRF905_BAND
    {
        NRF905_BAND_433 = 0,
        NRF905_BAND_868 = 2,
        NRF905_BAND_915 = 2
    };

    enum NRF905_TX_PWR
    {
        NRF905_TX_PWR_N10 = 0,
        NRF905_TX_PWR_N2 = 4,
        NRF905_TX_PWR_6 = 8,
        NRF905_TX_PWR_10 = 12
    };

    enum NRF905_CRC
    {
        NRF905_CRC_DISABLE = 0,
        NRF905_CRC_8 = 0x40,
        NRF905_CRC_16 = 0xC0
    };

    enum NRF905_OUTPUT_CLOCK
    {
        NRF905_OUTCLK_DISABLE = 0,
        NRF905_OUTCLK_4MHZ = 4,
        NRF905_OUTCLK_2MHZ = 5,
        NRF905_OUTCLK_1MHZ = 6,
        NRF905_OUTCLK_500KHZ = 7
    };

    enum NRF905_ADDR_SIZE
    {
        NRF905_ADDR_SIZE_1 = 1,
        NRF905_ADDR_SIZE_4 = 4
    };

    enum NRF905_NEXT_MODE
    {
        NRF905_NEXTMODE_STANDBY,
        NRF905_NEXTMODE_RX,
        NRF905_NEXTMODE_TX
    };

    enum NRF905_CLOCK_FREQ
    {
        NRF905_CLOCK_4MHZ = 0x00,
        NRF905_CLOCK_8MHZ = 0x08,
        NRF905_CLOCK_12MHZ = 0x10,
        NRF905_CLOCK_16MHZ = 0x18,
        NRF905_CLOCK_20MHZ = 0x20,
    };

    struct nrf905_configuration
    {
        uint8_t collision_avoidance;
        uint8_t use_power;
        uint8_t powered_up;
        uint8_t use_hw_am;
    };

    struct nrf905_pins
    {
        uint8_t data_ready_pin;
        uint8_t carrier_detect_pin;
        uint8_t address_match_pin;
        uint8_t transfer_enable_pin;
        uint8_t power_enable_pin;
        uint8_t standby_pin;
    };

    typedef struct nrf905
    {
        struct spi_information spi;
        struct nrf905_configuration config;
        struct nrf905_pins pins;
        enum NRF905_ADDR_SIZE address_size;
        enum NRF905_BAND band;
        uint32_t address;
        uint16_t channel;
    } nrf905_t;

    /**
     * @brief Initializes a nRF905 struct and returns it.
     *
     * @param spi_inst_t* spi_instance: SPI bus instance. Needed for multicore applications
     * @param uint8_t chip_select_pin: (CSN) pin
     * @param uint8_t data_ready_pin: (DR) pin
     * @param uint8_t carrier_detect_pin: (CD) pin
     * @param uint8_t transfer_enable_pin: (TXEN) pin
     * @param uint8_t standby_pin: (CE) pin
     * @param uint8_t power_enable_pin: (PWR) pin
     * @param uint8_t auto_setup: Setup all the pins automatically (initialize + set to input or output)
     * @return struct nrf905
     */
    struct nrf905 nrf905_init(spi_inst_t *spi_instance, const uint8_t chip_select_pin, const uint8_t data_ready_pin,
                              const uint8_t carrier_detect_pin, const uint8_t transfer_enable_pin, const uint8_t standby_pin,
                              const uint8_t power_enable_pin, const uint8_t auto_setup);

    /**
     * @brief Configure the nRF905 Module. Don't get discouraged by seeing all the parameters, you only need to call this function once.
     *
     * @param nrf905_t* self: Reference to itself
     * @param payload_size
     * @param address_size
     * @param clock_frequency
     * @param band
     * @param transmit_power
     * @param crc_specification
     * @param output_clock_freq
     * @param auto_retransmit
     * @param low_power_receive
     */
    void nrf905_config(struct nrf905 *self, uint8_t payload_size, enum NRF905_ADDR_SIZE address_size,
                       enum NRF905_CLOCK_FREQ clock_frequency, enum NRF905_BAND band, enum NRF905_TX_PWR transmit_power,
                       enum NRF905_CRC crc_specification, enum NRF905_OUTPUT_CLOCK output_clock_freq, const uint8_t auto_retransmit, const uint8_t low_power_receive);

    /**
     * @brief Sets up the device with default configurations applied
     *
     * @param nrf905_t* self: Reference to itself
     */
    void nrf905_default_config(struct nrf905 *self);

    /**
     * @brief Sets the address match pin (Optinal)
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t address_matched_pin: AM pin from nRF905
     * @param uint8_t auto_setup: Setup all the pins internally
     */
    void nrf905_set_am(struct nrf905 *self, const uint8_t address_matched_pin, const uint8_t auto_setup);

    /**
     * @brief Set channel to listen and transmit on
     *
     * 433MHz band: Channel 0 is 422.4MHz up to 511 which is 473.5MHz (Each channel is 100KHz apart)
     *
     * 868/915MHz band: Channel 0 is 844.8MHz up to 511 which is 947MHz (Each channel is 200KHz apart)
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint16_t channel: Channel value (0 - 511)
     */
    void nrf905_set_channel(struct nrf905 *self, uint16_t channel);

    /**
     * @brief Set frequency band
     *
     * @param nrf905_t* self: Reference to itself
     * @param NRF905_BAND band: Frequency band
     */
    void nrf905_set_band(struct nrf905 *self, enum NRF905_BAND band);

    /**
     * @brief Set the address and the address size of the device
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint32_t address: Device address (Default is 0xE7E7E7E7)
     * @param NRF905_ADDR_SIZE size: Address size
     */
    void nrf905_set_address(struct nrf905 *self, const uint32_t address, enum NRF905_ADDR_SIZE size);

    /**
     * @brief Set automatic retransmit
     *
     * If next mode is set to ::NRF905_NEXTMODE_TX when calling ::nrf905_tx() and auto-retransmit is enabled then it will constantly retransmit the payload,
     * otherwise a carrier wave with no data will be transmitted.
     *
     * Can be useful in areas with lots of interference, but you will need to make sure you can differentiate between re-transmitted packets and new packets.
     *
     * Other transmissions will be blocked if collision avoidance is enabled.
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t state: 0 to disable, otherwise enabled
     */
    void nrf905_set_auto_retransmit(struct nrf905 *self, uint8_t state);

    /**
     * @brief Set the power mode
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t state: 0 to disable, otherwise enabled
     */
    void nrf905_set_power_mode(struct nrf905 *self, const uint8_t state);

    /**
     * @brief Sets device to low power receive mode
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t state: 0 to disable, otherwise enabled
     */
    void nrf905_set_low_rx_power(struct nrf905 *self, uint8_t state);

    /**
     * @brief Set transmitter output power
     *
     * NRF905_TX_PWR_N10 | -10dBm = 100uW
     * NRF905_TX_PWR_N2 | -2dBm = 631uW
     * NRF905_TX_PWR_6 | 6dBm = 4mW
     * NRF905_TX_PWR_10 | 10dBm = 10mW
     *
     * @param nrf905_t* self: Reference to itself
     * @param NRF905_TX_PWR power: Output power level
     */
    void nrf905_set_tx_power(struct nrf905 *self, enum NRF905_TX_PWR power);

    /**
     * @brief Set cyclic redundancy check (CRC)
     *
     * If set to true, any false data packet with bit flips inside will be corrected automatically
     *
     * @param nrf905_t* self: Reference to itself
     * @param NRF905_CRC crc: CRC Type
     */
    void nrf905_set_crc(struct nrf905 *self, enum NRF905_CRC crc);

    /**
     * @brief Set clock output
     *
     * Output a clock signal on pin 3 of nRF905 MPU
     *
     * @param nrf905_t* self: Reference to itself
     * @param NRF905_OUTPUT_CLOCK clock: Clock output frequency
     */
    void nrf905_set_clock_out(struct nrf905 *self, enum NRF905_OUTPUT_CLOCK clock);

    /**
     * @brief Sets the payload size (1 - 32 bytes)
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t size: Payload size in bytes
     */
    void nrf905_set_payload_size(struct nrf905 *self, uint8_t size);

    /**
     * @brief Set RX payload size
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t size: RX payload size in bytes
     */
    void nrf905_set_rx_payload_size(struct nrf905 *self, uint8_t size);

    /**
     * @brief Set TX payload size
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t size: TX payload size in bytes
     */
    void nrf905_set_tx_payload_size(struct nrf905 *self, uint8_t size);

    /**
     * @brief Set the address size of the device
     *
     * @param nrf905_t* self: Reference to itself
     * @param NRF905_ADDR_SIZE size: Address size
     */
    void nrf905_set_address_size(struct nrf905 *self, enum NRF905_ADDR_SIZE size);

    /**
     * @brief See if an address match is asserted
     *
     * @param nrf905_t* self: Reference to itself
     * @return uint8_t: 1 if currently receiving payload or payload is ready to be read, otherwise 0
     */
    uint8_t nrf905_receive_busy(struct nrf905 *self);

    /**
     * @brief See if airway is busy (carrier detect pin asserted)
     *
     * @param nrf905_t* self: Reference to itself
     * @return uint8_t: 1 if other transmissions detected, otherwise 0
     */
    uint8_t nrf905_airway_busy(struct nrf905 *self);

    /**
     * @brief Set collision avoidance
     *
     * If a transmission is going on, the device will not send any data to avoid packet collisions
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t state: 0 to disable, otherwise enable
     */
    void nrf905_set_collision_avoidance(struct nrf905 *self, const uint8_t state);

    /**
     * @brief Set address to listen to
     *
     * @note From the datasheet: Each byte within the address should be unique. Repeating bytes within the address reduces the
     * effectivness of the address and increases its susceptibility to noise which increases the packet error rate. The address
     * should also have several level shifts (that is, 10101100) reducing the statistical effect of noise and the packet error rate.
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint32_t address: Device address (Default is 0xE7E7E7E7)
     */
    void nrf905_set_listen_address(struct nrf905 *self, uint32_t address);

    /**
     * @brief Begin a data transmission
     *
     * If the radio is still transmitting then the the payload and address will be updated as it is being sent, this means the payload on the receiving end may contain old and new data.\n
     * This also means that a node may receive part of a payload that was meant for a node with a different address.\n
     * Use the ::NRF905_CB_TXCOMPLETE callback to set a flag or something to ensure the transmission is complete before sending another payload.
     *
     * If the radio is in power down mode then this function will take an additional 3ms to complete.
     * If 3ms is too long then call ::nRF905_standby(), do whatever you need to do for at least 3ms then call ::nRF905_TX().
     *
     * If \p next is set to ::NRF905_NEXTMODE_RX then this function will take an additional 700us to complete.\n
     * If 700us is too long then set \p next to ::NRF905_NEXTMODE_STANDBY and call ::nRF905_RX() in the ::NRF905_CB_TXCOMPLETE callback instead.
     *
     * The ::NRF905_CB_TXCOMPLETE callback will only work if \p nextMode is set to ::NRF905_NEXTMODE_STANDBY
     *
     * If \p data is NULL and/or \p len is 0 then the payload will not be modified, whatever was previously transmitted will be sent again to the \p sendTo address.
     *
     * For the collision avoidance to work the radio should be in RX mode for around 5ms before attempting to transmit.
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint32_t address: Address to send the payload to
     * @param void* data: Data to send
     * @param uint8_t len: Length of data
     * @param NRF905_NEXT_MODE next: What mode to enter once the data transmission is complete
     * @return uint8_t: 0 if other transmissions are going on and collision avoidance is enabled, otherwise 1 if transmission successfully began
     */
    uint8_t nrf905_send_data(struct nrf905 *self, const uint32_t address, void *data, const uint8_t len, enum NRF905_NEXT_MODE next);

    /**
     * @brief Enter receive mode
     *
     * If the radio is currently transmitting then receive mode will be entered once it has finished.
     * This function will also automatically power up the radio and leave standby mode.
     *
     * @param nrf905_t* self: Reference to itself
     */
    void nrf905_receive_data(struct nrf905 *self);

    /**
     * @brief Get received payload
     *
     * This function can be called multiple times to read a few bytes at a time.
     * The payload is cleared when the radio enters power down mode or leaves standby and enters RX mode.
     * The radio will not receive anymore data until all of the payload has been read or cleared.
     *
     * @param nrf905_t* self: Reference to itself
     * @param void* data: Buffer to store the data
     * @param uint8_t len: Length of data
     */
    void nrf905_read(struct nrf905 *self, void *data, uint8_t len);

    /**
     * @brief Sets the power mode
     *
     * @param nrf905_t* self: Reference to itself
     * @param uint8_t state: 0 to enter sleep, otherwise enter standby with 3ms delay
     */
    void nrf905_set_power(struct nrf905 *self, const uint8_t state);

    /**
     * @brief Enter standby mode
     *
     * Similar to nrf905_set_power() but without any delays.
     * There must be a 3ms delay between powering up and beginning a transmission
     *
     * @param nrf905_t* self: Reference to itself
     */
    void nrf905_standby(struct nrf905 *self);

    uint8_t nrf905_data_ready(struct nrf905 *self);

    void nrf905_get_config_registers(struct nrf905 *self, void *regs);

#ifdef __cplusplus
}
#endif
#endif