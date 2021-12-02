#include "haw/nRF905.h"

// Macro based function
#define NRF905_CALC_CHANNEL(f, b) ((((f) / (1 + (b >> 1))) - 422400000UL) / 100000UL)

// Instructions
#define NRF905_CMD_NOP 0xFF
#define NRF905_CMD_W_CONFIG 0x00
#define NRF905_CMD_R_CONFIG 0x10
#define NRF905_CMD_W_TX_PAYLOAD 0x20
#define NRF905_CMD_R_TX_PAYLOAD 0x21
#define NRF905_CMD_W_TX_ADDRESS 0x22
#define NRF905_CMD_R_TX_ADDRESS 0x23
#define NRF905_CMD_R_RX_PAYLOAD 0x24
#define NRF905_CMD_CHAN_CONFIG 0x80

// Registers
#define NRF905_REG_CHANNEL 0x00
#define NRF905_REG_CONFIG1 0x01
#define NRF905_REG_ADDR_WIDTH 0x02
#define NRF905_REG_RX_PAYLOAD_SIZE 0x03
#define NRF905_REG_TX_PAYLOAD_SIZE 0x04
#define NRF905_REG_RX_ADDRESS 0x05
#define NRF905_REG_CONFIG2 0x09

// Register masks
#define NRF905_MASK_CHANNEL 0xFE
#define NRF905_MASK_AUTO_RETRAN 0xDF
#define NRF905_MASK_LOW_RX 0xEF
#define NRF905_MASK_PWR 0xF3
#define NRF905_MASK_BAND 0xFD
#define NRF905_MASK_CRC 0x3F
#define NRF905_MASK_CLK 0xC7
#define NRF905_MASK_OUTCLK 0xF8

static inline void cs_select(struct spi_information *spi)
{
    asm volatile("nop \n nop \n nop");
    gpio_put(spi->chip_select_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(struct spi_information *spi)
{
    asm volatile("nop \n nop \n nop");
    gpio_put(spi->chip_select_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static inline void write_spi(struct spi_information *spi, const uint8_t *data, size_t len)
{
    cs_select(spi);
    spi_write_blocking(spi->instance, data, len);
    cs_deselect(spi);
}

static uint8_t addressMatched(struct nrf905 *self)
{
    uint state = gpio_get(self->pins.address_match_pin);

    if (state)
        return 1;

    return 0;
}

static uint8_t readConfigRegister(struct nrf905 *self, const uint8_t reg)
{
    uint8_t val = 0;
    uint8_t data = NRF905_CMD_R_CONFIG | reg;

    cs_select(&self->spi);
    spi_write_blocking(self->spi.instance, &data, 1);
    spi_read_blocking(self->spi.instance, NRF905_CMD_NOP, &val, 1);
    cs_deselect(&self->spi);

    return val;
}

static void writeConfigRegister(struct nrf905 *self, const uint8_t reg, const uint8_t val)
{
    uint8_t data[2] = {NRF905_CMD_W_CONFIG | reg, val};
    write_spi(&self->spi, data, 2);
}

static void setConfigReg1(struct nrf905 *self, const uint8_t val, const uint8_t mask)
{
    writeConfigRegister(self, NRF905_REG_CONFIG1, (readConfigRegister(self, NRF905_REG_CONFIG1) & mask) | val);
}

static void setConfigReg2(struct nrf905 *self, const uint8_t val, const uint8_t mask)
{
    writeConfigRegister(self, NRF905_REG_CONFIG2, (readConfigRegister(self, NRF905_REG_CONFIG2) & mask) | val);
}

static void setAddress(struct nrf905 *self, uint32_t address, uint8_t cmd)
{
    uint8_t data[5] = {
        cmd,
        (uint8_t)address,
        address >> 8,
        address >> 16,
        address >> 24};

    write_spi(&self->spi, data, 5);
}

static inline void standbyEnter(struct nrf905 *self)
{
    gpio_put(self->pins.standby_pin, true);
}

static inline void standbyLeave(struct nrf905 *self)
{
    gpio_put(self->pins.standby_pin, false);
}

static inline void powerUp(struct nrf905 *self)
{
    self->config.powered_up = 1;
    gpio_put(self->pins.power_enable_pin, true);
}

static inline void powerDown(struct nrf905 *self)
{
    self->config.powered_up = 0;
    gpio_put(self->pins.power_enable_pin, false);
}

static inline void modeRX(struct nrf905 *self)
{
    gpio_put(self->pins.transfer_enable_pin, 0);
}

static inline void modeTX(struct nrf905 *self)
{
    gpio_put(self->pins.transfer_enable_pin, 1);
}

struct nrf905 nrf905_init(spi_inst_t *spi_instance, const uint8_t chip_select_pin, const uint8_t data_ready_pin,
                          const uint8_t carrier_detect_pin, const uint8_t transfer_enable_pin, const uint8_t standby_pin,
                          const uint8_t power_enable_pin, const uint8_t auto_setup)
{
    struct nrf905 nrf905;
    nrf905.spi.instance = spi_instance;
    nrf905.spi.chip_select_pin = chip_select_pin;
    nrf905.pins.data_ready_pin = data_ready_pin;
    nrf905.pins.transfer_enable_pin = transfer_enable_pin;
    nrf905.pins.power_enable_pin = power_enable_pin;
    nrf905.pins.standby_pin = standby_pin;
    nrf905.pins.address_match_pin = 0;

    // Standard configuration
    nrf905.config.collision_avoidance = 0;
    nrf905.config.powered_up = 0;
    nrf905.config.use_hw_am = 0;
    nrf905.config.use_power = 1;

    cs_deselect(&nrf905.spi);

    if (auto_setup)
    {
        gpio_init(chip_select_pin);
        gpio_set_dir(chip_select_pin, GPIO_OUT);
        gpio_put(chip_select_pin, 1);

        gpio_init(data_ready_pin);
        gpio_set_dir(data_ready_pin, GPIO_IN);

        gpio_init(carrier_detect_pin);
        gpio_set_dir(carrier_detect_pin, GPIO_IN);

        gpio_init(transfer_enable_pin);
        gpio_set_dir(transfer_enable_pin, GPIO_OUT);

        gpio_init(power_enable_pin);
        gpio_set_dir(power_enable_pin, GPIO_OUT);

        gpio_init(standby_pin);
        gpio_set_dir(standby_pin, GPIO_OUT);
    }

    powerDown(&nrf905);
    standbyEnter(&nrf905);
    modeRX(&nrf905);
    sleep_ms(3);

    nrf905_default_config(&nrf905);

    return nrf905;
}

void nrf905_config(struct nrf905 *self, uint8_t payload_size, enum NRF905_ADDR_SIZE address_size,
                   enum NRF905_CLOCK_FREQ clock_frequency, enum NRF905_BAND band, enum NRF905_TX_PWR transmit_power,
                   enum NRF905_CRC crc_specification, enum NRF905_OUTPUT_CLOCK output_clock_freq, uint8_t auto_retransmit, uint8_t low_power_receive)
{
    if (auto_retransmit)
        auto_retransmit = 1;

    if (low_power_receive)
        low_power_receive = 1;

    uint16_t channel = NRF905_CALC_CHANNEL(clock_frequency, band);

    if (payload_size > NRF905_MAX_PAYLOAD_SIZE)
        payload_size = NRF905_MAX_PAYLOAD_SIZE;

    uint8_t init_data[11] = {
        NRF905_CMD_W_CONFIG,
        (uint8_t)channel,
        auto_retransmit | low_power_receive | transmit_power | band | ((channel >> 8) & 1),
        (address_size << 4) | address_size,
        payload_size,
        payload_size,
        0xE7,
        0xE7,
        0xE7,
        0xE7,
        crc_specification | clock_frequency | output_clock_freq};
    write_spi(&self->spi, init_data, 11);

    // Setting default transmit address
    uint8_t address_data[5] = {
        NRF905_CMD_W_TX_ADDRESS,
        0xE7,
        0xE7,
        0xE7,
        0xE7,
    };
    write_spi(&self->spi, address_data, 5);

    const uint8_t bufferSize = payload_size;

    // Clear Transmit data payload
    uint8_t transmit_data[bufferSize + 1];
    transmit_data[0] = NRF905_CMD_W_TX_PAYLOAD;

    for (uint8_t i = 0; i < bufferSize; i++)
    {
        transmit_data[i + 1] = 0x00;
    }
    write_spi(&self->spi, transmit_data, bufferSize + 1);

    // Clear Data Ready Pin by reading received payload
    uint8_t dr_data[bufferSize + 1];
    dr_data[0] = NRF905_CMD_R_RX_PAYLOAD;

    for (uint8_t i = 0; i < bufferSize; i++)
    {
        dr_data[i + 1] = NRF905_CMD_NOP;
    }
    write_spi(&self->spi, dr_data, bufferSize + 1);

    if (self->config.use_power)
    {
        uint8_t clear_dr_data[bufferSize + 1];
        clear_dr_data[0] = NRF905_CMD_R_RX_PAYLOAD;

        for (uint8_t i = 0; i < bufferSize; i++)
        {
            clear_dr_data[i + 1] = NRF905_CMD_NOP;
        }

        write_spi(&self->spi, clear_dr_data, bufferSize + 1);
    }
}

void nrf905_default_config(struct nrf905 *self)
{
    nrf905_config(self, NRF905_MAX_PAYLOAD_SIZE, NRF905_ADDR_SIZE_4, NRF905_CLOCK_16MHZ, NRF905_BAND_433, NRF905_TX_PWR_10, NRF905_CRC_16, NRF905_OUTCLK_2MHZ, false, false);
    nrf905_set_listen_address(self, 0xDEADBEEF);
}

uint8_t nrf905_begin(struct nrf905 *self, uint8_t payload_size)
{
    // self->config.payload_size = payload_size;
}

void nrf905_set_am(struct nrf905 *self, uint8_t address_matched_pin, uint8_t auto_setup)
{
    self->config.use_hw_am = 1;
    self->pins.address_match_pin = address_matched_pin;

    if (auto_setup)
    {
        gpio_init(address_matched_pin);
        gpio_set_dir(address_matched_pin, GPIO_IN);
    }
}

void nrf905_set_address(struct nrf905 *self, const uint32_t address, enum NRF905_ADDR_SIZE size)
{
    nrf905_set_listen_address(self, address);
    nrf905_set_address_size(self, size);
}

void nrf905_set_band(struct nrf905 *self, enum NRF905_BAND band)
{
    self->band = band;
    uint8_t reg = (readConfigRegister(self, NRF905_REG_CONFIG1) & NRF905_MASK_BAND) | band;
    uint8_t data[2] = {NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1, reg};
    write_spi(&self->spi, data, 2);
}

uint8_t nrf905_receive_busy(struct nrf905 *self)
{
    return addressMatched(self);
}

uint8_t nrf905_airway_busy(struct nrf905 *self)
{
    int state = gpio_get(self->pins.carrier_detect_pin);

    if (state)
        return 1;

    return 0;
}

void nrf905_set_channel(struct nrf905 *self, uint16_t channel)
{
    if (channel > 511)
        channel = 511;

    self->channel = channel;
    uint8_t reg = (readConfigRegister(self, NRF905_REG_CONFIG1) & NRF905_MASK_CHANNEL) | (channel >> 8);
    uint8_t data[3] = {NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL, channel, reg};
    write_spi(&self->spi, data, 3);
}

void nrf905_set_auto_retransmit(struct nrf905 *self, uint8_t state)
{
    if (state)
        state = 0x20;

    setConfigReg1(self, state, NRF905_MASK_AUTO_RETRAN);
}

void nrf905_set_low_rx_power(struct nrf905 *self, uint8_t state)
{
    if (state)
        state = 0x10;

    setConfigReg1(self, state, NRF905_MASK_LOW_RX);
}

void nrf905_set_tx_power(struct nrf905 *self, enum NRF905_TX_PWR power)
{
    setConfigReg1(self, power, NRF905_MASK_PWR);
}

void nrf905_set_crc(struct nrf905 *self, enum NRF905_CRC crc)
{
    setConfigReg2(self, crc, NRF905_MASK_CRC);
}

void nrf905_set_clock_out(struct nrf905 *self, enum NRF905_OUTPUT_CLOCK clock)
{
    setConfigReg2(self, clock, NRF905_MASK_OUTCLK);
}

void nrf905_set_payload_size(struct nrf905 *self, uint8_t size)
{
    if (size < 1)
        size = 1;
    else if (size > NRF905_MAX_PAYLOAD_SIZE)
        size = NRF905_MAX_PAYLOAD_SIZE;

    uint8_t data[3] = {NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE, size, size};
    write_spi(&self->spi, data, 3);
}

void nrf905_set_rx_payload_size(struct nrf905 *self, uint8_t size)
{
    if (size < 1)
        size = 1;
    else if (size > NRF905_MAX_PAYLOAD_SIZE)
        size = NRF905_MAX_PAYLOAD_SIZE;

    uint8_t data[2] = {NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE, size};
    write_spi(&self->spi, data, 2);
}

void nrf905_set_tx_payload_size(struct nrf905 *self, uint8_t size)
{
    if (size < 1)
        size = 1;
    else if (size > NRF905_MAX_PAYLOAD_SIZE)
        size = NRF905_MAX_PAYLOAD_SIZE;

    uint8_t data[2] = {NRF905_CMD_W_CONFIG | NRF905_REG_TX_PAYLOAD_SIZE, size};
    write_spi(&self->spi, data, 2);
}

void nrf905_set_address_size(struct nrf905 *self, enum NRF905_ADDR_SIZE size)
{
    self->address_size = size;
    uint8_t data[2] = {NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH, (size << 4) | size};
    write_spi(&self->spi, data, 2);
}

void nrf905_set_collision_avoidance(struct nrf905 *self, const uint8_t state)
{
    if (state)
        self->config.collision_avoidance = 1;
    else
        self->config.collision_avoidance = 0;
}

void nrf905_set_listen_address(struct nrf905 *self, uint32_t address)
{
    self->address = address;
    setAddress(self, address, NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
}

void nrf905_set_power_mode(struct nrf905 *self, const uint8_t state)
{
    if (state)
        self->config.use_power = 1;
    else
        self->config.use_power = 0;
}

uint8_t nrf905_send_data(struct nrf905 *self, const uint32_t address, void *data, const uint8_t len, enum NRF905_NEXT_MODE next)
{
    if (self->config.collision_avoidance)
    {
        if (nrf905_airway_busy(self))
        {
            return 0;
        }
    }

    setAddress(self, address, NRF905_CMD_W_TX_ADDRESS);

    if (data != NULL)
    {
        uint8_t packets[len + 1];
        packets[0] = NRF905_CMD_W_TX_PAYLOAD;

        for (uint8_t i = 0; i < len; i++)
        {
            packets[i + 1] = (((uint8_t *)data)[i]);
        }

        write_spi(&self->spi, packets, len + 1);
    }

    if (self->config.use_power)
    {
        standbyEnter(self);
        powerUp(self);
        sleep_ms(3);
    }

    if (self->config.collision_avoidance)
    {
        if (nrf905_airway_busy(self))
        {
            return 0;
        }
    }

    modeTX(self);
    standbyLeave(self);

    if (next == NRF905_NEXTMODE_RX)
    {
        sleep_us(700);
        modeRX(self);
    }
    else if (next == NRF905_NEXTMODE_STANDBY)
    {
        sleep_us(14);
        standbyEnter(self);
    }

    return 1;
}

void nrf905_receive_data(struct nrf905 *self)
{
    modeRX(self);
    standbyLeave(self);
    powerUp(self);
}

void nrf905_read(struct nrf905 *self, void *data, uint8_t len)
{
    if (len > NRF905_MAX_PAYLOAD_SIZE)
        len = NRF905_MAX_PAYLOAD_SIZE;

    static const uint8_t cmd = NRF905_CMD_R_RX_PAYLOAD;

    cs_select(&self->spi);
    spi_write_blocking(self->spi.instance, &cmd, 1);
    spi_read_blocking(self->spi.instance, NRF905_CMD_NOP, data, len);
    cs_deselect(&self->spi);
}

void nrf905_set_power(struct nrf905 *self, const uint8_t state)
{
    if (state)
    {
        uint8_t was_powered_up = self->config.powered_up;
        standbyEnter(self);
        powerUp(self);

        if (was_powered_up)
            sleep_ms(3);
    }
    else
    {
        powerDown(self);
    }
}

void nrf905_standby(struct nrf905 *self)
{
    standbyEnter(self);
    powerUp(self);
}

uint8_t nrf905_data_ready(struct nrf905 *self)
{
    return gpio_get(self->pins.data_ready_pin);
}

void nrf905_get_config_registers(struct nrf905 *self, void *regs)
{
    uint8_t data = NRF905_CMD_R_CONFIG;

    cs_select(&self->spi);
    spi_write_blocking(self->spi.instance, &data, 1);
    spi_read_blocking(self->spi.instance, NRF905_CMD_NOP, regs, 10);
    cs_deselect(&self->spi);

    ((uint8_t *)regs)[10] = gpio_get(self->pins.carrier_detect_pin);
    ((uint8_t *)regs)[11] = gpio_get(self->pins.data_ready_pin);
}