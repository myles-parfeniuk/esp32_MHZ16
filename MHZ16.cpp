#include "MHZ16.hpp"

/**
 * @brief MHZ16 sensor constructor.
 *
 * Construct a MHZ16 object for managing a MHZ16 sensor.
 * Initializes required GPIO pins, UART peripheral, and warmup timer.
 *
 * @param cfg Configuration settings (optional), default settings can be seen in mhz16_config_t definition
 * @return void, nothing to return
 */
MHZ16::MHZ16(mhz16_config_t cfg)
    : cfg(cfg)
    , warm_sem(xSemaphoreCreateBinary())
    , uart_evt_queue(NULL)
{
    uart_config_t uart_cfg = {.baud_rate = 9600U, // as per data sheet only 9600 is acceptable
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT};

    ESP_ERROR_CHECK(uart_param_config(cfg.uart_peripheral, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg.uart_peripheral, cfg.io_tx, cfg.io_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 132 is minimum possible size for ring buffers (buffer_sz > UART_HW_FIFO_LEN(uart_num))
    ESP_ERROR_CHECK(uart_driver_install(cfg.uart_peripheral, 132, 132, 1, &uart_evt_queue, 0));

    esp_timer_create_args_t warmup_timer_conf = {
            .callback = warmup_timer_cb, .arg = this, .dispatch_method = ESP_TIMER_TASK, .name = "MHZ16 warmup timer", .skip_unhandled_events = true};

    ESP_ERROR_CHECK(esp_timer_create(&warmup_timer_conf, &warmup_timer_hdl));

    // 3 minute warmup time as per datasheet
    ESP_ERROR_CHECK(esp_timer_start_once(warmup_timer_hdl, WARMUP_PERIOD_US));
}

/**
 * @brief Enables/disables self calibration mode.
 *
 * Enables automatic 24hr calibration cycle, zero point is 400ppm.
 *
 * As per data sheet this mode is only suitable for home/office/warehouse environments.
 * If using sensor in an agricultural or outdoor settings, in extreme temperature (ex. greenhouses or refrigerators) this mode should be disabled.
 *
 * @param enable True if enabling self calibration mode, false if otherwise.
 *
 * @return void, nothing to return
 */
void MHZ16::set_self_calibration(bool enable)
{
    std::array<uint8_t, PACKET_SZ> packet;

    assemble_tx_packet(packet, CMD_SET_SELF_CALIBRATE, {enable ? static_cast<uint8_t>(0xA0) : static_cast<uint8_t>(0x00), 0, 0, 0, 0});

    uart_write_bytes(cfg.uart_peripheral, packet.data(), packet.size());
    vTaskDelay(100 / portTICK_PERIOD_MS); // allow some time for MHZ16 to execute command (it is really slow, documentation is poor on this)

    measure_CO2(); // flush first measurement
}

/**
 * @brief Sends command to calibrate zero point.
 *
 * Calibrates sensor zero point.
 * As per data sheet the sensor should be operated in a 400ppm CO2 environment for at least 20 minutes for this command is executed.
 *
 * @return void, nothing to return
 */
void MHZ16::calibrate_zero_point()
{
    std::array<uint8_t, PACKET_SZ> packet;

    assemble_tx_packet(packet, CMD_CAL_ZERO_POINT, {0, 0, 0, 0, 0});
    
    uart_write_bytes(cfg.uart_peripheral, packet.data(), PACKET_SZ);
    vTaskDelay(100 / portTICK_PERIOD_MS); // allow some time for MHZ16 to execute command (it is really slow, documentation is poor on this)

    measure_CO2(); // flush first measurement
}

/**
 * @brief Sends command to calibrate span point.
 *
 * Calibrates sensor span point.
 * As per data sheet the sensor should be operated in span_point ppm CO2 environment for at least 20 minutes for this command is executed.
 * For example, for a span point of 2000ppm, the sensor should be placed inside a controlled 2000ppm CO2 environment for 20 minutes.
 *
 * @param span_point Specified span point in ppm CO2 to calibrate for.
 *
 * @return void, nothing to return
 */
void MHZ16::calibrate_span_point(uint16_t span_point)
{
    std::array<uint8_t, PACKET_SZ> packet;

    assemble_tx_packet(packet, CMD_CAL_SPAN_POINT, {static_cast<uint8_t>(span_point / 256), static_cast<uint8_t>(span_point % 256), 0, 0, 0});

    uart_write_bytes(cfg.uart_peripheral, packet.data(), PACKET_SZ);
    vTaskDelay(100 / portTICK_PERIOD_MS); // allow some time for MHZ16 to execute command (it is really slow, documentation is poor on this)

    measure_CO2(); // flush first measurement
}

/**
 * @brief Sends command to adjust measurement range
 *
 * @param range Specified max range value in ppm CO2 (for ex, 5000 would set the measurement range to 0ppm -> 5000ppm)
 *
 * @return void, nothing to return
 */
void MHZ16::set_detection_range(uint32_t range)
{
    std::array<uint8_t, PACKET_SZ> packet;

    assemble_tx_packet(packet, CMD_SET_DETECTION_RANGE,
            {0, static_cast<uint8_t>((range & 0xFF000000) >> 3 * 8), static_cast<uint8_t>((range & 0x00FF0000) >> 2 * 8),
                    static_cast<uint8_t>((range & 0x0000FF00) >> 1 * 8), static_cast<uint8_t>((range & 0x000000FF) >> 0 * 8)});

    uart_write_bytes(cfg.uart_peripheral, packet.data(), PACKET_SZ);
    vTaskDelay(100 / portTICK_PERIOD_MS); // allow some time for MHZ16 to execute command (it is really slow, documentation is poor on this)

    measure_CO2(); // flush first measurement
}

/**
 * @brief Sends command to measure CO2 and returns back result from MHZ16.
 *
 * @return The measured CO2 in ppm.
 */
int16_t MHZ16::measure_CO2()
{
    std::array<uint8_t, PACKET_SZ> packet;

    assemble_tx_packet(packet, CMD_READ_CO2, {0, 0, 0, 0, 0});

    uart_write_bytes(cfg.uart_peripheral, packet.data(), PACKET_SZ);

    packet.fill(0);
    uart_read_bytes(cfg.uart_peripheral, packet.data(), PACKET_SZ, 100);

    if (verify_checksum(packet))
        return static_cast<int16_t>(packet[2] * 256 + packet[3]);
    else
        return -1;
}

/**
 * @brief Block until warmup period has expired (3 minutes after constructor call).
 *
 * As per datasheet, MHZ16 requires 3 minutes to warmup after boot before measurements can be considered accurate.
 *
 * Ideally, this warmup period should be adjusted or skipped using the RTC to see the last time since boot; but, it is
 * unfair to assume an external battery is connected to RTC pins, therefore a 3 minute timer started at object instantiation is used instead.
 * Please adjust this logic if using an RTC.
 *
 * @return void, nothing to return
 */
void MHZ16::wait_until_warm()
{
    ESP_LOGI(TAG, "Waiting 3 minutes for sensor warmup period.");
    // wait until semaphore is given by warmup_timer_cb
    xSemaphoreTake(warm_sem, portMAX_DELAY);
    xSemaphoreGive(warm_sem);
    ESP_LOGI(TAG, "Sensor warmup completed.");
}

/**
 * @brief Check if warmup period has expired (3 minutes after constructor call).
 *
 * As per datasheet, MHZ16 requires 3 minutes to warmup after boot before measurements can be considered accurate.
 *
 * @return False if sensor is still heating, true if sensor is ready.
 */
bool MHZ16::is_warm()
{
    if (xSemaphoreTake(warm_sem, 0) != pdFALSE)
    {
        xSemaphoreGive(warm_sem);
        return true;
    }

    return false;
}

/**
 * @brief Assembles a command packet to send to MHZ16.
 *
 * @param packet Reference to array to contain packet contents.
 * @param command The desired command to send.
 * @param data 5 byte wide data array to load into packet.
 *
 * @return void, nothing to return
 */
void MHZ16::assemble_tx_packet(std::array<uint8_t, PACKET_SZ>& packet, uint8_t command, const std::array<uint8_t, DATA_SZ> data)
{
    uint8_t i = 0;

    // assemble header
    packet[0] = 0xFF;    // start byte
    packet[1] = 0x01;    // reserved
    packet[2] = command; // command byte

    // assemble data
    for (const uint8_t& data_byte : data)
    {
        packet[3 + i] = data_byte;
        i++;
    }

    // calculate checksum
    packet[8] = calculate_checksum(packet);
}

/**
 * @brief Calculates checksum of passed packet.
 *
 * @param packet Reference to array containing packet contents.
 *
 * @return The calculated checksum.
 */
uint8_t MHZ16::calculate_checksum(const std::array<uint8_t, PACKET_SZ>& packet)
{
    int16_t checksum = 0;

    for (int i = 1; i < 8; i++)
        checksum += packet[i];

    checksum = 0xFF - checksum;
    checksum++;

    return static_cast<uint8_t>(checksum);
}

/**
 * @brief Calculates checksum and verifies if it matches one contained within packet.
 *
 * @param packet Reference to array containing packet contents.
 *
 * @return True if checksum was valid, false if glitch occurred.
 */
bool MHZ16::verify_checksum(const std::array<uint8_t, PACKET_SZ>& packet)
{

    if (calculate_checksum(packet) == packet[8])
        return true;
    else
        return false;
}

/**
 * @brief Warmup timer callback.
 *
 * One shot timer callback, executed 3 minutes after constructor call.
 * As per datasheet MHZ16 has a 3 minute warmup period.
 *
 * @param arg Pointer to MHZ16 object passed at timer creation, casted to void pointer.
 *
 * @return void, nothing to return
 */
void MHZ16::warmup_timer_cb(void* arg)
{
    MHZ16* sensor = static_cast<MHZ16*>(arg);

    xSemaphoreGive(sensor->warm_sem);
}