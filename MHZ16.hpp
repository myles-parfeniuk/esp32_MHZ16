#pragma once
// standard library includes
#include <array>
// esp-idf includes
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/// @brief MHZ16 configuration settings structure passed into MHZ16 constructor
typedef struct mhz16_config_t
{
        gpio_num_t io_tx;            ///< TX GPIO (connects to MHZ16 rx pin)
        gpio_num_t io_rx;            //< RX GPIO (connects to MHZ16 tx pin)
        uart_port_t uart_peripheral; ///< UART host peripheral (UART_NUM_0 to UART_NUM_2)

        /// @brief Default MHZ16 configuration settings constructor.
        /// To modify default GPIO pins, run "idf.py menuconfig" esp32_MHZ16->GPIO Configuration.
        /// Alternatively, edit the default values in "Kconfig.projbuild"
        mhz16_config_t()
            : io_tx(static_cast<gpio_num_t>(CONFIG_ESP32_MHZ16_GPIO_TX))              // default: io 17
            , io_rx(static_cast<gpio_num_t>(CONFIG_ESP32_MHZ16_GPIO_RX))              // default: io 16
            , uart_peripheral(static_cast<uart_port_t>(CONFIG_ESP32_MHZ16_UART_HOST)) // default: UART_NUM_2
        {
        }

        /// @brief Overloaded MHZ16 configuration settings constructor for custom pin settings
        mhz16_config_t(gpio_num_t io_tx, gpio_num_t io_rx, uart_port_t uart_peripheral)
            : io_tx(io_tx)
            , io_rx(io_rx)
            , uart_peripheral(uart_peripheral)
        {
        }

} mhz16_config_t;

class MHZ16
{
    public:
        MHZ16(mhz16_config_t cfg = mhz16_config_t());

        void set_self_calibration(bool enable);
        void calibrate_zero_point();
        void calibrate_span_point(uint16_t span_point);
        void set_detection_range(uint32_t range);
        int16_t measure_CO2();
        void wait_until_warm();
        bool is_warm();

    private:
        static const constexpr uint64_t WARMUP_PERIOD_US = 3 * 60 * (1000 * 1000); ///< 3 minute sensor warmup period
        static const constexpr uint8_t PACKET_SZ = 9U;                             ///< Data packet size
        static const constexpr uint8_t DATA_SZ = 5U;                               ///< Data field width within packet (in bytes)
        static const constexpr uint8_t CMD_READ_CO2 = 0x86;                        ///< Read CO2 ppm command
        static const constexpr uint8_t CMD_SET_SELF_CALIBRATE = 0x79;              ///< Enable/disable self calibration command.
        static const constexpr uint8_t CMD_SET_DETECTION_RANGE = 0x99;             ///< Set measurement range command.
        static const constexpr uint8_t CMD_CAL_ZERO_POINT = 0x87;                  ///< Calibrate zero point command.
        static const constexpr uint8_t CMD_CAL_SPAN_POINT = 0x88;                  ///< Calibrate span point command.

        static const constexpr char* TAG = "MHZ16"; ///< Class tag used in serial/ESP_LOG statements.

        mhz16_config_t cfg;                  ///< Structure containing GPIO and UART settings, initialized with constructor param.
        SemaphoreHandle_t warm_sem;          ///< Semaphore to indicate when warmup period has completed.
        QueueHandle_t uart_evt_queue;        ///< UART evt queue, used by esp-idf uart driver.
        esp_timer_handle_t warmup_timer_hdl; ///< One-shot 3 minute warmup timer handle.

        void assemble_tx_packet(std::array<uint8_t, PACKET_SZ>& packet, uint8_t command, const std::array<uint8_t, DATA_SZ> data);
        uint8_t calculate_checksum(const std::array<uint8_t, PACKET_SZ>& packet);
        bool verify_checksum(const std::array<uint8_t, PACKET_SZ>& packet);
        static void warmup_timer_cb(void* arg);
};
