---
name: esp32-esp-idf
description: ESP-IDF native framework for ESP32. Covers FreeRTOS, component architecture, driver APIs, and low-level ESP32 development.
user-invocable: false
---

# ESP-IDF Framework for ESP32

## Overview

ESP-IDF (Espressif IoT Development Framework) is the official native development framework for ESP32. It provides low-level control, advanced features, and professional-grade development tools.

## Key Differences from Arduino

### Architecture
- **ESP-IDF**: Component-based architecture, FreeRTOS RTOS, event-driven
- **Arduino**: Simple setup/loop, single-threaded, blocking I/O

### Language
- **ESP-IDF**: C (primarily), C++ supported
- **Arduino**: C++ with simplified API wrappers

### Entry Point
```c
// ESP-IDF
void app_main(void) {
    // Main application entry
}

// vs Arduino
void setup() { }
void loop() { }
```

## Core Concepts

### FreeRTOS Tasks

ESP-IDF is built on FreeRTOS. Programs use tasks instead of loop():

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void my_task(void *pvParameters) {
    while(1) {
        // Task code here
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
    }
}

void app_main(void) {
    xTaskCreate(my_task, "my_task", 2048, NULL, 5, NULL);
    // Task continues running, app_main() can return
}
```

### Component Architecture

ESP-IDF uses components for modular code organization:

```
project/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── components/
    └── my_component/
        ├── CMakeLists.txt
        ├── include/
        └── src/
```

### Event Loop

ESP-IDF uses event-driven architecture:

```c
#include "esp_event.h"

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &event_handler, NULL));
}
```

## GPIO Operations

### Digital I/O

```c
#include "driver/gpio.h"

// Configure GPIO as output
gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << GPIO_NUM_18),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};
gpio_config(&io_conf);

// Set level
gpio_set_level(GPIO_NUM_18, 1);  // HIGH
gpio_set_level(GPIO_NUM_18, 0);  // LOW

// Read input
gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
int level = gpio_get_level(GPIO_NUM_19);
```

### PWM (LEDC)

```c
#include "driver/ledc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          5000

void ledc_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_NUM_18,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_pwm_duty(uint32_t duty) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
```

## Communication Interfaces

### UART

```c
#include "driver/uart.h"

#define UART_NUM UART_NUM_2
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
}

void uart_send(const char* data) {
    uart_write_bytes(UART_NUM, data, strlen(data));
}
```

### I2C

```c
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
```

## WiFi

```c
#include "esp_wifi.h"
#include "esp_event.h"

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "YOUR_SSID",
            .password = "YOUR_PASSWORD"
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}
```

## Logging

```c
#include "esp_log.h"

static const char *TAG = "MyApp";

void app_main(void) {
    ESP_LOGI(TAG, "Starting application");
    ESP_LOGW(TAG, "This is a warning");
    ESP_LOGE(TAG, "This is an error");
    ESP_LOGD(TAG, "Debug message");  // Only shown if log level is DEBUG
}
```

## Build System

### Project Structure

```
my_project/
├── CMakeLists.txt           # Top-level CMake
├── sdkconfig                # Project configuration
├── main/
│   ├── CMakeLists.txt       # Main component CMake
│   └── main.c               # Entry point (app_main)
└── components/              # Optional custom components
```

### CMakeLists.txt (Top-level)

```cmake
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_project)
```

### CMakeLists.txt (main/)

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ".")
```

### Building

```bash
# Configure project (only needed once or after menuconfig)
idf.py set-target esp32

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration (menuconfig)

```bash
idf.py menuconfig
```

Key configuration areas:
- **Component config**: Configure components (WiFi, BT, etc.)
- **Serial flasher config**: Flash settings
- **Partition Table**: Flash layout
- **Compiler options**: Optimization level

## Common Patterns

### Task with Delay

```c
void periodic_task(void *pvParameters) {
    while(1) {
        ESP_LOGI(TAG, "Task running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    xTaskCreate(periodic_task, "periodic_task", 2048, NULL, 5, NULL);
}
```

### Task Communication (Queue)

```c
#include "freertos/queue.h"

QueueHandle_t queue;

void sender_task(void *pvParameters) {
    int value = 0;
    while(1) {
        xQueueSend(queue, &value, portMAX_DELAY);
        value++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void receiver_task(void *pvParameters) {
    int value;
    while(1) {
        if(xQueueReceive(queue, &value, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received: %d", value);
        }
    }
}

void app_main(void) {
    queue = xQueueCreate(10, sizeof(int));
    xTaskCreate(sender_task, "sender", 2048, NULL, 5, NULL);
    xTaskCreate(receiver_task, "receiver", 2048, NULL, 5, NULL);
}
```

### Non-Volatile Storage (NVS)

```c
#include "nvs_flash.h"
#include "nvs.h"

void nvs_example(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Open NVS
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));

    // Write
    int32_t value = 42;
    nvs_set_i32(my_handle, "my_value", value);
    nvs_commit(my_handle);

    // Read
    int32_t read_value = 0;
    nvs_get_i32(my_handle, "my_value", &read_value);

    // Close
    nvs_close(my_handle);
}
```

## Memory Management

```c
// Heap allocation
void *buffer = malloc(1024);
// ... use buffer
free(buffer);

// Get free heap
ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());

// Task stack watermark (debugging)
UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
ESP_LOGI(TAG, "Stack high water mark: %d", stack_high_water_mark);
```

## Error Checking

```c
esp_err_t ret = gpio_set_level(GPIO_NUM_18, 1);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
}

// Or use macro for critical errors
ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_18, 1));  // Aborts on error
```

## Performance Considerations

- Use task priorities appropriately (higher priority = more CPU time)
- Minimize heap fragmentation (prefer static allocation for long-lived objects)
- Use task notifications instead of queues for simple signaling
- Pin time-critical tasks to specific cores with `xTaskCreatePinnedToCore()`
- Use DMA for high-speed data transfers

## Libraries and Components

ESP-IDF includes many built-in components:
- **esp_wifi**: WiFi driver
- **esp_bt**: Bluetooth driver
- **esp_http_client**: HTTP client
- **esp_http_server**: HTTP server
- **mqtt**: MQTT client
- **fatfs**: FAT filesystem
- **spiffs**: SPIFFS filesystem
- **mbedtls**: TLS/SSL
- **json**: JSON parser (cJSON)

## Troubleshooting

### Build Errors
- Run `idf.py fullclean` to clean build artifacts
- Check ESP-IDF version matches project requirements
- Verify sdkconfig is correct

### Runtime Crashes
- Check stack overflow: Increase task stack size
- Check heap overflow: Monitor free heap
- Enable core dump: `idf.py menuconfig` → Core dump
- Use `esp_backtrace_print()` for debugging

### Flash Issues
- Hold BOOT button during flash
- Check USB cable and drivers
- Reduce flash speed in menuconfig

## Resources

- ESP-IDF Programming Guide: https://docs.espressif.com/projects/esp-idf/
- API Reference: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/
- Examples: `$IDF_PATH/examples/`
