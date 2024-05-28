#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// Definições de pinos I2C
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 0
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define TC74_ADDR 0x49

// Definições de pinos SPI do SD
#define PIN_NUM_MISO 4
#define PIN_NUM_MOSI 3
#define PIN_NUM_CLK 10
#define PIN_NUM_CS 2

static const char *TAG = "TC74_SD";

void init_i2c() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));
}

esp_err_t init_sdcard() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    ESP_LOGI(TAG, "Mounting SD card");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

void read_and_log_temperature() {
    while (1) {
        FILE *f = fopen("/sdcard/test.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }

        uint8_t txBuf[1] = {0x00};
        uint8_t rxBuf[1];

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TC74_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, txBuf, 1, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TC74_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, rxBuf, 1, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read from sensor: %s", esp_err_to_name(ret));
            fclose(f);
            vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos antes de tentar novamente
            continue;
        }

        int temp = (int8_t)rxBuf[0]; // Converte para um valor de temperatura
        int bytes_written = fprintf(f, "Temperature: %d\n", temp);
        if (bytes_written < 0) {
            ESP_LOGE(TAG, "Failed to write to file");
        } else {
            ESP_LOGI(TAG, "Temperature logged: %d", temp);
            fflush(f); // Sincroniza os dados com o sistema de arquivos
        }

        fclose(f); // Fecha o arquivo após a escrita

        vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos antes de tentar novamente
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C");
    init_i2c();

    ESP_LOGI(TAG, "Initializing SD card");
    if (init_sdcard() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card");
        return;
    }

    ESP_LOGI(TAG, "Starting temperature logging");
    read_and_log_temperature();
}