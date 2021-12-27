#include <stdio.h>
#include "storage.h"
#include "nvs_flash.h"
#include "nvs.h"

static nvs_handle_t my_handle;

void storage_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
}

uint8_t get_need_calibrate_from_storage()
{
    uint8_t val = 0;
    nvs_get_u8(my_handle, "need_calib", &val);
    return val;
}

void set_need_calibrate_to_storage(uint8_t val)
{
    nvs_set_u8(my_handle, "need_calib", val);
    nvs_commit(my_handle);
}