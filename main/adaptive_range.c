#include "adaptive_range.h"
#include <nvs.h>

esp_err_t adaptive_range_load(struct adaptive_range *range)
{
    if (range == NULL || range->name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Open storage
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(range->name, NVS_READONLY, &h);
    if (err != ESP_OK)
    {
        return err;
    }

    // Read stored values
    nvs_get_i32(h, "l", &range->low);
    nvs_get_i32(h, "h", &range->high);

    nvs_close(h);
    return ESP_OK;
}

esp_err_t adaptive_range_store(struct adaptive_range *range)
{
    if (range == NULL || range->name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Open storage
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(range->name, NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        return err;
    }

    // Write stored values
    err = nvs_set_i32(h, "l", range->low);
    if (err != ESP_OK)
    {
        goto exit;
    }

    err = nvs_set_i32(h, "h", range->high);
    if (err != ESP_OK)
    {
        goto exit;
    }

    // Commit
    err = nvs_commit(h);
    if (err != ESP_OK)
    {
        goto exit;
    }

    // Close and return
exit:
    nvs_close(h);
    return ESP_OK;
}

static float map(int32_t x, int32_t in_min, int32_t in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

esp_err_t adaptive_range_update(struct adaptive_range *range, int32_t value, float *percent)
{
    if (range == NULL || range->name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Compare range
    bool extend = false;
    if (value > range->high)
    {
        range->high = value;
        extend = true;
    }
    else if (value < range->low)
    {
        range->low = value;
        extend = true;
    }

    // Calculate
    if (percent)
    {
        *percent = map(value, range->low, range->high, 0.0f, 0.1f);
    }

    // Store changes
    if (extend)
    {
        return adaptive_range_store(range);
    }
    else
    {
        // Success
        return ESP_OK;
    }
}
