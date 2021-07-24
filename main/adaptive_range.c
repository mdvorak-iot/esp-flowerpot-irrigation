#include "adaptive_range.h"
#include <esp_log.h>
#include <nvs.h>

static const char TAG[] = "adaptive_range";

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
        // Ignore when not found, and return success
        return err != ESP_ERR_NVS_NOT_FOUND ? err : ESP_OK;
    }

    // Read stored values
    nvs_get_i32(h, "l", &range->low);
    nvs_get_i32(h, "h", &range->high);

    nvs_close(h);

    ESP_LOGI(TAG, "loaded %s {low=%d,high=%d}", range->name, range->low, range->high);
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

    ESP_LOGI(TAG, "stored %s {low=%d,high=%d}", range->name, range->low, range->high);

    // Close and return
exit:
    nvs_close(h);
    return err;
}

static int32_t constrain(int32_t x, int32_t min, int32_t max)
{
    return x < min ? min : (x > max ? max : x);
}

static float map(int32_t x, int32_t in_min, int32_t in_max, float out_min, float out_max)
{
    return (float)(constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

esp_err_t adaptive_range_update(struct adaptive_range *range, int32_t value, float *percent)
{
    if (range == NULL || range->name == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Compare and update range, but only if it overshoots edges by given tolerance
    bool extend = false;
    if (value - range->tolerance > range->high)
    {
        range->high = value - range->tolerance;
        extend = true;
    }
    else if (value + range->tolerance < range->low)
    {
        range->low = value + range->tolerance;
        extend = true;
    }

    // Calculate
    if (percent)
    {
        *percent = map(value, range->low, range->high, 0.0f, 1.0f);
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
