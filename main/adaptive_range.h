#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

struct adaptive_range
{
    const char *name;
    int32_t low;
    int32_t high;
    int32_t tolerance;
};

#define ADAPTIVE_RANGE_INIT(name_, low_, high_, tolerance_) \
    {                                                       \
        .name = (name_),                                    \
        .low = (low_),                                      \
        .high = (high_),                                    \
        .tolerance = (tolerance_),                          \
    }

esp_err_t adaptive_range_load(struct adaptive_range *range);

esp_err_t adaptive_range_update(struct adaptive_range *range, int32_t value, float *percent);

esp_err_t adaptive_range_store(struct adaptive_range *range);

#ifdef __cplusplus
}
#endif
