#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void serial_esp_init(void);
void serial_esp_tx(uint8_t* data, size_t len);
bool serial_esp_tx_ready();

#ifdef __cplusplus
}
#endif
