#ifndef I2S_PARALLEL_H
#define I2S_PARALLEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "soc/i2s_struct.h"
#include "rom/lldesc.h"


#define DMA_MAX (4096-12)
//#define DMA_MAX (512)

typedef enum {
    I2S_PARALLEL_BITS_8=8,
    I2S_PARALLEL_BITS_16=16,
    I2S_PARALLEL_BITS_32=32,
} i2s_parallel_cfg_bits_t;

typedef struct {
    void *memory;
    size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
    int gpio_bus[24];
    int gpio_clk;
    int clkspeed_hz;
    i2s_parallel_cfg_bits_t bits;
    i2s_parallel_buffer_desc_t *bufa;
    i2s_parallel_buffer_desc_t *bufb;
} i2s_parallel_config_t;

void i2s_parallel_setup_without_malloc(i2s_dev_t *dev, const i2s_parallel_config_t *cfg);
void link_dma_desc(volatile lldesc_t *dmadesc, volatile lldesc_t *prevdmadesc, void *memory, size_t size);

void i2s_parallel_flip_to_buffer(i2s_dev_t *dev, int bufid);
bool IRAM_ATTR i2s_parallel_is_previous_buffer_free();
  


typedef void (*callback)(void);
void setShiftCompleteCallback(callback f);

#ifdef __cplusplus
}
#endif

#endif
