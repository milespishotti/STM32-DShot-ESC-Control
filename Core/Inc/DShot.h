#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "main.h"

void DShot_Init(void);
void DShot_Send(uint16_t throttle_cmd);
void DShot_DMA_Complete(DMA_HandleTypeDef *hdma);

extern volatile uint8_t dshot_busy;
extern volatile uint32_t dshot_start_count;
extern volatile uint32_t dshot_done_count;
extern volatile uint32_t dshot_busy_skip_count;

#endif
