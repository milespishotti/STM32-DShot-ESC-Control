#include "DShot.h"

#define DSHOT_LEN 18
#define DSHOT_1_PULSE 420
#define DSHOT_0_PULSE 210

extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim4_up;

static uint16_t dshot_buffer[DSHOT_LEN] = {0};

volatile uint8_t dshot_busy = 0;
volatile uint32_t dshot_start_count = 0;
volatile uint32_t dshot_done_count = 0;
volatile uint32_t dshot_busy_skip_count = 0;

static void DShot_BuildPacket(uint16_t throttle_cmd)
{
    uint16_t telemetry = 0;
    uint16_t value = (throttle_cmd << 1) | telemetry;

    uint16_t csum = 0;
    uint16_t csum_data = value;

    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    csum &= 0x0F;

    uint16_t packet = (value << 4) | csum;

    for (int i = 0; i < 16; i++)
    {
        dshot_buffer[i] = (packet & (1 << (15 - i))) ? DSHOT_1_PULSE : DSHOT_0_PULSE;
    }

    dshot_buffer[16] = 0;
    dshot_buffer[17] = 0;
}

void DShot_Init(void)
{
    hdma_tim4_up.XferCpltCallback = DShot_DMA_Complete;
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void DShot_DMA_Complete(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim4_up)
    {
        TIM4->DIER &= ~TIM_DIER_UDE;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

        dshot_done_count++;
        dshot_busy = 0;
    }
}

void DShot_Send(uint16_t throttle_cmd)
{
    if (dshot_busy)
    {
        dshot_busy_skip_count++;
        return;
    }

    dshot_busy = 1;
    dshot_start_count++;

    DShot_BuildPacket(throttle_cmd);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

    HAL_StatusTypeDef status = HAL_DMA_Start_IT(
        &hdma_tim4_up,
        (uint32_t)&dshot_buffer[0],
        (uint32_t)&TIM4->CCR1,
        DSHOT_LEN
    );

    if (status != HAL_OK)
    {
        dshot_busy = 0;
        return;
    }

    TIM4->DIER |= TIM_DIER_UDE;
}
