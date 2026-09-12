#include "ax_vin.h"
#include "adc.h"

void AX_VIN_Init(void)
{
  /* ADC1 and PA4 are configured by MX_ADC1_Init(). */
}

uint16_t AX_VIN_GetVol_X100(void)
{
  uint32_t raw;
  uint64_t scaled;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  /* 3.3 V ADC reference, 1/11 divider, 0.999 correction; result is V x 100. */
  scaled = (uint64_t)raw * 330U * 11U * 999U;
  return (uint16_t)(scaled / (4095U * 1000U));
}
