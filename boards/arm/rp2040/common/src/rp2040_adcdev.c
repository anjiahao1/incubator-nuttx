#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/analog/adc.h>

#include "rp2040_adc.h"

int board_adcdev_initialize(void)
{
  int ret;
  FAR struct adc_dev_s *adc;
  /* Initialize adc device */

#ifdef CONFIG_RP2040_ADC_CHANNEL0
  adc = rp2040_adc_initialize(0);
  if (!adc)
    {
      aerr("ERROR: Failed to get ADC channel 0 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc0" */

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL1
  adc = rp2040_adc_initialize(1);
  if (!adc)
    {
      aerr("ERROR: Failed to get ADC channel 1 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc1" */

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL2
  adc = rp2040_adc_initialize(2);
  if (!adc)
    {
      aerr("ERROR: Failed to get ADC channel 2 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc2" */

  ret = adc_register("/dev/adc2", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL3
  adc = rp2040_adc_initialize(3);
  if (!adc)
    {
      aerr("ERROR: Failed to get ADC channel 3 dev\n");
      return -ENODEV;
    }

  /* Register the ADC driver at "/dev/adc3" */

  ret = adc_register("/dev/adc3", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
