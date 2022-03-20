#include <debug.h>
#include <sys/types.h>
#include <nuttx/semaphore.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include "arm_arch.h"
#include "hardware/rp2040_adc.h"
struct rp2040_adcdev_s
{
  struct adc_dev_s adcdev;
  uint32_t         adcbase;     /* ADC base address */
  uint32_t         channel;
  struct adc_callback_s *cb;
};

static sem_t g_sem_excl = SEM_INITIALIZER(1);

static int rp2040_adc_bind(FAR struct adc_dev_s *dev,
                           FAR const struct adc_callback_s *callback);
static void rp2040_adc_reset(FAR struct adc_dev_s *dev);
static int rp2040_adc_setup(FAR struct adc_dev_s *dev);
static void rp2040_adc_shutdown(FAR struct adc_dev_s *dev);
static void rp2040_adc_rxint(FAR struct adc_dev_s *dev, bool enable);

static int rp2040_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                            unsigned long arg);

static struct adc_ops_s g_adcops = {
  .ao_bind = rp2040_adc_bind,
  .ao_reset = rp2040_adc_reset,
  .ao_setup = rp2040_adc_setup,
  .ao_shutdown = rp2040_adc_shutdown,
  .ao_rxint = rp2040_adc_rxint,
  .ao_ioctl = rp2040_adc_ioctl
};

#ifdef CONFIG_RP2040_ADC_CHANNEL0
static struct rp2040_adcdev_s g_adc0dev =
{
  .adcdev.ad_ops = &g_adcops,
  .channel = 0,
  .adcbase = RP2040_ADC_BASE,
};
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL1
static struct rp2040_adcdev_s g_adc1dev =
{
  .adcdev.ad_ops = &g_adcops,
  .channel = 1,
  .adcbase = RP2040_ADC_BASE,
};
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL2
static struct rp2040_adcdev_s g_adc2dev =
{
  .adcdev.ad_ops = &g_adcops,
  .channel = 2,
  .adcbase = RP2040_ADC_BASE,
};
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL3
static struct rp2040_adcdev_s g_adc3dev =
{
  .adcdev.ad_ops = &g_adcops,
  .channel = 3,
  .adcbase = RP2040_ADC_BASE,
};
#endif

/****************************************************************************
 * Name: adc_getreg
 *
 * Description:
 *   Get the contents of the ADC register at offset
 *
 * Input Parameters:
 *   priv   - private ADC device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t adc_getreg(FAR struct rp2040_adcdev_s *priv,
                                  uint8_t offset)
{
  return getreg32(priv->adcbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: adc_putreg
 *
 * Description:
 *   Write a 32-bit value to the ADC register at offset
 *
 * Input Parameters:
 *   priv   - private ADC device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void adc_putreg(FAR struct rp2040_adcdev_s *priv,
                              uint8_t offset, uint32_t value)
{
  putreg32(value, priv->adcbase + (uint32_t)offset);
}

static void rp2040_adc_enable(struct rp2040_adcdev_s *priv)
{
  uint32_t reg = 0;
  reg = adc_getreg(priv, RP2040_ADC_CS_OFFSET);
  reg |= (1 << 0);
  adc_putreg(priv, RP2040_ADC_CS_OFFSET,reg);
}

static void rp2040_adc_disable(struct rp2040_adcdev_s *priv)
{
  uint32_t reg = 0;
  reg = adc_getreg(priv, RP2040_ADC_CS_OFFSET);
  reg |= (0 << 0);
  adc_putreg(priv, RP2040_ADC_CS_OFFSET,reg);
}

static uint32_t adc_read(struct rp2040_adcdev_s *priv)
{
  uint32_t reg = 0;
  reg = adc_getreg(priv, RP2040_ADC_CS_OFFSET);
  reg |= (1 << 2);
  adc_putreg(priv, RP2040_ADC_CS_OFFSET,reg);
  while(1)
  {
    reg = adc_getreg(priv, RP2040_ADC_CS_OFFSET);
    if (reg & (1 << 8))
    {
      break;
    }
  }

  reg = adc_getreg(priv, RP2040_ADC_RESULT_OFFSET);
  return reg;
}

static int rp2040_adc_bind(FAR struct adc_dev_s *dev,
                           FAR const struct adc_callback_s *callback)
{
  FAR struct rp2040_adcdev_s *priv = (FAR struct rp2040_adcdev_s *)dev;
  priv->cb = (FAR struct adc_callback_s *)callback;
  return 0;
}

static void rp2040_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct rp2040_adcdev_s *priv = (struct rp2040_adcdev_s *)dev;
  priv = priv;
}

static int rp2040_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct rp2040_adcdev_s *priv = (struct rp2040_adcdev_s *)dev;
  rp2040_adc_enable(priv);
  return 0;
}

static void rp2040_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{

}
static void rp2040_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct rp2040_adcdev_s *priv = (struct rp2040_adcdev_s *)dev;
  rp2040_adc_disable(priv);
}

static int rp2040_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                     unsigned long arg)
{
  FAR struct rp2040_adcdev_s *priv = (struct rp2040_adcdev_s *)dev;
  int ret = 0;
  switch(cmd)
  {
    case ANIOC_TRIGGER:
      {
        priv->cb->au_receive(dev,priv->channel,adc_read(priv));
        break;
      }

    default:
      {
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
      }
  }

  return ret;
}

FAR struct adc_dev_s *rp2040_adc_initialize(int channel)
{
  FAR struct rp2040_adcdev_s *priv;

  switch(channel)
  {
#ifdef CONFIG_RP2040_ADC_CHANNEL0
    case 0:
      priv = &g_adc0dev;
      break;
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL1
    case 1:
      priv = &g_adc1dev;
      break;
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL2
    case 2:
      priv = &g_adc2dev;
      break;
#endif

#ifdef CONFIG_RP2040_ADC_CHANNEL3
    case 3:
      priv = &g_adc3dev;
      break;
#endif

    default:
      {
        aerr("ERROR: No ADC interface defined\n");
        return NULL;
      }
  }

  return &priv->adcdev;
}