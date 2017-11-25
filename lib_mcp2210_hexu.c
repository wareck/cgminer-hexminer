/*$T indentinput.c GC 1.140 10/16/13 10:20:34 */
/*
Special thanks to Luke Dashjr - Nanofory code was adopted from bfgminer - mcp2210.c
*/

static void
libhexu_spi_emit_buf_reverse (unsigned char *buf, const void *p, size_t sz)
{
  const unsigned char *str = p;
  size_t i;
  for (i = 0; i < sz; ++i)
    {
      unsigned char v = str[i];
      v = ((v & 0xaa) >> 1) | ((v & 0x55) << 1);
      v = ((v & 0xcc) >> 2) | ((v & 0x33) << 2);
      v = ((v & 0xf0) >> 4) | ((v & 0x0f) << 4);
      *buf++ = v;
    }
}

static void
libhexu_spi_emit_data (size_t * pos, unsigned char *where, uint16_t addr,
                       const void *buf, size_t len)
{
  unsigned char *b = where + *pos;
  if (len < 4 || len > 128)
    return;
  *pos += len + 3;
  *b++ = (len / 4 - 1) | 0xE0;
  *b++ = (addr >> 8) & 0xFF;
  *b++ = addr & 0xFF;
  libhexu_spi_emit_buf_reverse (b, buf, len);
}

static void
libhexu_bitfury_config_reg (size_t * pos, unsigned char *where, int cfgreg,
                            int ena)
{
  static const uint8_t enaconf[4] = { 0xc1, 0x6a, 0x59, 0xe3 };
  static const uint8_t disconf[4] = { 0, 0, 0, 0 };

  if (ena)
    libhexu_spi_emit_data (pos, where, 0x7000 + cfgreg * 32, enaconf, 4);
  else
    libhexu_spi_emit_data (pos, where, 0x7000 + cfgreg * 32, disconf, 4);
}

static void
libhexu_spi_send_conf (size_t * pos, unsigned char *where)
{
  int i;
  for (i = 7; i <= 11; ++i)
    libhexu_bitfury_config_reg (pos, where, i, 0);
  libhexu_bitfury_config_reg (pos, where, 6, 1);
  libhexu_bitfury_config_reg (pos, where, 4, 1);
  for (i = 1; i <= 3; ++i)
    libhexu_bitfury_config_reg (pos, where, i, 0);
  libhexu_spi_emit_data (pos, where, 0x0100, bitfury_counters, 16);
}

static void
libhexu_spi_emit_break (size_t * pos, unsigned char *where)
{
  unsigned char *b = where + *pos;
  *b = '\x4';
  *pos += 1;
}

static bool
hex_mcp2210_io (struct cgpu_info *hexmineru, uint8_t * const cmd,
                uint8_t * const buf)
{
  int r_b = 0;
  return (64 == libhexu_sendHashData (hexmineru, cmd, 64, 300) &&
          64 == libhexu_readHashData (hexmineru, buf, &r_b, 300));
}

static bool
libhexu_mcp2210_get_configs (struct cgpu_info *hexmineru)
{
  uint8_t cmd[64], buf[64];
  memset (cmd, 0, 64);
  memset (buf, 0, 64);
  cmd[0] = 0x41;
  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    {
      applog (LOG_ERR, "%s: Failed to get current %s config", __func__,
              "SPI");
      return false;
    }
  memcpy (hexmineru->cfg_spi, &buf[4], sizeof (hexmineru->cfg_spi));

  cmd[0] = 0x20;
  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    {
      applog (LOG_ERR, "%s: Failed to get current %s config", __func__,
              "GPIO");
      return false;
    }
  memcpy (hexmineru->cfg_gpio, &buf[4], sizeof (hexmineru->cfg_gpio));
  return true;
}

static bool
hex_mcp2210_set_cfg_gpio (struct cgpu_info *hexmineru)
{
  uint8_t cmd[64], buf[64];
  memset (cmd, 0, 64);
  memset (buf, 0, 64);;
  cmd[0] = 0x21;
  memcpy (&cmd[4], hexmineru->cfg_gpio, 0xe);
  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    {
      applog (LOG_ERR, "%s: Failed to set current %s config", __func__,
              "GPIO");
      return false;
    }

  if (buf[1] != 0)
    {
      applog (LOG_ERR, "%s: Error setting current %s config (%d)", __func__,
              "GPIO", buf[1]);
      return false;
    }
  return true;
}

static enum mcp2210_gpio_value
hex_mcp2210_get_gpio_input (struct cgpu_info *hexmineru, const int pin)
{
  uint8_t cmd[64], buf[64];
  memset (cmd, 0, 64);
  memset (buf, 0, 64);;
  cmd[0] = 0x31;
  const int bit = 1 << (pin % 8);
  const int byte = (pin / 8);
  hexmineru->cfg_gpio[pin] = 0;
  hexmineru->cfg_gpio[byte + 0xb] |= bit;
  if (!hex_mcp2210_set_cfg_gpio (hexmineru))
    return MGV_ERROR;

  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    {
      applog (LOG_ERR, "%s: Failed to get current GPIO input values",
              __func__);
      return MGV_ERROR;
    }
  if (buf[byte + 4] & bit)
    return MGV_HIGH;
  else
    return MGV_LOW;
}

static bool
hex_mcp2210_set_gpio_output (struct cgpu_info *hexmineru, const int pin,
                             const enum mcp2210_gpio_value d)
{
  const int bit = 1 << (pin % 8);
  const int byte = (pin / 8);
  hexmineru->cfg_gpio[pin] = 0;
  hexmineru->cfg_gpio[byte + 0xb] &= ~bit;
  if (d == MGV_HIGH)
    hexmineru->cfg_gpio[byte + 9] |= bit;
  else
    hexmineru->cfg_gpio[byte + 9] &= ~bit;
  return hex_mcp2210_set_cfg_gpio (hexmineru);
}

static bool
hex_mcp2210_spi_cancel (struct cgpu_info *hexmineru)
{
  uint8_t cmd[64], buf[64];
  memset (cmd, 0, 64);
  memset (buf, 0, 64);;
  cmd[0] = 0x11;
  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    return false;
  return (buf[1] == 0);
}

static bool
hex_mcp2210_set_cfg_spi (struct cgpu_info *hexmineru)
{
  uint8_t cmd[64], buf[64];

  memset (cmd, 0, 64);
  memset (buf, 0, 64);;
  cmd[0] = 0x40;
  memcpy (&cmd[4], hexmineru->cfg_spi, sizeof (hexmineru->cfg_spi));
  if (!hex_mcp2210_io (hexmineru, cmd, buf))
    {
      applog (LOG_ERR, "%s: Failed to set current %s config", __func__,
              "SPI");
      return false;
    }

  if (buf[1] != 0)
    {
      applog (LOG_ERR, "%s: Error setting current %s config (%d)", __func__,
              "SPI", buf[1]);
      return false;
    }
  return true;
}

static bool
hex_mcp2210_configure_spi (struct cgpu_info *hexmineru,
                           const uint32_t bitrate, const uint16_t idlechipsel,
                           const uint16_t activechipsel,
                           const uint16_t chipseltodatadelay,
                           const uint16_t lastbytetocsdelay,
                           const uint16_t midbytedelay)
{
  uint8_t *const cfg = hexmineru->cfg_spi;

  cfg[0] = (bitrate >> 0x00) & 0xff;
  cfg[1] = (bitrate >> 0x08) & 0xff;
  cfg[2] = (bitrate >> 0x10) & 0xff;
  cfg[3] = (bitrate >> 0x18) & 0xff;
  cfg[4] = (idlechipsel >> 0) & 0xff;
  cfg[5] = (idlechipsel >> 8) & 0xff;
  cfg[6] = (activechipsel >> 0) & 0xff;
  cfg[7] = (activechipsel >> 8) & 0xff;
  cfg[8] = (chipseltodatadelay >> 0) & 0xff;
  cfg[9] = (chipseltodatadelay >> 8) & 0xff;
  cfg[0xa] = (lastbytetocsdelay >> 0) & 0xff;
  cfg[0xb] = (lastbytetocsdelay >> 8) & 0xff;
  cfg[0xc] = (midbytedelay >> 0) & 0xff;
  cfg[0xd] = (midbytedelay >> 8) & 0xff;
  return hex_mcp2210_set_cfg_spi (hexmineru);
}

static bool
hex_mcp2210_set_spimode (struct cgpu_info *hexmineru, const uint8_t spimode)
{
  uint8_t *const cfg = hexmineru->cfg_spi;
  cfg[0x10] = spimode;
  return hex_mcp2210_set_cfg_spi (hexmineru);
}

static bool
hex_mcp2210_spi_transfer (struct cgpu_info *hexmineru, const void *const tx,
                          void *const rx, uint8_t sz)
{
  uint8_t *const cfg = hexmineru->cfg_spi;
  struct hexmineru_info *info = hexmineru->device_data;
  uint8_t cmd[64], buf[64];
  memset (cmd, 0, 64);
  memset (buf, 0, 64);;
  cmd[0] = 0x42;
  uint8_t *p = rx;
  if (unlikely (sz > 60))
    {
      applog (LOG_ERR, "%s: SPI transfer too long (%d bytes)", __func__, sz);
      return false;
    }
  cfg[0xe] = sz;
  cfg[0xf] = 0;
  if (sz != info->ltsz)
    {
      if (!hex_mcp2210_set_cfg_spi (hexmineru))
        return false;
      info->ltsz = sz;
    }
  cmd[1] = sz;
  memcpy (&cmd[4], tx, sz);
  if (unlikely (!hex_mcp2210_io (hexmineru, cmd, buf)))
    {
      applog (LOG_ERR, "%s: Failed to issue SPI transfer", __func__);
      return false;
    }
  while (true)
    {
      switch (buf[1])
        {
        case 0:
          cmd[1] = 0;
          break;
        case 0xf8:
          applog (LOG_DEBUG,
                  "%s: SPI transfer rejected temporarily (%d bytes remaining)",
                  __func__, sz);
          goto retry;
        default:
          applog (LOG_ERR, "%s: SPI transfer error (%d) (%d bytes remaining)",
                  __func__, buf[1], sz);
          return false;
        }
      if (buf[2] >= sz)
        {
          if (buf[2] > sz)
            applog (LOG_ERR, "%s: Received %d extra bytes in SPI transfer",
                    __func__, sz - buf[2]);
          memcpy (p, &buf[4], sz);
          return true;
        }
      memcpy (p, &buf[4], buf[2]);
      p += buf[2];
      sz -= buf[2];
    retry:
      if (unlikely (!hex_mcp2210_io (hexmineru, cmd, buf)))
        {
          applog (LOG_ERR,
                  "%s: Failed to continue SPI transfer (%d bytes remaining)",
                  __func__, sz);
          return false;
        }
    }
}

static bool
hex_nanofury_checkport (struct cgpu_info *hexmineru)
{
  int i;
  const char tmp = 0;
  char tmprx;
  for (i = 0; i < 9; ++i)
    if (MGV_ERROR == hex_mcp2210_get_gpio_input (hexmineru, i))
      goto fail;
  if (!hex_mcp2210_set_gpio_output (hexmineru, NANOFURY_GP_PIN_LED, MGV_HIGH))
    goto fail;
  if (!hex_mcp2210_set_gpio_output
      (hexmineru, NANOFURY_GP_PIN_PWR_EN, MGV_HIGH))
    goto fail;
  hex_mcp2210_spi_cancel (hexmineru);
  if (!hex_mcp2210_configure_spi (hexmineru, 200000, 0xffff, 0xffef, 0, 0, 0))
    goto fail;
  if (!hex_mcp2210_set_spimode (hexmineru, 0))
    goto fail;
  if (!hex_mcp2210_spi_transfer (hexmineru, &tmp, &tmprx, 1))
    goto fail;
  if (hex_mcp2210_get_gpio_input (hexmineru, NANOFURY_GP_PIN_SCK_OVR) !=
      MGV_LOW)
    goto fail;
  if (!hex_mcp2210_set_spimode (hexmineru, 2))
    goto fail;
  if (!hex_mcp2210_spi_transfer (hexmineru, &tmp, &tmprx, 1))
    goto fail;
  if (hex_mcp2210_get_gpio_input (hexmineru, NANOFURY_GP_PIN_SCK_OVR) !=
      MGV_HIGH)
    goto fail;
  if (!hex_mcp2210_set_spimode (hexmineru, 0))
    goto fail;
  if (!hex_mcp2210_spi_transfer (hexmineru, &tmp, &tmprx, 1))
    goto fail;
  if (hex_mcp2210_get_gpio_input (hexmineru, NANOFURY_GP_PIN_SCK_OVR) !=
      MGV_LOW)
    goto fail;
  return true;
fail:
  return false;
}

static bool
libhexu_nanofury_spi_reset (struct cgpu_info *hexmineru)
{
  int r;
  char tx[1] = { 0x81 };
  char buf[1];
  if (!hex_mcp2210_set_gpio_output
      (hexmineru, NANOFURY_GP_PIN_SCK_OVR, MGV_HIGH))
    return false;
  for (r = 0; r < 16; ++r)
    if (!hex_mcp2210_spi_transfer (hexmineru, tx, buf, 1))
      return false;
  if (hex_mcp2210_get_gpio_input (hexmineru, NANOFURY_GP_PIN_SCK_OVR) ==
      MGV_ERROR)
    return false;
  return true;
}

static bool
libhexu_nanofury_spi_txrx (struct cgpu_info *hexmineru, size_t * bufsz,
                           unsigned char *wrbuf, unsigned char *rdbuf,
                           bool trash)
{
  const uint8_t *ptrwrbuf = wrbuf;
  uint8_t *ptrrdbuf = rdbuf;
  size_t speedup = NANOFURY_MAX_BYTES_PER_SPI_TRANSFER;
  if (*bufsz == 80)
    speedup = 40;
  while (*bufsz >= speedup)
    {
      if (!hex_mcp2210_spi_transfer (hexmineru, ptrwrbuf, ptrrdbuf, speedup))
        goto err;
      if (!trash)
        ptrrdbuf += speedup;
      ptrwrbuf += speedup;
      *bufsz -= speedup;
    }
  if (*bufsz > 0)
    {
      if (!hex_mcp2210_spi_transfer (hexmineru, ptrwrbuf, ptrrdbuf, *bufsz))
        goto err;
    }
  return true;
err:
  return false;
}

static void
libhexu_spi_send_init (size_t * pos, unsigned char *where)
{
  /* Prepare internal buffers */
  /* PREPARE BUFFERS (INITIAL PROGRAMMING) */
  unsigned w[16];
  memset (&w, 0, sizeof (w));
  w[3] = htole32 (0xffffffff);
  w[4] = htole32 (0x80000000);
  w[15] = htole32 (0x00000280);
  libhexu_spi_emit_data (pos, where, 0x1000, w, 16 * 4);
  libhexu_spi_emit_data (pos, where, 0x1400, w, 8 * 4);
  memset (w, 0, sizeof (w));
  w[0] = htole32 (0x80000000);
  w[7] = htole32 (0x100);
  libhexu_spi_emit_data (pos, where, 0x1900, &w[0], 8 * 4);     /* Prepare MS and W buffers! */
}
