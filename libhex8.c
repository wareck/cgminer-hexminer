static void
reajust_timings (struct cgpu_info *hexminer8)
{
  struct hexminer8_info *info = hexminer8->device_data;
  int i = 0;
  int engines = 0;
  while (i < HEX8_DEFAULT_ASIC_NUM)
    engines += info->engines[i++];
  if (engines == 0)
    engines = 6 * 32;
  info->wsem_ustiming =
    (int64_t) (0x100000000ll / (info->frequency * 4 * engines));
  info->ping_period =
    (int) (1000 * 1000 / info->wsem_ustiming * 60 / (engines / 32) / 17);
}

static int64_t
timediff (const struct timeval *a, const struct timeval *b)
{
  struct timeval diff;
  timersub (a, b, &diff);
  return diff.tv_sec * 1000000 + diff.tv_usec;
}

/*Thank you Zefir !!! */
static uint32_t
libhex8_get_target (double diff)
{
  unsigned nBits;
  int shift = 29;
  double ftarg = (double) 0x0000ffff / diff;
  while (ftarg < (double) 0x00008000)
    {
      shift--;
      ftarg *= 256.0;
    } while (ftarg >= (double) 0x00800000)
    {
      shift++;
      ftarg /= 256.0;
    } nBits = (int) ftarg + (shift << 24);
  return nBits;
}

static bool
libhex8_cachenonce (struct chip_results8 *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEX8_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEX8_NONCE_CASH_SIZE)
    return false;
  if (nonce_cache->nonce_cache_write_pos == HEX8_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhex8_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhex8_usb_dead (struct cgpu_info *hexminer8)
{
  struct cg_usb_device *usbdev;
  struct hexminer8_info *info = hexminer8->device_data;
  if (!info)
    return true;
  usbdev = hexminer8->usbdev;
  bool ret = (usbdev == NULL || usbdev->handle == NULL || hexminer8->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminer8->usbinfo.nodev || hexminer8->deven != DEV_ENABLED);
  return ret;
}

static int
libhex8_sendHashData (struct cgpu_info *hexminer8, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEX8_USB_WR_SIZE, buf_len - written),
                              &wrote, HEX8_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;
  return written;
}

static void
libhex8_reset (struct cgpu_info *hexminer8)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;
  info->usb_reset_count++;
}

static int
libhex8_readHashData (struct cgpu_info *hexminer8, unsigned char *hash,
                      int *hash_write_pos, int timeout)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int read = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  err =
    libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                          HEX8_USB_R_SIZE, &read, timeout);
  if (err == LIBUSB_SUCCESS)
    *hash_write_pos += MIN (read, HEX8_USB_R_SIZE);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_read = true;
  return err;
}

extern bool submit_tested_work_fast_clone (struct thr_info *thr,
                                           struct work *work, bool diff1);
static double
hexminer8_predecode_nonce (struct cgpu_info *hexminer8, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id, bool diff1)
{
  struct hexminer8_info *info = hexminer8->device_data;
  if (info->hexworks[work_id]->pool == NULL)
    return 0;
  double diff = (diff1
                 || info->hexworks[work_id]->ping ? 1 : info->
                 hexworks[work_id]->work_difficulty);
  if (test_nonce (info->hexworks[work_id], nonce))
    {
      submit_tested_work_fast_clone (thr, info->hexworks[work_id], diff1
                                     || info->hexworks[work_id]->ping);
      return diff;
    }
  return -diff;
}

static void
libhex8_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhex8_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhex8_eatHashData (struct work8_result *wr, unsigned char *hash,
                     int *hash_read_pos, int *hash_write_pos)
{
  uint8_t psum;
  int wrpos;
  unsigned char *csum_pos;
  bool ok;
  int places = 0;
eat:
  while (*hash_read_pos < *hash_write_pos && hash[*hash_read_pos] != 0x53)
    *hash_read_pos += 1;
  places = *hash_write_pos - *hash_read_pos;
  if (places < 8)
    return HEX8_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEX8_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  ok = (wr->command == 0x52)
    && ((wr->address == HEX8_WORKANSWER_ADR && wr->datalength == 0x06)
        || (wr->address == 0x3008 && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (places < HEX8_BASE_WORK_SIZE + wr->datalength * 2)
    return HEX8_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEX8_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  libhex8_csum (hash + *hash_read_pos, csum_pos, &psum);
  if (psum != *csum_pos)
    {
      *hash_read_pos += 1;
      return HEX8_BUF_ERR;
    }
  wrpos = (wr->address - HEX8_WORKANSWER_ADR) + HEX8_BASE_WORK_SIZE - 1;
  memcpy ((char *) &wr->startbyte + wrpos,
          &hash[*hash_read_pos + HEX8_BASE_WORK_SIZE - 1],
          wr->datalength * 2);
  *hash_read_pos += HEX8_BASE_WORK_SIZE + wr->datalength * 2;
  return HEX8_BUF_DATA;
}

static void
libhex8_set_word (struct cgpu_info *hexminer8, uint16_t address,
                  uint16_t word)
{
  unsigned char status[10];
  uint16_t wr_adr = htole16 (address);
  uint16_t ledata = htole16 (word);
  status[0] = 0x53;
  status[1] = 0x01;
  status[2] = 0x57;
  memcpy (status + 3, &wr_adr, 2);
  memcpy (status + 5, &ledata, 2);
  libhex8_csum (status, status + 7, status + 7);
  libhex8_sendHashData (hexminer8, status, 8);
}
