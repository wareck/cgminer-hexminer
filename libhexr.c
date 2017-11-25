static int64_t
timediff (const struct timeval *a, const struct timeval *b)
{
  struct timeval diff;
  timersub (a, b, &diff);
  return diff.tv_sec * 1000000 + diff.tv_usec;
}

static bool
libhexr_cachenonce (struct chip_resultsr *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEXR_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEXR_NONCE_CASH_SIZE)
    return false;
  if (nonce_cache->nonce_cache_write_pos == HEXR_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhexr_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhexr_usb_dead (struct cgpu_info *hexminerr)
{
  struct cg_usb_device *usbdev;
  struct hexminerr_info *info = hexminerr->device_data;
  if (!info)
    return true;
  usbdev = hexminerr->usbdev;
  bool ret = (usbdev == NULL || usbdev->handle == NULL || hexminerr->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminerr->usbinfo.nodev || hexminerr->deven != DEV_ENABLED);
  return ret;
}

static int
libhexr_sendHashData (struct cgpu_info *hexminerr, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminerr_info *info = hexminerr->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerr->usbdev;
  if (libhexr_usb_dead (hexminerr))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEXR_USB_WR_SIZE, buf_len - written),
                              &wrote, HEXR_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;
  return written;
}

static void
libhexr_reset (struct cgpu_info *hexminerr)
{
  struct hexminerr_info *info = hexminerr->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerr->usbdev;
  if (libhexr_usb_dead (hexminerr))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;
  info->usb_reset_count++;
}

static int
libhexr_readHashData (struct cgpu_info *hexminerr, unsigned char *hash,
                      int *hash_write_pos, int timeout)
{
  struct hexminerr_info *info = hexminerr->device_data;
  struct cg_usb_device *usbdev;
  int read = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerr->usbdev;
  if (libhexr_usb_dead (hexminerr))
    goto out;
  err =
    libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                          HEXR_USB_R_SIZE, &read, timeout);
  if (err == LIBUSB_SUCCESS)
    *hash_write_pos += MIN (read, HEXR_USB_R_SIZE);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_read = true;
  return err;
}

static double
hexminerr_predecode_nonce_roll (struct cgpu_info *hexminerr,
                                struct thr_info *thr, uint32_t nonce,
                                uint8_t work_id, uint8_t rolled)
{
  struct hexminerr_info *info = hexminerr->device_data;
  struct work *work_sub;
  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], rolled);
  if (test_nonce (work_sub, nonce))
    {
      submit_tested_work_no_clone (thr, work_sub, true);
      return 1;
    }
  free_work (work_sub);
  return 0;
}

static void
libhexr_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhexr_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhexr_eatHashData (struct workr_result *wr, unsigned char *hash,
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
    return HEXR_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEXR_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  ok = (wr->command == 0x52)
    && ((wr->address == HEXR_WORKANSWER_ADR && wr->datalength == 0x05)
        || (wr->address == 0x3008 && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (places < HEXR_BASE_WORK_SIZE + wr->datalength * 2)
    return HEXR_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEXR_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  libhexr_csum (hash + *hash_read_pos, csum_pos, &psum);
  if (psum != *csum_pos)
    {
      *hash_read_pos += 1;
      return HEXR_BUF_ERR;
    }
  wrpos = (wr->address - HEXR_WORKANSWER_ADR) + HEXR_BASE_WORK_SIZE - 1;
  memcpy ((char *) &wr->startbyte + wrpos,
          &hash[*hash_read_pos + HEXR_BASE_WORK_SIZE - 1],
          wr->datalength * 2);
  *hash_read_pos += HEXR_BASE_WORK_SIZE + wr->datalength * 2;
  return HEXR_BUF_DATA;
}

static void
libhexr_shutdown (struct cgpu_info *hexminerr, uint16_t address,
                  uint16_t word)
{
  unsigned char status[65];
  unsigned char *sendbuf = &status[0];
  unsigned char *hash = &status[0];
  struct hexminerr_info *info = hexminerr->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0, read = 0;
  int err = LIBUSB_SUCCESS;
  size_t buf_len = 8;
  usbdev = hexminerr->usbdev;
  uint16_t wr_adr = htole16 (address);
  uint16_t ledata = htole16 (word);
  status[0] = 0x53;
  status[1] = 0x01;
  status[2] = 0x57;
  memcpy (status + 3, &wr_adr, 2);
  memcpy (status + 5, &ledata, 2);
  libhexr_csum (status, status + 7, status + 7);
  if (usbdev == NULL || usbdev->handle == NULL || info->shut_read
      || info->shut_write || info->shut_reset)
    return;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEXR_USB_WR_SIZE, buf_len - written),
                              &wrote, HEXR_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
        //applog(LOG_ERR, "SHUT lib 2 written %i = 8 buf_len",written);
    }
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    return;
  err =
      libusb_bulk_transfer (usbdev->handle, 0x82, hash, HEXR_USB_R_SIZE,
                              &read, 200);
}
/*
static void
libhexr_set_words (struct cgpu_info *hexminerr, uint16_t address,
                   uint16_t word, uint16_t word1)
{
  unsigned char status[12];
  uint16_t wr_adr = htole16 (address);
  uint16_t ledata = htole16 (word);
  uint16_t ledata1 = htole16 (word1);
  status[0] = 0x53;
  status[1] = 0x02;
  status[2] = 0x57;
  memcpy (status + 3, &wr_adr, 2);
  memcpy (status + 5, &ledata, 2);
  memcpy (status + 7, &ledata1, 2);
  libhexr_csum (status, status + 9, status + 9);
  libhexr_sendHashData (hexminerr, status, 10);
}
*/