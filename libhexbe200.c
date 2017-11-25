static int64_t
timediff (const struct timeval *a, const struct timeval *b)
{
  struct timeval diff;
  timersub (a, b, &diff);
  return diff.tv_sec * 1000000 + diff.tv_usec;
}

static bool
libhex_be200_cachenonce (struct chip_resultsbe200 *nonce_cache,
                         uint32_t nonce)
{
  int i = 0;
  while (i < HEX_BE200NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEX_BE200NONCE_CASH_SIZE)
    return false;
  if (nonce_cache->nonce_cache_write_pos == HEX_BE200NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhex_be200_csum (unsigned char *startptr, unsigned char *endptr,
                   unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhex_be200_usb_dead (struct cgpu_info *hexminerbe200)
{
  struct cg_usb_device *usbdev;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  if (!info)
    return true;
  usbdev = hexminerbe200->usbdev;
  bool ret = (usbdev == NULL || usbdev->handle == NULL
              || hexminerbe200->shutdown || info->shut_read
              || info->shut_write || info->shut_reset
              || hexminerbe200->usbinfo.nodev
              || hexminerbe200->deven != DEV_ENABLED);
  return ret;
}

static int
libhex_be200_sendHashData (struct cgpu_info *hexminerbe200,
                           unsigned char *sendbuf, size_t buf_len)
{
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerbe200->usbdev;
  if (libhex_be200_usb_dead (hexminerbe200))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEX_BE200USB_WR_SIZE, buf_len - written),
                              &wrote, HEX_BE200USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;
  return written;
}

static void
libhex_be200_reset (struct cgpu_info *hexminerbe200)
{
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerbe200->usbdev;
  if (libhex_be200_usb_dead (hexminerbe200))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;
  info->usb_reset_count++;
}

static int
libhex_be200_readHashData (struct cgpu_info *hexminerbe200,
                           unsigned char *hash, int *hash_write_pos,
                           int timeout)
{
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct cg_usb_device *usbdev;
  int read = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerbe200->usbdev;
  if (libhex_be200_usb_dead (hexminerbe200))
    goto out;
  err =
    libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                          HEX_BE200USB_R_SIZE, &read, timeout);
  if (err == LIBUSB_SUCCESS)
    *hash_write_pos += MIN (read, HEX_BE200USB_R_SIZE);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    {
      info->shut_read = true;
    }
  return err;
}

/*LTE TBD!!! */
static double
be200_nonce_roll (struct cgpu_info *hexminerbe200, struct thr_info *thr,
                  uint32_t nonce, uint8_t work_id, uint8_t rolled,
                  uint8_t interval)
{
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct work *work_sub;
  int i = 1;
  double ret = 0;
  uint32_t nonce_to_test;
  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], rolled);
  if (test_nonce (work_sub, nonce))
    {
#ifdef DBG_HW_HEXBE200
      info->nincdec[interval]++;
#endif
      submit_tested_work_no_clone (thr, work_sub, true);
      ret = 1;
    }
  else
    {
      while (i < HEX_BE200NROLL)
        {
          nonce_to_test = nonce + i;
          if (test_nonce (work_sub, nonce_to_test))
            {
#ifdef DBG_HW_HEXBE200
              info->nincdec[interval + i]++;
#endif
              submit_tested_work_no_clone (thr, work_sub, true);
              ret = 1;
              break;
            }
          nonce_to_test = nonce - i;
          if (test_nonce (work_sub, nonce_to_test))
            {
#ifdef DBG_HW_HEXBE200
              info->nincdec[interval - i]++;
#endif
              submit_tested_work_no_clone (thr, work_sub, true);
              ret = 1;
              break;
            }
          i++;
        }
    }
  if (ret == 0)
    free_work (work_sub);
  return ret;
}

/*LTE TBD!!! */
static void
libhex_be200_getvoltage (uint16_t wr_bukvoltage,
                         int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhex_be200_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhex_be200_eatHashData (struct workbe200_result *wr, unsigned char *hash,
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
    return HEX_BE200BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEX_BE200BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  ok = (wr->command == 0x52)
    && ((wr->address == HEX_BE200WORKANSWER_ADR && wr->datalength == 0x05)
        || (wr->address == 0x3008 && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (places < HEX_BE200BASE_WORK_SIZE + wr->datalength * 2)
    return HEX_BE200BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEX_BE200BASE_WORK_SIZE + wr->datalength * 2 - 1;
  {
    libhex_be200_csum (hash + *hash_read_pos, csum_pos, &psum);
    if (psum != *csum_pos)
      {
        *hash_read_pos += 1;
        return HEX_BE200BUF_ERR;
      }
  }
  wrpos =
    (wr->address - HEX_BE200WORKANSWER_ADR) + HEX_BE200BASE_WORK_SIZE - 1;
  memcpy ((char *) &wr->startbyte + wrpos,
          &hash[*hash_read_pos + HEX_BE200BASE_WORK_SIZE - 1],
          wr->datalength * 2);
  *hash_read_pos += HEX_BE200BASE_WORK_SIZE + wr->datalength * 2;
  return HEX_BE200BUF_DATA;
}

static void
libhex_be200_shutdown (struct cgpu_info *hexminerbe200, uint16_t address,
                       uint16_t word)
{
  unsigned char status[65];
  unsigned char *sendbuf = &status[0];
  unsigned char *hash = &status[0];
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0, read = 0;
  int err = LIBUSB_SUCCESS;
  size_t buf_len = 8;
  usbdev = hexminerbe200->usbdev;
  uint16_t wr_adr = htole16 (address);
  uint16_t ledata = htole16 (word);
  status[0] = 0x53;
  status[1] = 0x01;
  status[2] = 0x57;
  memcpy (status + 3, &wr_adr, 2);
  memcpy (status + 5, &ledata, 2);
  libhex_be200_csum (status, status + 7, status + 7);
  if (usbdev == NULL || usbdev->handle == NULL || info->shut_read
      || info->shut_write || info->shut_reset)
    return;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEX_BE200USB_WR_SIZE, buf_len - written),
                              &wrote, HEX_BE200USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    return;
  //err = LIBUSB_SUCCESS;
  //while (err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x82, hash, HEX_BE200USB_R_SIZE,
                              &read, 200);
    }
}

static void
libhex_be200_set_words (struct cgpu_info *hexminerbe200, uint16_t address,
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
  libhex_be200_csum (status, status + 9, status + 9);
  libhex_be200_sendHashData (hexminerbe200, status, 10);
}
