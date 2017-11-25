/*$T indentinput.c GC 1.140 10/16/13 10:20:34 */
#define rotate(x, y)	((x << y) | (x >> (sizeof(x) * 8 - y)))
#define rotr(x, y)		((x >> y) | (x << (sizeof(x) * 8 - y)))
#define R(a, b, c, d, e, f, g, h, w, k) \
		h = h + \
		(rotate(e, 26) ^ rotate(e, 21) ^ rotate(e, 7)) + \
		(g ^ (e & (f ^ g))) + \
		k + \
		w; \
	d = d + h; \
	h = h + (rotate(a, 30) ^ rotate(a, 19) ^ rotate(a, 10)) + ((a & b) | (c & (a | b)))
const uint32_t SHA256_KC[3] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf };

static int64_t
timediff (const struct timeval *a, const struct timeval *b)
{
  struct timeval diff;
  timersub (a, b, &diff);
  return diff.tv_sec * 1000000 + diff.tv_usec;
}

static bool
libhexc_cachenonce (struct chip_resultsc *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEXC_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEXC_NONCE_CASH_SIZE)
    return false;
  if (nonce_cache->nonce_cache_write_pos == HEXC_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhexc_generatenrange_new (unsigned char *buf, int asic_num)
{
  uint32_t nonceAdd;
  int noncePos;
  int64_t nonceCalc = 0x100000000ll;
  nonceCalc /= asic_num;
  nonceAdd = (uint32_t) nonceCalc;
  uint32_t chip_noce;
  for (noncePos = 0; noncePos < asic_num; noncePos++)
    {                           /* * chip_noce = htole32(noncePos * nonceAdd);
                                 */
      chip_noce = noncePos * nonceAdd;
      memcpy (buf + noncePos * 4, &chip_noce, 4);
    }
}

static void
libhexc_calc_hexminer (struct work *work, struct hexminerc_task *ht)
{
  uint32_t a0a1a2e0e1e2[6];
  uint32_t A, B, C, D, E, F, G, H;
  uint32_t state[8];
  uint32_t data[3];
  memcpy (&state, work->midstate, 32);
  memcpy (&data, work->data + 64, 12);

#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
int i;
  for (i = 0; i < 8; i++)
    state[i] = htole32 (state[i]);
  for (i = 0; i < 3; i++)
    data[i] = htole32 (data[i]);

#endif
 A = state[0]; 
  B = state[1];
  C = state[2];
  D = state[3];
  E = state[4];
  F = state[5];
  G = state[6];
  H = state[7];
  R (A, B, C, D, E, F, G, H, data[0], SHA256_KC[0]);
  a0a1a2e0e1e2[0] = htole32 (H);
  a0a1a2e0e1e2[3] = htole32 (D);
  R (H, A, B, C, D, E, F, G, data[1], SHA256_KC[1]);
  a0a1a2e0e1e2[1] = htole32 (G);
  a0a1a2e0e1e2[4] = htole32 (C);
  R (G, H, A, B, C, D, E, F, data[2], SHA256_KC[2]);
  a0a1a2e0e1e2[2] = htole32 (F);
  a0a1a2e0e1e2[5] = htole32 (B);
  memcpy (&ht->a0, &a0a1a2e0e1e2[0], 4);
  memcpy (&ht->a1, &a0a1a2e0e1e2[1], 4);
  memcpy (&ht->a2, &a0a1a2e0e1e2[2], 4);
  memcpy (&ht->e0, &a0a1a2e0e1e2[3], 4);
  memcpy (&ht->e1, &a0a1a2e0e1e2[4], 4);
  memcpy (&ht->e2, &a0a1a2e0e1e2[5], 4);
}

static void
libhexc_generateclk (uint16_t HashClock, uint16_t XCLKIN, uint32_t * res)
{
  uint32_t configL = 0;
  uint32_t configH = 0;
  int RValue = XCLKIN;
  int NValue = (HashClock * 2 * RValue / XCLKIN);
  configL =
    ((uint32_t) RValue << 29) | ((uint32_t) NValue << 18) |
    HEXC_CLOCK_LOW_CFG;
  configH = ((uint32_t) RValue >> 3) | HEXC_CLOCK_HIGH_CFG;
  res[0] = htole32 (configL);
  res[1] = htole32 (configH);
}

static void
libhexc_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhexc_usb_dead (struct cgpu_info *hexminerc)
{
  struct cg_usb_device *usbdev;
  struct hexminerc_info *info = hexminerc->device_data;
  if (!info)
    return true;
  usbdev = hexminerc->usbdev;
  bool ret = (usbdev == NULL || usbdev->handle == NULL || hexminerc->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminerc->usbinfo.nodev || hexminerc->deven != DEV_ENABLED);
  return ret;
}

static int
libhexc_sendHashData (struct cgpu_info *hexminerc, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminerc_info *info = hexminerc->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerc->usbdev;
  if (libhexc_usb_dead (hexminerc))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEXC_USB_WR_SIZE, buf_len - written),
                              &wrote, HEXC_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;
  return written;
}

static void
libhexc_reset (struct cgpu_info *hexminerc)
{
  struct hexminerc_info *info = hexminerc->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerc->usbdev;
  if (libhexc_usb_dead (hexminerc))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;
  info->usb_reset_count++;
}

static int
libhexc_readHashData (struct cgpu_info *hexminerc, unsigned char *hash,
                      int *hash_write_pos, int timeout)
{
  struct hexminerc_info *info = hexminerc->device_data;
  struct cg_usb_device *usbdev;
  int read = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminerc->usbdev;
  if (libhexc_usb_dead (hexminerc))
    goto out;
  err =
    libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                          HEXC_USB_R_SIZE, &read, timeout);
  if (err == LIBUSB_SUCCESS)
    *hash_write_pos += MIN (read, HEXC_USB_R_SIZE);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
      info->shut_read = true;
  return err;
}

extern bool submit_tested_work_fast_clone (struct thr_info *thr,
                                           struct work *work, bool diff1);

static int
hexminerc_predecode_nonce (struct cgpu_info *hexminerc, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id)
{
  struct hexminerc_info *info = hexminerc->device_data;
  if (info->hexworks[work_id]->pool == NULL)
      return 0;
  if (test_nonce (info->hexworks[work_id], nonce))
    {
      submit_tested_work_fast_clone (thr, info->hexworks[work_id], true);
      return 1;
    }
  return 0;
}

static void
libhexc_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 1000 * (float) 3.3 /
             ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhexc_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhexc_eatHashData (struct workc_result *wr, unsigned char *hash,
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
    return HEXC_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEXC_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  ok = (wr->command == 0x52)
    && ((wr->address == HEXC_WORKANSWER_ADR && wr->datalength == 0x06));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (places < HEXC_BASE_WORK_SIZE + wr->datalength * 2)
    return HEXC_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEXC_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  libhexc_csum (hash + *hash_read_pos, csum_pos, &psum);
  if (psum != *csum_pos)
    {
      *hash_read_pos += 1;
      return HEXC_BUF_ERR;
    }
  wrpos = (wr->address - HEXC_WORKANSWER_ADR) + HEXC_BASE_WORK_SIZE - 1;
  memcpy ((char *) &wr->startbyte + wrpos,
          &hash[*hash_read_pos + HEXC_BASE_WORK_SIZE - 1],
          wr->datalength * 2);
  *hash_read_pos += HEXC_BASE_WORK_SIZE + wr->datalength * 2;
  return HEXC_BUF_DATA;
}

static void
libhexc_set_word (struct cgpu_info *hexminerc, uint16_t address,
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
  libhexc_csum (status, status + 7, status + 7);
  libhexc_sendHashData (hexminerc, status, 8);
}
