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
const uint32_t SHA256_K3[3] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf };

static int64_t
timediff (const struct timeval *a, const struct timeval *b)
{
  struct timeval diff;
  timersub (a, b, &diff);
  return diff.tv_sec * 1000000 + diff.tv_usec;
}

static void
reajust_timings3 (struct cgpu_info *hexminer3)
{
  struct hexminer3_info *info = hexminer3->device_data;
  int i = 0;
  int chips = 0;
  while (i < HEX3_DEFAULT_ASIC_NUM)
    if (!info->chip_is_dead[i++])
      chips++;
  if (chips == 0)
    chips = 6;
  info->wsem_ustiming =
    (int64_t) (0x100000000ll / (chips * 1.7275 * info->frequency));
}

static bool
libhex3_cachenonce (struct chip_results3 *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEX3_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEX3_NONCE_CASH_SIZE)
    return false;
  if (nonce_cache->nonce_cache_write_pos == HEX3_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhex3_calc_hexminer (struct work *work, struct hexminer3_task *ht)
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
  R (A, B, C, D, E, F, G, H, data[0], SHA256_K3[0]);
  a0a1a2e0e1e2[0] = htole32 (H);
  a0a1a2e0e1e2[3] = htole32 (D);
  R (H, A, B, C, D, E, F, G, data[1], SHA256_K3[1]);
  a0a1a2e0e1e2[1] = htole32 (G);
  a0a1a2e0e1e2[4] = htole32 (C);
  R (G, H, A, B, C, D, E, F, data[2], SHA256_K3[2]);
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
libhex3_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}


static bool
libhex3_usb_dead (struct cgpu_info *hexminer3)
{
  struct cg_usb_device *usbdev;
  struct hexminer3_info *info = hexminer3->device_data;
  if (!info)
    return true;
  usbdev = hexminer3->usbdev;
  bool ret = (usbdev == NULL || usbdev->handle == NULL || hexminer3->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminer3->usbinfo.nodev || hexminer3->deven != DEV_ENABLED);
  return ret;
}

static int
libhex3_sendHashData (struct cgpu_info *hexminer3, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x02, sendbuf + written,
                              MIN (HEX3_USB_WR_SIZE, buf_len - written),
                              &wrote, HEX3_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;
  return written;
}

static void
libhex3_reset (struct cgpu_info *hexminer3)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;
  info->usb_reset_count++;
}

static int
libhex3_readHashData (struct cgpu_info *hexminer3, unsigned char *hash,
                      int *hash_write_pos, int timeout)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int read = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  err =
    libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                          HEX3_USB_R_SIZE, &read, timeout);
  if (err == LIBUSB_SUCCESS)
    *hash_write_pos += MIN (read, HEX3_USB_R_SIZE);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_read = true;
  return err;
}

extern bool submit_tested_work_fast_clone (struct thr_info *thr,
                                           struct work *work, bool diff1);
static int
hexminer3_predecode_nonce (struct cgpu_info *hexminer3, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id)
{
  struct hexminer3_info *info = hexminer3->device_data;
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
libhex3_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhex3_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static void
libhex3_generatenrange_new (unsigned char *buf, int asic_num)
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

static int
libhex3_eatHashData (struct work3_result *wr, unsigned char *hash,
                     int *hash_read_pos, int *hash_write_pos)
{
  uint8_t psum;
  int wrpos;
  unsigned char *csum_pos;
  bool ok;
eat:
  while (*hash_read_pos < *hash_write_pos && hash[*hash_read_pos] != 0x53)
    {
      *hash_read_pos += 1;
    }
  if (*hash_write_pos - *hash_read_pos < 8)
    return HEX3_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEX3_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  ok = (wr->command == 0x52) &&
    ((wr->address == HEX3_WORKANSWER_ADR && wr->datalength == 0x05)
     || (wr->address == 0x3008 && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (*hash_write_pos - *hash_read_pos <
      HEX3_BASE_WORK_SIZE + wr->datalength * 2)
    return HEX3_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEX3_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  libhex3_csum (hash + *hash_read_pos, csum_pos, &psum);
  if (psum != *csum_pos)
    {
      *hash_read_pos += 1;
      return HEX3_BUF_ERR;
    }
  wrpos = (wr->address - HEX3_WORKANSWER_ADR) + HEX3_BASE_WORK_SIZE - 1;
  memcpy ((char *) &wr->startbyte + wrpos,
          &hash[*hash_read_pos + HEX3_BASE_WORK_SIZE - 1],
          wr->datalength * 2);
  *hash_read_pos += HEX3_BASE_WORK_SIZE + wr->datalength * 2;
  return HEX3_BUF_DATA;
}

static void
libhex3_set_word (struct cgpu_info *hexminer3, uint16_t address,
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
  libhex3_csum (status, status + 7, status + 7);
  libhex3_sendHashData (hexminer3, status, 8);
}
