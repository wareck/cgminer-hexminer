/*$T indentinput.c GC 1.140 10/16/13 10:19:47 */
#include "config.h"
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <unistd.h>
#ifndef WIN32
#include <sys/select.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#else
#include "compat.h"
#include <windows.h>
#include <io.h>
#endif
#include "elist.h"
#include "miner.h"
#include "usbutils.h"
#include "driver-hexminer3.h"
#include "util.h"
extern unsigned int work_block;
extern unsigned int work_pool_update;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
extern bool stale_work (struct work *work, bool share);
struct device_drv hexminer3_drv;
extern bool no_work;
int opt_hexminer3_chip_mask = 0xFF;
int opt_hexminer3_core_voltage = HEX3_DEFAULT_CORE_VOLTAGE;
#include "libhex3.c"

static int
hexminer3_send_task (struct hexminer3_task *ht, struct cgpu_info *hexminer3)
{
  int ret = 0;
  size_t nr_len = HEXMINER3_TASK_SIZE;
  struct hexminer3_info *info;
  info = hexminer3->device_data;
  libhex3_csum (&ht->startbyte, &ht->csum, &ht->csum);
  ret = libhex3_sendHashData (hexminer3, &ht->startbyte, nr_len);
  if (ret != nr_len)
    {
      libhex3_reset (hexminer3);
      info->usb_w_errors++;
      return -1;
    }
  return ret;
}

static inline void
hexminer3_create_task (bool reset_work, struct hexminer3_task *ht,
                       struct work *work)
{
  if (reset_work)
    ht->status = HEX3_STAT_NEW_WORK_CLEAR_OLD;
  else
    ht->status = HEX3_STAT_NEW_WORK;
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
  libhex3_calc_hexminer (work, ht);
}

static inline void
hexminer3_init_task_c (struct hexminer3_config_task *htc,
                       struct hexminer3_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminer3_config_task) - 6 - 2) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) info->frequency);
  libhex3_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->chip_mask = (uint8_t) info->chip_mask;
  htc->chipcount = htole16 (info->asic_count);
  libhex3_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminer3_init_task (struct hexminer3_task *ht, struct hexminer3_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINER3_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);
  ht->status = HEX3_STAT_NEW_WORK_CLEAR_OLD;
}

#ifdef PWR_HEX3
static bool
need_reset (struct cgpu_info *hexminer3)
{
  if (no_work)
    return false;
  struct hexminer3_info *info = hexminer3->device_data;
  time_t now = time (NULL);
  bool ret = false;
  int i = 0;
  int secs = 20;
  while (i < info->asic_count)
    {
      if (!info->chip_is_dead[i]
          && (info->chip_con_resets[i] < 5 && info->matching_work[i])
          && ((info->last_chip_valid_work[i] + secs) < now))
        {
          ret = true;
          info->chip_con_resets[i]++;
          info->last_chip_valid_work[i] = now;
          if (info->chip_con_resets[i] == 5)
            {
              info->chip_is_dead[i] = true;
            }
          break;
        }
      info->chip_con_resets[i] = 0;
      i++;
    }
  reajust_timings3 (hexminer3);
  info->timing_adjusted = true;
  return ret;
}
#endif

static struct cgpu_info *
hexminer3_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int asic_count, frequency;
  struct hexminer3_info *info;
  struct cgpu_info *hexminer3;
  bool configured = false;
  int i = 0;
  hexminer3 = usb_alloc_cgpu (&hexminer3_drv, HEX3_MINER_THREADS);
  if (!usb_init (hexminer3, dev, found))
    {
      usb_uninit (hexminer3);
      return NULL;
    }
  hexminer3->device_data = calloc (sizeof (struct hexminer3_info), 1);
  if (unlikely (!(hexminer3->device_data)))
    {
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }
  if (opt_hexminerc_options != NULL)
  	configured = (sscanf(opt_hexminerc_options, "%d:%d", &asic_count, &frequency) == 2);	
  
  if (opt_hexminer3_core_voltage < HEX3_MIN_COREMV
      || opt_hexminer3_core_voltage > HEX3_MAX_COREMV)
    {
      applog (LOG_ERR, "Invalid hexminer3-voltage %d must be %dmV - %dmV",
              opt_hexminer3_core_voltage, HEX3_MIN_COREMV, HEX3_MAX_COREMV);
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }
  info = hexminer3->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINER3_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }
  info->wr = (struct work3_result *) malloc (sizeof (struct work3_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_results3));
  info->readbuf = calloc (HEX3_HASH_BUF_SIZE, sizeof (unsigned char));
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEX3_DEFAULT_MINER_NUM;
  info->asic_count = HEX3_DEFAULT_ASIC_NUM;
  info->frequency = HEX3_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEX3_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminer3_core_voltage;
  info->chip_mask = opt_hexminer3_chip_mask;
  info->wr->buf_empty_space = 63;
  info->work_block_local = -1;
  info->work_pool_update = -1;
  info->reset_work = true;
  info->jobs_to_send = 20;
  info->roll = 0;
  info->timing_adjusted = false;
  info->ht = calloc (sizeof (struct hexminer3_task), 1);
  info->work = NULL;
  info->write_pos = 0;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  hexminer3_init_task (info->ht, info);
  gettimeofday (&info->last_wr, NULL);
  info->wsem_ustiming =
    (int64_t) (0x100000000ll / (info->asic_count * 1.7275 * info->frequency));
  if (!add_cgpu (hexminer3))
    {
      free (info->hexworks);
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      hexminer3 = usb_free_cgpu (hexminer3);
      usb_uninit (hexminer3);
      return NULL;
    }
  while (i < HEXMINER3_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    } i = 0;
  info->power_checked = time (NULL);
  while (i < HEX3_DEFAULT_ASIC_NUM)
    info->chip_is_dead[i++] = false;
  libhex3_generatenrange_new ((unsigned char *) &info->nonces_range,
                              info->asic_count);
  return hexminer3;
}

static void
hexminer3_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminer3_drv, hexminer3_detect_one);
}

static void
do_hexminer3_close (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;
  int i = 0;
  while (i < HEXMINER3_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  free (info->ht);
  if (info->work)
    free_work (info->work);
}

static void
hexminer3_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  do_hexminer3_close (thr);
  usb_nodev (hexminer3);
}

static bool
hexminer3_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;
  info->thr = thr;
  libhex3_set_word (hexminer3, 0x3080 + HEXMINER3_TASK_SIZE - 8, 0x0004);
  cgsleep_ms (100);
  struct hexminer3_config_task *htc;
  htc = calloc (sizeof (struct hexminer3_config_task), 1);
  hexminer3_init_task_c (htc, info);
  int ret = libhex3_sendHashData (hexminer3, &htc->startbyte,
                                  sizeof (struct hexminer3_config_task) - 2);
  if (ret != sizeof (struct hexminer3_config_task) - 2)
    applog (LOG_ERR, "HEX3 %i Send config failed", hexminer3->device_id);
  free (htc);
  ret = hexminer3_send_task (info->ht, hexminer3);
  cgsleep_ms (200);
  return true;
}

static void
do_write_hex3 (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;
  struct work *tmpwork = NULL;
  bool power;
  int jobs_to_send = info->jobs_to_send;
  int send_jobs, ret;

#ifdef PWR_HEX3
  if (time (NULL) - info->power_checked > 30)
    {
      info->power_checked = time (NULL);
      power = need_reset (hexminer3);
      if (power)
        {
          info->b_reset_count++;
          libhex3_set_word (hexminer3, 0x3080 + HEXMINER3_TASK_SIZE - 8,
                            0x0004);
          cgsleep_ms (100);
          struct hexminer3_config_task *htc;
          htc = calloc (sizeof (struct hexminer3_config_task), 1);
          hexminer3_init_task_c (htc, info);
          ret =
            libhex3_sendHashData (hexminer3, &htc->startbyte,
                                  sizeof (struct hexminer3_config_task) - 2);
          if (ret != sizeof (struct hexminer3_config_task) - 2)
            applog (LOG_ERR, "HEX3 %i Send config failed",
                    hexminer3->device_id);
          free (htc);
          hexminer3_init_task (info->ht, info);
          ret = hexminer3_send_task (info->ht, hexminer3);
          cgsleep_ms (200);
          gettimeofday (&info->last_wr, NULL);
          info->reset_work = true;
          jobs_to_send = 20;
        }
    }
#endif
  send_jobs = 0;
  while (!libhex3_usb_dead (hexminer3) && (send_jobs < jobs_to_send))
    {
    again:
      if (!info->work)
        {
          info->roll = 0;
          info->work = get_work (thr, thr->id);
          info->work->ping = 1;
          if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
            {
              info->reset_work = true;
              send_jobs = 0;
              jobs_to_send = 20;
              info->work_block_local = work_block;
              info->work_pool_update = work_pool_update;
            }
        }
      if (stale_work (info->work, false))
        {
          free_work (info->work);
          info->work = NULL;
          goto again;
        }
      if (info->write_pos >= HEXMINER3_ARRAY_SIZE_REAL || info->reset_work)
        info->write_pos = 0;
      info->work->subid = info->write_pos;
      tmpwork = copy_work_noffset_fast_no_id (info->work, info->roll++);
      hexminer3_create_task (info->reset_work, info->ht, tmpwork);
      free_work (info->hexworks[info->write_pos]);
      info->hexworks[info->write_pos] = tmpwork;
      if (info->work->drv_rolllimit)
        info->work->drv_rolllimit--;
      else
        {
          free_work (info->work);
          info->work = NULL;
        }
      ret = hexminer3_send_task (info->ht, hexminer3);
      info->write_pos++;
      send_jobs++;
      if (ret == HEXMINER3_TASK_SIZE && info->reset_work)
        {
          info->reset_work = false;
          gettimeofday (&info->last_wr, NULL);
        }
    }
}

static int64_t
hexminer3_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;
  uint32_t nonce;
  double found;
  double hash_count = 0;
  int i = 0, lastchippos = 0;
  int ret_r = 0;
  int64_t rethash_count = 0;
  int64_t tdif;
  int rminder = 0;
  struct timeval now;
  struct timeval diff;
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    {
      info->reset_work = true;
      info->jobs_to_send = 20;
      info->work_block_local = work_block;
      info->work_pool_update = work_pool_update;
      if (info->work)
        {
          free_work (info->work);
          info->work = NULL;
        }
      gettimeofday (&info->last_wr, NULL);
      do_write_hex3 (thr);
      goto done_wr;
    }
  gettimeofday (&now, NULL);
  tdif = timediff (&now, &info->last_wr);
  info->jobs_to_send = (int) (tdif / info->wsem_ustiming);
  rminder = (int) (tdif % info->wsem_ustiming);
  if (info->jobs_to_send > 0 || info->wr->buf_empty_space > 45)
    {
      gettimeofday (&info->last_wr, NULL);
      if (info->wr->buf_empty_space < 25)
        {
          goto done_wr;
        }
      now.tv_sec = 0;
      now.tv_usec = rminder;
      timersub (&info->last_wr, &now, &diff);
      memcpy (&info->last_wr, &diff, sizeof (struct timeval));
      if (info->wr->buf_empty_space > 45)
        info->jobs_to_send++;
      if (info->jobs_to_send > 20)
        info->jobs_to_send = 20;
      do_write_hex3 (thr);
    }
done_wr:
  if (libhex3_usb_dead (hexminer3))
    {
      hexminer3->shutdown = true;
      return -1;
    }
  if (info->hash_write_pos + HEX3_USB_R_SIZE > HEX3_HASH_BUF_SIZE_OK)
    {
      info->hash_write_pos = info->hash_write_pos - info->hash_read_pos;
      memcpy (info->readbuf, info->readbuf + info->hash_read_pos,
              info->hash_write_pos);
      info->hash_read_pos = 0;
    }
  if (info->hash_write_pos - info->hash_read_pos > 7)
    {
    again:
    	if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
      ret_r =
        libhex3_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEX3_BUF_DATA)
        goto out;
      if (info->wr->datalength == 1)
        goto done;
      if (info->wr->lastnonceid > HEXMINER3_ARRAY_SIZE_REAL)
        info->wr->lastnonceid = 0;
      nonce = htole32 (info->wr->lastnonce);
      i = 0;
      while (i < info->asic_count)
        {
          if (nonce < info->nonces_range[++i])
            {
              lastchippos = --i;
              break;
            }
        }
      if (i == info->asic_count)
        lastchippos = info->asic_count - 1;
      if (libhex3_cachenonce
          (&info->array_nonce_cache[lastchippos], info->wr->lastnonce))
        {
          found =
            hexminer3_predecode_nonce (hexminer3, thr, nonce,
                                       info->wr->lastnonceid);
          if (found > 0)
            {
#ifdef PWR_HEX3
              info->last_chip_valid_work[(uint8_t) lastchippos] = time (NULL);

#endif
              if (hash_count == 0)
                libhex3_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);
              hash_count += found;
              info->matching_work[(uint8_t) lastchippos]++;
            }
          else
            inc_hw_errors (thr);
        }
      else
        info->dupe[(uint8_t) lastchippos]++;
    out:
      if (ret_r == HEX3_BUF_ERR)
        info->usb_r_errors++;
    done:
      if (ret_r != HEX3_BUF_SKIP)
        goto again;
    }
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
  ret_r =
    libhex3_readHashData (hexminer3, info->readbuf, &info->hash_write_pos,
                          HEXMINER3_BULK_READ_TIMEOUT);
  if (ret_r != LIBUSB_SUCCESS)
    info->usb_bad_reads++;
  else
    info->usb_bad_reads = 0;
  if (info->usb_bad_reads > 20)
    libhex3_reset (hexminer3);
  new_block:
  rethash_count = (0xffffffffull * (int64_t) hash_count);
  if (libhex3_usb_dead (hexminer3))
    {
      hexminer3->shutdown = true;
      return -1;
    }
  return rethash_count;
}

#ifdef DBG_HEX3
static void
zero_hexminer3_stats (struct cgpu_info *hexminer3)
{
  if (!hexminer3->device_data)
    return;
  struct hexminer3_info *info = hexminer3->device_data;
  int i;
  for (i = 0; i < info->asic_count; i++)
    {
      info->matching_work[i] = 0;
      info->dupe[i] = 0;
    }
}
#endif

static void
get_hexminer3_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminer3)
{
  if (!hexminer3->device_data)
    return;
  struct hexminer3_info *info = hexminer3->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

extern void suffix_string (uint64_t val, char *buf, size_t bufsiz,
                           int sigdigits);

static struct api_data *
hexminer3_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct timeval now;
  struct hexminer3_info *info = cgpu->device_data;
  char displayed_hashes[16], displayed_rolling[16];
  double dev_runtime, hwp;
  uint64_t dh64, dr64;
  int i;
  if (!info)
    return NULL;
  hwp =
    (cgpu->hw_errors +
     cgpu->diff1) ? (double) (cgpu->hw_errors) / (double) (cgpu->hw_errors +
                                                           cgpu->diff1) : 0;
  if (cgpu->dev_start_tv.tv_sec == 0)
    dev_runtime = total_secs;
  else
    {
      cgtime (&now);
      dev_runtime = tdiff (&now, &(cgpu->dev_start_tv));
    }
  if (dev_runtime < 1.0)
    dev_runtime = 1.0;
  dh64 = (double) cgpu->total_mhashes / dev_runtime * 1000000ull;
  dr64 = (double) cgpu->rolling * 1000000ull;
  suffix_string (dh64, displayed_hashes, sizeof (displayed_hashes), 4);
  suffix_string (dr64, displayed_rolling, sizeof (displayed_rolling), 4);
  root = api_add_string (root, "MHS 5s", displayed_rolling, true);
  root = api_add_string (root, "MHS av", displayed_hashes, true);
  root = api_add_int (root, "Hardware Errors", &(cgpu->hw_errors), true);
  root = api_add_percent (root, "Hardware Errors%", &hwp, true);
  root = api_add_int (root, "USB Read Errors", &(info->usb_r_errors), true);
  root = api_add_int (root, "USB Write Errors", &(info->usb_w_errors), true);
  root =
    api_add_int (root, "USB Reset Count", &(info->usb_reset_count), true);
  root =
    api_add_int (root, "Miner Reset Count", &(info->b_reset_count), true);
  root =
    api_add_time (root, "Last Share Time", &(cgpu->last_share_pool_time),
                  true);
  root = api_add_int (root, "Chip Count", &(info->asic_count), true);
  root = api_add_int (root, "Frequency", &(info->frequency), true);
  root = api_add_int (root, "Core Voltage", &(info->core_voltage), true);
  root =
    api_add_int (root, "PIC Voltage Readings", &(info->pic_voltage_readings),
                 true);
  for (i = 0; i < info->asic_count; i++)
    {
      char mcw[24];
      sprintf (mcw, "Chip%d Nonces", i + 1);
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
    }
  return root;
}

struct device_drv hexminer3_drv = {
  .drv_id = DRIVER_hexminer3,
  .dname = "hexminer3",
  .name = "HEX3",
  .drv_detect = hexminer3_detect,
  .thread_init = hexminer3_thread_init,
  .hash_work = hash_driver_work,
  .scanwork = hexminer3_scanhash,
  .get_api_stats = hexminer3_api_stats,
  .get_statline_before = get_hexminer3_statline_before,
#ifdef DBG_HEX3
  .zero_stats = zero_hexminer3_stats,
#endif
  .thread_shutdown = hexminer3_shutdown,
};
