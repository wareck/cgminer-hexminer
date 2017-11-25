/*$T indentinput.c GC 1.140 10/16/13 10:19:47*/
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
#include "driver-hexminerbe200.h"
#include <math.h>
#include "util.h"
extern unsigned int work_block;
extern unsigned int work_pool_update;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
extern bool no_work;
struct device_drv hexminerbe200_drv;
int opt_hexminerbe200_chip_mask = 0xFFFF;
int opt_hexminerbe200_hw_err_res = 10;
int opt_hexminerbe200_nonce_timeout_secs = 10;
int opt_hexminerbe200_diff = 0;
int opt_hexminerbe200_skip_hw_res = 1;
int opt_hexminerbe200_core_voltage = HEX_BE200DEFAULT_CORE_VOLTAGE;
int opt_hexminerbe200_pic_roll = 0x3c;
#include "libhexbe200.c"

static int
hexminerbe200_send_task (struct hexminerbe200_task *ht,
                         struct cgpu_info *hexminerbe200)
{
  int ret = 0;
  size_t nr_len = HEXMINERBE200_TASK_SIZE;
  struct hexminerbe200_info *info;
  info = hexminerbe200->device_data;
  libhex_be200_csum (&ht->startbyte, &ht->csum, &ht->csum);
  ret = libhex_be200_sendHashData (hexminerbe200, &ht->startbyte, nr_len);
  if (ret != nr_len && info->usb_bad_reads > -1)
    {
      libhex_be200_reset (hexminerbe200);
      info->usb_w_errors++;
      return -1;
    }
  return ret;
}

static inline void
hexminerbe200_create_task (bool reset_work, struct hexminerbe200_task *ht,
                           struct work *work)
{
  if (reset_work)
    ht->status = (uint8_t) HEX_BE200STAT_NEW_WORK_CLEAR_OLD;
  else
    ht->status = (uint8_t) HEX_BE200STAT_NEW_WORK;
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
}

static inline void
hexminerbe200_init_task_c (struct hexminerbe200_config_task *htc,
                           struct hexminerbe200_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminerbe200_config_task) - 6) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) (info->frequency));
  libhex_be200_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->chip_mask = (uint16_t) htole16 (info->chip_mask);
  libhex_be200_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminerbe200_init_task (struct hexminerbe200_task *ht,
                         struct hexminerbe200_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERBE200_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);
  ht->status = (uint8_t) HEX_BE200STAT_NEW_WORK_CLEAR_OLD;
  ht->roll = (uint8_t) info->pic_roll;
  ht->diff = htole16 (info->diff);
}

static void
do_reset (struct cgpu_info *hexminerbe200)
{
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  time_t now = time (NULL);
  uint8_t i = 0;
  if (no_work)
    {
      while (i < info->asic_count)
        {
          info->hw_err[i] = 0;
          /*Pools and all to settle 10 secs add */
          if (info->last_chip_valid_work[i] > 0)
            info->last_chip_valid_work[i] = now + 20;
          i++;
        }
      return;
    }
  while (i < info->asic_count)
    {
      if (!info->chips_enabled[i])
        {
          i++;
          continue;
        }
      if (info->last_chip_valid_work[i] > 0
          && ((now - info->last_chip_valid_work[i]) >=
              info->opt_hexminerbe200_nonce_timeout_secs))
        {
          if (info->opt_hexminerbe200_skip_hw_res == 0)
            {
              libhex_be200_set_words (hexminerbe200, HEX_BE200RES_ADR,
                                      (uint16_t) i, 0x0007);
              applog (LOG_ERR,
                      "HEXE %i Chip[%i] last received nonce %i secs ago reset",
                      hexminerbe200->device_id, i,
                      (int) (now - info->last_chip_valid_work[i]));
            }
          else
            {
              applog (LOG_ERR,
                      "HEXE %i Chip[%i] last received nonce %i secs ago log only",
                      hexminerbe200->device_id, i,
                      (int) (now - info->last_chip_valid_work[i]));
            }
          info->last_chip_valid_work[i] = now;
          info->hw_err[i] = 0;
          info->res_timeout_err[i]++;
          info->b_reset_count++;
          break;
        }
      if (info->hw_err[i] > info->opt_hexminerbe200_hw_err_res)
        {
          if (info->opt_hexminerbe200_skip_hw_res == 0)
            {
              libhex_be200_set_words (hexminerbe200, HEX_BE200RES_ADR,
                                      (uint16_t) i, 0x0007);
              applog (LOG_ERR,
                      "HEXE %i Chip[%i] %i consecutive HW errors reset",
                      hexminerbe200->device_id, i, (int) info->hw_err[i]);
            }
          else
              applog (LOG_ERR,
                      "HEXE %i Chip[%i] %i consecutive HW errors log only",
                      hexminerbe200->device_id, i, (int) info->hw_err[i]);
          info->last_chip_valid_work[i] = now;
          info->hw_err[i] = 0;
          info->res_hew_err[i]++;
          info->b_reset_count++;
          break;
        }
      i++;
    }
}

static struct cgpu_info *
hexminerbe200_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int asic_count, frequency;
  struct hexminerbe200_info *info;
  struct cgpu_info *hexminerbe200;
  bool configured = false;
  int i = 0;
  hexminerbe200 = usb_alloc_cgpu (&hexminerbe200_drv, HEX_BE200MINER_THREADS);
  if (!usb_init (hexminerbe200, dev, found))
    {
      usb_uninit (hexminerbe200);
      return NULL;
    }
  hexminerbe200->device_data = calloc (sizeof (struct hexminerbe200_info), 1);
  if (unlikely (!(hexminerbe200->device_data)))
    {
      hexminerbe200->device_data = NULL;
      usb_uninit (hexminerbe200);
      return NULL;
    }
   if (opt_hexminerbe200_options != NULL)
  	configured = (sscanf(opt_hexminerbe200_options, "%d:%d", &asic_count, &frequency) == 2);	
  
  if (opt_hexminerbe200_core_voltage < HEX_BE200MIN_COREMV
      || opt_hexminerbe200_core_voltage > HEX_BE200MAX_COREMV)
    {
      applog (LOG_ERR, "Invalid hexminerbe200-voltage %d must be %dmV - %dmV",
              opt_hexminerbe200_core_voltage, HEX_BE200MIN_COREMV,
              HEX_BE200MAX_COREMV);
      free (hexminerbe200->device_data);
      hexminerbe200->device_data = NULL;
      usb_uninit (hexminerbe200);
      return NULL;
    }
  info = hexminerbe200->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERBE200_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerbe200->device_data);
      hexminerbe200->device_data = NULL;
      usb_uninit (hexminerbe200);
      return NULL;
    }
  info->wr =
    (struct workbe200_result *) malloc (sizeof (struct workbe200_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsbe200));
  info->readbuf = calloc (HEX_BE200HASH_BUF_SIZE, sizeof (unsigned char));
  info->write_pos = 0;
  info->usb_bad_reads = -30;
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEX_BE200DEFAULT_MINER_NUM;
  info->asic_count = HEX_BE200DEFAULT_ASIC_NUM;
  info->frequency = HEX_BE200DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEX_BE200DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerbe200_core_voltage;
  info->opt_hexminerbe200_hw_err_res = opt_hexminerbe200_hw_err_res;
  info->opt_hexminerbe200_nonce_timeout_secs =
    opt_hexminerbe200_nonce_timeout_secs;
  info->diff = opt_hexminerbe200_diff;
  info->opt_hexminerbe200_skip_hw_res = opt_hexminerbe200_skip_hw_res;
  info->chip_mask = opt_hexminerbe200_chip_mask;
  info->wr->buf_empty_space = 63;
  info->work_block_local = -1;
  info->work_pool_update = -1;
  info->reset_work = true;
  info->jobs_to_send = 1;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  info->pic_roll = opt_hexminerbe200_pic_roll;
  gettimeofday (&info->last_wr, NULL);
  info->wsem_ustiming =
    (int64_t) (0x100000000ll /
               (info->asic_count * info->frequency * 32 *
                (info->pic_roll + 1) * 0.75));
  info->ht = calloc (sizeof (struct hexminerbe200_task), 1);
  hexminerbe200_init_task (info->ht, info);
  while (i < HEXMINERBE200_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    }
  if (!add_cgpu (hexminerbe200))
    {
      free (info->hexworks);
      free (hexminerbe200->device_data);
      hexminerbe200->device_data = NULL;
      hexminerbe200 = usb_free_cgpu (hexminerbe200);
      usb_uninit (hexminerbe200);
      return NULL;
    }
  return hexminerbe200;
}

static void
hexminerbe200_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerbe200_drv, hexminerbe200_detect_one);
}

static void
do_hexminerbe200_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerbe200 = thr->cgpu;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  int i = 0;
  while (i < HEXMINERBE200_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  free (info->ht);
}

static void
hexminerbe200_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminerbe200 = thr->cgpu;
  do_hexminerbe200_close (thr);
  /*Power off and try to flush USB */
  libhex_be200_shutdown (hexminerbe200, 0x30AE, 0x0004);
  usb_nodev (hexminerbe200);
}

static bool
hexminerbe200_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerbe200 = thr->cgpu;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct hexminerbe200_config_task *htc;
  uint8_t i = 0;
  info->thr = thr;
  htc = calloc (sizeof (struct hexminerbe200_config_task), 1);
  hexminerbe200_init_task_c (htc, info);
  cgsleep_ms (200);
  int ret = libhex_be200_sendHashData (hexminerbe200, &htc->startbyte,
                                       sizeof (struct
                                               hexminerbe200_config_task));
  if (ret != sizeof (struct hexminerbe200_config_task))
  	{
    applog (LOG_ERR, "HEXE %i Send config failed disabling!!", hexminerbe200->device_id);
   	libhex_be200_reset(hexminerbe200);
   	info->shut_write = true;
   }
  free (htc);
  /*Pools and all to settle 10 secs add */
  time_t now = time (NULL) + 10;
  while (i < info->asic_count)
    {
      info->chips_enabled[i] =
        (((uint16_t) info->chip_mask & (uint16_t) pow (2, i)) ==
         (uint16_t) pow (2, i));
      info->last_chip_valid_work[i] = now;
      i++;
    }
  return true;
}

static void
do_write_hexm (struct thr_info *thr)
{
  struct cgpu_info *hexminerbe200 = thr->cgpu;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct work *work;
  int jobs_to_send = info->jobs_to_send;
  int send_jobs, ret;
  send_jobs = 0;
  while (!libhex_be200_usb_dead (hexminerbe200) && (send_jobs < jobs_to_send))
    {
      work = get_work (thr, thr->id);
      if (info->diff == 0)
        work->ping = 1;
      else
        work->ping = 0;
      if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
        {
          info->reset_work = true;
          send_jobs = 0;
          jobs_to_send = 2;
          info->work_block_local = work_block;
          info->work_pool_update = work_pool_update;
        }
      if (info->write_pos >= HEXMINERBE200_ARRAY_SIZE_REAL)
        info->write_pos = 0;
      work->subid = info->write_pos;
      hexminerbe200_create_task (info->reset_work, info->ht, work);
      free_work (info->hexworks[info->write_pos]);
      info->hexworks[info->write_pos] = work;
      ret = hexminerbe200_send_task (info->ht, hexminerbe200);
      info->write_pos++;
      send_jobs++;
      if (ret == HEXMINERBE200_TASK_SIZE && info->reset_work)
        {
          info->reset_work = false;
          gettimeofday (&info->last_wr, NULL);
        }
    }
}

extern void inc_hw_errors_hex8 (struct thr_info *thr, int diff);

static int64_t
hexminerbe200_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexminerbe200 = thr->cgpu;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  struct timeval now;
  struct timeval diff;
  int64_t tdif, rethash_count = 0;
  int ret_r, rminder = 0;
  double found, hash_count = 0;
  uint32_t nonce;
  uint8_t buf_thr = 61, buf_thrl = 57;
  do_reset (hexminerbe200);
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    {
      info->reset_work = true;
      info->work_block_local = work_block;
      info->work_pool_update = work_pool_update;
      info->jobs_to_send = 2;
      gettimeofday (&info->last_wr, NULL);
      do_write_hexm (thr);
      goto done_wr;
    }
  gettimeofday (&now, NULL);
  tdif = timediff (&now, &info->last_wr);
  info->jobs_to_send = (int) (tdif / info->wsem_ustiming);
  rminder = (int) (tdif % info->wsem_ustiming);
  if (info->jobs_to_send > 0 || info->wr->buf_empty_space > buf_thr)
    {
      gettimeofday (&info->last_wr, NULL);
      if (info->wr->buf_empty_space < buf_thrl)
        goto done_wr;
      now.tv_sec = 0;
      now.tv_usec = rminder;
      timersub (&info->last_wr, &now, &diff);
      memcpy (&info->last_wr, &diff, sizeof (struct timeval));
      if (info->wr->buf_empty_space > buf_thr)
        info->jobs_to_send++;
      if (info->jobs_to_send > 2)
        info->jobs_to_send = 2;
      do_write_hexm (thr);
    }
done_wr:
  if (libhex_be200_usb_dead (hexminerbe200))
    {
      hexminerbe200->shutdown = true;
      return -1;
    }
  if (info->hash_write_pos + HEX_BE200USB_R_SIZE > HEX_BE200HASH_BUF_SIZE_OK)
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
        libhex_be200_eatHashData (info->wr, info->readbuf,
                                  &info->hash_read_pos,
                                  &info->hash_write_pos);
      if (ret_r > HEX_BE200BUF_DATA)
        goto out;
      if (info->wr->datalength == 1)
        goto done;
      if (info->wr->lastnonceid > HEXMINERBE200_ARRAY_SIZE_REAL)
        info->wr->lastnonceid = 0;
      if (info->wr->lastchippos > HEXBE200_DEFAULT_RANGE)
        info->wr->lastchippos = HEXBE200_DEFAULT_RANGE;
      if (libhex_be200_cachenonce
          (&info->array_nonce_cache[info->wr->lastchippos],
           info->wr->lastnonce))
        {                       
        	/*LTE TBD!!! */
          nonce = htole32 (info->wr->lastnonce) + 1;
          found =
            be200_nonce_roll (hexminerbe200, thr, nonce,
                              info->wr->lastnonceid, info->wr->roll,
                              HEX_BE200NROLL);
          info->last_chip_valid_work[(uint8_t) info->wr->lastchippos] =
            time (NULL);
          if (found > 0)
            {
              info->hw_err[info->wr->lastchippos] = 0;
              if (hash_count == 0)
                libhex_be200_getvoltage (htole16 (info->wr->lastvoltage),
                                         &info->pic_voltage_readings);
              hash_count += found;
              info->matching_work[info->wr->lastchippos]++;
            }
          else
            {
              info->hw_err[info->wr->lastchippos]++;
              switch (info->diff)
                {
                case 1:
                  inc_hw_errors_hex8 (thr, -64);
                  break;
                case 0:
                  inc_hw_errors_hex8 (thr, -1);
                  break;
                case 2:
                  inc_hw_errors_hex8 (thr, -4096);
                  break;
                case 3:
                  inc_hw_errors_hex8 (thr, -262144);
                  break;
                default:
                  inc_hw_errors_hex8 (thr, -64);
                  break;
                }
            }
        }
      else
        info->dupe[info->wr->lastchippos]++;
    out:
      if (ret_r == HEX_BE200BUF_ERR)
        info->usb_r_errors++;
    done:
      if (ret_r != HEX_BE200BUF_SKIP)
        goto again;
    }
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
  ret_r =
    libhex_be200_readHashData (hexminerbe200, info->readbuf,
                               &info->hash_write_pos,
                               HEXMINERBE200_BULK_READ_TIMEOUT);
  if (ret_r != LIBUSB_SUCCESS)
    info->usb_bad_reads++;
  else
    info->usb_bad_reads = 0;
  if (info->usb_bad_reads > 20)
    libhex_be200_reset (hexminerbe200);
  new_block:
  if (libhex_be200_usb_dead (hexminerbe200))
    {
      hexminerbe200->shutdown = true;
      return -1;
    }
  switch (info->diff)
    {
    case 1:
      rethash_count = (0xffffffffull * (int64_t) hash_count * 64);
      break;
    case 0:
      rethash_count = (0xffffffffull * (int64_t) hash_count);
      break;
    case 2:
      rethash_count = (0xffffffffull * (int64_t) hash_count * 4096);
      break;
    case 3:
      rethash_count = (0xffffffffull * (int64_t) hash_count * 262144);
      break;
    default:
      rethash_count = (0xffffffffull * (int64_t) hash_count * 64);
      break;
    }
  return rethash_count;
}

static void
get_hexminerbe200_statline_before (char *buf, size_t bufsiz,
                                   struct cgpu_info *hexminerbe200)
{
  if (!hexminerbe200->device_data)
    return;
  struct hexminerbe200_info *info = hexminerbe200->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

extern void suffix_string (uint64_t val, char *buf, size_t bufsiz,
                           int sigdigits);
static struct api_data *
hexminerbe200_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct timeval now;
  struct hexminerbe200_info *info = cgpu->device_data;
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
      sprintf (mcw, "Chip%d Nonces", i );
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i );
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
      sprintf (mcw, "Chip%d Timouts Reset", i );
      root = api_add_int (root, mcw, &(info->res_timeout_err[i]), true);
      sprintf (mcw, "Chip%d HW Errors Reset", i );
      root = api_add_int (root, mcw, &(info->res_hew_err[i]), true);
    }
#ifdef DBG_HW_HEXBE200
#ifdef DBG_HW_HEXBE200_PRINT
  int vv = 0;
  while (vv < HEX_BE200NROLL_ARR)
    {
      char mcww[24];
      if (info->nincdec[vv] > 0)
        {
          sprintf (mcww, "nincdec[%d] ", vv);
          root = api_add_int (root, mcww, &(info->nincdec[vv]), true);
        }
      vv++;
    }
#endif
#endif
  return root;
}

struct device_drv hexminerbe200_drv = {
  .drv_id = DRIVER_hexminerbe200,
  .dname = "hexminerbe200",
  .name = "HEXE",
  .drv_detect = hexminerbe200_detect,
  .thread_init = hexminerbe200_thread_init,
  .hash_work = hash_driver_work,
  .scanwork = hexminerbe200_scanhash,
  .get_api_stats = hexminerbe200_api_stats,
  .get_statline_before = get_hexminerbe200_statline_before,
  .thread_shutdown = hexminerbe200_shutdown,
};
