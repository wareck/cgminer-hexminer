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
#include "driver-hexminerm.h"
#include <math.h>
#include "util.h"
extern unsigned int work_block;
extern unsigned int work_pool_update;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
struct device_drv hexminerm_drv;
extern bool no_work;
int opt_hexminerm_chip_mask = 0xFF;
int opt_hexminerm_hw_err_res = 0xFF;
int opt_hexminerm_nonce_timeout_secs = 0xFF;
int opt_hexminerm_core_voltage = HEXM_DEFAULT_CORE_VOLTAGE;
int opt_hexminerm_pic_roll = 0x3c;
int opt_hexminerm_reset_below_threshold = 94;
int opt_hexminerm_reset_below_threshold_wait = 190;

#include "libhexm.c"

static int
hexminerm_send_task (struct hexminerm_task *ht, struct cgpu_info *hexminerm)
{
  int ret = 0;
  size_t nr_len = HEXMINERM_TASK_SIZE;
  struct hexminerm_info *info;
  info = hexminerm->device_data;
  libhexm_csum (&ht->startbyte, &ht->csum, &ht->csum);
  ret = libhexm_sendHashData (hexminerm, &ht->startbyte, nr_len);
  if (ret != nr_len && info->usb_bad_reads > -1)
    {
      libhexm_reset (hexminerm);
      info->usb_w_errors++;
      return -1;
    }
  return ret;
}

static inline void
hexminerm_create_task (bool reset_work, struct hexminerm_task *ht,
                       struct work *work)
{
  if (reset_work)
    ht->status = (uint8_t) HEXM_STAT_NEW_WORK_CLEAR_OLD;
  else
    ht->status = (uint8_t) HEXM_STAT_NEW_WORK;
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
}

static inline void
hexminerm_init_task_c (struct hexminerm_config_task *htc,
                       struct hexminerm_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminerm_config_task) - 6) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) (info->frequency / 4));
  libhexm_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->chip_mask = (uint8_t) info->chip_mask;
  libhexm_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminerm_init_task (struct hexminerm_task *ht, struct hexminerm_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERM_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);
  ht->status = (uint8_t) HEXM_STAT_NEW_WORK_CLEAR_OLD;
  ht->roll = (uint8_t) info->pic_roll;
}

static void
do_reset (struct cgpu_info *hexminerm)
{
  struct hexminerm_info *info = hexminerm->device_data;
  time_t now = time (NULL);
  uint8_t i = 0;
  if (no_work)
    {
      while (i < info->asic_count)
        {
          info->hw_err[i] = 0;
/*Pools and all to settle 10 secs add */
          if (info->last_chip_valid_work[i] > 0)
            info->last_chip_valid_work[i] = now + 10;
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
              info->opt_hexminerm_nonce_timeout_secs))
        {
          libhexm_set_words (hexminerm, HEXM_RES_ADR, (uint16_t) i, 0x0007);
          applog (LOG_ERR,
                  "HEXM %i Chip[%i] last received nonce %i secs ago reset",
                  hexminerm->device_id, i,
                  (int) (now - info->last_chip_valid_work[i]));
          info->last_chip_valid_work[i] = now;
          info->hw_err[i] = 0;
          info->res_timeout_err[i]++;
          info->b_reset_count++;
          break;
        }
      if (info->hw_err[i] > info->opt_hexminerm_hw_err_res)
        {
          libhexm_set_words (hexminerm, HEXM_RES_ADR, (uint16_t) i, 0x0007);
          applog (LOG_ERR, "HEXM %i Chip[%i] %i consecutive HW errors reset",
                  hexminerm->device_id, i, (int) info->hw_err[i]);
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
hexminerm_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int asic_count, frequency;
  struct hexminerm_info *info;
  struct cgpu_info *hexminerm;
  bool configured = false;
  int i = 0;
  hexminerm = usb_alloc_cgpu (&hexminerm_drv, HEXM_MINER_THREADS);
  if (!usb_init (hexminerm, dev, found))
    {
      usb_uninit (hexminerm);
      return NULL;
    }
  hexminerm->device_data = calloc (sizeof (struct hexminerm_info), 1);
  if (unlikely (!(hexminerm->device_data)))
    {
      hexminerm->device_data = NULL;
      usb_uninit (hexminerm);
      return NULL;
    }
  if (opt_hexminerm_options != NULL)
  	configured = (sscanf(opt_hexminerm_options, "%d:%d", &asic_count, &frequency) == 2);	
  
  if (opt_hexminerm_core_voltage < HEXM_MIN_COREMV
      || opt_hexminerm_core_voltage > HEXM_MAX_COREMV)
    {
      applog (LOG_ERR, "Invalid hexminerm-voltage %d must be %dmV - %dmV",
              opt_hexminerm_core_voltage, HEXM_MIN_COREMV, HEXM_MAX_COREMV);
      free (hexminerm->device_data);
      hexminerm->device_data = NULL;
      usb_uninit (hexminerm);
      return NULL;
    }
  info = hexminerm->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERM_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerm->device_data);
      hexminerm->device_data = NULL;
      usb_uninit (hexminerm);
      return NULL;
    }
  info->wr = (struct workm_result *) malloc (sizeof (struct workm_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsm));
  info->readbuf = calloc (HEXM_HASH_BUF_SIZE, sizeof (unsigned char));
  info->write_pos = 0;
  info->usb_bad_reads = -30;
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEXM_DEFAULT_MINER_NUM;
  info->asic_count = HEXM_DEFAULT_ASIC_NUM;
  info->frequency = HEXM_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEXM_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerm_core_voltage;
  info->opt_hexminerm_hw_err_res = opt_hexminerm_hw_err_res;
  info->opt_hexminerm_nonce_timeout_secs = opt_hexminerm_nonce_timeout_secs;
  info->chip_mask = opt_hexminerm_chip_mask;
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
  info->pic_roll = opt_hexminerm_pic_roll;
  info->opt_hexminerm_reset_below_threshold = (double)opt_hexminerm_reset_below_threshold / 100;
  info->opt_hexminerm_reset_below_threshold_wait = (double)opt_hexminerm_reset_below_threshold_wait;
  gettimeofday (&info->last_wr, NULL);
  info->rolling1 = (double) (info->asic_count * info->frequency * 99  * info->opt_hexminerm_reset_below_threshold);
  
  info->wsem_ustiming =
    (int64_t) (0x100000000ll /
               (info->asic_count * info->frequency * 99 *
                (info->pic_roll + 1) * 0.75));
  info->ht = calloc (sizeof (struct hexminerm_task), 1);
  hexminerm_init_task (info->ht, info);
  while (i < HEXMINERM_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
  } if (!add_cgpu (hexminerm))
    {
      free (info->hexworks);
      free (hexminerm->device_data);
      hexminerm->device_data = NULL;
      hexminerm = usb_free_cgpu (hexminerm);
      usb_uninit (hexminerm);
      return NULL;
    }
  return hexminerm;
}

static void
hexminerm_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerm_drv, hexminerm_detect_one);
}

static void
do_hexminerm_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerm = thr->cgpu;
  struct hexminerm_info *info = hexminerm->device_data;
  int i = 0;
  while (i < HEXMINERM_ARRAY_SIZE)
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
hexminerm_shutdown (struct thr_info *thr)
{
	struct cgpu_info *hexminerm = thr->cgpu;
  do_hexminerm_close (thr);
  /*Power off and try to flush USB */
  libhexm_shutdown (hexminerm, 0x30AE, 0x0004);
  usb_nodev (hexminerm);
}

static bool
hexminerm_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerm = thr->cgpu;
  struct hexminerm_info *info = hexminerm->device_data;
  uint8_t i = 0;
  info->thr = thr;
  struct hexminerm_config_task *htc;
  htc = calloc (sizeof (struct hexminerm_config_task), 1);
  hexminerm_init_task_c (htc, info);
  cgsleep_ms (200);
  int ret = libhexm_sendHashData (hexminerm, &htc->startbyte,
                                  sizeof (struct hexminerm_config_task));
  if (ret != sizeof (struct hexminerm_config_task))
  	{
    	applog (LOG_ERR, "HEXM %i Send config failed disabling!!", hexminerm->device_id);
   		libhexm_reset(hexminerm);
   		info->shut_write = true;
    }
  free (htc);
/*Pools and all to settle 10 secs add */
  time_t now = time (NULL) + 10;
  while (i < info->asic_count)
    {
      info->chips_enabled[i] =
        (((uint8_t) info->chip_mask & (uint8_t) pow (2, i)) ==
         (uint8_t) pow (2, i));
      info->last_chip_valid_work[i] = now;
      i++;
    }
  return true;
}

static void
do_write_hexm (struct thr_info *thr)
{
  struct cgpu_info *hexminerm = thr->cgpu;
  struct hexminerm_info *info = hexminerm->device_data;
  struct work *work;
  int jobs_to_send = info->jobs_to_send;
  int send_jobs, ret;
  send_jobs = 0;
  while (!libhexm_usb_dead (hexminerm) && (send_jobs < jobs_to_send))
    {
      work = get_work (thr, thr->id);
      work->ping = 1;
      if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
        {
          info->reset_work = true;
          send_jobs = 0;
          jobs_to_send = 2;
          info->work_pool_update = work_pool_update;
          info->work_block_local = work_block;
        }
      if (info->write_pos >= HEXMINERM_ARRAY_SIZE_REAL)
        info->write_pos = 0;
      work->subid = info->write_pos;
      hexminerm_create_task (info->reset_work, info->ht, work);
      free_work (info->hexworks[info->write_pos]);
      info->hexworks[info->write_pos] = work;
      ret = hexminerm_send_task (info->ht, hexminerm);
      info->write_pos++;
      send_jobs++;
      if (ret == HEXMINERM_TASK_SIZE && info->reset_work)
        {
          info->reset_work = false;
          gettimeofday (&info->last_wr, NULL);
        }
    }
}

static int64_t
hexminerm_scanhash (struct thr_info *thr)
{
	
  struct cgpu_info *hexminerm = thr->cgpu;
  struct hexminerm_info *info = hexminerm->device_data;
  struct timeval now;
  struct timeval diff;
	double dev_runtime;
  int64_t tdif, rethash_count = 0;
  int ret_r, rminder = 0;
  double found, hash_count = 0;
  uint32_t nonce;
  uint8_t buf_thr = 61, buf_thrl = 57;
  do_reset (hexminerm);
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
  if (libhexm_usb_dead (hexminerm))
    {
      hexminerm->shutdown = true;
      return -1;
    }
  if (info->hash_write_pos + HEXM_USB_R_SIZE > HEXM_HASH_BUF_SIZE_OK)
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
        libhexm_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEXM_BUF_DATA)
        goto out;
      if (info->wr->datalength == 1)
        goto done;
      if (info->wr->lastnonceid > HEXMINERM_ARRAY_SIZE_REAL)
        info->wr->lastnonceid = 0;
      if (info->wr->lastchippos > HEXM_DEFAULT_RANGE)
        info->wr->lastchippos = HEXM_DEFAULT_RANGE;
      if (libhexm_cachenonce
          (&info->array_nonce_cache[info->wr->lastchippos],
           info->wr->lastnonce))
        {
          nonce = htole32 (info->wr->lastnonce);
          found =
            hexminerm_predecode_nonce_roll (hexminerm, thr, nonce,
                                            info->wr->lastnonceid,
                                            info->wr->roll);
          info->last_chip_valid_work[(uint8_t) info->wr->lastchippos] =
            time (NULL);
          if (found > 0)
            {
              info->hw_err[info->wr->lastchippos] = 0;
              if (hash_count == 0)
                libhexm_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);
              hash_count += found;
              info->matching_work[info->wr->lastchippos]++;
            }
          else
            {
              info->hw_err[info->wr->lastchippos]++;
              inc_hw_errors (thr);
            }
        }
      else
        info->dupe[info->wr->lastchippos]++;
    out:
      if (ret_r == HEXM_BUF_ERR)
        info->usb_r_errors++;
    done:
      if (ret_r != HEXM_BUF_SKIP)
        goto again;
    }
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
  ret_r =
    libhexm_readHashData (hexminerm, info->readbuf, &info->hash_write_pos,
                          HEXMINERM_BULK_READ_TIMEOUT);
  if (ret_r != LIBUSB_SUCCESS)
    info->usb_bad_reads++;
  else
    info->usb_bad_reads = 0;
  if (info->usb_bad_reads > 20)
    libhexm_reset (hexminerm);
  new_block:
  if (libhexm_usb_dead (hexminerm))
    {
      hexminerm->shutdown = true;
      return -1;
    }
	if (hexminerm->dev_start_tv.tv_sec == 0)
    	dev_runtime = total_secs;  
  else
    {
      cgtime (&now);
      dev_runtime = tdiff (&now, &(hexminerm->dev_start_tv));
    }
  if ((dev_runtime > info->opt_hexminerm_reset_below_threshold_wait) && (hexminerm->rolling1 < info->rolling1)) {
  		libhexm_shutdown (hexminerm, 0x30AE, 0x0004);
  		info->shut_write = true;  
  		usb_nodev(hexminerm);
    	hexminerm->shutdown = true;	
  		applog (LOG_ERR,
                  "HEXM %i restarting due to low hash rate (%f < %f). Uptime %f secs",
                  hexminerm->device_id, hexminerm->rolling1 , info->rolling1, dev_runtime);
      return -1;
   }
  rethash_count = (0xffffffffull * (int64_t) hash_count);
  return rethash_count;
}

static void
get_hexminerm_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminerm)
{
  if (!hexminerm->device_data)
    return;
  struct hexminerm_info *info = hexminerm->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

extern void suffix_string (uint64_t val, char *buf, size_t bufsiz,
                           int sigdigits);
static struct api_data *
hexminerm_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct timeval now;
  struct hexminerm_info *info = cgpu->device_data;
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
  return root;
}

struct device_drv hexminerm_drv = {
  .drv_id = DRIVER_hexminerm,
  .dname = "hexminerm",
  .name = "HEXM",
  .drv_detect = hexminerm_detect,
  .thread_init = hexminerm_thread_init,
  .hash_work = hash_driver_work,
  .scanwork = hexminerm_scanhash,
  .get_api_stats = hexminerm_api_stats,
  .get_statline_before = get_hexminerm_statline_before,
  .thread_shutdown = hexminerm_shutdown,
};
