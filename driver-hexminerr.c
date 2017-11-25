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
#include "driver-hexminerr.h"
#include <math.h>
#include "util.h"
extern unsigned int work_block;
extern unsigned int work_pool_update;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
struct device_drv hexminerr_drv;
extern bool no_work;
int opt_hexminerr_chip_mask = 0xFF;
//int opt_hexminerr_hw_err_res = 0xFF;
//int opt_hexminerr_nonce_timeout_secs = 0xFF;
int opt_hexminerr_core_voltage = HEXR_DEFAULT_CORE_VOLTAGE;
int opt_hexminerr_pic_roll = 0x3c;
//int opt_hexminerr_reset_below_threshold = 94;
//int opt_hexminerr_reset_below_threshold_wait = 190;
int opt_hexminerr_leading_zeros = 1;


#include "libhexr.c"

static int
hexminerr_send_task (struct hexminerr_task *ht, struct cgpu_info *hexminerr)
{
  int ret = 0;
  size_t nr_len = HEXMINERR_TASK_SIZE;
  struct hexminerr_info *info;
  info = hexminerr->device_data;
  libhexr_csum (&ht->startbyte, &ht->csum, &ht->csum);
  ret = libhexr_sendHashData (hexminerr, &ht->startbyte, nr_len);
  if (ret != nr_len && info->usb_bad_reads > -1)
    {
      libhexr_reset (hexminerr);
      info->usb_w_errors++;
      return -1;
    }
  return ret;
}

static inline void
hexminerr_create_task (bool reset_work, struct hexminerr_task *ht,
                       struct work *work)
{
  if (reset_work)
    ht->status = (uint8_t) HEXR_STAT_NEW_WORK_CLEAR_OLD;
  else
    ht->status = (uint8_t) HEXR_STAT_NEW_WORK;
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
}

static inline void
hexminerr_init_task_c (struct hexminerr_config_task *htc,
                       struct hexminerr_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminerr_config_task) - 6) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) (info->frequency));
  libhexr_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->chip_mask = (uint8_t) info->chip_mask;
  libhexr_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminerr_init_task (struct hexminerr_task *ht, struct hexminerr_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERR_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);
  ht->status = (uint8_t) HEXR_STAT_NEW_WORK_CLEAR_OLD;
  ht->roll = (uint8_t) info->pic_roll;
  ht->leading_zeros = (uint8_t)(info->opt_hexminerr_leading_zeros + 30);
}
/*
static void
do_reset (struct cgpu_info *hexminerr)
{
  struct hexminerr_info *info = hexminerr->device_data;
  time_t now = time (NULL);
  uint8_t i = 0;
  if (no_work)
    {
      while (i < info->asic_count)
        {
          info->hw_err[i] = 0;
//Pools and all to settle 10 secs add
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
              info->opt_hexminerr_nonce_timeout_secs))
        {
          libhexr_set_words (hexminerr, HEXR_RES_ADR, (uint16_t) i, 0x0007);
          applog (LOG_ERR,
                  "HEXR %i Chip[%i] last received nonce %i secs ago reset",
                  hexminerr->device_id, i,
                  (int) (now - info->last_chip_valid_work[i]));
          info->last_chip_valid_work[i] = now;
          info->hw_err[i] = 0;
          info->res_timeout_err[i]++;
          info->b_reset_count++;
          break;
        }
      if (info->hw_err[i] > info->opt_hexminerr_hw_err_res)
        {
          libhexr_set_words (hexminerr, HEXR_RES_ADR, (uint16_t) i, 0x0007);
          applog (LOG_ERR, "HEXR %i Chip[%i] %i consecutive HW errors reset",
                  hexminerr->device_id, i, (int) info->hw_err[i]);
          info->last_chip_valid_work[i] = now;
          info->hw_err[i] = 0;
          info->res_hew_err[i]++;
          info->b_reset_count++;
          break;
        }
      i++;
    }
}
*/
static struct cgpu_info *
hexminerr_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int asic_count, frequency;
  struct hexminerr_info *info;
  struct cgpu_info *hexminerr;
  bool configured = false;
  int i = 0;
  hexminerr = usb_alloc_cgpu (&hexminerr_drv, HEXR_MINER_THREADS);
  if (!usb_init (hexminerr, dev, found))
    {
      usb_uninit (hexminerr);
      return NULL;
    }
  hexminerr->device_data = calloc (sizeof (struct hexminerr_info), 1);
  if (unlikely (!(hexminerr->device_data)))
    {
      hexminerr->device_data = NULL;
      usb_uninit (hexminerr);
      return NULL;
    }
  if (opt_hexminerr_options != NULL)
  	configured = (sscanf(opt_hexminerr_options, "%d:%d", &asic_count, &frequency) == 2);	
  
  if (opt_hexminerr_core_voltage < HEXR_MIN_COREMV
      || opt_hexminerr_core_voltage > HEXR_MAX_COREMV)
    {
      applog (LOG_ERR, "Invalid hexminerr-voltage %d must be %dmV - %dmV",
              opt_hexminerr_core_voltage, HEXR_MIN_COREMV, HEXR_MAX_COREMV);
      free (hexminerr->device_data);
      hexminerr->device_data = NULL;
      usb_uninit (hexminerr);
      return NULL;
    }
  info = hexminerr->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERR_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerr->device_data);
      hexminerr->device_data = NULL;
      usb_uninit (hexminerr);
      return NULL;
    }
  info->wr = (struct workr_result *) malloc (sizeof (struct workr_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsr));
  info->readbuf = calloc (HEXR_HASH_BUF_SIZE, sizeof (unsigned char));
  info->write_pos = 0;
  info->usb_bad_reads = -30;
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEXR_DEFAULT_MINER_NUM;
  info->asic_count = HEXR_DEFAULT_ASIC_NUM;
  info->frequency = HEXR_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEXR_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerr_core_voltage;
  //info->opt_hexminerr_hw_err_res = opt_hexminerr_hw_err_res;
  //info->opt_hexminerr_nonce_timeout_secs = opt_hexminerr_nonce_timeout_secs;
  info->chip_mask = opt_hexminerr_chip_mask;
  info->wr->buf_empty_space = 4;
  info->work_block_local = -1;
	info->work_pool_update = -1;
  info->reset_work = true;
  info->jobs_to_send = 1;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  info->pic_roll = opt_hexminerr_pic_roll;
  //info->pic_roll = 31;
  //info->opt_hexminerr_reset_below_threshold = (double)opt_hexminerr_reset_below_threshold / 100;
  //info->opt_hexminerr_reset_below_threshold_wait = (double)opt_hexminerr_reset_below_threshold_wait;
  info->opt_hexminerr_leading_zeros  = opt_hexminerr_leading_zeros;
  info->hexminerr_work_count = pow( 2, (opt_hexminerr_leading_zeros - 1) );
  info->work_ping = (opt_hexminerr_leading_zeros == 1);
  gettimeofday (&info->last_wr, NULL);
  //info->rolling1 = (double) (info->asic_count * info->frequency * 193  * info->opt_hexminerr_reset_below_threshold);
  info->wsem_ustiming =
    (int64_t) (0x100000000ll /
               (info->asic_count * info->frequency * 193 *
                (info->pic_roll + 1) * 0.7));
  info->ht = calloc (sizeof (struct hexminerr_task), 1);
  hexminerr_init_task (info->ht, info);
  while (i < HEXMINERR_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
  } if (!add_cgpu (hexminerr))
    {
      free (info->hexworks);
      free (hexminerr->device_data);
      hexminerr->device_data = NULL;
      hexminerr = usb_free_cgpu (hexminerr);
      usb_uninit (hexminerr);
      return NULL;
    }
  return hexminerr;
}

static void
hexminerr_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerr_drv, hexminerr_detect_one);
}

static void
do_hexminerr_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerr = thr->cgpu;
  struct hexminerr_info *info = hexminerr->device_data;
  int i = 0;
  while (i < HEXMINERR_ARRAY_SIZE)
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
hexminerr_shutdown (struct thr_info *thr)
{
	struct cgpu_info *hexminerr = thr->cgpu;
  do_hexminerr_close (thr);
  /*Power off and try to flush USB */
  libhexr_shutdown (hexminerr, 0x30AE, 0x0004);
  usb_nodev (hexminerr);
}

static bool
hexminerr_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerr = thr->cgpu;
  struct hexminerr_info *info = hexminerr->device_data;
  uint8_t i = 0;
  info->thr = thr;
  struct hexminerr_config_task *htc;
  htc = calloc (sizeof (struct hexminerr_config_task), 1);
  hexminerr_init_task_c (htc, info);
  
  int ret = libhexr_sendHashData (hexminerr, &htc->startbyte,
                                  sizeof (struct hexminerr_config_task));
  cgsleep_ms (200);
  if (ret != sizeof (struct hexminerr_config_task))
  	{
    	applog (LOG_ERR, "HEXR %i Send config failed disabling!!", hexminerr->device_id);
   		libhexr_reset(hexminerr);
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
do_write_hexr (struct thr_info *thr)
{
  struct cgpu_info *hexminerr = thr->cgpu;
  struct hexminerr_info *info = hexminerr->device_data;
  struct work *work;
  int jobs_to_send = info->jobs_to_send;
  int send_jobs, ret;
  send_jobs = 0;
  while (!libhexr_usb_dead (hexminerr) && (send_jobs < jobs_to_send))
    {
      work = get_work (thr, thr->id);
      work->ping = info->work_ping;
     
      if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
        {
          
					info->reset_work = true;
          send_jobs = 0;
          jobs_to_send = 2;
          info->work_pool_update = work_pool_update;
          info->work_block_local = work_block;
        }
      
      if (info->write_pos >= HEXMINERR_ARRAY_SIZE_REAL)
        info->write_pos = 0;
      work->subid = info->write_pos;
      hexminerr_create_task (info->reset_work, info->ht, work);
      free_work (info->hexworks[info->write_pos]);
      info->hexworks[info->write_pos] = work;
      ret = hexminerr_send_task (info->ht, hexminerr);
      info->write_pos++;
      send_jobs++;
      if (ret == HEXMINERR_TASK_SIZE && info->reset_work)
        {
          info->reset_work = false;
          gettimeofday (&info->last_wr, NULL);
        }
    }
}

extern void inc_hw_errors_hex8 (struct thr_info *thr, int diff);

static int64_t
hexminerr_scanhash (struct thr_info *thr)
{
	
  struct cgpu_info *hexminerr = thr->cgpu;
  struct hexminerr_info *info = hexminerr->device_data;
  struct timeval now;
  struct timeval diff;
	//double dev_runtime;
  int64_t tdif, rethash_count = 0;
  int ret_r, rminder = 0;
  double found, hash_count = 0;
  uint32_t nonce;
  //uint8_t buf_thr = 2, buf_thrl = 4;
  uint8_t buf_thr = 60, buf_thrl = 57;
  //do_reset (hexminerr);
 /* 
if (info->wr->buf_empty_space >= HEXMINERR_ARRAY_SIZE_REAL) {
        applog (LOG_ERR,
                  "HEXR %i Bugec info->wr->buf_empty_space %i",
                  hexminerr->device_id, info->wr->buf_empty_space);
}

if (info->wr->buf_empty_space < 50) {
        applog (LOG_ERR,
                  "HEXR %i Bugec info->wr->buf_empty_space %i",
                  hexminerr->device_id, info->wr->buf_empty_space);
}
*/
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    {
      info->reset_work = true;
      info->work_block_local = work_block;
      info->work_pool_update = work_pool_update;
      info->jobs_to_send = 2;
      gettimeofday (&info->last_wr, NULL);
      do_write_hexr (thr);
      //cgsleep_ms (10);
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
      do_write_hexr (thr);
    }
done_wr:
  if (libhexr_usb_dead (hexminerr))
    {
      hexminerr->shutdown = true;
      return -1;
    }
  if (info->hash_write_pos + HEXR_USB_R_SIZE > HEXR_HASH_BUF_SIZE_OK)
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
        libhexr_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEXR_BUF_DATA)
        goto out;
      if (info->wr->datalength == 1)
        goto done;
      if (info->wr->lastnonceid >= HEXMINERR_ARRAY_SIZE_REAL) {
       // applog (LOG_ERR,
         //         "HEXR %i Bug info->wr->lastnonceid %i",
           //       hexminerr->device_id, info->wr->lastnonceid);
        info->wr->lastnonceid = 0;
         
       }
      if (info->wr->lastchippos > HEXR_DEFAULT_RANGE)
        info->wr->lastchippos = HEXR_DEFAULT_RANGE;
      if (libhexr_cachenonce
          (&info->array_nonce_cache[info->wr->lastchippos],
           info->wr->lastnonce))
        {
          nonce = htole32 (info->wr->lastnonce);
          found =
            hexminerr_predecode_nonce_roll (hexminerr, thr, nonce,
                                            info->wr->lastnonceid,
                                            info->wr->roll);
          info->last_chip_valid_work[(uint8_t) info->wr->lastchippos] =
            time (NULL);
          if (found > 0)
            {
              info->hw_err[info->wr->lastchippos] = 0;
              if (hash_count == 0)
                libhexr_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);
              hash_count += found;
              info->matching_work[info->wr->lastchippos]++;
            }
          else
            {
              info->hw_err[info->wr->lastchippos]++;
              inc_hw_errors_hex8 (thr, -info->hexminerr_work_count);
            }
        }
      else
        info->dupe[info->wr->lastchippos]++;
    out:
      if (ret_r == HEXR_BUF_ERR)
        info->usb_r_errors++;
    done:
      if (ret_r != HEXR_BUF_SKIP)
        goto again;
    }
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
  ret_r =
    libhexr_readHashData (hexminerr, info->readbuf, &info->hash_write_pos,
                          HEXMINERR_BULK_READ_TIMEOUT);
  if (ret_r != LIBUSB_SUCCESS)
    info->usb_bad_reads++;
  else
    info->usb_bad_reads = 0;
  if (info->usb_bad_reads > 20)
    libhexr_reset (hexminerr);
  new_block:
  if (libhexr_usb_dead (hexminerr))
    {
      hexminerr->shutdown = true;
      return -1;
    }
 /*   
	if (hexminerr->dev_start_tv.tv_sec == 0)
    	dev_runtime = total_secs;  
  else
    {
      cgtime (&now);
      dev_runtime = tdiff (&now, &(hexminerr->dev_start_tv));
    }
  if ((dev_runtime > info->opt_hexminerr_reset_below_threshold_wait) && (hexminerr->rolling1 < info->rolling1)) {
  		libhexr_shutdown (hexminerr, 0x30AE, 0x0004);
  		info->shut_write = true;  
  		usb_nodev(hexminerr);
    	hexminerr->shutdown = true;	
  		applog (LOG_ERR,
                  "HEXR %i restarting due to low hash rate (%f < %f). Uptime %f secs",
                  hexminerr->device_id, hexminerr->rolling1 , info->rolling1, dev_runtime);
      return -1;
   }
  */
  rethash_count = (0xffffffffull * (int64_t) hash_count * (int64_t) info->hexminerr_work_count);
  return rethash_count;
}

static void
get_hexminerr_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminerr)
{
  if (!hexminerr->device_data)
    return;
  struct hexminerr_info *info = hexminerr->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

extern void suffix_string (uint64_t val, char *buf, size_t bufsiz,
                           int sigdigits);
static struct api_data *
hexminerr_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct timeval now;
  struct hexminerr_info *info = cgpu->device_data;
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

struct device_drv hexminerr_drv = {
  .drv_id = DRIVER_hexminerr,
  .dname = "hexminerr",
  .name = "HEXR",
  .drv_detect = hexminerr_detect,
  .thread_init = hexminerr_thread_init,
  .hash_work = hash_driver_work,
  .scanwork = hexminerr_scanhash,
  .get_api_stats = hexminerr_api_stats,
  .get_statline_before = get_hexminerr_statline_before,
  .thread_shutdown = hexminerr_shutdown,
};
