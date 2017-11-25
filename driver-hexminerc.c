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
#include "driver-hexminerc.h"
#include "util.h"
extern unsigned int work_block;
extern unsigned int work_pool_update;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
extern bool stale_work (struct work *work, bool share);
struct device_drv hexminerc_drv;
int opt_hexminerc_core_voltage = HEXC_DEFAULT_CORE_VOLTAGE;

#include "libhexc.c"
extern bool no_work;

static int
hexminerc_send_task (struct hexminerc_task *ht, struct cgpu_info *hexminerc)
{
  int ret = 0;
  size_t nr_len = HEXMINERC_TASK_SIZE;
  struct hexminerc_info *info;
  info = hexminerc->device_data;
  libhexc_csum (&ht->startbyte, &ht->csum, &ht->csum);
  ret = libhexc_sendHashData (hexminerc, &ht->startbyte, nr_len);
  if (ret != nr_len)
    {
      libhexc_reset (hexminerc);
      info->usb_w_errors++;
      return -1;
    }
  return ret;
}

static inline void
hexminerc_create_task (bool reset_work, struct hexminerc_task *ht,
                       struct work *work)
{
  if (reset_work)
    ht->status = HEXC_STAT_NEW_WORK_CLEAR_OLD;
  else
    ht->status = HEXC_STAT_NEW_WORK;
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
  libhexc_calc_hexminer (work, ht);
}

static inline void
hexminerc_init_task (struct hexminerc_task *ht, struct hexminerc_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERC_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (HEXC_WORKQUEUE_ADR);
  libhexc_generateclk (info->frequency, HEXC_DEFAULT_XCLKIN_CLOCK,
                       (uint32_t *) & ht->clockcfg[0]);
  libhexc_setvoltage (info->core_voltage, &ht->refvoltage);
  ht->chipcount = htole16 (info->asic_count);
  ht->hashclock = htole16 ((uint16_t) info->frequency);
  ht->startnonce = 0x00000000;
  ht->status = HEXC_STAT_NEW_WORK_CLEAR_OLD;
}

static bool
need_reset (struct cgpu_info *hexminerc)
{
  if (no_work)
    return false;
  struct hexminerc_info *info = hexminerc->device_data;
  time_t now = time (NULL);
  bool ret = false;
  int i = 0;
  int secs = 90;
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
            info->chip_is_dead[i] = true;
          break;
        }
      info->chip_con_resets[i] = 0;
      i++;
    }
  return ret;
}

static struct cgpu_info *
hexminerc_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int asic_count, frequency;
  struct hexminerc_info *info;
  struct cgpu_info *hexminerc;
  bool configured = false;
  int i = 0;
  hexminerc = usb_alloc_cgpu (&hexminerc_drv, HEXC_MINER_THREADS);
  if (!usb_init (hexminerc, dev, found))
    {
      usb_uninit (hexminerc);
      return NULL;
    }
  hexminerc->device_data = calloc (sizeof (struct hexminerc_info), 1);
  if (unlikely (!(hexminerc->device_data)))
    {
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
 	if (opt_hexminerc_options != NULL)
  	configured = (sscanf(opt_hexminerc_options, "%d:%d", &asic_count, &frequency) == 2);	
  if (opt_hexminerc_core_voltage < HEXC_MIN_COREMV
      || opt_hexminerc_core_voltage > HEXC_MAX_COREMV)
    {
      applog (LOG_ERR, "Invalid hexminerc-voltage %d must be %dmV - %dmV",
              opt_hexminerc_core_voltage, HEXC_MIN_COREMV, HEXC_MAX_COREMV);
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
  info = hexminerc->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERC_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
  info->wr = (struct workc_result *) malloc (sizeof (struct workc_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsc));
  info->readbuf = calloc (HEXC_HASH_BUF_SIZE, sizeof (unsigned char));
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->wr->status = HEXC_STAT_IDLE;
  info->miner_count = HEXC_DEFAULT_MINER_NUM;
  info->asic_count = HEXC_DEFAULT_ASIC_NUM;
  info->frequency = HEXC_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEXC_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerc_core_voltage;
  info->work_block_local = -1;
info->work_pool_update = -1;
  info->reset_work = true;
  info->jobs_to_send = 8;
  info->roll = 0;
  info->ht = calloc (sizeof (struct hexminerc_task), 1);
  info->work = NULL;
  info->write_pos = 0;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  gettimeofday (&info->last_wr, NULL);
  info->wr->lastnonceid = 0;
  info->wsem_ustiming = (int64_t) (0x100000000ll / (16 * info->frequency));
  while (i < HEXMINERC_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    } i = 0;
  info->power_checked = time (NULL);
  while (i < HEXC_DEFAULT_ASIC_NUM)
    info->chip_is_dead[i++] = false;
  libhexc_generatenrange_new ((unsigned char *) &info->nonces_range,
                              info->asic_count);
  if (!add_cgpu (hexminerc))
    {
      free (info->hexworks);
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      hexminerc = usb_free_cgpu (hexminerc);
      usb_uninit (hexminerc);
      return NULL;
    }
  return hexminerc;
}

static void
hexminerc_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerc_drv, hexminerc_detect_one);
}

static void
do_hexminerc_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  int i = 0;
  while (i < HEXMINERC_ARRAY_SIZE)
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
hexminerc_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  do_hexminerc_close (thr);
  usb_nodev (hexminerc);
}

static bool
hexminerc_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  info->thr = thr;
  cgsleep_ms (100);
  hexminerc_init_task (info->ht, info);
  hexminerc_send_task (info->ht, hexminerc);
  cgsleep_ms (300);
  return true;
}

static void
do_write_hexc (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  struct work *tmpwork = NULL;
  bool power;
  int send_jobs, ret;
  int jobs_to_send = info->jobs_to_send;
  send_jobs = 0;
  if (time (NULL) - info->power_checked > 30)
    {
      info->power_checked = time (NULL);
      power = need_reset (hexminerc);
      if (power)
        {
          info->b_reset_count++;
          libhexc_set_word (hexminerc, HEXC_WORKQUEUE_ADR + 80, 0x0004);
          cgsleep_ms (100);
          hexminerc_init_task (info->ht, info);
          hexminerc_send_task (info->ht, hexminerc);
          cgsleep_ms (300);
          gettimeofday (&info->last_wr, NULL);
          info->reset_work = true;
          jobs_to_send = 8;
        }
    }
  while (!libhexc_usb_dead (hexminerc) && (send_jobs < jobs_to_send))
    {
    again:
      if (!info->work)
        {
          info->roll = 0;
          info->work = get_work (thr, thr->id);
          info->work->ping = 1;
        }
      if (stale_work (info->work, false))
        {
          free_work (info->work);
          info->work = NULL;
          if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
            {
              info->reset_work = true;
              send_jobs = 0;
              jobs_to_send = 8;
              info->work_block_local = work_block;
              info->work_pool_update = work_pool_update;
            }
          goto again;
        }
      if (info->write_pos >= HEXMINERC_ARRAY_SIZE_REAL || info->reset_work)
        info->write_pos = 0;
      info->work->subid = info->write_pos;
      tmpwork = copy_work_noffset_fast_no_id (info->work, info->roll++);
      hexminerc_create_task (info->reset_work, info->ht, tmpwork);
      free_work (info->hexworks[info->write_pos]);
      info->hexworks[info->write_pos] = tmpwork;
      if (info->work->drv_rolllimit)
        info->work->drv_rolllimit--;
      else
        {
          free_work (info->work);
          info->work = NULL;
        }
      ret = hexminerc_send_task (info->ht, hexminerc);
      info->write_pos++;
      send_jobs++;
      if (ret == HEXMINERC_TASK_SIZE && info->reset_work)
        {
          info->reset_work = false;
          gettimeofday (&info->last_wr, NULL);
        }
    }
}

static int
free_buff_space (int cur, int last)
{
  int ret = cur - last;
  if (ret > 0)
    return ret;
  ret += 254;
  return ret;
}

static int64_t
hexminerc_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  uint32_t nonce;
  int notdupe, found, i, lastchippos = 0;
  int ret_r = 0;
  int64_t hash_count = 0;
  int64_t tdif;
  int rminder = 0;
  struct timeval now;
  struct timeval diff;
  int free_sp = free_buff_space (info->write_pos, info->wr->lastnonceid);
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    {
      info->reset_work = true;
      info->jobs_to_send = 8;
      info->work_block_local = work_block;
      info->work_pool_update = work_pool_update;
      if (info->work)
        {
          free_work (info->work);
          info->work = NULL;
        }
      gettimeofday (&info->last_wr, NULL);
      do_write_hexc (thr);
      goto done_wr;
    }
  gettimeofday (&now, NULL);
  tdif = timediff (&now, &info->last_wr);
  info->jobs_to_send = (int) (tdif / info->wsem_ustiming);
  rminder = (int) (tdif % info->wsem_ustiming);
  if (info->jobs_to_send > 0 || free_sp < 16)
    {
      gettimeofday (&info->last_wr, NULL);
      if (free_sp > 16)
        goto done_wr;
      now.tv_sec = 0;
      now.tv_usec = rminder;
      timersub (&info->last_wr, &now, &diff);
      memcpy (&info->last_wr, &diff, sizeof (struct timeval));
      if (free_sp < 16)
        info->jobs_to_send++;
      if (info->jobs_to_send > 8)
        info->jobs_to_send = 8;
      do_write_hexc (thr);
    }
done_wr:
  if (libhexc_usb_dead (hexminerc))
    {
      hexminerc->shutdown = true;
      return -1;
    }
  if (info->hash_write_pos + HEXC_USB_R_SIZE > HEXC_HASH_BUF_SIZE_OK)
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
        libhexc_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEXC_BUF_DATA)
        goto out;
      if (info->wr->lastnonceid > HEXMINERC_ARRAY_SIZE_REAL)
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
      notdupe =
        libhexc_cachenonce (&info->array_nonce_cache[lastchippos], nonce);
      if (lastchippos > 0)
        notdupe &= libhexc_cachenonce (&info->array_nonce_cache[0], nonce);
      if (notdupe)
        {
          found =
            hexminerc_predecode_nonce (hexminerc, thr, nonce,
                                       info->wr->lastnonceid);
          if (found > 0)
            {
              info->matching_work[lastchippos]++;
              info->last_chip_valid_work[(uint8_t) lastchippos] = time (NULL);
              if (hash_count == 0)
                libhexc_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);
              hash_count += found;
            }
          else
            inc_hw_errors (thr);
        }
      else
        info->dupe[lastchippos]++;
    out:
      if (ret_r == HEXC_BUF_ERR)
        info->usb_r_errors++;
      if (ret_r != HEXC_BUF_SKIP)
        goto again;
    }
  if (info->work_block_local != work_block || info->work_pool_update != work_pool_update)
    		goto new_block;
  ret_r =
    libhexc_readHashData (hexminerc, info->readbuf, &info->hash_write_pos,
                          HEXMINERC_BULK_READ_TIMEOUT);
  if (ret_r != LIBUSB_SUCCESS)
    info->usb_bad_reads++;
  else
    info->usb_bad_reads = 0;
  if (info->usb_bad_reads > 20)
    libhexc_reset (hexminerc);
  new_block:
  hash_count = (int64_t) (0xffffffffull * hash_count);
  if (libhexc_usb_dead (hexminerc))
    {
      hexminerc->shutdown = true;
      return -1;
    }
  return hash_count;
}

static void
get_hexminerc_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminerc)
{
  if (!hexminerc->device_data)
    return;
  struct hexminerc_info *info = hexminerc->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

extern void suffix_string (uint64_t val, char *buf, size_t bufsiz,
                           int sigdigits);

static struct api_data *
hexminerc_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct timeval now;
  struct hexminerc_info *info = cgpu->device_data;
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
      /*~ */
      char mcw[24];
      /*~ */
      sprintf (mcw, "Chip%d Nonces", i + 1);
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
    }
  return root;
}

struct device_drv hexminerc_drv = {
  .drv_id = DRIVER_hexminerc,
  .dname = "hexminerc",
  .name = "HEXc",
  .thread_init = hexminerc_thread_init,
  .drv_detect = hexminerc_detect,
  .hash_work = hash_driver_work,
  .scanwork = hexminerc_scanhash,
  .get_api_stats = hexminerc_api_stats,
  .get_statline_before = get_hexminerc_statline_before,
  .thread_shutdown = hexminerc_shutdown,
};
