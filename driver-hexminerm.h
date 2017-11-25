/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEXM_H
#define HEXM_H
#ifdef USE_HEXMINERM
#include "util.h"

/* hexminerm_task/work_reply status Definitions: */
#define HEXM_STAT_IDLE 0        /* Idle or data already Sent to the buffer */
#define HEXM_STAT_NEW_WORK 1    /* Request for write in the buffer */
#define HEXM_STAT_WAITING 2     /* Wait For Buffer Empty Position */
#define HEXM_STAT_CLR_BUFF 3    /* Force Buffer Empty */
#define HEXM_STAT_STOP_REQ 4    /* Stop Request */
#define HEXM_STAT_NEW_WORK_CLEAR_OLD 5  /* Clear Buffers and after that fill the first buffer */
#define HEXM_STAT_UNUSED 6
#define HEXM_STAT_RES_HW 12
#define HEXM_STAT_RES_NO_NONCE 13
/* libhexm_eatHashData/BUF_reply status Definitions: */
#define HEXM_BUF_DATA 0
#define HEXM_BUF_ERR 1
#define HEXM_BUF_SKIP 2
/* MISC */
#define HEXMINERM_ARRAY_PIC_SIZE 64
#define HEXMINERM_ARRAY_SIZE HEXMINERM_ARRAY_PIC_SIZE
#define HEXMINERM_ARRAY_SIZE_REAL HEXMINERM_ARRAY_SIZE - 2
#define HEXM_NONCE_CASH_SIZE 4
#define HEXM_USB_R_SIZE 64
#define HEXM_USB_WR_SIZE 64
#define HEXM_HASH_BUF_SIZE 2048
#define HEXM_HASH_BUF_SIZE_OK HEXM_HASH_BUF_SIZE - 4
#define HEXMINERM_BULK_READ_TIMEOUT 1000
#define HEXM_USB_WR_TIME_OUT 500
#define HEXM_MINER_THREADS 1
#define HEXM_DEFAULT_MINER_NUM 0x01
#define HEXM_DEFAULT_ASIC_NUM 0x04
#define HEXM_DEFAULT_RANGE HEXM_DEFAULT_ASIC_NUM - 1
#define HEXM_MIN_FREQUENCY 0
#define HEXM_MAX_FREQUENCY 1200
#define HEXM_DEFAULT_FREQUENCY 1024
#define HEXM_DEFAULT_CORE_VOLTAGE 820   /* in millivolts */
#define HEXM_MIN_COREMV 600     /* in millivolts */
#define HEXM_MAX_COREMV 1001    /* in millivolts */
struct chip_resultsm
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXM_NONCE_CASH_SIZE];
};
struct hexminerm_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint8_t midstate[32];
  uint32_t merkle[3];
  uint8_t roll;
  uint8_t id;
  uint8_t status;
  uint8_t dum;
  uint8_t csum;
  uint8_t pad[6];
} __attribute__ ((packed, aligned (4)));
struct workm_result
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t lastnonce;
  uint8_t roll;
  uint8_t lastnonceid;
  uint16_t lastvoltage;
  uint8_t lastchippos;
  uint8_t buf_empty_space;
  uint8_t csum;
  uint8_t pad[4];
} __attribute__ ((packed, aligned (4)));
struct hexminerm_info
{
  struct timeval last_wr;
  int jobs_to_send;
  int64_t wsem_ustiming;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  bool reset_work;
  int usb_bad_reads;
  int write_pos;
  int chip_mask;
  bool chips_enabled[HEXM_DEFAULT_ASIC_NUM];
  int miner_count;
  int asic_count;
  int pic_roll;
  int core_voltage;
  int opt_hexminerm_hw_err_res;
  int opt_hexminerm_nonce_timeout_secs;
  double opt_hexminerm_reset_below_threshold;
  double opt_hexminerm_reset_below_threshold_wait;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  int b_reset_count;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int dupe[HEXM_DEFAULT_ASIC_NUM];
  int res_hew_err[HEXM_DEFAULT_ASIC_NUM];
  int res_timeout_err[HEXM_DEFAULT_ASIC_NUM];
  int matching_work[HEXM_DEFAULT_ASIC_NUM];
  int hw_err[HEXM_DEFAULT_ASIC_NUM];
  unsigned int work_block_local;
  unsigned int work_pool_update;
  unsigned char *readbuf;
  struct workm_result *wr;
  struct chip_resultsm *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
  time_t last_chip_valid_work[HEXM_DEFAULT_ASIC_NUM];
  struct hexminerm_task *ht;
  double rolling1;
};
struct hexminerm_config_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint16_t hashclock;
  uint16_t refvoltage;
  uint32_t difficulty;
  uint8_t chip_mask;
  uint8_t wr_interwal;
  uint8_t csum;
} __attribute__ ((packed, aligned (4)));

#define HEXM_WORKANSWER_ADR 0x3000
#define HEXM_RES_ADR 0x30AC
#define HEXMINERM_TASK_SIZE (sizeof(struct hexminerm_task)-6)
#define HEXM_MAX_WORK_SIZE (sizeof(struct workm_result)-4)
#define HEXM_BASE_WORK_SIZE 6
extern int opt_hexminerm_core_voltage;
extern int opt_hexminerm_chip_mask;
extern int opt_hexminerm_hw_err_res;
extern int opt_hexminerm_pic_roll;
extern int opt_hexminerm_nonce_timeout_secs;
extern int opt_hexminerm_reset_below_threshold;
extern int opt_hexminerm_reset_below_threshold_wait;
extern struct hexminerm_info **hexminerm_info;

#endif /* USE_HEXMINERM */
#endif /* HEXM_H */
