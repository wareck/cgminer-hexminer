/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEXR_H
#define HEXR_H
#ifdef USE_HEXMINERR
#include "util.h"

/* hexminerr_task/work_reply status Definitions: */
#define HEXR_STAT_IDLE 0        /* Idle or data already Sent to the buffer */
#define HEXR_STAT_NEW_WORK 1    /* Request for write in the buffer */
#define HEXR_STAT_WAITING 2     /* Wait For Buffer Empty Position */
#define HEXR_STAT_CLR_BUFF 3    /* Force Buffer Empty */
#define HEXR_STAT_STOP_REQ 4    /* Stop Request */
#define HEXR_STAT_NEW_WORK_CLEAR_OLD 5  /* Clear Buffers and after that fill the first buffer */
#define HEXR_STAT_UNUSED 6
#define HEXR_STAT_RES_HW 12
#define HEXR_STAT_RES_NO_NONCE 13
/* libhexr_eatHashData/BUF_reply status Definitions: */
#define HEXR_BUF_DATA 0
#define HEXR_BUF_ERR 1
#define HEXR_BUF_SKIP 2
/* MISC */
#define HEXMINERR_ARRAY_PIC_SIZE 64
#define HEXMINERR_ARRAY_SIZE HEXMINERR_ARRAY_PIC_SIZE
#define HEXMINERR_ARRAY_SIZE_REAL HEXMINERR_ARRAY_SIZE - 2
#define HEXR_NONCE_CASH_SIZE 4
#define HEXR_USB_R_SIZE 64
#define HEXR_USB_WR_SIZE 64
#define HEXR_HASH_BUF_SIZE 2048
#define HEXR_HASH_BUF_SIZE_OK HEXR_HASH_BUF_SIZE - 4
#define HEXMINERR_BULK_READ_TIMEOUT 1000
#define HEXR_USB_WR_TIME_OUT 500
#define HEXR_MINER_THREADS 1
#define HEXR_DEFAULT_MINER_NUM 0x01
#define HEXR_DEFAULT_ASIC_NUM 0x04
#define HEXR_DEFAULT_RANGE HEXR_DEFAULT_ASIC_NUM - 1
#define HEXR_MIN_FREQUENCY 0
#define HEXR_MAX_FREQUENCY 1500
#define HEXR_DEFAULT_FREQUENCY 650
#define HEXR_DEFAULT_CORE_VOLTAGE 690   /* in millivolts */
#define HEXR_MIN_COREMV 600     /* in millivolts */
#define HEXR_MAX_COREMV 1001    /* in millivolts */
struct chip_resultsr
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXR_NONCE_CASH_SIZE];
};
struct hexminerr_task
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
  uint8_t leading_zeros;
  uint8_t dum1;
  uint8_t csum;
  uint8_t pad[4];
  //uint8_t csum;
  //uint8_t pad[6];
} __attribute__ ((packed, aligned (4)));
struct workr_result
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
struct hexminerr_info
{
  struct timeval last_wr;
  int jobs_to_send;
  int64_t wsem_ustiming;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  bool reset_work;
  bool work_ping;
  int usb_bad_reads;
  int write_pos;
  int chip_mask;
  bool chips_enabled[HEXR_DEFAULT_ASIC_NUM];
  int miner_count;
  int asic_count;
  int pic_roll;
  int core_voltage;
  //int opt_hexminerr_hw_err_res;
  //int opt_hexminerr_nonce_timeout_secs;
  //double opt_hexminerr_reset_below_threshold;
  //double opt_hexminerr_reset_below_threshold_wait;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  int b_reset_count;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int opt_hexminerr_leading_zeros;
  int hexminerr_work_count;
  int dupe[HEXR_DEFAULT_ASIC_NUM];
  int res_hew_err[HEXR_DEFAULT_ASIC_NUM];
  int res_timeout_err[HEXR_DEFAULT_ASIC_NUM];
  int matching_work[HEXR_DEFAULT_ASIC_NUM];
  int hw_err[HEXR_DEFAULT_ASIC_NUM];
  unsigned int work_block_local;
  unsigned int work_pool_update;
  unsigned char *readbuf;
  struct workr_result *wr;
  struct chip_resultsr *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
  time_t last_chip_valid_work[HEXR_DEFAULT_ASIC_NUM];
  struct hexminerr_task *ht;
  //double rolling1;
};
struct hexminerr_config_task
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

#define HEXR_WORKANSWER_ADR 0x3000
#define HEXR_RES_ADR 0x30AC
//#define HEXMINERR_TASK_SIZE (sizeof(struct hexminerr_task)-6)
#define HEXMINERR_TASK_SIZE (sizeof(struct hexminerr_task)-4)
#define HEXR_MAX_WORK_SIZE (sizeof(struct workr_result)-4)
#define HEXR_BASE_WORK_SIZE 6
extern int opt_hexminerr_core_voltage;
extern int opt_hexminerr_chip_mask;
//extern int opt_hexminerr_hw_err_res;
extern int opt_hexminerr_pic_roll;
//extern int opt_hexminerr_nonce_timeout_secs;
//extern int opt_hexminerr_reset_below_threshold;
//extern int opt_hexminerr_reset_below_threshold_wait;
extern int opt_hexminerr_leading_zeros;
extern struct hexminerr_info **hexminerr_info;

#endif /* USE_HEXMINERR */
#endif /* HEXR_H */
