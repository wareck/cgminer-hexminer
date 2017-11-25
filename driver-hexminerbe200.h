/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEX_BE200H
#define HEX_BE200H
#ifdef USE_HEXMINERBE200
#include "util.h"
#define DBG_HW_HEXBE200
#define DBG_HW_HEXBE200_PRINT

/* hexminerbe200_task/work_reply status Definitions */
#define HEX_BE200STAT_IDLE 0    /* Idle or data already Sent to the buffer */
#define HEX_BE200STAT_NEW_WORK 1        /* Request for write in the buffer */
#define HEX_BE200STAT_WAITING 2 /* Wait For Buffer Empty Position */
#define HEX_BE200STAT_CLR_BUFF 3        /* Force Buffer Empty */
#define HEX_BE200STAT_STOP_REQ 4        /* Stop Request */
#define HEX_BE200STAT_NEW_WORK_CLEAR_OLD 5      /* Clear Buffers and after that fill the first buffer */
#define HEX_BE200STAT_UNUSED 6
#define HEX_BE200STAT_RES_HW 12
#define HEX_BE200STAT_RES_NO_NONCE 13

/* libhex_be200_eatHashData/BUF_reply status Definitions */
#define HEX_BE200BUF_DATA 0
#define HEX_BE200BUF_ERR 1
#define HEX_BE200BUF_SKIP 2

/* MISC */
#define HEX_BE200NROLL 45
#define HEX_BE200NROLL_ARR 2*HEX_BE200NROLL + 5
#define HEXMINERBE200_ARRAY_PIC_SIZE 64
#define HEXMINERBE200_ARRAY_SIZE HEXMINERBE200_ARRAY_PIC_SIZE
#define HEXMINERBE200_ARRAY_SIZE_REAL HEXMINERBE200_ARRAY_SIZE - 2
#define HEX_BE200NONCE_CASH_SIZE 4
#define HEX_BE200USB_R_SIZE 64
#define HEX_BE200USB_WR_SIZE 64
#define HEX_BE200HASH_BUF_SIZE 1024
#define HEX_BE200HASH_BUF_SIZE_OK HEX_BE200HASH_BUF_SIZE - 4
#define HEXMINERBE200_BULK_READ_TIMEOUT 1000
#define HEX_BE200USB_WR_TIME_OUT 500
#define HEX_BE200MINER_THREADS 1
#define HEX_BE200DEFAULT_MINER_NUM 0x01
#define HEX_BE200DEFAULT_ASIC_NUM 0x10
#define HEXBE200_DEFAULT_RANGE HEX_BE200DEFAULT_ASIC_NUM - 1
#define HEX_BE200MIN_FREQUENCY 0
#define HEX_BE200MAX_FREQUENCY 400
#define HEX_BE200DEFAULT_FREQUENCY 300
#define HEX_BE200DEFAULT_CORE_VOLTAGE 820       /* in millivolts */
#define HEX_BE200MIN_COREMV 100 /* in millivolts */
#define HEX_BE200MAX_COREMV 2001        /* in millivolts */
struct chip_resultsbe200
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEX_BE200NONCE_CASH_SIZE];
};
struct hexminerbe200_task
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
  uint16_t diff;
  uint8_t csum;
  uint8_t pad[4];
} __attribute__ ((packed, aligned (4)));
struct workbe200_result
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
struct hexminerbe200_info
{
  struct timeval last_wr;
  int jobs_to_send;
  int64_t wsem_ustiming;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  bool reset_work;
  bool chips_enabled[HEX_BE200DEFAULT_ASIC_NUM];
  int usb_bad_reads;
  int write_pos;
  int chip_mask;
#ifdef DBG_HW_HEXBE200
  int nincdec[HEX_BE200NROLL_ARR];
#endif
  int miner_count;
  int asic_count;
  int pic_roll;
  int diff;
  int core_voltage;
  int opt_hexminerbe200_skip_hw_res;
  int opt_hexminerbe200_hw_err_res;
  int opt_hexminerbe200_nonce_timeout_secs;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  int b_reset_count;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int dupe[HEX_BE200DEFAULT_ASIC_NUM];
  int res_hew_err[HEX_BE200DEFAULT_ASIC_NUM];
  int res_timeout_err[HEX_BE200DEFAULT_ASIC_NUM];
  int matching_work[HEX_BE200DEFAULT_ASIC_NUM];
  int hw_err[HEX_BE200DEFAULT_ASIC_NUM];
  unsigned int work_block_local;
  unsigned int work_pool_update;
  unsigned char *readbuf;
  struct workbe200_result *wr;
  struct chip_resultsbe200 *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
  time_t last_chip_valid_work[HEX_BE200DEFAULT_ASIC_NUM];
  struct hexminerbe200_task *ht;
};
struct hexminerbe200_config_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint16_t hashclock;
  uint16_t refvoltage;
  uint32_t difficulty;
  uint16_t chip_mask;
  uint8_t csum;
} __attribute__ ((packed, aligned (4)));
#define HEX_BE200WORKANSWER_ADR 0x3000
#define HEX_BE200RES_ADR 0x30AC
#define HEXMINERBE200_TASK_SIZE (sizeof(struct hexminerbe200_task)-4)
#define HEX_BE200MAX_WORK_SIZE (sizeof(struct workbe200_result)-4)
#define HEX_BE200BASE_WORK_SIZE 6
extern int opt_hexminerbe200_core_voltage;
extern int opt_hexminerbe200_chip_mask;
extern int opt_hexminerbe200_hw_err_res;
extern int opt_hexminerbe200_pic_roll;
extern int opt_hexminerbe200_nonce_timeout_secs;
extern int opt_hexminerbe200_diff;
extern int opt_hexminerbe200_skip_hw_res;

//extern char *libhex_be200_set_config_chip_mask (char *arg);

extern struct hexminerbe200_info **hexminerbe200_info;
#endif /* USE_HEXMINERBE200 */
#endif /* HEX_BE200H */
