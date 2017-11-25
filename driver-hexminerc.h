/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEXC_H
#define HEXC_H
#ifdef USE_HEXMINERC
#include "util.h"
#define HEXC_USB_WR_TIME_OUT 500
#define HEXMINERC_BULK_READ_TIMEOUT 1000
#define HEXC_USB_R_SIZE 64

/* hexminerc_task/work_reply status Definitions: */
#define HEXC_STAT_IDLE 0        /* Idle or data already Sent to the buffer */
#define HEXC_STAT_NEW_WORK 1    /* Request for write in the buffer */
#define HEXC_STAT_WAITING 2     /* Wait For Buffer Empty Position */
#define HEXC_STAT_CLR_BUFF 3    /* Force Buffer Empty */
#define HEXC_STAT_STOP_REQ 4    /* Stop Request */
#define HEXC_STAT_NEW_WORK_CLEAR_OLD 5  /* Clear Buffers and after that fill the first buffer */
#define HEXC_STAT_UNUSED 6

/* libhexc_eatHashData/BUF_reply status Definitions: */
#define HEXC_BUF_DATA 0
#define HEXC_BUF_ERR 1
#define HEXC_BUF_SKIP 2

/*MISC*/
#define HEXC_DEFAULT_XCLKIN_CLOCK 32    /* In MHz */
#define HEXC_CLOCK_LOW_CFG 0x00030017
#define HEXC_CLOCK_HIGH_CFG (0x0000002e << 3)   /* = 0x00000170 */
#define HEXMINERC_ARRAY_PIC_SIZE 64
#define HEXMINERC_ARRAY_SIZE HEXMINERC_ARRAY_PIC_SIZE * 4
#define HEXMINERC_ARRAY_SIZE_REAL HEXMINERC_ARRAY_SIZE - 2
#define HEXC_NONCE_CASH_SIZE 6
#define HEXMINERC_WORK_FACTOR 0.9
#define HEXC_USB_WR_SIZE 64
#define HEXC_HASH_BUF_SIZE 512
#define HEXC_HASH_BUF_SIZE_OK HEXC_HASH_BUF_SIZE - 4
#define HEXC_MINER_THREADS 1
#define HEXC_DEFAULT_MINER_NUM 0x01
#define HEXC_DEFAULT_ASIC_NUM 0x10
#define HEXC_MIN_FREQUENCY 100
#define HEXC_MAX_FREQUENCY 2500
#define HEXC_DEFAULT_FREQUENCY 1500
#define HEXC_DEFAULT_CORE_VOLTAGE 1100  /* in millivolts */
#define HEXC_MIN_COREMV 100 /* in millivolts */ /* Do not touch it!!! 1.6V is above the chip specs already */
#define HEXC_MAX_COREMV 1630    /* in millivolts */

struct chip_resultsc
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXC_NONCE_CASH_SIZE];
};
struct workc_result
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t lastnonce;
  uint8_t lastnonceid;
  uint8_t status;
  uint16_t lastvoltage;
  uint16_t lasttemperature;
  uint16_t lastfanrpm;
  uint8_t csum;
  uint8_t pad[2];
} __attribute__ ((packed, aligned (4)));
struct hexminerc_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t clockcfg[2];
  uint32_t merkle[3];
  uint32_t a1;
  uint32_t a0;
  uint32_t e2;
  uint32_t e1;
  uint32_t e0;
  uint8_t midstate[32];
  uint32_t a2;
  uint32_t startnonce;
  uint8_t id;
  uint8_t status;
  uint16_t hashclock;
  uint16_t chipcount;
  uint16_t refvoltage;
  uint16_t reftemperature;
  uint16_t reffanrpm;
  uint8_t csum;
  uint8_t pad[2];
} __attribute__ ((packed, aligned (4)));
struct hexminerc_info
{
  struct timeval last_wr;
  int jobs_to_send;
  int64_t wsem_ustiming;
  bool reset_work;
  int usb_bad_reads;
  int write_pos;
  int roll;
  int miner_count;
  int asic_count;
  int core_voltage;
  int frequency;
  int hash_read_pos;
  int hash_write_pos;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  int b_reset_count;
  int pic_voltage_readings;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  int dupe[HEXC_DEFAULT_ASIC_NUM];
  int matching_work[HEXC_DEFAULT_ASIC_NUM];
  int chip_con_resets[16];
  bool chip_is_dead[16];
  time_t last_chip_valid_work[HEXC_DEFAULT_ASIC_NUM];
  time_t power_checked;
  unsigned char *readbuf;
  struct workc_result *wr;
  struct chip_resultsc *array_nonce_cache;
  uint32_t nonces_range[HEXC_DEFAULT_ASIC_NUM];
  struct thr_info *thr;
  struct work **hexworks;
  struct hexminerc_task *ht;
  struct work *work;
  unsigned int work_block_local;
  unsigned int work_pool_update;
};

#define HEXC_WORKANSWER_ADR 0x3000
#define HEXC_WORKQUEUE_ADR 0x4000
#define HEXC_PTCON_ADR 0x0C00
#define HEXC_START_STOP_ADR 0x646E
#define HEXMINERC_TASK_SIZE (sizeof(struct hexminerc_task)-2)
#define HEXC_MAX_WORK_SIZE (sizeof(struct workc_result) - 2)
#define HEXC_BASE_WORK_SIZE 6
extern int opt_hexminerc_core_voltage;
extern struct hexminerc_info **hexminerc_info;

#endif /* USE_HEXMINERC */
#endif /* HEXC_H */
