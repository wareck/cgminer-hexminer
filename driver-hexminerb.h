/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEXB_H
#define HEXB_H
#ifdef USE_HEXMINERB
#include "util.h"

/* hexminerb_task/work_reply status Definitions: */
#define HEXB_STAT_IDLE 0        /* Idle or data already Sent to the buffer */
#define HEXB_STAT_NEW_WORK 6    /* Request for write in the buffer */
#define HEXB_STAT_WAITING 2     /* Wait For Buffer Empty Position */
#define HEXB_STAT_CLR_BUFF 3    /* Force Buffer Empty */
#define HEXB_STAT_STOP_REQ 4    /* Stop Request */
#define HEXB_STAT_NEW_WORK_CLEAR_OLD 5  /* Clear Buffers and after that fill the first buffer */
#define HEXB_STAT_UNUSED 7

/* libhexb_eatHashData/BUF_reply status Definitions: */
#define HEXB_BUF_DATA 0
#define HEXB_BUF_ERR 1
#define HEXB_BUF_SKIP 2

 /*MISC*/
#define HEXMINERB_ARRAY_PIC_SIZE 64
#define HEXMINERB_ARRAY_SIZE HEXMINERB_ARRAY_PIC_SIZE * 4
#define HEXMINERB_ARRAY_SIZE_REAL HEXMINERB_ARRAY_SIZE - 2
#define HEXB_NONCE_CASH_SIZE 16
#define HEXB_USB_R_SIZE 64
#define HEXB_USB_WR_SIZE 64
#define HEXB_HASH_BUF_SIZE 768
#define HEXB_HASH_BUF_SIZE_OK HEXB_HASH_BUF_SIZE - 4
#define HEXB_USB_R_BAD_ID 32
#define HEXB_USB_WR_TIME_OUT 500
#define HEXMINERB_BULK_READ_TIMEOUT 1000
#define HEXB_MINER_THREADS 1
#define HEXB_DEFAULT_MINER_NUM 0x01
#define HEXB_DEFAULT_ASIC_NUM 0x10
#define HEXB_MIN_FREQUENCY 0    //Bits / 10
#define HEXB_MAX_FREQUENCY 610  //Bits / 10
#define HEXB_DEFAULT_FREQUENCY 540      //Bits / 10 - That is Max which works 40 GHs for 16 chips
#define HEXB_DEFAULT_CORE_VOLTAGE 840   /* in millivolts */
#define HEXB_MIN_COREMV 700     /* in millivolts */
#define HEXB_MAX_COREMV 1101    /* in millivolts */

struct chip_resultsb
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXB_NONCE_CASH_SIZE];
};
struct workb_result
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t lastnonce;
  uint8_t lastnonceid;
  uint8_t status;
  uint16_t lastvoltage;
  uint8_t lastchippos;
  uint8_t prevnonceid;
  uint8_t csum;
} __attribute__ ((packed, aligned (4)));
struct hexminerb_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
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
struct hexminerb_info
{
  struct timeval last_wr;
  int jobs_to_send;
  int64_t wsem_ustiming;
  bool reset_work;
  int roll;
  int usb_bad_reads;
  struct work *work;
  unsigned int work_block_local;
  unsigned int work_pool_update;
  int write_pos;
  struct hexminerb_task *ht;
  int miner_count;
  int asic_count;
  int core_voltage;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int dupe[HEXB_DEFAULT_ASIC_NUM];
  int matching_work[HEXB_DEFAULT_ASIC_NUM];
  unsigned char *readbuf;
  struct workb_result *wr;
  struct chip_resultsb *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
};

#define HEXB_WORKANSWER_ADR 0x3000
#define HEXB_WORKQUEUE_ADR 0x4008
#define HEXB_PTCON_ADR 0x0C00
#define HEXB_START_STOP_ADR 0x646E
#define HEXMINERB_TASK_SIZE (sizeof(struct hexminerb_task) - 2)
#define HEXB_MAX_WORK_SIZE (sizeof(struct workb_result))
#define HEXB_BASE_WORK_SIZE 6
extern int opt_hexminerb_core_voltage;
extern struct hexminerb_info **hexminerb_info;

#endif /* USE_HEXMINERB */
#endif /* HEXB_H */
