/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
#ifndef HEXU_H
#define HEXU_H
#ifdef USE_HEXMINERU
#include "util.h"
enum mcp2210_gpio_value
{ MGV_LOW, MGV_HIGH, MGV_ERROR, };
#define NANOFURY_GP_PIN_LED 0
#define NANOFURY_GP_PIN_SCK_OVR 5
#define NANOFURY_GP_PIN_PWR_EN 6
#define NANOFURY_MAX_BYTES_PER_SPI_TRANSFER 60  // due to MCP2210 limitation
#define HEXMINERU_ARRAY_SIZE 9
#define HEXMINERU_ARRAY_SIZE_REAL HEXMINERU_ARRAY_SIZE - 2
#define HEXU_NONCE_CASH_SIZE 256
#define HEXMINERU_PUSH_THRESH 2 /* At least 2 queued works available to be written to PIC */
#define HEXMINERU_ARRAY_MAX_POP 1
#define HEXU_USB_R_SIZE 64
#define HEXU_USB_WR_SIZE 64
#define HEXU_HASH_BUF_SIZE 1024
#define HEXU_MINER_THREADS 1
#define HEXU_MIN_FREQUENCY 0    //Bits
#define HEXU_MAX_FREQUENCY 64   //Bits
#define HEXU_DEFAULT_FREQUENCY 54       //Bits Stable which works 2.5 GHs for 1 chip
#define HEXU_DEFAULT_CORE_VOLTAGE 0     /* in millivolts */
struct chip_resultsu
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXU_NONCE_CASH_SIZE];
};
struct hexmineru_info
{
  uint8_t frequency;
  uint8_t ltsz;
  int dev_reset_count;
  int dupe[1];
  int roll;
  struct thr_info *thr;
  struct work **hexworks;
  unsigned char wr_spi[80];
  uint32_t read_spi[256];
  uint32_t atrvecs[HEXMINERU_ARRAY_SIZE][19];
  struct chip_resultsu *array_nonce_cache;
  uint32_t buf_switch;
  size_t spipos;
  uint32_t atrvec[19];
  bool job_switch;
  bool shut_read;
  bool shut_write;
  struct work *work;
  unsigned int work_block;
  unsigned int work_pool_update;
  int c_job_id;
  int l_job_id;
  int read_pos;
  cgsem_t qsem;
};
struct hexmineru_task
{
  unsigned char midstate[32];
  unsigned int junk[8];
  unsigned m7;
  unsigned ntime;
  unsigned nbits;
} __attribute__ ((packed, aligned (4)));
extern int opt_hexmineru_core_freq;
#define HEXMINERU_TASK_SIZE (sizeof(struct hexmineru_task)) extern struct hexmineru_info **hexmineru_info;
#endif /* USE_HEXMINERU */
#endif /* HEXU_H */
