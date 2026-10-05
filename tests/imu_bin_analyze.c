/**
 * @file imu_bin_analyze.c
 * @brief 用正式后端 imu_sensor.c 分析串口原始录制 .bin 的主机工具。
 *
 * 复用被测实现做 7E23 解析：把整个录制按 64 字节分块喂入（合成时间戳
 * 按 115200 波特率折算），输出帧统计、各功能字实际频率与样本值。
 * 仅用于离线分析录制文件，不代表在线链路。
 *
 * 编译：gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc \
 *           tests/imu_bin_analyze.c -o build/imu_bin_analyze.exe
 * 用法：build/imu_bin_analyze.exe <recording.bin> [baud]
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../Core/Inc/imu_sensor.h"
#include "../Core/Src/imu_sensor.c"

static int Test_TxSilent(void *user, const uint8_t *data, uint16_t len)
{
  (void)user;
  (void)data;
  (void)len;
  return -1; /* 分析工具绝不发送。 */
}

static void HexDump(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    printf("%02X ", data[i]);
    if ((i % 16U) == 15U)
    {
      printf("\n");
    }
  }
  if ((len % 16U) != 0U)
  {
    printf("\n");
  }
}

/** 可打印 ASCII 时以文本展示，便于识别横幅类内容。 */
static void PrintAsciiPreview(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t printable = 1U;

  for (i = 0U; i < len; i++)
  {
    if ((data[i] < 0x20U) || (data[i] > 0x7EU))
    {
      printable = 0U;
      break;
    }
  }
  if (printable)
  {
    printf("ascii: \"");
    for (i = 0U; i < len; i++)
    {
      putchar(data[i]);
    }
    printf("\"\n");
  }
}

int main(int argc, char **argv)
{
  ImuSensor_Config cfg;
  ImuSensor_Sample s;
  FILE *f;
  long file_len;
  uint8_t *data;
  uint32_t pos = 0U;
  uint32_t baud = 115200U;
  uint64_t fed_bytes = 0U;
  uint32_t raw_snapshots = 0U;
  uint32_t quat_snapshots = 0U;
  uint32_t euler_snapshots = 0U;
  double span_s;

  if (argc < 2)
  {
    fprintf(stderr, "usage: %s <recording.bin> [baud]\n", argv[0]);
    return 2;
  }
  if (argc >= 3)
  {
    baud = (uint32_t)strtoul(argv[2], NULL, 10);
    if (baud == 0U)
    {
      baud = 115200U;
    }
  }

  f = fopen(argv[1], "rb");
  if (f == NULL)
  {
    fprintf(stderr, "cannot open %s\n", argv[1]);
    return 2;
  }
  fseek(f, 0, SEEK_END);
  file_len = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (file_len <= 0)
  {
    fprintf(stderr, "empty file\n");
    fclose(f);
    return 2;
  }
  data = (uint8_t *)malloc((size_t)file_len);
  if (data == NULL)
  {
    fclose(f);
    return 2;
  }
  if (fread(data, 1U, (size_t)file_len, f) != (size_t)file_len)
  {
    free(data);
    fclose(f);
    return 2;
  }
  fclose(f);

  cfg.tx = Test_TxSilent;
  cfg.tx_user = NULL;
  cfg.request_timeout_us = 0U;
  ImuSensor_Init(&cfg);

  printf("file: %s (%ld bytes)\n", argv[1], file_len);
  printf("first-64-hexdump:\n");
  HexDump(data, (uint16_t)((file_len < 64) ? file_len : 64));
  PrintAsciiPreview(data, (uint16_t)((file_len < 64) ? file_len : 64));

  while (pos < (uint32_t)file_len)
  {
    uint16_t chunk = 64U;
    uint64_t now_us;

    if ((uint32_t)(file_len - pos) < chunk)
    {
      chunk = (uint16_t)(file_len - pos);
    }
    now_us = (uint64_t)(((uint64_t)fed_bytes * 1000000ULL) / baud);
    (void)ImuSensor_Feed(&data[pos], chunk, now_us);
    fed_bytes += chunk;
    ImuSensor_Process(now_us + 1ULL);

    if (ImuSensor_GetSample(&s, now_us + 1ULL) == 1)
    {
      if ((s.raw_seq > raw_snapshots) && (raw_snapshots < 3U))
      {
        raw_snapshots = s.raw_seq;
        printf("RAW#%u t=%.3fs accel_g=[%.4f %.4f %.4f] gyro_rad_s=[%.4f %.4f %.4f] mag=[%.4f %.4f %.4f]\n",
               s.raw_seq, (double)s.raw_time_us / 1.0e6,
               s.accel_g[0], s.accel_g[1], s.accel_g[2],
               s.gyro_rad_s[0], s.gyro_rad_s[1], s.gyro_rad_s[2],
               s.mag_units[0], s.mag_units[1], s.mag_units[2]);
      }
      if ((s.quat_seq > quat_snapshots) && (quat_snapshots < 3U))
      {
        quat_snapshots = s.quat_seq;
        printf("QUAT#%u t=%.3fs wxyz=[%.5f %.5f %.5f %.5f]\n",
               s.quat_seq, (double)s.quat_time_us / 1.0e6,
               s.quat_wxyz[0], s.quat_wxyz[1], s.quat_wxyz[2],
               s.quat_wxyz[3]);
      }
      if ((s.euler_seq > euler_snapshots) && (euler_snapshots < 3U))
      {
        euler_snapshots = s.euler_seq;
        printf("EULER#%u t=%.3fs rpy_rad=[%.5f %.5f %.5f] deg=[%.2f %.2f %.2f]\n",
               s.euler_seq, (double)s.euler_time_us / 1.0e6,
               s.euler_rpy_rad[0], s.euler_rpy_rad[1], s.euler_rpy_rad[2],
               s.euler_rpy_rad[0] * 57.29577951308232,
               s.euler_rpy_rad[1] * 57.29577951308232,
               s.euler_rpy_rad[2] * 57.29577951308232);
      }
    }
    pos = (uint32_t)(pos + chunk);
  }

  span_s = ((double)fed_bytes * 1.0) / (double)baud;
  (void)ImuSensor_GetSample(&s, (uint64_t)(fed_bytes * 1000000ULL / baud) + 1ULL);

  printf("---- summary ----\n");
  printf("bytes=%llu span=%.3fs\n",
         (unsigned long long)fed_bytes, span_s);
  printf("good_frames=%u raw=%u quat=%u euler=%u baro=%u\n",
         s.good_frames, s.raw_seq, s.quat_seq, s.euler_seq, s.baro_seq);
  if (span_s > 0.0)
  {
    printf("rate_hz: raw=%.2f quat=%.2f euler=%.2f baro=%.2f\n",
           (double)s.raw_seq / span_s, (double)s.quat_seq / span_s,
           (double)s.euler_seq / span_s, (double)s.baro_seq / span_s);
  }
  printf("errors: bad_checksum=%u bad_length=%u nonfinite=%u unknown_func=%u unexpected=%u drops=%u\n",
         s.bad_checksum_frames, s.bad_length_frames, s.nonfinite_frames,
         s.unknown_func_frames, s.unexpected_replies, s.rx_dropped_bytes);
  printf("version_valid=%u\n", s.version_valid);
  free(data);
  return 0;
}
