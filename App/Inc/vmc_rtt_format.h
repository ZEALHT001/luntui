#ifndef VMC_RTT_FORMAT_H
#define VMC_RTT_FORMAT_H

#include <stdint.h>

/*
 * Pure formatting helpers for the SEGGER RTT telemetry streams.
 *
 * These replace the per-field snprintf calls.  The control task used to enter
 * the libc printf machinery about 17850 times per second (137 fields per
 * VMCVOFA frame at 50 Hz plus 44 per VMCFAST frame at 250 Hz), all of it
 * synchronous inside the control loop.  The functions below produce the same
 * bytes as the format strings they replace; tools/test_rtt_format.c checks
 * that exhaustively against snprintf on the host.
 *
 * Everything here is static inline and free of HAL/RTOS dependencies so the
 * firmware and the host regression test compile exactly the same code.
 */

#define VMC_LQR_RTT_TELEM_SCALE 1000L
#define VMC_LQR_RTT_QR_SCALE 1000000000LL

/* Write an unsigned decimal integer, most significant digit first.  Returns
 * the number of characters written.  The caller must guarantee 10 bytes. */
static inline int vmc_rtt_put_dec(char *dst, uint32_t value)
{
  char tmp[10];
  int n = 0;
  int i;

  if (value == 0U) {
    dst[0] = '0';
    return 1;
  }
  while (value != 0U && n < (int)sizeof(tmp)) {
    tmp[n++] = (char)('0' + (int)(value % 10U));
    value /= 10U;
  }
  for (i = 0; i < n; i++) {
    dst[i] = tmp[n - 1 - i];
  }
  return n;
}

/* Append ",<value>" with exactly three decimals.  Mirrors the previous
 * snprintf(",%ld.%03ld", whole, frac) encoding, including the "-0.000" form
 * produced for small negative inputs.  Returns the new offset, or max_len when
 * the remaining space cannot hold the widest field. */
static inline int vmc_lqr_rtt_append_scaled(char *line, int offset, int max_len,
                                            double value)
{
  const long scaled = (long)(value * (double)VMC_LQR_RTT_TELEM_SCALE);
  const int negative = (scaled < 0L || (value < 0.0 && scaled == 0L)) ? 1 : 0;
  unsigned long magnitude;
  unsigned long frac;
  uint32_t whole;
  int pos;

  if (scaled < 0L) {
    /* Written this way so LONG_MIN does not overflow the negation. */
    magnitude = (unsigned long)(-(scaled + 1L)) + 1UL;
  } else if (negative != 0) {
    magnitude = 1UL;
  } else {
    magnitude = (unsigned long)scaled;
  }

  whole = (uint32_t)(magnitude / (unsigned long)VMC_LQR_RTT_TELEM_SCALE);
  frac = magnitude % (unsigned long)VMC_LQR_RTT_TELEM_SCALE;

  /* Worst case: ',' '-' 10 digits '.' 3 digits = 16 characters. */
  if ((max_len - offset) < 17) {
    return max_len;
  }

  pos = offset;
  line[pos++] = ',';
  if (negative != 0) {
    line[pos++] = '-';
  }
  pos += vmc_rtt_put_dec(&line[pos], whole);
  line[pos++] = '.';
  line[pos++] = (char)('0' + (int)((frac / 100UL) % 10UL));
  line[pos++] = (char)('0' + (int)((frac / 10UL) % 10UL));
  line[pos++] = (char)('0' + (int)(frac % 10UL));
  return pos;
}

/* Append ",<value>" for an unsigned integer. */
static inline int vmc_lqr_rtt_append_u32(char *line, int offset, int max_len,
                                         uint32_t value)
{
  int pos;

  if ((max_len - offset) < 12) {
    return max_len;
  }
  pos = offset;
  line[pos++] = ',';
  pos += vmc_rtt_put_dec(&line[pos], value);
  return pos;
}

/* Append a nine-decimal Q/R value.  Q/R span a huge dynamic range: a
 * physically calibrated r_wheel is around 1e-4 m2/s2 while the legacy
 * defaults were around 1e2, and a three-decimal wire format rounds the small
 * end to zero.  Values are clamped to +/-9e9 so the integer part always fits
 * the 10-character buffer above. */
static inline int vmc_lqr_rtt_append_qr(char *line, int offset, int max_len,
                                        double value)
{
  long long scaled;
  unsigned long long magnitude;
  unsigned long long whole;
  unsigned long long frac;
  int negative;
  int pos;
  int i;
  char tmp[20];

  if (value > 9.0e9) {
    value = 9.0e9;
  } else if (value < -9.0e9) {
    value = -9.0e9;
  }
  scaled = (long long)(value * (double)VMC_LQR_RTT_QR_SCALE);
  negative = (scaled < 0LL || (value < 0.0 && scaled == 0LL)) ? 1 : 0;

  if (scaled < 0LL) {
    magnitude = (unsigned long long)(-(scaled + 1LL)) + 1ULL;
  } else {
    magnitude = (unsigned long long)scaled;
  }
  whole = magnitude / (unsigned long long)VMC_LQR_RTT_QR_SCALE;
  frac = magnitude % (unsigned long long)VMC_LQR_RTT_QR_SCALE;

  /* Worst case: ',' '-' 10 digits '.' 9 digits = 22 characters. */
  if ((max_len - offset) < 23) {
    return max_len;
  }

  pos = offset;
  line[pos++] = ',';
  if (negative != 0) {
    line[pos++] = '-';
  }
  if (whole == 0ULL) {
    line[pos++] = '0';
  } else {
    i = 0;
    while (whole != 0ULL && i < (int)sizeof(tmp)) {
      tmp[i++] = (char)('0' + (int)(whole % 10ULL));
      whole /= 10ULL;
    }
    while (i > 0) {
      line[pos++] = tmp[--i];
    }
  }
  line[pos++] = '.';
  for (i = 8; i >= 0; i--) {
    unsigned long long divisor = 1ULL;
    int k;
    for (k = 0; k < i; k++) {
      divisor *= 10ULL;
    }
    line[pos++] = (char)('0' + (int)((frac / divisor) % 10ULL));
  }
  return pos;
}

/*
 * Binary VMCFAST encoding helpers.
 *
 * The high-rate stream is little-endian fixed point rather than ASCII.  ASCII
 * cost 277 B per frame for 43 fields (mean 5.4 B/field) where int16 costs 2 B,
 * and the shrink is what buys back RTT buffer headroom.
 */

/* Quantise to int16 with round-half-away-from-zero and saturation.  A scale of
 * 0 means "already an integer in range". */
static inline int16_t vmc_rtt_scale_i16(double value, int32_t scale)
{
  double scaled;

  if (scale == 0) {
    scaled = value;
  } else {
    scaled = value * (double)scale;
  }
  if (scaled > 32767.0) {
    return (int16_t)32767;
  }
  if (scaled < -32768.0) {
    return (int16_t)-32768;
  }
  scaled = (scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5);
  return (int16_t)scaled;
}

static inline void vmc_rtt_put_u16le(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static inline void vmc_rtt_put_u32le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static inline void vmc_rtt_put_i16le(uint8_t *dst, int16_t value)
{
  vmc_rtt_put_u16le(dst, (uint16_t)value);
}

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).  The frame is small enough
 * that the bitwise form costs well under 1% of the control loop. */
static inline uint16_t vmc_rtt_crc16(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;
  uint32_t i;
  int bit;

  for (i = 0U; i < length; i++) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

#endif /* VMC_RTT_FORMAT_H */
