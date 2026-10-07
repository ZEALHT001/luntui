/*
 * Host regression test for the RTT telemetry formatters.
 *
 * The formatting helpers in App/Inc/vmc_rtt_format.h replaced the per-field
 * snprintf calls in the control loop.  The telemetry wire format must not
 * change, so this test re-derives the old snprintf output for a wide sweep of
 * values and compares it byte for byte against the new helpers.
 *
 * Build and run (executed automatically by tools/test_rtt_format.py):
 *   cc -std=c11 -O2 -IApp/Inc -o /tmp/tools_test_rtt_format \
 *      tools/test_rtt_format.c -lm
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vmc_rtt_format.h"

static int failures;
static int checks;

static void fail(const char *what, double value, const char *expected,
                 const char *actual)
{
  if (failures < 20) {
    printf("FAIL %s value=%.17g\n  expected \"%s\"\n  actual   \"%s\"\n",
           what, value, expected, actual);
  }
  failures++;
}

/* Reference encoder: the exact snprintf call the helper replaced. */
static void reference_scaled(char *out, double value)
{
  const long scaled = (long)(value * (double)VMC_LQR_RTT_TELEM_SCALE);
  const int negative = (scaled < 0L || (value < 0.0 && scaled == 0L)) ? 1 : 0;
  long magnitude = scaled;
  long whole;
  long frac;

  if (magnitude < 0L) {
    magnitude = -magnitude;
  } else if (negative != 0) {
    magnitude = 1L;
  }
  whole = magnitude / VMC_LQR_RTT_TELEM_SCALE;
  frac = magnitude % VMC_LQR_RTT_TELEM_SCALE;
  snprintf(out, 64, negative != 0 ? ",-%ld.%03ld" : ",%ld.%03ld", whole, frac);
}

static void check_scaled(double value)
{
  char expected[64];
  char actual[64];
  int offset;

  reference_scaled(expected, value);
  offset = vmc_lqr_rtt_append_scaled(actual, 0, (int)sizeof(actual), value);
  actual[offset] = '\0';
  checks++;
  if (strcmp(expected, actual) != 0) {
    fail("scaled", value, expected, actual);
  }
}

static void check_u32(uint32_t value)
{
  char expected[64];
  char actual[64];
  int offset;

  snprintf(expected, sizeof(expected), ",%lu", (unsigned long)value);
  offset = vmc_lqr_rtt_append_u32(actual, 0, (int)sizeof(actual), value);
  actual[offset] = '\0';
  checks++;
  if (strcmp(expected, actual) != 0) {
    fail("u32", (double)value, expected, actual);
  }
}

static void check_qr(double value)
{
  char expected[64];
  char actual[64];
  long long scaled;
  unsigned long long magnitude;
  unsigned long long whole;
  unsigned long long frac;
  int negative;
  int offset;
  double clamped = value;

  if (clamped > 9.0e9) {
    clamped = 9.0e9;
  } else if (clamped < -9.0e9) {
    clamped = -9.0e9;
  }
  scaled = (long long)(clamped * (double)VMC_LQR_RTT_QR_SCALE);
  negative = (scaled < 0LL || (clamped < 0.0 && scaled == 0LL)) ? 1 : 0;
  magnitude = (scaled < 0LL)
                  ? (unsigned long long)(-(scaled + 1LL)) + 1ULL
                  : (unsigned long long)scaled;
  whole = magnitude / (unsigned long long)VMC_LQR_RTT_QR_SCALE;
  frac = magnitude % (unsigned long long)VMC_LQR_RTT_QR_SCALE;

  {
    char body[48];
    snprintf(body, sizeof(body), "%llu.%09llu", whole, frac);
    snprintf(expected, sizeof(expected), negative != 0 ? ",-%s" : ",%s", body);
  }
  offset = vmc_lqr_rtt_append_qr(actual, 0, (int)sizeof(actual), value);
  actual[offset] = '\0';
  checks++;
  if (strcmp(expected, actual) != 0) {
    fail("qr", value, expected, actual);
  }
}

int main(void)
{
  /* Dense sweep over magnitudes that appear in the telemetry. */
  static const double magnitudes[] = {
      0.0, 1e-7, 5e-4, 1e-3, 0.1234, 1.0, 12.3456, 1234.5678, 1e5, 2.1e9};
  size_t i;
  int step;
  int sign;

  for (i = 0; i < sizeof(magnitudes) / sizeof(magnitudes[0]); i++) {
    for (sign = 0; sign < 2; sign++) {
      double base = sign ? -magnitudes[i] : magnitudes[i];
      for (step = -3; step <= 3; step++) {
        check_scaled(base + (double)step * 0.000999);
        check_scaled(base + (double)step * 0.4999);
      }
      check_scaled(base);
    }
  }
  /* Field ranges used by the burst payload. */
  for (step = -40; step <= 40; step++) {
    check_scaled((double)step * 0.07);
    check_scaled((double)step * 1234.5);
  }

  check_u32(0U);
  check_u32(1U);
  check_u32(4294967295U);
  check_u32(1677613U);
  for (step = 0; step < 200; step++) {
    check_u32((uint32_t)(step * 7919U));
  }

  check_qr(0.0);
  check_qr(4e-7);
  check_qr(2.072e-5);
  check_qr(1.11759e-4);
  check_qr(8.3363e-3);
  check_qr(1.0);
  check_qr(12345.6789);
  check_qr(9.0e9);
  check_qr(-9.0e9);
  check_qr(1e12);
  check_qr(-1e12);
  for (step = -50; step <= 50; step++) {
    check_qr((double)step * 1.7e-6);
    check_qr((double)step * 0.37);
  }

  /* int16 quantiser sanity: saturation and rounding. */
  checks++;
  if (vmc_rtt_scale_i16(3.3, 1000) != 3300) {
    fail("scale_i16 round", 3.3, "3300", "?");
  }
  checks++;
  if (vmc_rtt_scale_i16(-3.3, 1000) != -3300) {
    fail("scale_i16 round neg", -3.3, "-3300", "?");
  }
  checks++;
  if (vmc_rtt_scale_i16(1e9, 1000) != 32767) {
    fail("scale_i16 sat hi", 1e9, "32767", "?");
  }
  checks++;
  if (vmc_rtt_scale_i16(-1e9, 1000) != -32768) {
    fail("scale_i16 sat lo", -1e9, "-32768", "?");
  }

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
