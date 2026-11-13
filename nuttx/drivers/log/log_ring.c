/* SPDX-License-Identifier: Apache-2.0 */
/* See log_ring.h.  No OS calls: the host tests build this file as is. */

#include "log_ring.h"

#include <string.h>

/* Zephyr's severity[] (subsys/logging/log_output.c). */

static const char *const severity[] =
{
  NULL,
  "err",
  "wrn",
  "inf",
  "dbg"
};

void log_ring_validate(struct retained_log *log)
{
  if (log->magic != LOG_RAM_MAGIC || log->magic_inverse != ~LOG_RAM_MAGIC ||
      log->start >= LOG_RAM_SIZE || log->length > LOG_RAM_SIZE)
    {
      log->start = 0;
      log->length = 0;
      log->magic_inverse = ~LOG_RAM_MAGIC;
      log->magic = LOG_RAM_MAGIC;
    }
}

void log_ring_store(struct retained_log *log, uint64_t *dropped,
                    const char *data, size_t length)
{
  /* The same result as Zephyr's byte loop (push the oldest byte out when
   * full, then append), done with at most two copies.
   */

  size_t over;
  size_t end;
  size_t first;

  if (length > LOG_RAM_SIZE)
    {
      /* Only the last LOG_RAM_SIZE bytes stay, where the byte loop would
       * have put them; everything before passes through.
       */

      size_t skip = length - LOG_RAM_SIZE;

      *dropped += log->length + skip;
      log->start = (log->start + log->length + skip % LOG_RAM_SIZE) %
                   LOG_RAM_SIZE;
      log->length = 0;
      data += skip;
      length = LOG_RAM_SIZE;
    }

  over = log->length + length > LOG_RAM_SIZE ?
         log->length + length - LOG_RAM_SIZE : 0;
  if (over > 0)
    {
      log->start = (log->start + over) % LOG_RAM_SIZE;
      log->length -= over;
      *dropped += over;
    }

  end = (log->start + log->length) % LOG_RAM_SIZE;
  first = LOG_RAM_SIZE - end < length ? LOG_RAM_SIZE - end : length;
  memcpy(log->buffer + end, data, first);
  memcpy(log->buffer, data + first, length - first);
  log->length += length;
}

size_t log_ring_piece(const struct retained_log *log, uint64_t dropped,
                      uint64_t *position, bool *first, char *piece,
                      size_t max)
{
  size_t available;
  size_t n;
  size_t k;

  if (*first || *position < dropped)
    {
      *position = dropped;  /* start, or skip what was overwritten */
      *first = false;
    }

  available = (size_t)(dropped + log->length - *position);
  n = available < max ? available : max;
  for (k = 0; k < n; k++)
    {
      piece[k] = log->buffer[(log->start + (size_t)(*position - dropped) +
                              k) % LOG_RAM_SIZE];
    }

  return n;
}

/* Append n as decimal with at least two digits. */

static char *put_uint(char *p, uint64_t n, int width)
{
  char digits[20];
  int count = 0;

  do
    {
      uint64_t q = n / 10U;

      digits[count++] = (char)('0' + (n - q * 10U));
      n = q;
    }
  while (n != 0);

  while (count < width)
    {
      digits[count++] = '0';
    }

  while (count > 0)
    {
      *p++ = digits[--count];
    }

  return p;
}

/* Append a 32-bit value below 1000 with exactly the given width. */

static char *put_small(char *p, uint32_t n, int width)
{
  if (width == 3)
    {
      *p++ = (char)('0' + n / 100U);
      n %= 100U;
    }

  *p++ = (char)('0' + n / 10U);
  *p++ = (char)('0' + n % 10U);
  return p;
}

static char *put_str(char *p, const char *end, const char *s)
{
  while (*s != '\0' && p < end)
    {
      *p++ = *s++;
    }

  return p;
}

int log_ring_prefix(char *buf, size_t size, uint64_t us, int level,
                    const char *module, const char *func)
{
  /* "[" + up to 20 digits + ":MM:SS.mmm,uuu] " + "<lvl> " */

  char head[48];
  char *p = head;
  uint64_t total_seconds;
  uint32_t remainder;
  uint32_t ms;
  uint32_t usec;
  uint64_t hours;
  uint32_t seconds;
  uint32_t mins;
  size_t length;
  const char *lvl = (level >= 1 && level <= 4) ? severity[level] : "";

  /* The 64-bit division is slow on the Cortex-M0+ (libgcc); below 2^32 us
   * (71 minutes) 32-bit arithmetic gives the same values.
   */

  if (us <= UINT32_MAX)
    {
      uint32_t u = (uint32_t)us;
      uint32_t secs = u / 1000000U;

      remainder = u - secs * 1000000U;
      total_seconds = secs;
      hours = secs / 3600U;
      seconds = secs - (uint32_t)hours * 3600U;
    }
  else
    {
      total_seconds = us / 1000000U;
      remainder = (uint32_t)(us - total_seconds * 1000000U);
      hours = total_seconds / 3600U;
      seconds = (uint32_t)(total_seconds - hours * 3600U);
    }

  ms = remainder / 1000U;
  usec = remainder - ms * 1000U;
  mins = seconds / 60U;
  seconds -= mins * 60U;

  *p++ = '[';
  p = put_uint(p, hours, 2);
  *p++ = ':';
  p = put_small(p, mins, 2);
  *p++ = ':';
  p = put_small(p, seconds, 2);
  *p++ = '.';
  p = put_small(p, ms, 3);
  *p++ = ',';
  p = put_small(p, usec, 3);
  *p++ = ']';
  *p++ = ' ';
  *p++ = '<';
  p = put_str(p, head + sizeof(head), lvl);
  *p++ = '>';
  *p++ = ' ';

  /* Then "module: " or "module.func: ", cut to the buffer like snprintf. */

  length = (size_t)(p - head);
  {
    size_t total = length + strlen(module) + 2 +
                   (func != NULL ? strlen(func) + 1 : 0);
    char *out = buf;
    char *end = size > 0 ? buf + size - 1 : buf;

    for (size_t i = 0; i < length && out < end; i++)
      {
        *out++ = head[i];
      }

    out = put_str(out, end, module);
    if (func != NULL)
      {
        out = put_str(out, end, ".");
        out = put_str(out, end, func);
      }

    out = put_str(out, end, ": ");
    if (size > 0)
      {
        *out = '\0';
      }

    return (int)total;
  }
}

bool log_ring_ratelimit(uint32_t *last_ms, uint32_t *skipped,
                        uint32_t now_ms, uint32_t rate_ms,
                        uint32_t *skipped_out)
{
  uint32_t last = *last_ms;

  if (last == 0 || now_ms - last >= rate_ms)
    {
      *last_ms = now_ms;
      *skipped_out = *skipped;
      *skipped = 0;
      return true;
    }

  (*skipped)++;
  return false;
}
