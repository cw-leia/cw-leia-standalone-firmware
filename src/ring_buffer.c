#include "ring_buffer.h"

void init_ring_buffer(volatile struct s_ring* ring_buffer)
{
  /* init flags */
  int i = 0;
  ring_buffer->end = 0;
  ring_buffer->start = ring_buffer->end;
  ring_buffer->full = false;
  ring_buffer->cmd = false;

  /* memsetting buffer
   */
  for (i = 0; i < RINGBUF_MAX; i++) {
    ring_buffer->buf[i] = '\0';
  }
}


