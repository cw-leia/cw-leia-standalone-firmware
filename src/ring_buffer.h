#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#define RINGBUF_SIZE        4096
#define RINGBUF_MAX         (RINGBUF_SIZE - 1)

#include "helpers.h"

typedef enum { false, true } bool;
struct s_ring {
  unsigned int start;
  unsigned int end;
  bool     full;
  bool     cmd;
  char buf[RINGBUF_SIZE];
};
typedef struct s_ring s_ring_t;

static inline int ring_buffer_write_char(volatile struct s_ring*, const char);
static inline int ring_buffer_read_char(volatile struct s_ring*, char*);

void init_ring_buffer(volatile struct s_ring* ring_buffer);


static inline int ring_buffer_write_char(volatile struct s_ring* ring_buffer, const char c)
{
  /* if the ring buffer is full when we try to put char in it,
   * the car is discared, waiting for the ring buffer to be flushed.
   */
  if (ring_buffer->full) {
    goto err;
  }

  ring_buffer->buf[ring_buffer->end] = c;
  if (((ring_buffer->end + 1) % RINGBUF_MAX) != ring_buffer->start) {
    ring_buffer->end++;
    ring_buffer->end %= RINGBUF_MAX;
  } else {
    /* full buffer detection */
    ring_buffer->full = true;
  }

  return 0;

 err:
  return -1;
}

static inline int ring_buffer_read_char(volatile struct s_ring* ring_buffer, char *c)
{
  if(c == NULL){
    goto err;
  }
  
  /* Is ring buffer empty? */
  if(ring_buffer->start == ring_buffer->end){
    goto err;
  }
  *c = ring_buffer->buf[ring_buffer->start];
  ring_buffer->start = (ring_buffer->start + 1) % RINGBUF_MAX;
  ring_buffer->full = false;
  if(ring_buffer->start == ring_buffer->end){
    ring_buffer->cmd = false;
  }

  return 0;

 err:
  return -1;
}


#endif /* __RINGBUFFER_H__ */
