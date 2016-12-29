/*
 * fifo.h
 *
 *  Created on: 24.12.2016
 *      Author: gda
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <string.h>

#define FIFO_CREATE(_name, _fifo_size, _entry_size) \
struct {                                            \
    uint8_t _read;                                  \
    uint8_t _write;                                 \
    uint8_t _size;                                  \
    struct {                                        \
        uint8_t len;                                \
        uint8_t _buffer[(_entry_size)];             \
    } _entry[(_fifo_size)];                         \
} (_name) = {0, 0, (_fifo_size)};

#define FIFO_POP(_name, _data, _len) {                                                              \
    if ((_name)._read != (_name)._write) {                                                          \
        (_name)._read = ((_name)._read + 1) % (_name).size;                                         \
        (_len) = (_name)._entry[(_name)._read].len;                                                 \
        memcpy((_data), (_name)._entry[(_name)._read].buffer, (_name)._entry[(_name)._read].len);   \
    } else {                                                                                        \
        (_len) = 0;                                                                                 \
        *(_data) = 0;                                                                               \
    }                                                                                               \
}

#define FIFO_PUSH(_name, _data, _len) {                                                         \
    uint8_t tmphead = ((_name)._write + 1 ) % (_name).size;     /* calculate buffer index */    \
    if (tmphead != (_name)._read) {                             /* if buffer is not full */     \
       memcpy((_name)._entry[tmphead].buffer, (_data), (_len)); /* store data in buffer */      \
       (_name)._write = tmphead;                                /* store new index */           \
    }                                                                                           \
}

#endif /* FIFO_H_ */
