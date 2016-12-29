/*
 * utils.h
 *
 *  Created on: 23.12.2016
 *      Author: gda
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stddef.h>

extern const char hex_asc[];

#define hex_asc_lo(x)   hex_asc[((x) & 0x0f)]
#define hex_asc_hi(x)   hex_asc[((x) & 0xf0) >> 4]

static inline char *hex_byte_pack(char *buf, uint8_t byte)
{
    *buf++ = hex_asc_hi(byte);
    *buf++ = hex_asc_lo(byte);
    return buf;
}

extern int hex_to_bin(char ch);
extern int hex2bin(uint8_t *dst, const char *src, size_t count);
extern char *bin2hex(char *dst, const void *src, size_t count);

#endif /* UTILS_H_ */
