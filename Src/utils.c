/*
 * utils.c
 *
 *  Created on: 23.12.2016
 *      Author: gda
 */

#include <ctype.h>

#include "utils.h"

const char hex_asc[] = "0123456789abcdef";

/**
 * hex_to_bin - convert a hex digit to its real value
 * @ch: ascii character represents hex digit
 *
 * hex_to_bin() converts one hex digit to its actual value or -1 in case of bad
 * input.
 */
int hex_to_bin(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
        return ch - '0';
    ch = tolower(ch);
    if ((ch >= 'a') && (ch <= 'f'))
        return ch - 'a' + 10;
    return -1;
}

/**
 * hex2bin - convert an ascii hexadecimal string to its binary representation
 * @dst: binary result
 * @src: ascii hexadecimal string
 * @count: result length
 *
 * Return 0 on success, -1 in case of bad input.
 */
int hex2bin(uint8_t *dst, const char *src, size_t count)
{
    while (count--)
    {
        int hi = hex_to_bin(*src++);
        int lo = hex_to_bin(*src++);

        if ((hi < 0) || (lo < 0))
            return -1;

        *dst++ = (hi << 4) | lo;
    }
    return 0;
}

char *bin2hex(char *dst, const void *src, size_t count)
{
        const unsigned char *_src = src;

        while (count--)
                dst = hex_byte_pack(dst, *_src++);
        return dst;
}
