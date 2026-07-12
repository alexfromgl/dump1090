/* Mode1090, a Mode S messages decoder for RTLSDR devices.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Web-UI-only build derived from dump1090.
 *
 * Fixed configuration:
 *   RTL-SDR device: 0
 *   Frequency:      1090 MHz
 *   Sample rate:    2 MHz
 *   Gain:           maximum available
 *   HTTP port:      8081
 *   Units:          metric in /data.json
 *
 * Run with no command-line arguments. The program serves gmap.html from
 * its current working directory and aircraft data at /data.json.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "rtl-sdr.h"

#define MODES_DEFAULT_RATE         2000000
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_ASYNC_BUF_NUMBER     12
#define MODES_DATA_LEN             (16*16384)
#define MODES_FULL_LEN             (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
#define MODES_PREAMBLE_US          8
#define MODES_LONG_MSG_BITS        112
#define MODES_SHORT_MSG_BITS       56
#define MODES_LONG_MSG_BYTES       (MODES_LONG_MSG_BITS/8)
#define MODES_ICAO_CACHE_LEN       1024
#define MODES_ICAO_CACHE_TTL       60
#define MODES_UNIT_FEET            0
#define MODES_UNIT_METERS          1
#define MODES_MAX_BITERRORS        2
#define NERRORINFO                 5778
#define MODES_INTERACTIVE_TTL      60
#define MODES_NET_MAX_FD           1024
#define MODES_NET_HTTP_PORT        8081
#define MODES_CLIENT_BUF_SIZE      4096
#define MODES_NOTUSED(V)           ((void)(V))

struct client {
    int fd;
    char buf[MODES_CLIENT_BUF_SIZE + 1];
    int buflen;
};

struct aircraft {
    uint32_t addr;
    char hexaddr[7];
    char flight[9];
    int altitude;
    int speed;
    int track;
    time_t seen;
    long messages;
    int odd_cprlat;
    int odd_cprlon;
    int even_cprlat;
    int even_cprlon;
    double lat;
    double lon;
    long long odd_cprtime;
    long long even_cprtime;
    struct aircraft *next;
};

struct {
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;
    pthread_cond_t data_cond;
    unsigned char *data;
    uint16_t *magnitude;
    uint32_t data_len;
    int data_ready;
    uint32_t *icao_cache;
    uint16_t *maglut;

    rtlsdr_dev_t *dev;

    struct client *clients[MODES_NET_MAX_FD];
    int maxfd;
    int https;

    struct aircraft *aircrafts;

    double ref_lat;
    double ref_lon;
    int ref_count;
} Modes;

struct modesMessage {
    unsigned char msg[MODES_LONG_MSG_BYTES];
    int msgbits;
    int msgtype;
    int crcok;
    uint32_t crc;
    int errorbit;
    int aa1, aa2, aa3;
    int phase_corrected;
    int ca;
    int iid;
    int metype;
    int mesub;
    int heading_is_valid;
    int heading;
    int aircraft_type;
    int fflag;
    int tflag;
    int raw_latitude;
    int raw_longitude;
    char flight[9];
    int ew_dir;
    int ew_velocity;
    int ns_dir;
    int ns_velocity;
    int vert_rate_source;
    int vert_rate_sign;
    int vert_rate;
    int velocity;
    int movement;
    int movement_valid;
    int ground_track;
    int ground_track_valid;
    int fs;
    int dr;
    int um;
    int identity;
    int altitude;
    int unit;
};

struct errorinfo {
    uint32_t syndrome;
    int bits;
    int pos[MODES_MAX_BITERRORS];
};

static struct errorinfo bitErrorTable[NERRORINFO];

static void modesInitErrorInfo(void);
static int modesMessageLenByType(int type);
static int fixBitErrors(unsigned char *msg, int bits, int maxfix, int *fixedbits);
static void useModesMessage(struct modesMessage *mm);
static struct aircraft *interactiveReceiveData(struct modesMessage *mm);
static void decodeCPRSurface(struct aircraft *a, int fflag, int raw_lat, int raw_lon);
static int decodeMovementField(int movement);

static long long mstime(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((long long)tv.tv_sec * 1000) + tv.tv_usec / 1000;
}

static void die(const char *message) {
    perror(message);
    exit(EXIT_FAILURE);
}

static void modesInit(void) {
    int i, q;

    pthread_mutex_init(&Modes.data_mutex, NULL);
    pthread_cond_init(&Modes.data_cond, NULL);

    Modes.data_len = MODES_DATA_LEN + (MODES_FULL_LEN - 1) * 4;
    Modes.icao_cache = calloc(MODES_ICAO_CACHE_LEN * 2, sizeof(uint32_t));
    Modes.data = malloc(Modes.data_len);
    Modes.magnitude = malloc(Modes.data_len * sizeof(uint16_t));
    Modes.maglut = malloc(129 * 129 * sizeof(uint16_t));

    if (!Modes.icao_cache || !Modes.data || !Modes.magnitude || !Modes.maglut) {
        fprintf(stderr, "Out of memory.\n");
        exit(EXIT_FAILURE);
    }

    memset(Modes.data, 127, Modes.data_len);
    for (i = 0; i <= 128; i++) {
        for (q = 0; q <= 128; q++) {
            Modes.maglut[i * 129 + q] = (uint16_t)round(sqrt(i*i + q*q) * 360);
        }
    }

    modesInitErrorInfo();
}

static void modesInitRTLSDR(void) {
    uint32_t device_count = rtlsdr_get_device_count();
    char vendor[256], product[256], serial[256];
    int gains[100];
    int numgains;

    if (device_count == 0) {
        fprintf(stderr, "No supported RTL-SDR devices found.\n");
        exit(EXIT_FAILURE);
    }

    rtlsdr_get_device_usb_strings(0, vendor, product, serial);
    fprintf(stderr, "Using RTL-SDR device 0: %s, %s, SN: %s\n", vendor, product, serial);

    if (rtlsdr_open(&Modes.dev, 0) < 0) {
        fprintf(stderr, "Unable to open RTL-SDR device 0.\n");
        exit(EXIT_FAILURE);
    }

    rtlsdr_set_tuner_gain_mode(Modes.dev, 1);
    numgains = rtlsdr_get_tuner_gains(Modes.dev, gains);
    if (numgains > 0) {
        int gain = gains[numgains - 1];
        rtlsdr_set_tuner_gain(Modes.dev, gain);
        fprintf(stderr, "Gain: %.1f dB\n", gain / 10.0);
    }

    rtlsdr_set_freq_correction(Modes.dev, 0);
    rtlsdr_set_center_freq(Modes.dev, MODES_DEFAULT_FREQ);
    rtlsdr_set_sample_rate(Modes.dev, MODES_DEFAULT_RATE);
    rtlsdr_reset_buffer(Modes.dev);
}

static void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {
    MODES_NOTUSED(ctx);
    pthread_mutex_lock(&Modes.data_mutex);
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;
    memcpy(Modes.data, Modes.data + MODES_DATA_LEN, (MODES_FULL_LEN - 1) * 4);
    memcpy(Modes.data + (MODES_FULL_LEN - 1) * 4, buf, len);
    Modes.data_ready = 1;
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
}

static void *readerThreadEntryPoint(void *arg) {
    MODES_NOTUSED(arg);
    rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                      MODES_ASYNC_BUF_NUMBER, MODES_DATA_LEN);
    return NULL;
}

/* Parity table for MODE S Messages.
 * The table contains 112 elements, every element corresponds to a bit set
 * in the message, starting from the first bit of actual data after the
 * preamble.
 *
 * For messages of 112 bit, the whole table is used.
 * For messages of 56 bits only the last 56 elements are used.
 *
 * The algorithm is as simple as xoring all the elements in this table
 * for which the corresponding bit on the message is set to 1.
 *
 * The latest 24 elements in this table are set to 0 as the checksum at the
 * end of the message should not affect the computation.
 *
 * Note: this function can be used with DF11 and DF17, other modes have
 * the CRC xored with the sender address as they are reply to interrogations,
 * but a casual listener can't split the address from the checksum.
 */
uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

/* Compute the CRC of a Mode S message (data portion only).
 * This returns just the CRC computed from the data bits, without XORing
 * with the transmitted CRC/AP field. Used by bruteForceAP. */
static uint32_t modesComputeCRC(unsigned char *msg, int bits) {
    uint32_t crc = 0;
    int offset = (bits == 112) ? 0 : (112-56);
    int j;

    /* Compute CRC of data portion (exclude last 24 CRC bits). */
    for(j = 0; j < bits - 24; j++) {
        int byte = j/8;
        int bit = j%8;
        int bitmask = 1 << (7-bit);

        /* If bit is set, xor with corresponding table entry. */
        if (msg[byte] & bitmask)
            crc ^= modes_checksum_table[j+offset];
    }
    return crc & 0x00FFFFFF;
}

/* Compute the CRC syndrome of a Mode S message.
 *
 * This function computes the CRC of the data portion (excluding the last
 * 24 bits which are the transmitted CRC), then XORs with the transmitted
 * CRC. The result is the "syndrome":
 *
 *   - For a valid message: syndrome = 0
 *   - For a corrupted message: syndrome = XOR of table entries for flipped bits
 *
 * This property makes syndrome-based error correction possible: we can
 * precompute syndromes for all possible single/double bit errors and look
 * them up in a table. */
static uint32_t modesChecksum(unsigned char *msg, int bits) {
    uint32_t crc = modesComputeCRC(msg, bits);
    uint32_t rem;

    /* XOR with the transmitted CRC (last 3 bytes) to get the syndrome. */
    rem = ((uint32_t)msg[(bits/8)-3] << 16) |
          ((uint32_t)msg[(bits/8)-2] << 8) |
           (uint32_t)msg[(bits/8)-1];
    return (crc ^ rem) & 0x00FFFFFF; /* 24 bit syndrome. */
}

/* Given the Downlink Format (DF) of the message, return the message length
 * in bits. */
static int modesMessageLenByType(int type) {
    if (type == 16 || type == 17 ||
        type == 18 || type == 19 ||
        type == 20 || type == 21)
        return MODES_LONG_MSG_BITS;
    else
        return MODES_SHORT_MSG_BITS;
}

/* Comparison function for qsort/bsearch on errorinfo by syndrome. */
static int cmpErrorInfo(const void *a, const void *b) {
    const struct errorinfo *ea = (const struct errorinfo *)a;
    const struct errorinfo *eb = (const struct errorinfo *)b;
    if (ea->syndrome < eb->syndrome) return -1;
    if (ea->syndrome > eb->syndrome) return 1;
    return 0;
}

/* ===================== Error correction via syndrome table ================
 *
 * What is a syndrome?
 * -------------------
 * In CRC-based error detection, a "syndrome" is the CRC computed on a
 * received message. For a valid message, the syndrome is zero (or matches
 * the expected CRC). When bits get corrupted during transmission, the
 * syndrome becomes non-zero.
 *
 * The key insight is that the syndrome depends ONLY on which bits were
 * flipped, not on the message content itself. If we flip bit X in any
 * Mode S message, we always get the same syndrome value S_x. This property
 * comes from the linearity of CRC (it's based on polynomial arithmetic
 * over GF(2) - basically XOR operations).
 *
 * For two-bit errors, flipping bits X and Y produces syndrome S_x XOR S_y.
 *
 * So we can precompute a lookup table:
 *   - For each single bit position X, compute S_x
 *   - For each pair of bit positions X,Y, compute S_x XOR S_y
 *
 * When we receive a message with a non-zero syndrome S, we search the table
 * for S. If found, we know exactly which bit(s) to flip to fix the message.
 *
 * This turns O(n) or O(n^2) brute force error correction into O(log n)
 * binary search, making it much faster for real-time decoding.
 * ========================================================================= */

/* Compute the table of all syndromes for 1-bit and 2-bit error vectors.
 * We don't include the first 5 bits (the DF type field) since corrupting
 * those would change the message type and length, making correction risky. */
static void modesInitErrorInfo(void) {
    unsigned char msg[MODES_LONG_MSG_BYTES];
    int i, j, n;
    uint32_t crc;

    n = 0;
    memset(bitErrorTable, 0, sizeof(bitErrorTable));
    memset(msg, 0, MODES_LONG_MSG_BYTES);

    /* Add all possible single and double bit errors.
     * Don't include errors in first 5 bits (DF type). */
    for (i = 5; i < MODES_LONG_MSG_BITS; i++) {
        int bytepos0 = (i >> 3);
        int mask0 = 1 << (7 - (i & 7));
        msg[bytepos0] ^= mask0;  /* Create error at bit i. */
        crc = modesChecksum(msg, MODES_LONG_MSG_BITS);

        /* Single bit error case. */
        bitErrorTable[n].syndrome = crc;
        bitErrorTable[n].bits = 1;
        bitErrorTable[n].pos[0] = i;
        bitErrorTable[n].pos[1] = -1;
        n++;

        /* Two-bit error cases. */
        for (j = i + 1; j < MODES_LONG_MSG_BITS; j++) {
            int bytepos1 = (j >> 3);
            int mask1 = 1 << (7 - (j & 7));
            msg[bytepos1] ^= mask1;  /* Create error at bit j. */
            crc = modesChecksum(msg, MODES_LONG_MSG_BITS);

            if (n >= NERRORINFO) break;

            bitErrorTable[n].syndrome = crc;
            bitErrorTable[n].bits = 2;
            bitErrorTable[n].pos[0] = i;
            bitErrorTable[n].pos[1] = j;
            n++;

            msg[bytepos1] ^= mask1;  /* Revert error at bit j. */
        }
        msg[bytepos0] ^= mask0;  /* Revert error at bit i. */
    }

    /* Sort by syndrome for binary search. */
    qsort(bitErrorTable, NERRORINFO, sizeof(struct errorinfo), cmpErrorInfo);
}

/* Given a received message, compute its syndrome and look it up in the
 * precomputed table. If we find a match, we know the exact bit positions
 * that were corrupted during transmission, and we can flip them back.
 *
 * The 'maxfix' parameter limits how many bits we're willing to correct:
 * fixing 1 bit is quite safe, but 2-bit correction has a higher chance
 * of "fixing" a random message into a valid but wrong one (birthday paradox).
 *
 * Returns the number of bits corrected (0 if syndrome not found or would
 * require too many bit fixes). If 'fixedbits' is not NULL, stores the
 * corrected bit positions there. */
static int fixBitErrors(unsigned char *msg, int bits, int maxfix, int *fixedbits) {
    struct errorinfo *pei;
    struct errorinfo ei;
    int bitpos, offset, i, res;

    memset(&ei, 0, sizeof(struct errorinfo));
    ei.syndrome = modesChecksum(msg, bits);

    pei = bsearch(&ei, bitErrorTable, NERRORINFO,
                  sizeof(struct errorinfo), cmpErrorInfo);
    if (pei == NULL) {
        return 0;  /* No matching syndrome found. */
    }

    /* Check if syndrome fixes more bits than allowed. */
    if (pei->bits > maxfix) {
        return 0;
    }

    /* Check that all bit positions lie inside the message length. */
    offset = MODES_LONG_MSG_BITS - bits;
    for (i = 0; i < pei->bits; i++) {
        bitpos = pei->pos[i] - offset;
        if ((bitpos < 0) || (bitpos >= bits)) {
            return 0;
        }
    }

    /* Fix the bits. */
    res = 0;
    for (i = 0; i < pei->bits; i++) {
        bitpos = pei->pos[i] - offset;
        msg[bitpos >> 3] ^= (1 << (7 - (bitpos & 7)));
        if (fixedbits) {
            fixedbits[res++] = bitpos;
        } else {
            res++;
        }
    }
    return res;
}

/* Hash the ICAO address to index our cache of MODES_ICAO_CACHE_LEN
 * elements, that is assumed to be a power of two. */
static uint32_t ICAOCacheHashAddress(uint32_t a) {
    /* The following three rounds wil make sure that every bit affects
     * every output bit with ~ 50% of probability. */
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODES_ICAO_CACHE_LEN-1);
}

/* Add the specified entry to the cache of recently seen ICAO addresses.
 * Note that we also add a timestamp so that we can make sure that the
 * entry is only valid for MODES_ICAO_CACHE_TTL seconds. */
static void addRecentlySeenICAOAddr(uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    Modes.icao_cache[h*2] = addr;
    Modes.icao_cache[h*2+1] = (uint32_t) time(NULL);
}

/* Returns 1 if the specified ICAO address was seen in a DF format with
 * proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
 * seconds ago. Otherwise returns 0. */
static int ICAOAddressWasRecentlySeen(uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    uint32_t a = Modes.icao_cache[h*2];
    uint32_t t = Modes.icao_cache[h*2+1];

    return a && a == addr && time(NULL)-t <= MODES_ICAO_CACHE_TTL;
}

/* If the message type has the checksum xored with the ICAO address, try to
 * brute force it using a list of recently seen ICAO addresses.
 *
 * Do this in a brute-force fashion by xoring the predicted CRC with
 * the address XOR checksum field in the message. This will recover the
 * address: if we found it in our cache, we can assume the message is ok.
 *
 * This function expects mm->msgtype and mm->msgbits to be correctly
 * populated by the caller.
 *
 * On success the correct ICAO address is stored in the modesMessage
 * structure in the aa3, aa2, and aa1 fiedls.
 *
 * If the function successfully recovers a message with a correct checksum
 * it returns 1. Otherwise 0 is returned. */
static int bruteForceAP(unsigned char *msg, struct modesMessage *mm) {
    unsigned char aux[MODES_LONG_MSG_BYTES];
    int msgtype = mm->msgtype;
    int msgbits = mm->msgbits;

    if (msgtype == 0 ||         /* Short air surveillance */
        msgtype == 4 ||         /* Surveillance, altitude reply */
        msgtype == 5 ||         /* Surveillance, identity reply */
        msgtype == 16 ||        /* Long Air-Air survillance */
        msgtype == 20 ||        /* Comm-A, altitude request */
        msgtype == 21 ||        /* Comm-A, identity request */
        msgtype == 24)          /* Comm-C ELM */
    {
        uint32_t addr;
        uint32_t crc;
        int lastbyte = (msgbits/8)-1;

        /* Work on a copy. */
        memcpy(aux,msg,msgbits/8);

        /* Compute the CRC of the message and XOR it with the AP field
         * so that we recover the address, because:
         *
         * (ADDR xor CRC) xor CRC = ADDR.
         * We use modesComputeCRC (not modesChecksum) to get just the CRC. */
        crc = modesComputeCRC(aux,msgbits);
        aux[lastbyte] ^= crc & 0xff;
        aux[lastbyte-1] ^= (crc >> 8) & 0xff;
        aux[lastbyte-2] ^= (crc >> 16) & 0xff;
        
        /* If the obtained address exists in our cache we consider
         * the message valid. */
        addr = aux[lastbyte] | (aux[lastbyte-1] << 8) | (aux[lastbyte-2] << 16);
        if (ICAOAddressWasRecentlySeen(addr)) {
            mm->aa1 = aux[lastbyte-2];
            mm->aa2 = aux[lastbyte-1];
            mm->aa3 = aux[lastbyte];
            return 1;
        }
    }
    return 0;
}

/* Decode the 13 bit AC altitude field (in DF 20 and others).
 * Returns the altitude, and set 'unit' to either MODES_UNIT_METERS
 * or MDOES_UNIT_FEETS. */
static int decodeAC13Field(unsigned char *msg, int *unit) {
    int m_bit = msg[3] & (1<<6);
    int q_bit = msg[3] & (1<<4);

    if (!m_bit) {
        *unit = MODES_UNIT_FEET;
        if (q_bit) {
            /* N is the 11 bit integer resulting from the removal of bit
             * Q and M */
            int n = ((msg[2]&31)<<6) |
                    ((msg[3]&0x80)>>2) |
                    ((msg[3]&0x20)>>1) |
                     (msg[3]&15);
            /* The final altitude is due to the resulting number multiplied
             * by 25, minus 1000. */
            return n*25-1000;
        } else {
            /* TODO: Implement altitude where Q=0 and M=0 */
        }
    } else {
        *unit = MODES_UNIT_METERS;
        /* TODO: Implement altitude when meter unit is selected. */
    }
    return 0;
}

/* Decode the 12 bit AC altitude field (in DF 17 and others).
 * Returns the altitude or 0 if it can't be decoded. */
static int decodeAC12Field(unsigned char *msg, int *unit) {
    int q_bit = msg[5] & 1;

    if (q_bit) {
        /* N is the 11 bit integer resulting from the removal of bit
         * Q */
        *unit = MODES_UNIT_FEET;
        int n = ((msg[5]>>1)<<4) | ((msg[6]&0xF0) >> 4);
        /* The final altitude is due to the resulting number multiplied
         * by 25, minus 1000. */
        return n*25-1000;
    } else {
        return 0;
    }
}
static void decodeModesMessage(struct modesMessage *mm, unsigned char *msg) {
    char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

    /* Work on our local copy */
    memcpy(mm->msg,msg,MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    /* Get the message type ASAP as other operations depend on this */
    mm->msgtype = msg[0]>>3;    /* Downlink Format */
    mm->msgbits = modesMessageLenByType(mm->msgtype);

    /* Compute the syndrome (CRC XOR transmitted CRC).
     * For a valid message, syndrome = 0. */
    mm->crc = modesChecksum(msg, mm->msgbits);

    /* Check CRC (syndrome == 0 means valid) and fix bit errors using
     * the CRC when possible (DF 11, 17, 18). Use fast table-based lookup. */
    mm->errorbit = -1;  /* No error */
    mm->iid = 0;        /* Interrogator Identifier (used by DF 11) */
    mm->crcok = (mm->crc == 0);

    if (!mm->crcok && 1 &&
        (mm->msgtype == 11 || mm->msgtype == 17 || mm->msgtype == 18))
    {
        int maxfix = 1;
        int fixedbits[MODES_MAX_BITERRORS];
        int nfixed = fixBitErrors(msg, mm->msgbits, maxfix, fixedbits);
        if (nfixed > 0) {
            mm->crc = modesChecksum(msg, mm->msgbits);
            mm->crcok = (mm->crc == 0);
            mm->errorbit = fixedbits[0];
        }
    }

    /* Note that most of the other computation happens *after* we fix
     * the single bit errors, otherwise we would need to recompute the
     * fields again. */
    mm->ca = msg[0] & 7;        /* Responder capabilities. */

    /* ICAO address */
    mm->aa1 = msg[1];
    mm->aa2 = msg[2];
    mm->aa3 = msg[3];

    /* DF 17 type (assuming this is a DF17, otherwise not used) */
    mm->metype = msg[4] >> 3;   /* Extended squitter message type. */
    mm->mesub = msg[4] & 7;     /* Extended squitter message subtype. */

    /* Fields for DF4,5,20,21 */
    mm->fs = msg[0] & 7;        /* Flight status for DF4,5,20,21 */
    mm->dr = msg[1] >> 3 & 31;  /* Request extraction of downlink request. */
    mm->um = ((msg[1] & 7)<<3)| /* Request extraction of downlink request. */
              msg[2]>>5;

    /* In the squawk (identity) field bits are interleaved like that
     * (message bit 20 to bit 32):
     *
     * C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
     *
     * So every group of three bits A, B, C, D represent an integer
     * from 0 to 7.
     *
     * The actual meaning is just 4 octal numbers, but we convert it
     * into a base ten number tha happens to represent the four
     * octal numbers.
     *
     * For more info: http://en.wikipedia.org/wiki/Gillham_code */
    {
        int a,b,c,d;

        a = ((msg[3] & 0x80) >> 5) |
            ((msg[2] & 0x02) >> 0) |
            ((msg[2] & 0x08) >> 3);
        b = ((msg[3] & 0x02) << 1) |
            ((msg[3] & 0x08) >> 2) |
            ((msg[3] & 0x20) >> 5);
        c = ((msg[2] & 0x01) << 2) |
            ((msg[2] & 0x04) >> 1) |
            ((msg[2] & 0x10) >> 4);
        d = ((msg[3] & 0x01) << 2) |
            ((msg[3] & 0x04) >> 1) |
            ((msg[3] & 0x10) >> 4);
        mm->identity = a*1000 + b*100 + c*10 + d;
    }

    /* DF 11, 17 & 18: try to populate our ICAO addresses whitelist.
     * DFs with an AP field (xored addr and crc), try to decode it. */
    if (mm->msgtype != 11 && mm->msgtype != 17 && mm->msgtype != 18) {
        /* Check if we can check the checksum for the Downlink Formats where
         * the checksum is xored with the aircraft ICAO address. We try to
         * brute force it using a list of recently seen aircraft addresses. */
        if (bruteForceAP(msg,mm)) {
            /* We recovered the message, mark the checksum as valid. */
            mm->crcok = 1;
        } else {
            mm->crcok = 0;
        }
    } else {
        /* If this is DF 11, 17 or 18 and the checksum was ok,
         * we can add this address to the list of recently seen
         * addresses. */
        uint32_t addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;
        if (mm->crcok && mm->errorbit == -1) {
            addRecentlySeenICAOAddr(addr);
        }

        /* DF 11: For messages with small CRC residual (<80), treat as
         * valid IID (Interrogator Identifier) if ICAO is known. */
        if (mm->msgtype == 11 && !mm->crcok && mm->crc < 80) {
            if (ICAOAddressWasRecentlySeen(addr)) {
                mm->iid = mm->crc;
                mm->crcok = 1;
            }
        }
    }

    /* Decode 13 bit altitude for DF0, DF4, DF16, DF20 */
    if (mm->msgtype == 0 || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20) {
        mm->altitude = decodeAC13Field(msg, &mm->unit);
    }

    /* Decode extended squitter specific stuff.
     * DF 17 = ADS-B from transponder, DF 18 = ADS-B from non-transponder.
     * Both use the same extended squitter format. */
    if (mm->msgtype == 17 || mm->msgtype == 18) {
        /* Decode the extended squitter message. */

        if (mm->metype >= 1 && mm->metype <= 4) {
            /* Aircraft Identification and Category */
            mm->aircraft_type = mm->metype-1;
            mm->flight[0] = ais_charset[msg[5]>>2];
            mm->flight[1] = ais_charset[((msg[5]&3)<<4)|(msg[6]>>4)];
            mm->flight[2] = ais_charset[((msg[6]&15)<<2)|(msg[7]>>6)];
            mm->flight[3] = ais_charset[msg[7]&63];
            mm->flight[4] = ais_charset[msg[8]>>2];
            mm->flight[5] = ais_charset[((msg[8]&3)<<4)|(msg[9]>>4)];
            mm->flight[6] = ais_charset[((msg[9]&15)<<2)|(msg[10]>>6)];
            mm->flight[7] = ais_charset[msg[10]&63];
            mm->flight[8] = '\0';
        } else if (mm->metype >= 5 && mm->metype <= 8) {
            /* Surface position Message.
             *
             * The message format is:
             * - Type code (TC): 5 bits (value 5-8)
             * - Movement: 7 bits (encoded ground speed)
             * - Ground track status: 1 bit (1 = valid)
             * - Ground track: 7 bits (0-127 -> 0-360 degrees)
             * - Time flag: 1 bit
             * - CPR format: 1 bit (0 = even, 1 = odd)
             * - CPR latitude: 17 bits
             * - CPR longitude: 17 bits */
            mm->movement = ((msg[4] & 0x07) << 4) | (msg[5] >> 4);
            mm->movement_valid = (mm->movement != 0);
            mm->ground_track_valid = (msg[5] >> 3) & 1;
            mm->ground_track = ((msg[5] & 0x07) << 4) | (msg[6] >> 4);
            mm->ground_track = mm->ground_track * 360 / 128;
            mm->fflag = (msg[6] >> 2) & 1;
            mm->tflag = (msg[6] >> 3) & 1;
            mm->raw_latitude = ((msg[6] & 3) << 15) |
                                (msg[7] << 7) |
                                (msg[8] >> 1);
            mm->raw_longitude = ((msg[8] & 1) << 16) |
                                 (msg[9] << 8) |
                                 msg[10];
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            /* Airborne position Message */
            mm->fflag = msg[6] & (1<<2);
            mm->tflag = msg[6] & (1<<3);
            mm->altitude = decodeAC12Field(msg,&mm->unit);
            mm->raw_latitude = ((msg[6] & 3) << 15) |
                                (msg[7] << 7) |
                                (msg[8] >> 1);
            mm->raw_longitude = ((msg[8]&1) << 16) |
                                 (msg[9] << 8) |
                                 msg[10];
        } else if (mm->metype == 19 && mm->mesub >= 1 && mm->mesub <= 4) {
            /* Airborne Velocity Message */
            if (mm->mesub == 1 || mm->mesub == 2) {
                mm->ew_dir = (msg[5]&4) >> 2;
                mm->ew_velocity = ((msg[5]&3) << 8) | msg[6];
                mm->ns_dir = (msg[7]&0x80) >> 7;
                mm->ns_velocity = ((msg[7]&0x7f) << 3) | ((msg[8]&0xe0) >> 5);
                mm->vert_rate_source = (msg[8]&0x10) >> 4;
                mm->vert_rate_sign = (msg[8]&0x8) >> 3;
                mm->vert_rate = ((msg[8]&7) << 6) | ((msg[9]&0xfc) >> 2);
                /* Compute velocity and angle from the two speed
                 * components. */
                mm->velocity = sqrt(mm->ns_velocity*mm->ns_velocity+
                                    mm->ew_velocity*mm->ew_velocity);
                if (mm->velocity) {
                    int ewv = mm->ew_velocity;
                    int nsv = mm->ns_velocity;
                    double heading;

                    if (mm->ew_dir) ewv *= -1;
                    if (mm->ns_dir) nsv *= -1;
                    heading = atan2(ewv,nsv);

                    /* Convert to degrees. */
                    mm->heading = heading * 360 / (M_PI*2);
                    /* We don't want negative values but a 0-360 scale. */
                    if (mm->heading < 0) mm->heading += 360;
                } else {
                    mm->heading = 0;
                }
            } else if (mm->mesub == 3 || mm->mesub == 4) {
                mm->heading_is_valid = msg[5] & (1<<2);
                mm->heading = (360.0/128) * (((msg[5] & 3) << 5) |
                                              (msg[6] >> 3));
            }
        }
    }
    mm->phase_corrected = 0; /* Set to 1 by the caller if needed. */
}
static void computeMagnitudeVector(void) {
    uint16_t *m = Modes.magnitude;
    unsigned char *p = Modes.data;
    uint32_t j;

    /* Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
     * we rescale to the 0-255 range to exploit the full resolution. */
    for (j = 0; j < Modes.data_len; j += 2) {
        int i = p[j]-127;
        int q = p[j+1]-127;

        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j/2] = Modes.maglut[i*129+q];
    }
}

/* Scale a sample value by a fixed-point factor, clamping to avoid overflow.
 * The scale is in 16384ths (so 16384 = 1.0, 32768 = 2.0, etc). */
static uint16_t scaleSample(uint16_t v, uint16_t scale) {
    uint32_t result = (uint32_t)v * scale / 16384;
    return (result > 65535) ? 65535 : (uint16_t)result;
}

/* Phase enhancement algorithm.
 *
 * This function estimates whether we are sampling early or late relative to
 * the signal phase, and by how much, by examining the energy distribution
 * in the preamble. It then corrects each sample based on the decoded bit
 * value and the estimated phase offset.
 *
 * The preamble has known 1-bits at positions 0, 2, 7, 9 and known 0-bits
 * elsewhere. By looking at the energy that "leaks" into adjacent 0-bit
 * positions, we can estimate the phase offset:
 * - Energy at positions -1 and 6 indicates early sampling (we're catching
 *   some of the previous bit's energy)
 * - Energy at positions 3 and 10 indicates late sampling (we're catching
 *   some of the next bit's energy)
 *
 * Once we know the phase direction and magnitude, we walk through the
 * message and adjust each sample based on the bit value we decoded for
 * the adjacent bit.
 *
 * Original algorithm by Oliver Jowett. */
static void applyPhaseCorrection(uint16_t *m) {
    int j;

    /* Measure energy in known 1-bit positions (should be high). */
    uint32_t onTime = m[0] + m[2] + m[7] + m[9];

    /* Measure energy leaking into positions before expected 1-bits.
     * This indicates we're sampling early (catching previous bit). */
    uint32_t early = (m[-1] + m[6]) * 2;

    /* Measure energy leaking into positions after expected 1-bits.
     * This indicates we're sampling late (catching next bit). */
    uint32_t late = (m[3] + m[10]) * 2;

    if (early > late) {
        /* Sampling late: each sample contains energy from the following bit.
         * Scale factor is proportional to how much we're off. */
        uint16_t scaleUp = 16384 + 16384 * early / (early + onTime);
        uint16_t scaleDown = 16384 - 16384 * early / (early + onTime);

        /* Last data sample: trailing bits are 0, so it will be low. Scale up. */
        m[MODES_PREAMBLE_US*2 + MODES_LONG_MSG_BITS*2 - 1] =
            scaleSample(m[MODES_PREAMBLE_US*2 + MODES_LONG_MSG_BITS*2 - 1], scaleUp);

        /* Walk backwards through message samples. */
        for (j = MODES_PREAMBLE_US*2 + MODES_LONG_MSG_BITS*2 - 2;
             j > MODES_PREAMBLE_US*2; j -= 2) {
            if (m[j] > m[j+1]) {
                /* This bit is 1: previous sample overlapped with high energy,
                 * so it's slightly high. Scale it down. */
                m[j-1] = scaleSample(m[j-1], scaleDown);
            } else {
                /* This bit is 0: previous sample overlapped with low energy,
                 * so it's slightly low. Scale it up. */
                m[j-1] = scaleSample(m[j-1], scaleUp);
            }
        }
    } else {
        /* Sampling early: each sample contains energy from the previous bit.
         * Scale factor is proportional to how much we're off. */
        uint16_t scaleUp = 16384 + 16384 * late / (late + onTime);
        uint16_t scaleDown = 16384 - 16384 * late / (late + onTime);

        /* First data sample: leading bits are 0, so it will be low. Scale up. */
        m[MODES_PREAMBLE_US*2] = scaleSample(m[MODES_PREAMBLE_US*2], scaleUp);

        /* Walk forwards through message samples. */
        for (j = MODES_PREAMBLE_US*2;
             j < MODES_PREAMBLE_US*2 + MODES_LONG_MSG_BITS*2 - 2; j += 2) {
            if (m[j] > m[j+1]) {
                /* This bit is 1: next sample overlapped with low energy
                 * (the 0 half of this bit), so it's slightly low. Scale up. */
                m[j+2] = scaleSample(m[j+2], scaleUp);
            } else {
                /* This bit is 0: next sample overlapped with high energy
                 * (the 1 half of this bit), so it's slightly high. Scale down. */
                m[j+2] = scaleSample(m[j+2], scaleDown);
            }
        }
    }
}

/* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
 * size 'mlen' bytes. Every detected Mode S message is convert it into a
 * stream of bits and passed to the function to display it. */
static void detectModeS(uint16_t *m, uint32_t mlen) {
    unsigned char bits[MODES_LONG_MSG_BITS];
    unsigned char msg[MODES_LONG_MSG_BITS/2];
    uint16_t aux[MODES_LONG_MSG_BITS*2];
    uint32_t j;
    int use_correction = 0;

    /* The Mode S preamble is made of impulses of 0.5 microseconds at
     * the following time offsets:
     *
     * 0   - 0.5 usec: first impulse.
     * 1.0 - 1.5 usec: second impulse.
     * 3.5 - 4   usec: third impulse.
     * 4.5 - 5   usec: last impulse.
     * 
     * Since we are sampling at 2 Mhz every sample in our magnitude vector
     * is 0.5 usec, so the preamble will look like this, assuming there is
     * an impulse at offset 0 in the array:
     *
     * 0   -----------------
     * 1   -
     * 2   ------------------
     * 3   --
     * 4   -
     * 5   --
     * 6   -
     * 7   ------------------
     * 8   --
     * 9   -------------------
     */
    for (j = 0; j < mlen - MODES_FULL_LEN*2; j++) {
        int low, high, delta, i, errors;
        int good_message = 0;

        if (use_correction) goto good_preamble; /* We already checked it. */

        /* First check of relations between the first 10 samples
         * representing a valid preamble. We don't even investigate further
         * if this simple test is not passed. */
        if (!(m[j] > m[j+1] &&
            m[j+1] < m[j+2] &&
            m[j+2] > m[j+3] &&
            m[j+3] < m[j] &&
            m[j+4] < m[j] &&
            m[j+5] < m[j] &&
            m[j+6] < m[j] &&
            m[j+7] > m[j+8] &&
            m[j+8] < m[j+9] &&
            m[j+9] > m[j+6]))
        {
            continue;
        }

        /* The samples between the two spikes must be < than the 2/3 of
         * the average of the high spikes level. We don't test bits too near to
         * the high levels as signals can be out of phase so part of the
         * energy can be in the near samples. */
        high = (m[j]+m[j+2]+m[j+7]+m[j+9])/6;
        if (m[j+4] >= high ||
            m[j+5] >= high)
        {
            continue;
        }

        /* Similarly samples in the range 11-14 must be low, as it is the
         * space between the preamble and real data. Again we don't test
         * bits too near to high levels, see above. */
        if (m[j+11] >= high ||
            m[j+12] >= high ||
            m[j+13] >= high ||
            m[j+14] >= high)
        {
            continue;
        }

good_preamble:
        /* If the previous attempt with this message failed, retry using
         * magnitude correction. */
        if (use_correction) {
            memcpy(aux,m+j+MODES_PREAMBLE_US*2,sizeof(aux));
            /* Apply phase correction unconditionally on retry (j > 0 ensures
             * we can access m[-1] for preamble energy measurement). */
            if (j) {
                applyPhaseCorrection(m+j);
            }
        }

        /* Decode all the next 112 bits, regardless of the actual message
         * size. We'll check the actual message type later. */
        errors = 0;
        for (i = 0; i < MODES_LONG_MSG_BITS*2; i += 2) {
            low = m[j+i+MODES_PREAMBLE_US*2];
            high = m[j+i+MODES_PREAMBLE_US*2+1];
            delta = low-high;
            if (delta < 0) delta = -delta;

            if (i > 0 && delta < 256) {
                bits[i/2] = bits[i/2-1];
            } else if (low == high) {
                /* Checking if two adiacent samples have the same magnitude
                 * is an effective way to detect if it's just random noise
                 * that was detected as a valid preamble. */
                bits[i/2] = 2; /* error */
                if (i < MODES_SHORT_MSG_BITS*2) errors++;
            } else if (low > high) {
                bits[i/2] = 1;
            } else {
                /* (low < high) for exclusion  */
                bits[i/2] = 0;
            }
        }

        /* Restore the original message if we used magnitude correction. */
        if (use_correction)
            memcpy(m+j+MODES_PREAMBLE_US*2,aux,sizeof(aux));

        /* Pack bits into bytes */
        for (i = 0; i < MODES_LONG_MSG_BITS; i += 8) {
            msg[i/8] =
                bits[i]<<7 | 
                bits[i+1]<<6 | 
                bits[i+2]<<5 | 
                bits[i+3]<<4 | 
                bits[i+4]<<3 | 
                bits[i+5]<<2 | 
                bits[i+6]<<1 | 
                bits[i+7];
        }

        int msgtype = msg[0]>>3;
        int msglen = modesMessageLenByType(msgtype)/8;

        /* Last check, high and low bits are different enough in magnitude
         * to mark this as real message and not just noise? */
        delta = 0;
        for (i = 0; i < msglen*8*2; i += 2) {
            delta += abs(m[j+i+MODES_PREAMBLE_US*2]-
                         m[j+i+MODES_PREAMBLE_US*2+1]);
        }
        delta /= msglen*4;

        /* Filter for an average delta of three is small enough to let almost
         * every kind of message to pass, but high enough to filter some
         * random noise. */
        if (delta < 10*255) {
            use_correction = 0;
            continue;
        }

        /* If we reached this point, and error is zero, we are very likely
         * with a Mode S message in our hands, but it may still be broken
         * and CRC may not be correct. This is handled by the next layer. */
        if (errors == 0) {
            struct modesMessage mm;

            /* Decode the received message and update statistics */
            decodeModesMessage(&mm,msg);

            /* Skip this message if we are sure it's fine. */
            if (mm.crcok) {
                j += (MODES_PREAMBLE_US+(msglen*8))*2;
                good_message = 1;
                if (use_correction)
                    mm.phase_corrected = 1;
            }

            /* Pass data to the next layer */
            useModesMessage(&mm);
        }

        /* Retry with phase correction if possible. */
        if (!good_message && !use_correction) {
            j--;
            use_correction = 1;
        } else {
            use_correction = 0;
        }
    }
}
struct aircraft *interactiveCreateAircraft(uint32_t addr) {
    struct aircraft *a = malloc(sizeof(*a));

    a->addr = addr;
    snprintf(a->hexaddr,sizeof(a->hexaddr),"%06x",(int)addr);
    a->flight[0] = '\0';
    a->altitude = 0;
    a->speed = 0;
    a->track = 0;
    a->odd_cprlat = 0;
    a->odd_cprlon = 0;
    a->odd_cprtime = 0;
    a->even_cprlat = 0;
    a->even_cprlon = 0;
    a->even_cprtime = 0;
    a->lat = 0;
    a->lon = 0;
    a->seen = time(NULL);
    a->messages = 0;
    a->next = NULL;
    return a;
}

/* Return the aircraft with the specified address, or NULL if no aircraft
 * exists with this address. */
struct aircraft *interactiveFindAircraft(uint32_t addr) {
    struct aircraft *a = Modes.aircrafts;

    while(a) {
        if (a->addr == addr) return a;
        a = a->next;
    }
    return NULL;
}

/* Always positive MOD operation, used for CPR decoding. */
static int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}

/* The NL function uses the precomputed table from 1090-WP-9-14 */
static int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat; /* Table is simmetric about the equator. */
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}

static int cprNFunction(double lat, int isodd) {
    int nl = cprNLFunction(lat) - isodd;
    if (nl < 1) nl = 1;
    return nl;
}

static double cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

/* This algorithm comes from:
 * http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
 *
 *
 * A few remarks:
 * 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
 * 2) We assume that we always received the odd packet as last packet for
 *    simplicity. This may provide a position that is less fresh of a few
 *    seconds.
 */
static void decodeCPR(struct aircraft *a) {
    const double AirDlat0 = 360.0 / 60;
    const double AirDlat1 = 360.0 / 59;
    double lat0 = a->even_cprlat;
    double lat1 = a->odd_cprlat;
    double lon0 = a->even_cprlon;
    double lon1 = a->odd_cprlon;

    /* Compute the Latitude Index "j" */
    int j = floor(((59*lat0 - 60*lat1) / 131072) + 0.5);
    double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072);
    double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072);

    if (rlat0 >= 270) rlat0 -= 360;
    if (rlat1 >= 270) rlat1 -= 360;

    /* Check that both are in the same latitude zone, or abort. */
    if (cprNLFunction(rlat0) != cprNLFunction(rlat1)) return;

    /* Compute ni and the longitude index m */
    if (a->even_cprtime > a->odd_cprtime) {
        /* Use even packet. */
        int ni = cprNFunction(rlat0,0);
        int m = floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                        (lon1 * cprNLFunction(rlat0))) / 131072) + 0.5);
        a->lon = cprDlonFunction(rlat0,0) * (cprModFunction(m,ni)+lon0/131072);
        a->lat = rlat0;
    } else {
        /* Use odd packet. */
        int ni = cprNFunction(rlat1,1);
        int m = floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                        (lon1 * cprNLFunction(rlat1))) / 131072.0) + 0.5);
        a->lon = cprDlonFunction(rlat1,1) * (cprModFunction(m,ni)+lon1/131072);
        a->lat = rlat1;
    }
    if (a->lon > 180) a->lon -= 360;
}

/* Decode surface CPR position using local decoding with reference position.
 *
 * Surface position messages use a different CPR encoding than airborne:
 * - Latitude zones cover 90 degrees instead of 360 degrees
 * - Only 17 bits are transmitted, but the encoding conceptually uses 19 bits,
 *   so there is inherent ambiguity requiring a reference position
 *
 * The reference position must be within 45 nautical miles of the true position
 * for unambiguous decoding. We use the average of all airborne positions we've
 * decoded as our reference, which works because if we can receive surface
 * traffic at an airport, we almost certainly receive airborne traffic too.
 *
 * This function uses local (relative) decoding with a single message, which
 * requires a reference position but doesn't need even/odd pairs. */
static void decodeCPRSurface(struct aircraft *a, int fflag, int raw_lat, int raw_lon) {
    const double SurfDlat0 = 90.0 / 60;
    const double SurfDlat1 = 90.0 / 59;
    double dlat = fflag ? SurfDlat1 : SurfDlat0;
    double ref_lat = Modes.ref_lat;
    double ref_lon = Modes.ref_lon;
    double lat, lon;
    int j, ni, m;

    /* No reference position yet, can't decode surface position. */
    if (Modes.ref_count == 0) return;

    /* Compute latitude index j from reference latitude. */
    j = (int)floor(ref_lat / dlat) +
        (int)floor(0.5 + cprModFunction((int)ref_lat, (int)dlat) / dlat -
                   (double)raw_lat / 131072);

    lat = dlat * (j + (double)raw_lat / 131072);

    /* Pick the latitude solution closest to the reference. We may need to
     * try ±90 degrees since surface encoding covers only 90 degrees. */
    if (fabs(lat - ref_lat) > 45) {
        if (lat > ref_lat) lat -= 90;
        else lat += 90;
    }

    /* Sanity check: latitude must be in valid range. */
    if (lat < -90 || lat > 90) return;

    /* Compute longitude. */
    ni = cprNFunction(lat, fflag);
    if (ni == 0) ni = 1;  /* Avoid division by zero near poles. */
    m = (int)floor(ref_lon / (90.0 / ni)) +
        (int)floor(0.5 + cprModFunction((int)ref_lon, (int)(90.0 / ni)) /
                   (90.0 / ni) - (double)raw_lon / 131072);
    lon = (90.0 / ni) * (m + (double)raw_lon / 131072);

    /* Pick the longitude solution closest to reference. Surface encoding
     * covers only 90 degrees, so we may need to adjust by ±90 or ±180. */
    while (lon > ref_lon + 45) lon -= 90;
    while (lon < ref_lon - 45) lon += 90;

    /* Normalize longitude to -180..180 range. */
    if (lon > 180) lon -= 360;
    if (lon < -180) lon += 360;

    a->lat = lat;
    a->lon = lon;
}

/* Decode ground speed from surface movement field.
 * Returns speed in knots, or -1 if speed is not available. */
static int decodeMovementField(int movement) {
    if (movement == 0) return -1;           /* Not available */
    if (movement == 1) return 0;            /* Stopped (< 0.125 kt) */
    if (movement <= 8) return (movement - 2) * 0.125 + 0.125;  /* 0.125-1 kt */
    if (movement <= 12) return (movement - 9) * 0.25 + 1;      /* 1-2 kt */
    if (movement <= 38) return (movement - 13) * 0.5 + 2;      /* 2-15 kt */
    if (movement <= 93) return (movement - 39) + 15;           /* 15-70 kt */
    if (movement <= 108) return (movement - 94) * 2 + 70;      /* 70-100 kt */
    if (movement <= 123) return (movement - 109) * 5 + 100;    /* 100-175 kt */
    return 175;  /* >= 175 kt */
}

/* Receive new messages and populate the interactive mode with more info. */
struct aircraft *interactiveReceiveData(struct modesMessage *mm) {
    uint32_t addr;
    struct aircraft *a, *aux;

    if (!mm->crcok) return NULL;
    addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;

    /* Loookup our aircraft or create a new one. */
    a = interactiveFindAircraft(addr);
    if (!a) {
        a = interactiveCreateAircraft(addr);
        a->next = Modes.aircrafts;
        Modes.aircrafts = a;
    } else {
        /* If it is an already known aircraft, move it on head
         * so we keep aircrafts ordered by received message time.
         *
         * However move it on head only if at least one second elapsed
         * since the aircraft that is currently on head sent a message,
         * othewise with multiple aircrafts at the same time we have an
         * useless shuffle of positions on the screen. */
        if (0 && Modes.aircrafts != a && (time(NULL) - a->seen) >= 1) {
            aux = Modes.aircrafts;
            while(aux->next != a) aux = aux->next;
            /* Now we are a node before the aircraft to remove. */
            aux->next = aux->next->next; /* removed. */
            /* Add on head */
            a->next = Modes.aircrafts;
            Modes.aircrafts = a;
        }
    }

    a->seen = time(NULL);
    a->messages++;

    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 20) {
        a->altitude = mm->altitude;
    } else if (mm->msgtype == 17 || mm->msgtype == 18) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            memcpy(a->flight, mm->flight, sizeof(a->flight));
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            a->altitude = mm->altitude;
            if (mm->fflag) {
                a->odd_cprlat = mm->raw_latitude;
                a->odd_cprlon = mm->raw_longitude;
                a->odd_cprtime = mstime();
            } else {
                a->even_cprlat = mm->raw_latitude;
                a->even_cprlon = mm->raw_longitude;
                a->even_cprtime = mstime();
            }
            /* If the two data is less than 10 seconds apart, compute
             * the position. */
            if (llabs(a->even_cprtime - a->odd_cprtime) <= 10000) {
                double prev_lat = a->lat;
                double prev_lon = a->lon;
                decodeCPR(a);
                /* If we successfully decoded a new position, update the
                 * receiver reference position. This is used to decode
                 * surface positions which require a nearby reference. */
                if (a->lat != prev_lat || a->lon != prev_lon) {
                    if (Modes.ref_count == 0) {
                        Modes.ref_lat = a->lat;
                        Modes.ref_lon = a->lon;
                    } else {
                        /* Incremental average update. */
                        Modes.ref_lat += (a->lat - Modes.ref_lat) /
                                         (Modes.ref_count + 1);
                        Modes.ref_lon += (a->lon - Modes.ref_lon) /
                                         (Modes.ref_count + 1);
                    }
                    /* Cap at 10000 so the average can adapt if antenna moves. */
                    if (Modes.ref_count < 10000) Modes.ref_count++;
                }
            }
        } else if (mm->metype >= 5 && mm->metype <= 8) {
            /* Surface position message. Only decode if we have a reference
             * position from earlier airborne position decodes. Without a
             * reference, surface CPR decoding is ambiguous. */
            if (Modes.ref_count) {
                if (mm->ground_track_valid) a->track = mm->ground_track;
                if (mm->movement_valid)
                    a->speed = decodeMovementField(mm->movement);
                a->altitude = 0;  /* On ground. */
                decodeCPRSurface(a, mm->fflag, mm->raw_latitude,
                                 mm->raw_longitude);
            }
        } else if (mm->metype == 19) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                a->speed = mm->velocity;
                a->track = mm->heading;
            }
        }
    }
    return a;
}
static void interactiveRemoveStaleAircrafts(void) {
    struct aircraft *a = Modes.aircrafts;
    struct aircraft *prev = NULL;
    time_t now = time(NULL);

    while(a) {
        if ((now - a->seen) > MODES_INTERACTIVE_TTL) {
            struct aircraft *next = a->next;
            /* Remove the element from the linked list, with care
             * if we are removing the first element. */
            free(a);
            if (!prev)
                Modes.aircrafts = next;
            else
                prev->next = next;
            a = next;
        } else {
            prev = a;
            a = a->next;
        }
    }
}

static int setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void modesInitNet(void) {
    struct sockaddr_in addr;
    int yes = 1;

    Modes.https = socket(AF_INET, SOCK_STREAM, 0);
    if (Modes.https == -1) die("socket");
    setsockopt(Modes.https, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(MODES_NET_HTTP_PORT);

    if (bind(Modes.https, (struct sockaddr *)&addr, sizeof(addr)) == -1) die("bind");
    if (listen(Modes.https, 16) == -1) die("listen");
    if (setNonBlocking(Modes.https) == -1) die("fcntl");

    memset(Modes.clients, 0, sizeof(Modes.clients));
    Modes.maxfd = -1;
    signal(SIGPIPE, SIG_IGN);
    fprintf(stderr, "Web UI: http://localhost:%d\n", MODES_NET_HTTP_PORT);
}

static void modesFreeClient(int fd) {
    close(fd);
    free(Modes.clients[fd]);
    Modes.clients[fd] = NULL;
    if (Modes.maxfd == fd) {
        while (Modes.maxfd >= 0 && Modes.clients[Modes.maxfd] == NULL) {
            Modes.maxfd--;
        }
    }
}

static void modesAcceptClients(void) {
    for (;;) {
        int fd = accept(Modes.https, NULL, NULL);
        struct client *c;
        if (fd == -1) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                perror("accept");
            }
            return;
        }
        if (fd >= MODES_NET_MAX_FD || setNonBlocking(fd) == -1) {
            close(fd);
            continue;
        }
        c = calloc(1, sizeof(*c));
        if (!c) {
            close(fd);
            continue;
        }
        c->fd = fd;
        Modes.clients[fd] = c;
        if (fd > Modes.maxfd) Modes.maxfd = fd;
    }
}

static int writeAll(int fd, const void *buffer, size_t length) {
    const unsigned char *p = buffer;
    while (length) {
        ssize_t written = write(fd, p, length);
        if (written > 0) {
            p += written;
            length -= (size_t)written;
        } else if (written == -1 && errno == EINTR) {
            continue;
        } else {
            return -1;
        }
    }
    return 0;
}

static char *aircraftsToJson(int *len) {
    struct aircraft *a = Modes.aircrafts;
    int buflen = 1024; /* The initial buffer is incremented as needed. */
    char *buf = malloc(buflen), *p = buf;
    int l;

    l = snprintf(p,buflen,"[\n");
    p += l; buflen -= l;
    while(a) {
        int altitude = a->altitude, speed = a->speed;

        /* The web-only build always returns metric values. */
        altitude = (int)(altitude / 3.28084);
        speed = (int)(speed * 1.852);

        if (a->lat != 0 && a->lon != 0) {
            l = snprintf(p,buflen,
                "{\"hex\":\"%s\", \"flight\":\"%s\", \"lat\":%f, "
                "\"lon\":%f, \"altitude\":%d, \"track\":%d, "
                "\"speed\":%d},\n",
                a->hexaddr, a->flight, a->lat, a->lon, altitude, a->track,
                speed);
            p += l; buflen -= l;
            /* Resize if needed. */
            if (buflen < 256) {
                int used = p-buf;
                buflen += 1024; /* Our increment. */
                buf = realloc(buf,used+buflen);
                p = buf+used;
            }
        }
        a = a->next;
    }
    /* Remove the final comma if any, and closes the json array. */
    if (*(p-2) == ',') {
        *(p-2) = '\n';
        p--;
        buflen++;
    }
    l = snprintf(p,buflen,"]\n");
    p += l; buflen -= l;

    *len = p-buf;
    return buf;
}
#define MODES_CONTENT_TYPE_HTML "text/html;charset=utf-8"
#define MODES_CONTENT_TYPE_JSON "application/json;charset=utf-8"

/* Get an HTTP request header and write the response to the client.
 * Again here we assume that the socket buffer is enough without doing
 * any kind of userspace buffering.
 *
 * Returns 1 on error to signal the caller the client connection should
 * be closed. */
static int handleHTTPRequest(struct client *c) {
    char hdr[512];
    int clen = 0, hdrlen;
    int httpver, keepalive;
    char *p, *url, *content;
    char *ctype;

    /* Minimally parse the request. */
    httpver = (strstr(c->buf, "HTTP/1.1") != NULL) ? 11 : 10;
    if (httpver == 10) {
        /* HTTP 1.0 defaults to close, unless otherwise specified. */
        keepalive = strstr(c->buf, "Connection: keep-alive") != NULL;
    } else if (httpver == 11) {
        /* HTTP 1.1 defaults to keep-alive, unless close is specified. */
        keepalive = strstr(c->buf, "Connection: close") == NULL;
    }

    /* Identify he URL. */
    p = strchr(c->buf,' ');
    if (!p) return 1; /* There should be the method and a space... */
    url = ++p; /* Now this should point to the requested URL. */
    p = strchr(p, ' ');
    if (!p) return 1; /* There should be a space before HTTP/... */
    *p = '\0';

    /* Select the content to send, we have just two so far:
     * "/" -> Our google map application.
     * "/data.json" -> Our ajax request to update planes. */
    if (strstr(url, "/data")) {
        content = aircraftsToJson(&clen);
        ctype = MODES_CONTENT_TYPE_JSON;
    } else {
        const char *filename = "gmap.html";

        if (strcmp(url, "/style.css") == 0) {
            filename = "style.css";
            ctype = "text/css";
        } else if (strcmp(url, "/script.js") == 0) {
            filename = "script.js";
            ctype = "application/javascript";
        } else {
            filename = "gmap.html";
            ctype = MODES_CONTENT_TYPE_HTML;
        }

        struct stat sbuf;
        int fd = -1;

        if (stat(filename, &sbuf) != -1 &&
            (fd = open(filename, O_RDONLY)) != -1)
        {
            content = malloc(sbuf.st_size);

            if (content == NULL) {
                clen = 0;
            } else {
                ssize_t bytes_read = read(fd, content, sbuf.st_size);

                if (bytes_read == -1) {
                    free(content);

                    char buf[256];
                    clen = snprintf(
                        buf,
                        sizeof(buf),
                        "Error reading %s: %s",
                        filename,
                        strerror(errno)
                    );

                    content = strdup(buf);
                    ctype = "text/plain";
                } else {
                    clen = (int)bytes_read;
                }
            }
        } else {
            char buf[256];

            clen = snprintf(
                buf,
                sizeof(buf),
                "Error opening %s: %s",
                filename,
                strerror(errno)
            );

            content = strdup(buf);
            ctype = "text/plain";
        }

        if (fd != -1) {
            close(fd);
        }
    }

    /* Create the header and send the reply. */
    hdrlen = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Server: Dump1090\r\n"
        "Content-Type: %s\r\n"
        "Connection: %s\r\n"
        "Content-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n",
        ctype,
        keepalive ? "keep-alive" : "close",
        clen);

    /* Send header and content. */
    if (writeAll(c->fd, hdr, (size_t)hdrlen) == -1 ||
        writeAll(c->fd, content, (size_t)clen) == -1)
    {
        free(content);
        return 1;
    }
    free(content);
    return !keepalive;
}
static void modesReadFromClient(struct client *c, char *sep,
                         int(*handler)(struct client *))
{
    while(1) {
        int left = MODES_CLIENT_BUF_SIZE - c->buflen;
        int nread = read(c->fd, c->buf+c->buflen, left);
        int fullmsg = 0;
        int i;
        char *p;

        if (nread <= 0) {
            if (nread == 0 || errno != EAGAIN) {
                /* Error, or end of file. */
                modesFreeClient(c->fd);
            }
            break; /* Serve next client. */
        }
        c->buflen += nread;

        /* Always null-term so we are free to use strstr() */
        c->buf[c->buflen] = '\0';

        /* If there is a complete message there must be the separator 'sep'
         * in the buffer, note that we full-scan the buffer at every read
         * for simplicity. */
        while ((p = strstr(c->buf, sep)) != NULL) {
            i = p - c->buf; /* Turn it as an index inside the buffer. */
            c->buf[i] = '\0'; /* Te handler expects null terminated strings. */
            /* Call the function to process the message. It returns 1
             * on error to signal we should close the client connection. */
            if (handler(c)) {
                modesFreeClient(c->fd);
                return;
            }
            /* Move what's left at the start of the buffer. */
            i += strlen(sep); /* The separator is part of the previous msg. */
            memmove(c->buf,c->buf+i,c->buflen-i);
            c->buflen -= i;
            c->buf[c->buflen] = '\0';
            /* Maybe there are more messages inside the buffer.
             * Start looping from the start again. */
            fullmsg = 1;
        }
        /* If our buffer is full discard it, this is some badly
         * formatted shit. */
        if (c->buflen == MODES_CLIENT_BUF_SIZE) {
            c->buflen = 0;
            /* If there is garbage, read more to discard it ASAP. */
            continue;
        }
        /* If no message was decoded process the next client, otherwise
         * read more data from the same client. */
        if (!fullmsg) break;
    }
}

static void modesReadFromClients(void) {
    int fd;
    for (fd = 0; fd <= Modes.maxfd; fd++) {
        struct client *c = Modes.clients[fd];
        if (c) modesReadFromClient(c, "\r\n\r\n", handleHTTPRequest);
    }
}

static void backgroundTasks(void) {
    modesAcceptClients();
    modesReadFromClients();
    interactiveRemoveStaleAircrafts();
}

static void useModesMessage(struct modesMessage *mm) {
    interactiveReceiveData(mm);
}

int main(void) {
    modesInit();
    modesInitRTLSDR();
    modesInitNet();

    if (pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL) != 0) {
        die("pthread_create");
    }

    pthread_mutex_lock(&Modes.data_mutex);
    for (;;) {
        while (!Modes.data_ready) {
            pthread_cond_wait(&Modes.data_cond, &Modes.data_mutex);
        }

        computeMagnitudeVector();
        Modes.data_ready = 0;
        pthread_cond_signal(&Modes.data_cond);

        pthread_mutex_unlock(&Modes.data_mutex);
        detectModeS(Modes.magnitude, Modes.data_len / 2);
        backgroundTasks();
        pthread_mutex_lock(&Modes.data_mutex);
    }

    return 0;
}
