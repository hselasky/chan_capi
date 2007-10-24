/*-
 * Copyright (c) 2006-2007 Hans Petter Selasky. All rights reserved.
 * Copyright (C) 2005 Cytronics & Melware, Armin Schindler
 * Copyright (C) 2002-2005 Junghanns.NET GmbH, Klaus-Peter Junghanns
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * chan_capi.c  - Common ISDN API 2.0 for Asterisk / OpenPBX
 *
 */
#include "config.h"

#if (CC_AST_VERSION >= 0x10400)
#include <asterisk.h>
#endif

#include <asterisk/lock.h>
#include <asterisk/frame.h> 
#include <asterisk/channel.h>
#ifndef CC_AST_HAVE_TECH_PVT
#include <asterisk/channel_pvt.h>
#endif
#include <asterisk/logger.h>
#include <asterisk/module.h>
#include <asterisk/pbx.h>
#include <asterisk/config.h>
#include <asterisk/options.h>
#include <asterisk/features.h>
#include <asterisk/utils.h>
#include <asterisk/cli.h>
#include <asterisk/causes.h>
#include <asterisk/ulaw.h>
#include <asterisk/alaw.h>
#ifndef CC_AST_NO_DEVICESTATE
#include <asterisk/devicestate.h>
#endif
#include <sys/time.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <asterisk/dsp.h>
#include "xlaw.h"

#define CAPI_MAKE_TRANSLATOR
#define panic(...) 
#include "chan_capi20.h"
#include "chan_capi.h"

/*
 * local variables
 */
#define CHAN_CAPI_DESC "Common ISDN API 2.0 Driver " ASTERISKVERSION
#define CHAN_CAPI_APP  "capiCommand"

#ifdef CC_AST_HAVE_TECH_PVT
static const char chan_capi_pbx_type[] = "CAPI";
extern const struct ast_channel_tech chan_capi_tech;
#else
static char *chan_capi_pbx_type = "CAPI";
#endif

static const char * const config_file = "capi.conf";

#if (CC_AST_VERSION < 0x10400)
STANDARD_LOCAL_USER;
LOCAL_USER_DECL;
#else
static int unload_module();
#undef CC_AST_CUSTOM_FUNCTION
#define AST_MODULE "chan_capi"
#endif

/*
 * LOCKING RULES
 * =============
 *
 * This channel driver uses several locks. One must be 
 * careful not to reverse the locking order, which will
 * lead to a so called deadlock. Here is the locking order
 * that must be followed:
 *
 * struct call_desc *cd;
 * struct cc_capi_application *p_app;
 *
 * 1. cc_mutex_lock(&do_periodic_lock);
 *
 * 2. cc_mutex_lock(&modlock); (See Asterisk source code)
 *
 * 3. cc_mutex_lock(&chlock); (See Asterisk source code)
 *
 * 4. cc_mutex_lock(&cd->pbx_chan->lock); **
 *
 * 5. cc_mutex_lock(&p_app->lock); ***
 *
 * 6. cc_mutex_lock(&capi_global_lock);
 *
 * 7. cc_mutex_lock(&capi_verbose_lock);
 *
 *
 *  ** the PBX will call the callback functions with 
 *     this lock locked. This lock protects the 
 *     structure pointed to by 'cd->pbx_chan'. Also note
 *     that calling some PBX functions will lock
 *     this lock!
 *
 *  *** in case multiple CAPI applications should be locked 
 *      in series, the CAPI application with the lowest 
 *      memory address should be locked first.
 */

static ast_mutex_t do_periodic_lock;

static ast_mutex_t capi_global_lock;

static ast_mutex_t capi_verbose_lock;

static pthread_t periodic_thread;

static u_int16_t chan_capi_load_level = 0;

static struct config_entry_global capi_global;

static struct cc_capi_application *capi_application[CAPI_MAX_APPLICATIONS];

static struct cc_capi_controller capi_controller[CAPI_MAX_CONTROLLERS];

static u_int8_t capi_controller_used_mask[(CAPI_MAX_CONTROLLERS+7)/8];

static struct config_entry_iface *cep_root_ptr;

static struct config_entry_iface *cep_free_ptr;

static const char * const empty_string = "\0\0";

static const u_int8_t sending_complete_struct[] = { 2, 1, 0 };

static const u_int8_t sending_not_complete_struct[] = { 2, 0, 0 };

static uint8_t update_use_count = 0;

/* external prototypes */
extern const char *capi_info_string(u_int16_t wInfo);

/*===========================================================================*
 * ring buffer routines
 *===========================================================================*/

static void
buf_init(struct ring_buffer *buffer)
{
    bzero(buffer, sizeof(*buffer));
    buffer->end_pos = sizeof(buffer->data);
    buffer->last_byte = 0xFF; /* ISDN default */

    buffer->bf_free_len = FIFO_BF_SIZE;
    return;
}

static void
buf_write_block(struct ring_buffer *buffer, 
		const u_int8_t *data, u_int16_t len_data)
{
    u_int16_t len_max = buffer->end_pos - buffer->bf_write_pos;

    if (len_data > buffer->bf_free_len) {

        /* data overflow */

        len_data = buffer->bf_free_len;
    }

    if (len_data > 0) {
        buffer->last_byte = data[len_data-1];
    }

    /* update free data length */

    buffer->bf_free_len -= len_data;
    buffer->bf_used_len += len_data;

    /* copy data */

    if(len_data >= len_max) {

	/* wrapped write */

	bcopy(data, &(buffer->data[buffer->bf_write_pos]), len_max);

	buffer->bf_write_pos = 0;
	len_data -= len_max;
	data += len_max;
    }

    bcopy(data, &buffer->data[buffer->bf_write_pos], len_data);
    buffer->bf_write_pos += len_data;
    return;
}

static void
buf_read_block(struct ring_buffer *buffer, void **p_ptr, u_int16_t *p_len)
{
    u_int8_t temp[FIFO_BLOCK_SIZE];
    u_int16_t len;

    if(buffer->bf_used_len < FIFO_BLOCK_SIZE) {

        /* data underflow */

        len = FIFO_BLOCK_SIZE - buffer->bf_used_len;

        memset(temp, buffer->last_byte, len);

        buf_write_block(buffer, temp, len);
    }

    *p_ptr = &(buffer->data[buffer->bf_read_pos]);
    *p_len = FIFO_BLOCK_SIZE;

    if(buffer->bf_used_len >= FIFO_BLOCK_SIZE) {
	buffer->bf_used_len -= FIFO_BLOCK_SIZE;
	buffer->bf_free_len += FIFO_BLOCK_SIZE;
    }

    buffer->bf_read_pos += FIFO_BLOCK_SIZE;

    if (buffer->bf_read_pos >= buffer->end_pos) {
        buffer->bf_read_pos -= buffer->end_pos;
    }
    return;
}

/*===========================================================================*
 * software echo suppression
 *===========================================================================*/

/* routine to compute the square root */

u_int16_t 
sqrt_32(u_int32_t a) {
    u_int32_t b = 0x40000000;
    u_int32_t x = 0x40000000;

    while(1) {

        if(a >= b) {

	   a -= b;

	   if(b & 1) {
	      b >>= 1;
	      b |= x;
	      break;
	   }
	   b >>= 1;
	   b |= x;
	   x >>= 1;
	   b ^= x;
	   x >>= 1;
	   b ^= x;

	} else {

	   if(b & 1) {
	      b >>= 1;
	      break;
	   }
	   b >>= 1;
	   x >>= 1;
	   b ^= x;
	   x >>= 1;
	   b ^= x;
	}
    }
    return b;
}

#define EC_FACTOR_MAX 0x100

static int32_t
soft_echo_suppress_get_factor(struct call_desc *cd,
			      struct soft_echo_suppress *rx,
			      struct soft_echo_suppress *tx)
{
    u_int16_t rx_power = rx->power_avg[rx->offset];
    u_int16_t tx_power = ((tx->stuck < EC_STUCK_OFFSET) ? 
			  tx->power_avg[tx->offset] : 0);
    int32_t factor;

    if(cd->options.echo_suppress_fax) {

        /* assure simplex sound */

        if (tx_power > (rx_power/2)) {
	    if(tx->active) {
	      factor = 0x00;
	    } else {
	      rx->active = 1;
	      factor = 0xFF;
	    }
	} else {
	    factor = 0x00;
	    rx->active = 0;
	}

    } else {

        /* duplex sound */

        factor  = tx_power;

	/* the following factor has been 
	 * set according to listening tests:
	 */
	factor *= 38; 

	if ((factor > rx_power) &&
	    (tx_power > 32)) {

	    if(tx->active) {
	        factor = 0x00;
	    } else {
	        rx->active = 1;

		/* activate the echo suppressor
		 *
		 * NOTE: typical "rx_power:tx_power" 
		 * ratio when only echo is received 
		 * is 1:32
		 */
		factor /= (rx_power ? rx_power : 1);

		if (factor > (EC_FACTOR_MAX-1)) {
		    factor = (EC_FACTOR_MAX-1);
		}
		if (factor < 4) {
		    factor = 0x00;
		    rx->active = 0;
		}
	    }
	} else {
	    factor = 0x00;
	    rx->active = 0;
	}
    }
#if 0
    cc_log(LOG_NOTICE, "%s r0x%04x t0x%04x f0x%04x a%d\n", 
	   (rx > tx) ? "                          " : "", 
	   rx_power, tx_power, factor, rx->active);
#endif
    return factor;
}

static const int16_t sin_2100_demux[80] = {
  0x0000, 0x7f99, 0xebfb, 0x838b, 0x278d, 0x7640, 0xc5e5, 0x92de, 
  0x4b3b, 0x6154, 0xa57f, 0xace0, 0x678d, 0x42e0, 0x8df5, 0xcf05, 
  0x79bb, 0x1de1, 0x8195, 0xf5f6, 0x7fff, 0xf5f6, 0x8195, 0x1de1, 
  0x79bb, 0xcf05, 0x8df5, 0x42e0, 0x678d, 0xace0, 0xa57f, 0x6154, 
  0x4b3b, 0x92de, 0xc5e5, 0x7640, 0x278d, 0x838b, 0xebfb, 0x7f99, 
  0x0000, 0x8067, 0x1405, 0x7c75, 0xd873, 0x89c0, 0x3a1b, 0x6d22, 
  0xb4c5, 0x9eac, 0x5a81, 0x5320, 0x9873, 0xbd20, 0x720b, 0x30fb, 
  0x8645, 0xe21f, 0x7e6b, 0x0a0a, 0x8001, 0x0a0a, 0x7e6b, 0xe21f, 
  0x8645, 0x30fb, 0x720b, 0xbd20, 0x9873, 0x5320, 0x5a81, 0x9eac, 
  0xb4c5, 0x6d22, 0x3a1b, 0x89c0, 0xd873, 0x7c75, 0x1405, 0x8067, };

static const int16_t cos_2100_demux[80] = {
  0x7fff, 0xf5f6, 0x8195, 0x1de1, 0x79bb, 0xcf05, 0x8df5, 0x42e0, 
  0x678d, 0xace0, 0xa57f, 0x6154, 0x4b3b, 0x92de, 0xc5e5, 0x7640, 
  0x278d, 0x838b, 0xebfb, 0x7f99, 0x0000, 0x8067, 0x1405, 0x7c75, 
  0xd873, 0x89c0, 0x3a1b, 0x6d22, 0xb4c5, 0x9eac, 0x5a81, 0x5320, 
  0x9873, 0xbd20, 0x720b, 0x30fb, 0x8645, 0xe21f, 0x7e6b, 0x0a0a, 
  0x8001, 0x0a0a, 0x7e6b, 0xe21f, 0x8645, 0x30fb, 0x720b, 0xbd20, 
  0x9873, 0x5320, 0x5a81, 0x9eac, 0xb4c5, 0x6d22, 0x3a1b, 0x89c0, 
  0xd873, 0x7c75, 0x1405, 0x8067, 0x0000, 0x7f99, 0xebfb, 0x838b, 
  0x278d, 0x7640, 0xc5e5, 0x92de, 0x4b3b, 0x6154, 0xa57f, 0xace0, 
  0x678d, 0x42e0, 0x8df5, 0xcf05, 0x79bb, 0x1de1, 0x8195, 0xf5f6, };

static void
soft_echo_suppress_process(struct call_desc *cd, struct soft_echo_suppress *rx, 
			   struct soft_echo_suppress *tx, u_int8_t *ptr, 
			   u_int16_t len)
{
    int32_t pbx_capability = cd->pbx_capability;
    int32_t sound_factor = soft_echo_suppress_get_factor(cd, rx, tx);
    int32_t noise_factor = ((tx->stuck < EC_STUCK_OFFSET) ? 
			    tx->power_avg[tx->offset] : 0) / 256;
    int32_t temp;
    int32_t white_noise;
    u_int16_t x;
    u_int16_t y;
    u_int8_t silence;

    /* clear the stuck variable */
    rx->stuck = 0;

    if (pbx_capability == AST_FORMAT_ULAW) {
        silence = capi_signed_to_ulaw(0);
    } else {
        silence = capi_signed_to_alaw(0);
    }

    while(len--) {
        if (pbx_capability == AST_FORMAT_ULAW) {
	  temp = capi_ulaw_to_signed[*ptr];
	} else {
	  temp = capi_alaw_to_signed[*ptr];
	}

	/* sum up signal power */

	rx->power_acc += (temp * temp) / EC_WINDOW_LEN;

	/* demultiplex FAX tone */

	rx->sin_2100_amp_curr += 
	  (temp * ((int32_t)(sin_2100_demux[rx->sincos_2100_count_1]))) /
	  0x40; 

	rx->cos_2100_amp_curr += 
	  (temp * ((int32_t)(cos_2100_demux[rx->sincos_2100_count_1]))) /
	  0x40;

	if (sound_factor) {

	    /* simple prime white noise generator */

	    white_noise = cd->white_noise_rem;

	    if (white_noise & 1) {
	        white_noise += EC_NOISE_PRIME;
	    }
	    white_noise /= 2;

	    cd->white_noise_rem = white_noise;

	    /* convert unsigned to signed */

	    white_noise ^= 0x800000;
	    if(white_noise & 0x800000) {
	       white_noise |= (-0x800000);
	    }

	    white_noise /= 256;
	    white_noise *= noise_factor;
	    white_noise /= 65536;

	    if(sound_factor >= 0xFF)
	      temp = silence;
	    else
	      temp = (temp * ((EC_FACTOR_MAX-1) - sound_factor)) / EC_FACTOR_MAX;

	    temp += (white_noise * sound_factor) / EC_FACTOR_MAX;

	    if (pbx_capability == AST_FORMAT_ULAW) {
	        *ptr = capi_signed_to_ulaw(temp);
	    } else {
	        *ptr = capi_signed_to_alaw(temp);
	    }
	}

	rx->sincos_2100_count_1++;
	if (rx->sincos_2100_count_1 >= 80) {

	    int32_t dx,dy,dr;

	    rx->sin_2100_amp_curr /= 0x10000;
	    rx->cos_2100_amp_curr /= 0x10000;

	    temp = ((rx->sin_2100_amp_curr*rx->sin_2100_amp_curr) +
		    (rx->cos_2100_amp_curr*rx->cos_2100_amp_curr));

	    dx = (rx->cos_2100_amp_curr -
		  rx->cos_2100_amp_last);

	    dy = (rx->sin_2100_amp_curr -
		  rx->sin_2100_amp_last);

	    dr = ((dx*dx) + (dy*dy));
#if 0
	    cc_log(LOG_NOTICE, "FAX amplitude: %s d=%d t=0x%08x r=0x%08x %d\n", 
		   (rx > tx) ? "                             " : "",
		   cd->options.echo_suppress_fax, temp, dr, temp - dr);
#endif
	    /*
	     * detect 2100 +/- 16.7 Hz 
	     * according to ITU G.168
	     */
	    if((dr >= 0) &&
	       (dr < temp) && 
	       (temp > 0x10000))
	    {
	        rx->sincos_2100_count_2++;

	        /* disable the echo suppressor */

	        if((rx->sincos_2100_count_2 >= 20) &&
		   (cd->options.echo_suppress_fax == 0)) {
		   cd->options.echo_suppress_fax = 1;

		   cd_verbose(cd, 1, 0, 3, "FAX tone detected, "
			      "switching echo suppressor!\n");
		}
	    } else {
	        rx->sincos_2100_count_2 = 0;
	    }

	    rx->sincos_2100_count_1 = 0;

	    rx->sin_2100_amp_last = 
	      rx->sin_2100_amp_curr;

	    rx->cos_2100_amp_last = 
	      rx->cos_2100_amp_curr;

	    rx->sin_2100_amp_curr = 0;
	    rx->cos_2100_amp_curr = 0;
	}

	rx->samples++;
	if (rx->samples >= EC_WINDOW_LEN) {

	    /* increase stuck variable of the peer */

	    if(tx->stuck != 0xFF) {
	       tx->stuck++;
	    }

	    /* clear current RX power */

	    rx->power_avg[rx->offset] = 0;

	    /* advance power average offset */

	    rx->offset++;
	    if(rx->offset >= EC_WINDOW_COUNT) {
	        rx->offset = 0;
	    }

	    /* square root the accumulated power */

	    rx->power_acc = sqrt_32(rx->power_acc);

	    /* store RX power */

	    for(x = cd->options.echo_suppress_offset; x--; ) {

	      y = (x + rx->offset) % EC_WINDOW_COUNT;

	      if(rx->power_avg[y] < rx->power_acc) {
		 rx->power_avg[y] = rx->power_acc;
	      }
	    }

	    /* get next value */

	    sound_factor = soft_echo_suppress_get_factor(cd, rx, tx);

	    rx->power_acc = 0;
	    rx->samples = 0;
	}

        ptr++;
    }
    return;
}

/*===========================================================================*
 * various CAPI helper functions
 *===========================================================================*/

#if (CAPI_OS_HINT == 0)
/*
 * Copyright (c) 1998 Todd C. Miller <Todd.Miller@courtesan.com>
 *
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
static size_t
strlcpy(char *dst, const char *src, size_t siz)
{
        char *d = dst;
        const char *s = src;
        size_t n = siz;

        /* Copy as many bytes as will fit */
        if (n != 0 && --n != 0) {
                do {
                        if ((*d++ = *s++) == 0)
                                break;
                } while (--n != 0);
        }

        /* Not enough room in dst, add NUL and traverse rest of src */
        if (n == 0) {
                if (siz != 0)
                        *d = '\0';              /* NUL-terminate dst */
                while (*s++)
                        ;
        }
        return(s - src - 1);    /* count does not include NUL */
}

/*
 * Appends src to string dst of size siz (unlike strncat, siz is the
 * full size of dst, not space left).  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz <= strlen(dst)).
 * Returns strlen(src) + MIN(siz, strlen(initial dst)).
 * If retval >= siz, truncation occurred.
 */
static size_t
strlcat(dst, src, siz)
        char *dst;
        const char *src;
        size_t siz;
{
        char *d = dst;
        const char *s = src;
        size_t n = siz;
        size_t dlen;

        /* Find the end of dst and adjust bytes left but don't go past end */
        while (n-- != 0 && *d != '\0')
                d++;
        dlen = d - dst;
        n = siz - dlen;

        if (n == 0)
                return(dlen + strlen(s));
        while (*s != '\0') {
                if (n != 1) {
                        *d++ = *s;
                        n--;
                }
                s++;
        }
        *d = '\0';

        return(dlen + (s - src));       /* count does not include NUL */
}
#endif

static void
__cc_mutex_assert(ast_mutex_t *p_lock, u_int32_t flags, 
		  const char *file, const char *func, u_int32_t line)
{
#if 0 
    /* this is not the right way to do it, but one can enable
     * this code to have some kind of mutex assertion.
     */
    int locked = ast_mutex_unlock(p_lock);

    if(locked == 0) ast_mutex_lock(p_lock);

    locked = (locked == 0);

    if(locked && (flags & MA_NOTOWNED)) {
      cc_log(LOG_ERROR, "Mutex is owned at %s:%s:%d!\n",
	     file, func, line);
    }

    if((!locked) && (flags & MA_OWNED)) {
      cc_log(LOG_ERROR, "Mutex is not owned at %s:%s:%d!\n",
	     file, func, line);
    }
#endif
    return;
}

/* simple counter */

static u_int32_t
capi_get_counter()
{
    static u_int32_t count = 0;
    u_int32_t temp;
    cc_mutex_lock(&capi_global_lock);
    temp = count++;
    cc_mutex_unlock(&capi_global_lock);
    return temp;
}

/* check if a CAPI structure read, at the given offset, is valid */

static u_int8_t
capi_get_valid(const void *ptr, u_int16_t offset)
{
    const u_int8_t *data = (const u_int8_t *)ptr;
    u_int16_t len;

    if (data == NULL) {
        return 0;
    }

    if (data[0] == 0xff) {
        len = data[1] | (data[2] << 8);
    } else {
        len = data[0];
    }
    return ((offset < len) ? 1 : 0);
}

/* read a byte from a CAPI structure */

static u_int8_t
capi_get_1(const void *ptr, u_int16_t offset)
{
    const u_int8_t *data = (const u_int8_t *)ptr;
    u_int16_t len;

    if (data == NULL) {
        return 0;
    }

    if (data[0] == 0xff) {
        len = data[1] | (data[2] << 8);
	data += 2;
    } else {
        len = data[0];
	data += 1;
    }
    return ((offset < len) ? data[offset] : 0);
}

/* read a word from a CAPI structure */

static u_int16_t 
capi_get_2(const void *ptr, u_int16_t offset)
{
    return (capi_get_1(ptr,offset)|(capi_get_1(ptr,offset+1) << 8));
}

/* read a dword from a CAPI structure */

static u_int32_t 
capi_get_4(const void *ptr, u_int16_t offset)
{
    return (capi_get_2(ptr,offset)|(capi_get_2(ptr,offset+2) << 16));
}

/* convert a CAPI structure into a zero terminated string */

static void
capi_get_multi_1(const void *src, u_int16_t offset, 
		 void *dst, u_int16_t max_len)
{
    const u_int8_t *data = (const u_int8_t *)src;
    u_int16_t len;

    if (max_len == 0) {
        return;
    }

    /* reserve one byte for 
     * the terminating zero:
     */
    max_len--;

    if (data == NULL) {

        len = 0;

    } else {

        if (data[0] == 0xff) {
	    len = data[1] | (data[2] << 8);
	    data += 2;
	} else {
	    len = data[0];
	    data += 1;
	}

    }

    if (offset >= len) {
        ((u_int8_t *)dst)[0] = 0;
	return;
    }

    len -= offset;
    data += offset;

    if (len > max_len) {
        len = max_len;
    }

    bcopy(data, dst, len);
    ((u_int8_t *)dst)[len] = '\0';

    return;
}

/* write a byte to a CAPI structure */

static void
capi_put_1(void *ptr, u_int16_t offset, u_int8_t value)
{
    u_int8_t *data = (u_int8_t *)ptr;
    u_int16_t len;

    if (data == NULL) {
        return;
    }

    if (data[0] == 0xff) {
        len = data[1] | (data[2] << 8);
	data += 2;
    } else {
        len = data[0];
	data += 1;
    }

    if(offset < len) {
       data[offset] = value;
    }
    return;
}

/* write a word to a CAPI structure */

static void
capi_put_2(void *ptr, u_int16_t offset, u_int16_t value)
{
    capi_put_1(ptr, offset, value);
    capi_put_1(ptr, offset+1, value >> 8);
    return;
}

/* write a dword to a CAPI structure */

static void
capi_put_4(void *ptr, u_int16_t offset, u_int32_t value)
{
    capi_put_2(ptr, offset, value);
    capi_put_2(ptr, offset+2, value >> 16);
    return;
}

/* build a CAPI structure */

static void
capi_build_struct(void *dst, u_int16_t max_len, 
		  const void *src1, u_int16_t len1, 
		  const void *src2, u_int16_t len2,
		  const void *src3, u_int16_t len3)
{
    u_int32_t temp = (len1 + len2 + len3);
    u_int8_t *dst_ptr = (u_int8_t *)dst;

    if (max_len < 3) {
        /* just forget it */
        return;
    }

    max_len -= 3;

    if (temp >= max_len) {
        /* truncate */
        temp = max_len;
    }

    max_len += 3;

    if (temp >= 0xFF) {

	dst_ptr[0] = 0xFF;
	dst_ptr[1] = (temp & 0xFF);
	dst_ptr[2] = (temp >> 8);

	dst_ptr += 3;
	max_len -= 3;

    } else {

	dst_ptr[0] = temp;

	dst_ptr += 1;
	max_len -= 1;
    }

    if (len1) {

#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

        temp = min(max_len, len1);

	bcopy(src1, dst_ptr, temp);

	dst_ptr += temp;
	max_len -= temp;
    }

    if (len2) {

        temp = min(max_len, len2);

	bcopy(src2, dst_ptr, temp);

	dst_ptr += temp;
	max_len -= temp;
    }

    if (len3) {

        temp = min(max_len, len3);

	bcopy(src3, dst_ptr, temp);

	dst_ptr += temp;
	max_len -= temp;
    }
    return;
}

/* copy sound and do sound conversion */

static void
capi_copy_sound(const void *_src, void *_dst, 
		u_int16_t len, const u_int8_t *p_table)
{
    const u_int8_t *src = (const u_int8_t *)(_src);
    u_int8_t       *dst = (u_int8_t *)(_dst);

    if (p_table) {

        if (src == dst) {

	    while(len--) {
	        *dst = p_table[*dst];
		dst++;
	    }

	} else {

	    while(len--) {
	        *dst = p_table[*src];
		dst++;
		src++;
	    }
	}

    } else {

        if(src != dst) {
	    bcopy(src,dst,len);
	}
    }
    return;
}

/*  command to string function */

static const char *
capi_command_to_string(u_int16_t wCmd)
{
    enum { lowest_value = CAPI_P_MIN,
	   end_value = CAPI_P_MAX,
	   range = end_value - lowest_value,
    };

#undef  CHAN_CAPI_COMMAND_DESC
#define CHAN_CAPI_COMMAND_DESC(n, ENUM, value)		\
	[CAPI_P_REQ(ENUM)-(n)]  = #ENUM "_REQ",		\
	[CAPI_P_CONF(ENUM)-(n)] = #ENUM "_CONF",	\
	[CAPI_P_IND(ENUM)-(n)]  = #ENUM "_IND",		\
	[CAPI_P_RESP(ENUM)-(n)] = #ENUM "_RESP",

    static const char * const table[range] = {
        CAPI_COMMANDS(CHAN_CAPI_COMMAND_DESC, lowest_value)
    };

    wCmd -= lowest_value;

    if (wCmd >= range) {
        goto error;
    }

    if (table[wCmd] == NULL) {
        goto error;
    }
    return table[wCmd];

 error:
    return "UNDEFINED";
}

/*  show the text for a CAPI message info value */

static void 
capi_show_info(u_int16_t info)
{
    const char *p;
	
    if (info == 0x0000) {
        /* no error, do nothing */
        return;
    }

    if (!(p = capi_info_string((u_int32_t)info))) {
        /* message not available */
        return;
    }

    cc_verbose(3, 0, VERBOSE_PREFIX_4 "CAPI INFO "
	       "0x%04x: %s\n", info, p);
    return;
}

/* show error in confirmation */

static void
capi_show_conf_error(struct call_desc *cd, u_int32_t PLCI, 
		     u_int16_t wInfo, u_int16_t wCmd)
{
    const char *name = chan_capi_pbx_type;

    if (cd && cd->cep) {
        name = cd->cep->name;
    }
		
    if ((wInfo == 0x2002) && cd) {
        cd_verbose(cd, 1, 1, 3, "0x%04x (wrong state) "
		   "Command=%s,0x%04x\n",
		   wInfo, capi_command_to_string(wCmd), wCmd);
    } else {
        cc_log(LOG_WARNING, "%s: conf_error 0x%04x "
	       "PLCI=0x%08x Command=%s,0x%04x\n",
	       name, wInfo, PLCI, capi_command_to_string(wCmd), wCmd);
    }
    return;
}


/*===========================================================================*
 * call descriptor and configuration management functions
 *===========================================================================*/

/* prototypes */

static void
cd_free(struct call_desc *cd, u_int8_t hangup_what);

static u_int8_t
cd_set_cep(struct call_desc *cd, struct config_entry_iface *cep);

static void
cd_root_shrink(struct call_desc *cd);

static void *
capi_do_monitor(void *data);

static u_int16_t
capi_send_disconnect_req(struct call_desc *cd);

static u_int16_t
capi_send_connect_resp(struct call_desc *cd, u_int16_t wReject, 
		       const u_int16_t *p_bprot);
static u_int16_t
capi_send_connect_b3_req(struct call_desc *cd);

static void
capi_handle_cmsg(struct cc_capi_application *p_app, _cmsg *CMSG);

static u_int16_t
chan_capi_cmd_progress(struct call_desc *cd, struct call_desc *cd_unknown, 
		       char *param);

static u_int16_t
chan_capi_cmd_retrieve(struct call_desc *cd, struct call_desc *cd_unknown, 
		       char *param);

static u_int16_t
chan_capi_cmd_hold(struct call_desc *cd, struct call_desc *cd_unknown, 
		   char *param);

#ifndef CC_AST_HAVE_TECH_PVT
static void
chan_capi_fill_pvt(struct ast_channel *pbx_chan);
#endif

static u_int16_t
chan_capi_fill_controller_info(struct cc_capi_application *p_app,
			       const u_int16_t controller_unit);

static struct call_desc *
cd_by_pbx_chan(struct ast_channel *pbx_chan);

static int 
cd_send_pbx_frame(struct call_desc *cd, int frametype, int subclass, 
		  const void *data, u_int16_t len);
static u_int8_t
capi_application_usleep(struct cc_capi_application *p_app, u_int32_t us);

static int
chan_capi_scan_config(struct ast_config *cfg);

static u_int16_t
chan_capi_post_init(struct cc_capi_application *p_app);

#define CD_IS_UNUSED(cd)			\
        (((cd)->msg_num == 0x0000) &&		\
         ((cd)->msg_plci == 0x0000) &&		\
         ((cd)->state == 0x00))

#define CD_NO_HANGUP(cd) \
        (((cd)->hangup_chan == NULL) &&	\
         ((cd)->free_chan == NULL))

/* initialize sound conversion tables */

static void
cep_init_convert_tables(struct config_entry_iface *cep)
{
    const int dp = 256; /* decimal point */
    const int gain_max = ((dp*256)-1);
    int capability;
    int rx_gain = (int)(cep->rx_gain * 256.0);
    int tx_gain = (int)(cep->tx_gain * 256.0);
    int n = 0;
    int x = 0;

    cc_mutex_lock(&capi_global_lock);
    capability = capi_global.capability;
    cc_mutex_unlock(&capi_global_lock);

    if (rx_gain > gain_max) {
        rx_gain = gain_max;
    }

    if (tx_gain > gain_max) {
        tx_gain = gain_max;
    }

    if (rx_gain < -gain_max) {
        rx_gain = -gain_max;
    }

    if (tx_gain < -gain_max) {
        tx_gain = -gain_max;
    }

    if (rx_gain != dp) {

        for (n = 0; n < 256; n++) {

	        if (capability == AST_FORMAT_ULAW) {
		    x = (capi_ulaw_to_signed[n] * rx_gain) / dp;
		} else {
		    x = (capi_alaw_to_signed[n] * rx_gain) / dp;
		}

		if (capability == AST_FORMAT_ULAW) {
		    cep->rx_convert[n] = capi_reverse_bits[capi_signed_to_ulaw(x)];
		} else {
		    cep->rx_convert[n] = capi_reverse_bits[capi_signed_to_alaw(x)];
		}
	}

    } else {

        for (n = 0; n < 256; n++) {
	        cep->rx_convert[n] = capi_reverse_bits[n];
	}
    }

    if (tx_gain != dp) {

        for (n = 0; n < 256; n++) {

	        if (capability == AST_FORMAT_ULAW) {
		    x = (capi_ulaw_to_signed[capi_reverse_bits[n]] * tx_gain) / dp;
		} else {
		    x = (capi_alaw_to_signed[capi_reverse_bits[n]] * tx_gain) / dp;
		}

		if (capability == AST_FORMAT_ULAW) {
		    cep->tx_convert[n] = capi_signed_to_ulaw(x);
		} else {
		    cep->tx_convert[n] = capi_signed_to_alaw(x);
		}
	}

    } else {

        for (n = 0; n < 256; n++) {
		cep->tx_convert[n] = capi_reverse_bits[n];
	}
    }
    return;
}

static void
cep_root_prepend(struct config_entry_iface *cep)
{
    cc_mutex_lock(&capi_global_lock);
    cep->next = cep_root_ptr;
    cep_root_ptr = cep;
    cc_mutex_unlock(&capi_global_lock);
    return;
}

static struct config_entry_iface *
cep_alloc(const char *name)
{
    struct config_entry_iface *cep;
    struct config_entry_iface **cep_p;

    if (name == NULL) {
        return NULL;
    }

    cc_mutex_lock(&capi_global_lock);
    cep = cep_free_ptr;
    cep_p = &cep_free_ptr;
    while (cep) {

        if (strcmp(cep->name, name) == 0) {
	    /* remove entry from linked list */
	    cep_p[0] = cep->next;
	    cep->next = NULL;
	    break;
	}
	cep_p = &cep->next;
        cep = cep->next;
    }
    cc_mutex_unlock(&capi_global_lock);

    if (cep == NULL) {
	cep = malloc(sizeof(*cep));

	if (cep) {
	    bzero(cep, sizeof(*cep));
	    strlcpy(cep->name, name, sizeof(cep->name));
	}
    } else {

        bzero(&cep->dummy_zero_start[0],
	      &cep->dummy_zero_end[0] -
	      &cep->dummy_zero_start[0]);
    }
    return cep;
}

static void
cep_unload()
{
    struct config_entry_iface **cep_p;
    struct config_entry_iface *cep;

    cc_mutex_lock(&capi_global_lock);

    cep = cep_root_ptr;

    while (cep) {

      /* If there are active channels
       * the counters won't reach zero,
       * therefore max is subtracted instead:
       */
      cep->b_channels_curr -= cep->b_channels_max;
      cep->b_channels_max = 0;

      cep->d_channels_curr -= cep->d_channels_max;
      cep->d_channels_max = 0;

      cep = cep->next;
    }

    cep = cep_free_ptr;
    cep_p = &cep_free_ptr;

    while (cep) {
      cep_p = &cep->next;
      cep = cep->next;
    }

    /* move all configuration 
     * entries over to free 
     * list:
     */
    cep_p[0] = cep_root_ptr;
    cep_root_ptr = NULL;

    cc_mutex_unlock(&capi_global_lock);
    return;
}

static struct config_entry_iface *
cep_root_acquire()
{
    struct config_entry_iface *cep;

    cc_mutex_lock(&capi_global_lock);
    cep = cep_root_ptr;
    return cep;
}

static void
cep_root_release()
{
    /* currently does nothing: eventually this function
     * should do some cleanup, update config ...
     */
    cc_mutex_unlock(&capi_global_lock);
    return;
}

static void
cep_queue_last(struct config_entry_iface *cep_last)
{
    struct config_entry_iface *ce_p;
    struct config_entry_iface **ce_pp;

    cc_mutex_assert(&capi_global_lock, MA_OWNED);

    ce_p = cep_root_ptr;
    ce_pp = &cep_root_ptr;
    while(ce_p)
    {
        if(ce_p == cep_last) {
	    /* remove entry */
	    ce_pp[0] = ce_p->next;
	    ce_p = ce_p->next;
	} else {
	    /* get next entry */
	    ce_pp = &(ce_p->next);
	    ce_p = ce_p->next;
	}
    }

    /* insert last */
    ce_pp[0] = cep_last;
    cep_last->next = NULL;
    return;
}

/*---------------------------------------------------------------------------*
 *      capi_application_free - free a CAPI application
 *---------------------------------------------------------------------------*/
static void
capi_application_free(struct cc_capi_application *p_app)
{
    struct call_desc *cd;
    struct call_desc *temp;

    if (p_app == NULL) {
        return;
    }

    cc_mutex_assert(&do_periodic_lock, MA_OWNED);

    cc_mutex_lock(&p_app->lock);

    if(p_app->monitor_thread_created) {
	    pthread_cancel(p_app->monitor_thread);
	    pthread_kill(p_app->monitor_thread, SIGURG);
	    pthread_join(p_app->monitor_thread, NULL);
    }

    cd = p_app->cd_root_ptr;
    p_app->cd_root_ptr = NULL; /* the list is gone */

    while (cd) {

        if(!CD_IS_UNUSED(cd)) {
	    cc_log(LOG_WARNING, "CAPI call descriptor is still active!\n");

#warning "What about cd->next after free? Currently 'cd' is not freed by cd_free()."

	    cd_free(cd, 1);
	}

	temp = cd;
        cd = cd->next;

	cd_root_shrink(temp);
    }

    /* wait for calls to hang up */

    capi_application_usleep(p_app, 2000*1000);

    capi20_release(p_app->application_id);

    cc_mutex_unlock(&p_app->lock);

    free(p_app);

    return;
}

/*---------------------------------------------------------------------------*
 *      capi_application_alloc - allocate a new CAPI application
 *---------------------------------------------------------------------------*/
static struct cc_capi_application *
capi_application_alloc()
{
    struct cc_capi_application *p_app;
    u_int32_t error;
    u_int32_t app_id;

    error = capi20_isinstalled();

    if (error) {
        cc_log(LOG_WARNING, "The CAPI device is "
	       "not present or accessible!\n");
	return NULL;
    }
#if (CAPI_OS_HINT == 2)
    error = capi20_register(CAPI_BCHANS, (CAPI_MAX_B3_BLOCKS+1)/2,
			    CAPI_MAX_B3_BLOCK_SIZE, &app_id,
			    CAPI_STACK_VERSION);
#else
    error = capi20_register(CAPI_BCHANS, (CAPI_MAX_B3_BLOCKS+1)/2,
			    CAPI_MAX_B3_BLOCK_SIZE, &app_id);
#endif
    if (error) {
        cc_log(LOG_NOTICE, "unable to register a CAPI application, "
	       "error=0x%04x!\n", error);
	return NULL;
    }

    p_app = malloc(sizeof(*p_app));

    if (p_app == NULL) {
        goto error;
    }

    bzero(p_app, sizeof(*p_app));

    cc_mutex_init(&p_app->lock);

    cc_mutex_lock(&p_app->lock);

    p_app->cd_alloc_rate_max = 16; /* max 16 calls per second */
    p_app->application_id = app_id;

    error = ast_pthread_create(&p_app->monitor_thread, NULL, &capi_do_monitor, p_app);

    if (error) {
        cc_log(LOG_ERROR, "Unable to start monitor thread!\n");
	goto error;
    }

    p_app->monitor_thread_created = 1;

    cc_mutex_unlock(&p_app->lock);

    return p_app;

 error:
    if (p_app) {
      cc_mutex_unlock(&p_app->lock);
      cc_mutex_lock(&do_periodic_lock);
      capi_application_free(p_app);
      cc_mutex_unlock(&do_periodic_lock);
    } else {
      capi20_release(app_id);
    }
    return NULL;
}

/*---------------------------------------------------------------------------*
 *      capi_application_usleep - sleep a CAPI application
 *---------------------------------------------------------------------------*/
static u_int8_t
capi_application_usleep(struct cc_capi_application *p_app, u_int32_t us)
{
    cc_mutex_assert(&p_app->lock, MA_OWNED);

    p_app->sleep_count++;

    cc_mutex_unlock(&p_app->lock);

    if (us >= 1000000) {

        sleep(us / 1000000);

	us %= 1000000;
    }

    if (us) {

        usleep(us);
    }
    cc_mutex_lock(&p_app->lock);

    p_app->sleep_count--;
    return 0;
}

/*---------------------------------------------------------------------------*
 *      get_msg_num_other - get a new CAPI message number
 *---------------------------------------------------------------------------*/
static u_int16_t
get_msg_num_other(struct cc_capi_application *p_app)
{
    cc_mutex_assert(&p_app->lock, MA_OWNED);

    p_app->message_number_other++;
    p_app->message_number_other &= 0x7FFF;

    if (p_app->message_number_other == 0) {
        /* avoid zero */
        p_app->message_number_other = 1;
    }
    return p_app->message_number_other;
}

/*---------------------------------------------------------------------------*
 *      get_msg_num_dial - get a new CAPI message number
 *
 * NOTE: one does not want to mix "dial" message numbers with
 *       the "other" message numbers !
 *---------------------------------------------------------------------------*/
static u_int16_t
get_msg_num_dial(struct cc_capi_application *p_app)
{
    cc_mutex_assert(&p_app->lock, MA_OWNED);

    p_app->message_number_dial++;
    p_app->message_number_dial &= 0x7FFF;

    if (p_app->message_number_dial == 0) {
        /* avoid zero */
        p_app->message_number_dial = 1;
    }
    return (p_app->message_number_dial | 0x8000);
}

#define cd_lock(cd) cc_mutex_lock(&(cd)->p_app->lock)
#define cd_unlock(cd) cc_mutex_unlock(&(cd)->p_app->lock)

/*---------------------------------------------------------------------------*
 *      cd_mutex_unlock_double - unlock two call descriptors
 *---------------------------------------------------------------------------*/
static void
cd_mutex_unlock_double(struct call_desc *cd0,
		       struct call_desc *cd1)
{
    if (cd0 == cd1) {

        cc_mutex_unlock(&cd0->p_app->lock);

    } else if (cd0->p_app < cd1->p_app) {

        cc_mutex_unlock(&cd0->p_app->lock);
	cc_mutex_unlock(&cd1->p_app->lock);

    } else {

	cc_mutex_unlock(&cd1->p_app->lock);
        cc_mutex_unlock(&cd0->p_app->lock);

    }
    return;
}

/*---------------------------------------------------------------------------*
 *      cd_mutex_lock_double - lock two call descriptors
 *---------------------------------------------------------------------------*/
static void
cd_mutex_lock_double(struct call_desc *cd0,
		     struct call_desc *cd1)
{
    if (cd0 == cd1) {

        cc_mutex_lock(&cd0->p_app->lock);

    } else if (cd0->p_app < cd1->p_app) {

        cc_mutex_lock(&cd0->p_app->lock);
	cc_mutex_lock(&cd1->p_app->lock);

    } else {

	cc_mutex_lock(&cd1->p_app->lock);
        cc_mutex_lock(&cd0->p_app->lock);

    }
    return;
}

/*---------------------------------------------------------------------------*
 *      cd_mutex_lock_double_pbx_chan - lock two PBX channels
 *---------------------------------------------------------------------------*/
static u_int8_t
cd_mutex_lock_double_pbx_chan(struct ast_channel *pbx_chan0,
			      struct ast_channel *pbx_chan1,
			      struct call_desc **pp_cd0,
			      struct call_desc **pp_cd1)
{
    struct call_desc *cd0 = CC_CHANNEL_PVT(pbx_chan0);
    struct call_desc *cd1 = CC_CHANNEL_PVT(pbx_chan1);

#if 0
    cc_mutex_assert(&pbx_chan0->lock, MA_OWNED);
    cc_mutex_assert(&pbx_chan1->lock, MA_OWNED);
#endif

    if (cd0 == cd1)
    {
        cd1 = cd0 = cd_by_pbx_chan(pbx_chan0);
    }
    else if (cd0->p_app < cd1->p_app)
    {
        cd0 = cd_by_pbx_chan(pbx_chan0);
        cd1 = cd_by_pbx_chan(pbx_chan1);
    }
    else
    {
        cd1 = cd_by_pbx_chan(pbx_chan1);
        cd0 = cd_by_pbx_chan(pbx_chan0);
    }

    if (cd1 && cd0) 
    {
        /* success */
        *pp_cd0 = cd0;
	*pp_cd1 = cd1;
        return 0;
    }

    /* failure */

    if (cd0) 
    {
        cd_unlock(cd0);
    }

    if (cd1)
    {
        cd_unlock(cd1);
    }

    /* no channel interface */
    cc_log(LOG_ERROR, "PBX channel has no interface!\n");
    return 1;
}

/*---------------------------------------------------------------------------*
 *      cd_usleep - sleep a call descriptor
 *
 * returns 0 on success. Else call descriptor is gone.
 *---------------------------------------------------------------------------*/
static u_int8_t
cd_usleep(struct call_desc *cd0, struct call_desc *cd1, u_int32_t us)
{
    struct ast_channel *pbx_chan0;
    struct ast_channel *pbx_chan1;

    cc_mutex_assert(&cd0->p_app->lock, MA_OWNED);
    cc_mutex_assert(&cd1->p_app->lock, MA_OWNED);

    pbx_chan0 = cd0->pbx_chan;
    pbx_chan1 = cd1->pbx_chan;

    cd0->p_app->sleep_count++;
    cd1->p_app->sleep_count++;

    cd_mutex_unlock_double(cd0, cd1);

    usleep(us);

    if (cd0 == cd1) {

      cc_mutex_lock(&(cd0)->p_app->lock);

    } else if (cd0->p_app < cd1->p_app) {

      cc_mutex_lock(&(cd0)->p_app->lock);
      cc_mutex_lock(&(cd1)->p_app->lock);

    } else {

      cc_mutex_lock(&(cd1)->p_app->lock);
      cc_mutex_lock(&(cd0)->p_app->lock);

    }

    cd0->p_app->sleep_count--;
    cd1->p_app->sleep_count--;

    if ((cd0->pbx_chan == pbx_chan0) &&
	(cd1->pbx_chan == pbx_chan1) &&
	(!CD_IS_UNUSED(cd0)) &&
	(!CD_IS_UNUSED(cd1))) {
        return 0;
    }

    /* the call descriptor is gone */

    return 1;
}

/*---------------------------------------------------------------------------*
 *      cd_pbx_read - read data from PBX
 *
 * cd1: second call descriptor in case of bridging
 *---------------------------------------------------------------------------*/
static struct ast_frame *
cd_pbx_read(struct call_desc *cd0, struct call_desc *cd1)
{
    struct ast_channel *pbx_chan0 = cd0->pbx_chan;
    struct ast_channel *pbx_chan1 = cd1->pbx_chan;
    struct ast_frame *f;

    cc_mutex_assert(&cd0->p_app->lock, MA_OWNED);
    cc_mutex_assert(&cd1->p_app->lock, MA_OWNED);

    cd_mutex_unlock_double(cd0, cd1);

    f = ast_read(pbx_chan0);

    cd_mutex_lock_double(cd0, cd1);

    if ((cd0->pbx_chan != pbx_chan0) ||
	(cd1->pbx_chan != pbx_chan1))
    {
        /* call descriptor is not valid */

        if(f)
	{
	    ast_frfree(f);
	}
	f = NULL;
    }
    return f;
}

/*---------------------------------------------------------------------------*
 *      cd_pbx_write - write data to PBX
 *
 * cd1: second call descriptor in case of bridging
 *---------------------------------------------------------------------------*/
static int
cd_pbx_write(struct call_desc *cd0, struct call_desc *cd1, struct ast_frame *f)
{
    struct ast_channel *pbx_chan0 = cd0->pbx_chan;
    struct ast_channel *pbx_chan1 = cd1->pbx_chan;
    int error;

    cc_mutex_assert(&cd0->p_app->lock, MA_OWNED);
    cc_mutex_assert(&cd1->p_app->lock, MA_OWNED);

    cd_mutex_unlock_double(cd0, cd1);

    error = ast_write(pbx_chan0, f);

    cd_mutex_lock_double(cd0, cd1);

    if ((cd0->pbx_chan != pbx_chan0) ||
	(cd1->pbx_chan != pbx_chan1))
    {
        error = -1;
    }
    return error;
}

/*---------------------------------------------------------------------------*
 *      cd_root_shrink - free a call descriptor
 *---------------------------------------------------------------------------*/
static void
cd_root_shrink(struct call_desc *cd)
{
    if (cd->pbx_dsp) {
        ast_dsp_free(cd->pbx_dsp);
        cd->pbx_dsp = NULL;
    }

    free(cd);

    return;
}

/*---------------------------------------------------------------------------*
 *      cd_root_grow - allocate a new call descritor
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_root_grow(struct cc_capi_application *p_app)
{
    struct call_desc *cd;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    cd = (struct call_desc *)malloc(sizeof(*cd));

    if (cd) {

        bzero(cd, sizeof(*cd));

	cd->p_app = p_app;

	cd->pbx_dsp = ast_dsp_new();

	if (cd->pbx_dsp == NULL) {
	    cd_root_shrink(cd);
	    return NULL;
	}

	cd->fd[0] = -1;
	cd->fd[1] = -1;

	/* insert call descriptor into list */

	cd->next = p_app->cd_root_ptr;
	p_app->cd_root_ptr = cd;
	p_app->cd_root_allocated++;
    }
    return cd;
}

/*---------------------------------------------------------------------------*
 *      cd_by_plci - find call descriptor by CAPI PLCI
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_by_plci(struct cc_capi_application *p_app, u_int16_t plci)
{
    struct call_desc *cd;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

#if 0
    if ((plci & 0xFF00) == 0x0000) {

        /* this PLCI indicates that a controller
	 * is addressed, and not a call, and is
	 * therefore invalid
	 */
        return NULL;
    }
#endif

    cd = p_app->cd_root_ptr;

    while(cd) {

        if (cd->msg_plci == plci) {
	    break;
	}
        cd = cd->next;
    }

    return cd;
}

/*---------------------------------------------------------------------------*
 *      cd_by_plci_on_hold - find "on hold" call descriptor by CAPI PLCI
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_by_plci_on_hold(struct cc_capi_application *p_app, u_int16_t plci)
{
    struct call_desc *cd;

    cd = cd_by_plci(p_app, plci);

    if (cd && (!(cd->flags.hold_is_active))) {
        cd = NULL;
    }
    return cd;
}

/*---------------------------------------------------------------------------*
 *      pbx_chan_by_plci_on_hold - find "on hold" PBX channel by CAPI PLCI
 *---------------------------------------------------------------------------*/
static struct ast_channel *
pbx_chan_by_plci_on_hold(u_int16_t plci)
{
    struct call_desc *cd;
    struct ast_channel *pbx_chan = NULL;
    struct cc_capi_application *p_app;
    u_int16_t n;

    for(n = 0; n < CAPI_MAX_APPLICATIONS; n++) 
    {
        p_app = capi_application[n];

	if (p_app) {

	  cc_mutex_lock(&p_app->lock);

	  cd = cd_by_plci_on_hold(p_app, plci);

	  if (cd) {
	      pbx_chan = cd->pbx_chan;
	  }

	  cc_mutex_unlock(&p_app->lock);
	}

	if (pbx_chan) break;
    }
    return pbx_chan;
}

/*---------------------------------------------------------------------------*
 *	cd_by_pbx_chan - find "call descriptor" by PBX channel
 *
 * NOTE: returns a locked call descriptor
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_by_pbx_chan(struct ast_channel *pbx_chan)
{
#if 0

    /* in the future the PBX should lock our
     * private lock before calling any callbacks !
     */
    struct call_desc *cd = CC_CHANNEL_PVT(pbx_chan);

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    XXX chan_capi_fixup() should use:
    XXX struct call_desc *cd = CC_CHANNEL_PVT(newchan);

#else
    struct call_desc *cd = NULL;
    struct cc_capi_application *p_app;
    u_int16_t n;

    for(n = 0; n < CAPI_MAX_APPLICATIONS; n++) 
    {
        p_app = capi_application[n];

	if (p_app) {

	  cc_mutex_lock(&p_app->lock);

	  cd = p_app->cd_root_ptr;

	  while(cd) {

	    if ((cd->pbx_chan == pbx_chan) &&
		(!CD_IS_UNUSED(cd)))
	    {
	        goto found;
	    }
	    cd = cd->next;
	  }

	  cc_mutex_unlock(&p_app->lock);
	}
    }
 found:

    if ((cd == NULL) && (capi_global.debug)) {

        cc_log(LOG_ERROR, "PBX channel has no interface!\n");

    }
#endif

    return cd;
}


/*---------------------------------------------------------------------------*
 *      cd_by_msg_num - find call descriptor by message number
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_by_msg_num(struct cc_capi_application *p_app, u_int16_t msg_num)
{
    struct call_desc *cd;

    if (msg_num == 0x0000) {
        /* invalid message number */
        return NULL;
    }

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    cd = p_app->cd_root_ptr;

    while(cd) {

        if (cd->msg_num == msg_num) {
            break;
        }
        cd = cd->next;
    }

    return cd;
}

/*---------------------------------------------------------------------------*
 *      plci_to_controller - return CAPI controller by PLCI
 *---------------------------------------------------------------------------*/
static struct cc_capi_controller *
plci_to_controller(u_int8_t controller)
{
    return &capi_controller[(controller < CAPI_MAX_CONTROLLERS) ? 
			    controller : (CAPI_MAX_CONTROLLERS-1)];
}

/*---------------------------------------------------------------------------*
 *      cd_free_channel - free channel allocated by call descriptor, if any
 *---------------------------------------------------------------------------*/
static void
cd_free_channel(struct call_desc *cd)
{
    struct config_entry_iface *cep = cd->cep;

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    cc_mutex_lock(&capi_global_lock);

    if (cep) {

        if (cd->channel_type == 'B') {

	    cep->b_channels_curr++;

	} else if (cd->channel_type == 'D') {

	    cep->d_channels_curr++;

	} else if (cd->channel_type != 0x00) {

	    cc_log(LOG_ERROR, "invalid CAPI channel type: "
		   "0x%02x!\n", cd->channel_type);

	}
    } else {
        /* the configuration entry might have 
	 * disappeared due to a "chan_capi" reload  
	 */
    }

    cc_mutex_unlock(&capi_global_lock);

    cd->channel_type = 0x00;
    return;
}

/*---------------------------------------------------------------------------*
 *      cd_alloc_channel - allocate a channel for the given call descriptor
 *
 * Returns 0 on success.
 *
 * NOTE: must be called with "p_app->lock" locked
 *---------------------------------------------------------------------------*/
static u_int8_t
cd_alloc_channel(struct call_desc *cd, u_int8_t channel_type)
{
    struct config_entry_iface *cep = cd->cep;

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    cc_mutex_lock(&capi_global_lock);

    if (cd->channel_type != 0x00) {
        /* channel already allocated */
        goto error;
    }

    if (cep == NULL) {
        /* no configuration entry */
        goto error;
    }

    if (channel_type == 'B') {

      if (cep->b_channels_curr > 0) {
	  cep->b_channels_curr--;
      } else {
	  goto error;
      }

    } else if (channel_type == 'D') {

      if (cep->d_channels_curr > 0) {
	  cep->d_channels_curr--;
      } else {
	  goto error;
      }

    } else {
        cc_log(LOG_ERROR, "invalid CAPI channel type: "
	       "0x%02x!\n", channel_type);
        goto error;
    }

    cc_mutex_unlock(&capi_global_lock);
    cd->channel_type = channel_type;
    return 0;

 error:
    cc_mutex_unlock(&capi_global_lock);
    return 1;
}

/*---------------------------------------------------------------------------*
 *      cd_free - free a call descriptor 
 *
 * This is the function where all calls end up when
 * they are finished.
 *
 * hangup_what: 1 (hangup PBX)
 * hangup_what: 2 (free pbx_chan)
 *
 * NOTE: must be called with "p_app->lock" locked
 *---------------------------------------------------------------------------*/
static void
cd_free(struct call_desc *cd, u_int8_t hangup_what)
{
    struct ast_channel *pbx_chan = cd->pbx_chan;
    struct cc_capi_application *p_app = cd->p_app;
    u_int16_t wCause_in = cd->wCause_in;
    u_int8_t hard_hangup = ((cd->flags.pbx_started == 0) &&
			    (cd->flags.dir_outgoing == 0) &&
			    (hangup_what & 1));
    u_int8_t dir_outgoing = cd->flags.dir_outgoing;

    if (p_app == NULL) {
        /* should not happen */
        cc_log(LOG_ERROR, "cd->p_app == NULL!\n");
	return;
    }

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    cd_verbose(cd, 2, 1, 2, "\n");

    if (cd->flags.send_release_complete)
    {
        if ((cd->state == CAPI_STATE_ALERTING) ||
	    (cd->state == CAPI_STATE_DID) ||
	    (cd->state == CAPI_STATE_INCALL)) {

	    capi_send_connect_resp(cd, cd->wCause_out ? cd->wCause_out : 
				   /* ignore call */0x0001, NULL);
	}
	else
	{
	    capi_send_disconnect_req(cd);
	}
    }

    /* close data pipes, if any */

    if (cd->fd[0] > -1) {
        close(cd->fd[0]);
    }
    if (cd->fd[1] > -1) {
        close(cd->fd[1]);
    }

    cd->fd[0] = -1;
    cd->fd[1] = -1;

    /* close FAX file, if any */

    if (cd->fax_file) {

        /* check for zero length fax file */

	if (ftell(cd->fax_file) == 0) {
	    cd->flags.fax_error = 1;
	}

	fclose(cd->fax_file);
	cd->fax_file = NULL;

	if (cd->flags.fax_error) {

	    cd_verbose(cd, 2, 0, 1, "FAX receive failed, "
		       "reason=0x%04x, reasonB3=0x%04x\n",
		       cd->wCause_in, cd->wCause_in_b3);

	    if (cd->fax_fname) {
	        unlink(cd->fax_fname);
	    }

	} else {

	   cd_verbose(cd, 2, 0, 1, "FAX receive success!\n");
	}
    }

    if (cd->fax_fname) {
        free (cd->fax_fname);
	cd->fax_fname = NULL;
    }

    /* free allocated channel, if any */

    cd_free_channel(cd);

    /* clear the configuration entry pointer */

    cd_set_cep(cd, NULL);

    /* reset some variables to zero (all in one go) */

    bzero(&cd->dummy_zero_start[0], 
	  (&cd->dummy_zero_end[0] - &cd->dummy_zero_start[0]));


    /* update free statistic */

    if (p_app->cd_free_stats != 0xFFFFFFFF) {
        p_app->cd_free_stats++;
    }

    /* update use count */

    p_app->cd_root_used--;

    /* tell PBX to update use count */

    cc_mutex_lock(&capi_global_lock);
    update_use_count = 1;
    cc_mutex_unlock(&capi_global_lock);

    /* NOTE: one cannot call into Asterisk
     * while holding "p_app->lock", hence this
     * will cause a possible deadlock! See the
     * "LOCKING RULES". But one cannot unlock
     * "p_app->lock" either, hence "pbx_chan->lock"
     * might be held, and then there is a deadlock
     * also. This code will therefore just write
     * some values unlocked, in some cases.
     *
     * WARNING: Asterisk is not safe against
     * race conditions. But that is a problem
     * inside Asterisk. There is nothing one can
     * do about it until Asterisk changes its
     * routines.
     */
    if (pbx_chan)
    {
        if (dir_outgoing) {
	    cc_mutex_assert(&pbx_chan->lock, MA_OWNED);

	    ast_setstate(pbx_chan, AST_STATE_DOWN);

	    pbx_chan->hangupcause =
	      ((wCause_in & 0xFF00) == 0x3400) ?
	      (wCause_in & 0x7F) : AST_CAUSE_NORMAL_CLEARING;
	}

	if (hard_hangup) {
	    cd->hangup_chan = pbx_chan;
	}

	if (hangup_what & 2) {
	    cd->free_chan = pbx_chan;
	}

	/* else just wait for "ast_read()" to 
	 * return NULL
	 */
    }
    return;
}

/*---------------------------------------------------------------------------*
 *      cd_alloc - allocate a new call descriptor
 *
 * This is the function where all calls start. All calls are allocated from
 * a CAPI application! Upon success, a call descriptor pointer is returned. 
 * Else NULL is returned. This call descriptor pointer must be passed to 
 * "cd_free()" when the call is complete.
 *---------------------------------------------------------------------------*/
static struct call_desc *
cd_alloc(struct cc_capi_application *p_app, u_int16_t plci)
{
    struct ast_channel *pbx_chan = NULL;
    struct call_desc *cd = NULL;
    char buffer[16];
    int fds[2] = { 0, 0 };
    int fmt;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    /* try to handle telephony storms nicely */

    if (p_app->cd_alloc_rate_curr > p_app->cd_alloc_rate_max) {

        if (!(p_app->cd_alloc_rate_warned)) {
	      p_app->cd_alloc_rate_warned = 1;
	      cc_log(LOG_WARNING, "CAPI telephony storm detected "
		     "at %d calls/second! (ignored)\n",
		     p_app->cd_alloc_rate_curr);
	}
 	return NULL;
    }

    /* one more call, means that the call rate will increase */
    p_app->cd_alloc_rate_curr++;

    /* try to lookup a free call descriptor first */

    cd = p_app->cd_root_ptr;

    while (cd) {

        if (CD_IS_UNUSED(cd) && CD_NO_HANGUP(cd)) {
            break;
        }
        cd = cd->next;
    }

    if (cd == NULL) {

        /* try to allocate a new call descriptor */

        cd = cd_root_grow(p_app);

	if (cd == NULL) {
	    cc_log(LOG_WARNING, "could not allocate "
		   "memory for call descriptor!\n");
	    goto error;
	}
    }

    /* update use count first, hence it
     * will be decremented on "cd_free()"
     */
    p_app->cd_root_used++;

    /* keep some statistics */

    if (p_app->cd_alloc_stats != 0xFFFFFFFF) {
        p_app->cd_alloc_stats++;
    }

    /* tell PBX to update use count */

    cc_mutex_lock(&capi_global_lock);
    update_use_count = 1;
    cc_mutex_unlock(&capi_global_lock);

    /* try to allocate a pair of pipes */

    if (pipe(fds) != 0) {
        cc_log(LOG_ERROR, "Unable to create pipe\n");
	goto error;
    }

    cd->fd[0] = fds[0];
    cd->fd[1] = fds[1];


    /* try to allocate a PBX channel */

#if (CC_AST_VERSION >= 0x10400)
#if (CC_AST_VERSION >= 0x10403)
    pbx_chan = ast_channel_alloc(0, 0, 0, 0, "", "", "", 0, 0);
#else
    pbx_chan = ast_channel_alloc(0, 0, 0, 0, "");
#endif
#else
    pbx_chan = ast_channel_alloc(0);
#endif

    if (pbx_chan == NULL) {
        cc_log(LOG_ERROR, "Unable to allocate a PBX channel!\n");
	goto error;
    }

    cd->pbx_chan = pbx_chan;

#if (CC_AST_VERSION < 0x10400)
    pbx_chan->type                = chan_capi_pbx_type;
#endif
    pbx_chan->fds[0]              = fds[0];
    CC_CHANNEL_PVT(pbx_chan)      = cd;

    cc_mutex_lock(&capi_global_lock);
    fmt = capi_global.capability;
    cc_mutex_unlock(&capi_global_lock);

    cd->pbx_capability            = fmt;

#ifdef CC_OLD_CODEC_FORMATS
    pbx_chan->nativeformats       = fmt;
#else
    ast_codec_pref_init(&pbx_chan->nativeformats);
    ast_codec_pref_append(&pbx_chan->nativeformats, fmt);
#endif

    fmt = ast_best_codec(fmt);

#ifdef CC_AST_HAVE_SET_READ_FORMAT
    ast_set_read_format(pbx_chan, fmt);
#endif
#ifdef CC_AST_HAVE_SET_WRITE_FORMAT
    ast_set_write_format(pbx_chan, fmt);
#endif

    pbx_chan->readformat          = fmt; /* XXX cleanup */
    pbx_chan->writeformat         = fmt; /* XXX cleanup */

#ifdef CC_AST_HAVE_TECH_PVT
    pbx_chan->tech                = &chan_capi_tech;
    pbx_chan->rawreadformat       = fmt; /* XXX cleanup */
    pbx_chan->rawwriteformat      = fmt; /* XXX cleanup */
#else
    if (pbx_chan->pvt) {
        chan_capi_fill_pvt(pbx_chan);
    } else {
        cc_log(LOG_ERROR, "PVT structure not allocated!\n");
	goto error;
    }
    pbx_chan->pvt->rawreadformat  = fmt; /* XXX cleanup */
    pbx_chan->pvt->rawwriteformat = fmt; /* XXX cleanup */
#endif

    /* initialize call descriptor */

    cd->msg_plci = plci;
    cd->digit_time_last = p_app->application_uptime;
    cd->white_noise_rem = 1;
    cd->rx_time = ast_tvnow();

    cc_mutex_lock(&capi_global_lock);
    cd->support = plci_to_controller(plci)->support; /* copy support bits */
    cc_mutex_unlock(&capi_global_lock);

    if (plci < 0x0100)
    {
        /* outgoing call */

        cd->state = CAPI_STATE_DISCONNECTED;
	cd->flags.dir_outgoing = 1;
	cd->msg_num = get_msg_num_dial(cd->p_app);

	ast_setstate(pbx_chan, AST_STATE_RESERVED);
    }
    else
    {
        /* incoming call */

        cd->state = CAPI_STATE_DID;
	cd->flags.dir_outgoing = 0;

	ast_setstate(pbx_chan, AST_STATE_DOWN);

	snprintf(buffer, sizeof(buffer), "%d", plci);
	pbx_builtin_setvar_helper(pbx_chan, "PLCI", &buffer[0]);
    }
    return cd;

 error:

    if (pbx_chan) {
        if (cd) {
	    cd->pbx_chan = NULL;
	}
	ast_channel_free(pbx_chan);
    }

    if(cd) {
        cd_free(cd, 0);
    }
    return NULL;

}

/*---------------------------------------------------------------------------*
 *      cd_set_cep - set call descriptor configuration entry pointer
 *
 * returns 0 on success
 *---------------------------------------------------------------------------*/
static u_int8_t
cd_set_cep(struct call_desc *cd, struct config_entry_iface *cep)
{
    const struct cc_capi_options options_zero = { /* zero */ };
    u_int8_t channel_type = ((cd->bchannelinfo[0] != '0') && 
			     (cd->bchannelinfo[0] != 0)) ? 'D' : 'B';

    struct ast_channel *pbx_chan = cd->pbx_chan;
    struct ast_dsp *pbx_dsp = cd->pbx_dsp;

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    if(cd->cep) {
        /* cleanup */
        cd_free_channel(cd);
    }

    /* set new config entry pointer */
    cd->cep = cep;

    if(cep == NULL) {
        cd->options = options_zero;
	return 0;
    }

    if(cd_alloc_channel(cd, channel_type)) {
        cd->options = options_zero;
	return 1;
    }

    /* make a copy of the options */

    cd->options = cep->options;

    /* implication */

    if (cd->options.send_complete_force) {
        cd->flags.sending_complete_received = 1;
    }

    /* configure PBX channel, if present */

    if(pbx_chan && (cd->flags.dir_outgoing == 0)) {

	  pbx_chan->callgroup = cep->call_group;

	  strlcpy(pbx_chan->context, cep->context, 
		  sizeof(pbx_chan->context));

#if (CC_AST_VERSION >= 0x10400)
	  ast_string_field_set(pbx_chan, accountcode, cep->accountcode);
	  ast_string_field_set(pbx_chan, language, cep->language);
#else
	  strlcpy(pbx_chan->accountcode, cep->accountcode, 
		  sizeof(pbx_chan->accountcode));

	  strlcpy(pbx_chan->language, cep->language, 
		  sizeof(pbx_chan->language));
#endif
    }

    /* configure DSP, if present */

    if (pbx_dsp) {
        ast_dsp_set_features(pbx_dsp, DSP_FEATURE_DTMF_DETECT);
	if (cep->options.dtmf_detect_relax) {
	    ast_dsp_digitmode(pbx_dsp, DSP_DIGITMODE_DTMF | DSP_DIGITMODE_RELAXDTMF);
	} else {
	    ast_dsp_digitmode(pbx_dsp, DSP_DIGITMODE_DTMF);
	}
    }
    return 0;
}

/*---------------------------------------------------------------------------*
 *      cd_detect_dtmf - detect DTMF digits in the sound
 *---------------------------------------------------------------------------*/
static void
cd_detect_dtmf(struct call_desc *cd, int subclass, const void *__data, int len)
{
    struct ast_frame temp_fr;
    u_int8_t *input_data = (u_int8_t *)__data;
    u_int16_t *short_data;
    u_int16_t x;
    int digit;

    /* convert all sound to short data */

    switch(subclass) {
    case AST_FORMAT_SLINEAR:
        short_data = (void *)__data;
	break;

    case AST_FORMAT_ULAW:
        short_data = alloca(len * 2);

	for (x=0; x < len; x++) {
	    short_data[x] = AST_MULAW(input_data[x]);
	}
	len = len * 2;
	break;

    case AST_FORMAT_ALAW:
        short_data = alloca(len * 2);

	for (x=0; x < len; x++) {
	    short_data[x] = AST_ALAW(input_data[x]);
	}
	len = len * 2;
	break;

    default:
        cd_log(cd, LOG_WARNING, "Unknown voice frame "
	       "subclass: %d!\n", subclass);

        short_data = NULL;
	len = 0;
	break;
    }

    bzero(&temp_fr, sizeof(temp_fr));

    temp_fr.frametype = AST_FRAME_VOICE;
    temp_fr.subclass = AST_FORMAT_SLINEAR;
    temp_fr.data = short_data;
    temp_fr.datalen = len;
    temp_fr.samples = len / 2;

    digit = ast_dsp_digitdetect(cd->pbx_dsp, &temp_fr);

    if ((cd->last_dtmf_digit != digit) && digit) {

        bzero(&temp_fr, sizeof(temp_fr));

	temp_fr.frametype = AST_FRAME_DTMF;
	temp_fr.subclass = digit;

        cd_verbose(cd, 1, 1, 3, "Detected DTMF "
		   "digit: '%c'\n", digit);

	cd->tx_queue_len++;

	len = write(cd->fd[1], &temp_fr, sizeof(temp_fr));

	if (len < sizeof(temp_fr)) {

	    cd_log(cd, LOG_ERROR, "wrote %d bytes instead "
		   "of %d bytes!\n", len, (uint32_t)sizeof(temp_fr));

	}
    }
    cd->last_dtmf_digit = digit;
    return;
}

/*---------------------------------------------------------------------------*
 *      cd_send_pbx_voice - send voice to the PBX via pipe
 *
 * returns 0 on success
 *---------------------------------------------------------------------------*/
static int
cd_send_pbx_voice(struct call_desc *cd, const void *data_ptr, u_int32_t data_len)
{
    struct ast_channel *pbx_chan = cd->pbx_chan;
    struct ast_frame temp_fr;
    struct ast_frame copy_fr;
    int len = 0;

    bzero(&temp_fr, sizeof(temp_fr));

    temp_fr.frametype = AST_FRAME_VOICE;
    temp_fr.subclass = cd->pbx_capability;
    temp_fr.data = (void *)data_ptr;
    temp_fr.datalen = data_len;
    temp_fr.samples = data_len;
    temp_fr.offset = AST_FRIENDLY_OFFSET;

    bcopy(&temp_fr, &copy_fr, sizeof(copy_fr));

    cd_verbose(cd, 8, 1, 3, "temp_fr.datalen=%d, "
	       "temp_fr.subclass=%d\n", temp_fr.datalen, 
	       temp_fr.subclass);

    if (cd->fd[1] == -1) {
        cc_log(LOG_ERROR, "No pipe for %s\n",
	       pbx_chan->name);
	return -1;
    }

    if (cd->options.dtmf_detect_in_software &&
	(cd->pbx_dsp != NULL)) {

        cd_detect_dtmf(cd, 
		       temp_fr.subclass,
		       temp_fr.data, 
		       temp_fr.datalen);
    }

    if (cd->tx_queue_len < CAPI_MAX_QLEN) {

        cd->tx_queue_len++;

	/* compute correct delivery */

	cd->rx_time.tv_usec += (temp_fr.samples * 125);
	if (cd->rx_time.tv_usec >= 1000000) {
	    cd->rx_time.tv_usec -= 1000000;
	    cd->rx_time.tv_sec += 1;
	}

	temp_fr.delivery = cd->rx_time;

	len = write(cd->fd[1], &temp_fr, sizeof(temp_fr));

	if (len < sizeof(temp_fr)) {

	    cd_log(cd, LOG_ERROR, "wrote %d bytes instead "
		   "of %d bytes!\n", len, (uint32_t)sizeof(temp_fr));
	    return -1;
	}
    }
    return 0;
}

/*---------------------------------------------------------------------------*
 *      cd_send_pbx_frame - send a frame to the PBX via pipe
 *
 * returns 0 on success
 *---------------------------------------------------------------------------*/
static int 
cd_send_pbx_frame(struct call_desc *cd, int frametype, int subclass, 
		  const void *data, u_int16_t len)
{
    struct ast_channel *pbx_chan = cd->pbx_chan;
    struct ast_frame temp_fr;

    if (cd->fd[1] == -1) {
        cc_log(LOG_ERROR, "No pipe for %s\n",
	       pbx_chan->name);
	return -1;
    }

    if (frametype == AST_FRAME_VOICE) {
        return -1;
    }

    bzero(&temp_fr, sizeof(temp_fr));

    temp_fr.frametype = frametype;
    temp_fr.subclass = subclass;

    if (len) {
      
        temp_fr.data = malloc(len+AST_FRIENDLY_OFFSET);

	if (temp_fr.data) {

	    bcopy(data, ((u_int8_t *)temp_fr.data)+
		  AST_FRIENDLY_OFFSET, len);
	    temp_fr.offset = AST_FRIENDLY_OFFSET;
	    temp_fr.datalen = len;
	    temp_fr.mallocd = AST_MALLOCD_DATA;
	}
    }

    cd->tx_queue_len++;

    len = write(cd->fd[1], &temp_fr, sizeof(temp_fr));

    if (len != sizeof(temp_fr)) {

        cd_log(cd, LOG_ERROR, "wrote %d bytes instead "
	       "of %d bytes!\n", len, (uint32_t)sizeof(temp_fr));
	return -1;
    }
    return 0;
}

/*---------------------------------------------------------------------------*
 *      cd_handle_dst_telno - handle updates to the destination 
 *                            telephone number
 *
 * returns 0 on success
 *---------------------------------------------------------------------------*/
static int
cd_handle_dst_telno(struct call_desc *cd, const u_int8_t *p_dst_telno)
{
    int error = 0;

    if (cd->state != CAPI_STATE_DID) {
        cd_verbose(cd, 4, 1, 4, "Additional destination telephone "
		   "number digits ignored. Wrong state.\n");
	goto done;
    }

    if (p_dst_telno[0] == 0) {
        goto done;
    }

    strlcat(cd->dst_telno, p_dst_telno, sizeof(cd->dst_telno));

    cd->digit_time_last = cd->p_app->application_uptime;

    if (cd->pbx_chan->pbx) {

        /* the PBX has been started. Forward the digits as DTMF */

        while (p_dst_telno[0]) {

	    error = cd_send_pbx_frame(cd, AST_FRAME_DTMF, 
				      p_dst_telno[0], NULL, 0);

	    if(error) break;

	    p_dst_telno++;
	}
    }

 done:
    return error;
}

/*---------------------------------------------------------------------------*
 *      cd_send_pbx_progress - forward progress to the PBX
 *---------------------------------------------------------------------------*/
static void
cd_send_pbx_progress(struct call_desc *cd)
{
	if (cd->flags.dir_outgoing &&
	    cd->flags.progress_received &&
	    (cd->flags.progress_transmitted == 0)) {

	    cd->flags.progress_transmitted = 1;

	    /* connect B-channel */
	    capi_send_connect_b3_req(cd);

	    /* forward PBX signal */
	    cd_send_pbx_frame(cd, AST_FRAME_CONTROL, 
			      AST_CONTROL_PROGRESS, NULL, 0);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *      cd_handle_progress_indicator - handle progress indicators
 *
 * returns 0 on success
 *---------------------------------------------------------------------------*/
static int
cd_handle_progress_indicator(struct call_desc *cd, const u_int8_t prog_ind)
{
    switch(prog_ind) {
    case 0x01:
        cd_verbose(cd, 4, 1, 4, "Not end-to-end ISDN\n");
	break;
    case 0x02:
        cd_verbose(cd, 4, 1, 4, "Destination is non ISDN\n");
	cd->flags.dst_telno_is_not_isdn = 1;
	break;
    case 0x03:
        cd_verbose(cd, 4, 1, 4, "Origination is non ISDN\n");
	break;
    case 0x04:
        cd_verbose(cd, 4, 1, 4, "Call returned to ISDN\n");
	break;
    case 0x05:
        cd_verbose(cd, 4, 1, 4, "Interworking occured\n");
	break;
    case 0x08:
        cd_verbose(cd, 4, 1, 4, "In-band information available\n");
	cd->flags.progress_received = 1;

	if(cd->flags.b3_on_progress) {
	   cd_send_pbx_progress(cd);
	}
	break;
    default:
        cd_verbose(cd, 3, 1, 4, "Unknown progress "
		   "description: 0x%02x\n", prog_ind);
	break;
    }
    return 0;
}

static void
cd_set_fax_config(struct call_desc *cd, u_int16_t fax_format, 
		  const char *stationid, const char *headline)
{
#if (CAPI_OS_HINT == 0)
#warning "TODO: need to add support for capi_encode() to Linux!"
#else
	struct CAPI_B3_CONFIG_FAX_G3_DECODED b3_conf;

	cc_verbose(3, 1, VERBOSE_PREFIX_3 "Setup fax b3conf fmt=%d, stationid='%s' "
		   "headline='%s'\n", fax_format, stationid, headline);

	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	bzero(&b3_conf, sizeof(b3_conf));

	CAPI_INIT(CAPI_B3_CONFIG_FAX_G3, &b3_conf);

	b3_conf.wResolution = 0;
	b3_conf.wFormat = fax_format;
	b3_conf.CallingStationID.ptr = (void *)stationid;
	b3_conf.CallingStationID.len = strlen(stationid);
	b3_conf.Headline.ptr = (void *)headline;
	b3_conf.Headline.len = strlen(headline);

	cd->b3_config[0] = 
	  capi_encode(&cd->b3_config[1], sizeof(cd->b3_config)-1, &b3_conf);
#endif
	return;
}

/*===========================================================================*
 * CAPI helper functions to transfer CAPI messages
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *      capi_check_wait_get_cmsg - get a CAPI message from a CAPI application
 *
 * NOTE: must be called with "p_app->lock" locked
 * NOTE: this function can sleep 
 *---------------------------------------------------------------------------*/
static u_int16_t
capi_check_wait_get_cmsg(struct cc_capi_application *p_app, _cmsg *CMSG)
{
    struct timeval tv = { /* zero */ };
    u_int16_t error;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

 repeat:

    error = capi_get_cmsg(CMSG, p_app->application_id);

    if (error != 0x0000) {

        /* wait 1 second for a message */

        tv.tv_sec = 1;
	tv.tv_usec = 0;

	cc_mutex_unlock(&p_app->lock);

	error = capi20_waitformessage(p_app->application_id, &tv);

	cc_mutex_lock(&p_app->lock);

	if (error == 0x0000) {
	    goto repeat;
	}

	if (error != 0x1104) {
	    if (capi_global.debug) {
	        cc_log(LOG_DEBUG, "Error waiting for cmsg, "
		       "error = 0x%04x\n", error);
	    }
	}
    }

    return error;
}

/*---------------------------------------------------------------------------*
 *      __capi_put_cmsg - write a CAPI message to a CAPI application
 *
 * NOTE: must be called with "p_app->lock" locked
 *---------------------------------------------------------------------------*/
static u_int16_t 
__capi_put_cmsg(struct cc_capi_application *p_app, _cmsg *CMSG)
{
    u_int16_t error;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    error = capi20_put_cmsg(CMSG);
	
    if (error) {
        cc_log(LOG_ERROR, "CAPI error sending %s (NCCI=%#x) (error=%#x)\n",
	       capi_cmsg2str(CMSG), (u_int32_t)HEADER_CID(CMSG), error);
    } else {
        u_int16_t wCmd = HEADER_CMD(CMSG);
	if ((wCmd == CAPI_P_REQ(DATA_B3)) ||
	    (wCmd == CAPI_P_RESP(DATA_B3))) {
	    cc_verbose(9, 1, "%s\n", capi_cmsg2str(CMSG));
	} else {
	    cc_verbose(4, 1, "%s\n", capi_cmsg2str(CMSG));
	}
    }
    return error;
}

/*---------------------------------------------------------------------------*
 *      capi_do_monitor - CAPI "interrupt handler"
 *---------------------------------------------------------------------------*/
static void *
capi_do_monitor(void *data)
{
    struct cc_capi_application *p_app = data;
    u_int16_t error;
    _cmsg monCMSG;

    if (p_app == NULL) {
        cc_log(LOG_ERROR, "function argument is NULL!\n");
	return NULL;
    }

    cc_mutex_lock(&p_app->lock);

    while(1) {

        error = capi_check_wait_get_cmsg(p_app, &monCMSG);

	switch(error) {
	case 0x0000:
	    capi_handle_cmsg(p_app, &monCMSG);
	    break;

	case 0x1104:
	    /* CAPI queue is empty */
	    break;

	case 0x1101:
	    /* The application ID is no longer valid.
	     * This error is fatal, and "chan_capi" 
	     * should restart.
	     */
	    cc_log(LOG_ERROR, "Application ID is no longer valid! "
		   "CAPI monitor thread will terminate immediately!\n");
	    goto done;

	default:
	    /* something is wrong! */
	  break;
	} /* switch */
    } /* for */

 done:
    cc_mutex_unlock(&p_app->lock);
    return NULL;
}

static u_int16_t
capi_send_listen_req(struct cc_capi_application *p_app, 
		     u_int8_t controller, u_int32_t cip_mask)
{
    u_int16_t error;
    u_int16_t to = 0x80;
    _cmsg     CMSG;

    cc_mutex_assert(&p_app->lock, MA_OWNED);

    LISTEN_REQ_HEADER(&CMSG, p_app->application_id, 
		      get_msg_num_other(p_app), controller);

    LISTEN_REQ_INFOMASK(&CMSG) = 0xffff; /* lots of info ;) + early B3 connect */
			/* 0x00ff if no early B3 should be done */
		
    LISTEN_REQ_CIPMASK(&CMSG) = cip_mask;
    error = __capi_put_cmsg(p_app, &CMSG);

    if (error) {
        goto done;
    }

    p_app->received_listen_conf = 0;

    while((p_app->received_listen_conf == 0) && --to) {

        capi_application_usleep(p_app, 10000); /* sleep 10 ms */
    }

 done:
    return error;
}

static u_int16_t
capi_send_data_b3_req(struct call_desc *cd, u_int16_t wHandle,
                      void *data, u_int16_t wLen)
{
    _cmsg CMSG;

    DATA_B3_REQ_HEADER(&CMSG, cd->p_app->application_id, 
		       get_msg_num_other(cd->p_app), cd->msg_ncci);
    DATA_B3_REQ_DATALENGTH(&CMSG) = wLen;
    DATA_B3_REQ_FLAGS(&CMSG) = 0; 
    DATA_B3_REQ_DATAHANDLE(&CMSG) = wHandle;
    DATA_B3_REQ_DATA(&CMSG) = data;

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_connect_resp(struct call_desc *cd, u_int16_t wReject, 
		       const u_int16_t *p_bprot)
{
    _cmsg CMSG;
    char buf[CAPI_MAX_STRING];
    const char *p_dst_telno;
    struct tm tm;
    char tm_buf[7];
    int len = 0;
    time_t t;

    if ((cd->state != CAPI_STATE_ALERTING) &&
	(cd->state != CAPI_STATE_DID) &&
	(cd->state != CAPI_STATE_INCALL)) {

        return 0x2001; /* message not allowed now */
    }

    CONNECT_RESP_HEADER(&CMSG, cd->p_app->application_id, 
			cd->msg_num, cd->msg_plci);
    CONNECT_RESP_REJECT(&CMSG) = wReject;

    if (wReject == 0) {

        /* accept call */

        if (cd->dst_strip_len && 
	    (!cd->flags.connected_number_set)) {

	    /* strip digits at the beginning of
	     * the telephone number
	     */
	    len = strlen(cd->dst_telno);

	    if (len > cd->dst_strip_len) {
	        len = cd->dst_strip_len;
	    }
	}

	p_dst_telno = cd->dst_telno + len;

	len = strlen(p_dst_telno);
	if (len) {
	    buf[0] = len + 2;
	    buf[1] = 0x00;
	    buf[2] = 0x80;
	    strncpy(&buf[3], p_dst_telno, sizeof(buf) - 4);
	    CONNECT_RESP_CONNECTEDNUMBER(&CMSG) = (_cstruct)buf;

	    cd_verbose(cd, 3, 0, 2, "Connected to %s\n", p_dst_telno);
	}

	if (p_bprot) {

	  CONNECT_RESP_B1PROTOCOL(&CMSG) = p_bprot[0];
	  CONNECT_RESP_B2PROTOCOL(&CMSG) = p_bprot[1];
	  CONNECT_RESP_B3PROTOCOL(&CMSG) = p_bprot[2];
	}

	if (cd->b3_config[0]) {

	    CONNECT_RESP_B3CONFIGURATION(&CMSG) = (_cstruct)&cd->b3_config[0];
	}

	if (cd->options.ntmode) {

	    /* provide the time to 
	     * connected equipment:
	     */
	    t = time(NULL);

#if (CC_AST_VERSION >= 0x10408)
	    if (ast_localtime(&t, &tm, NULL)) {
#else
	    if (localtime_r(&t, &tm)) {
#endif
	        tm_buf[0] = 6;
		capi_put_1(tm_buf, 0, tm.tm_year % 100);
		capi_put_1(tm_buf, 1, tm.tm_mon + 1);
		capi_put_1(tm_buf, 2, tm.tm_mday);
		capi_put_1(tm_buf, 3, tm.tm_hour);
		capi_put_1(tm_buf, 4, tm.tm_min);
		capi_put_1(tm_buf, 5, tm.tm_sec);

		CONNECT_RESP_DATE_TIME(&CMSG) = (_cstruct)tm_buf;
	    }
	}

	cd->state = CAPI_STATE_ANSWERING;

    } else {

        /* reject or ignore call */

        CONNECT_RESP_BPROTOCOL(&CMSG) = CAPI_DEFAULT;
	CONNECT_RESP_ADDITIONALINFO(&CMSG) = CAPI_DEFAULT;
    }
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_connect_resp_app(struct cc_capi_application *p_app, 
			   uint16_t wMsgNum, uint16_t plci, 
			   uint16_t wReject)
{
    _cmsg CMSG;
    CONNECT_RESP_HEADER(&CMSG, p_app->application_id, 
			wMsgNum, plci);
    CONNECT_RESP_REJECT(&CMSG) = wReject;
    return __capi_put_cmsg(p_app, &CMSG);
}

static u_int16_t
capi_send_disconnect_req(struct call_desc *cd)
{
    _cmsg CMSG;

    cd_verbose(cd, 4, 1, 3, "\n");

    DISCONNECT_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			  get_msg_num_other(cd->p_app), cd->msg_plci);
    DISCONNECT_REQ_ADDITIONALINFO(&CMSG) = CAPI_DEFAULT;
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_connect_b3_req(struct call_desc *cd)
{
    _cmsg CMSG;

    if (cd->flags.b3_active || 
	cd->flags.b3_pending) {
        return 0;
    }

    cd->flags.b3_pending = 1;

    cd_verbose(cd, 4, 1, 3, "\n");

    CONNECT_B3_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			  get_msg_num_other(cd->p_app), cd->msg_plci);

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_info_digits(struct call_desc *cd, const char *digits, 
		      u_int16_t len)
{
    _cmsg CMSG;
    char buf[32];

    if (len > (sizeof(buf)-2)) {
        len = (sizeof(buf)-2);
    }

    buf[0] = len + 1;
    buf[1] = 0x80;
    while(len--) {
        cd_verbose(cd, 3, 0, 4, "digit[%d] = '%c'\n", 
		   len, digits[len]);

        buf[len + 2] = digits[len];
    }

    INFO_REQ_HEADER(&CMSG, cd->p_app->application_id, 
		    get_msg_num_other(cd->p_app), cd->msg_plci);
    INFO_REQ_CALLEDPARTYNUMBER(&CMSG) = (_cstruct)buf;

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_inband_digits(struct call_desc *cd, const char *digits, 
			u_int16_t len)
{
    _cmsg CMSG;
    char buf[32];

    if (len > (sizeof(buf)-8)) {
        len = (sizeof(buf)-8);
    }

    buf[0] = len + 7;
    capi_put_2(buf, 0, 3 /* send DTMF digit */);
    capi_put_2(buf, 2, CAPI_DTMF_DURATION);
    capi_put_2(buf, 4, CAPI_DTMF_DURATION);
    capi_put_1(buf, 6, len); /* struct size */

    while (len--) {
        cd_verbose(cd, 3, 0, 4, "digits[%d] = '%c'\n", 
		   len, digits[len]);

	capi_put_1(buf, 7 + len, digits[len]);
    }

    FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			get_msg_num_other(cd->p_app), cd->msg_plci);
    FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_DTMF;
    FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)buf;
        
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_alert_req(struct call_desc *cd, u_int8_t flag)
{
    _cmsg CMSG;

    if ((cd->state != CAPI_STATE_INCALL) &&
	(cd->state != CAPI_STATE_DID)) {
        cd_verbose(cd, 2, 1, 3, "Attempting ALERT in "
		   "state %d\n", cd->state);
	return 0;
    }

    ALERT_REQ_HEADER(&CMSG, cd->p_app->application_id, 
		     get_msg_num_other(cd->p_app), 
		     cd->msg_plci);

    if (flag & 1) {

        /* send CALL PROCEEDING instead of ALERT REQ, 
	 * to indicate end of overlap sending
	 */

        ALERT_REQ_SENDINGCOMPLETE(&CMSG) = 
	  (_cstruct)&sending_complete_struct[0];

	cd->state = CAPI_STATE_INCALL;

    } else {

        /* remote end is ringing */

        cd->state = CAPI_STATE_ALERTING;

	if (cd->pbx_chan->_state == AST_STATE_RING) {
	    cd->pbx_chan->rings = 1;
	}
    }
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_ect_req(struct call_desc *cd)
{
    _cmsg CMSG;
    char fac[8];

    fac[0] = 7;	/* msg len */
    capi_put_1(fac, 0, 0x06);	/* ECT (function) */
    capi_put_1(fac, 1, 0x00);
    capi_put_1(fac, 2, 0x04);	/* len */
    capi_put_4(fac, 3, cd->ect_plci);

    FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			get_msg_num_other(cd->p_app), cd->msg_plci);
    FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_SUPPLEMENTARY;
    FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)&fac;

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_disconnect_b3_req(struct call_desc *cd)
{
    _cmsg CMSG;

    DISCONNECT_B3_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			     get_msg_num_other(cd->p_app), 
			     cd->msg_ncci);
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_fac_suppl_req(struct call_desc *cd, u_int16_t wFunction)
{
    _cmsg CMSG;
    char fac[4];

    fac[0] = 3;	/* len */
    fac[1] = wFunction & 0xFF;
    fac[2] = (wFunction >> 8) & 0xFF;
    fac[3] = 0;	

    FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			get_msg_num_other(cd->p_app), cd->msg_plci);
    FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_SUPPLEMENTARY;
    FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)&fac;

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

/*
 * Echo cancellation is for cards with integrated
 * echo cancellation only, like Eicon active cards
 */
#define EC_FUNCTION_ENABLE              1
#define EC_FUNCTION_DISABLE             2
#define EC_FUNCTION_FREEZE              3
#define EC_FUNCTION_RESUME              4
#define EC_FUNCTION_RESET               5
#define EC_OPTION_DISABLE_NEVER         0
#define EC_OPTION_DISABLE_G165          (1<<2)
#define EC_OPTION_DISABLE_G164_OR_G165  ((1<<1) | (1<<2))
#define EC_DEFAULT_TAIL                 64

static u_int16_t
capi_send_echo_cancel_req(struct call_desc *cd, u_int16_t function)
{
    _cmsg CMSG;
    char buf[10];

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    if (cd->flags.disconnect_received ||
	(!cd->support.echo_cancel) || 
	(!cd->options.echo_cancel_in_hardware)) {
        return 0;
    }

    cd_verbose(cd, 2, 0, 2, "Setting up echo canceller, "
	       "function=%d, options=%d, tail=%d.\n",
	       function, cd->options.echo_cancel_option,
	       cd->options.echo_cancel_tail);

    bzero(buf, sizeof(buf));

    buf[0] = 9; /* msg size */

    capi_put_2(buf, 0, function);

    if (function == EC_FUNCTION_ENABLE) {

        /* echo cancel param struct size */
        capi_put_1(buf, 2, 6); 

	/* bit field - ignore echo canceller disable tone */
	capi_put_2(buf, 3, cd->options.echo_cancel_option); 

	/* tail length, ms */
	capi_put_2(buf, 5, cd->options.echo_cancel_tail);

	/* pre-delay length in milliseconds */
	capi_put_2(buf, 7, 0);
    }

    FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			get_msg_num_other(cd->p_app), cd->msg_plci);

    FACILITY_REQ_FACILITYSELECTOR(&CMSG) = cd->options.echo_cancel_selector;
    FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)buf;

    return __capi_put_cmsg(cd->p_app, &CMSG);
}

static u_int16_t
capi_send_detect_dtmf_req(struct call_desc *cd, u_int8_t flag)
{
    _cmsg CMSG;
    char buf[9];

    cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

    if (cd->flags.disconnect_received) {
        return 0;
    }

    cd_verbose(cd, 2, 0, 2, "Setting up DTMF "
	       "detector, flag=%d\n", flag);
	
    if (cd->support.dtmf && (!(cd->options.dtmf_detect_in_software))) {

        bzero(buf, sizeof(buf));

	buf[0] = 8; /* msg length */

	capi_put_2(buf, 0, flag ? 
		   /* start DTMF */ 1 : 
		   /* stop DTMF */ 2);

	capi_put_2(buf, 2, CAPI_DTMF_DURATION);
	capi_put_2(buf, 4, CAPI_DTMF_DURATION);

	FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			    get_msg_num_other(cd->p_app), cd->msg_plci);
	FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_DTMF;
	FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)buf;
        
	return __capi_put_cmsg(cd->p_app, &CMSG);
    } else {

        if (flag == 1)
	    cd->options.dtmf_detect_in_software = 1;
	else
	    cd->options.dtmf_detect_in_software = 0;
    }
    return 0;
}

static u_int16_t
capi_send_line_interconnect_req(struct call_desc *cd0, 
				struct call_desc *cd1, u_int8_t start)
{
    _cmsg CMSG;
    char buf[20];

    cc_mutex_assert(&cd0->p_app->lock, MA_OWNED);
    cc_mutex_assert(&cd1->p_app->lock, MA_OWNED);

    if ((cd0->flags.disconnect_received) ||
	(cd1->flags.disconnect_received)) {

        return 0;
    }

    if ((!(cd0->flags.b3_active)) || 
	(!(cd1->flags.b3_active))) {

        cd_verbose(cd0, 3, 1, 2, "line interconnect aborted. "
		   "At least one channel is not connected!\n");

	cd_verbose(cd1, 3, 1, 2, "line interconnect aborted. "
		   "At least one channel is not connected!\n");

	return 0x2001; /* message not allowed yet */
    }

    bzero(buf, sizeof(buf));

    if (start) {
        /* connect */

        cd0->flags.line_interconnect_active = 1;
	cd1->flags.line_interconnect_active = 1;

        buf[0] = 17; /* msg size */

	capi_put_2(buf,  0, 1);
	capi_put_1(buf,  2, 14); /* struct size LI Request Parameter */
	capi_put_4(buf,  3, 0);  /* Data Path */
	capi_put_1(buf,  7, 9);  /* struct size */
	capi_put_1(buf,  8, 8);  /* struct size LI Request Connect Participant */
	capi_put_4(buf,  9, cd1->msg_plci);
	capi_put_4(buf, 13, 3);  /* Data Path Participant */

    } else {
        /* disconnect */

        cd0->flags.line_interconnect_active = 0;
	cd1->flags.line_interconnect_active = 0;

        buf[0] = 7; /* msg size */

	capi_put_2(buf, 0, 2);
	capi_put_1(buf, 2, 4); /* struct size */
	capi_put_4(buf, 3, cd1->msg_plci);
    }

    FACILITY_REQ_HEADER(&CMSG, cd0->p_app->application_id, 
			get_msg_num_other(cd0->p_app), cd0->msg_plci);
    FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_LINE_INTERCONNECT;
    FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)buf;
        
    return __capi_put_cmsg(cd0->p_app, &CMSG);
}

static u_int16_t
capi_send_select_fax_prot_req(struct call_desc *cd) 
{
    _cmsg CMSG;

    if (cd->flags.b3_active || cd->flags.b3_pending) {

        capi_send_disconnect_b3_req(cd);

	cd->flags.fax_set_prot_on_b3_disc = 1;
	return 0;
    }

    SELECT_B_PROTOCOL_REQ_HEADER(&CMSG, cd->p_app->application_id, 
				 get_msg_num_other(cd->p_app), cd->msg_plci);
    SELECT_B_PROTOCOL_REQ_B1PROTOCOL(&CMSG) = 4;
    SELECT_B_PROTOCOL_REQ_B2PROTOCOL(&CMSG) = 4;
    SELECT_B_PROTOCOL_REQ_B3PROTOCOL(&CMSG) = 4;
    SELECT_B_PROTOCOL_REQ_B1CONFIGURATION(&CMSG) = NULL;
    SELECT_B_PROTOCOL_REQ_B2CONFIGURATION(&CMSG) = NULL;

    if (cd->b3_config[0]) {

        SELECT_B_PROTOCOL_REQ_B3CONFIGURATION(&CMSG) = (_cstruct)&cd->b3_config[0];
    }
    return __capi_put_cmsg(cd->p_app, &CMSG);
}

#ifdef CC_AST_CHANNEL_HAS_TRANSFERCAP
/*
 *  TCAP -> CIP Translation Table (TransferCapability->CommonIsdnProfile)
 */
static const struct {
	u_int16_t tcap;
	u_int16_t cip;
} translate_tcap2cip[] = {
	{ PRI_TRANS_CAP_SPEECH,                 CAPI_CIPI_SPEECH },
	{ PRI_TRANS_CAP_DIGITAL,                CAPI_CIPI_DIGITAL },
	{ PRI_TRANS_CAP_RESTRICTED_DIGITAL,     CAPI_CIPI_RESTRICTED_DIGITAL },
	{ PRI_TRANS_CAP_3K1AUDIO,               CAPI_CIPI_3K1AUDIO },
	{ PRI_TRANS_CAP_DIGITAL_W_TONES,        CAPI_CIPI_DIGITAL_W_TONES },
	{ PRI_TRANS_CAP_VIDEO,                  CAPI_CIPI_VIDEO }
};

static int
tcap2cip(u_int16_t tcap)
{
	int x;
	
	for (x = 0; x < sizeof(translate_tcap2cip) / sizeof(translate_tcap2cip[0]); x++) {
		if (translate_tcap2cip[x].tcap == tcap)
			return (int)translate_tcap2cip[x].cip;
	}
	return 0;
}

/*
 *  CIP -> TCAP Translation Table (CommonIsdnProfile->TransferCapability)
 */
static const struct {
	u_int16_t cip;
	u_int16_t tcap;
} translate_cip2tcap[] = {
	{ CAPI_CIPI_SPEECH,                  PRI_TRANS_CAP_SPEECH },
	{ CAPI_CIPI_DIGITAL,                 PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_RESTRICTED_DIGITAL,      PRI_TRANS_CAP_RESTRICTED_DIGITAL },
	{ CAPI_CIPI_3K1AUDIO,                PRI_TRANS_CAP_3K1AUDIO },
	{ CAPI_CIPI_7KAUDIO,                 PRI_TRANS_CAP_DIGITAL_W_TONES },
	{ CAPI_CIPI_VIDEO,                   PRI_TRANS_CAP_VIDEO },
	{ CAPI_CIPI_PACKET_MODE,             PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_56KBIT_RATE_ADAPTION,    PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_DIGITAL_W_TONES,         PRI_TRANS_CAP_DIGITAL_W_TONES },
	{ CAPI_CIPI_TELEPHONY,               PRI_TRANS_CAP_SPEECH },
	{ CAPI_CIPI_FAX_G2_3,                PRI_TRANS_CAP_3K1AUDIO },
	{ CAPI_CIPI_FAX_G4C1,                PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_FAX_G4C2_3,              PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_TELETEX_PROCESSABLE,     PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_TELETEX_BASIC,           PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_VIDEOTEX,                PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_TELEX,                   PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_X400,                    PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_X200,                    PRI_TRANS_CAP_DIGITAL },
	{ CAPI_CIPI_7K_TELEPHONY,            PRI_TRANS_CAP_DIGITAL_W_TONES },
	{ CAPI_CIPI_VIDEO_TELEPHONY_C1,      PRI_TRANS_CAP_DIGITAL_W_TONES },
	{ CAPI_CIPI_VIDEO_TELEPHONY_C2,      PRI_TRANS_CAP_DIGITAL }
};

static u_int16_t 
cip2tcap(int cip)
{
	int x;
	
	for (x = 0;x < sizeof(translate_cip2tcap) / sizeof(translate_cip2tcap[0]); x++) {
		if (translate_cip2tcap[x].cip == (u_int16_t)cip)
			return translate_cip2tcap[x].tcap;
	}
	return 0;
}

/*
 *  TransferCapability to String conversion
 */
static char *
transfercapability2str(int transfercapability)
{
	switch(transfercapability) {
	case PRI_TRANS_CAP_SPEECH:
		return "SPEECH";
	case PRI_TRANS_CAP_DIGITAL:
		return "DIGITAL";
	case PRI_TRANS_CAP_RESTRICTED_DIGITAL:
		return "RESTRICTED_DIGITAL";
	case PRI_TRANS_CAP_3K1AUDIO:
		return "3K1AUDIO";
	case PRI_TRANS_CAP_DIGITAL_W_TONES:
		return "DIGITAL_W_TONES";
	case PRI_TRANS_CAP_VIDEO:
		return "VIDEO";
	default:
		return "UNKNOWN";
	}
}
#endif /* CC_AST_CHANNEL_HAS_TRANSFERCAP */


/*===========================================================================*
 * exported PBX functions
 *===========================================================================*/

/* parse dialstring */

static void
parse_dialstring(char *buffer, const char **interface, const char **dest, 
		 const char **param, const char **ocid)
{
	char *ptr = buffer;
	char *oc;
	u_int8_t cp = 0;

	/* interface is the first part of the string */
	*interface = buffer;

	/* set default values */
	*dest = empty_string;
	*param = empty_string;
	*ocid = NULL;

	while (*ptr) {
	    if (*ptr == '/') {
	        *ptr = 0;
		 ptr++;

		 if (cp == 0) {
		     *dest = ptr;
		      cp++;
		 } else if (cp == 1) {
		     *param = ptr;
		      cp++;
		 } else {
		     cc_log(LOG_WARNING, "Too many parts "
			    "in dialstring '%s'\n", buffer);
		     break;
		 }
		 continue;
	    }
	    ptr++;
	}
	if ((oc = strchr(*dest, ':')) != NULL) {
		*ocid = *dest;
		*oc = '\0';
		*dest = oc + 1;
	}
	cc_verbose(3, 1, VERBOSE_PREFIX_4 "parsed dialstring: '%s' "
		   "'%s' '%s' '%s'\n", *interface, (*ocid) ? *ocid : 
		   "NULL", *dest, *param);
	return;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_request - called from "ast_request()"
 *
 * Prepare an outgoing call
 *---------------------------------------------------------------------------*/
static struct ast_channel *
#ifdef CC_OLD_CODEC_FORMATS
#ifdef CC_AST_HAVE_TECH_PVT
chan_capi_request(const char *type, int format, void *data, int *cause)
#else
chan_capi_request(char *type, int format, void *data)
#endif
#else
chan_capi_request(const char *type, const struct ast_codec_pref *formats, 
		  void *data, int *cause)
#endif
{
	struct call_desc *cd = NULL;
	struct ast_channel *pbx_chan = NULL;
	const char *dest;
	const char *interface;
	const char *param;
	const char *ocid;
	char buffer[CAPI_MAX_STRING];
	u_int32_t capigroup = 0;
	u_int8_t controller = 0xFF;
	struct config_entry_iface *cep;
	struct cc_capi_application *p_app = capi_application[0]; /* default */

	cc_verbose(1, 1, VERBOSE_PREFIX_4 "data = %s\n", (const char *)data);

	strlcpy(buffer, (const char *)data, sizeof(buffer));
	parse_dialstring(buffer, &interface, &dest, &param, &ocid);

	if ((!interface) || (!dest)) {
		cc_log(LOG_ERROR, "Syntax error in dialstring. "
		       "Read the documentation!\n");
#ifdef CC_AST_HAVE_TECH_PVT
		*cause = AST_CAUSE_INVALID_NUMBER_FORMAT;
#endif
		return NULL;
	}

	if (interface[0] == 'g') {
		capigroup = ast_get_group((char *)(interface + 1));
		cc_verbose(1, 1, VERBOSE_PREFIX_4 "capi request "
			   "group = %d\n", capigroup);

	} else if (!strncmp(interface, "contr", 5)) {

		controller = atoi(interface + 5);

		cc_verbose(1, 1, VERBOSE_PREFIX_4 "capi request "
			   "controller = %d\n", controller);
	} else {
		cc_verbose(1, 1, VERBOSE_PREFIX_4 "capi request for "
			   "interface '%s'\n", interface);
 	}

	/* have to lock the CAPI application first! */

	cc_mutex_lock(&p_app->lock);

	cep = cep_root_acquire();
	while(cep) {

	    if (cep->b_channels_curr > 0) {
	        if (controller < CAPI_MAX_CONTROLLERS) {
		    /* DIAL(CAPI/contrX/...) */
		    if(CC_GET_BIT(cep->controller_mask, controller)) {
		        break;
		    }
		} else {
		    /* DIAL(CAPI/gX/...) */
		    if (interface[0] == 'g') {
		        if(cep->group & capigroup) {
			    /* perform round robin on 
			     * the configuration entries:
			     */
			    cep_queue_last(cep);
			    break;
			}
		    } else {
		        /* DIAL(CAPI/<interface-name>/...) */
		        if (strcmp(cep->name, interface) == 0) {
			    break;
			}
		    }
		}
	    }
	    cep = cep->next;
	}

	if (cep == NULL) {
	    goto done;
	}

	if (controller == 0xFF) {
	    for(controller = 0; 
		controller < CAPI_MAX_CONTROLLERS; 
		controller++) {

	        if(CC_GET_BIT(cep->controller_mask, controller)) {
		    break;
		}
	    }
	}

	if (controller < CAPI_MAX_CONTROLLERS) {

	    cd = cd_alloc(p_app, controller & 0xFF);

	    if (cd == NULL) {
	        goto done;
 	    }

	    if (cd_set_cep(cd, cep)) {
	        cd_free(cd, 2);
		cd = NULL;
		goto done;
	    }

	    strlcpy(cd->dst_telno, dest, sizeof(cd->dst_telno));

	    pbx_chan = cd->pbx_chan;

	    /* set default channel name */

#if (CC_AST_VERSION >= 0x10400)
	    ast_string_field_build(pbx_chan, name,
				   "CAPI/%s/%s", cep->name, dest);
#else
	    snprintf(pbx_chan->name, sizeof(pbx_chan->name),
		     "CAPI/%s/%s", cep->name, dest);
#endif
	}

 done:
	cep_root_release();
	cc_mutex_unlock(&p_app->lock);

	if (cd) {
	    return pbx_chan;
	}

	cc_verbose(2, 0, VERBOSE_PREFIX_3 "didn't find CAPI device "
		   "for interface '%s' or out of memory!\n", interface);

#ifdef CC_AST_HAVE_TECH_PVT
	*cause = AST_CAUSE_REQUESTED_CHAN_UNAVAIL;
#endif
	return NULL;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_call - called from "ast_call()"
 *
 * Outgoing call from PBX goes here, after that "chan_capi_request()" 
 * has been called.
 *---------------------------------------------------------------------------*/
static int
__chan_capi_call(struct call_desc *cd, const char *idest, int timeout)
{
	static u_int8_t bc_bprot_alaw[] = { 0x05, 0x04, 0x03, 0x80, 0x90, 0xA3 };
	static u_int8_t bc_bprot_ulaw[] = { 0x05, 0x04, 0x03, 0x80, 0x90, 0xA2 };

	_cmsg CMSG;
	const char *dest;
	const char *interface;
	const char *param;
	const char *ocid;
	const char *ton;
	const char *p;
	struct ast_channel *pbx_chan = cd->pbx_chan;
	char dstring[AST_MAX_EXTENSION];
	char called[AST_MAX_EXTENSION];
	char calling[AST_MAX_EXTENSION];
	char second_calling[AST_MAX_EXTENSION];
	char callerid[AST_MAX_EXTENSION];
	char callingsubaddress[AST_MAX_EXTENSION];
	char calledsubaddress[AST_MAX_EXTENSION];
#ifdef CONNECT_REQ_DISPLAY
	char display[AST_MAX_EXTENSION];
#endif
	char useruser[AST_MAX_EXTENSION];
	u_int8_t buffer[8];
	u_int8_t bchaninfo[4];
	u_int8_t CLIR = 0;
	u_int8_t callernplan = 0;
	u_int8_t use_dst_default = 0;
	u_int8_t sending_complete = 1; /* default */

	strlcpy(dstring, idest, sizeof(dstring));
	parse_dialstring(dstring, &interface, &dest, &param, &ocid);

	/* parse the parameters */
	while ((param) && (*param)) {
		switch (*param) {
		case 'b':	/* early B3 on progress */
			cd->flags.b3_on_progress = 1;
			break;
		case 'B':	/* no early B3 on disconnect */
			cd->flags.want_late_inband = 0;
			break;
		case 'o':	/* overlap sending of digits */
			sending_complete = 0;
			break;
		case 'd':	/* use default cid */
			use_dst_default = 1;
			break;
		case 'l':	/* late inband signalling */
		       cd->flags.want_late_inband = 1;
		       break;
		case 'r':
			cd->flags.b3_on_alert = 1;
			break;

		default:
			cc_log(LOG_WARNING, "Unknown parameter '%c' in '%s', ignoring.\n",
				*param, idest);
		}
		param++;
	}
	if (!dest) {
		cc_log(LOG_ERROR, "No destination or "
		       "dialtone requested in '%s'\n", idest);
		return -1;
	}

#ifdef CC_AST_CHANNEL_HAS_CID
	CLIR = pbx_chan->cid.cid_pres;
	callernplan = pbx_chan->cid.cid_ton & 0x7f;
#else    
	CLIR = pbx_chan->callingpres;
#endif
	if ((ton = pbx_builtin_getvar_helper(pbx_chan, "CALLERTON"))) {
		callernplan = atoi(ton) & 0x7f;
	}

	cd_verbose(cd, 1, 1, 2, "Outgoing call to '%s', %s, %s, "
		   "sending%s complete, pres=0x%02x, ton=0x%02x\n",
		   dest, cd->flags.b3_on_progress ? "early-B3 on progress" : " ",
		   cd->flags.b3_on_alert ? "early-B3 on alert" : " ",
		   sending_complete ? "" : " not", CLIR, callernplan);

	/* set FD for PBX */
	pbx_chan->fds[0] = cd->fd[0];

	CONNECT_REQ_HEADER(&CMSG, cd->p_app->application_id, cd->msg_num, cd->msg_plci);
#ifdef CC_AST_CHANNEL_HAS_TRANSFERCAP
	CONNECT_REQ_CIPVALUE(&CMSG) = tcap2cip(pbx_chan->transfercapability);
#else
	CONNECT_REQ_CIPVALUE(&CMSG) = 0x10; /* Telephony */
#endif

	buffer[0] = 0x80;

	capi_build_struct(&called, sizeof(called), 
			  &buffer, 1, dest, strlen(dest), NULL, 0);

	CONNECT_REQ_CALLEDPARTYNUMBER(&CMSG) = (_cstruct)called;

	if ((p = pbx_builtin_getvar_helper(pbx_chan, "CALLEDSUBADDRESS"))) {

		buffer[0] = 0x80; /* number plan */

		capi_build_struct(&calledsubaddress, sizeof(calledsubaddress),
				  &buffer, 1, p, strlen(p), NULL, 0);

		CONNECT_REQ_CALLEDPARTYSUBADDRESS(&CMSG) = (_cstruct)calledsubaddress;
	}

#ifdef CC_AST_CHANNEL_HAS_CID
	if (pbx_chan->cid.cid_num) 
		strlcpy(callerid, pbx_chan->cid.cid_num, sizeof(callerid));
#else
	if (pbx_chan->callerid) 
		strlcpy(callerid, pbx_chan->callerid, sizeof(callerid));
#endif
	else
		callerid[0] = '\0';

	if (use_dst_default && cd->cep) {
		strlcpy(callerid, cd->cep->dst_default, sizeof(callerid));
	} else if (ocid) {
		strlcpy(callerid, ocid, sizeof(callerid));
	}

	buffer[0] = callernplan;
	buffer[1] = 0x80 | (CLIR & 0x63);

	capi_build_struct(&calling, sizeof(calling),
			  &buffer, 2, callerid, strlen(callerid), NULL, 0);

	CONNECT_REQ_CALLINGPARTYNUMBER(&CMSG) = (_cstruct)calling;

	if ((p = pbx_builtin_getvar_helper(pbx_chan, "CALLINGSUBADDRESS"))) {

	    buffer[0] = 0x80; /* number plan */

	    capi_build_struct(&callingsubaddress, sizeof(callingsubaddress),
			      &buffer, 1, p, strlen(p), NULL, 0);

	    CONNECT_REQ_CALLINGPARTYSUBADDRESS(&CMSG) = (_cstruct)callingsubaddress;
	}

#if ((CAPI_OS_HINT == 2) && (CAPI_STACK_VERSION >= 206))
	if ((p = pbx_builtin_getvar_helper(pbx_chan, "SECONDCALLERID"))) {

	    buffer[0] = callernplan;
	    buffer[1] = 0x80 | (CLIR & 0x63);

	    capi_build_struct(&second_calling, sizeof(second_calling),
			      &buffer, 2, p, strlen(p), NULL, 0);

	    CONNECT_REQ_SECONDCALLINGPARTYNUMBER(&CMSG) = (_cstruct)second_calling;
	}
#endif
#ifdef CONNECT_REQ_DISPLAY
	p = pbx_builtin_getvar_helper(pbx_chan, "DISPLAY");

	if (p == NULL) {
	    p = pbx_chan->cid.cid_name;
	}

	if (p) {
	    capi_build_struct(&display, sizeof(display), 
			      p, strlen(p), NULL, 0, NULL, 0);

	    CONNECT_REQ_DISPLAY(&CMSG) = (_cstruct)display;
	}
#else
#warning "The display variable is not supported by CAPI!"
#endif
	if ((p = pbx_builtin_getvar_helper(pbx_chan, "USERUSERINFO"))) {

	    capi_build_struct(&useruser, sizeof(useruser), 
			      p, strlen(p), NULL, 0, NULL, 0);

	    CONNECT_REQ_USERUSERDATA(&CMSG) = (_cstruct)useruser;
	}

	CONNECT_REQ_B1PROTOCOL(&CMSG) = 1;
	CONNECT_REQ_B2PROTOCOL(&CMSG) = 1;
	CONNECT_REQ_B3PROTOCOL(&CMSG) = 0;

	bchaninfo[0] = 0x2;
	bchaninfo[1] = 0x0;
	bchaninfo[2] = 0x0;
	CONNECT_REQ_BCHANNELINFORMATION(&CMSG) = (_cstruct)bchaninfo; /* 0 */

	CONNECT_REQ_SENDINGCOMPLETE(&CMSG) = (sending_complete) ?
	  (_cstruct)&sending_complete_struct[0] :
	  (_cstruct)&sending_not_complete_struct[0];

#if ((CAPI_OS_HINT == 2) && (CAPI_STACK_VERSION > 204))
	if(cd->flags.want_late_inband) {
	    CONNECT_REQ_FLAG0(&CMSG) = 0x01;
	}
#endif
	if (cd->pbx_capability == AST_FORMAT_ULAW)
	  CONNECT_REQ_BC(&CMSG) = bc_bprot_ulaw;
	else if (cd->pbx_capability == AST_FORMAT_ALAW)
	  CONNECT_REQ_BC(&CMSG) = bc_bprot_alaw;

	if (__capi_put_cmsg(cd->p_app, &CMSG)) {

	    /* cleanup by ast_hangup() */

	    return -1;
	}

	cd->state = CAPI_STATE_CONNECTPENDING;
	ast_setstate(pbx_chan, AST_STATE_DIALING);

	return 0;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_indicate - called from "ast_indicate()"
 *
 * NOTE: "chan_capi" currently depens on the CAPI controller 
 * to generate indications.
 *---------------------------------------------------------------------------*/
static int 
__chan_capi_indicate(struct call_desc *cd, int condition)
{
	int error = 0;

	switch (condition) {
	case AST_CONTROL_RINGING:
		cd_verbose(cd, 3, 1, 2, "Requested RINGING-Indication\n");

		/* TODO: somehow enable unhold on ringing, 
		 * but when wanted only 
		 */
		/* 
		if (cd->flags.hold_is_pending)
			chan_capi_cmd_retrieve(cd, NULL);
		*/

		capi_send_alert_req(cd, 0);
		break;

	case AST_CONTROL_BUSY:
		cd_verbose(cd, 3, 1, 2, "Requested BUSY-Indication\n");

		/* send a CONNECT_RESP and 
		 * wait for the DISCONNECT_IND 
		 */
		if (capi_send_connect_resp(cd, 0x0003, NULL)) {
		    error = -1;
		}
		break;

	case AST_CONTROL_CONGESTION:
		cd_verbose(cd, 3, 1, 2, "Requested CONGESTION-Indication\n");

		/* send a CONNECT_RESP and 
		 * wait for the DISCONNECT_IND 
		 */
		if(capi_send_connect_resp(cd, 0x0004, NULL)) {
		    error = -1;
		}
		break;

	case AST_CONTROL_PROGRESS:
		cd_verbose(cd, 3, 1, 2, "Requested PROGRESS-Indication\n");
		if (cd->options.ntmode) chan_capi_cmd_progress(cd, cd, NULL);
		break;

	case AST_CONTROL_PROCEEDING:
		cd_verbose(cd, 3, 1, 2, "Requested PROCEEDING-Indication\n");
		capi_send_alert_req(cd, 1);
		break;

#ifdef CC_AST_CONTROL_HOLD
	case AST_CONTROL_HOLD:
		cd_verbose(cd, 3, 1, 2, "Requested HOLD-Indication\n");
		if (cd->options.hold_type != CC_HOLDTYPE_LOCAL) {
		    error = chan_capi_cmd_hold(cd, cd, NULL);
		}
		break;

	case AST_CONTROL_UNHOLD:
		cd_verbose(cd, 3, 1, 2, "Requested UNHOLD-Indication\n");
		if (cd->options.hold_type != CC_HOLDTYPE_LOCAL) {
		    error = chan_capi_cmd_retrieve(cd, cd, NULL);
		}
		break;
#endif
	case -1: /* stop indications */
		cd_verbose(cd, 3, 1, 2, "Requested Indication-STOP\n");
		if (cd->flags.hold_is_pending) {
		    chan_capi_cmd_retrieve(cd, cd, NULL);
		}
		break;
	default:
		cd_verbose(cd, 3, 1, 2, "Requested unknown "
			   "indication: %d\n", condition);
		break;
	}
	return error;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_bridge - called from "ast_channel_bridge()"
 *
 * This function implements "native bridging" or "line interconnect"
 *---------------------------------------------------------------------------*/
static CC_BRIDGE_RETURN
__chan_capi_bridge(struct call_desc *cd0,
		   struct call_desc *cd1,
		   int flags, 
		   struct ast_frame **fo, 
		   struct ast_channel **rc
#ifdef CC_AST_BRIDGE_WITH_TIMEOUTMS
		   , int timeoutms
#endif
		   )
{
	CC_BRIDGE_RETURN ret = AST_BRIDGE_COMPLETE;
	struct ast_channel *pbx_chan0 = cd0->pbx_chan;
	struct ast_channel *pbx_chan1 = cd1->pbx_chan;
	u_int16_t waitcount = 20;

	cd_verbose(cd0, 3, 1, 2, "Native bridge!\n");
	cd_verbose(cd1, 3, 1, 2, "Native bridge!\n");

	if ((!cd0->options.bridge) || 
	    (!cd1->options.bridge) ||
	    (!cd0->support.lineinterconnect) ||
	    (!cd1->support.lineinterconnect)) {

	    return AST_BRIDGE_FAILED_NOWARN;
	}

	while (((!(cd0->flags.b3_active)) || 
		(!(cd1->flags.b3_active))) &&
	       (waitcount > 0)) {

		/* wait a moment for the B-channel to come up */

		if(cd_usleep(cd0, cd1, 20000)) {
		    return AST_BRIDGE_FAILED_NOWARN;
		}

		waitcount--;
	}

	if (!(flags & AST_BRIDGE_DTMF_CHANNEL_0)) {
		capi_send_detect_dtmf_req(cd0, 0);
	}

	if (!(flags & AST_BRIDGE_DTMF_CHANNEL_1)) {
		capi_send_detect_dtmf_req(cd1, 0);
	}

	capi_send_echo_cancel_req(cd0, EC_FUNCTION_DISABLE);
	capi_send_echo_cancel_req(cd1, EC_FUNCTION_DISABLE);
	
	if (capi_send_line_interconnect_req(cd0, cd1, 1)) {
		ret = AST_BRIDGE_FAILED;
		goto done;
	}

	for (;;) {
		struct ast_channel *c0_priority[2] = {pbx_chan0, pbx_chan1};
		struct ast_channel *c1_priority[2] = {pbx_chan1, pbx_chan0};
		int priority = 0;
		struct ast_frame *f;
		struct ast_channel *who;
#ifndef CC_AST_BRIDGE_WITH_TIMEOUTMS
		int timeoutms;

		timeoutms = -1;
#endif
		cd_mutex_unlock_double(cd0, cd1);

		who = ast_waitfor_n(priority ? c0_priority : c1_priority, 2, &timeoutms);

		cd_mutex_lock_double(cd0, cd1);

		if ((cd0->pbx_chan != pbx_chan0) ||
		    (cd1->pbx_chan != pbx_chan1))
		{
		    /* this call is gone */
		    return AST_BRIDGE_FAILED;
		}

		if (!who) {
#ifdef CC_AST_BRIDGE_WITH_TIMEOUTMS
			if (!timeoutms) {
				ret = AST_BRIDGE_RETRY;
				break;
			}
#else
			cc_log(LOG_DEBUG, "Ooh, empty read...\n");
#endif
			continue;
		}

		f = cd_pbx_read((who == pbx_chan0) ? cd0 : cd1,
				(who == pbx_chan0) ? cd0 : cd1);

		if (!f || (f->frametype == AST_FRAME_CONTROL)
		       || (f->frametype == AST_FRAME_DTMF)) {
			*fo = f;
			*rc = who;
			ret = AST_BRIDGE_COMPLETE;
			break;
		}

		/* NOTE: Sometimes AST_FRAME_NULL can be
		 * received here, and we don't want to
		 * forward such frames, hence they cause
		 * powerof(0) errors in "ast_translate_write()"
		 */

		if (f->frametype == AST_FRAME_VOICE) {
		    if (cd_pbx_write((who == pbx_chan0) ? cd1 : cd0,
				     (who == pbx_chan0) ? cd1 : cd0, f) < 0)
		    {
			ast_frfree(f);
			*fo = NULL;
			*rc = who;
			ret = AST_BRIDGE_COMPLETE;
			break;
		    }
		}

		ast_frfree(f);

		/* swap who gets priority */
		priority = !priority;
	}

	capi_send_line_interconnect_req(cd0, cd1, 0);

 done:

	if (!(flags & AST_BRIDGE_DTMF_CHANNEL_0)) {
		capi_send_detect_dtmf_req(cd0, 1);
	}

	if (!(flags & AST_BRIDGE_DTMF_CHANNEL_1)) {
		capi_send_detect_dtmf_req(cd1, 1);
	}

	capi_send_echo_cancel_req(cd0, EC_FUNCTION_ENABLE);
	capi_send_echo_cancel_req(cd1, EC_FUNCTION_ENABLE);

	return ret;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_answer - called from "ast_answer()"
 *---------------------------------------------------------------------------*/
static int
__chan_capi_answer(struct call_desc *cd)
{
	static const u_int16_t bprot[3] = { 1, 1, 0 };
	const char *temp;
	int error = 0;

	if ((cd->state == CAPI_STATE_ALERTING) ||
	    (cd->state == CAPI_STATE_DID) ||
	    (cd->state == CAPI_STATE_INCALL)) {

	    cc_mutex_assert(&cd->pbx_chan->lock, MA_OWNED);

	    /* copy in connected number, when the PBX 
	     * has the channel locked:
	     */
	    temp = pbx_builtin_getvar_helper(cd->pbx_chan, "CONNECTEDNUMBER");

	    if (temp) {
	        strlcpy(cd->dst_telno, temp, sizeof(cd->dst_telno));
		cd->flags.connected_number_set = 1;
	    }

	    if (capi_send_connect_resp(cd, 0, &bprot[0])) {
		error = -1;
	    }
	}
	return error;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_read - called from "ast_read()"
 *---------------------------------------------------------------------------*/
static struct ast_frame * 
__chan_capi_read(struct call_desc *cd)
{
	struct ast_frame * p_frame = NULL;
#if 0
	struct ast_channel * pbx_chan = cd->pbx_chan;
#endif
	int len;

	if (cd->state == CAPI_STATE_ONHOLD) {

	    goto done;
	}

 repeat:

	len = read(cd->fd[0], &cd->pbx_rd, sizeof(cd->pbx_rd));

	if (len < 0) {

	    goto done;

	} else if (len < sizeof(cd->pbx_rd)) {

	    goto repeat;
	}

	if (cd->pbx_rd.frametype == AST_FRAME_NULL) {

	    goto done;
	}

	if (cd->pbx_rd.frametype == AST_FRAME_VOICE) {

	    if(cd->tx_queue_len) {
	       cd->tx_queue_len--;
	    }
	}

	if ((cd->pbx_rd.frametype == AST_FRAME_DTMF) && 
	    (cd->pbx_rd.subclass == 'f')) {
#if 0
	    XXX This code is disabled because calling "ast_async_goto()"
	    XXX causes locking order reversal.
	    XXX TODO: Execute this code out of order

	    if (strcmp(pbx_chan->exten, "fax")) {
	      if (ast_exists_extension(pbx_chan, ast_strlen_zero(pbx_chan->macrocontext) ? 
				       pbx_chan->context : pbx_chan->macrocontext, 
				       "fax", 1, cd->src_telno)) {

		cd_verbose(cd, 2, 0, 3, "Redirecting to fax extension\n");

		/* save the DID/DNIS when we transfer the fax call to a "fax" extension */
		pbx_builtin_setvar_helper(pbx_chan, "FAXEXTEN", pbx_chan->exten);
		if (ast_async_goto(pbx_chan, pbx_chan->context, "fax", 1)) {
		  cc_log(LOG_WARNING, "Failed to async goto '%s' "
			 "into fax of '%s'\n", pbx_chan->name, pbx_chan->context);
		}
	      } else {
		cd_verbose(cd, 3, 0, 3, "Fax detected, but no fax extension\n");
	      }
	    } else {
	      cc_log(LOG_DEBUG, "Already in a fax extension, not redirecting\n");
	    }
#endif
	}
	p_frame = &cd->pbx_rd;

 done:
	return p_frame;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_write - called from "ast_write()"
 *
 * NOTE: if this function returns an error, then a softhangup
 * will be issued, which is not what one wants
 *---------------------------------------------------------------------------*/
static int
__chan_capi_write(struct call_desc *cd, struct ast_frame *frame)
{
	void *ptr;

	if ((!(cd->flags.b3_active)) || 
	    (!(cd->msg_ncci))) {
		goto done;
	}

	if ((!(cd->options.ntmode)) && 
	    (cd->state != CAPI_STATE_CONNECTED)) {
		goto done;
	}

	if (frame->frametype != AST_FRAME_VOICE) {

		if (frame->frametype == AST_FRAME_NULL) {
		    goto done;
		}

		if (frame->frametype == AST_FRAME_DTMF) {
		    cc_log(LOG_ERROR, "DTMF frame should be written\n");
		    goto done;
		}

		cd_verbose(cd, 3, 0, 2, "Not a voice frame, "
			   "frametype=%d\n", frame->frametype);
		goto done;
	}

	if (cd->flags.fax_receiving || 
	    cd->flags.fax_pending) {
		cd_verbose(cd, 3, 1, 2, "receiving a FAX, write ignored!\n");
		goto done;
	}

	if (frame->subclass != cd->pbx_capability) {
		cc_log(LOG_ERROR, "dont know how to write "
		       "subclass %d\n", frame->subclass);
		goto done;
	}

	if ((!frame->data) || 
	    (!frame->datalen)) {
		cd_verbose(cd, 3, 0, 2, "No data for voice frame\n");
		goto done;
	}

	ptr = alloca(frame->datalen);

	/* compute new amplitude */

	capi_copy_sound(frame->data, ptr, frame->datalen,
			cd->cep ? cd->cep->tx_convert : NULL);

	/* software echo suppression */

	if (cd->options.echo_suppress_in_software) {
	    soft_echo_suppress_process(cd, 
				       &cd->soft_es_tx, 
				       &cd->soft_es_rx, ptr, frame->datalen);
	}

	/* write data to ring buffer */

	buf_write_block(&(cd->ring_buf), ptr, frame->datalen);

 done:
	return 0;
}

/*---------------------------------------------------------------------------*
 *      __chan_capi_send_digit - called from "ast_write()" and "ast_senddigit()"
 *---------------------------------------------------------------------------*/
static int
__chan_capi_send_digit(struct call_desc *cd, const char digit)
{
	u_int8_t buf[16];

	buf[0] = digit;
	buf[1] = 0;

	if (cd->state == CAPI_STATE_CONNECTPENDING) {

		strlcat(cd->dst_telno, buf, sizeof(cd->dst_telno));

		if (capi_send_info_digits(cd, &digit, 1)) {
		    return -1;
		}
	}

	/* All DTMF sound should already be inband. 
	 * There is no need to generate DTMF!
	 *
	 * Else one ends up with duplicate DTMF
	 * tones.
	 */
	if(cd->options.dtmf_generate) {
		if (capi_send_inband_digits(cd, &digit, 1)) {
		    return -1;
		}
	}
	return 0;
}

/*===========================================================================*
 * wrapper functions for the exported PBX functions
 *
 * WARNING: The currently used callback model does not support private, 
 * per channel mutexes. The private per channel mutex is therefore locked
 * by the wrapper function, and some extra checking is performed to see
 * if the call did not disappear during an unlocked period. The unlocked
 * period arises when "chan_capi" wants to call the PBX. The reason is 
 * that the PBX uses one mutex to protect the channel and "chan_capi" 
 * uses another. To avoid deadlock, "chan_capi" must unlock its private, 
 * mutex, before calling the PBX. The correct is the opposite: The PBX 
 * should call all "chan_capi" callbacks with the private, 
 * per channel mutex, provied by "chan_capi".
 *
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *      chan_capi_call - called from "ast_call()"
 *
 * Outgoing call from PBX goes here, after that "chan_capi_request()" 
 * has been called.
 *---------------------------------------------------------------------------*/
static int
chan_capi_call(struct ast_channel *pbx_chan, char *idest, int timeout)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    int error = 0;

    if (cd == NULL) {
	return -1;
    }

    cd_verbose(cd, 3, 0, 2, "\n");

    /* call descriptor is still valid */
    error = __chan_capi_call(cd,idest,timeout);

    cd_unlock(cd);
    return error;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_fixup - called from "ast_do_masquerade()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_fixup(struct ast_channel *oldchan, struct ast_channel *newchan)
{
    struct call_desc *cd = cd_by_pbx_chan(oldchan);

    if (cd == NULL) {
	/* should not happen */
	return -1;
    }

    cd_verbose(cd, 3, 1, 2, "old channel = %s\n",
	       oldchan->name);

    cd->pbx_chan = newchan;

    cd_unlock(cd);

    return 0;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_indicate - called from "ast_indicate()"
 *---------------------------------------------------------------------------*/
static int
#if (CC_AST_VERSION >= 0x10400)
chan_capi_indicate(struct ast_channel *pbx_chan, int condition, 
		   const void *data, size_t datalen)
#else
chan_capi_indicate(struct ast_channel *pbx_chan, int condition)
#endif
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    int error = 0;

    if (cd == NULL) {
        goto done;
    }

    cd_verbose(cd, 3, 0, 2, "\n");

    /* call descriptor is still valid */
    error = __chan_capi_indicate(cd,condition);

    cd_unlock(cd);

 done:
    return error;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_bridge - called from "ast_channel_bridge()"
 *---------------------------------------------------------------------------*/
static CC_BRIDGE_RETURN
chan_capi_bridge(struct ast_channel *pbx_chan0,
		 struct ast_channel *pbx_chan1,
		 int flags, 
		 struct ast_frame **fo,
		 struct ast_channel **rc
#ifdef CC_AST_BRIDGE_WITH_TIMEOUTMS
		 , int timeoutms
#endif
		 )
{
    struct call_desc *cd0;
    struct call_desc *cd1;
    int error = AST_BRIDGE_COMPLETE;

    if(cd_mutex_lock_double_pbx_chan(pbx_chan0,pbx_chan1,&cd0,&cd1)) {
	return AST_BRIDGE_FAILED_NOWARN;
    }

    cd_verbose(cd0, 3, 0, 2, "\n");
    cd_verbose(cd1, 3, 0, 2, "\n");

    /* call descriptors are still valid */
    error = __chan_capi_bridge(cd0,cd1,flags,fo,rc
#ifdef CC_AST_BRIDGE_WITH_TIMEOUTMS
			       , timeoutms
#endif
			       );
    cd_mutex_unlock_double(cd0, cd1);
    return error;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_answer - called from "ast_answer()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_answer(struct ast_channel *pbx_chan)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    int error = 0;

    if (cd == NULL) {
        goto done;
    }

    cd_verbose(cd, 3, 0, 2, "\n");

    /* call descriptor is still valid */

    error = __chan_capi_answer(cd);

    cd_unlock(cd);

 done:
    return error;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_hangup - called from "ast_hangup()"
 *
 * NOTE: this function handles passive and active disconnect
 *---------------------------------------------------------------------------*/
static int 
chan_capi_hangup(struct ast_channel *pbx_chan)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    const char *cause;

    if (cd == NULL) {

        /* clear pointer to PVT */
        CC_CHANNEL_PVT(pbx_chan) = NULL;
	return 0;
    }

    cd_verbose(cd, 3, 0, 2, "\n");

    cc_mutex_assert(&pbx_chan->lock, MA_OWNED);

    cause = pbx_builtin_getvar_helper(pbx_chan, "PRI_CAUSE");

    /* get the hangup cause */

    cd->wCause_out =
      (((cause) ? atoi(cause) : pbx_chan->hangupcause) & 0x7F) | 0x3480;

    /* call descriptor is still valid */

    cd_free(cd, 0);

    /* IMPORTANT: clear pointer to PVT so 
     * that blocked threads, waiting to lock
     * this call descriptor will see that 
     * the call is gone, and not continue:
     */
    CC_CHANNEL_PVT(pbx_chan) = NULL;

    cd_unlock(cd);

    return 0;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_read - called from "ast_read()"
 *
 * NOTE: when "chan_capi_read" returns a NULL frame, then
 * the channel is hung up.
 *---------------------------------------------------------------------------*/
static struct ast_frame * 
chan_capi_read(struct ast_channel *pbx_chan)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    struct ast_frame *p_frame = NULL;

    if (cd == NULL) {
        goto done;
    }

    cd_verbose(cd, 3, 1, 2, "\n");

    /* call descriptor is still valid */

    p_frame = __chan_capi_read(cd);

    cd_unlock(cd);

 done:

    return p_frame;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_write - called from "ast_write()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_write(struct ast_channel *pbx_chan, struct ast_frame *p_frame)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    int error = 0;

    if (cd == NULL) {
        goto done;
    }

    cd_verbose(cd, 3, 1, 2, "\n");

    /* call descriptor is still valid */

    error = __chan_capi_write(cd, p_frame);

    cd_unlock(cd);

 done:
    return error;
}

#if (CC_AST_VERSION >= 0x10400)
/*---------------------------------------------------------------------------*
 *      chan_capi_send_digit_begin - not needed
 *---------------------------------------------------------------------------*/
static int
chan_capi_send_digit_begin(struct ast_channel *pbx_chan, const char digit)
{
    return 0;
}
#endif

/*---------------------------------------------------------------------------*
 *      chan_capi_send_digit - called from "ast_write()" and "ast_senddigit()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_send_digit(struct ast_channel *pbx_chan, const char digit)
{
    struct call_desc *cd = cd_by_pbx_chan(pbx_chan);
    int error = 0;

    if (cd == NULL) {
        goto done;
    }

    cd_verbose(cd, 3, 0, 2, "\n");

    /* call descriptor is still valid */

    error = __chan_capi_send_digit(cd,digit);

    cd_unlock(cd);

 done:
    return error;
}

#if (CC_AST_VERSION >= 0x10400)
/*---------------------------------------------------------------------------*
 *      chan_capi_send_digit_end - called from "ast_write()" and "ast_senddigit()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_send_digit_end(struct ast_channel *pbx_chan, const char digit,
			 unsigned int duration)
{
    return chan_capi_send_digit(pbx_chan, digit);
}
#endif

#ifndef CC_AST_NO_DEVICESTATE
/*---------------------------------------------------------------------------*
 *      chan_capi_devicestate - called from "ast_devicstate()"
 *---------------------------------------------------------------------------*/
static int
chan_capi_devicestate(void *data)
{
	if (data == NULL) {
	    cc_verbose(3, 1, VERBOSE_PREFIX_2 "No data\n");
	} else {
	    cc_verbose(3, 1, VERBOSE_PREFIX_4 "Data = '%s'\n", (const char *)data);
	}
	return AST_DEVICE_UNKNOWN;
}
#endif

/*
 * fax guard tone -- Handle and return NULL
 */
static void
capi_handle_dtmf_fax(struct call_desc *cd)
{
	struct ast_channel *pbx_chan = cd->pbx_chan;

	if (cd->flags.fax_handled) {
		cc_log(LOG_DEBUG, "Fax already handled\n");
		return;
	}
	
	cd->flags.fax_handled = 1;
	
	if (!strcmp(pbx_chan->exten, "fax")) {
		cc_log(LOG_DEBUG, "Already in a FAX "
		       "extension, not redirecting\n");
		return;
	}

	if (!ast_exists_extension(pbx_chan, pbx_chan->context, "fax", 1, 
				  cd->src_telno)) {
		cc_verbose(3, 0, VERBOSE_PREFIX_3 "Fax tone detected, "
			   "but no fax extension for %s\n", 
			   pbx_chan->name);
		return;
	}

	cd_verbose(cd, 2, 0, 3, "Redirecting call "
		   "to FAX extension\n");
			
	/* Save the DID/DNIS when we transfer the fax call to a "fax" extension */
	pbx_builtin_setvar_helper(pbx_chan, "FAXEXTEN", pbx_chan->exten);
	
	if (ast_async_goto(pbx_chan, pbx_chan->context, "fax", 1)) {
	    cc_log(LOG_WARNING, "Failed to async goto '%s' "
		   "into fax of '%s'\n", pbx_chan->name, 
		   pbx_chan->context);
	}
	return;
}

/*
 * send control according to cause code
 */
static u_int16_t
cd_send_pbx_cause_control(struct call_desc *cd, u_int8_t control)
{
	int cause = cd->pbx_chan->hangupcause;

	if (control) {

	  cause = ((cause == AST_CAUSE_NORMAL_CIRCUIT_CONGESTION) ||
		   (cd->wCause_in == 0x34a2)) ?
	      AST_CONTROL_CONGESTION : AST_CONTROL_BUSY;

	} else {

	    cause = 0;
	}

	return cd_send_pbx_frame(cd, control ? AST_FRAME_CONTROL : 
				 AST_FRAME_NULL, cause, NULL, 0);
}

/*
 * Disconnect via INFO_IND
 */
static void
handle_info_disconnect(_cmsg *CMSG, struct call_desc *cd)
{
	cd_verbose(cd, 4, 1, 3, "\n");

	cd->flags.disconnect_received = 1;

	if (cd->flags.ect_pending) {
		cd_verbose(cd, 4, 1, 3, "Disconnect ECT call.\n");

		/* just wait for DISCONNECT_IND */
		return;
	}

	if (cd->flags.hold_is_active) {
		cd_verbose(cd, 4, 1, 3, "Disconnect onhold call.\n");

		/* send a DISCONNECT_REQ, and wait for DISCONNECT_IND */

		capi_send_disconnect_req(cd);

		return;
	}

	if(cd->flags.want_late_inband) {
#warning "Should send a messages, but which?"
		return;
	}

	if (cd->flags.dir_outgoing) {

		cd_send_pbx_cause_control(cd, 1);

	} else {

		/* just hangup */

		capi_send_disconnect_req(cd);
	}
	return;
}

/*
 * CAPI INFO_IND
 */
static void
capi_handle_info_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	const char *desc = NULL;

	u_int8_t exten_buf[AST_MAX_EXTENSION];
	u_int8_t reason_buf[16];
	u_int8_t ie_buf[8];
	u_int8_t x;
	_cmsg CMSG2;

	INFO_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
			 HEADER_MSGNUM(CMSG), cd->msg_plci);
	__capi_put_cmsg(cd->p_app, &CMSG2);

	/* get the first couple of bytes 
	 * into a buffer:
	 */
	for(x = 0; x < sizeof(ie_buf); x++) {
	    ie_buf[x] = capi_get_1(INFO_IND_INFOELEMENT(CMSG), x);
	}

	switch(INFO_IND_INFONUMBER(CMSG)) {
	case 0x0008:	/* Cause */
		cd_verbose(cd, 3, 1, 3, "CAUSE 0x%02x 0x%02x\n", 
			   ie_buf[0], ie_buf[1]);

		x = (ie_buf[1] & 0x7f);

		/* When a call has ended and the Network side
		 * wants to play back sound, the correct is to
		 * send a DISCONNECT_INDICATION message with a
		 * PROGRESS_INDICATOR.
		 *
		 * The following code tries to handle at least
		 * one case, where PROGRESS is received with
		 * cause equal to "User Busy".
		 */
		if (!(cd->flags.want_late_inband)) {

		    if (x == 0x11) {
		        cd_send_pbx_frame(cd, AST_FRAME_NULL, 0, NULL, 0);
		    }
		}
		cd->wCause_in = 0x3400 | x;
		break;

	case 0x0014:	/* Call State */
		cd_verbose(cd, 3, 1, 3, "CALL STATE 0x%02x\n",
			   ie_buf[0]);
		break;

	case 0x0018:	/* Channel Identification */
		cd_verbose(cd, 3, 1, 3, "CHANNEL IDENTIFICATION "
			   "0x%02x\n", ie_buf[0]);
		break;

	case 0x001c:	/*  Facility Q.932 */
		cd_verbose(cd, 3, 1, 3, "FACILITY\n");
		break;

	case 0x001e:	/* Progress Indicator */
		cd_verbose(cd, 3, 1, 3, "PI 0x%02x 0x%02x 0x%02x\n",
			   ie_buf[0], ie_buf[1], ie_buf[2]);

		x = (ie_buf[2] & 0x7F);

		cd_handle_progress_indicator(cd, x);
		break;

	case 0x0027:	/*  Notification Indicator */

		switch (ie_buf[0]) {
		case 0:
		    desc = "User suspended";
		    break;
		case 1:
		    desc = "User resumed";
		    break;
		case 2:
		    desc = "Bearer service changed";
		    break;
		default:
		    desc = "Unknown";
		    break;
		}

		cd_verbose(cd, 3, 1, 3, "NOTIFICATION INDICATOR '%s' (0x%02x)\n",
			   desc, ie_buf[0]);
		break;

	case 0x0028:	/* Display */
		cd_verbose(cd, 3, 1, 3, "Display\n");

		capi_get_multi_1(INFO_IND_INFOELEMENT(CMSG), 0, 
				 &exten_buf, sizeof(exten_buf));
#if 0
		cc_log(LOG_NOTICE, "%s\n", exten_buf);
		cd_send_pbx_frame(cd, AST_FRAME_TEXT, 0, 
				  &exten_buf, strlen(exten_buf));
#endif
		break;

	case 0x0029:	/* Date/Time */
		cd_verbose(cd, 3, 1, 3, "Date/Time %02d/%02d/%02d %02d:%02d\n",
			   ie_buf[0], ie_buf[1], ie_buf[2], ie_buf[3], ie_buf[4]);
		break;

	case 0x0070:	/* Called Party Number */
		capi_get_multi_1(INFO_IND_INFOELEMENT(CMSG), 1, 
				 &exten_buf, sizeof(exten_buf));

		cd_verbose(cd, 3, 1, 3, "CALLED PARTY NUMBER '%s'\n", 
			   exten_buf);

		if (cd->flags.received_setup) {
		    cd_handle_dst_telno(cd, exten_buf);
		}
		break;

	case 0x0074:	/* Redirecting Number */

		capi_get_multi_1(INFO_IND_INFOELEMENT(CMSG), 3, 
				 &exten_buf, sizeof(exten_buf));

		x = (ie_buf[2] & 0x0f);

		snprintf(reason_buf, sizeof(reason_buf), "%d", x); 

		cd_verbose(cd, 3, 1, 3, "REDIRECTING NUMBER '%s', "
			   "Reason=0x%02x\n", exten_buf, x);

#warning "Cannot do this, because one cannot lock 'pbx_chan' here!"
#if 0
		pbx_builtin_setvar_helper(pbx_chan, "REDIRECTINGNUMBER", exten_buf);
		pbx_builtin_setvar_helper(pbx_chan, "REDIRECTREASON", reason_buf);

#ifdef CC_AST_CHANNEL_HAS_CID
		if (pbx_chan->cid.cid_rdnis) {
		    free(pbx_chan->cid.cid_rdnis);
		}
		pbx_chan->cid.cid_rdnis = strdup(exten_buf);
#else
		if (pbx_chan->rdnis) {
		    free(pbx_chan->rdnis);
		}
		pbx_chan->rdnis = strdup(exten_buf);
#endif
#endif
		break;

	case 0x00a1:	/* Sending Complete */
		cd_verbose(cd, 3, 1, 3, "Sending Complete\n");
		cd->flags.sending_complete_received = 1;
		break;

	case 0x4000:	/* CHARGE in UNITS */
		cd_verbose(cd, 3, 1, 3, "CHARGE in UNITS\n");
		break;

	case 0x4001:	/* CHARGE in CURRENCY */
		cd_verbose(cd, 3, 1, 3, "CHARGE in CURRENCY\n");
		break;

	case 0x8001:	/* ALERTING */
		if (cd->flags.b3_on_alert) {
		    cd_send_pbx_progress(cd);
		}
		cd_verbose(cd, 3, 1, 3, "ALERTING\n");
		cd_send_pbx_frame(cd, AST_FRAME_CONTROL, 
				  AST_CONTROL_RINGING, NULL, 0);
		break;

	case 0x8002:	/* CALL PROCEEDING */
		cd_verbose(cd, 3, 1, 3, "CALL PROCEEDING\n");
		cd_send_pbx_frame(cd, AST_FRAME_CONTROL,
				  AST_CONTROL_PROCEEDING, NULL, 0);
		break;

	case 0x8003:	/* PROGRESS */
		cd_verbose(cd, 3, 1, 3, "PROGRESS\n");
		break;

	case 0x8005:	/* SETUP */
		cd_verbose(cd, 3, 1, 3, "SETUP\n");
		cd->flags.received_setup = 1;
		break;

	case 0x8007:	/* CONNECT */
		cd_verbose(cd, 3, 1, 3, "CONNECT\n");
		break;

	case 0x800d:	/* SETUP ACK */
		cd_verbose(cd, 3, 1, 3, "SETUP ACK\n");
		break;

	case 0x800f:	/* CONNECT ACK */
		cd_verbose(cd, 3, 1, 3, "CONNECT ACK\n");
		break;

	case 0x8045:	/* DISCONNECT */
		cd_verbose(cd, 3, 1, 3, "DISCONNECT\n");
		handle_info_disconnect(CMSG, cd);
		break;

	case 0x804d:	/* RELEASE */
		cd_verbose(cd, 3, 1, 3, "RELEASE\n");
		break;

	case 0x805a:	/* RELEASE COMPLETE */
		cd_verbose(cd, 3, 1, 3, "RELEASE COMPLETE\n");
		break;

	case 0x8062:	/* FACILITY */
		cd_verbose(cd, 3, 1, 3, "FACILITY\n");
		break;

	case 0x806e:	/* NOTIFY */
		cd_verbose(cd, 3, 1, 3, "NOTIFY\n");

		switch(ie_buf[1]) {
#ifdef CC_AST_CONTROL_HOLD
		case 0xf9: /* hold indicator */
		    cd_send_pbx_frame(cd, AST_CONTROL_HOLD, 0, NULL, 0);
		    break;

		case 0xfa: /* retrieve indicator */
		    cd_send_pbx_frame(cd, AST_CONTROL_UNHOLD, 0, NULL, 0);
		    break;
#endif
		default:
		    break;
		}
		break;

	case 0x807b:	/* INFORMATION */
		cd_verbose(cd, 3, 1, 3, "INFORMATION\n");
		break;

	case 0x807d:	/* STATUS */
		cd_verbose(cd, 3, 1, 3, "STATUS\n");
		break;

	default:
		cd_verbose(cd, 3, 1, 3, "Unknown INFO_IND, 0x%02x\n",
			   INFO_IND_INFONUMBER(CMSG));
		break;
	}
	return;
}

/*
 * CAPI FACILITY_IND
 */
static void
capi_handle_facility_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	_cmsg CMSG2;
	const void *parm;
	u_int16_t temp;
	u_int16_t reason = 0xFFFF;
	u_int8_t dtmf;

	FACILITY_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
			     HEADER_MSGNUM(CMSG), cd->msg_plci);
	FACILITY_RESP_FACILITYSELECTOR(&CMSG2) = 
	  FACILITY_IND_FACILITYSELECTOR(CMSG);
	FACILITY_RESP_FACILITYRESPONSEPARAMETERS(&CMSG2) = 
	  FACILITY_IND_FACILITYINDICATIONPARAMETER(CMSG);

	__capi_put_cmsg(cd->p_app, &CMSG2);
	
	parm = FACILITY_IND_FACILITYINDICATIONPARAMETER(CMSG);

	if (FACILITY_IND_FACILITYSELECTOR(CMSG) == FACILITYSELECTOR_LINE_INTERCONNECT) {
		temp = capi_get_2(parm,0);

		/* line interconnect */
		if (temp == 0x0001) {
		    cd_verbose(cd, 3, 0, 3, "Interconnect activated\n");
		}
		if (temp == 0x0002) {

		    reason = capi_get_2(parm,7);
		}
	}
	
	if (FACILITY_IND_FACILITYSELECTOR(CMSG) == FACILITYSELECTOR_DTMF) {

		/* DTMF */
		if ((!(cd->options.ntmode)) || (cd->state == CAPI_STATE_CONNECTED)) {

		    temp = 0;
		    while(capi_get_valid(parm,temp)) {

		        dtmf = capi_get_1(parm,temp);

		        if ((dtmf == 'X') || (dtmf == 'Y')) {
			    capi_handle_dtmf_fax(cd);
			} else {
			    cd_send_pbx_frame(cd, AST_FRAME_DTMF, dtmf, 
					      NULL, 0);
			}
			temp++;
		    }
		}
	}

	if (FACILITY_IND_FACILITYSELECTOR(CMSG) == 
	    FACILITYSELECTOR_SUPPLEMENTARY) {

		/* supplementary sservices */

		temp = (capi_get_1(parm,0) | (capi_get_1(parm,2) << 8));
		reason = capi_get_2(parm,3);

		/* ECT */
		if (temp == 0x0206) {

		    cd_verbose(cd, 1, 1, 3, "ECT reason = 0x%04x\n", 
			       reason);
		}

		/* RETRIEVE */
		if (temp == 0x0203) {

		    if (reason) {

		      cd_verbose(cd, 1, 0, 3, "Unable to retrieve, "
				 "reason = 0x%04x\n", reason);

		    } else {

		        cd->state = CAPI_STATE_CONNECTED;
			cd->flags.hold_is_active = 0;
			cd_verbose(cd, 1, 1, 3, "Call retrieved\n");
			capi_send_connect_b3_req(cd);
		    }
		}
		
		/* HOLD */
		if (temp == 0x0202) {

		    if (reason) {
		        cd->flags.hold_is_active = 0;
			cd_verbose(cd, 1, 0, 3, 
				   "unable to put call on hold, "
				   "reason=0x%04x, maybe you need to "
				   "subscribe for this...\n", reason);
		    } else {
		        cd->state = CAPI_STATE_ONHOLD;
			cd_verbose(cd, 1, 1, 3, "put on hold\n");

			if (cd->flags.send_retrieve_req_on_hold_ind) {
			    cd->flags.send_retrieve_req_on_hold_ind = 0;

			    capi_send_fac_suppl_req(cd, 0x0003 /* retrieve */);
			}
		    }
		}
	}

	if (reason != 0xFFFF) {
	    capi_show_info(reason);
	}
	return;
}

static void
capi_detect_silence(struct call_desc *cd, u_int8_t *ptr, u_int16_t len)
{
    int32_t pbx_capability = cd->pbx_capability;
    u_int8_t silence;
    int16_t temp;

    if((cd->flags.dir_outgoing == 0) ||
       (cd->options.ntmode) ||
       (cd->state != CAPI_STATE_CONNECTPENDING)) {

        /* should not wait for silence */

        cd->options.wait_silence_samples = 0;
	goto done;
    }

    if (pbx_capability == AST_FORMAT_ULAW) {
        silence = capi_signed_to_ulaw(0);
    } else {
        silence = capi_signed_to_alaw(0);
    }

    while(len--) {

        if (pbx_capability == AST_FORMAT_ULAW) {
	  temp = capi_ulaw_to_signed[*ptr];
  	} else {
	  temp = capi_alaw_to_signed[*ptr];
	}

	if((temp < 256) && 
	   (temp > -256)) {
	    cd->wait_silence_count++;
	} else {
	    cd->wait_silence_count = 0;
	}

	if(cd->wait_silence_count >=
	   cd->options.wait_silence_samples) {

	    cd_verbose(cd, 1, 0, 3, "Silence detected, "
		       "switching audio on!\n");

	    cd->options.wait_silence_samples = 0;
	    break;
	} else {
	    *ptr = silence;
	}

	ptr++;
    }

 done:
    return;
}

/*
 * CAPI DATA_B3_IND
 */
static void
capi_handle_data_b3_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	_cmsg CMSG2;
	u_int8_t *ptr_curr;
	u_int16_t len_curr;

	cd->rx_buffer_handle++;
	if (cd->rx_buffer_handle >= CAPI_MAX_B3_BLOCKS) {
	    cd->rx_buffer_handle = 0;
	}

	len_curr = DATA_B3_IND_DATALENGTH(CMSG);
	ptr_curr = RX_BUFFER_BY_HANDLE(cd, cd->rx_buffer_handle);

	/* extra length checks, should not trigger */

	if (len_curr > CAPI_MAX_B3_BLOCK_SIZE) {
	    len_curr = CAPI_MAX_B3_BLOCK_SIZE;
	}

	cd->rx_buffer_len[cd->rx_buffer_handle] =  len_curr;

	bcopy(DATA_B3_IND_DATA(CMSG), ptr_curr, len_curr);

	/* send a DATA_B3_RESP very quickly to free the buffer in capi */
	DATA_B3_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
			    HEADER_MSGNUM(CMSG), cd->msg_ncci);
	DATA_B3_RESP_DATAHANDLE(&CMSG2) = DATA_B3_IND_DATAHANDLE(CMSG);
	__capi_put_cmsg(cd->p_app, &CMSG2);

	if (cd->fax_file && cd->flags.fax_receiving) {

		/* write data to FAX file */

		cd_verbose(cd, 6, 1, 3, "len=%d, FAX\n", len_curr);

		if (fwrite(ptr_curr, 1, len_curr, cd->fax_file) != len_curr) {

		    cd_log(cd, LOG_WARNING, "error writing output "
			   "file (%s)\n", strerror(errno));
		}
		return;
	}

	/* wait silence detection */

	if (cd->options.wait_silence_samples) {
		capi_detect_silence(cd, ptr_curr, len_curr);
	}

	/* software echo suppression */

	if (cd->options.echo_suppress_in_software) {
		soft_echo_suppress_process(cd,
					   &cd->soft_es_rx, 
					   &cd->soft_es_tx, 
					   ptr_curr, len_curr);
	}

	/* convert sound last */

	capi_copy_sound(ptr_curr, ptr_curr, len_curr, 
			cd->cep ? cd->cep->rx_convert : NULL);

	cd_send_pbx_voice(cd, ptr_curr, len_curr);
	return;
}

/*
 * CAPI DATA_B3_CONF
 */
static void
capi_handle_data_b3_confirmation(struct call_desc *cd, u_int16_t wInfo)
{
	void *ptr;
	u_int16_t len;

	if (cd->flags.fax_receiving || 
	    cd->flags.fax_pending) {
		cd_verbose(cd, 3, 1, 2, "receiving a FAX, "
			   "data confirmation ignored!\n");
		return;
	}

	if(cd->flags.b3_active == 0) {
		cd_verbose(cd, 3, 1, 2, "B3 is not active, "
			   "data confirmation ignored!\n");
		return;
	}

	if(wInfo) {
		cd_verbose(cd, 3, 1, 2, "Received an error, "
			   "data confirmation ignored!\n");
		return;
	}

	/* feed more data */

	buf_read_block(&(cd->ring_buf), &ptr, &len);

	capi_send_data_b3_req(cd, 0, ptr, len);

	return;
}

/*
 * CAPI CONNECT_ACTIVE_IND
 */
static void
capi_handle_connect_active_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	uint8_t *date_time;
	_cmsg CMSG2;
	
	CONNECT_ACTIVE_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
				   HEADER_MSGNUM(CMSG), cd->msg_plci);
	__capi_put_cmsg(cd->p_app, &CMSG2);
	
	cd->state = CAPI_STATE_CONNECTED;

	date_time = CONNECT_ACTIVE_IND_DATE_TIME(CMSG);

	if (date_time && (date_time[0] > 0)) {
	    cd_verbose(cd, 2, 0, 3, "Received Date/Time = Y%02d/M%02d/D%02d "
		       "H%02d/M%02d/S%02d\n", 
		       capi_get_1(date_time,0), capi_get_1(date_time,1),
		       capi_get_1(date_time,2), capi_get_1(date_time,3),
		       capi_get_1(date_time,4), capi_get_1(date_time,5));
	}

	/* normal processing */
			    
	if (!(cd->flags.b3_active || cd->flags.b3_pending)) {
		/* send a CONNECT_B3_REQ */
		if (cd->flags.dir_outgoing) {
			/* outgoing call */
			capi_send_connect_b3_req(cd);
		} else {
			/* incoming call */
			/* RESP already sent ... wait for CONNECT_B3_IND */
		}
	} else {
		if (cd->flags.pbx_state_up == 0) {
		    cd->flags.pbx_state_up = 1;
			/* special treatment for early B3 connects */
			ast_setstate(cd->pbx_chan, AST_STATE_UP);
			cd_send_pbx_frame(cd, AST_FRAME_CONTROL, 
					  AST_CONTROL_ANSWER, NULL, 0);
		}
	}
	return;
}

/*
 * CAPI CONNECT_B3_ACTIVE_IND
 */
static void
capi_handle_connect_b3_active_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	u_int8_t temp;
	_cmsg CMSG2;

	/* then send a CONNECT_B3_ACTIVE_RESP */
	CONNECT_B3_ACTIVE_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
				      HEADER_MSGNUM(CMSG), cd->msg_ncci);
	__capi_put_cmsg(cd->p_app, &CMSG2);

	cd->flags.b3_active = 1;
	cd->flags.b3_pending = 0;

	buf_init(&(cd->ring_buf));

	for(temp = 0; temp < FIFO_BLOCK_START; temp++) {

	    capi_handle_data_b3_confirmation(cd, 0);

	}

	if (cd->flags.fax_pending || 
	    cd->flags.fax_receiving) {
		cd_verbose(cd, 3, 1, 3, "FAX connected!\n");

		cd->flags.fax_pending = 0;
		cd->flags.fax_receiving = 1;

	} else {

		capi_send_echo_cancel_req(cd, EC_FUNCTION_ENABLE);
		capi_send_detect_dtmf_req(cd, 1);
	}

	if ((cd->state == CAPI_STATE_CONNECTED) &&
	    (cd->flags.pbx_state_up == 0)) {
		cd->flags.pbx_state_up = 1;
		ast_setstate(cd->pbx_chan, AST_STATE_UP);
		cd_send_pbx_frame(cd, AST_FRAME_CONTROL, 
				  AST_CONTROL_ANSWER, NULL, 0);
	}
	return;
}

/*
 * CAPI DISCONNECT_B3_IND
 */
static void
capi_handle_disconnect_b3_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	const u_int8_t *ncpi = (const u_int8_t *)DISCONNECT_B3_IND_NCPI(CMSG);
	struct call_desc *cd = *pp_cd;
	char buffer[CAPI_MAX_STRING];
	_cmsg CMSG2;

	DISCONNECT_B3_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
				  HEADER_MSGNUM(CMSG), cd->msg_ncci);
	__capi_put_cmsg(cd->p_app, &CMSG2);

	cd->wCause_in_b3 = DISCONNECT_B3_IND_REASON_B3(CMSG);
	cd->msg_ncci = 0;

	cd->flags.b3_active = 0;
	cd->flags.b3_pending = 0;

	if (cd->flags.fax_receiving && (cd->pbx_chan)) {

	    /* if we have fax infos, set them as variables */
	    if (ncpi) {

#warning "Maybe cannot write these variables here! Locking issue!"

	        snprintf(buffer, sizeof(buffer), "%d", capi_get_2(ncpi,0));
		pbx_builtin_setvar_helper(cd->pbx_chan, "FAXRATE", buffer);
		snprintf(buffer, sizeof(buffer), "%d", capi_get_2(ncpi,2));
		pbx_builtin_setvar_helper(cd->pbx_chan, "FAXRESOLUTION", buffer);
		snprintf(buffer, sizeof(buffer), "%d", capi_get_2(ncpi,4));
		pbx_builtin_setvar_helper(cd->pbx_chan, "FAXFORMAT", buffer);
		snprintf(buffer, sizeof(buffer), "%d", capi_get_2(ncpi,6));
		pbx_builtin_setvar_helper(cd->pbx_chan, "FAXPAGES", buffer);

#warning "Does not handle length == 0xFF"

		memcpy(buffer, &ncpi[10], capi_get_1(ncpi,8));
		buffer[capi_get_1(ncpi,8)] = 0;
		pbx_builtin_setvar_helper(cd->pbx_chan, "FAXID", buffer);
	    }
	}

	if (cd->flags.fax_set_prot_on_b3_disc) {
	    cd->flags.fax_set_prot_on_b3_disc = 0;

	    if (capi_send_select_fax_prot_req(cd)) {
	        capi_send_disconnect_req(cd);
	    }
	}

	if (cd->flags.send_ect_on_b3_disc) {
	    cd->flags.send_ect_on_b3_disc = 0;

	    if (capi_send_ect_req(cd)) {
	        capi_send_disconnect_req(cd);
	    }
	}

	return;
}

/*
 * CAPI CONNECT_B3_IND
 */
static void
capi_handle_connect_b3_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	_cmsg CMSG2;

	/* then send a CONNECT_B3_RESP */
	CONNECT_B3_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
			       HEADER_MSGNUM(CMSG), cd->msg_ncci);
	CONNECT_B3_RESP_REJECT(&CMSG2) = 0;
	__capi_put_cmsg(cd->p_app, &CMSG2);

	return;
}

/*
 * CAPI DISCONNECT_IND
 */
static void 
capi_handle_disconnect_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	_cmsg CMSG2;

	DISCONNECT_RESP_HEADER(&CMSG2, cd->p_app->application_id, 
			       HEADER_MSGNUM(CMSG), cd->msg_plci);
	__capi_put_cmsg(cd->p_app, &CMSG2);
	
	cd->state = CAPI_STATE_DISCONNECTED;
	cd->wCause_in = DISCONNECT_IND_REASON(CMSG);

	capi_show_info(cd->wCause_in);

	cd->flags.fax_error = 
	  (((cd->wCause_in == 0x3490) || 
	    (cd->wCause_in == 0x349f)) &&
	   (cd->wCause_in_b3 == 0)) ? 0 : 1;

	/* the call is gone, free it */

	cd->flags.send_release_complete = 0;

	cd_free(cd, 1);
	cd = NULL;
	pp_cd[0] = cd;

	return;
}

static void
search_cep(struct call_desc *cd)
{
	char buffer[CAPI_MAX_STRING];
	u_int16_t strip_len;
	u_int8_t  match;
	u_int8_t  controller;
	const char *ptr;
	const char *msn;
	      char *buffer_pp;
	struct config_entry_iface *cep;

	if (cd->cep) {
	    return;
	}

	controller = (cd->msg_plci & 0xff);
	if (controller >= CAPI_MAX_CONTROLLERS) {
	    return;
	}

	cep = cep_root_acquire();

	while (cep) {

	    if (CC_GET_BIT(cep->controller_mask, controller)) {

	        strlcpy(buffer, cep->incomingmsn, sizeof(buffer));

		buffer_pp = NULL;

		for (msn = strtok_r(buffer, ",", &buffer_pp); 
		     msn; 
		     msn = strtok_r(NULL, ",", &buffer_pp))
		{
		    strip_len = 0;
		    match = 0;
		    ptr = cd->dst_telno;

		    while (1) {

		      if (*msn == '*') {
			  match = 1;
			  break;
		      }
		      if (*msn == '\0') {
			  strip_len = 0;
			  match = 1;
			  break;
		      }

		      if (*ptr == '\0') {
			  break;
		      }

		      if (*ptr != *msn) {
			  break;
		      }
		      ptr++;
		      msn++;
		      strip_len++;
		    }

		    if (match) {
		      if (cd_set_cep(cd, cep)) {
			  /* error */
			  continue;
		      } else {

			  /* got a match and a channel */

			  /* set strip len */

			  cd->dst_strip_len = strip_len;

			  cd_verbose(cd, 3, 0, 2, "Incoming call '%s' -> "
				     "'%s'\n", cd->src_telno, cd->dst_telno);
			  break;
		      }
		    }
		}
		if (cd->cep) break;
	    }
	    cep = cep->next;
	}

	cep_root_release();

	return;
}

/*
 * copy various information from call descriptor into
 * the PBX channel before starting the PBX:
 */
static void
cd_copy_telno_ext(struct call_desc *cd, const char *exten)
{
    struct ast_channel *pbx_chan = cd->pbx_chan;
    struct config_entry_iface *cep = cd->cep;
    char src_telno[AST_MAX_EXTENSION];

    if (cd->flags.dir_outgoing == 0) {

        pbx_chan->priority = 1;

        strlcpy(pbx_chan->exten, exten, sizeof(pbx_chan->exten));

#if (CC_AST_VERSION >= 0x10400)
	ast_string_field_build(pbx_chan, name, "CAPI/%s/%s-%x",
			       cep->name, cd->dst_telno, capi_get_counter());
#else
	snprintf(pbx_chan->name, sizeof(pbx_chan->name), "CAPI/%s/%s-%x",
		 cep->name, cd->dst_telno, capi_get_counter());
#endif

	strlcpy(src_telno, cd->src_telno, sizeof(src_telno));

	cc_mutex_lock(&capi_global_lock);

	if ((cd->src_ton & 0x70) == CAPI_ETSI_NPLAN_NATIONAL) {

	    snprintf(cd->src_telno, sizeof(cd->src_telno), "%s%s%s",
		     cep->prefix, capi_global.national_prefix, 
		     src_telno);

	} else if ((cd->src_ton & 0x70) == CAPI_ETSI_NPLAN_INTERNAT) {

	    snprintf(cd->src_telno, sizeof(cd->src_telno), "%s%s%s",
		     cep->prefix, capi_global.international_prefix, 
		     src_telno);

	} else {

	    snprintf(cd->src_telno, sizeof(cd->src_telno), "%s%s",
		     cep->prefix, src_telno);
	}

	cc_mutex_unlock(&capi_global_lock);

#ifdef CC_AST_CHANNEL_HAS_CID
	if (pbx_chan->cid.cid_num) {
	    free(pbx_chan->cid.cid_num);
	}
	if (pbx_chan->cid.cid_dnid) {
	    free(pbx_chan->cid.cid_dnid);
	}

	pbx_chan->cid.cid_num = strdup(cd->src_telno);
	pbx_chan->cid.cid_dnid = strdup(cd->dst_telno);
	pbx_chan->cid.cid_ton = 0; /* NOTE: already prefixed number! */
#else
	if (pbx_chan->callerid) {
	    free(pbx_chan->callerid);
	}
	if (pbx_chan->dnid) {
	    free(pbx_chan->dnid);
	}
	pbx_chan->callerid = strdup(cd->src_telno);
	pbx_chan->dnid = strdup(cd->dst_telno);
#endif
    }
    return;
}

/* check if the there is an extension for this call in the PBX */

static void
cd_start_pbx(struct call_desc **pp_cd, const char *exten)
{
	struct call_desc *cd = *pp_cd;
	struct ast_channel *pbx_chan = cd->pbx_chan;

	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (cd->flags.pbx_search_complete) {
	    cd_verbose(cd, 3, 0, 2,  "PBX already started!\n");
	    return;
	}

	cd->flags.pbx_search_complete = 1;

	cd_copy_telno_ext(cd, exten);

	ast_setstate(pbx_chan, AST_STATE_RING);

	if (ast_pbx_start(cd->pbx_chan)) {
	    goto error;
	}

	cd->flags.pbx_started = 1;

	/* send CALL PROCEEDING back to caller, 
	 * which will also set new state
	 */
	capi_send_alert_req(cd, 1);

	cd_verbose(cd, 2, 0, 2, "Started PBX\n");

	return;

 error:

	/* ignore call */

	cd_verbose(cd, 2, 0, 2, "Unable to start PBX for "
		   "destination '%s'!\n", cd->dst_telno);

	cd->wCause_out = 0x0001; /* ignore call */

	cd_free(cd, 1);
	cd = NULL;
	pp_cd[0] = cd;

	return;
}

/*
 * CAPI CONNECT_IND
 */
static void
capi_handle_connect_indication(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	struct ast_channel *pbx_chan = cd->pbx_chan;
	u_int8_t start_immediate;
	char buffer[AST_MAX_EXTENSION];

	cd->bchannelinfo[0] = 
	  capi_get_1(CONNECT_IND_BCHANNELINFORMATION(CMSG),0) + '0';

	cd->flags.sending_complete_received = 
	  (capi_get_2(CONNECT_IND_SENDINGCOMPLETE(CMSG), 0) == 0x0001);

	capi_get_multi_1(CONNECT_IND_CALLEDPARTYNUMBER(CMSG), 1, 
			 &cd->dst_telno, sizeof(cd->dst_telno));
	cd->dst_ton = 
	  capi_get_1(CONNECT_IND_CALLEDPARTYNUMBER(CMSG), 0) & 0x7f;

	capi_get_multi_1(CONNECT_IND_CALLINGPARTYNUMBER(CMSG), 2, 
			 &cd->src_telno, sizeof(cd->src_telno));

	cd->src_ton = 
	  capi_get_1(CONNECT_IND_CALLINGPARTYNUMBER(CMSG), 0) & 0x7f;

	cd->src_pres = 
	  capi_get_1(CONNECT_IND_CALLINGPARTYNUMBER(CMSG), 1) & 0x63;

	cd->msg_cip = CONNECT_IND_CIPVALUE(CMSG);

#ifdef CC_AST_CHANNEL_HAS_TRANSFERCAP	
	pbx_chan->transfercapability = cip2tcap(cd->msg_cip);
#endif
#ifdef CC_AST_CHANNEL_HAS_CID
	pbx_chan->cid.cid_pres = cd->src_pres;
#else    
	pbx_chan->callingpres = cd->src_pres;
#endif
#ifdef CC_AST_CHANNEL_HAS_TRANSFERCAP	
	pbx_builtin_setvar_helper(pbx_chan, "TRANSFERCAPABILITY", 
             transfercapability2str(pbx_chan->transfercapability));
#endif
	pbx_builtin_setvar_helper(pbx_chan, "BCHANNELINFO", 
				  &cd->bchannelinfo[0]);

	snprintf(buffer, sizeof(buffer), "%d", cd->dst_ton);
	pbx_builtin_setvar_helper(pbx_chan, "CALLEDTON", &buffer[0]);

	capi_get_multi_1(CONNECT_IND_CALLINGPARTYSUBADDRESS(CMSG), 1, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "CALLINGSUBADDRESS", &buffer[0]);

	capi_get_multi_1(CONNECT_IND_CALLEDPARTYSUBADDRESS(CMSG), 1, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "CALLEDSUBADDRESS", &buffer[0]);

	capi_get_multi_1(CONNECT_IND_USERUSERDATA(CMSG), 0, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "USERUSERINFO", &buffer[0]);

	capi_get_multi_1(CONNECT_IND_SECONDCALLINGPARTYNUMBER(CMSG), 2, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "SECONDCALLERID", &buffer[0]);

#ifdef CONNECT_IND_DISPLAY
	capi_get_multi_1(CONNECT_IND_DISPLAY(CMSG), 0, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "DISPLAY", &buffer[0]);
#endif

	capi_get_multi_1(CONNECT_IND_KEYPADFACILITY(CMSG), 0, 
			 &buffer, sizeof(buffer));

	pbx_builtin_setvar_helper(pbx_chan, "KEYPAD", &buffer[0]);
#if 0
	capi_get_multi_1(.....);
	pbx_builtin_setvar_helper(pbx_chan, "ANI2", &buffer[0]);
#endif

	cd_verbose(cd, 1, 0, 3, "Incoming call from '%s' to '%s', "
		   "CIP=0x%04x, sending_complete=%s\n",
		   cd->src_telno, cd->dst_telno, cd->msg_cip,
		   cd->flags.sending_complete_received ? "yes" : "no");

	search_cep(cd);

	if (cd->cep) {

	    start_immediate = (cd->options.immediate && (cd->dst_telno[0] == 0));

	    if (cd->flags.sending_complete_received || start_immediate) {

	        /* number is complete, try to start the PBX */

	        cd_start_pbx(pp_cd, start_immediate ? "s" : &(cd->dst_telno[0]));
		cd = *pp_cd;
	    }
	}

	/* else wait for the periodic thread to handle it ... */

	return;
}

/*
 * CAPI FACILITY_CONF
 */
static void
capi_handle_facility_confirmation_app(_cmsg *CMSG, 
				      struct cc_capi_application *p_app)
{
	const void *param = FACILITY_CONF_FACILITYCONFIRMATIONPARAMETER(CMSG);
	u_int16_t wSelector = FACILITY_CONF_FACILITYSELECTOR(CMSG);
	struct cc_capi_controller *p_ctrl = p_app->temp_p_ctrl;
	u_int16_t wFunction;
	u_int32_t dwServices;

	if (p_ctrl == NULL) {
	    return;
	}

	if (wSelector == FACILITYSELECTOR_SUPPLEMENTARY) {

	    wFunction = capi_get_2(param,0);

	    if (wFunction == 0x0000) {

	        /* get supported services */

	        p_app->received_facility_conf = 1;
		p_app->temp_p_ctrl = NULL;

		if (capi_get_2(param,3) != 0x0000) {
		    cc_log(LOG_NOTICE, "supplementary services "
		       "info = 0x%04x\n", capi_get_2(param,3));
		    return;
		}

		dwServices = capi_get_4(param,5);
		cc_verbose(3, 0, VERBOSE_PREFIX_4 "supplementary "
			   "services : 0x%08x\n", dwServices);
	
		/* success, so set the features we have */

		if (dwServices & 0x0001) {
		    p_ctrl->support.holdretrieve = 1;
		}
		if (dwServices & 0x0002) {
		    p_ctrl->support.terminalportability = 1;
		}
		if (dwServices & 0x0004) {
		    p_ctrl->support.ECT = 1;
		}
		if (dwServices & 0x0008) {
		    p_ctrl->support.threePTY = 1;
		}
		if (dwServices & 0x0010) {
		    p_ctrl->support.CF = 1;
		}
		if (dwServices & 0x0020) {
		    p_ctrl->support.CD = 1;
		}
		if (dwServices & 0x0040) {
		    p_ctrl->support.MCID = 1;
		}
		if (dwServices & 0x0080) {
		    p_ctrl->support.CCBS = 1;
		}
		if (dwServices & 0x0100) {
		    p_ctrl->support.MWI = 1;
		}
		if (dwServices & 0x0200) {
		    p_ctrl->support.CCNR = 1;
		}
		if (dwServices & 0x0400) {
		    p_ctrl->support.CONF = 1;
		}
	    }
	}
	return;
}

/*
 * CAPI FACILITY_CONF
 */
static void
capi_handle_facility_confirmation_cd(_cmsg *CMSG, struct call_desc **pp_cd)
{
	struct call_desc *cd = *pp_cd;
	u_int16_t wSelector = FACILITY_CONF_FACILITYSELECTOR(CMSG);
	const void *parm = FACILITY_CONF_FACILITYCONFIRMATIONPARAMETER(CMSG);
	u_int16_t wTemp;

	if (wSelector == FACILITYSELECTOR_DTMF) {
		cd_verbose(cd, 2, 1, 4, "DTMF CONF\n");
		return;
	}
	if (wSelector == cd->options.echo_cancel_selector) {
		if (FACILITY_CONF_INFO(CMSG)) {
		    cd_verbose(cd, 2, 0, 3, "Error setting up "
			       "echo canceller\n");
		    return;
		}
		if (capi_get_1(parm,0) == EC_FUNCTION_DISABLE) {
		    cd_verbose(cd, 2, 0, 3, "Echo canceller successfully "
			       "disabled\n");
		} else {
		    cd_verbose(cd, 2, 0, 3, "Echo canceller successfully "
			       "set up\n");
		}
		return;
	}
	if (wSelector == FACILITYSELECTOR_SUPPLEMENTARY) {
		/* HOLD */

		if ((capi_get_1(parm,0) == 0x02) &&
		    (capi_get_1(parm,1) == 0x00) &&
		    ((capi_get_1(parm,3) != 0x00) ||
		     (capi_get_1(parm,4) != 0x00))) {

		    cd_verbose(cd, 2, 0, 3, "Call on hold\n");
		}
		return;
	}
	if (wSelector == FACILITYSELECTOR_LINE_INTERCONNECT) {

		wTemp = capi_get_2(parm,0);

		if (wTemp == 0x0001) {

			/* enable */
			capi_show_info(capi_get_2(parm,11));

		} else if (wTemp == 0x0002) {

			/* disable */
			capi_show_info(capi_get_2(parm,11));
		}
		return;
	}
	cd_log(cd, LOG_ERROR, "unhandled FACILITY_CONF "
	       "0x%04x\n", wSelector);
	return;
}

/*---------------------------------------------------------------------------*
 *      capi_handle_cmsg - handle a CAPI message for a CAPI application
 *
 * NOTE: must be called with "p_app->lock" locked
 * NOTE: this function can sleep
 *---------------------------------------------------------------------------*/
static void
capi_handle_cmsg(struct cc_capi_application *p_app, _cmsg *CMSG)
{
	u_int32_t NCCI = HEADER_CID(CMSG);
	u_int32_t PLCI = (NCCI & 0xffff);
	u_int16_t wCmd = HEADER_CMD(CMSG);
	u_int16_t wMsgNum = HEADER_MSGNUM(CMSG);
	u_int16_t wInfo = 0xffff;
	struct call_desc *cd;

	if ((wCmd == CAPI_P_IND(DATA_B3)) ||
	    (wCmd == CAPI_P_CONF(DATA_B3))) {
		cc_verbose(9, 1, "%s\n", capi_cmsg2str(CMSG));
	} else {
		cc_verbose(4, 1, "%s\n", capi_cmsg2str(CMSG));
	}

	cc_mutex_assert(&p_app->lock, MA_OWNED);

	if ((PLCI & 0xFF00) == 0) {

	    /* the PLCI is addressing a controller */

	    if (wCmd == CAPI_P_CONF(LISTEN)) {

	        p_app->received_listen_conf = 1;

	    } else if (wCmd == CAPI_P_CONF(FACILITY)) {

		wInfo = FACILITY_CONF_INFO(CMSG);
		capi_handle_facility_confirmation_app(CMSG, p_app);

	    } else if (wCmd == CAPI_P_CONF(CONNECT)) {

	        goto handle_connect_conf;
	    }

	    /* nothing more to do */
	    goto done;
	}

	cd = cd_by_plci(p_app, PLCI);

	if (wCmd == CAPI_P_IND(CONNECT)) {

	    if(cd) {
	        /* chan_capi does not support 
		 * double connect indications !
		 * (This is used to update 
		 *  telephone numbers and 
		 *  other information)
		 */
	        goto done;
	    }

	    cd = cd_alloc(p_app, PLCI);

	    if (cd == NULL) {
	        /* requested circuit channel not available */
	        capi_send_connect_resp_app(p_app, wMsgNum, PLCI, 0x0004);
	    }

	} else if (wCmd == CAPI_P_CONF(CONNECT)) {

	    if (cd) {
	        cc_log(LOG_ERROR, "Duplicate CONNECT_CONF "
		       "received! (ignored)\n");
		goto done;
	    }

	handle_connect_conf:

	    cd = cd_by_msg_num(p_app, wMsgNum);
	}

	if(cd == NULL) {

	    /* if there is no call descriptor 
	     * for this PLCI then there is 
	     * nothing more to do !
	     */
	    goto done;
	}

	/* main switch table */

	switch (wCmd) {

	  /*
	   * CAPI indications
	   */
	case CAPI_P_IND(CONNECT):
		cd->flags.send_release_complete = 1;
		cd->msg_num = wMsgNum; /* store message number */
		capi_handle_connect_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(DATA_B3):
		capi_handle_data_b3_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(CONNECT_B3):
		cd->msg_ncci = NCCI;
		capi_handle_connect_b3_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(CONNECT_B3_ACTIVE):
		cd->msg_ncci = NCCI;
		capi_handle_connect_b3_active_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(DISCONNECT_B3):
		capi_handle_disconnect_b3_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(DISCONNECT):
		capi_handle_disconnect_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(FACILITY):
		capi_handle_facility_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(INFO):
		capi_handle_info_indication(CMSG, &cd);
		break;
	case CAPI_P_IND(CONNECT_ACTIVE):
		capi_handle_connect_active_indication(CMSG, &cd);
		break;

	  /*
	   * CAPI confirmations
	   */

	case CAPI_P_CONF(FACILITY):
		wInfo = FACILITY_CONF_INFO(CMSG);
		capi_handle_facility_confirmation_cd(CMSG, &cd);
		break;
	case CAPI_P_CONF(CONNECT):
		wInfo = CONNECT_CONF_INFO(CMSG);

		/* update PLCI */
		cd->msg_plci = PLCI;

		cd_verbose(cd, 1, 1, 3, "received CONNECT_CONF\n");

		if (wInfo) {
			cd->flags.send_release_complete = 0;

			cd_free(cd, 1);
			cd = NULL;
		} else {

		    cd->flags.send_release_complete = 1;
		}
		break;
	case CAPI_P_CONF(CONNECT_B3):
		wInfo = CONNECT_B3_CONF_INFO(CMSG);
		if (wInfo == 0) {
			cd->msg_ncci = NCCI;
		} else {
			cd->flags.b3_active = 0;
			cd->flags.b3_pending = 0;
		}
		break;
	case CAPI_P_CONF(ALERT):
		wInfo = ALERT_CONF_INFO(CMSG);
		break;
	case CAPI_P_CONF(SELECT_B_PROTOCOL):
		wInfo = SELECT_B_PROTOCOL_CONF_INFO(CMSG);
		if (wInfo == 0) {
			if ((cd->pbx_chan) && (cd->flags.fax_receiving || cd->flags.fax_pending)) {
				capi_send_echo_cancel_req(cd, EC_FUNCTION_DISABLE);
				capi_send_detect_dtmf_req(cd, 0);
			}
		} else {
			cd->flags.b3_active = 0;
			cd->flags.b3_pending = 0;
		}
		break;
	case CAPI_P_CONF(DATA_B3):
		wInfo = DATA_B3_CONF_INFO(CMSG);
		capi_handle_data_b3_confirmation(cd, wInfo);
		break;
 
	case CAPI_P_CONF(DISCONNECT):
		wInfo = DISCONNECT_CONF_INFO(CMSG);
		break;

	case CAPI_P_CONF(DISCONNECT_B3):
		wInfo = DISCONNECT_B3_CONF_INFO(CMSG);
		break;

	case CAPI_P_CONF(LISTEN):
		wInfo = LISTEN_CONF_INFO(CMSG);
		break;

	case CAPI_P_CONF(INFO):
		wInfo = INFO_CONF_INFO(CMSG);
		break;

	default:
		cc_log(LOG_WARNING, "Unknown command=%s,0x%04x",
		       capi_command_to_string(wCmd), wCmd);
		break;
	}

 done:
	if (cd == NULL) {
	    cc_verbose(2, 1, VERBOSE_PREFIX_4
		       "CAPI: Command=%s, 0x%04x: no call descriptor "
		       "for PLCI=0x%04x, MSGNUM=0x%04x:\n", 
		       capi_command_to_string(wCmd), wCmd, PLCI, wMsgNum);
	}

	if (wInfo != 0xffff) {
		if (wInfo) {
			capi_show_conf_error(cd, PLCI, wInfo, wCmd);
		}
		capi_show_info(wInfo);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_progress - activate PROGRESS for an 
 *                               incoming call [in NT-mode]
 *
 * param: not used
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_progress(struct call_desc *cd, struct call_desc *cd_unused, 
		       char *param)
{
	_cmsg CMSG;

	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if ((cd->state != CAPI_STATE_DID) && 
	    (cd->state != CAPI_STATE_INCALL) &&
	    (cd->state != CAPI_STATE_ALERTING)) {
		cd_log(cd, LOG_WARNING, "wrong channel state to "
		       "signal PROGRESS\n");
		return 0;
	}

	if (!(cd->options.ntmode)) {
		cd_log(cd, LOG_WARNING, "PROGRESS is usually not "
		       "supported in TE-mode!\n");

		/* just continue */
	}

	if (cd->flags.dir_outgoing) {
		cd_log(cd, LOG_WARNING, "PROGRESS transmission "
		       "is not supported for outgoing "
		       "calls!\n");
		return 0;
	}

	if (cd->flags.b3_active || 
	    cd->flags.b3_pending) {
		cd_verbose(cd, 4, 1, 4, "B-channel already "
			   "pending or up.\n");
		return 0;
	}

	cd->flags.b3_pending = 1;

	SELECT_B_PROTOCOL_REQ_HEADER(&CMSG, cd->p_app->application_id, 
				     get_msg_num_other(cd->p_app), cd->msg_plci);
	SELECT_B_PROTOCOL_REQ_B1PROTOCOL(&CMSG) = 1;
	SELECT_B_PROTOCOL_REQ_B2PROTOCOL(&CMSG) = 1;
	SELECT_B_PROTOCOL_REQ_B3PROTOCOL(&CMSG) = 0;
	SELECT_B_PROTOCOL_REQ_B1CONFIGURATION(&CMSG) = NULL;
	SELECT_B_PROTOCOL_REQ_B2CONFIGURATION(&CMSG) = NULL;
	SELECT_B_PROTOCOL_REQ_B3CONFIGURATION(&CMSG) = NULL;

	return __capi_put_cmsg(cd->p_app, &CMSG);
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_call_deflect - deflect a call
 *
 * param: deflect telephone number
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_call_deflect(struct call_desc *cd, struct call_desc *cd_unknown,
			   char *param)
{
	int param_len = strlen(param);
	u_int8_t fac[64];
	_cmsg CMSG;

	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (!cd->support.CD) {
	    cd_log(cd, LOG_NOTICE, "call deflection "
		   "is not supported by CAPI controller!\n");
	    return -1;
	}

	if ((cd->state != CAPI_STATE_INCALL) &&
	    (cd->state != CAPI_STATE_DID) &&
	    (cd->state != CAPI_STATE_ALERTING)) {
	    cd_log(cd, LOG_WARNING, "wrong state of call for "
		   "call deflection!\n");
	    return -1;
	}

	if (cd->state != CAPI_STATE_ALERTING) {
	    capi_send_alert_req(cd, 0);
	}

	cd_verbose(cd, 2, 1, 3, "\n");

	if (param_len > (sizeof(fac)-0x0B)) {
	    cd_log(cd, LOG_WARNING, "truncating long deflection "
		   "number from %d to %d bytes!\n",
		   param_len, (uint32_t)(sizeof(fac)-0x0B));
	    param_len = sizeof(fac)-0x0B;
	}

	fac[0] = 0x0A + param_len; /* length */
	fac[1] = 0x0D; /* call deflection */
	fac[2] = 0x00;
	fac[3] = 0x07 + param_len; /* struct len */
	fac[4] = 0x01; /* display of own address allowed */
	fac[5] = 0x00;
	fac[6] = 0x03 + param_len;
	fac[7] = 0x00; /* type of facility number */
	fac[8] = 0x80; /* number plan */
	fac[9] = 0x80; /* presentation allowed */
	fac[10+param_len] = 0x00; /* empty subaddress */

	bcopy(param, fac+10, param_len);

	FACILITY_REQ_HEADER(&CMSG, cd->p_app->application_id, 
			    get_msg_num_other(cd->p_app), cd->msg_plci);
	FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_SUPPLEMENTARY;
	FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)&fac;

	cd->support.CD = 0; /* only send once per call */

	return __capi_put_cmsg(cd->p_app, &CMSG);
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_receive_fax - receive a fax
 *
 * param: FAX receive filename
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_receive_fax(struct call_desc *cd, struct call_desc *cd_unused, 
			  char *param)
{
	static const u_int16_t bprot[3] = { 4, 4, 4 };

	const char *filename;
	const char *stationid;
	const char *headline;
	u_int16_t error = 0;

	filename = strsep(&param, "|");
	stationid = strsep(&param, "|");
	headline = param;

	if (!stationid)
		stationid = empty_string;
	if (!headline)
		headline = empty_string;

	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (cd->fax_file || 
	    cd->flags.fax_receiving || 
	    cd->flags.fax_pending) {
	    cd_log(cd, LOG_WARNING, "Already received a FAX!\n");
	    return -1;
	}

	if (cd->fax_fname) {
	    filename = cd->fax_fname;
	} else {
	    cd->fax_fname = strdup(filename);
	}

	cd->fax_file = fopen(filename, "wb");

	if (cd->fax_file == NULL) {
	    cd_log(cd, LOG_WARNING, "cannot create FAX "
		   "output file, %s\n", strerror(errno));
	    error = -1;
	    goto done;
	}

	cd->flags.fax_pending = 1;

	cd_set_fax_config(cd, FAX_SFF_FORMAT, stationid, headline);

	switch (cd->state) {
	case CAPI_STATE_ALERTING:
	case CAPI_STATE_DID:
	case CAPI_STATE_INCALL:
		if(capi_send_connect_resp(cd, 0, bprot)) {
		    error = -1;
		    goto done;
		}
		break;

	case CAPI_STATE_CONNECTED:
		if(capi_send_select_fax_prot_req(cd)) {
		    error = -1;
		    goto done;
		}
		break;

	default:
		cd_log(cd, LOG_WARNING, "Receiving FAX in "
		       "wrong state (%d)\n", cd->state);
		error = -1;
		goto done;
	}

	while (cd->flags.fax_receiving || cd->flags.fax_pending) {

	    if(capi_application_usleep(cd->p_app, 10000)) {

	         break;
	    }
	}

 done:
	if (error) {
	    cd->flags.fax_receiving = 0;
	    cd->flags.fax_pending = 0;
	}
	return error;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_echosquelch - set echo squelch
 *
 * param: "yes" or "no"
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_echosquelch(struct call_desc *cd, struct call_desc *cd_unknown,
			  char *param)
{
	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (strcasecmp(param, "soft") == 0) {
		cd->options.echo_cancel_in_software = 1;
	} else if (strcasecmp(param, "fax") == 0) {
		cd->options.echo_suppress_fax = 1;
		cd->options.echo_suppress_in_software = 1;
	} else if (ast_true(param)) {
		cd->options.echo_suppress_fax = 0;
		cd->options.echo_suppress_in_software = 1;
	} else if (ast_false(param)) {
		cd->options.echo_suppress_fax = 0;
		cd->options.echo_suppress_in_software = 0;
	} else {
		cd_log(cd, LOG_WARNING, "Parameter, '%s', for "
		       "echosquelch is invalid.\n", param);
		return -1;
	}
	cd_verbose(cd, 2, 1, 4, "echosquelch switched %s%s\n",
		   cd->options.echo_suppress_in_software ? "ON" : "OFF",
		   cd->options.echo_suppress_fax ? " (FAX)" : "");
	return 0;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_malicious - report a malicious call
 *
 * param: not used
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_malicious(struct call_desc *cd, struct call_desc *cd_unknown,
			char *param)
{
	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (!cd->support.MCID) {
		cd_log(cd, LOG_NOTICE, "MCID is not supported "
		       "by CAPI controller.\n");
		return -1;
	}

	cd->support.MCID = 0; /* only send once per call */

	cd_verbose(cd, 2, 1, 4, "sending MCID\n");

	return capi_send_fac_suppl_req(cd, 0x000E /* MCID */);
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_hold - put a call on hold
 *
 * param: not used
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_hold(struct call_desc *cd, struct call_desc *cd_unknown, 
		   char *param)
{
	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	/* TODO: support holdtype notify */

	if (cd->flags.send_retrieve_req_on_hold_ind) {
		cd_log(cd, LOG_NOTICE, "Retrieve already buffered. "
		       "Please try to hold again.\n");
		return -1;
	}

	if (cd->flags.hold_is_pending) {
		cd_log(cd, LOG_NOTICE, "Already on hold.\n");
		return 0;
	}

	if (!(cd->flags.b3_active)) {
		cd_log(cd, LOG_NOTICE, "Cannot set on hold "
		       "while not connected.\n");
		return 0;
	}

	if (!(cd->support.holdretrieve)) {
		cd_log(cd, LOG_NOTICE,"HOLD is not supported "
		       "by controller.\n");
		return 0;
	}

	cd_verbose(cd, 2, 1, 4, "sending HOLD\n");

	cd->flags.hold_is_active = 1;
	cd->flags.hold_is_pending = 1;

	return capi_send_fac_suppl_req(cd, 0x0002 /* HOLD */);
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_holdtype - set hold type
 *
 * param: hold type
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_holdtype(struct call_desc *cd, struct call_desc *cd_unknown, 
		       char *param)
{
	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (!strcasecmp(param, "hold")) {
		cd->options.hold_type = CC_HOLDTYPE_HOLD;
	} else if (!strcasecmp(param, "notify")) {
		cd->options.hold_type = CC_HOLDTYPE_NOTIFY;
	} else if (!strcasecmp(param, "local")) {
		cd->options.hold_type = CC_HOLDTYPE_LOCAL;
	} else {
		cd_log(cd, LOG_WARNING, "Parameter for holdtype "
		       "invalid.\n");
		return -1;
	}
	cd_verbose(cd, 2, 1, 4, "holdtype switched %d\n",
		   cd->options.hold_type);
	return 0;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_retrieve - retrieve a call on hold
 *
 * param: not used
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_retrieve(struct call_desc *cd, struct call_desc *cd_unknown,
		       char *param)
{
	cc_mutex_assert(&cd->p_app->lock, MA_OWNED);

	if (cd->flags.send_retrieve_req_on_hold_ind) {
		cd_log(cd, LOG_NOTICE, "Retrieve already buffered. "
		       "Please try to hold again.\n");
		return 0;
	}

	if (!(cd->flags.hold_is_active)) {
	    cd_log(cd, LOG_WARNING, "Call is not on hold!\n");
	    return -1;
	}

	if (!(cd->support.holdretrieve)) {
	    cd_log(cd, LOG_NOTICE, "RETRIEVE is not supported "
		   "by CAPI controller!\n");
	    return -1;
	}

	if ((cd->state != CAPI_STATE_ONHOLD) &&
	    (cd->flags.hold_is_pending)) {

	    cd->flags.send_retrieve_req_on_hold_ind = 1;

	} else {

	    capi_send_fac_suppl_req(cd, 0x0003 /* retrieve */);
	}

	cd_verbose(cd, 2, 1, 4, "retrieving call\n");

	cd->flags.hold_is_pending = 0;

	return 0;
}

/*---------------------------------------------------------------------------*
 *      chan_capi_cmd_ect - transfer a call on hold
 *
 * param: second call descriptor indentifier
 *---------------------------------------------------------------------------*/
static u_int16_t
chan_capi_cmd_ect(struct call_desc *cd0, struct call_desc *cd1, 
		  char *param)
{
	cc_mutex_assert(&cd0->p_app->lock, MA_OWNED);
	cc_mutex_assert(&cd1->p_app->lock, MA_OWNED);

	if (!cd0->support.ECT) {
	    cd_log(cd0, LOG_WARNING, "ECT is not supported "
		   "by CAPI controller.\n");
	    return -1;
	}

	if (!cd1->support.ECT) {
	    cd_log(cd1, LOG_WARNING, "ECT is not supported "
		   "by CAPI controller.\n");
	    return -1;
	}

	if (!cd1->flags.hold_is_pending) {
	    cd_log(cd1, LOG_WARNING, "call is not on hold!\n");
	    return -1;
	}

	if (cd0->flags.ect_pending) {
	    cd_log(cd0, LOG_WARNING, "ECT is already pending.\n");
	    return -1;
	}

	if (cd1->flags.ect_pending) {
	    cd_log(cd1, LOG_WARNING, "ECT is already pending.\n");
	    return -1;
	}

	cd_verbose(cd0, 2, 1, 4, "doing ECT\n");
	cd_verbose(cd1, 2, 1, 4, "doing ECT\n");

	/* store ECT plci */

	cd0->ect_plci = cd1->msg_plci;

	if (cd0->flags.b3_active) {

	    cd0->flags.send_ect_on_b3_disc = 1;

	    capi_send_disconnect_b3_req(cd0);

	} else {

	    capi_send_ect_req(cd0);

	}

	cd1->flags.hold_is_pending = 0;
	cd1->flags.ect_pending = 1;
	cd0->flags.ect_pending = 1;

	/* wait for disconnect ind */

	cd0->flags.send_release_complete = 0;
	cd1->flags.send_release_complete = 0;

	return 0;
}

/* list of "chan_capi" commands */

struct chan_capi_command_entry {
	const char *cmd_name;
	u_int16_t (*cmd_func)(struct call_desc *, struct call_desc *, char *);
	u_int32_t capi_only : 1;
	u_int32_t arg_is_plci : 1;
	u_int32_t arg_is_non_zero : 1;
};

static const struct chan_capi_command_entry
chan_capi_commands[] = {
	{ "progress",     &chan_capi_cmd_progress,        1, 0, 0 },
	{ "deflect",      &chan_capi_cmd_call_deflect,    1, 0, 1 },
	{ "receivefax",   &chan_capi_cmd_receive_fax,     1, 0, 1 },
	{ "echosquelch",  &chan_capi_cmd_echosquelch,     1, 0, 1 },
	{ "malicious",    &chan_capi_cmd_malicious,       1, 0, 0 },
	{ "hold",         &chan_capi_cmd_hold,            1, 0, 0 },
	{ "holdtype",     &chan_capi_cmd_holdtype,        1, 0, 1 },
	{ "retrieve",     &chan_capi_cmd_retrieve,        1, 0, 0 },
	{ "ect",          &chan_capi_cmd_ect,             1, 1, 1 },
	{ NULL, NULL, 0 }
};

/*---------------------------------------------------------------------------*
 *      chan_capi_command_exec - "chan_capi" command interface
 *---------------------------------------------------------------------------*/
static int
chan_capi_command_exec(struct ast_channel *chan, void *data)
{
#if (CC_AST_VERSION >= 0x10400)
	struct ast_module_user *u;
#else
	struct localuser *u = NULL;
#endif
	struct ast_channel *chan_second = NULL;
	struct call_desc *cd0;
	struct call_desc *cd1;
	char *stringp;
	const char *command;
	const char *params;
	const struct chan_capi_command_entry *
	  capicmd = &chan_capi_commands[0];
	int error = 0;

	if (!data) {
		cc_log(LOG_WARNING, "capiCommand requires arguments!\n");
		return -1;
	}

#if (CC_AST_VERSION >= 0x10400)
	u = ast_module_user_add(chan);
#else
	LOCAL_USER_ADD(u);
#endif

	/* duplicate string using alloca */

	stringp = ast_strdupa(data);
	command = strsep(&stringp, "|");
	params = stringp;

	cc_verbose(2, 1, VERBOSE_PREFIX_3 "capiCommand: "
		   "'%s' '%s'\n", command, params);

	/* lookup "chan_capi" command */

	while (1) {

	    if (capicmd->cmd_func == NULL) {
		cc_log(LOG_WARNING, "Unknown command '%s' "
		       "for capiCommand\n", command);
		error = -1;
		goto done;
	    }

	    if (!strcasecmp(capicmd->cmd_name, command)) {
	        break;
	    }

	    capicmd++;
	}

	if (capicmd->capi_only) {
#if (CC_AST_VERSION >= 0x10400)
	    if (chan->tech != &chan_capi_tech) {
#else
	    if (strcmp(chan->type, "CAPI")) {
#endif
	        cc_log(LOG_WARNING, "capiCommand works on CAPI "
		       "channels only. Check your "
		       "'extensions.conf'!\n");
		error = -1;
		goto done;
	    }
	}

	if (capicmd->arg_is_plci) {

	    u_int16_t plci = strtoul(params, NULL, 0) & 0xFFFF;

	    chan_second = pbx_chan_by_plci_on_hold(plci);
	}

	if (capicmd->arg_is_non_zero) {

	    if ((params == NULL) || (params[0] == 0)) {

	        cc_log(LOG_WARNING, "capiCommand(%s) requires "
		       "a non-zero, second argument!\n", 
		       capicmd->cmd_name);
	    }
	}

	/* make a writeable copy of the parameters */

	params = ast_strdupa((char *)params);

	/* make sure that the second channel is not zero */

	if (chan_second == NULL) {
	    chan_second = chan;
	}

	/* do locking right */

	if(cd_mutex_lock_double_pbx_chan(chan, chan_second, &cd0, &cd1)) {
	    return -1;
	}
	
	error = ((capicmd->cmd_func)(cd0, cd1, (char *)params)) ? -1 : 0;

	/* unlock */

	cd_mutex_unlock_double(cd0, cd1);

 done:
	if (u) {
#if (CC_AST_VERSION >= 0x10400)
	    ast_module_user_remove(u);
#else
	    LOCAL_USER_REMOVE(u);
#endif
	}

	return error;
}

/*
 * periodic thread
 *
 * Intention:
 *
 * Hence Asterisk does not support updating the extension,
 * after that the PBX has been started, one has to collect
 * the complete number first, before starting the PBX. That
 * is the job of this thread. Else one should just have 
 * forwarded the digits to the PBX, and a flag saying if
 * it is sending complete or not.
 */
static void *
do_periodic(void *data)
{
	struct cc_capi_application *p_app;
	struct ast_channel *pbx_chan;
	struct call_desc *cd;
	u_int32_t temp;
	u_int16_t x;

	while (1) {
	  
	    cc_mutex_lock(&do_periodic_lock);

	    for (x = 0; x < CAPI_MAX_APPLICATIONS; x++) {

	        p_app = capi_application[x];

		if (p_app) {

		    cc_mutex_lock(&p_app->lock);

		repeat:

		    cd = p_app->cd_root_ptr;
		    while (cd) {

			if (cd->pbx_chan && (cd->state == CAPI_STATE_DID)) {

			    /* compute time since last digit was received  */

			    temp = p_app->application_uptime - cd->digit_time_last;

			    if ((temp >= (cd->dst_telno[0] ? cd->options.digit_time_out : 15)) ||
				(cd->flags.sending_complete_received)) {

			        search_cep(cd);

				if (cd->cep) {
				    cd_start_pbx(&cd, &cd->dst_telno[0]);

				    if(cd == NULL) {
				      goto repeat;
				    }
				} else {


				}
			    }
			}

			/* the following is here just to avoid deadlocks: */

			if (cd->hangup_chan) {
			    pbx_chan = cd->hangup_chan;
			    cd->hangup_chan = NULL;
			    cc_mutex_unlock(&p_app->lock);
			    cc_verbose(3, 0, VERBOSE_PREFIX_4 "Out of order hangup, "
				       "pbx_chan=%p\n", pbx_chan);
			    ast_hangup(pbx_chan);
			    cc_mutex_lock(&p_app->lock);	
			    goto repeat;
			}

			/* the following is here just to avoid deadlocks: */

			if (cd->free_chan) {
			    pbx_chan = cd->free_chan;
			    cd->free_chan = NULL;
			    cc_mutex_unlock(&p_app->lock);
			    cc_verbose(3, 0, VERBOSE_PREFIX_4 "Out of order channel free, "
				       "pbx_chan=%p\n", pbx_chan);
			    ast_channel_free(pbx_chan);
			    cc_mutex_lock(&p_app->lock);
			    goto repeat;
			}

			/* XXX are there more things that must be 
			 * checked regularly ?
			 */

#warning "TODO: check for calls that never received connect_conf;"

			cd = cd->next;
		    }

		    /* increment the uptime */
		    p_app->application_uptime++;

		    /* keep the highest allocation rate so far */
		    if (p_app->cd_alloc_rate_record < 
			p_app->cd_alloc_rate_curr) {
		        p_app->cd_alloc_rate_record = 
			  p_app->cd_alloc_rate_curr;
		    }

		    /* reset the allocation rate */
		    p_app->cd_alloc_rate_curr = 0;

		    /* reset warning flag */
		    p_app->cd_alloc_rate_warned = 0;

		    cc_mutex_unlock(&p_app->lock);

		    cc_mutex_lock(&capi_global_lock);
		    temp  = update_use_count;
		    update_use_count = 0;
		    cc_mutex_unlock(&capi_global_lock);

		    if (temp) {
		        cc_verbose(3, 0, VERBOSE_PREFIX_4 "Out of order "
				 "update usecount!\n");
			ast_update_use_count();
		    }
		}
	    }

	    cc_mutex_unlock(&do_periodic_lock);

	    /* wait a little */
	    sleep(1);
	}
	return NULL;
}

/*
 * get supported services
 */
static void
capi_get_supported_sservices(struct cc_capi_application *p_app, 
			     struct cc_capi_controller *p_ctrl,
			     const u_int16_t controller_unit)
{
	_cmsg CMSG;
	u_int16_t to = 0x80;
	static const u_int8_t fac_struct[4] = { 3, 0, 0, 0 };

	FACILITY_REQ_HEADER(&CMSG, p_app->application_id, 
			    get_msg_num_other(p_app), controller_unit);
	FACILITY_REQ_FACILITYSELECTOR(&CMSG) = FACILITYSELECTOR_SUPPLEMENTARY;
	FACILITY_REQ_FACILITYREQUESTPARAMETER(&CMSG) = (_cstruct)&fac_struct;

	if (__capi_put_cmsg(p_app, &CMSG)) {
	    /* if the message was not written, 
	     * then no answer will appear, so
	     * just return;
	     */
	    cc_log(LOG_NOTICE, "could not send FACILITY REQUEST!\n");
	    return;
	}

	p_app->received_facility_conf = 0;
	p_app->temp_p_ctrl = p_ctrl;

	while((p_app->received_facility_conf == 0) && --to) {

	    capi_application_usleep(p_app, 10000);

	}

	p_app->temp_p_ctrl = NULL;
	return;
}

/* reload "capi.conf" */

static int
chan_capi_reload(int fd, int argc, char *argv[])
{
    u_int16_t error = 0;
    struct cc_capi_application *p_app = NULL;
    struct ast_config * cfg = NULL;

    if (argc != 2) {
        return RESULT_SHOWUSAGE;
    }
		
    /* load configuration file */

    cfg = ast_config_load((char *)config_file);

    if (!cfg) {
        if (fd >= 0) {
	    ast_cli(fd, "Unable to "
		    "load %s!\n", config_file);
	}
	goto done;
    }

    /* do the re-load */

    p_app = capi_application[0];

    if (p_app) {

	cc_mutex_lock(&p_app->lock);

	error = chan_capi_scan_config(cfg);

	if (error) {
	    goto done;
	}

	error = chan_capi_post_init(p_app);

	if (error) {
	    goto done;
	}
    }

 done:

    if (p_app) {
        cc_mutex_unlock(&p_app->lock);
    }

    if (cfg) {
        ast_config_destroy(cfg);
    }

    if(error) {
        if (fd >= 0) {
	    ast_cli(fd, "chan_capi reload "
		    "error=0x%04x\n", error);
	}
    }

    return RESULT_SUCCESS;
}

/* return information about a specific CAPI channel to the PBX */

static int
chan_capi_get_channel_info(int fd, int argc, char *argv[]) 
{
    struct call_desc *cd;
    struct cc_capi_application *p_app = capi_application[0];
    uint16_t plci;

    if ((argc != 4) || (argv[3] == NULL)) {
        return RESULT_SHOWUSAGE;
    }

    if (p_app == NULL) {
	ast_cli(fd, "No CAPI application!\n");
	return 0;
    }

    plci = strtoul(argv[3], NULL, 0) & 0xFFFF;

    if ((plci & 0xFF00) == 0x0000) {
	ast_cli(fd, "Invalid ID is addressing a controller and not a call!\n");
	return 0;
    }

    cc_mutex_lock(&(p_app->lock));

    cd = cd_by_plci(p_app, plci);

    if (cd) {
	ast_cli(fd, 
		"CAPI ID 0x%04x = {\n"
		"  direction = %s\n"
		"  source_nr = %s\n"
		"  dest_nr   = %s\n"
		"  state     = %d\n"
		"  on_hold   = %s\n"
		"}\n",
		cd->msg_plci,
		cd->flags.dir_outgoing ? "outgoing" : "incoming",
		cd->src_telno,
		cd->dst_telno,
		cd->state,
		cd->flags.hold_is_active ? "true" : "false");
    } else {
	ast_cli(fd, "No such active call ID!\n");
    }
    cc_mutex_unlock(&(p_app->lock));

    return 0;
}

/* return information about "chan_capi" to the PBX */

static int
chan_capi_get_info(int fd, int argc, char *argv[])
{
    struct cc_capi_application *p_app;
    struct config_entry_iface *cep;
    struct call_desc *cd;
    u_int16_t use_count_total[CAPI_MAX_CONTROLLERS] = { /* zero */ };
    u_int16_t use_count_on_hold[CAPI_MAX_CONTROLLERS] = { /* zero */ };
    u_int16_t n;
    u_int16_t x;

    if ((argc != 2) && (argc != 3)) {
        return RESULT_SHOWUSAGE;
    }
		
    for (n = 0; n < CAPI_MAX_APPLICATIONS; n++) {

        p_app = capi_application[n];

	if (p_app) {
	    cc_mutex_lock(&p_app->lock);
	    ast_cli(fd, 
		    "CAPI thread [0x%x] {\n"
		    " Application ID     : 0x%08x\n"
		    " Application uptime : 0x%08x seconds\n"
		    "\n"
		    " Call descriptor statistics:\n"
		    "    allocation count        : 0x%08x call descriptors\n"
		    "    free count              : 0x%08x call descriptors\n"
		    "    in memory count         : 0x%08x call descriptors\n"
		    "    in use count            : 0x%08x call descriptors\n"
		    "    record allocation rate  :     0x%04x calls/second\n"
		    "    limit allocation rate   :     0x%04x calls/second\n"
		    " Currently active calls:\n"
		    "    channel name, source->destination, call ID, in/out\n",
		    n,
		    p_app->application_id,
		    p_app->application_uptime,
		    p_app->cd_alloc_stats,
		    p_app->cd_free_stats,
		    p_app->cd_root_allocated,
		    p_app->cd_root_used,
		    p_app->cd_alloc_rate_record,
		    p_app->cd_alloc_rate_max);

	    /* collect per-controller statistics */

	    cd = p_app->cd_root_ptr;

	    while (cd) {

	        if (!CD_IS_UNUSED(cd)) {

		    x = (cd->msg_plci & 0xFF);

		    if (x > (CAPI_MAX_CONTROLLERS-1)) {
		        x = (CAPI_MAX_CONTROLLERS-1);
		    }

		    if (use_count_total[x] != 0xFFFF) {
		        use_count_total[x] ++;
		    }

		    if (cd->flags.hold_is_active) {

		        if (use_count_on_hold[x] != 0xFFFF) {
			    use_count_on_hold[x] ++;
			}
		    }

		    ast_cli(fd, "    %s: %s->%s, 0x%04x, %s\n",
			    cd->pbx_chan ? cd->pbx_chan->name : "<no PBX channel>",
			    cd->src_telno, cd->dst_telno,
			    cd->msg_plci, cd->flags.dir_outgoing ? "out" : "in");
		}
		cd = cd->next;
	    }
            cc_mutex_unlock(&p_app->lock);

	    ast_cli(fd, "}\n");
	}
    }

    for (n = 0; n < CAPI_MAX_CONTROLLERS; n++) {

        if (use_count_total[n]) {

	    ast_cli(fd,
		    "CAPI controller 0x%x {\n"
		    " active     : 0x%04x call descriptors\n"
		    " on hold    : 0x%04x call descriptors\n"
		    " B-channels : 0x%04x units\n"
		    "}\n",
		    n, 
		    use_count_total[n], 
		    use_count_on_hold[n],
		    capi_controller[n].b_channels_max);
	}
    }

    cep = cep_root_acquire();
    while(cep) {

        ast_cli(fd,
		"Config entry '%s' {\n"
		" b_channels_curr : %d call descriptor(s) free\n"
		" b_channels_max  : %d call descriptor(s) total\n"
		"}\n",
		cep->name,
		cep->b_channels_curr,
		cep->b_channels_max);

        cep = cep->next;
    }
    cep_root_release();

    ast_cli(fd, "\n");

    return RESULT_SUCCESS;
}

/* enable "chan_capi" debugging */

static int
chan_capi_enable_debug(int fd, int argc, char *argv[])
{
    if (argc != 2) {
        return RESULT_SHOWUSAGE;
    }
		
    capi_global.debug = 1;

    ast_cli(fd, "CAPI Debugging Enabled\n");
	
    return RESULT_SUCCESS;
}

/* disable "chan_capi" debugging */

static int
chan_capi_disable_debug(int fd, int argc, char *argv[])
{
    if (argc != 3) {
        return RESULT_SHOWUSAGE;
    }

    capi_global.debug = 0;

    ast_cli(fd, "CAPI Debugging Disabled\n");
	
    return RESULT_SUCCESS;
}

/*
 * define "chan_capi" commands
 */
static struct ast_cli_entry  cli_reload =
	{ { "capi", "reload", NULL }, chan_capi_reload,
	  "Reload CAPI configuration",
	  "Usage: capi reload\n"
	  "       Reload and merge new configuration with existing.\n" };

static struct ast_cli_entry  cli_info =
	{ { "capi", "info", NULL }, chan_capi_get_info, 
	  "Show CAPI info",
	  "Usage: capi info\n"
	  "       Show info about B channels.\n" };

static struct ast_cli_entry  cli_show_channel =
	{ { "capi", "show", "channel", NULL }, chan_capi_get_channel_info, 
	  "Show information about an active CAPI channel",
	  "Usage: capi show channel <id>\n"
	  "       Show info about a channel given by ID.\n" };

static struct ast_cli_entry  cli_show_channels =
	{ { "capi", "show", "channels", NULL }, chan_capi_get_info, 
	  "Show active CAPI channels",
	  "Usage: capi show channels\n"
	  "       Show info about B channels.\n" };

static struct ast_cli_entry  cli_debug =
	{ { "capi", "debug", NULL }, chan_capi_enable_debug, 
	  "Enable CAPI debugging",
	  "Usage: capi debug\n"
	  "       Enables dumping of CAPI packets for debugging purposes\n" };

static struct ast_cli_entry  cli_no_debug =
	{ { "capi", "no", "debug", NULL }, chan_capi_disable_debug, 
	  "Disable CAPI debugging", 
	  "Usage: capi no debug\n"
	  "       Disables dumping of CAPI packets for debugging purposes\n" };

#ifdef CC_AST_HAVE_TECH_PVT
const struct ast_channel_tech chan_capi_tech = {
	.type               = chan_capi_pbx_type,
	.description        = CHAN_CAPI_DESC,
	.capabilities       = AST_FORMAT_ALAW,
	.requester          = chan_capi_request,
#if (CC_AST_VERSION >= 0x10400)
	.send_digit_begin   = chan_capi_send_digit_begin,
	.send_digit_end     = chan_capi_send_digit_end,
#else
	.send_digit         = chan_capi_send_digit,
#endif
	.send_text          = NULL,
	.call               = chan_capi_call,
	.hangup             = chan_capi_hangup,
	.answer             = chan_capi_answer,
	.read               = chan_capi_read,
	.write              = chan_capi_write,
	.bridge             = chan_capi_bridge,
	.exception          = NULL,
	.indicate           = chan_capi_indicate,
	.fixup              = chan_capi_fixup,
	.setoption          = NULL,
#ifndef CC_AST_NO_DEVICESTATE
	.devicestate        = chan_capi_devicestate,
#endif
};
#else
static void
chan_capi_fill_pvt(struct ast_channel *pbx_chan)
{
    pbx_chan->pvt->call           = &chan_capi_call;
    pbx_chan->pvt->fixup          = &chan_capi_fixup;
    pbx_chan->pvt->indicate       = &chan_capi_indicate;
    pbx_chan->pvt->bridge         = &chan_capi_bridge;
    pbx_chan->pvt->answer         = &chan_capi_answer;
    pbx_chan->pvt->hangup         = &chan_capi_hangup;
    pbx_chan->pvt->read           = &chan_capi_read;
    pbx_chan->pvt->write          = &chan_capi_write;
    pbx_chan->pvt->send_digit     = &chan_capi_send_digit;
    return;
}
#endif

/* fill controller information */

static u_int16_t
chan_capi_fill_controller_info(struct cc_capi_application *p_app,
			       const u_int16_t controller_unit)
{
	struct cc_capi_profile profile;
	struct cc_capi_controller ctrl_temp;

	u_int16_t error;

	if (p_app == NULL) {
	    /* no application, nothing to do */
	    return 0;
	}

	cc_mutex_assert(&p_app->lock, MA_OWNED);

	bzero(&profile, sizeof(profile));

	bzero(&ctrl_temp, sizeof(ctrl_temp));

#if (CAPI_OS_HINT == 1)
	error = capi20_get_profile(controller_unit, (CAPIProfileBuffer_t *)&profile);
#elif (CAPI_OS_HINT == 2)
	error = capi20_get_profile(controller_unit, &profile, sizeof(profile));
#else
	error = capi20_get_profile(controller_unit, (u_int8_t *)&profile);
#endif

	if (error) {
	    cc_log(LOG_WARNING, "unable to get CAPI profile "
		   "for controller %d, error=0x%04x!\n", 
		   controller_unit, error);
	}

	ctrl_temp.valid = (error == 0);
	ctrl_temp.b_channels_max =
	  (profile.nbchannels[0] | 
	   (profile.nbchannels[1] << 8));

	if (profile.globaloptions[0] & 0x08) {
	    ctrl_temp.support.dtmf = 1;
	}
		
	if (profile.globaloptions[1] & 0x02) {
	    ctrl_temp.support.echo_cancel = 1;
	}

	if (profile.globaloptions[0] & 0x10) {
	    ctrl_temp.support.sservices = 1;
	    capi_get_supported_sservices(p_app, &ctrl_temp, 
					 controller_unit);
	}

	if (profile.globaloptions[0] & 0x80) {
	    ctrl_temp.support.lineinterconnect = 1;
	}

	cc_verbose(3, 0, VERBOSE_PREFIX_3 "CAPI controller %d "
		   "supports: "
		   "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n", controller_unit,
		   ctrl_temp.support.holdretrieve ? "[HOLD/RETRIEVE]" : "",
		   ctrl_temp.support.terminalportability ? "[TERMINAL PORTABILITY]" : "",
		   ctrl_temp.support.ECT ? "[ECT]" : "",
		   ctrl_temp.support.threePTY ? "[3PTY]" : "",
		   ctrl_temp.support.CF ? "[CF]" : "",
		   ctrl_temp.support.CD ? "[CD]" : "",
		   ctrl_temp.support.MCID ? "[MCID]" : "",
		   ctrl_temp.support.CCBS ? "[CCBS]" : "",
		   ctrl_temp.support.MWI ? "[MWI]" : "",
		   ctrl_temp.support.CCNR ? "[CCNR]" : "",
		   ctrl_temp.support.CONF ? "[CONF]" : "",
		   ctrl_temp.support.dtmf ? "[DTMF]" : "",
		   ctrl_temp.support.echo_cancel ? "[echo cancellation]" : "",
		   ctrl_temp.support.sservices ? "[supplementary]" : "",
		   ctrl_temp.support.lineinterconnect ? "[line interconnect]" : "");

	cc_mutex_lock(&capi_global_lock);
	bcopy(&ctrl_temp, &capi_controller[controller_unit], 
	      sizeof(capi_controller[0]));
	cc_mutex_unlock(&capi_global_lock);

	return error;
}

/* post-initialize "chan_capi" */

static u_int16_t
chan_capi_post_init(struct cc_capi_application *p_app)
{
	u_int16_t controller;

	for (controller = 0; 
	     controller < CAPI_MAX_CONTROLLERS; 
	     controller++) {

	    if (CC_GET_BIT(capi_controller_used_mask, controller)) {

	        chan_capi_fill_controller_info(p_app, controller);

	        if (capi_send_listen_req(p_app, controller, ALL_SERVICES)) {

		    cc_log(LOG_ERROR,"Unable to listen on "
			   "controller=%d!\n", controller);

		} else {

		    cc_verbose(2, 0, VERBOSE_PREFIX_3 "listening on "
			       "controller=%d, cip_mask=0x%08x\n",
			       controller, ALL_SERVICES);
		}

	    } else {
	        cc_verbose(3, 1, VERBOSE_PREFIX_3 "Unused "
			   "controller=%d\n", controller);
	    }
	}
	return 0; /* success */
}

#define CONF_GET_STRING(v, var, token)		\
  if (!strcasecmp((v)->name, token)) {		\
      strlcpy(var, (v)->value, sizeof(var));	\
      continue;					\
  }

#define CONF_GET_INTEGER(v, var, token)		\
  if (!strcasecmp((v)->name, token)) {		\
      (var) = atoi((v)->value);			\
      continue;					\
  }

#define CONF_GET_TRUE(v, var, token, val)	\
  if (!strcasecmp((v)->name, token)) {		\
      if (ast_true((v)->value)) {		\
	  (var) = (val);			\
      }						\
      continue;					\
  }

static void
capi_parse_controller_string(struct config_entry_iface *cep, const char *src)
{
	char temp[CAPI_MAX_STRING];
	char *last = NULL;
	char *curr;
	u_int16_t controller;

	if(src == NULL) return;

	strlcpy(temp, src, sizeof(temp));

	for (curr = strtok_r(temp, ",", &last);
	     curr;
	     curr = strtok_r(NULL, ",", &last))
	{
	    controller = atoi(curr);

	    if (controller < CAPI_MAX_CONTROLLERS) {
	        CC_SET_BIT(capi_controller_used_mask, controller);
		CC_SET_BIT(cep->controller_mask, controller);
	    } else {
	        cc_log(LOG_ERROR, "invalid CAPI controller "
		       "unit, %d! (ignored)\n", controller);
	    }
	}
	return;
}

static struct config_entry_iface *
capi_parse_iface_config(struct ast_variable *v, const char *name)
{
	struct config_entry_iface *cep;
	u_int16_t x;
	int16_t b_channels_max = 0;
	int16_t d_channels_max = 1;

	cep = cep_alloc(name);

	if(cep == NULL) {
	    goto done;
	}

	/* initialize global config entry and set defaults */

	cc_mutex_lock(&capi_global_lock);

	cep->rx_gain = capi_global.rx_gain;
	cep->tx_gain = capi_global.tx_gain;
	cep->options.digit_time_out = capi_global.digit_time_out;
	strlcpy(cep->language, capi_global.default_language, sizeof(cep->language));

	cc_mutex_unlock(&capi_global_lock);

	cep->options.echo_cancel_option = EC_OPTION_DISABLE_G165;
	cep->options.echo_cancel_tail = EC_DEFAULT_TAIL;
	cep->options.echo_cancel_selector = FACILITYSELECTOR_ECHO_CANCEL;
	cep->options.echo_suppress_offset = EC_POWER_OFFSET;

	for (; v; v = v->next) {
	    CONF_GET_INTEGER(v, b_channels_max, "devices");
	    CONF_GET_INTEGER(v, b_channels_max, "b_channels");
	    CONF_GET_INTEGER(v, d_channels_max, "d_channels");

	    CONF_GET_STRING(v, cep->context, "context");
	    CONF_GET_STRING(v, cep->incomingmsn, "incomingmsn");
	    CONF_GET_STRING(v, cep->dst_default, "defaultcid");

	    if ((!strcasecmp(v->name, "controller")) ||
		(!strcasecmp(v->name, "controller+")))
	    {
	        capi_parse_controller_string(cep, v->value);
		continue;
	    }

	    CONF_GET_STRING(v, cep->prefix, "prefix");
	    CONF_GET_STRING(v, cep->accountcode, "accountcode");
	    CONF_GET_STRING(v, cep->language, "language");

	    CONF_GET_INTEGER(v, cep->options.wait_silence_samples, 
			     "wait_silence_samples");

	    CONF_GET_INTEGER(v, cep->options.digit_time_out, "digit_timeout");

	    CONF_GET_TRUE(v, cep->options.dtmf_detect_in_software, "softdtmf", 1);
	    CONF_GET_TRUE(v, cep->options.dtmf_detect_relax, "relaxdtmf", 1);

	    if (!strcasecmp(v->name, "isdnmode")) {
	        if (!strcasecmp(v->value, "msn")) {
		    cep->options.send_complete_force = 1;
		}
		continue;
	    }

	    if (!strcasecmp(v->name, "holdtype")) {
	        if (!strcasecmp(v->value, "hold")) {

		    cep->options.hold_type = CC_HOLDTYPE_HOLD;

		} else if (!strcasecmp(v->value, "notify")) {

		    cep->options.hold_type = CC_HOLDTYPE_NOTIFY;

		} else {

		    cep->options.hold_type = CC_HOLDTYPE_LOCAL;

		}
		continue;
	    }

	    CONF_GET_TRUE(v, cep->options.immediate, "immediate", 1);
	    CONF_GET_TRUE(v, cep->options.bridge, "bridge", 1);
	    CONF_GET_TRUE(v, cep->options.ntmode, "ntmode", 1);
	    CONF_GET_TRUE(v, cep->options.dtmf_generate, "dtmf_generate", 1);

	    if((!strcasecmp(v->name, "echosquelch")) ||
	       (!strcasecmp(v->name, "echo_squelch")) ||
	       (!strcasecmp(v->name, "echosuppress")) ||
	       (!strcasecmp(v->name, "echo_suppress"))) {

	        if (strcasecmp(v->value, "fax") == 0) {
		    cep->options.echo_suppress_fax = 1;
		    cep->options.echo_suppress_in_software = 1;
		} else if (ast_true(v->value)) {
		    cep->options.echo_suppress_fax = 0;
		    cep->options.echo_suppress_in_software = 1;
		} else if (ast_false(v->value)) {
		    cep->options.echo_suppress_fax = 0;
		    cep->options.echo_suppress_in_software = 0;
		} else {
		    cc_log(LOG_ERROR, "Unknown echosquelch or "
			   "echo_suppress parameter '%s' (ignored)\n",
			   v->value);
		}
		continue;
	    }

	    if (!strcasecmp(v->name, "callgroup")) {
	        cep->call_group = ast_get_group(v->value);
		continue;
	    }
	    if (!strcasecmp(v->name, "group")) {
	        cep->group = ast_get_group(v->value);
		continue;
	    }
	    if (!strcasecmp(v->name, "rxgain")) {
	        if (sscanf(v->value, "%f", &cep->rx_gain) != 1) {
		    cc_log(LOG_ERROR, "invalid CAPI rxgain (ignored)\n");
		}
		continue;
	    }
	    if (!strcasecmp(v->name, "txgain")) {
	        if (sscanf(v->value, "%f", &cep->tx_gain) != 1) {
		    cc_log(LOG_ERROR, "invalid CAPI txgain (ignored)\n");
		}
		continue;
	    }
	    if (!strcasecmp(v->name, "echocancelold")) {
	        if (ast_true(v->value)) {
		    cep->options.echo_cancel_selector = 6;
		}
		continue;
	    }

	    if ((!strcasecmp(v->name, "echocancel")) ||
		(!strcasecmp(v->name, "echo_cancel"))) {

	        if (ast_true(v->value)) {
		    cep->options.echo_cancel_in_hardware = 1;
		    cep->options.echo_cancel_option = EC_OPTION_DISABLE_G165;
		}	
		else if (ast_false(v->value)) {
		    cep->options.echo_cancel_in_hardware = 0;
		    cep->options.echo_cancel_option = 0;
		}	
		else if (!strcasecmp(v->value, "g165") || !strcasecmp(v->value, "g.165")) {
		    cep->options.echo_cancel_in_hardware = 1;
		    cep->options.echo_cancel_option = EC_OPTION_DISABLE_G165;
		}	
		else if (!strcasecmp(v->value, "g164") || !strcasecmp(v->value, "g.164")) {
		    cep->options.echo_cancel_in_hardware = 1;
		    cep->options.echo_cancel_option = EC_OPTION_DISABLE_G164_OR_G165;
		}	
		else if (!strcasecmp(v->value, "force")) {
		    cep->options.echo_cancel_in_hardware = 1;
		    cep->options.echo_cancel_option = EC_OPTION_DISABLE_NEVER;
		}
		else if((!strcasecmp(v->value, "soft")) ||
			(!strcasecmp(v->value, "software"))) {
		    cep->options.echo_cancel_in_software = 1;
		}
		else {
		    cc_log(LOG_ERROR, "Unknown echocancel parameter "
			   "'%s' (ignored)\n", v->value);
		}
		continue;
	    }

	    if (!strcasecmp(v->name, "echotail")) {
	        cep->options.echo_cancel_tail = atoi(v->value);
		if (cep->options.echo_cancel_tail > 255) {
		    cep->options.echo_cancel_tail = 255;
		} 
		continue;
	    }

	    if (!strcasecmp(v->name, "echo_offset")) {
	        cep->options.echo_suppress_offset = atoi(v->value);
		if (cep->options.echo_suppress_offset < 1) {
		    cep->options.echo_suppress_offset = 1;
		}
		if (cep->options.echo_suppress_offset > EC_WINDOW_COUNT) {
		    cep->options.echo_suppress_offset = EC_WINDOW_COUNT;
		} 
		continue;
	    }

	    cc_log(LOG_ERROR, "Unknown parameter "
		   "'%s' = '%s' (ignored)\n", v->name, 
		   v->value ? v->value : "");
	}

	/* some parameters have implications */

	if(cep->options.dtmf_detect_relax) {
	    cep->options.dtmf_detect_in_software = 1;
	}

	cc_mutex_lock(&capi_global_lock);

	cep->d_channels_curr += d_channels_max;
	cep->b_channels_curr += b_channels_max;

	cep->d_channels_max = d_channels_max;
	cep->b_channels_max = b_channels_max;

	cc_mutex_unlock(&capi_global_lock);

	cep_init_convert_tables(cep);

	for (x = 0; x < CAPI_MAX_CONTROLLERS; x++) {

	  if(CC_GET_BIT(cep->controller_mask,x)) {
	      if(cep->controller_first == 0) {
		  cep->controller_first = x;
	      }
	      cep->controller_last = x;
	  }
	}

	cc_verbose(2, 0, VERBOSE_PREFIX_3 "config entry '%s' T=(%s,%s,%d) "
		   "C=[%d,%d] E=(%d,%d,%d,%d) G=(%f/%f) H=(%d)\n",
		   cep->name, 
		   cep->incomingmsn, 
		   cep->context, 
		   cep->b_channels_max,
		   cep->controller_first,
		   cep->controller_last,
		   cep->options.echo_cancel_in_hardware, 
		   cep->options.echo_cancel_option, 
		   cep->options.echo_cancel_tail,
		   cep->options.echo_suppress_in_software, 
		   cep->rx_gain,
		   cep->tx_gain, 
		   cep->call_group);

 done:
	return cep;
}

static void
chan_capi_parse_global_config(struct ast_variable *v,
			      struct config_entry_global *cep)
{
	/* initialize global config entry and set defaults */

	bzero(cep, sizeof(*cep));

	strlcpy(cep->national_prefix, CAPI_NATIONAL_PREF, 
		sizeof(cep->national_prefix));

	strlcpy(cep->international_prefix, CAPI_INTERNAT_PREF, 
		sizeof(cep->international_prefix));

	cep->rx_gain = 1.0;
	cep->tx_gain = 1.0;
	cep->capability = AST_FORMAT_ALAW;
	cep->digit_time_out = 5; /* seconds */

	/* parse the general section */

	for (; v; v = v->next) {

	    if (!strcasecmp(v->name, "nationalprefix")) {

	        strlcpy(cep->national_prefix, v->value, 
			sizeof(cep->national_prefix));

	    } else if (!strcasecmp(v->name, "internationalprefix")) {

	        strlcpy(cep->international_prefix, v->value, 
			sizeof(cep->international_prefix));

	    } else if (!strcasecmp(v->name, "language")) {

	        strlcpy(cep->default_language, v->value, 
			sizeof(cep->default_language));

	    } else if (!strcasecmp(v->name, "rxgain")) {

	        if (sscanf(v->value, "%f", &cep->rx_gain) != 1) {
		    cc_log(LOG_ERROR,"invalid rxgain\n");
		}

	    } else if (!strcasecmp(v->name, "txgain")) {

	        if (sscanf(v->value, "%f", &cep->tx_gain) != 1) {
		    cc_log(LOG_ERROR,"invalid txgain\n");
		}

	    } else if (!strcasecmp(v->name, "ulaw")) {

	        if (ast_true(v->value)) {
		    cep->capability = AST_FORMAT_ULAW;
		}

	    } else if (!strcasecmp(v->name, "debug")) {

	        cep->debug = ast_true(v->value) ? 1 : 0;

	    } else if (!strcasecmp(v->name, "digit_timeout")) {

	        cep->digit_time_out = atoi(v->value);

		if(cep->digit_time_out > 32) {
		   cep->digit_time_out = 32; /* seconds */
		}
	    }
	}
	return;
}

/* scan "capi.conf" */

static int
chan_capi_scan_config(struct ast_config *cfg)
{
	struct config_entry_iface *cep = NULL;
	struct config_entry_global capi_global_shadow;
	char *cat = NULL;

	/* unload existing configuration */

	cep_unload();

	chan_capi_parse_global_config
	  (ast_variable_browse(cfg, "general"), &capi_global_shadow);

	cc_mutex_lock(&capi_global_lock);
	capi_global = capi_global_shadow;
	cc_mutex_unlock(&capi_global_lock);

	/* go through the other sections, which are the interfaces */

	while((cat = ast_category_browse(cfg, cat))) {

		if (!strcasecmp(cat, "general")) {
			continue;
		}

		if (!strcasecmp(cat, "interfaces")) {
			cc_log(LOG_WARNING, "Config file syntax has changed! "
			       "Don't use 'interfaces'\n");
			return -1;
		}

		cc_verbose(4, 0, VERBOSE_PREFIX_2 "Reading config "
			   "for %s\n", cat);

		cep = capi_parse_iface_config(ast_variable_browse(cfg, cat), 
					      cat);

		if (cep) {
		    cep_root_prepend(cep);
		} else {
		    cc_log(LOG_ERROR, "Error interface config.\n");
		    return -1;
		}
	}
	return 0;
}

#ifdef CC_AST_CUSTOM_FUNCTION
/*
 * convert letters into digits according to international keypad
 */
static char *
chan_capi_vanitynumber(struct ast_channel *chan, char *cmd, char *data, 
		       char *buf, size_t len)
{
	u_int8_t c;
	int pos;
	
	*buf = 0;

	if (!data) {
		cc_log(LOG_WARNING, "This function requires a parameter name.\n");
		return NULL;
	}

	for (pos = 0; (pos < strlen(data)) && (pos < len); pos++) {
		c = toupper(data[pos]);
		switch(c) {
		case 'A': case 'B': case 'C':
			buf[pos] = '2';
			break;
		case 'D': case 'E': case 'F':
			buf[pos] = '3';
			break;
		case 'G': case 'H': case 'I':
			buf[pos] = '4';
			break;
		case 'J': case 'K': case 'L':
			buf[pos] = '5';
			break;
		case 'M': case 'N': case 'O':
			buf[pos] = '6';
			break;
		case 'P': case 'Q': case 'R': case 'S':
			buf[pos] = '7';
			break;
		case 'T': case 'U': case 'V':
			buf[pos] = '8';
			break;
		case 'W': case 'X': case 'Y': case 'Z':
			buf[pos] = '9';
			break;
		default:
			buf[pos] = data[pos];
		}
	}
	buf[pos] = 0;

	return buf;
}

static struct ast_custom_function vanitynumber_function = {
	.name = "VANITYNUMBER",
	.synopsis = "Vanity number: convert letter into digits according to international dialpad.",
	.syntax = "VANITYNUMBER(<vanitynumber to convert>)",
	.read = chan_capi_vanitynumber,
};
#endif

/*
 * main: load the module
 */
#if (CC_AST_VERSION >= 0x10400)
static
#endif
int load_module(void)
{
	struct cc_capi_application *p_app = NULL;
	struct ast_config * cfg = NULL;
	u_int8_t app_locked = 0;
	int error = 0;


	/* first initialize some mutexes */

	cc_mutex_init(&do_periodic_lock);

	cc_mutex_init(&capi_global_lock);

	cc_mutex_init(&capi_verbose_lock);


	/* load configuration file */

	cfg = ast_config_load((char *)config_file);

	if (!cfg) {
	    cc_log(LOG_ERROR, "Unable to load the "
		   "config file, %s!\n", config_file);
	    error = 0; /* ignore it */
	    goto done;
	}
	chan_capi_load_level = 1;

	p_app = capi_application_alloc();

	if (p_app == NULL) {
	    error = -1;
	    goto done;
	}
	chan_capi_load_level = 2;

	/* NOTE: In the future one wants more than one application
	 * to take full advantage of SMP enabled systems:
	 */
	capi_application[0] = p_app;

	cc_mutex_lock(&p_app->lock);

	app_locked = 1;

	error = chan_capi_scan_config(cfg);

	if (error) {
	    goto done;
	}
	chan_capi_load_level = 4;

	error = chan_capi_post_init(p_app);

	if (error) {
	    goto done;
	}
	chan_capi_load_level = 5;

	cc_mutex_unlock(&p_app->lock);
	app_locked = 0;
	
#ifdef CC_AST_HAVE_TECH_PVT
	error = ast_channel_register(&chan_capi_tech);
#else	
	error = ast_channel_register(chan_capi_pbx_type, CHAN_CAPI_DESC,
				     capi_global.capability, chan_capi_request);
#endif
	if (error) {
	    cc_log(LOG_ERROR, "Unable to register channel "
		   "class %s\n", chan_capi_pbx_type);
	    goto done;
	}
	chan_capi_load_level = 6;

	error = ast_cli_register(&cli_reload);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_reload\n");
	    goto done;
	}
	chan_capi_load_level = 8;

	error = ast_cli_register(&cli_info);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_info\n");
	    goto done;
	}
	chan_capi_load_level = 9;

	error = ast_cli_register(&cli_show_channels);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_show_channels\n");
	    goto done;
	}
	chan_capi_load_level = 10;

	error = ast_cli_register(&cli_debug);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_debug\n");
	    goto done;
	}
	chan_capi_load_level = 11;

	error = ast_cli_register(&cli_show_channel);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_show_channel\n");
	    goto done;
	}
	chan_capi_load_level = 12;

	error = ast_cli_register(&cli_no_debug);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register cli_no_debug\n");
	    goto done;
	}
	chan_capi_load_level = 16;

	error = ast_register_application
	  (CHAN_CAPI_APP, &chan_capi_command_exec,
	   "Execute special CAPI commands",
	   "CAPI command interface.");

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register CAPI application\n");
	    goto done;
	}
	chan_capi_load_level = 20;

#ifdef CC_AST_CUSTOM_FUNCTION
	error = ast_custom_function_register(&vanitynumber_function);

	if (error) {
	    cc_log(LOG_ERROR, "Unable to register vanitynumber_function!\n");
	    goto done;
	}
	chan_capi_load_level = 24;
#endif

	chan_capi_load_level = 28;

	error = ast_pthread_create(&periodic_thread, NULL, do_periodic, NULL);

	if (error) {
		cc_log(LOG_ERROR, "Unable to start periodic thread!\n");
		goto done;
	}
	chan_capi_load_level = 32;

 done:
	if (app_locked) {
	    cc_mutex_unlock(&p_app->lock);
	}

	if (cfg) {
	    ast_config_destroy(cfg);
	}

	if (error) {
	    unload_module();
	    cc_log(LOG_WARNING, "CAPI is disabled!\n");
	    error = -1; /* force default error */
	}
	return error;
}

/*
 * unload the module
 */
#if (CC_AST_VERSION >= 0x10400)
static
#endif
int unload_module()
{
	u_int16_t x;

	switch (chan_capi_load_level) {

	case 32:
	    pthread_cancel(periodic_thread);
	    pthread_kill(periodic_thread, SIGURG);
	    pthread_join(periodic_thread, NULL);

	case 28:

	case 24:
#ifdef CC_AST_CUSTOM_FUNCTION
	    ast_custom_function_unregister(&vanitynumber_function);
#endif
	case 20:
	    ast_unregister_application(CHAN_CAPI_APP);
	case 16:
	    ast_cli_unregister(&cli_no_debug);
	case 12:
	    ast_cli_unregister(&cli_show_channel);
	case 11:
	    ast_cli_unregister(&cli_debug);
	case 10:
	    ast_cli_unregister(&cli_show_channels);
	case 9:
	    ast_cli_unregister(&cli_info);
	case 8:
	    ast_cli_unregister(&cli_reload);
	case 6:
#ifdef CC_AST_HAVE_TECH_PVT
	    ast_channel_unregister(&chan_capi_tech);
#else
	    ast_channel_unregister(chan_capi_pbx_type);
#endif
	case 5:
	case 4:
	case 3:
	case 2:
	    cc_mutex_lock(&do_periodic_lock);
	    for(x = 0; x < CAPI_MAX_APPLICATIONS; x++) {
	        if (capi_application[x]) {
		    capi_application_free(capi_application[x]);
		    capi_application[x] = NULL;
		}
	    }
	    cc_mutex_unlock(&do_periodic_lock);

	case 1:
	case 0:
	    if (capi_global.debug) {
	        cc_log(LOG_NOTICE, "'chan_capi' unloaded from "
		       "level %d\n", chan_capi_load_level);
	    }
	    chan_capi_load_level = 0;
	    break;

	default:
	    cc_log(LOG_ERROR, "Invalid 'chan_capi' load level: %d\n",
		   chan_capi_load_level);
	    break;
	}
	return 0;
}

#if (CC_AST_VERSION >= 0x10400)
static int
reload_module(void)
{
	return chan_capi_reload(-1,2,0);
}

AST_MODULE_INFO(ASTERISK_GPL_KEY, AST_MODFLAG_DEFAULT, CHAN_CAPI_DESC,
		.load = &load_module,
		.unload = &unload_module,
		.reload = &reload_module);
#else
int usecount()
{
	struct cc_capi_application *p_app;
	int cd_root_used = 0;
	u_int16_t x;

	for(x = 0; x < CAPI_MAX_APPLICATIONS; x++) {

	    p_app = capi_application[x];

	    if (p_app) {
	        cc_mutex_lock(&p_app->lock);

	        if (p_app->cd_root_used > 0) {
		    cd_root_used += p_app->cd_root_used;
		}

		cc_mutex_unlock(&p_app->lock);
	    }
	}
	return cd_root_used;
}

char *description()
{
	return "Common ISDN API 2.0, CAPI2.0, for Asterisk";
}

#ifdef ASTERISK_GPL_KEY
char *key()
{
	return ASTERISK_GPL_KEY;
}
#endif
#endif
