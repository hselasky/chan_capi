/*-
 *
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2005 Cytronics & Melware, Armin Schindler
 * Copyright (c) 2002-2005 Junghanns.NET GmbH, Klaus-Peter Junghanns
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
 */
#ifndef __CHAN_CAPI_H__
#define __CHAN_CAPI_H__

#define CAPI_MAX_CONTROLLERS             64
#define CAPI_MAX_APPLICATIONS             8
#define CAPI_MAX_B3_BLOCKS                7

/* was : 130 bytes Alaw = 16.25 ms audio not suitable for VoIP */
/* now : 160 bytes Alaw = 20 ms audio */
/* you can tune this to your need. higher value == more latency */
#define CAPI_MAX_B3_BLOCK_SIZE          160
#define CAPI_MAX_QLEN                    50 /* frames */

#define CAPI_BCHANS                     120
#define ALL_SERVICES             0x1FFF03FF

#define FIFO_BLOCK_START ((CAPI_MAX_B3_BLOCKS+1)/2) /* blocks */
#define FIFO_BLOCK_SIZE  (CAPI_MAX_B3_BLOCK_SIZE) /* bytes */
#define FIFO_BF_SIZE     (CAPI_MAX_B3_BLOCKS * FIFO_BLOCK_SIZE) /* bytes */

struct ring_buffer {
    u_int8_t  data[FIFO_BF_SIZE];

    u_int16_t bf_read_pos;
    u_int16_t bf_write_pos;
    u_int16_t bf_free_len;
    u_int16_t bf_used_len;

    u_int16_t end_pos;
    u_int8_t  last_byte;
};

/*
 * helper for ast_verbose with different verbose settings
 */
#define cc_verbose(o_v, c_d, fmt, ...)				\
  do {								\
      if ((o_v == 0) || (option_verbose > o_v)) {		\
	  if ((!c_d) || ((c_d) && (capi_global.debug))) {	\
	      ast_mutex_lock(&capi_verbose_lock);		\
	      ast_verbose(fmt,## __VA_ARGS__);			\
	      ast_mutex_unlock(&capi_verbose_lock);		\
	  }							\
      }								\
  } while(0)

#define cd_verbose(cd,a,b,c,fmt,...) \
  cc_verbose(a,b,VERBOSE_PREFIX_##c "%s:%d:ENTRY=%s:PLCI=0x%04x:" \
	     "PBX_CHAN=%s:\n" VERBOSE_PREFIX_##c "  " fmt, \
	     __PRETTY_FUNCTION__, __LINE__, \
	     (((cd) && (cd)->cep) ? (const char *)(cd)->cep->name : \
	      (const char *)""), ((cd) ? (cd)->msg_plci : 0), \
	     (((cd) && (cd)->pbx_chan) ? (const char *)(cd)->pbx_chan->name : \
	      (const char *)"") ,## __VA_ARGS__)

/*
 * PBX mutex wrappers
 */
#define CC_MUTEX_DEFINE_STATIC AST_MUTEX_DEFINE_STATIC
#define cc_mutex_init(x) ast_mutex_init(x)
#define cc_mutex_lock(x) { if(ast_mutex_lock(x) || ast_mutex_lock(x)) cc_log(LOG_NOTICE, "locking error\n"); }
#define cc_mutex_unlock(x) { if(ast_mutex_unlock(x) || ast_mutex_unlock(x)) cc_log(LOG_NOTICE, "double unlock\n"); }
#define cc_mutex_assert(x,what) __cc_mutex_assert(x,what,__FILE__,__PRETTY_FUNCTION__,__LINE__)
#define MA_OWNED        0x01
#define MA_NOTOWNED     0x02
#define MA_RECURSED     0x04
#define MA_NOTRECURSED  0x08

#define cd_log(cd,x,fmt,...) cc_log(x,fmt,## __VA_ARGS__)
#define cc_log(...) ast_log(__VA_ARGS__)

/*
 * definitions for compatibility with older versions of ast*
 */
#ifdef CC_AST_HAVE_TECH_PVT
#define CC_CHANNEL_PVT(c) (c)->tech_pvt
#else
#define CC_CHANNEL_PVT(c) (c)->pvt->pvt
#endif

#ifdef CC_AST_HAS_BRIDGED_CHANNEL
#define CC_AST_BRIDGED_CHANNEL(x) ast_bridged_channel(x)
#else
#define CC_AST_BRIDGED_CHANNEL(x) (x)->bridge
#endif

#ifdef CC_AST_HAS_BRIDGE_RESULT
#define CC_BRIDGE_RETURN enum ast_bridge_result
#else
#define CC_BRIDGE_RETURN int
#define AST_BRIDGE_COMPLETE        0
#define AST_BRIDGE_FAILED         -1
#define AST_BRIDGE_FAILED_NOWARN  -2
#define AST_BRIDGE_RETRY          -3
#endif

#ifndef AST_MUTEX_DEFINE_STATIC
#define AST_MUTEX_DEFINE_STATIC(mutex)		\
	static ast_mutex_t mutex = AST_MUTEX_INITIALIZER
#endif

/* FAX Resolutions */
#define FAX_STANDARD_RESOLUTION         0
#define FAX_HIGH_RESOLUTION             1

/* FAX Formats */
#define FAX_SFF_FORMAT                  0
#define FAX_PLAIN_FORMAT                1
#define FAX_PCX_FORMAT                  2
#define FAX_DCX_FORMAT                  3
#define FAX_TIFF_FORMAT                 4
#define FAX_ASCII_FORMAT                5
#define FAX_EXTENDED_ASCII_FORMAT       6
#define FAX_BINARY_FILE_TRANSFER_FORMAT 7

/* duration in ms for sending and detecting dtmfs */
#define CAPI_DTMF_DURATION              0x40 /* ms */

#define CAPI_NATIONAL_PREF               "0"
#define CAPI_INTERNAT_PREF              "00"

#define FACILITYSELECTOR_DTMF              1
#define FACILITYSELECTOR_SUPPLEMENTARY     3
#define FACILITYSELECTOR_LINE_INTERCONNECT 5
#define FACILITYSELECTOR_ECHO_CANCEL       8

#define CC_HOLDTYPE_LOCAL               0
#define CC_HOLDTYPE_HOLD                1
#define CC_HOLDTYPE_LOCAL_MOH           2

/*
 * state combination for a normal incoming call:
 * DIS -> INCALL / DID -> ALERT -> CON -> DIS
 *
 * outgoing call:
 * DIS -> CONP -> CON -> DIS
 */

#define CAPI_STATE_NULL                 0 /* default */

/* incoming call states */

#define CAPI_STATE_INCALL               1
#define CAPI_STATE_ALERTING             2
#define CAPI_STATE_DID                  3
#define CAPI_STATE_ANSWERING            4

/* outgoing call states */

#define CAPI_STATE_CONNECTPENDING       5

/* neutral call states */

#define CAPI_STATE_CONNECTED            6

#define CAPI_STATE_DISCONNECTED         7

#define CAPI_STATE_ONHOLD               8

#define CAPI_MAX_STRING              2048

struct cc_capi_support {

	/* features: */
	u_int32_t dtmf : 1;
	u_int32_t echo_cancel : 1;
	u_int32_t sservices : 1; /* supplementray services */
	u_int32_t lineinterconnect : 1;

	/* supported sservices: */
	u_int32_t holdretrieve : 1;
	u_int32_t terminalportability : 1;
	u_int32_t ECT : 1;
	u_int32_t threePTY : 1;
	u_int32_t CF : 1;
	u_int32_t CD : 1;
	u_int32_t MCID : 1;
	u_int32_t CCBS : 1;
	u_int32_t MWI : 1;
	u_int32_t CCNR : 1;
	u_int32_t CONF : 1;
};

struct cc_capi_flags {

	/* set if outgoing call: */
	u_int32_t dir_outgoing : 1;

	/* set if sending complete has been received */
	u_int32_t sending_complete_received : 1;

	/* set if "release complete" should be sent upon calldescriptor free */
	u_int32_t send_release_complete : 1;

	/* set if the destination is not using ISDN */
	u_int32_t dst_telno_is_not_isdn : 1;

	/* set if "chan_capi" should set FAX protocol 
	 * when B-channel has been disconnected
	 */
	u_int32_t fax_set_prot_on_b3_disc : 1;

	/* set if "chan_capi" is pending to receive a FAX */
	u_int32_t fax_pending : 1;

	/* set if "chan_capi" is receiving a FAX */
	u_int32_t fax_receiving : 1;

	/* set if "chan_capi" has already handled a FAX */
	u_int32_t fax_handled : 1;

	/* set if "chan_capi" received a FAX in error */
	u_int32_t fax_error : 1;

	/* set if early B3 is enabled on progress */
	u_int32_t b3_on_progress : 1;

	/* set if early B3 is enabled on alert */
	u_int32_t b3_on_alert : 1;

	/* set if the B3 link is active or up */
	u_int32_t b3_active : 1;

	/* set if the B3 link is pending to become active */
	u_int32_t b3_pending : 1;

	/* set if this call descriptor is on hold */
	u_int32_t hold_is_active : 1;

	/* set if this call descriptor is pending to or from hold */
	u_int32_t hold_is_pending : 1;

	/* set if line interconnect is active */
	u_int32_t line_interconnect_active : 1;

	/* set if "disconnect" has been received */
	u_int32_t disconnect_received : 1;

	/* set if "progress" has been sent */
	u_int32_t progress_transmitted : 1;

	/* set if "progress" has been received */
	u_int32_t progress_received : 1;

	/* set if "ECT" is pending */
	u_int32_t ect_pending : 1;

	/* set if "ECT" should be sent on B3 disc. indication */
	u_int32_t send_ect_on_b3_disc : 1;

	/* set if SETUP message was received */
	u_int32_t setup_received : 1;

	/* set if the PBX extension search ended */
	u_int32_t pbx_search_complete : 1;

	/* set if the PBX has been started */
	u_int32_t pbx_started : 1;

	/* set if the PBX has been set to state UP */
	u_int32_t pbx_state_up : 1;

	/* set if the "CONNECTEDNUMBER" variable was set */
	u_int32_t connected_number_set : 1;

	/* set if the "SETUP" message was received */
	u_int32_t received_setup : 1;

        /* set if retrive req should be sent on hold ind */
	u_int32_t send_retrieve_req_on_hold_ind : 1;

	/* set if late inband signalling is enabled */
	u_int32_t want_late_inband : 1;
};

struct cc_capi_options {

	/* set if NT-mode is selected: */
	u_int32_t ntmode : 1;

	/* set if "s" extension should be used 
	 * for empty destination telephone numbers:
	 */
	u_int32_t immediate : 1;

	/* set if sending complete should be forced */
	u_int32_t send_complete_force : 1;

	/* set if line interconnect is allowed: */
	u_int32_t bridge : 1;

	/* set if hardware echo canceling is enabled */
	u_int32_t echo_cancel_in_hardware : 1;

	/* set if software echo canceling is enabled */
	u_int32_t echo_cancel_in_software : 1;

	/* set if DTMF detection should be done in software. 
	 * Else in hardware:
	 */
	u_int32_t dtmf_detect_in_software : 1;

	/* set if relaxed DTMF detection should be done */
	u_int32_t dtmf_detect_relax : 1;

	/* set if DTMF should be generated by CAPI */
	u_int32_t dtmf_generate : 1;

	/* CAPI echo cancel parameters (active CAPI devices) */
	u_int16_t echo_cancel_option;
	u_int16_t echo_cancel_tail;
	u_int16_t echo_cancel_selector;

	/* CAPI digit timeout, in seconds */
	u_int16_t digit_time_out;

	/* CAPI wait silence, in samples */
	u_int16_t wait_silence_samples;

	/* CAPI hold type */
	u_int8_t hold_type;
};

/* global config entry structure */

struct config_entry_global {
	char national_prefix[AST_MAX_EXTENSION];
	char international_prefix[AST_MAX_EXTENSION];
	char default_language[MAX_LANGUAGE];
	char debug;
	float rx_gain;
	float tx_gain;
	int capability;
	u_int16_t digit_time_out; /* in seconds */
};

/* interface config entry structure */

struct config_entry_iface {

	/* interface name */
	char name[AST_MAX_EXTENSION];

	/* counters to keep track of 
	 * available B- and D-channels:
	 */
	int16_t b_channels_max;
	int16_t b_channels_curr;

	int16_t d_channels_max;
	int16_t d_channels_curr;

  /* NOTE: all fields between the 
   * dummies are zeroed by default
   */

        u_int8_t dummy_zero_start[1];

	/* language */
	char language[MAX_LANGUAGE];

	/*! Multiple Subscriber Number "chan_capi" listens to, 
	 * as a comma separated list:
	 */
	char incomingmsn[CAPI_MAX_STRING];	

	/*! Default destination telephone number */
	char dst_default[AST_MAX_EXTENSION];

	char context[AST_MAX_EXTENSION];

	/*! Prefix for source telephone number */
	char prefix[AST_MAX_EXTENSION];
	char accountcode[20];

	struct cc_capi_options options;

	u_int32_t call_group;
	u_int32_t group;

#define CC_GET_BIT(var,x)			\
  ((var)[(x)/8] & (1 << ((x) % 8)))

#define CC_SET_BIT(var,x)			\
  (var)[(x)/8] |= (1 << ((x) % 8))

	/* controller bit-mask */
	u_int8_t controller_mask[(CAPI_MAX_CONTROLLERS+7)/8];

	u_int8_t controller_first; /* inclusive */
	u_int8_t controller_last; /* inclusive */

	float tx_gain;
	float rx_gain;

	u_int8_t tx_convert[256];
	u_int8_t rx_convert[256];

	struct config_entry_iface *next;

	u_int8_t dummy_zero_end[1];
};

struct cc_capi_application;

/* CAPI call descriptor structure */

struct call_desc {

	/*! PBX DSP pointer */
	struct ast_dsp *pbx_dsp;

	/*! CAPI application pointer */
	struct cc_capi_application *p_app;

	/* CAPI data pipe */
	int fd[2];

	/* --- START OF ZERO DEFAULT REGION --- 
	 *
	 * "cd_free()" will automatically zero
	 * all variables in this region!
	 */
	u_int8_t dummy_zero_start[1];

	/*! CAPI echo canceller ring buffer */
	struct ring_buffer ring_buf;

	struct timeval rx_time;

	struct ast_channel *hangup_chan;
	struct ast_channel *free_chan;

	/*! PBX capability */
	int32_t pbx_capability;

	/*! PBX temporary read frame */
	struct ast_frame pbx_rd;

	/*! PBX channel pointer */
	struct ast_channel *pbx_chan;

	/*! CAPI time of last received digit */
	u_int32_t digit_time_last;

	/*! CAPI white noise generator remainder */
	u_int32_t white_noise_rem;

	/*! CAPI configuration entry pointer */
	struct config_entry_iface *cep;
	
	/*! CAPI message number */
	u_int16_t msg_num;

	/*! CAPI connection indentificators */
	u_int16_t msg_plci;
	u_int32_t msg_ncci;

	/*! CAPI explicit call transfer indentificator */
	u_int16_t ect_plci;

	/*! CAPI Common ISDN Profile, CIP */
	u_int16_t msg_cip;

	/*! state of call descriptor */
	u_int8_t state;

	/*! CAPI flags */
	struct cc_capi_flags flags;

	/*! CAPI options (copy from config) */
	struct cc_capi_options options;

	/*! CAPI controller features (copy from config) */
	struct cc_capi_support support;

	u_int8_t bchannelinfo[4];

	/*! CAPI source telephone number, if available */
	char src_telno[AST_MAX_EXTENSION];	

	/*! CAPI source telephone number type */
	u_int8_t src_ton;

	/*! CAPI source telephone number presentation */
	u_int8_t src_pres;

	/*! CAPI destination telephone number, if available */
	char dst_telno[AST_MAX_EXTENSION];

	/*! CAPI destination telephone number type */
	u_int8_t dst_ton;

	/*! Number of bytes to strip from the beginning of the
	 *  telephone number.
	 */
	u_int16_t dst_strip_len;

	/*! CAPI transmit queue length */
	u_int16_t tx_queue_len;

	/*! CAPI receive buffer */
	u_int8_t  rx_buffer_data[(CAPI_MAX_B3_BLOCK_SIZE + 
				  AST_FRIENDLY_OFFSET) * CAPI_MAX_B3_BLOCKS];

#define RX_BUFFER_BY_HANDLE(cd, handle) \
  ((cd)->rx_buffer_data + AST_FRIENDLY_OFFSET + \
   ((CAPI_MAX_B3_BLOCK_SIZE + AST_FRIENDLY_OFFSET) * (handle)))

	u_int16_t rx_buffer_len[CAPI_MAX_B3_BLOCKS];

	u_int16_t rx_buffer_handle;
	u_int16_t rx_noise_count;

	/*! CAPI hangup cause received */
	u_int16_t wCause_in;

	/*! CAPI B3 hangup cause received */
	u_int16_t wCause_in_b3;

	/*! CAPI hangup cause transmitted */
	u_int16_t wCause_out;

	/*! CAPI receive FAX file pointer */
	FILE *fax_file;

	/*! CAPI receive FAX file name, malloced, if any */
	char *fax_fname;

	/*! CAPI structure: B3 config */
	u_int8_t b3_config[128];

	/**/
	u_int8_t channel_type;

	/* last received DTMF digit */
	int last_dtmf_digit;

	/* current number of silence bytes */
	u_int16_t wait_silence_count;

	/* --- END OF ZERO DEFAULT REGION --- */

	u_int8_t dummy_zero_end[1];

	/*! Next call descriptor in list */
	struct call_desc *next;
};

struct cc_capi_profile {

	/* NOTE: all fields are little endian */

	u_int8_t ncontrollers[2];
	u_int8_t nbchannels[2];
	u_int8_t globaloptions[4];
	u_int8_t b1protocols[4];
	u_int8_t b2protocols[4];
	u_int8_t b3protocols[4];
	u_int8_t reserved3[6*4];
	u_int8_t manufacturer[5*4];

} __attribute__((__packed__));

struct cc_capi_application {

	/*! lock protecting this structure and
	 *  all associated call descriptors:
	 */
	ast_mutex_t lock;

	/*! CAPI application ID */
	u_int32_t application_id;

	/*! CAPI application uptime, in seconds */
	u_int32_t application_uptime;
  
	/*! number of CAPI applications sleeping */
	u_int16_t sleep_count;

	/*! CAPI message number (other) */
	u_int16_t message_number_other;

	/*! CAPI message number (dial) */
	u_int16_t message_number_dial;

	/* current call descriptor allocation rate */
	u_int16_t cd_alloc_rate_curr;

	/* the highest call descriptor allocation rate so far */
	u_int16_t cd_alloc_rate_record;

	/* maximum call descriptor allocation rate (exclusive) */
	u_int16_t cd_alloc_rate_max;

	/* call descriptor allocation count (total) */
        u_int32_t cd_alloc_stats;

	/* call descriptor free count (total) */
        u_int32_t cd_free_stats;

	/* number of call descriptors currently in use */
	int32_t cd_root_used;

	/* number of call descriptors currently in memory */
	int32_t cd_root_allocated;

	/* call descriptor root pointer */
	struct call_desc *cd_root_ptr;

	/* temporary controller pointer */	
	struct cc_capi_controller *temp_p_ctrl;

	pthread_t monitor_thread;
	u_int8_t monitor_thread_created : 1;
	u_int8_t cd_alloc_rate_warned : 1;
	u_int8_t received_listen_conf : 1;
	u_int8_t received_facility_conf : 1;
};

struct cc_capi_controller {

	u_int16_t b_channels_max;

	/* valid:
	 * ======
	 * 1: profile was retrieved without
	 *    error.
	 * 0: profile was retrieved with 
	 *    error.
	 */
	u_int8_t valid; 

	struct cc_capi_support support;
};

/* ETSI 300 102-1 information element identifiers */
#define CAPI_ETSI_IE_CAUSE                      0x08
#define CAPI_ETSI_IE_PROGRESS_INDICATOR         0x1e
#define CAPI_ETSI_IE_CALLED_PARTY_NUMBER        0x70

/* ETIS 300 102-1 message types */
#define CAPI_ETSI_ALERTING                      0x01
#define CAPI_ETSI_SETUP_ACKKNOWLEDGE            0x0d
#define CAPI_ETSI_DISCONNECT                    0x45

/* ETSI 300 102-1 Numbering Plans */
#define CAPI_ETSI_NPLAN_NATIONAL                0x20
#define CAPI_ETSI_NPLAN_INTERNAT                0x10

/* Common ISDN Profiles (CIP) */
#define CAPI_CIPI_SPEECH                        0x01
#define CAPI_CIPI_DIGITAL                       0x02
#define CAPI_CIPI_RESTRICTED_DIGITAL            0x03
#define CAPI_CIPI_3K1AUDIO                      0x04
#define CAPI_CIPI_7KAUDIO                       0x05
#define CAPI_CIPI_VIDEO                         0x06
#define CAPI_CIPI_PACKET_MODE                   0x07
#define CAPI_CIPI_56KBIT_RATE_ADAPTION          0x08
#define CAPI_CIPI_DIGITAL_W_TONES               0x09
#define CAPI_CIPI_TELEPHONY                     0x10
#define CAPI_CIPI_FAX_G2_3                      0x11
#define CAPI_CIPI_FAX_G4C1                      0x12
#define CAPI_CIPI_FAX_G4C2_3                    0x13
#define CAPI_CIPI_TELETEX_PROCESSABLE           0x14
#define CAPI_CIPI_TELETEX_BASIC                 0x15
#define CAPI_CIPI_VIDEOTEX                      0x16
#define CAPI_CIPI_TELEX                         0x17
#define CAPI_CIPI_X400                          0x18
#define CAPI_CIPI_X200                          0x19
#define CAPI_CIPI_7K_TELEPHONY                  0x1a
#define CAPI_CIPI_VIDEO_TELEPHONY_C1            0x1b
#define CAPI_CIPI_VIDEO_TELEPHONY_C2            0x1c

/* Transfer capabilities */
#define PRI_TRANS_CAP_SPEECH                    0x00
#define PRI_TRANS_CAP_DIGITAL                   0x08
#define PRI_TRANS_CAP_RESTRICTED_DIGITAL        0x09
#define PRI_TRANS_CAP_3K1AUDIO                  0x10
#define PRI_TRANS_CAP_DIGITAL_W_TONES           0x11
#define PRI_TRANS_CAP_VIDEO                     0x18

#endif /* __CHAN_CAPI_H__ */
