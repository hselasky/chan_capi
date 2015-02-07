/*-
 *
 * Copyright (c) 2006,2013 Hans Petter Selasky. All rights reserved.
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
#define	CAPI_MAX_HOSTNAME		256
#define	CAPI_MAX_PORTNAME		 32
#define	CAPI_MAX_BACKENDNAME		 32
#define	CAPI_MAX_USERNAME		128
#define	CAPI_MAX_PASSWORD		256

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
    uint8_t  data[FIFO_BF_SIZE];

    uint16_t bf_read_pos;
    uint16_t bf_write_pos;
    uint16_t bf_free_len;
    uint16_t bf_used_len;

    uint16_t end_pos;
    uint8_t  last_byte;
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
	     (((cd) && (cd)->pbx_chan) ? (const char *)CC_CHANNEL_NAME((cd)->pbx_chan) : \
	      (const char *)"") ,## __VA_ARGS__)

/*
 * PBX mutex wrappers
 */
#define CC_MUTEX_DEFINE_STATIC AST_MUTEX_DEFINE_STATIC
#define cc_mutex_init(x) ast_mutex_init(x)
#define cc_mutex_lock(x) do { ast_mutex_lock(x); } while (0)
#define cc_mutex_unlock(x) do { ast_mutex_unlock(x); } while (0)
#define cc_mutex_assert(x,what) do { } while (0)
#define MA_OWNED        0x01
#define MA_NOTOWNED     0x02
#define MA_RECURSED     0x04
#define MA_NOTRECURSED  0x08

#define cd_log(cd,x,fmt,...) cc_log(x,fmt,## __VA_ARGS__)
#define cc_log(...) ast_log(__VA_ARGS__)

/*
 * definitions for compatibility with older versions of ast*
 */
#if (CC_AST_VERSION >= 0x110000)
#define CC_CHANNEL_NAME(chan) ast_channel_name(chan)
#define	CC_CHANNEL_PVT(c) ast_channel_tech_pvt(c)
#define	CC_CHANNEL_SET_PVT(c,x) ast_channel_tech_pvt_set((c), (x))
#define	CC_CHANNEL_HANGUPCAUSE(c) ast_channel_hangupcause(c)
#define	CC_CHANNEL_SET_HANGUPCAUSE(c,v) ast_channel_hangupcause_set(c,v)
#define	CC_CHANNEL_RINGS(c) ast_channel_rings(c)
#define	CC_CHANNEL_SET_RINGS(c,v) ast_channel_rings_set(c,v)
#define	CC_CHANNEL_CONTEXT(c) ast_channel_context(c)
#define	CC_CHANNEL_NATIVEFORMATS(c) ast_channel_nativeformats(c)
#define	CC_CHANNEL_PBX(c) ast_channel_pbx(c)
#define	CC_CHANNEL_STATE(c) ast_channel_state(c)
#define	CC_CHANNEL_CDR(c) ast_channel_cdr(c)
#define	CC_CHANNEL_EXTEN(c) ast_channel_exten(c)

#define	CC_CHANNEL_SET_NATIVEFORMATS(c,v) ast_channel_nativeformats_set(c,v)
#define	CC_CHANNEL_SET_CALLGROUP(c,x) ast_channel_callgroup_set(c,x)
#define	CC_CHANNEL_SET_CONTEXT(c,x) ast_channel_context_set(c,x)
#define	CC_CHANNEL_SET_ACCOUNTCODE(c,x) ast_channel_accountcode_set(c,x)
#define	CC_CHANNEL_SET_LANGUAGE(c,x) ast_channel_language_set(c,x)
#define	CC_CHANNEL_SET_EXTEN(c,v) ast_channel_exten_set(c,v)
#define	CC_CHANNEL_SET_PRIORITY(c,v) ast_channel_priority_set(c,v)
#define	CC_CHANNEL_SET_FLAG(c,v) ast_set_flag(ast_channel_flags(c),(v))
#else

#ifdef CC_AST_HAVE_TECH_PVT
#define	CC_CHANNEL_PVT(c) (c)->tech_pvt
#else
#define	CC_CHANNEL_PVT(c) (c)->pvt->pvt
#endif

#define	CC_CHANNEL_NAME(chan) ((chan)->name)
#define	CC_CHANNEL_HANGUPCAUSE(c) (c)->hangupcause
#define	CC_CHANNEL_RINGS(c) (c)->rings
#define	CC_CHANNEL_NATIVEFORMATS(c) (c)->nativeformats
#define	CC_CHANNEL_CALLGROUP(c) (c)->callgroup
#define	CC_CHANNEL_CONTEXT(c) (c)->context
#define	CC_CHANNEL_PBX(c) (c)->pbx
#define	CC_CHANNEL_STATE(c) (c)->_state
#define	CC_CHANNEL_CDR(c) (c)->cdr
#define	CC_CHANNEL_EXTEN(c) (c)->exten

#define	CC_CHANNEL_SET_PVT(c,x) CC_CHANNEL_PVT(c) = (x)
#define	CC_CHANNEL_SET_HANGUPCAUSE(c,x) CC_CHANNEL_HANGUPCAUSE(c) = (x)
#define	CC_CHANNEL_SET_RINGS(c,x) CC_CHANNEL_RINGS(c) = (x)
#define	CC_CHANNEL_SET_NATIVEFORMATS(c,x) CC_CHANNEL_NATIVEFORMATS(c) = (x)
#define	CC_CHANNEL_SET_CALLGROUP(c,x) CC_CHANNEL_CALLGROUP(c) = (x)
#define	CC_CHANNEL_SET_CONTEXT(c,x) strlcpy(CC_CHANNEL_CONTEXT(c), (x), sizeof(CC_CHANNEL_CONTEXT(c)))
#define	CC_CHANNEL_SET_EXTEN(c,x) strlcpy(CC_CHANNEL_EXTEN(c), (x), sizeof(CC_CHANNEL_EXTEN(c)))
#define	CC_CHANNEL_SET_PRIORITY(c,x) (c)->priority = (x)
#define	CC_CHANNEL_SET_FLAG(c,v) ast_set_flag(c,v)

#if (CC_AST_VERSION >= 0x10400)
#define	CC_CHANNEL_SET_ACCOUNTCODE(c,x) \
	ast_string_field_set((c), accountcode, (x));
#define	CC_CHANNEL_SET_LANGUAGE(c,x) \
	ast_string_field_set((c), language, (x));
#else
#define	CC_CHANNEL_SET_ACCOUNTCODE(c,x) \
	strlcpy((c)->accountcode, (x), sizeof((c)->accountcode));
#define	CC_CHANNEL_SET_LANGUAGE(c,x) \
	strlcpy((c)->language, (x), sizeof((c)->language));
#endif
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
	uint32_t dtmf : 1;
	uint32_t echo_cancel : 1;
	uint32_t sservices : 1; /* supplementray services */
	uint32_t lineinterconnect : 1;

	/* supported sservices: */
	uint32_t holdretrieve : 1;
	uint32_t terminalportability : 1;
	uint32_t ECT : 1;
	uint32_t threePTY : 1;
	uint32_t CF : 1;
	uint32_t CD : 1;
	uint32_t MCID : 1;
	uint32_t CCBS : 1;
	uint32_t MWI : 1;
	uint32_t CCNR : 1;
	uint32_t CONF : 1;
};

struct cc_capi_flags {

	/* set if outgoing call: */
	uint32_t dir_outgoing : 1;

	/* set if sending complete has been received */
	uint32_t sending_complete_received : 1;

	/* set if "release complete" should be sent upon calldescriptor free */
	uint32_t send_release_complete : 1;

	/* set if the destination is not using ISDN */
	uint32_t dst_telno_is_not_isdn : 1;

	/* set if "chan_capi" should set FAX protocol 
	 * when B-channel has been disconnected
	 */
	uint32_t fax_set_prot_on_b3_disc : 1;

	/* set if "chan_capi" is pending to receive a FAX */
	uint32_t fax_pending : 1;

	/* set if "chan_capi" is receiving a FAX */
	uint32_t fax_receiving : 1;

	/* set if "chan_capi" has already handled a FAX */
	uint32_t fax_handled : 1;

	/* set if "chan_capi" received a FAX in error */
	uint32_t fax_error : 1;

	/* set if early B3 is enabled on progress */
	uint32_t b3_on_progress : 1;

	/* set if early B3 is enabled on alert */
	uint32_t b3_on_alert : 1;

	/* set if the B3 link is active or up */
	uint32_t b3_active : 1;

	/* set if the B3 link is pending to become active */
	uint32_t b3_pending : 1;

	/* set if this call descriptor is on hold */
	uint32_t hold_is_active : 1;

	/* set if this call descriptor is pending to or from hold */
	uint32_t hold_is_pending : 1;

	/* set if line interconnect is active */
	uint32_t line_interconnect_active : 1;

	/* set if "disconnect" has been received */
	uint32_t disconnect_received : 1;

	/* set if "progress" has been sent */
	uint32_t progress_transmitted : 1;

	/* set if "progress" has been received */
	uint32_t progress_received : 1;

	/* set if "ECT" is pending */
	uint32_t ect_pending : 1;

	/* set if "ECT" should be sent on B3 disc. indication */
	uint32_t send_ect_on_b3_disc : 1;

	/* set if we have seen the destination Type Of Number */
	uint32_t seen_dst_ton : 1;

	/* set if the PBX extension search ended */
	uint32_t pbx_search_complete : 1;

	/* set if the PBX has been started */
	uint32_t pbx_started : 1;

	/* set if the PBX has been set to state UP */
	uint32_t pbx_state_up : 1;

	/* set if the "CONNECTEDNUMBER" variable was set */
	uint32_t connected_number_set : 1;

	/* set if the "SETUP" message was received */
	uint32_t received_setup : 1;

        /* set if retrive req should be sent on hold ind */
	uint32_t send_retrieve_req_on_hold_ind : 1;

	/* set if late inband signalling is enabled */
	uint32_t want_late_inband : 1;
};

struct cc_capi_options {

	/* set if NT-mode is selected: */
	uint32_t ntmode : 1;

	/* set if "s" extension should be used 
	 * for empty destination telephone numbers:
	 */
	uint32_t immediate : 1;

	/* converts ton info to (inter)national prefix: */
	uint32_t ton2digit : 1;

	/* set if call proceeding should not be sent
	 * immediately after an incomplete setup
	 */
	uint32_t late_callproc : 1;

	/* set if sending complete should be forced */
	uint32_t send_complete_force : 1;

	/* set if line interconnect is allowed: */
	uint32_t bridge : 1;

	/* set if hardware echo canceling is enabled */
	uint32_t echo_cancel_in_hardware : 1;

	/* set if software echo canceling is enabled */
	uint32_t echo_cancel_in_software : 1;

	/* set if DTMF detection should be done in software. 
	 * Else in hardware:
	 */
	uint32_t dtmf_detect_in_software : 1;

	/* set if relaxed DTMF detection should be done */
	uint32_t dtmf_detect_relax : 1;

	/* set if DTMF should be generated by CAPI */
	uint32_t dtmf_generate : 1;

	/* CAPI echo cancel parameters (active CAPI devices) */
	uint16_t echo_cancel_option;
	uint16_t echo_cancel_tail;
	uint16_t echo_cancel_selector;

	/* CAPI digit timeout, in seconds */
	uint16_t digit_time_out;

	/* CAPI alert timeout, in seconds */
	uint16_t alert_time_out;

	/* CAPI wait silence, in samples */
	uint16_t wait_silence_samples;

	/* CAPI hold type */
	uint8_t hold_type;
};

/* global config entry structure */

struct config_entry_global {
	char host[CAPI_MAX_HOSTNAME];
	char port[CAPI_MAX_PORTNAME];
	char backend[CAPI_MAX_BACKENDNAME];
	char user[CAPI_MAX_USERNAME];
	char pass[CAPI_MAX_PASSWORD];
	char national_prefix[AST_MAX_EXTENSION];
	char international_prefix[AST_MAX_EXTENSION];
	char default_language[MAX_LANGUAGE];
	char debug;
	float rx_gain;
	float tx_gain;
	int capability;
	uint16_t digit_time_out; /* in seconds */
	uint16_t alert_time_out; /* in seconds */
	uint8_t ton2digit;
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

        uint8_t dummy_zero_start[1];

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

	uint32_t call_group;
	uint32_t group;

#define CC_GET_BIT(var,x)			\
  ((var)[(x)/8] & (1 << ((x) % 8)))

#define CC_SET_BIT(var,x)			\
  (var)[(x)/8] |= (1 << ((x) % 8))

	/* controller bit-mask */
	uint8_t controller_mask[(CAPI_MAX_CONTROLLERS+7)/8];

	uint8_t controller_first; /* inclusive */
	uint8_t controller_last; /* inclusive */

	float tx_gain;
	float rx_gain;

	uint8_t tx_convert[256];
	uint8_t rx_convert[256];

	struct config_entry_iface *next;

	uint8_t dummy_zero_end[1];
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
	uint8_t dummy_zero_start[1];

	/*! CAPI echo canceller ring buffer */
	struct ring_buffer ring_buf;

	uint64_t rx_time_us;

	struct ast_channel *hangup_chan;
	struct ast_channel *free_chan;

	/*! PBX capability */
	int32_t pbx_capability;

	/*! PBX temporary read frame */
	struct ast_frame pbx_rd;

	/*! PBX channel pointer */
	struct ast_channel *pbx_chan;

	/*! CAPI time of last received digit */
	uint32_t digit_time_last;

	/*! CAPI time proceeding was received */
	uint32_t proc_time_last;

	/*! CAPI time alert was received */
	uint32_t alert_time_last;

	/*! CAPI white noise generator remainder */
	uint32_t white_noise_rem;

	/*! CAPI configuration entry pointer */
	struct config_entry_iface *cep;
	
	/*! CAPI message number */
	uint16_t msg_num;

	/*! CAPI connection indentificators */
	uint16_t msg_plci;
	uint32_t msg_ncci;

	/*! CAPI explicit call transfer indentificator */
	uint16_t ect_plci;

	/*! CAPI Common ISDN Profile, CIP */
	uint16_t msg_cip;

	/*! state of call descriptor */
	uint8_t state;

	/*! CAPI flags */
	struct cc_capi_flags flags;

	/*! CAPI options (copy from config) */
	struct cc_capi_options options;

	/*! CAPI controller features (copy from config) */
	struct cc_capi_support support;

	uint8_t bchannelinfo[4];

	/*! CAPI source telephone number, if available */
	char src_telno[AST_MAX_EXTENSION];	

	/*! CAPI source telephone number type */
	uint8_t src_ton;

	/*! CAPI source telephone number presentation */
	uint8_t src_pres;

	/*! CAPI destination telephone number, if available */
	char dst_telno[AST_MAX_EXTENSION];

	/*! CAPI destination telephone number type */
	uint8_t dst_ton;

	/*! Number of bytes to strip from the beginning of the
	 *  telephone number.
	 */
	uint16_t dst_strip_len;

	/*! Channel name
	 */
	char chan_name[AST_MAX_EXTENSION];

	/*! CAPI transmit queue length */
	uint16_t tx_queue_len;

	/*! CAPI receive buffer */
	uint8_t  rx_buffer_data[(CAPI_MAX_B3_BLOCK_SIZE + 
				  AST_FRIENDLY_OFFSET) * CAPI_MAX_B3_BLOCKS];

#define RX_BUFFER_BY_HANDLE(cd, handle) \
  ((cd)->rx_buffer_data + \
   ((CAPI_MAX_B3_BLOCK_SIZE + AST_FRIENDLY_OFFSET) * (handle)))

	uint16_t rx_buffer_handle;

	/*! CAPI hangup cause received */
	uint16_t wCause_in;

	/*! CAPI B3 hangup cause received */
	uint16_t wCause_in_b3;

	/*! CAPI hangup cause transmitted */
	uint16_t wCause_out;

	/*! CAPI receive FAX file pointer */
	FILE *fax_file;

	/*! CAPI receive FAX file name, malloced, if any */
	char *fax_fname;

	/*! CAPI structure: B3 config */
	uint8_t b3_config[128];

	/**/
	uint8_t channel_type;

	/* last received DTMF digit */
	int last_dtmf_digit;

	/* current number of silence bytes */
	uint16_t wait_silence_count;

	/* received proceeding */
	uint8_t proc_received:1;

	/* received alert */
	uint8_t alert_received:1;

	/* --- END OF ZERO DEFAULT REGION --- */

	uint8_t dummy_zero_end[1];

	/*! Next call descriptor in list */
	struct call_desc *next;
};

struct cc_capi_profile {

	/* NOTE: all fields are little endian */

	uint8_t ncontrollers[2];
	uint8_t nbchannels[2];
	uint8_t globaloptions[4];
	uint8_t b1protocols[4];
	uint8_t b2protocols[4];
	uint8_t b3protocols[4];
	uint8_t reserved3[6*4];
	uint8_t manufacturer[5*4];

} __attribute__((__packed__));

struct cc_capi_application {

	/*! lock protecting this structure and
	 *  all associated call descriptors:
	 */
	ast_mutex_t lock;

	struct ast_channel *pbx_chan_temp;

	/*! CAPI application ID */
	uint32_t application_id;

	/*! CAPI application uptime, in seconds */
	uint32_t application_uptime;
  
	/*! number of CAPI applications sleeping */
	uint16_t sleep_count;

	/*! CAPI message number (other) */
	uint16_t message_number_other;

	/*! CAPI message number (dial) */
	uint16_t message_number_dial;

	/* current call descriptor allocation rate */
	uint16_t cd_alloc_rate_curr;

	/* the highest call descriptor allocation rate so far */
	uint16_t cd_alloc_rate_record;

	/* maximum call descriptor allocation rate (exclusive) */
	uint16_t cd_alloc_rate_max;

	/* call descriptor allocation count (total) */
        uint32_t cd_alloc_stats;

	/* call descriptor free count (total) */
        uint32_t cd_free_stats;

	/* number of call descriptors currently in use */
	int32_t cd_root_used;

	/* number of call descriptors currently in memory */
	int32_t cd_root_allocated;

	/* call descriptor root pointer */
	struct call_desc *cd_root_ptr;

	/* temporary controller pointer */	
	struct cc_capi_controller *temp_p_ctrl;

	/* CAPI backend pointer */
	struct capi20_backend *cbe_p;

	pthread_t monitor_thread;
	uint8_t monitor_thread_created : 1;
	uint8_t cd_alloc_rate_warned : 1;
	uint8_t received_listen_conf : 1;
	uint8_t received_facility_conf : 1;
};

struct cc_capi_controller {

	uint16_t b_channels_max;

	/* valid:
	 * ======
	 * 1: profile was retrieved without
	 *    error.
	 * 0: profile was retrieved with 
	 *    error.
	 */
	uint8_t valid; 

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
#define CAPI_ETSI_NPLAN_OTHER                   0x00

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
