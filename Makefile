#
#
# chan_capi Makefile
#
#

OSNAME=${shell uname}

.EXPORT_ALL_VARIABLES:

INSTALL_PREFIX=

ASTERISK_HEADER_DIR=$(INSTALL_PREFIX)/usr/include

ifeq (${OSNAME},FreeBSD)
ASTERISK_HEADER_DIR=$(INSTALL_PREFIX)/usr/local/include
endif

ifeq (${OSNAME},NetBSD)
ASTERISK_HEADER_DIR=$(INSTALL_PREFIX)/usr/pkg/include
endif

ASTERISKVERSION=$(shell if [ -f .version ]; then cat .version; else if [ -d CVS ]; then if [ -f CVS/Tag ] ; then echo "CVS-`sed 's/^T//g' CVS/Tag`-`date +"%D-%T"`"; else echo "CVS-HEAD-`date +"%D-%T"`"; fi; fi; fi)

MODULES_DIR=$(INSTALL_PREFIX)/usr/lib/asterisk/modules

ifeq (${OSNAME},FreeBSD)
MODULES_DIR=$(INSTALL_PREFIX)/usr/local/lib/asterisk/modules
endif

ifeq (${OSNAME},NetBSD)
MODULES_DIR=$(INSTALL_PREFIX)/usr/pkg/lib/asterisk/modules
endif

CONFIG_DIR=$(INSTALL_PREFIX)/etc/asterisk

ifeq (${OSNAME},FreeBSD)
CONFIG_DIR=$(INSTALL_PREFIX)/usr/local/etc/asterisk
endif

ifeq (${OSNAME},NetBSD)
CONFIG_DIR=$(INSTALL_PREFIX)/usr/pkg/etc/asterisk
endif

PROC=$(shell uname -m)

DEBUG=-g #-pg
INCLUDE=-I$(ASTERISK_HEADER_DIR)
ifeq (${OSNAME},FreeBSD)
INCLUDE+=$(shell [ -d /usr/include/i4b/include ] && \
	echo -n -I/usr/include/i4b/include)
endif
ifeq (${OSNAME},NetBSD)
INCLUDE+=$(shell [ -d /usr/include/i4b/include ] && \
        echo -n -I/usr/include/i4b/include)
endif
CFLAGS=-pipe -fPIC -Wall -Wmissing-prototypes -Wmissing-declarations $(DEBUG) $(INCLUDE) -D_REENTRANT -D_GNU_SOURCE
CFLAGS+=$(OPTIMIZE)
CFLAGS+=-O6
CFLAGS+=$(shell if $(CC) -march=$(PROC) -S -o /dev/null -xc /dev/null >/dev/null 2>&1; then echo "-march=$(PROC)"; fi)
CFLAGS+=$(shell if uname -m | grep -q ppc; then echo "-fsigned-char"; fi)
CFLAGS+=-Wformat

CFLAGS+=-DASTERISKVERSION=\"$(ASTERISKVERSION)\"

LIBS=-ldl -lpthread -lm
CC=gcc
INSTALL=install

SHAREDOS=chan_capi.so

OBJECTS=chan_capi.o c20msg.o

CFLAGS+=-Wno-missing-prototypes -Wno-missing-declarations

CFLAGS+=-DCRYPTO

all: config.h $(SHAREDOS)

clean:
	rm -f config.h
	rm -f *.so *.o

config.h: ${ASTERISK_HEADER_DIR}/asterisk/version.h
	./create_config.sh "$(ASTERISK_HEADER_DIR)"

chan_capi.so: $(OBJECTS)
	$(CC) -shared -Xlinker -x -o $@ $^ -lcapi20

install: all
	$(INSTALL) -d -m 755 $(MODULES_DIR)
	for x in $(SHAREDOS); do $(INSTALL) -m 755 $$x $(MODULES_DIR) ; done

install_config: capi.conf
	$(INSTALL) -d -m 755 ${CONFIG_DIR}
	$(INSTALL) -m 644 capi.conf ${CONFIG_DIR}

samples: install_config

