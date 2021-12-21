<IMG SRC="https://raw.githubusercontent.com/hselasky/chan_capi/main/www/chan_capi.jpg"></IMG>

# ABOUT

Chan-capi-hps is a channel driver for various open-source PBX systems like
<A HREF="http://www.asterisk.org">Asterisk</A>, <A HREF="http://www.openpbx.org">OpenPBX</A>,
<A HREF="http://www.callweaver.org">CallWeaver</A> and
compatible. Chan-capi-hps allows you to connect your ISDN devices
running in NT- and TE-mode to the above mentions PBX systems, using
the <A HREF="http://www.capi.org/">CAPI2.0 standard</A>.

# AUTHORS

- Hans Petter Selasky <hps@selasky.org>
- Armin Schindler <armin@melware.de>

# DEBUGGERS, BUGFIXERS AND CONTRIBUTORS

- Lele Forzani <lele@windmill.it>
- Florian Overkamp <florian@obsimref.com>
- Gareth Watts <gareth@omnipotent.net>
- Jeff Noxon <jeff@planetfall.com>
- Petr Michalek <petr.michalek@aca.cz>
- Jan Stocker <Jan.Stocker@t-online.de>
- Frank Sautter, levigo group
- Rob Thomas <xrobau@gmail.com>
- Klaus-Peter Junghanns <kpj@junghanns.net>

- ...and all the others that have been forgotten :-)


# FEATURES

- Incoming and outgoing calls
- Overlap sending (dialtone and additional digits)
- DID: dial in digits
- CID,DNID (callling party, called party)
- CLIR/CLIP
- *CAPI CD: call deflection
- *CAPI ECT: explicit call transfer
- *CAPI HOLD
- *CAPI RETRIVE
- *CAPI Line Interconnect
- *CAPI DTMF detection
- Sofware DTMF detection
- Early B3 connects (always,success,never)
- Alaw and Ulaw support 
- ACO: Reject call waiting
- Tuneable latency, see "CAPI_MAX_B3_BLOCK_SIZE" in "chan_capi.h".
- Eicon CAPI echo cancelation (echocancel=1)
- Rx/Tx gains (rxgain=1.0)
- [Inter-] national dialing prefix (for callerid) configurable in "capi.conf"
- receive FAX using CAPI

* if supported by the CAPI controller


# SUPPORTED OPERATING SYSTEMS

- Linux
- NetBSD
- FreeBSD

# DEVICE PERMISSIONS

OpenPBX.org, by default, runs as the non-root user/group
openpbx/openpbx.  You must make sure that the /dev/capi* device files
are readable by OpenPBX.org either by changing the ownership or the
permissions of the the device files or by running OpenPBX.org as root.

# EXAMPLE CONFIGURATION FILES

See "capi.conf" and "extensions.conf"

# CAPI COMMAND APPLICATION

See example "extensions.conf".

<PRE>
ECT:
    Explicit call transfer of the call on hold (must put call on hold first!)
        [macro-capiect]
        exten => s,1,capiCommand(ect)
        [default]
        exten => s,1,capiCommand(hold)
        exten => s,2,Wait(1)
        exten => s,3,Dial(CAPI/contr1/1234,60,M(capiect))
</PRE>

# Short HOWTO of capiCommand(receivefax|<filename>[|<stationid>|<headline>]):

For those of you who have a CAPI card with an on-board DSP (like some Eicon and
DIVA Server), this patch allows you to receive faxes.
If you want to answer a channel in fax mode, use capicommand(receivefax|...)
instead of Answer().
If you use Answer(), you will be in voice mode. If the hardware DSP detects 
fax tone, you can switch from voice to fax mode by calling capicommand(receivefax|...).
The parameter <filename> is mandatory and the parameters <stationid> and
<headline> are optional.

Example of use:
<PRE>
line number 123, play something, if a fax tone is detected, handle it
line number 124, answer directly in fax mode

[incoming]
exten => 123,1,Answer()
exten => 123,2,BackGround(jpop)
exten => 124,1,Goto(handle_fax,s,1)
exten => fax,1,Goto(handle_fax,s,1)

[handle_fax]
exten => s,1,capicommand(receivefax|/tmp/${UNIQUEID}[|<stationid>|<headline>])
exten => s,2,Hangup()
exten => h,1,deadagi,fax.php // Run sfftobmp and mail it.
</PRE>

The output of capicommand(receivefax|...) is a SFF file.
Use sfftobmp to convert it.
With a DIVA Server, following features are provided:
 - fax up to 33600
 - high resolution
 - Color Fax 
 - JPEG Compression is disabled (not tested yet)

After successful receive of a fax, the following variables will be set for that channel:
FAXRATE       : The baud rate of the fax connection
FAXRESOLUTION : 0 = standard, 1 = high
FAXFORMAT     : 0 = SFF
FAXPAGES      : Number of pages received
FAXID         : The ID of the remote fax maschine



# PBX VARIABLES USED/SET BY CHAN_CAPI

BCHANNELINFO
    On incomming call, this variable is set with the B-channel information value:
     '0' : B-channel is used (default)
     '1' : D-channel is used (not implemented yet)
     '2' : neither B nor D channel is used (e.g. call waiting)
    Call-Waiting: an incoming call with BCHANNELINFO not '0' cannot be accepted.
    Another connection must be dropped before accepting or use
    capicommand(deflect|<number>) to initiate call deflection to another destination.

CALLEDTON
    The 'type of number' value of the called number is saved in this variable on
    incomming call.

_CALLERHOLDID
    If a call is put on hold (ISDN-HOLD), the reference id is saved in this variable.
    This variable is inherited as CALLERHOLDID to the dialed channel and will be used
    if e.g. capicommand(ect) is used to transfer the held call.

CALLINGSUBADDRESS
    If set on dial(), the calling subaddress will be set to the content.

CALLEDSUBADDRESS
    If set on dial(), the called subaddress will be set to the content.
 
CONNECTEDNUMBER
    Can be set before answering and if set, the content is used for
    IE 'Connected Number' on answering.

PEER_IS_ANALOG
    Is set for all incoming calls if the peer is analog.

FAXEXTEN
    If chan_capi sends the call to extensions 'fax', the original extension number
    is saved in this variable.

PRI_CAUSE
    If set, this value will be used as hangup cause on hangup.

REDIRECTINGNUMBER
    On incoming call, if the call was redirected to you by someone, the
    number of the redirecting party is saved in this variable.
    RDNIS is set as well.

REDIRECTREASON
    If the incoming call was redirected to you, this variable is set
    with the reason value.
 
# Functions (available with newer Asterisk only)

VANITYNUMBER(<vanitynumber>)
    Converts the 'vanitynumber' into a digit-only string. International keypad is
    used, e.g. ABC=1, DEF=2, ...

# History

<PRE>
chan-capi (junghans.net)
   |
   |
   |
   v
chan-capi-cm ---> chan-capi-hps
   |                 |
   v                 v
</PRE>

Chan-capi-hps is a complete rewrite of Chan-capi-cm, which author is
Armin Schindler. Chan-capi-hps includes several enhancements for
example the ability reload the "capi.conf" configuration file on the
fly using the "capi reload" command. Chan-capi-cm can be obtained from
<A HREF="http://www.chan-capi.org">http://www.chan-capi.org</A>. Bugs
can be reported to <A HREF="mailto:hps&#x40;selasky.org">HPS</A>,
<A HREF="mailto:freebsd-isdn&#x40;freebsd.org">freebsd-isdn&#x40;freebsd.org</A>
or <A HREF="http://lists.melware.net/mailman/subscribe/chan-capi-users">http://lists.melware.net/mailman/listinfo/chan-capi-users</A>.

--HPS
