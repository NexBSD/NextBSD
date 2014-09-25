#
# $FreeBSD$
#

.include <src.opts.mk>

LIB=			ndmp
SHLIBDIR?=		/lib
SHLIB_MAJOR=	1
#MAN=
.PATH:          ${.CURDIR}/../../contrib/ndmp/libndmp/common

CFLAGS+= -DNEED_SOLARIS_BOOLEAN
CFLAGS+=	-I/usr/local/include 
CFLAGS+= -I../../contrib/ndmp/libndmp/common
CFLAGS+= -I../../cddl/contrib/opensolaris/lib/libnvpair
CFLAGS+= -I../../sys/cddl/contrib/opensolaris/uts/common
CFLAGS+= -I../../sys/cddl/compat/opensolaris
CFLAGS+= -I../../cddl/contrib/opensolaris/head
CFLAGS+= -I../../contrib/ndmp/ndmpd/include

CFLAGS+= -Wno-unknown-pragmas
SRCS+= libndmp_door_data.c
SRCS+= libndmp.c
SRCS+= libndmp_base64.c
SRCS+= libndmp_error.c
#SRCS+= libndmp_prop.c
WARNS=0
.include <bsd.lib.mk>
