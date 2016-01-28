#
# $FreeBSD$
#

.include <src.opts.mk>

LIB=			ndmp
SHLIBDIR?=		/lib
SHLIB_MAJOR=	1
#MAN=
.PATH:          ${.CURDIR}/../../contrib/ndmp/libndmp/common

CFLAGS+= -DNEED_SOLARIS_BOOLEAN -g -O0
#CFLAGS+=	-I/usr/local/include 
CFLAGS+= -I${.CURDIR}/../../contrib/ndmp/libndmp/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libnvpair
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common
CFLAGS+= -I${.CURDIR}/../../sys/cddl/compat/opensolaris
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/head
CFLAGS+= -I${.CURDIR}/../../contrib/ndmp/ndmpd/include

CFLAGS+= -Wno-unknown-pragmas
SRCS+= libndmp_door_data.c
SRCS+= libndmp.c
SRCS+= libndmp_base64.c
SRCS+= libndmp_error.c
SRCS+= libndmp_prop.c
WARNS=0
.include <bsd.lib.mk>
