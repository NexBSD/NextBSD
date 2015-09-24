# $FreeBSD$

.include <src.opts.mk>

NDMPADM_DIR=${.CURDIR}/../../contrib/ndmp/ndmpadm

.PATH: ${NDMPADM_DIR}
BINDIR= /usr/sbind
PROG= ndmpadm

SRCS=	ndmpadm_main.c ndmpadm_print.c

BINOWN= root

CFLAGS+= -DNEED_SOLARIS_BOOLEAN -g
CFLAGS+= -I/usr/local/include
CFLAGS+= -I${.CURDIR}/../../contrib/ndmp/libndmp/common
CFLAGS+= -I${.CURDIR}/../../cddl/compat/opensolaris/include
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libnvpair
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common
CFLAGS+= -I${.CURDIR}/../../sys/cddl/compat/opensolaris
CFLAGS+= -L${.CURDIR}/../../lib/libndmp
CFLAGS+= -L/usr/local/lib

WARNS?= 1

LDADD= -lndmp -ljansson -lintl -lpthread

.include <bsd.prog.mk>

