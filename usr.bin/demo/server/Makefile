.include <src.opts.mk>
PROG= demo-server
SRCS= server.c
BINDIR?= /usr/bin

MK_MAN=no
CFLAGS= -I${.CURDIR}/../../../include/apple -fblocks
LDADD+= -lSystem

.include <bsd.prog.mk>
