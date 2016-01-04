.include <src.opts.mk>
PROG= set-bport
SRCS= set-bport.c
BINDIR?= /usr/bin

MK_MAN=no
CFLAGS= -I${.CURDIR}/../../../include/apple
LDFLAGS+= -lmach -pthread

.include <bsd.prog.mk>
