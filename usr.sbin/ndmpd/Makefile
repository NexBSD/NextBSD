#	@(#)Makefile	8.8 (Berkeley) 3/28/97
# $FreeBSD$

.include <src.opts.mk>

NDMPD_DIR=${.CURDIR}/../../contrib/ndmp/ndmpd
NDMP_DIR=${NDMPD_DIR}/ndmp
TLM_DIR=${NDMPD_DIR}/tlm
SOL_LIB=${.CURDIR}/../../cddl/contrib/opensolaris/lib
SOL_UTS=${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common
SOL_SYS_COMPAT=${.CURDIR}/../../sys/cddl/compat/opensolaris
SOL_COMPAT=${.CURDIR}/../../cddl/compat/opensolaris/include
SOL_ZFS=${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common/fs/zfs
SMDIR=${NDMPD_DIR}/ndmp
.PATH: ${SMDIR}


RPCGEN=/usr/bin/rpcgen
.include "${NDMP_DIR}/Makefile.rpcgen"

BINDIR=	/usr/sbin

PROG=	ndmpd
#MAN=	ndmpd.1m
#MLINKS+=sendmail.8 hoststat.8

# ndmpd_doors.c ndmpd_tape.c ndmpd_scsi.c
SRCS= ndmpd_callbacks.c ndmpd_chkpnt.c ndmpd_comm.c ndmpd_config.c \
	ndmpd_connect.c ndmpd_data.c ndmpd_dtime.c ndmpd_fhistory.c \
	ndmpd_handler.c ndmpd_log.c ndmpd_main.c ndmpd_mark.c ndmpd_mover.c \
	ndmpd_prop.c ndmpd_tar.c ndmpd_tar3.c \
	ndmpd_util.c ndmpd_zfs.c ndmp_xdr.c

.PATH:${TLM_DIR}
# tlm_init.c 
SRCS+= tlm_backup_reader.c tlm_bitmap.c tlm_buffers.c tlm_info.c \
	tlm_lib.c tlm_restore_writer.c tlm_traverse.c tlm_util.c tlm_hardlink.c

BINOWN=	root
#BINGRP=

CFLAGS+= -DNEED_SOLARIS_BOOLEAN -g

CFLAGS+= -I${.CURDIR}/../../sys/cddl/compat/opensolaris
CFLAGS+= -I${.CURDIR}/../../cddl/compat/opensolaris/include
CFLAGS+= -I${.CURDIR}/../../cddl/compat/opensolaris/lib/libumem
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libnvpair
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libuutil/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libinetutil/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libbsm/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libzfs/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libzfs_core/common
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/lib/libzpool/common
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common/fs/zfs
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/uts/common/sys
CFLAGS+= -I${.CURDIR}/../../sys/cddl/contrib/opensolaris/common/zfs
CFLAGS+= -I${.CURDIR}/../../cddl/contrib/opensolaris/head

CFLAGS+= -I${.CURDIR}/../../sys/
CFLAGS+= -I${SOL_SYS_COMPAT} -I${SOL_COMPAT}
CFLAGS+= -I${SMDIR} -I${NDMPD_DIR}/include -I.
CFLAGS+= -I${SOL_LIB}/libnvpair -I${SOL_LIB}/libzfs/common
CFLAGS+= -I${SOL_UTS} -I${SOL_ZFS}
CFLAGS+= -I${SMDIR}/../../libndmp/common
CFLAGS+= -I/usr/local/include
CFLAGS+= -L${.CURDIR}/../../lib/libndmp -L/usr/local/lib

.if ${MK_INET6_SUPPORT} != "no"
CFLAGS+= -DNETINET6
.endif

WARNS?=	1

LDADD= -lnvpair -luutil -lgeom -lbsm -lzfs -lzpool -lndmp -ljansson

.include <bsd.prog.mk>
