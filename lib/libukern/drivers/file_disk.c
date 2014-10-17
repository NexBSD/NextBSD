/*-
 * Copyright (c) 2000 Michael Smith
 * Copyright (c) 2001 Scott Long
 * Copyright (c) 2000 BSDi
 * Copyright (c) 2001 Adaptec, Inc.
 * All rights reserved.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_aac.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/disk.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/md_var.h>
#include <machine/bus.h>
#include <sys/rman.h>


#include <sys/bio.h>
#include <sys/callout.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/selinfo.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <geom/geom_disk.h>


/*
 * Per-controller structure.
 */
struct vd_softc
{
        /* bus connections */
        device_t                vd_dev;

	    /* command/fib resources */
        TAILQ_HEAD(,vd_fibmap) vd_fibmap_tqh;
        u_int                   total_fibs;
        struct vd_command      *vd_commands;

        /* command management */
        TAILQ_HEAD(,vd_command) vd_free;      /* command structures
                                                 * available for reuse */
        TAILQ_HEAD(,vd_command) vd_ready;     /* commands on hold for
                                                 * controller resources */
        TAILQ_HEAD(,vd_command) vd_busy;
        TAILQ_HEAD(,vd_event)  vd_ev_cmfree;
        struct bio_queue_head   vd_bioq;
        struct vd_queue_table  *vd_queues;
        struct vd_queue_entry  *vd_qentries[VD_QUEUE_COUNT];

        struct vd_qstat        vd_qstat[AACQ_COUNT];  /* queue statistics */

        /* connected containters */
        TAILQ_HEAD(,vd_container)      vd_container_tqh;
        struct mtx              vd_container_lock;

        /*
         * The general I/O lock.  This protects the sync fib, the lists, the
         * queues, and the registers.
         */
        struct mtx              vd_io_lock;

        /* delayed activity infrastructure */
        struct task             vd_task_complete;      /* deferred-completion
                                                         * task */
        struct intr_config_hook vd_ich;

        /* management interface */
        struct mtx              vd_aifq_lock;
        struct vd_fib          vd_aifq[VD_AIFQ_LENGTH];
        int                     aifq_idx;
        int                     aifq_filled;
        struct vd_fib_context *fibctx;
        struct selinfo          rcv_select;
        struct proc             *aifthread;

    	u_int32_t               supported_options;
        u_int32_t               scsi_method_id;
        TAILQ_HEAD(,vd_sim)    vd_sim_tqh;

        struct callout  vd_daemontime;         /* clock daemon callout */

        u_int32_t       vd_sg_tablesize;               /* max. sg count from host */
        u_int32_t       vd_max_sectors;                /* max. I/O size from host (blocks) */
};

/*
 * Per-disk structure
 */
struct vdisk
{
	device_t                        vd_dev;
	struct vd_container            *vd_container;
	struct disk                     *vd_disk;
	int                             vd_flags;
#define VD_DISK_OPEN   (1<<0)
	int                             vd_cylinders;
	int                             vd_heads;
	int                             vd_sectors;
	u_int64_t                       vd_size;
	int                             unit;
};

/*
 * Interface to parent.
 */
static void vdisk_identify(driver_t *driver, device_t parent);
static int vdisk_probe(device_t dev);
static int vdisk_attach(device_t dev);
static int vdisk_detach(device_t dev);

/*
 * Interface to the device switch.
 */
static	disk_open_t	vdisk_open;
static	disk_close_t	vdisk_close;
static	disk_strategy_t	vdisk_strategy;

static devclass_t	vdisk_devclass;

static device_method_t vdisk_methods[] = {
	DEVMETHOD(device_identify,	vdisk_identify),
	DEVMETHOD(device_probe,	vdisk_probe),	
	DEVMETHOD(device_attach,	vdisk_attach),
	DEVMETHOD(device_detach,	vdisk_detach),
	DEVMETHOD_END
};

static driver_t vdisk_driver = {
	"vdisk",
	vdisk_methods,
	sizeof(struct vdisk)
};

DRIVER_MODULE(aacd, aac, vdisk_driver, vdisk_devclass, NULL, NULL);

/*
 * Handle open from generic layer.
 *
 * This is called by the diskslice code on first open in order to get the
 * basic device geometry paramters.
 */
static int
vdisk_open(struct disk *dp)
{
	struct vdisk	*sc;

	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	sc = (struct vdisk *)dp->d_drv1;

	if (sc == NULL) {
		printf("vdisk_open: No Softc\n");
		return (ENXIO);
	}

	/* check that the controller is up and running */
	if (sc->vd_controller->vd_state & VD_STATE_SUSPEND) {
		device_printf(sc->vd_controller->vd_dev,
		    "Controller Suspended controller state = 0x%x\n",
		    sc->vd_controller->vd_state);
		return(ENXIO);
	}

	sc->vd_flags |= VDISK_OPEN;
	return (0);
}

/*
 * Handle last close of the disk device.
 */
static int
vdisk_close(struct disk *dp)
{
	struct vdisk	*sc;

	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	sc = (struct vdisk *)dp->d_drv1;

	if (sc == NULL)
		return (ENXIO);

	sc->vd_flags &= ~VDISK_OPEN;
	return (0);
}

/*
 * Handle an I/O request.
 */
static void
vdisk_strategy(struct bio *bp)
{
	struct vdisk	*sc;

	sc = (struct vdisk *)bp->bio_disk->d_drv1;
	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	/* bogus disk? */
	if (sc == NULL) {
		bp->bio_flags |= BIO_ERROR;
		bp->bio_error = EINVAL;
		biodone(bp);
		return;
	}

	/* do-nothing operation? */
	if (bp->bio_bcount == 0) {
		bp->bio_resid = bp->bio_bcount;
		biodone(bp);
		return;
	}

	/* perform accounting */

	/* pass the bio to the controller - it can work out who we are */
	mtx_lock(&sc->lock);
	vdisk_submit_bio(bp);
	mtx_unlock(&sc->lock);
}

/*
 * Handle completion of an I/O request.
 */
void
vd_biodone(struct bio *bp)
{
	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	if (bp->bio_flags & BIO_ERROR) {
		bp->bio_resid = bp->bio_bcount;
		disk_err(bp, "hard error", -1, 1);
	}

	biodone(bp);
}

static void
vdisk_identify(driver_t *driver, device_t parent)
{

	BUS_ADD_CHILD(parent, 0, driver->name, 0);
}

/*
 * Stub only.
 */
static int
vdisk_probe(device_t dev)
{

	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	return (0);
}

/*
 * Attach a unit to the controller.
 */
static int
vdisk_attach(device_t dev)
{
	struct vdisk	*sc;
	
	sc = (struct vdisk *)device_get_softc(dev);
	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	/* initialise our softc */
	sc->vd_container = device_get_ivars(dev);
	sc->vd_dev = dev;

	/*
	 * require that extended translation be enabled - other drivers read the
	 * disk!
	 */
	sc->vd_size = sc->vd_container->co_mntobj.Capacity;
	if (sc->vd_controller->flags & VD_FLAGS_LBA_64BIT)
		sc->vd_size += (u_int64_t)
			sc->vd_container->co_mntobj.CapacityHigh << 32;
	if (sc->vd_size >= (2 * 1024 * 1024)) {		/* 2GB */
		sc->vd_heads = 255;
		sc->vd_sectors = 63;
	} else if (sc->vd_size >= (1 * 1024 * 1024)) {	/* 1GB */
		sc->vd_heads = 128;
		sc->vd_sectors = 32;
	} else {
		sc->vd_heads = 64;
		sc->vd_sectors = 32;
	}
	sc->vd_cylinders = (sc->vd_size / (sc->vd_heads * sc->vd_sectors));

	device_printf(dev, "%juMB (%ju sectors)\n",
		      (intmax_t)sc->vd_size / ((1024 * 1024) / VD_BLOCK_SIZE),
		      (intmax_t)sc->vd_size);

	/* attach a generic disk device to ourselves */
	sc->unit = device_get_unit(dev);
	sc->vd_disk = disk_alloc();
	sc->vd_disk->d_drv1 = sc;
	sc->vd_disk->d_flags = 0 /* DISKFLAG_UNMAPPED_BIO - just writev*/;
	sc->vd_disk->d_name = "vdisk";
	sc->vd_disk->d_maxsize = sc->vd_controller->vd_max_sectors << 9;
	sc->vd_disk->d_open = vdisk_open;
	sc->vd_disk->d_close = vdisk_close;
	sc->vd_disk->d_strategy = vdisk_strategy;
	sc->vd_disk->d_sectorsize = VD_BLOCK_SIZE;
	sc->vd_disk->d_mediasize = (off_t)sc->vd_size * VD_BLOCK_SIZE;
	sc->vd_disk->d_fwsectors = sc->vd_sectors;
	sc->vd_disk->d_fwheads = sc->vd_heads;
	sc->vd_disk->d_unit = sc->unit;
	disk_create(sc->vd_disk, DISK_VERSION);

	return (0);
}

/*
 * Disconnect ourselves from the system.
 */
static int
vdisk_detach(device_t dev)
{
	struct vdisk *sc;

	sc = (struct vdisk *)device_get_softc(dev);
	fwprintf(NULL, HBA_FLAGS_DBG_FUNCTION_ENTRY_B, "");

	if (sc->vd_flags & VDISK_OPEN)
		return(EBUSY);

	disk_destroy(sc->vd_disk);

	return(0);
}
