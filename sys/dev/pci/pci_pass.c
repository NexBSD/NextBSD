/*-
 * Copyright (c) 2014, Matthew Macy <kmacy@FreeBSD.ORG>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	$FreeBSD$
 *
 */

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/interrupt.h>
#include <sys/libkern.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rwlock.h>
#include <sys/pci_pass.h>
#include <sys/proc.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sleepqueue.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysent.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pci_pass.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_param.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>
#include <vm/vm_extern.h>

#include <machine/intr_machdep.h>
#include <machine/smp.h>
#include <machine/cpu.h>
#include <machine/resource.h>
#include <x86/frame.h>

#define PPDC_DOOMED  0x7
#define FIRST_TIMER_VECTOR 512


static int	unit;
extern int sztrapcode;
extern caddr_t trapcode;
struct dev_pass_softc;
struct pci_pass_driver_context;

static void _dev_pass_softc_free(struct dev_pass_softc *dp);
static struct dev_pass_softc *_dev_pass_softc_find(pid_t pid);
static struct thread_link *_tl_find(struct thread *td, int alloc);

static void _unmap_uva(void *kva);
static void *_map_uva(caddr_t uva);
static void _setup_trapframe(struct thread *td, struct pci_pass_driver_context *ctx);

static void _thread_exit_cleanup(void *arg, struct thread *td);


static int	pci_pass_probe(device_t dev);
static int	pci_pass_attach(device_t dev);
static device_method_t pci_pass_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		pci_pass_probe),
    DEVMETHOD(device_attach,	pci_pass_attach),
    { 0, 0 }
};

/*
 * XXX handle thread exit
 *  register EVENTHANDLER for thread_dtor to clear td pointer
 */
struct pci_pass_driver_context;

struct pci_pass_softc {
	device_t pp_pci_dev;
	struct cdev *pp_pci_cdev;
	pid_t pp_pid;
	struct mtx pp_ctx_mtx;
	LIST_HEAD(, pci_pass_driver_context) pp_ctx_list;
};

struct thread_link {
	struct thread *tl_td;
	struct dev_pass_tdvcpumap *tl_dpt;
	LIST_ENTRY(thread_link) link;
	LIST_HEAD(,pci_pass_driver_context) tl_ctx_list;
};

LIST_HEAD(, dev_pass_softc) dp_softc_list;
LIST_HEAD(, thread_link) thread_ctx_list;
struct mtx sclist_mtx;

struct dev_pass_tdvcpumap {
	uint8_t	dpt_vcpuid;
	uint8_t	dpt_cpuid;
	lwpid_t dpt_tid;
	caddr_t dpt_kstack;
	vm_offset_t dpt_ustack;
	struct thread_link *dpt_tl;
	struct thread *dpt_td;
	struct callout dpt_c;
};

struct dev_pass_softc {
	struct pass_status_page *dp_status_page;
	caddr_t dp_code_page_kern;
	vm_offset_t dp_code_page;
	struct dev_pass_tdvcpumap *dp_vcpumap;
	vm_offset_t dp_trap;
	int dp_nvcpus;
	int dp_ticks;
	pid_t dp_pid;
	volatile int dp_refcnt;
	LIST_ENTRY(dev_pass_softc) link;
};

struct pci_pass_driver_context {
	device_t ppdc_dev;
	struct resource *ppdc_irq;
	void *ppdc_tag;
	struct pci_pass_softc *ppdc_parent;
	int ppdc_vector;
	int ppdc_vcpuid;
	struct dev_pass_softc *ppdc_dp;
	vm_offset_t ppdc_trap;
	void (*ppdc_post_filter)(void *);
	int ppdc_flags;
	LIST_ENTRY(pci_pass_driver_context) pp_link;
	LIST_ENTRY(pci_pass_driver_context) tl_link;
};

static driver_t pci_pass_driver = {
    "pci_pass",
    pci_pass_methods,
    sizeof(struct pci_pass_softc),
};

static devclass_t pci_pass_devclass;

DRIVER_MODULE_ORDERED(pci_pass, pci, pci_pass_driver, pci_pass_devclass, 0, 0,
	SI_ORDER_MIDDLE);

static d_close_t pci_pass_close;
static d_open_t pci_pass_open;
static d_ioctl_t pci_pass_ioctl;
static d_mmap_single_t pci_pass_mmap_single;

static struct cdevsw pci_pass_cdevsw = {
       .d_version =    D_VERSION,
       .d_flags =      0,
       .d_open =       pci_pass_open,
       .d_close =      pci_pass_close,
	   .d_ioctl =      pci_pass_ioctl,
       .d_mmap_single =      pci_pass_mmap_single,
       .d_name =       "pcipass",
};

static int
pci_pass_probe(device_t dev)
{
	char devstr[24];
	char filter_list[128], *filterstr;
	char *curdevid;

	snprintf(devstr, sizeof(devstr), "%02x:%02x:%02x:%02x", pci_get_domain(dev),
			 pci_get_bus(dev),pci_get_slot(dev), pci_get_function(dev));

	if (getenv_string("device_filter_list", filter_list, sizeof(filter_list)) == 0)
		return (ENXIO);

	filterstr = filter_list;
	while ((curdevid = strsep(&filterstr, ",")) != NULL)
		if (strncmp(devstr, curdevid, sizeof(devstr)) == 0)
			return (BUS_PROBE_SPECIFIC);

    return(ENXIO);
}

static int
pci_pass_attach(device_t dev)
{
	struct pci_pass_softc *sc = device_get_softc(dev);
	sc->pp_pci_cdev =
		make_dev(&pci_pass_cdevsw, unit, UID_ROOT, GID_WHEEL, 0660,
				 "pci%02x:%02x:%02x:%02x", pci_get_domain(dev),
				 pci_get_bus(dev),pci_get_slot(dev), pci_get_function(dev));
	if (sc->pp_pci_cdev == NULL)
		return (ENOMEM);
	sc->pp_pci_dev = dev;
	sc->pp_pid = 0;
	LIST_INIT(&sc->pp_ctx_list);
	mtx_init(&sc->pp_ctx_mtx, "pci_pass", NULL, MTX_DEF);
	unit++;
	device_set_desc(dev, "pcipass");
	sc->pp_pci_cdev->si_drv1 = (void *)sc;
	return (0);
}

static int
pci_pass_open(struct cdev *dev, int flags, int fmp, struct thread *td)
{
	struct pci_pass_softc *sc = dev->si_drv1;

	printf("pci_pass_open(...) called\n");
	if (sc->pp_pid != 0)
		return (EBUSY);
	sc->pp_pid = td->td_proc->p_pid;
	return (0);
}

static int
pci_pass_close(struct cdev *cdev, int flags, int fmt, struct thread *td)
{
	struct pci_pass_softc *sc = cdev->si_drv1;
	struct pci_pass_driver_context *ctx;

	printf("pci_pass_close(...) called\n");
	sc->pp_pid = 0;
	if (LIST_EMPTY(&sc->pp_ctx_list))
		return (0);
	mtx_lock(&sc->pp_ctx_mtx);
	while (!LIST_EMPTY(&sc->pp_ctx_list)) {
		ctx = LIST_FIRST(&sc->pp_ctx_list);
		LIST_REMOVE(ctx, pp_link);
		mtx_unlock(&sc->pp_ctx_mtx);
		mtx_lock(&sclist_mtx);
		LIST_REMOVE(ctx, tl_link);
		mtx_unlock(&sclist_mtx);
		bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		bus_release_resource(ctx->ppdc_dev, SYS_RES_IRQ, ctx->ppdc_vector,
			ctx->ppdc_irq);
		_dev_pass_softc_free(ctx->ppdc_dp);
		free(ctx, M_DEVBUF);
		mtx_lock(&sc->pp_ctx_mtx);
	}
	mtx_unlock(&sclist_mtx);
	return (0);
}

struct pci_pass_handle {
	struct cdev *cdev;
	size_t		size;
	vm_paddr_t	paddr;
};

static int
pci_pass_dev_pager_ctor(void *handle, vm_ooffset_t size, vm_prot_t prot,
    vm_ooffset_t foff, struct ucred *cred, u_short *color)
{
	struct pci_pass_handle *pph = handle;

	dev_ref(pph->cdev);
	return (0);
}

static void
pci_pass_dev_pager_dtor(void *handle)
{
	struct pci_pass_handle *pph = handle;
	struct cdev *cdev = pph->cdev;

	free(pph, M_DEVBUF);
	dev_rel(cdev);
}

static int
pci_pass_dev_pager_fault(vm_object_t object, vm_ooffset_t offset,
	int prot, vm_page_t *mres)
{
	struct pci_pass_handle *pph = object->handle;
	vm_paddr_t paddr;
	vm_page_t page;
	vm_memattr_t memattr;
	vm_pindex_t pidx;

	printf("pci_pass_dev_pager_fault: object %p offset %jx prot %d mres %p",
			object, (intmax_t)offset, prot, mres);

	if (offset < pph->paddr || offset > pph->paddr + pph->size)
		return (VM_PAGER_FAIL);

	memattr = object->memattr;
	paddr = offset;
	pidx = OFF_TO_IDX(offset);

	if (((*mres)->flags & PG_FICTITIOUS) != 0) {
		/*
		 * If the passed in result page is a fake page, update it with
		 * the new physical address.
		 */
		page = *mres;
		vm_page_updatefake(page, paddr, memattr);
	} else {
		/*
		 * Replace the passed in reqpage page with our own fake page and
		 * free up the all of the original pages.
		 */
		VM_OBJECT_WUNLOCK(object);
		page = vm_page_getfake(paddr, memattr);
		VM_OBJECT_WLOCK(object);
		vm_page_lock(*mres);
		vm_page_free(*mres);
		vm_page_unlock(*mres);
		*mres = page;
		vm_page_insert(page, object, pidx);
	}
	page->valid = VM_PAGE_BITS_ALL;
	return (VM_PAGER_OK);
}


static struct cdev_pager_ops pci_pass_cdev_pager_ops = {
	.cdev_pg_ctor = pci_pass_dev_pager_ctor,
	.cdev_pg_dtor = pci_pass_dev_pager_dtor,
	.cdev_pg_fault = pci_pass_dev_pager_fault,
};

#define IDX_SHIFT 6
#define IDX_MASK  (64-1)

static int
_intr_masked(struct pass_status_page *si, int vector)
{
	int arridx = vector >> IDX_SHIFT;
	int bitidx = vector & IDX_MASK;
	int orpendval = 1 << bitidx;

	return (si->evtchn_mask[arridx] & orpendval);
}

static int
_set_intr_pending(struct pass_status_page *si,
				  struct pass_vcpu_info *vi, int vector)
{
	int arridx = vector >> IDX_SHIFT;
	int bitidx = vector & IDX_MASK;
	int orpendval = 1 << bitidx;
	int orselval = 1 << arridx;
	int masked;

	atomic_set_long(&si->evtchn_pending[arridx], orpendval);
	atomic_set_long(&vi->evtchn_pending_sel, orselval);
	vi->evtchn_upcall_pending = 1;
	masked = _intr_masked(si, vector);
	if (masked == 0)
		masked = atomic_swap_char(&vi->evtchn_upcall_mask, 1);
	return (masked);
}

/*
 * The force of a signal has been directed against a single
 * thread.  We need to see what we can do about knocking it
 * out of any sleep it may be in etc.
 */
static void
_intr_tdsigwakeup(struct thread *td, int intrval)
{
	struct proc *p = td->td_proc;

	PROC_SLOCK(p);
	thread_lock(td);
	if (TD_ON_SLEEPQ(td)) {
		/*
		 * If thread is sleeping uninterruptibly
		 * we can't interrupt the sleep... the signal will
		 * be noticed when the process returns through
		 * trap() or syscall().
		 */
		if ((td->td_flags & TDF_SINTR) == 0)
			goto out;

		/*
		 * Give low priority threads a better chance to run.
		 */
		if (td->td_priority > PUSER)
			sched_prio(td, PUSER);

		sleepq_abort(td, intrval);
	}
out:
	PROC_SUNLOCK(p);
	thread_unlock(td);
}

static int
_pci_pass_driver_filter(void *arg)
{
	struct pci_pass_driver_context *ctx = arg;
	struct thread *td;
	struct pass_status_page *si;
	struct pass_vcpu_info *vi;
	int masked, needwakeup = 0;

	if (ctx->ppdc_flags & PPDC_DOOMED)
		return (FILTER_STRAY);

	td = ctx->ppdc_dp->dp_vcpumap[ctx->ppdc_vcpuid].dpt_td;
	/* our thread has exited */
	if (td == NULL)
		return (FILTER_STRAY);

	si = ctx->ppdc_dp->dp_status_page;
	vi = &si->vcpu_info[ctx->ppdc_vcpuid];
	masked = _set_intr_pending(si, vi, ctx->ppdc_vector);
	if (masked)
		return (FILTER_HANDLED);

	thread_lock(td);
	if ((td == curthread && TD_IS_RUNNING(td)) || TD_ON_RUNQ(td)) {
		/* fast path */
		_setup_trapframe(td, ctx);
	} else if ((td != curthread) && TD_IS_RUNNING(td)){
		/* running on another cpu - send upcall */
		td->td_flags |= (TDF_ASTPENDING|TDF_CALLBACK);
		ipi_cpu(td->td_oncpu, IPI_AST);
	} else {
		td->td_flags |= (TDF_ASTPENDING|TDF_CALLBACK);
		needwakeup = 1;
	}
	thread_unlock(td);
	if (needwakeup)
		_intr_tdsigwakeup(td, EINTR);
	return (FILTER_HANDLED);
}

static void
_vcpu_timer(void *arg)
{
	struct pci_pass_driver_context *ctx = arg;
	int cpuid;
	struct dev_pass_tdvcpumap *dpt;

	(void)_pci_pass_driver_filter(ctx);
	cpuid = ctx->ppdc_vcpuid;
	dpt = &ctx->ppdc_dp->dp_vcpumap[cpuid];
	callout_reset_curcpu(&dpt->dpt_c, ctx->ppdc_dp->dp_ticks, _vcpu_timer, arg);
}

static int
pci_pass_mmap_single(struct cdev *cdev, vm_ooffset_t *offset,
		   vm_size_t size, struct vm_object **objp, int nprot)
{
	struct vm_object *obj;
	struct pci_pass_handle *pph;
	
	printf("pci_pass_mmap_single(%p, %p=%lx,%ld, %p, %d)\n",
		   cdev, offset, *offset, size, objp, nprot);

	pph = malloc(sizeof(struct pci_pass_handle), M_DEVBUF, M_NOWAIT|M_ZERO);
	if (pph == NULL)
		return (ENOMEM);
	pph->cdev = cdev;
	pph->size = size;
	/* XXX validate that this address actually belongs to this device */
	pph->paddr = (vm_paddr_t)*offset;
	obj = cdev_pager_allocate(pph, OBJT_DEVICE, &pci_pass_cdev_pager_ops, size,
							  nprot, *offset, NULL);
	if (obj == NULL) {
		free(pph, M_DEVBUF);
		return (ENXIO);
	}
	*objp = obj;
	return (0);
}

static struct pci_pass_driver_context *
_ctx_alloc(device_t dev, struct resource *irq,
		   struct pci_pass_softc *sc, int vector, int vcpuid,
		   struct dev_pass_softc *dp, void *trap, struct thread *td)
{
	struct pci_pass_driver_context *ctx;
	struct thread_link *tl;

	ctx = malloc(sizeof(*ctx), M_DEVBUF, M_WAITOK|M_ZERO);
	ctx->ppdc_dev = dev;
	ctx->ppdc_irq = irq;
	ctx->ppdc_parent = sc;
	ctx->ppdc_vector = vector;
	ctx->ppdc_vcpuid = vcpuid;
	ctx->ppdc_dp = dp;
	ctx->ppdc_trap = (vm_offset_t)trap;
	if (sc) {
		mtx_lock(&sc->pp_ctx_mtx);
		LIST_INSERT_HEAD(&sc->pp_ctx_list, ctx, pp_link);
		mtx_unlock(&sc->pp_ctx_mtx);
	}
	if (td) {
		if ((tl = _tl_find(td, FALSE)) == NULL)
			panic("inconsistent thread state");
		mtx_lock(&sclist_mtx);
		/* keep the timer interrupt first */
		LIST_INSERT_AFTER(LIST_FIRST(&tl->tl_ctx_list), ctx, tl_link);
		mtx_unlock(&sclist_mtx);
	}
	return (ctx);
}

static void
_vcpu_unmap(struct dev_pass_softc *dp)
{
	int i;
	struct dev_pass_tdvcpumap *dpt = dp->dp_vcpumap;

	for (i = 0; i < dp->dp_nvcpus; i++, dpt++)
		_thread_exit_cleanup(NULL, dpt->dpt_td);

	free(dp->dp_vcpumap, M_DEVBUF);
	dp->dp_vcpumap = NULL;
	dp->dp_nvcpus = 0;
}

static int
pci_pass_ioctl(struct cdev *cdev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	struct pci_pass_softc *sc = cdev->si_drv1;
	device_t dev = sc->pp_pci_dev;
	struct pci_pass_driver_context *ctx;
	struct intsrc *isrc;

	printf("pci_pass_ioctl(...) called\n");

	switch (cmd) {
	case PCIPASSIOCSETUPINTR: {
		struct pci_pass_setup_intr *ppsi = (struct pci_pass_setup_intr *)data;
		struct resource *r;
		struct thread *itd;
		struct thread_link *tl;
		struct dev_pass_softc *dpsc;
		lwpid_t tid;
		int cpuid, rc, rid = ppsi->ppsi_vector;

		if ((dpsc = _dev_pass_softc_find(td->td_proc->p_pid)) == NULL ||
			dpsc->dp_status_page == NULL)
			return (ENXIO);

		tid = dpsc->dp_vcpumap[ppsi->ppsi_vcpuid].dpt_tid;
		if ((itd = tdfind(tid, curproc->p_pid)) == NULL) {
			rc = EINVAL;
			goto fail;
		}
		PROC_UNLOCK(curproc);
		tl = _tl_find(itd, TRUE);
		if ((dpsc->dp_nvcpus == 0) || (dpsc->dp_vcpumap == NULL) ||
			(ppsi->ppsi_vcpuid > dpsc->dp_nvcpus + 1)) {
			rc = EINVAL;
			goto fail;
		}

		if ((r = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE)) == NULL) {
			rc = ENOMEM;
			goto fail;
		}
		ctx = _ctx_alloc(dev, r, sc, rid, ppsi->ppsi_vcpuid, dpsc, ppsi->ppsi_trap, itd);
		if ((rc = bus_setup_intr(dev, r, INTR_TYPE_NET, _pci_pass_driver_filter,
								 NULL, ctx, &ctx->ppdc_tag)) != 0) {
			free(ctx, M_DEVBUF);
			bus_release_resource(dev, SYS_RES_IRQ, rid, r);
			goto fail;
		}

		/* we need the APIC disabled until the user can re-enable */
		isrc = intr_lookup_source(rid);
		KASSERT(isrc, ("interrupt not registered!"));


		ctx->ppdc_post_filter = isrc->is_event->ie_post_filter;
		isrc->is_event->ie_post_filter = isrc->is_event->ie_pre_ithread;

		cpuid = dpsc->dp_vcpumap[ppsi->ppsi_vcpuid].dpt_cpuid;
		intr_bind(rid, cpuid);
		/*
		 * interrupt setup complete - pass back the results
		 */
		ppsi->ppsi_vector = rid;
		ppsi->ppsi_tag = ctx->ppdc_tag;
		/* here's your cookie - don't lose it
		 * you won't be able to get your interrupt back without it
		 */
		ppsi->ppsi_cookie = isrc;
		return (0);
		fail:
		_dev_pass_softc_free(dpsc);
		return (rc);
		break;
	}
	case PCIPASSIOCTEARDOWNINTR: {
		struct pci_pass_teardown_intr *ppti = (void *)data;

		mtx_lock(&sc->pp_ctx_mtx);
		LIST_FOREACH(ctx, &sc->pp_ctx_list, pp_link) {
			if (ctx->ppdc_tag == ppti->ppti_tag) {
				LIST_REMOVE(ctx, pp_link);
				break;
			}
		}
		mtx_unlock(&sc->pp_ctx_mtx);
		if (ctx == NULL)
			return (ENOENT);
		ctx->ppdc_flags |= PPDC_DOOMED;
		mtx_lock(&sclist_mtx);
		LIST_REMOVE(ctx, tl_link);
		mtx_unlock(&sclist_mtx);

		isrc = intr_lookup_source(ctx->ppdc_vector);
		isrc->is_event->ie_post_filter = ctx->ppdc_post_filter;
		intr_bind(ctx->ppdc_vector, NOCPU);
		bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		bus_release_resource(ctx->ppdc_dev, SYS_RES_IRQ, ctx->ppdc_vector,
							 ctx->ppdc_irq);
		free(ctx, M_DEVBUF);
		break;
	}
	case PCIPASSIOCPOSTFILTER:
#ifdef notyet
		ctx->ppdc_flags |= PPDC_DISABLE_INTR;
		ctx->ppdc_write_reg = pppf->pppf_write_reg;
		ctx->ppdc_write_val = pppf->pppf_write_val;
		isrc->is_event->ie_post_filter = ctx->ppdc_post_filter;
#endif
/* FALLTHROUGH */
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static int dev_pass_syscall = NO_SYSCALL;

static d_close_t dev_pass_close;
static d_open_t dev_pass_open;
static d_ioctl_t dev_pass_ioctl;

static struct cdevsw dev_pass_cdevsw = {
       .d_version =    D_VERSION,
       .d_flags =      0,
       .d_open =       dev_pass_open,
       .d_close =      dev_pass_close,
	   .d_ioctl =	   dev_pass_ioctl,
       .d_name =       "devpass",
};

static void
_dev_pass_softc_free(struct dev_pass_softc *sc)
{
	int refcnt;

	if ((refcnt = atomic_fetchadd_int(&sc->dp_refcnt, -1)) > 1)
		return;
	mtx_lock(&sclist_mtx);
	LIST_REMOVE(sc, link);
	mtx_unlock(&sclist_mtx);
	_vcpu_unmap(sc);
	free(sc, M_DEVBUF);

	KASSERT(refcnt > 0, ("bad refcnt"));
}

static struct dev_pass_softc *
_dev_pass_softc_find(pid_t pid)
{
	struct dev_pass_softc *sciter;

	mtx_lock(&sclist_mtx);
	LIST_FOREACH(sciter, &dp_softc_list, link) {
		if (sciter->dp_pid == pid) {
			atomic_add_int(&sciter->dp_refcnt, 1);
			break;
		}
	}
	mtx_unlock(&sclist_mtx);
	return (sciter);
}

static struct thread_link *
_tl_find(struct thread *td, int alloc)
{
	struct thread_link *tl;

	mtx_lock(&sclist_mtx);
	LIST_FOREACH(tl, &thread_ctx_list, link) {
		if (tl->tl_td == td)
			break;
	}
	mtx_unlock(&sclist_mtx);
	if (tl != NULL || alloc == 0)
		return (tl);

	tl = malloc(sizeof(*tl), M_DEVBUF, M_WAITOK|M_ZERO);
	tl->tl_td = td;
	LIST_INIT(&tl->tl_ctx_list);
	mtx_lock(&sclist_mtx);
	/* technically we should check for duplicates */
	LIST_INSERT_HEAD(&thread_ctx_list, tl, link);
	mtx_unlock(&sclist_mtx);
	return (tl);
}

static void
_thread_exit_cleanup(void *arg, struct thread *td)
{
	struct pci_pass_driver_context *ctx;
	struct thread_link *tl;
	struct callout *c;

	if ((tl = _tl_find(td, FALSE)) == NULL)
		return;

	while (!LIST_EMPTY(&tl->tl_ctx_list)) {
		ctx = LIST_FIRST(&tl->tl_ctx_list);
		LIST_REMOVE(ctx, tl_link);
		if (ctx->ppdc_parent) {
			mtx_lock(&ctx->ppdc_parent->pp_ctx_mtx);
			LIST_REMOVE(ctx, pp_link);
			mtx_unlock(&ctx->ppdc_parent->pp_ctx_mtx);
			bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		} else {
			ctx->ppdc_flags |= PPDC_DOOMED;
			c = &ctx->ppdc_dp->dp_vcpumap[ctx->ppdc_vcpuid].dpt_c;
			callout_drain(c);
		}
		free(ctx, M_DEVBUF);
	}
	mtx_lock(&sclist_mtx);
	LIST_REMOVE(tl, link);
	mtx_unlock(&sclist_mtx);
	_unmap_uva(tl->tl_dpt->dpt_kstack - PAGE_SIZE);
	bzero(tl->tl_dpt, sizeof(struct dev_pass_tdvcpumap));
	free(tl, M_DEVBUF);
}

#if defined(__amd64__)
static void *
_map_uva(caddr_t uva)
{
	vm_offset_t kva;
	vm_paddr_t pa;
	vm_page_t m;

	if (useracc(uva, PAGE_SIZE, 1) == FALSE)
		return (NULL);
	/* XXX this breaks if uva is not physically contiguous
	 * should be fine for our purposes
	 */
	pa = pmap_extract(&curproc->p_vmspace->vm_pmap, (vm_offset_t)uva);
	m = PHYS_TO_VM_PAGE(pa);
	vm_page_lock(m);
	vm_page_wire(m);
	vm_page_unlock(m);
	kva = PHYS_TO_DMAP(pa);

	return ((void *)kva);
}

static void
_unmap_uva(void *kva)
{
	vm_paddr_t pa = DMAP_TO_PHYS((vm_offset_t)kva);
	vm_page_t m;

	m = PHYS_TO_VM_PAGE(pa);
	vm_page_lock(m);
	vm_page_unwire(m, PQ_INACTIVE);
	vm_page_unlock(m);
}

/*
 * Initial frame:
 *
 * trap handler
 * mask addr
 * %rax
 * %rbx
 * %rsp
 * %rip
 */
#define INIT_FRAME_SLOTS 6
#define INIT_FRAME_SIZE INIT_FRAME_SLOTS*sizeof(register_t)
static void
_setup_trapframe(struct thread *td, struct pci_pass_driver_context *ctx)
{
	int cpuid = ctx->ppdc_vcpuid;
	struct dev_pass_softc *dp = ctx->ppdc_dp;
	struct dev_pass_tdvcpumap *vcpumap = &dp->dp_vcpumap[cpuid];
	vm_offset_t code_page = dp->dp_code_page;
	caddr_t kstack = vcpumap->dpt_kstack;
	vm_offset_t curustack, ustack = vcpumap->dpt_ustack;
	struct pass_vcpu_info *vi;
	uint64_t *sp;
	struct trapframe *fp;

	if (td == curthread) {
		fp = td->td_intr_frame;
		/* Was the thread in the kernel when it was interrupted */
		if (!TRAPF_USERMODE(fp))
			fp = td->td_frame;
	} else
		fp = td->td_frame;

	/* Is this a nested interrupt */
	if (fp->tf_rsp < ustack &&
		fp->tf_rsp > ustack - PAGE_SIZE) {
		sp = (uint64_t *)kstack - (ustack - fp->tf_rsp);
		curustack = fp->tf_rsp;
	} else {
		sp = (uint64_t *)kstack;
		curustack = ustack;
	}

	vi = &ctx->ppdc_dp->dp_status_page->vcpu_info[ctx->ppdc_vcpuid];
	if (code_page && (fp->tf_rip >= code_page) &&
		(fp->tf_rip < code_page + sztrapcode)) {
		/* We've interrupted trap return - the values on the stack
		 * are still correct
		 */
		fp->tf_rsp = ustack - INIT_FRAME_SIZE;
	} else {
		sp--;
		*(sp--) = fp->tf_rip;
		*(sp--) = fp->tf_rsp;
		*(sp--) = fp->tf_rbx;
		*(sp--) = fp->tf_rax;
		*(sp--) = (register_t)&vi->evtchn_upcall_mask;
		*sp = (register_t)ctx->ppdc_trap;
		fp->tf_rsp = curustack - INIT_FRAME_SIZE;
	}
	if (code_page)
		fp->tf_rip = code_page;
	else
		fp->tf_rip = (register_t)ctx->ppdc_trap;
}

#else

static void *
_map_user_va(struct thread *td, caddr_t user_va)
{
	vm_offset_t kaddr;
	struct munlock_args uap;

	if (useracc(user_va, PAGE_SIZE, 1) == FALSE)
		return (NULL);
	if (vm_mlock(td->td_proc, td->td_ucred, user_va, PAGE_SIZE) != 0)
		return (NULL);
	if ((kaddr = kva_alloc(PAGE_SIZE)) == 0) {
		uap.addr = user_va;
		uap.len = PAGE_SIZE;
		sys_munlock(td, &uap);
		return (NULL);
	}
	pmap_qenter(kaddr, PHYS_TO_VM_PAGE(vtophys(user_va)), 1);

	return ((void *)kaddr);
}

static void
_unmap_uva(XXX)
{
	if (useracc(uva, PAGE_SIZE, 1) == TRUE) {
		uap.addr = uva;
		uap.len = PAGE_SIZE;
		sys_munlock(td, &uap);
	}
	pmap_qremove((vm_offset_t)kva, 1);
	kva_free((vm_offset_t)kva, PAGE_SIZE);
}
#endif

static void
_setup_code_page(void *va)
{

	memcpy(va, trapcode, sztrapcode);
}

static void
_setup_status_page(void *va)
{
	int i;
	struct pass_status_page *s;

	s = va;
	for (i = 0; i < sizeof(unsigned long)*8; i++) {
		s->evtchn_pending[i] = 0;
		s->evtchn_mask[i] = (unsigned long)-1LL;
	}
}

static int
dev_pass_open(struct cdev *dev, int flags, int fmp, struct thread *td)
{
	struct dev_pass_softc *sc;
	pid_t curpid;

	printf("dev_pass_open(...) called\n");
	curpid = td->td_proc->p_pid;
	sc = _dev_pass_softc_find(curpid);
	if (sc != NULL)
		return (0);
	if ((sc = malloc(sizeof(struct dev_pass_softc), M_DEVBUF, M_WAITOK|M_ZERO)) == NULL)
		return (ENOMEM);
	sc->dp_pid = curpid;
	sc->dp_refcnt = 1;
	mtx_lock(&sclist_mtx);
	LIST_INSERT_HEAD(&dp_softc_list, sc, link);
	mtx_unlock(&sclist_mtx);
	return (0);
}

static int
dev_pass_close(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	struct dev_pass_softc *sc;

	printf("dev_pass_close(...) called\n");
	sc = _dev_pass_softc_find(td->td_proc->p_pid);
	KASSERT(sc, ("sc not found in dev_pass_close"));

	atomic_add_int(&sc->dp_refcnt, -1);
	KASSERT(sc->dp_refcnt > 0, ("refcnt zero before free"));
	_dev_pass_softc_free(sc);
	return (0);
}

static struct pci_pass_driver_context *
_find_vector(struct thread *td, struct dev_pass_softc *sc, int vector)
{
	struct thread_link *tl = _tl_find(td, FALSE);
	struct pci_pass_driver_context *ctx = NULL;

	if (tl == NULL)
		return (NULL);
	mtx_lock(&sclist_mtx);
	LIST_FOREACH(ctx, &tl->tl_ctx_list, tl_link) {
		if (ctx->ppdc_vector == vector)
			break;
	}
	mtx_unlock(&sclist_mtx);
	return (ctx);
}

static int
dev_pass_ioctl(struct cdev *dev, u_long cmd, caddr_t data,
			   int flag, struct thread *td)
{
	struct dev_pass_softc *sc;
	int i, rc, vector;
	void *va;

	printf("dev_pass_ioctl(...) called\n");

	switch (cmd) {
	case DEVPASSIOCCODEPAGE: {
		caddr_t *uvap = (caddr_t *)data;
		sc = _dev_pass_softc_find(td->td_proc->p_pid);
		if ((va = _map_uva(*uvap)) == NULL) {
			rc = EFAULT;
			goto codedone;
		}
		/* dirt bag user tried to get two status pages */
		if (sc->dp_code_page != 0) {
			_unmap_uva(va);
			rc = EINVAL;
			goto codedone;
		}
		sc->dp_code_page_kern = va;
		sc->dp_code_page = (vm_offset_t)*uvap;
		_setup_code_page(va);
		rc = 0;
	codedone:
		_dev_pass_softc_free(sc);
		return (rc);
		break;
	}
	case DEVPASSIOCSTATUSPAGE: {
		caddr_t *uvap = (caddr_t *)data;
		sc = _dev_pass_softc_find(td->td_proc->p_pid);
		/* Don't lose your status page - you don't get another */
		if (sc->dp_status_page != NULL) {
			rc = EINVAL;
			goto statusdone;
		}
		if ((va = _map_uva(*uvap)) == NULL) {
			rc = EFAULT;
			goto statusdone;
		}
		/* dirt bag user tried to get two status pages */
		if (sc->dp_status_page != NULL) {
			_unmap_uva(va);
			rc = EINVAL;
			goto statusdone;
		}
		sc->dp_status_page = va;
		_setup_status_page(va);
		rc = 0;
	statusdone:
		_dev_pass_softc_free(sc);
		return (rc);
		break;
	}
	case DEVPASSIOCCHKIRQ:
		vector = *(int *)data;
		if (vector >= NUM_IO_INTS)
			return (EINVAL);
		if (intr_lookup_source(vector) == NULL)
			return (ENXIO);
		return (0);
		break;
	case DEVPASSIOCSYS:
		*(int *)data = dev_pass_syscall;
		break;
	case DEVPASSIOCVCPUMAP: {
		struct dev_pass_vcpumap *dpv = (void *)data;
		struct dev_pass_tidvcpumap *dpviter;
		struct dev_pass_tdvcpumap *dpt;
		struct pci_pass_driver_context *ctx;
		struct thread_link *tl;
		cpuset_t mask;
		struct thread *vcputd;
		int ncpus = dpv->dpv_nvcpus;

		if (ncpus > PCI_PASS_MAX_VCPUS)
			return (E2BIG);
		/* Also need the O(n^2) search to guarantee that the same thread
		 * isn't used twice
		 */
		for (dpviter = dpv->dpv_map, i = 0; i < ncpus; i++, dpviter++) {
			if (dpviter->dpt_cpuid > mp_ncpus)
				return (ERANGE);
			if (tdfind(dpviter->dpt_tid, curproc->p_pid) == NULL) {
				printf("failed to find tid: %d\n", dpviter->dpt_tid);
				return (EINVAL);
			}
			PROC_UNLOCK(curproc);
		}
		sc = _dev_pass_softc_find(curproc->p_pid);
		if ((dpv->dpv_flags & PCI_PASS_C_TRAPFRAME) &&
			sc->dp_code_page_kern == NULL) {
			printf("need to setup code page to setup C trapframe\n");
			return (EINVAL);
		}
		sc->dp_nvcpus = ncpus;
		sc->dp_trap = (vm_offset_t) dpv->dpv_trap;
		if (sc->dp_vcpumap != NULL)
			free(sc->dp_vcpumap, M_DEVBUF);
		dpt = sc->dp_vcpumap = malloc(sizeof(*dpt)*ncpus,
								M_DEVBUF, M_WAITOK|M_ZERO);
		for (dpviter = dpv->dpv_map, i = 0; i < ncpus; i++, dpt++, dpviter++) {
			dpt->dpt_cpuid = dpviter->dpt_cpuid;
			dpt->dpt_tid = dpviter->dpt_tid;
			dpviter->dpt_stk -= PAGE_SIZE;
			if ((dpt->dpt_kstack = _map_uva(dpviter->dpt_stk)) == NULL)
				break;
			dpviter->dpt_stk += PAGE_SIZE;
			dpt->dpt_kstack += PAGE_SIZE;
			dpt->dpt_ustack = (vm_offset_t)dpviter->dpt_stk;
		}
		if (i != ncpus) {
			int unmap_idx = i;

			for (i = 0, dpt = sc->dp_vcpumap; i < unmap_idx; i++, dpt++)
				_unmap_uva(dpt->dpt_kstack - PAGE_SIZE);
			return (EFAULT);
		}
		CPU_ZERO(&mask);
		for (dpviter = dpv->dpv_map, i = 0; i < ncpus; i++, dpviter++) {

			if ((vcputd = tdfind(dpviter->dpt_tid, curproc->p_pid)) == NULL) {
				printf("failed to find tid: %d\n", dpviter->dpt_tid);
				_dev_pass_softc_free(sc);
				return (EINVAL);
			}
			PROC_UNLOCK(curproc);
			sc->dp_vcpumap[i].dpt_td = vcputd;
			tl = _tl_find(vcputd, TRUE);
			sc->dp_vcpumap[i].dpt_tl = tl;
			tl->tl_dpt = &sc->dp_vcpumap[i];

			/* block interrupts and clear pending*/
			sc->dp_status_page->vcpu_info[i].evtchn_upcall_pending = 0;
			sc->dp_status_page->vcpu_info[i].evtchn_pending_sel = 0;
			sc->dp_status_page->vcpu_info[i].evtchn_upcall_mask = 1;

			ctx = _ctx_alloc(NULL, NULL, NULL, 0, i, sc, dpv->dpv_trap, NULL);
			/* add to thread list in case this thread goes away */
			mtx_lock(&sclist_mtx);
			/* NULL interrupt is the first */
			LIST_INSERT_HEAD(&tl->tl_ctx_list, ctx, tl_link);
			mtx_unlock(&sclist_mtx);

			/* bind thread */
			CPU_SET(dpviter->dpt_cpuid, &mask);
			rc = cpuset_setthread(dpviter->dpt_tid, &mask);
			CPU_CLR(dpviter->dpt_cpuid, &mask);

			if (rc != 0)
				printf("cpuset_setthread failed: %d\n", rc);
			callout_init(&sc->dp_vcpumap[i].dpt_c, TRUE);
		}
		_dev_pass_softc_free(sc);
		break;
	}
	case DEVPASSIOCTIMER: {
		struct dev_pass_softc *sc;
		struct dev_pass_timer *dpt = (void *)data;
		struct pci_pass_driver_context *ctx;
		struct dev_pass_tdvcpumap *dptv;
		void  *trap;
		int curvcpuid, vector;

		sc = _dev_pass_softc_find(curproc->p_pid);
		dptv = sc->dp_vcpumap;
		for (curvcpuid = 0; curvcpuid < sc->dp_nvcpus; curvcpuid++, dptv++)
			if (dptv->dpt_td == td)
				break;
		if (curvcpuid == sc->dp_nvcpus)
			goto timerfail;
		if (dpt->dpt_hz >= hz) {
			dpt->dpt_hz = hz;
			sc->dp_ticks = 1;
		} else if (dpt->dpt_hz == 0)
			goto timerfail;
		else
			sc->dp_ticks = hz/dpt->dpt_hz;

		trap = (dpt->dpt_trap != NULL) ? dpt->dpt_trap : (void *)sc->dp_trap;
		dpt->dpt_vector = vector = FIRST_TIMER_VECTOR + curvcpuid;
		if (_find_vector(td, sc, vector) == NULL) {
			ctx = _ctx_alloc(NULL, NULL, NULL, vector, curvcpuid, sc, trap, td);
			callout_reset_on(&dptv->dpt_c, sc->dp_ticks, _vcpu_timer, ctx,
							 dptv->dpt_cpuid);
		}

		_dev_pass_softc_free(sc);
		return (0);
		timerfail:
		_dev_pass_softc_free(sc);
		return (EINVAL);
	}
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static int
pci_pass_sys(struct thread *td, void *args)
{
	struct dps_args *uap = args;
	int vector;
	struct thread_link *tl;
	struct pci_pass_driver_context *ctx;

	switch (uap->sycall) {
	case PCI_PASS_APIC_ENABLE: {
		void *cookie;
		struct intsrc *isrc;

		cookie = uap->u.ppae.cookie;
		vector = uap->u.ppae.vector;

		if (vector == 0) {
			/* the user just wants to force a trap */
			if ((tl = _tl_find(td, FALSE)) == NULL)
				return (ENOENT);
			if ((ctx = LIST_FIRST(&tl->tl_ctx_list)) == NULL)
				return (ENOENT);
			(void)_pci_pass_driver_filter(ctx);
			return (0);
		}
		isrc = intr_lookup_source(vector);
		if (isrc == NULL)
			return (ENOENT);
		if (isrc != cookie)
			return (EINVAL);
		isrc->is_pic->pic_enable_source(isrc);
		break;
	}
	case PCI_PASS_IPI: {
		struct dev_pass_softc *dp;
		int vcpuid;

		vector = uap->u.ppi.vector;
		vcpuid = uap->u.ppi.vcpuid;
		if ((dp = _dev_pass_softc_find(curproc->p_pid)) == NULL)
			return (EINVAL);
		if (vcpuid > dp->dp_nvcpus)
			return (EINVAL);
		td = dp->dp_vcpumap[vcpuid].dpt_td;
		if (td == NULL)
			return (EINVAL);
		if ((ctx = LIST_FIRST(&dp->dp_vcpumap[vcpuid].dpt_tl->tl_ctx_list)) == NULL)
			return (EINVAL);
		(void)_pci_pass_driver_filter(ctx);
		break;
	}
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static void
dev_pass_ast_callback(struct thread *td)
{
	struct thread_link *tl;
	struct pci_pass_driver_context *ctx;

	if ((tl = _tl_find(td, FALSE)) == NULL)
		return;
	if ((ctx = LIST_FIRST(&tl->tl_ctx_list)) == NULL)
		return;
	(void)_pci_pass_driver_filter(ctx);
}

static int
dev_pass_init(void)
{
	struct sysent old_sysent, new_sysent;

	LIST_INIT(&dp_softc_list);
	LIST_INIT(&thread_ctx_list);
	mtx_init(&sclist_mtx, "dpsc list", NULL, MTX_DEF);
	make_dev(&dev_pass_cdevsw, unit, UID_ROOT, GID_WHEEL, 0660, "devpass");
	EVENTHANDLER_REGISTER(thread_dtor, _thread_exit_cleanup, NULL,
						  EVENTHANDLER_PRI_ANY);
	if (ast_cb != NULL) {
		printf("ast callback already registered\n");
		return (EBUSY);
	}
	ast_cb = dev_pass_ast_callback;
	new_sysent.sy_narg = 2;
	new_sysent.sy_call = pci_pass_sys;
	syscall_register(&dev_pass_syscall, &new_sysent, &old_sysent);
	/* GREAT EVIL in the name of great performance */
	sysent[dev_pass_syscall].sy_thrcnt = SY_THR_STATIC;
	return (0);
}

static int
devpass_module_event_handler(module_t mod, int what, void *arg)
{
	int err;

	switch (what) {
	case MOD_LOAD:
		if ((err = dev_pass_init()) != 0)
			return (err);
		break;
	case MOD_UNLOAD:
		return (EBUSY);
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static moduledata_t devpass_moduledata = {
	"devpass",
	devpass_module_event_handler,
	NULL
};

DECLARE_MODULE(devpass, devpass_moduledata, SI_SUB_DRIVERS, SI_ORDER_FIRST);
