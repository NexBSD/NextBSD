
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

static int	unit;
struct dev_pass_softc;
static void _dev_pass_softc_free(struct dev_pass_softc *dp);
static struct dev_pass_softc *_dev_pass_softc_find(pid_t pid);
static struct thread_link *_tl_find(struct thread *td, int alloc);

static void _unmap_va(void *kva);
static void *_map_user_va(struct thread *td, caddr_t uva, size_t size);
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
	volatile int tl_refcnt;
	LIST_ENTRY(thread_link) link;
	LIST_HEAD(,pci_pass_driver_context) tl_ctx_list;
};

LIST_HEAD(, dev_pass_softc) dp_softc_list;
LIST_HEAD(, thread_link) thread_ctx_list;
struct mtx sclist_mtx;

struct dev_pass_softc {
	void *dp_status_page;
	pid_t dp_pid;
	volatile int dp_refcnt;
	LIST_ENTRY(dev_pass_softc) link;
};

struct pci_pass_driver_context {
	device_t ppdc_dev;
	struct resource *ppdc_irq;
	struct thread *ppdc_td;
	void *ppdc_tag;
	vm_offset_t ppdc_ustack;
	vm_offset_t ppdc_trap_handler;
	void *ppdc_kstack;
	struct pci_pass_softc *ppdc_parent;
	int ppdc_vector;
	int ppdc_vcpuid;
	struct thread_link *ppdc_tl;
	struct dev_pass_softc *ppdc_dp;
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
		if (atomic_fetchadd_int(&ctx->ppdc_tl->tl_refcnt, -1) == 1)
			LIST_REMOVE(ctx->ppdc_tl, link);
		else
			ctx->ppdc_tl = NULL;
		mtx_unlock(&sclist_mtx);
		if (ctx->ppdc_tl != NULL)
			free(ctx->ppdc_tl, M_DEVBUF);

		_dev_pass_softc_free(ctx->ppdc_dp);
		bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		_unmap_va(ctx->ppdc_kstack);
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

static void
_set_intr_pending(struct shared_info *si, struct vcpu_info *vi,
				  int vector)
{
	int arridx = vector >> IDX_SHIFT;
	int bitidx = vector & IDX_MASK;
	int orpendval = 1 << bitidx;
	int orselval = 1 << arridx;

	atomic_set_long(&si->evtchn_pending[arridx], orpendval);
	atomic_set_long(&vi->evtchn_pending_sel, orselval);
	vi->evtchn_upcall_pending = 1;
}

static int
_intr_masked(struct shared_info *si, int vector)
{
	int arridx = vector >> IDX_SHIFT;
	int bitidx = vector & IDX_MASK;
	int orpendval = 1 << bitidx;

	return (si->evtchn_mask[arridx] & orpendval);
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
	} else {
		/*
		 * Other states do nothing with the signal immediately,
		 * other than kicking ourselves if we are running.
		 * It will either never be noticed, or noticed very soon.
		 */
#ifdef SMP
		if (TD_IS_RUNNING(td) && td != curthread)
			forward_signal(td);
#endif
	}
out:
	PROC_SUNLOCK(p);
	thread_unlock(td);
}

static int
_pci_pass_driver_filter(void *arg)
{
	struct pci_pass_driver_context *ctx = arg;
	struct thread *td = ctx->ppdc_td;
	struct shared_info *si;
	struct vcpu_info *vi;
	int needwakeup = 0;

	if (ctx->ppdc_flags & PPDC_DOOMED)
		return (FILTER_STRAY);

	si = ctx->ppdc_dp->dp_status_page;
	vi = &si->vcpu_info[ctx->ppdc_vcpuid];
	_set_intr_pending(si, vi, ctx->ppdc_vector);

	if (vi->evtchn_upcall_mask || _intr_masked(si, ctx->ppdc_vector))
		return (FILTER_HANDLED);

	thread_lock(td);
	if ((td != curthread) && TD_IS_RUNNING(td)){
		/* running on another cpu - send signal - overloading SIGVTALRM */
		td->td_flags |= (TDF_ASTPENDING|TDF_ALRMPEND);
		ipi_cpu(td->td_oncpu, IPI_AST);
	} else if ((td == curthread && TD_IS_RUNNING(td)) || TD_ON_RUNQ(td)) {
		_setup_trapframe(ctx);
	} else {
		td->td_flags |= (TDF_ASTPENDING|TDF_ALRMPEND);
		needwakeup = 1;
	}
	thread_unlock(td);
	if (needwakeup)
		_intr_tdsigwakeup(td, EINTR);
	return (FILTER_HANDLED);
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
		void *kstack;
		int refcnt, rc, rid = ppsi->ppsi_vector;

		if (ppsi->ppsi_vcpuid > PCI_PASS_MAX_VCPUS)
			return (EINVAL);
		if ((itd = tdfind(ppsi->ppsi_tid, td->td_proc->p_pid)) == NULL)
			return (EINVAL);
		if ((kstack = _map_user_va(itd, ppsi->ppsi_stk, PAGE_SIZE)) == NULL)
			return (EFAULT);
		if ((dpsc = _dev_pass_softc_find(td->td_proc->p_pid)) == NULL ||
			dpsc->dp_status_page == NULL)
			return (ENXIO);
		tl = _tl_find(itd, TRUE);
		ctx = malloc(sizeof(*ctx), M_DEVBUF, M_WAITOK|M_ZERO);

		if ((r = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE)) == NULL) {
			rc = ENOMEM;
			goto tlfail;
		}
		if ((rc = bus_setup_intr(dev, r, INTR_TYPE_NET, _pci_pass_driver_filter,
								 NULL, ctx, &ctx->ppdc_tag)) != 0) {
			bus_release_resource(dev, SYS_RES_IRQ, rid, r);
			goto tlfail;
		}

		/* we need the APIC disabled until the user can re-enable */
		isrc = intr_lookup_source(rid);
		KASSERT(isrc, ("interrupt not registered!"));
		ctx->ppdc_post_filter = isrc->is_event->ie_post_filter;
		isrc->is_event->ie_post_filter = isrc->is_event->ie_pre_ithread;
		ctx->ppdc_dev = dev;
		ctx->ppdc_irq = r;
		ctx->ppdc_vector = rid;
		ctx->ppdc_ustack = (vm_offset_t)ppsi->ppsi_stk;
		ctx->ppdc_kstack = kstack;
		ctx->ppdc_parent = sc;
		ctx->ppdc_tl = tl;
		ctx->ppdc_dp = dpsc;
		ctx->ppdc_vcpuid = ppsi->ppsi_vcpuid;
		ctx->ppdc_trap_handler = (vm_offset_t)ppsi->ppsi_trap_handler;
		mtx_lock(&sclist_mtx);
		LIST_INSERT_HEAD(&tl->tl_ctx_list, ctx, tl_link);
		mtx_unlock(&sclist_mtx);
		mtx_lock(&sc->pp_ctx_mtx);
		LIST_INSERT_HEAD(&sc->pp_ctx_list, ctx, pp_link);
		mtx_unlock(&sc->pp_ctx_mtx);
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
		tlfail:
		_unmap_va(kstack);
		mtx_lock(&sclist_mtx);
		if ((refcnt = atomic_fetchadd_int(&tl->tl_refcnt, -1)) == 1) {
			LIST_REMOVE(tl, link);
			mtx_unlock(&sclist_mtx);
			free(tl, M_DEVBUF);
		} else
			mtx_unlock(&sclist_mtx);
		KASSERT(refcnt > 0, ("bad refcnt"));
		free(ctx, M_DEVBUF);
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
		if (atomic_fetchadd_int(&ctx->ppdc_tl->tl_refcnt, -1) == 1)
			LIST_REMOVE(ctx->ppdc_tl, link);
		else
			ctx->ppdc_tl = NULL;
		mtx_unlock(&sclist_mtx);
		if (ctx->ppdc_tl != NULL)
			free(ctx->ppdc_tl, M_DEVBUF);

		isrc = intr_lookup_source(ctx->ppdc_vector);
		isrc->is_event->ie_post_filter = ctx->ppdc_post_filter;
		bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		bus_release_resource(ctx->ppdc_dev, SYS_RES_IRQ, ctx->ppdc_vector,
							 ctx->ppdc_irq);
		_unmap_va(ctx->ppdc_kstack);
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

static int apic_enable_syscall = NO_SYSCALL;

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
	if (sc->dp_status_page != NULL)
		_unmap_va(sc->dp_status_page);
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
		if (tl->tl_td == td) {
			atomic_add_int(&tl->tl_refcnt, 1);
			break;
		}
	}
	mtx_unlock(&sclist_mtx);
	if (tl != NULL || alloc == 0)
		return (tl);

	tl = malloc(sizeof(*tl), M_DEVBUF, M_WAITOK);
	tl->tl_refcnt = 1;
	tl->tl_td = td;
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

	if ((tl = _tl_find(td, FALSE)) == NULL)
		return;

	while (!LIST_EMPTY(&tl->tl_ctx_list)) {
		ctx = LIST_FIRST(&tl->tl_ctx_list);
		LIST_REMOVE(ctx, tl_link);
		mtx_lock(&ctx->ppdc_parent->pp_ctx_mtx);
		LIST_REMOVE(ctx, pp_link);
		mtx_unlock(&ctx->ppdc_parent->pp_ctx_mtx);
		bus_teardown_intr(ctx->ppdc_dev, ctx->ppdc_irq, ctx->ppdc_tag);
		_unmap_va(ctx->ppdc_kstack);
		free(ctx, M_DEVBUF);
	}
}

#if defined(__amd64__)
static void *
_map_user_va(struct thread *td, caddr_t uva, size_t size)
{
	vm_offset_t kva;
	vm_paddr_t pa;
	vm_page_t m;

	if (useracc(uva, PAGE_SIZE, size >> PAGE_SHIFT) == FALSE)
		return (NULL);
	/* XXX this breaks if uva is not physically contiguous
	 * should be fine for our purposes
	 */
	pa = pmap_extract(&td->td_proc->p_vmspace->vm_pmap, (vm_offset_t)uva);
	m = PHYS_TO_VM_PAGE(pa);
	vm_page_lock(m);
	vm_page_wire(m);
	vm_page_unlock(m);
	kva = PHYS_TO_DMAP(pa);

	return ((void *)kva);
}

static void
_unmap_va(void *kva)
{
	vm_paddr_t pa = DMAP_TO_PHYS((vm_offset_t)kva);
	vm_page_t m;

	m = PHYS_TO_VM_PAGE(pa);
	vm_page_lock(m);
	vm_page_unwire(m, PQ_INACTIVE);
	vm_page_unlock(m);
}


static void
_setup_trapframe(struct pci_pass_driver_context *ctx)
{
	struct thread *td = ctx->ppdc_td;
	uint64_t *sp = ctx->ppdc_kstack;
	struct trapframe *fp;

	if (td == curthread) {
		fp = td->td_intr_frame;
		/* Was the thread in the kernel when it was interrupted */
		if (!TRAPF_USERMODE(fp))
			fp = td->td_frame;
	} else
		fp = td->td_frame;
	sp--;
	*(sp--) = fp->tf_rip;
	*sp = fp->tf_rsp;
	fp->tf_rip = (register_t)ctx->ppdc_trap_handler;
	/* Is this a nested interrupt */
	if (fp->tf_rsp < ctx->ppdc_ustack &&
		fp->tf_rsp > ctx->ppdc_ustack - PAGE_SIZE) {
		fp->tf_rsp -= 16;
	} else
		fp->tf_rsp = ctx->ppdc_ustack-16;
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
_unmap_va(XXX)
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
	if ((sc = malloc(sizeof(struct dev_pass_softc), M_DEVBUF, M_WAITOK)) == NULL)
		return (ENOMEM);
	sc->dp_pid = curpid;
	sc->dp_status_page = NULL;
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
	_dev_pass_softc_free(sc);
	return (0);
}

static int
dev_pass_ioctl(struct cdev *dev, u_long cmd, caddr_t data,
			   int flag, struct thread *td)
{
	struct dev_pass_softc *sc;
	int rc, vector;
	void *va;

	printf("dev_pass_ioctl(...) called\n");

	switch (cmd) {
	case DEVPASSIOCSTATUSPAGE:
		sc = _dev_pass_softc_find(td->td_proc->p_pid);
		/* Don't lose your status page - you don't get another */
		if (sc->dp_status_page != NULL) {
			rc = EINVAL;
			goto done;
		}
		if ((va = _map_user_va(curthread, data, PAGE_SIZE)) == NULL) {
			rc = EFAULT;
			goto done;
		}
		/* dirt bag user tried to get two status pages */
		if (sc->dp_status_page != NULL) {
			_unmap_va(va);
			rc = EINVAL;
			goto done;
		}
		sc->dp_status_page = (void *)va;
		rc = 0;
	done:
		_dev_pass_softc_free(sc);
		return (rc);
		break;
	case DEVPASSIOCCHKIRQ:
		vector = *(int *)data;
		if (vector >= NUM_IO_INTS)
			return (EINVAL);
		if (intr_lookup_source(vector) == NULL)
			return (ENXIO);
		return (0);
		break;
	case DEVPASSIOCAPICENABLESYS:
		*(int *)data = apic_enable_syscall;
		break;
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static int
pci_pass_apic_enable(struct thread *td, void *args)
{
	struct ppae_args *uap = args;
	void *cookie = uap->cookie;
	int vector = uap->vector;
	struct intsrc *isrc = intr_lookup_source(vector);

	if (isrc == NULL)
		return (ENOENT);
	if (isrc != cookie)
		return (EINVAL);
	isrc->is_pic->pic_enable_source(isrc);
	return (0);
}

static int
dev_pass_init(void)
{
	struct sysent *new_sysent, *old_sysent = NULL;

	LIST_INIT(&dp_softc_list);
	LIST_INIT(&thread_ctx_list);
	mtx_init(&sclist_mtx, "dpsc list", NULL, MTX_DEF);
	make_dev(&dev_pass_cdevsw, unit, UID_ROOT, GID_WHEEL, 0660, "devpass");
	EVENTHANDLER_REGISTER(thread_dtor, _thread_exit_cleanup, NULL,
						  EVENTHANDLER_PRI_ANY);

	new_sysent = malloc(sizeof(struct sysent), M_DEVBUF, M_WAITOK|M_ZERO);
	new_sysent->sy_narg = 2;
	new_sysent->sy_call = pci_pass_apic_enable;
	syscall_register(&apic_enable_syscall, new_sysent, old_sysent);
	/* GREAT EVIL in the name of great performance */
	new_sysent->sy_thrcnt = SY_THR_STATIC;
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
