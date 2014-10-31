/******************************************************************************
 * ukern_intr.c
 *
 *
 * Copyright (c) 2002-2005, K A Fraser
 * Copyright (c) 2005, Intel Corporation <xiaofeng.ling@intel.com>
 * Copyright (c) 2012, Spectra Logic Corporation
 * Copyright (c) 2014, Matthew Macy <kmacy@freebsd.org>
 *
 * This file may be distributed separately from the Linux kernel, or
 * incorporated into other software packages, subject to the following license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/interrupt.h>
#include <sys/pcpu.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/intr_machdep.h>
#include <x86/apicvar.h>
#include <x86/apicreg.h>
#include <machine/smp.h>
#include <machine/stdarg.h>

#include <machine/xen/synch_bitops.h>

#include <ukern_intr.h>
#include <ukern_device.h>
#include <dev/pci/pcivar.h>

#ifdef DDB
#include <ddb/ddb.h>
#endif

int *__error();
extern int ioctl(int fd, unsigned long request, ...);

static MALLOC_DEFINE(M_UKERNINTR, "ukern_intr", "Ukern Interrupt Services");

/**
 * Per-cpu event channel processing state.
 */
struct ukern_intr_pcpu_data {
	/**
	 * The last event channel bitmap section (level one bit) processed.
	 * This is used to ensure we scan all ports before
	 * servicing an already servied port again.
	 */
	u_int	last_processed_l1i;

	/**
	 * The last event channel processed within the event channel
	 * bitmap being scanned.
	 */
	u_int	last_processed_l2i;

	/** Pointer to this CPU's interrupt statistic counter. */
	u_long *evtchn_intrcnt;

	/**
	 * A bitmap of ports that can be serviced from this CPU.
	 * A set bit means interrupt handling is enabled.
	 */
	u_long	evtchn_enabled[sizeof(u_long) * 8];
};

/*
 * Start the scan at port 0 by initializing the last scanned
 * location as the highest numbered event channel port.
 */
DPCPU_DEFINE(struct ukern_intr_pcpu_data, ukern_intr_pcpu) = {
	.last_processed_l1i = LONG_BIT - 1,
	.last_processed_l2i = LONG_BIT - 1
};

DPCPU_DEFINE(struct pass_vcpu_info *, vcpu_info);

#define is_valid_evtchn(x)	((x) != 0)

#define	XEN_EEXIST		17 /* Xen "already exists" error */
#define	XEN_ALLOCATE_VECTOR	0 /* Allocate a vector for this event channel */

struct ukernisrc {
	struct intsrc	xi_intsrc;
	int		xi_cpu;		/* VCPU for delivery. */
	int		xi_vector;	/* Global isrc vector number. */
	int		xi_pirq;
	int		xi_virq;
	u_int		xi_close:1;	/* close on unbind? */
	u_int		xi_shared:1;	/* Shared with other domains. */
	u_int		xi_activehi:1;
	u_int		xi_edgetrigger:1;
	enum evtchn_type xi_type;
	evtchn_port_t	xi_port;
};

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof(a[0]))

static void	ukern_intr_suspend(struct pic *);
static void	ukern_intr_resume(struct pic *, bool suspend_cancelled);
static void	ukern_intr_enable_source(struct intsrc *isrc);
static void	ukern_intr_disable_source(struct intsrc *isrc, int eoi);
static void	ukern_intr_eoi_source(struct intsrc *isrc);
static void	ukern_intr_enable_intr(struct intsrc *isrc);
static void	ukern_intr_disable_intr(struct intsrc *isrc);
static int	ukern_intr_vector(struct intsrc *isrc);
static int	ukern_intr_source_pending(struct intsrc *isrc);
static int	ukern_intr_config_intr(struct intsrc *isrc,
		     enum intr_trigger trig, enum intr_polarity pol);
static int	ukern_intr_assign_cpu(struct intsrc *isrc, u_int apic_id);

static void	ukern_intr_pirq_enable_source(struct intsrc *isrc);
static void	ukern_intr_pirq_disable_source(struct intsrc *isrc, int eoi);
static void	ukern_intr_pirq_eoi_source(struct intsrc *isrc);
static void	ukern_intr_pirq_enable_intr(struct intsrc *isrc);
static void	ukern_intr_pirq_disable_intr(struct intsrc *isrc);
static int	ukern_intr_pirq_config_intr(struct intsrc *isrc,
		     enum intr_trigger trig, enum intr_polarity pol);

/**
 * PIC interface for all event channel port types except physical IRQs.
 */
struct pic ukern_intr_pic = {
	.pic_enable_source  = ukern_intr_enable_source,
	.pic_disable_source = ukern_intr_disable_source,
	.pic_eoi_source     = ukern_intr_eoi_source,
	.pic_enable_intr    = ukern_intr_enable_intr,
	.pic_disable_intr   = ukern_intr_disable_intr,
	.pic_vector         = ukern_intr_vector,
	.pic_source_pending = ukern_intr_source_pending,
	.pic_suspend        = ukern_intr_suspend,
	.pic_resume         = ukern_intr_resume,
	.pic_config_intr    = ukern_intr_config_intr,
	.pic_assign_cpu     = ukern_intr_assign_cpu
};

/**
 * PIC interface for all event channel representing
 * physical interrupt sources.
 */
struct pic ukern_intr_pirq_pic = {
	.pic_enable_source  = ukern_intr_pirq_enable_source,
	.pic_disable_source = ukern_intr_pirq_disable_source,
	.pic_eoi_source     = ukern_intr_pirq_eoi_source,
	.pic_enable_intr    = ukern_intr_pirq_enable_intr,
	.pic_disable_intr   = ukern_intr_pirq_disable_intr,
	.pic_vector         = ukern_intr_vector,
	.pic_source_pending = ukern_intr_source_pending,
	.pic_config_intr    = ukern_intr_pirq_config_intr,
	.pic_assign_cpu     = ukern_intr_assign_cpu
};

static struct mtx	 ukern_intr_isrc_lock;
static int		 ukern_intr_auto_vector_count;
static struct ukernisrc	*ukern_intr_port_to_isrc[NR_EVENT_CHANNELS];
static u_long		*ukern_intr_pirq_eoi_map;

/*------------------------- Private Functions --------------------------------*/
/**
 * Disable signal delivery for an event channel port on the
 * specified CPU.
 *
 * \param port  The event channel port to mask.
 *
 * This API is used to manage the port<=>CPU binding of event
 * channel handlers.
 *
 * \note  This operation does not preclude reception of an event
 *        for this event channel on another CPU.  To mask the
 *        event channel globally, use evtchn_mask().
 */
static inline void
evtchn_cpu_mask_port(u_int cpu, evtchn_port_t port)
{
	struct ukern_intr_pcpu_data *pcpu;

	pcpu = DPCPU_ID_PTR(cpu, ukern_intr_pcpu);
	clear_bit(port, pcpu->evtchn_enabled);
}

/**
 * Enable signal delivery for an event channel port on the
 * specified CPU.
 *
 * \param port  The event channel port to unmask.
 *
 * This API is used to manage the port<=>CPU binding of event
 * channel handlers.
 *
 * \note  This operation does not guarantee that event delivery
 *        is enabled for this event channel port.  The port must
 *        also be globally enabled.  See evtchn_unmask().
 */
static inline void
evtchn_cpu_unmask_port(u_int cpu, evtchn_port_t port)
{
	struct ukern_intr_pcpu_data *pcpu;

	pcpu = DPCPU_ID_PTR(cpu, ukern_intr_pcpu);
	set_bit(port, pcpu->evtchn_enabled);
}

/**
 * Allocate and register a per-cpu ukern upcall interrupt counter.
 *
 * \param cpu  The cpu for which to register this interrupt count.
 */
static void
ukern_intr_intrcnt_add(u_int cpu)
{
	char buf[MAXCOMLEN + 1];
	struct ukern_intr_pcpu_data *pcpu;

	pcpu = DPCPU_ID_PTR(cpu, ukern_intr_pcpu);
	if (pcpu->evtchn_intrcnt != NULL)
		return;

	snprintf(buf, sizeof(buf), "cpu%d:xen", cpu);
	intrcnt_add(buf, &pcpu->evtchn_intrcnt);
}

/**
 * Search for an already allocated but currently unused ukern interrupt
 * source object.
 *
 * \param type  Restrict the search to interrupt sources of the given
 *              type.
 *
 * \return  A pointer to a free ukern interrupt source object or NULL.
 */
static struct ukernisrc *
ukern_intr_find_unused_isrc(enum evtchn_type type)
{
	int isrc_idx;

	KASSERT(mtx_owned(&ukern_intr_isrc_lock), ("Evtchn isrc lock not held"));

	for (isrc_idx = 0; isrc_idx < ukern_intr_auto_vector_count; isrc_idx ++) {
		struct ukernisrc *isrc;
		u_int vector;

		vector = FIRST_EVTCHN_INT + isrc_idx;
		isrc = (struct ukernisrc *)intr_lookup_source(vector);
		if (isrc != NULL
		 && isrc->xi_type == EVTCHN_TYPE_UNBOUND) {
			KASSERT(isrc->xi_intsrc.is_handlers == 0,
			    ("Free evtchn still has handlers"));
			isrc->xi_type = type;
			return (isrc);
		}
	}
	return (NULL);
}

/**
 * Allocate a ukern interrupt source object.
 *
 * \param type  The type of interrupt source to create.
 *
 * \return  A pointer to a newly allocated ukern interrupt source
 *          object or NULL.
 */
static struct ukernisrc *
ukern_intr_alloc_isrc(enum evtchn_type type, int vector)
{
	static int warned;
	struct ukernisrc *isrc;

	KASSERT(mtx_owned(&ukern_intr_isrc_lock), ("Evtchn alloc lock not held"));

	if (ukern_intr_auto_vector_count > NR_EVENT_CHANNELS) {
		if (!warned) {
			warned = 1;
			printf("ukern_intr_alloc: Event channels exhausted.\n");
		}
		return (NULL);
	}

	KASSERT((intr_lookup_source(vector) == NULL),
	    ("Trying to use an already allocated vector"));

	mtx_unlock(&ukern_intr_isrc_lock);
	isrc = malloc(sizeof(*isrc), M_UKERNINTR, M_WAITOK | M_ZERO);
	isrc->xi_intsrc.is_pic =
	    (type == EVTCHN_TYPE_PIRQ) ? &ukern_intr_pirq_pic : &ukern_intr_pic;
	isrc->xi_vector = vector;
	isrc->xi_type = type;
	intr_register_source(&isrc->xi_intsrc);
	mtx_lock(&ukern_intr_isrc_lock);

	return (isrc);
}

/**
 * Attempt to free an active ukern interrupt source object.
 *
 * \param isrc  The interrupt source object to release.
 *
 * \returns  EBUSY if the source is still in use, otherwise 0.
 */
static int
ukern_intr_release_isrc(struct ukernisrc *isrc)
{

	mtx_lock(&ukern_intr_isrc_lock);
	if (isrc->xi_intsrc.is_handlers != 0) {
		mtx_unlock(&ukern_intr_isrc_lock);
		return (EBUSY);
	}
	evtchn_mask_port(isrc->xi_port);
	evtchn_clear_port(isrc->xi_port);

	/* Rebind port to CPU 0. */
	evtchn_cpu_mask_port(isrc->xi_cpu, isrc->xi_port);
	evtchn_cpu_unmask_port(0, isrc->xi_port);

	if (isrc->xi_close != 0 && is_valid_evtchn(isrc->xi_port)) {
#if 0
		struct evtchn_close close = { .port = isrc->xi_port };
		if (HYPERVISOR_event_channel_op(EVTCHNOP_close, &close))
			panic("EVTCHNOP_close failed");
#endif		
	}

	ukern_intr_port_to_isrc[isrc->xi_port] = NULL;
	isrc->xi_cpu = 0;
	isrc->xi_type = EVTCHN_TYPE_UNBOUND;
	isrc->xi_port = 0;
	mtx_unlock(&ukern_intr_isrc_lock);
	return (0);
}

/**
 * Associate an interrupt handler with an already allocated local Xen
 * event channel port.
 *
 * \param isrcp       The returned Xen interrupt object associated with
 *                    the specified local port.
 * \param local_port  The event channel to bind.
 * \param type        The event channel type of local_port.
 * \param intr_owner  The device making this bind request.
 * \param filter      An interrupt filter handler.  Specify NULL
 *                    to always dispatch to the ithread handler.
 * \param handler     An interrupt ithread handler.  Optional (can
 *                    specify NULL) if all necessary event actions
 *                    are performed by filter.
 * \param arg         Argument to present to both filter and handler.
 * \param irqflags    Interrupt handler flags.  See sys/bus.h.
 * \param handlep     Pointer to an opaque handle used to manage this
 *                    registration.
 *
 * \returns  0 on success, otherwise an errno.
 */
#if 0
static int
ukern_intr_bind_isrc(struct ukernisrc **isrcp, evtchn_port_t local_port,
    enum evtchn_type type, device_t intr_owner, driver_filter_t filter,
    driver_intr_t handler, void *arg, enum intr_type flags,
    ukern_intr_handle_t *port_handlep)
{
	struct ukernisrc *isrc;
	int error;

	*isrcp = NULL;
	if (port_handlep == NULL) {
		device_printf(intr_owner,
			      "ukern_intr_bind_isrc: Bad event handle\n");
		return (EINVAL);
	}

	mtx_lock(&ukern_intr_isrc_lock);
	isrc = ukern_intr_find_unused_isrc(type);
	if (isrc == NULL) {
		isrc = ukern_intr_alloc_isrc(type, XEN_ALLOCATE_VECTOR);
		if (isrc == NULL) {
			mtx_unlock(&ukern_intr_isrc_lock);
			return (ENOSPC);
		}
	}
	isrc->xi_port = local_port;
	ukern_intr_port_to_isrc[local_port] = isrc;
	mtx_unlock(&ukern_intr_isrc_lock);

	error = intr_add_handler(device_get_nameunit(intr_owner),
				 isrc->xi_vector, filter, handler, arg,
				 flags|INTR_EXCL, port_handlep);
	if (error != 0) {
		device_printf(intr_owner,
			      "ukern_intr_bind_irq: intr_add_handler failed\n");
		ukern_intr_release_isrc(isrc);
		return (error);
	}
	*isrcp = isrc;
	evtchn_unmask_port(local_port);
	return (0);
}

/**
 * Lookup a Xen interrupt source object given an interrupt binding handle.
 * 
 * \param handle  A handle initialized by a previous call to
 *                ukern_intr_bind_isrc().
 *
 * \returns  A pointer to the Xen interrupt source object associated
 *           with the given interrupt handle.  NULL if no association
 *           currently exists.
 */
static struct ukernisrc *
ukern_intr_isrc(ukern_intr_handle_t handle)
{
	struct intr_handler *ih;

	ih = handle;
	if (ih == NULL || ih->ih_event == NULL)
		return (NULL);

	return (ih->ih_event->ie_source);
}
#endif

/**
 * Determine the event channel ports at the given section of the
 * event port bitmap which have pending events for the given cpu.
 * 
 * \param pcpu  The Xen interrupt pcpu data for the cpu being querried.
 * \param sh    The Xen shared info area.
 * \param idx   The index of the section of the event channel bitmap to
 *              inspect.
 *
 * \returns  A u_long with bits set for every event channel with pending
 *           events.
 */
static inline u_long
ukern_intr_active_ports(struct ukern_intr_pcpu_data *pcpu, struct pass_status_page *sh,
    u_int idx)
{
	return (sh->evtchn_pending[idx]
	      & ~sh->evtchn_mask[idx]
	      & pcpu->evtchn_enabled[idx]);
}

/**
 * Interrupt handler for processing all Xen event channel events.
 * 
 * \param trap_frame  The trap frame context for the current interrupt.
 */
void
ukern_intr(struct trapframe *trap_frame)
{
	u_int l1i, l2i, port, cpu;
	u_long masked_l1, masked_l2;
	struct ukernisrc *isrc;
	struct pass_status_page *s;
	struct pass_vcpu_info *v;
	struct ukern_intr_pcpu_data *pc;
	u_long l1, l2;

	/*
	 * Disable preemption in order to always check and fire events
	 * on the right vCPU
	 */
	critical_enter();

	cpu = PCPU_GET(cpuid);
	pc  = DPCPU_PTR(ukern_intr_pcpu);
	s   = status_page;
	v   = DPCPU_GET(vcpu_info);

	v->evtchn_upcall_pending = 0;

#if 0
#ifndef CONFIG_X86 /* No need for a barrier -- XCHG is a barrier on x86. */
	/* Clear master flag /before/ clearing selector flag. */
	wmb();
#endif
#endif

	l1 = atomic_readandclear_long(&v->evtchn_pending_sel);

	l1i = pc->last_processed_l1i;
	l2i = pc->last_processed_l2i;
	(*pc->evtchn_intrcnt)++;

	while (l1 != 0) {

		l1i = (l1i + 1) % LONG_BIT;
		masked_l1 = l1 & ((~0UL) << l1i);

		if (masked_l1 == 0) {
			/*
			 * if we masked out all events, wrap around
			 * to the beginning.
			 */
			l1i = LONG_BIT - 1;
			l2i = LONG_BIT - 1;
			continue;
		}
		l1i = ffsl(masked_l1) - 1;

		do {
			l2 = ukern_intr_active_ports(pc, s, l1i);

			l2i = (l2i + 1) % LONG_BIT;
			masked_l2 = l2 & ((~0UL) << l2i);

			if (masked_l2 == 0) {
				/* if we masked out all events, move on */
				l2i = LONG_BIT - 1;
				break;
			}
			l2i = ffsl(masked_l2) - 1;

			/* process port */
			port = (l1i * LONG_BIT) + l2i;
			synch_clear_bit(port, &s->evtchn_pending[0]);

			isrc = ukern_intr_port_to_isrc[port];
			if (__predict_false(isrc == NULL))
				continue;

			/* Make sure we are firing on the right vCPU */
			KASSERT((isrc->xi_cpu == PCPU_GET(cpuid)),
				("Received unexpected event on vCPU#%d, event bound to vCPU#%d",
				PCPU_GET(cpuid), isrc->xi_cpu));

			intr_execute_handlers(&isrc->xi_intsrc, trap_frame);

			/*
			 * If this is the final port processed,
			 * we'll pick up here+1 next time.
			 */
			pc->last_processed_l1i = l1i;
			pc->last_processed_l2i = l2i;

		} while (l2i != LONG_BIT - 1);

		l2 = ukern_intr_active_ports(pc, s, l1i);
		if (l2 == 0) {
			/*
			 * We handled all ports, so we can clear the
			 * selector bit.
			 */
			l1 &= ~(1UL << l1i);
		}
	}
	critical_exit();
}

static int
ukern_intr_init(void *dummy __unused)
{
	struct pass_status_page *s = status_page;
	struct ukern_intr_pcpu_data *pcpu;
	int i;

	mtx_init(&ukern_intr_isrc_lock, "ukern-irq-lock", NULL, MTX_DEF);

	/*
	 * Register interrupt count manually as we aren't
	 * guaranteed to see a call to ukern_intr_assign_cpu()
	 * before our first interrupt. Also set the per-cpu
	 * mask of CPU#0 to enable all, since by default
	 * all event channels are bound to CPU#0.
	 */
	CPU_FOREACH(i) {
		pcpu = DPCPU_ID_PTR(i, ukern_intr_pcpu);
		memset(pcpu->evtchn_enabled, i == 0 ? ~0 : 0,
		       sizeof(pcpu->evtchn_enabled));
		ukern_intr_intrcnt_add(i);
	}

	for (i = 0; i < nitems(s->evtchn_mask); i++)
		atomic_store_rel_long(&s->evtchn_mask[i], ~0);
#if 0
	/* Try to register PIRQ EOI map */
	ukern_intr_pirq_eoi_map = malloc(PAGE_SIZE, M_UKERNINTR, M_WAITOK | M_ZERO);
	eoi_gmfn.gmfn = atop(vtophys(ukern_intr_pirq_eoi_map));
	rc = HYPERVISOR_physdev_op(PHYSDEVOP_pirq_eoi_gmfn_v2, &eoi_gmfn);
	if (rc != 0 && bootverbose)
		printf("Xen interrupts: unable to register PIRQ EOI map\n");
	else
		ukern_intr_pirq_eoi_map_enabled = true;
#endif
	intr_register_pic(&ukern_intr_pic);
	intr_register_pic(&ukern_intr_pirq_pic);

	if (bootverbose)
		printf("Xen interrupt system initialized\n");

	return (0);
}
SYSINIT(ukern_intr_init, SI_SUB_INTR, SI_ORDER_SECOND, ukern_intr_init, NULL);

/*--------------------------- Common PIC Functions ---------------------------*/
/**
 * Prepare this PIC for system suspension.
 */
static void
ukern_intr_suspend(struct pic *unused)
{
}

static void
xen_rebind_ipi(struct ukernisrc *isrc)
{
#if 0 && defined(SMP)
	int cpu = isrc->xi_cpu;
	int vcpu_id = pcpu_find(cpu)->pc_vcpu_id;
	int error;
	struct evtchn_bind_ipi bind_ipi = { .vcpu = vcpu_id };

	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_ipi,
	                                    &bind_ipi);
	if (error != 0)
		panic("unable to rebind xen IPI: %d", error);

	isrc->xi_port = bind_ipi.port;
	isrc->xi_cpu = 0;
	ukern_intr_port_to_isrc[bind_ipi.port] = isrc;

	error = ukern_intr_assign_cpu(&isrc->xi_intsrc,
	                            cpu_apic_ids[cpu]);
	if (error)
		panic("unable to bind xen IPI to CPU#%d: %d",
		      cpu, error);

	evtchn_unmask_port(bind_ipi.port);
#else
	panic("Resume IPI event channel on UP");
#endif
}

static void
xen_rebind_virq(struct ukernisrc *isrc)
{
#if 0	
	int cpu = isrc->xi_cpu;
	int vcpu_id = pcpu_find(cpu)->pc_vcpu_id;
	int error;
	struct evtchn_bind_virq bind_virq = { .virq = isrc->xi_virq,
	                                      .vcpu = vcpu_id };

	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_virq,
	                                    &bind_virq);
	if (error != 0)
		panic("unable to rebind xen VIRQ#%d: %d", isrc->xi_virq, error);

	isrc->xi_port = bind_virq.port;
	isrc->xi_cpu = 0;
	ukern_intr_port_to_isrc[bind_virq.port] = isrc;

#ifdef SMP
	error = ukern_intr_assign_cpu(&isrc->xi_intsrc,
	                            cpu_apic_ids[cpu]);
	if (error)
		panic("unable to bind xen VIRQ#%d to CPU#%d: %d",
		      isrc->xi_virq, cpu, error);
#endif

	evtchn_unmask_port(bind_virq.port);
#endif
}

/**
 * Return this PIC to service after being suspended.
 */
static void
ukern_intr_resume(struct pic *unused, bool suspend_cancelled)
{
	struct pass_status_page *s = status_page;
	struct ukernisrc *isrc;
	u_int isrc_idx;
	int i;

	if (suspend_cancelled)
		return;

	/* Reset the per-CPU masks */
	CPU_FOREACH(i) {
		struct ukern_intr_pcpu_data *pcpu;

		pcpu = DPCPU_ID_PTR(i, ukern_intr_pcpu);
		memset(pcpu->evtchn_enabled,
		       i == 0 ? ~0 : 0, sizeof(pcpu->evtchn_enabled));
	}

	/* Mask all event channels. */
	for (i = 0; i < nitems(s->evtchn_mask); i++)
		atomic_store_rel_long(&s->evtchn_mask[i], ~0);

	/* Remove port -> isrc mappings */
	memset(ukern_intr_port_to_isrc, 0, sizeof(ukern_intr_port_to_isrc));

	/* Free unused isrcs and rebind VIRQs and IPIs */
	for (isrc_idx = 0; isrc_idx < ukern_intr_auto_vector_count; isrc_idx++) {
		u_int vector;

		vector = FIRST_EVTCHN_INT + isrc_idx;
		isrc = (struct ukernisrc *)intr_lookup_source(vector);
		if (isrc != NULL) {
			isrc->xi_port = 0;
			switch (isrc->xi_type) {
			case EVTCHN_TYPE_IPI:
				xen_rebind_ipi(isrc);
				break;
			case EVTCHN_TYPE_VIRQ:
				xen_rebind_virq(isrc);
				break;
			default:
				isrc->xi_cpu = 0;
				break;
			}
		}
	}
}

/**
 * Disable a Xen interrupt source.
 *
 * \param isrc  The interrupt source to disable.
 */
static void
ukern_intr_disable_intr(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc = (struct ukernisrc *)base_isrc;

	evtchn_mask_port(isrc->xi_port);
}

/**
 * Determine the global interrupt vector number for
 * a Xen interrupt source.
 *
 * \param isrc  The interrupt source to query.
 *
 * \return  The vector number corresponding to the given interrupt source.
 */
static int
ukern_intr_vector(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc = (struct ukernisrc *)base_isrc;

	return (isrc->xi_vector);
}

/**
 * Determine whether or not interrupt events are pending on the
 * the given interrupt source.
 *
 * \param isrc  The interrupt source to query.
 *
 * \returns  0 if no events are pending, otherwise non-zero.
 */
static int
ukern_intr_source_pending(struct intsrc *isrc)
{
	/*
	 * EventChannels are edge triggered and never masked.
	 * There can be no pending events.
	 */
	return (0);
}

/**
 * Perform configuration of an interrupt source.
 *
 * \param isrc  The interrupt source to configure.
 * \param trig  Edge or level.
 * \param pol   Active high or low.
 *
 * \returns  0 if no events are pending, otherwise non-zero.
 */
static int
ukern_intr_config_intr(struct intsrc *isrc, enum intr_trigger trig,
    enum intr_polarity pol)
{
	/* Configuration is only possible via the evtchn apis. */
	return (ENODEV);
}

/**
 * Configure CPU affinity for interrupt source event delivery.
 *
 * \param isrc     The interrupt source to configure.
 * \param apic_id  The apic id of the CPU for handling future events.
 *
 * \returns  0 if successful, otherwise an errno.
 */
static int
ukern_intr_assign_cpu(struct intsrc *base_isrc, u_int apic_id)
{
#if 0 && defined(SMP)
	struct evtchn_bind_vcpu bind_vcpu;
	struct ukernisrc *isrc;
	u_int to_cpu, vcpu_id;
	int error;

#ifdef XENHVM
	if (xen_vector_callback_enabled == 0)
		return (EOPNOTSUPP);
#endif

	to_cpu = apic_cpuid(apic_id);
	vcpu_id = pcpu_find(to_cpu)->pc_vcpu_id;
	ukern_intr_intrcnt_add(to_cpu);

	mtx_lock(&ukern_intr_isrc_lock);
	isrc = (struct ukernisrc *)base_isrc;
	if (!is_valid_evtchn(isrc->xi_port)) {
		mtx_unlock(&ukern_intr_isrc_lock);
		return (EINVAL);
	}

	if ((isrc->xi_type == EVTCHN_TYPE_VIRQ) ||
		(isrc->xi_type == EVTCHN_TYPE_IPI)) {
		/*
		 * Virtual IRQs are associated with a cpu by
		 * the Hypervisor at evtchn_bind_virq time, so
		 * all we need to do is update the per-CPU masks.
		 */
		evtchn_cpu_mask_port(isrc->xi_cpu, isrc->xi_port);
		isrc->xi_cpu = to_cpu;
		evtchn_cpu_unmask_port(isrc->xi_cpu, isrc->xi_port);
		mtx_unlock(&ukern_intr_isrc_lock);
		return (0);
	}

	bind_vcpu.port = isrc->xi_port;
	bind_vcpu.vcpu = vcpu_id;

	/*
	 * Allow interrupts to be fielded on the new VCPU before
	 * we ask the hypervisor to deliver them there.
	 */
	evtchn_cpu_unmask_port(to_cpu, isrc->xi_port);
	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_vcpu, &bind_vcpu);
	if (isrc->xi_cpu != to_cpu) {
		if (error == 0) {
			/* Commit to new binding by removing the old one. */
			evtchn_cpu_mask_port(isrc->xi_cpu, isrc->xi_port);
			isrc->xi_cpu = to_cpu;
		} else {
			/* Roll-back to previous binding. */
			evtchn_cpu_mask_port(to_cpu, isrc->xi_port);
		}
	}
	mtx_unlock(&ukern_intr_isrc_lock);
	return (0);
#else
	return (EOPNOTSUPP);
#endif
}

/*------------------- Virtual Interrupt Source PIC Functions -----------------*/
/*
 * Mask a level triggered interrupt source.
 *
 * \param isrc  The interrupt source to mask (if necessary).
 * \param eoi   If non-zero, perform any necessary end-of-interrupt
 *              acknowledgements.
 */
static void
ukern_intr_disable_source(struct intsrc *isrc, int eoi)
{
}

/*
 * Unmask a level triggered interrupt source.
 *
 * \param isrc  The interrupt source to unmask (if necessary).
 */
static void
ukern_intr_enable_source(struct intsrc *isrc)
{
}

/*
 * Perform any necessary end-of-interrupt acknowledgements.
 *
 * \param isrc  The interrupt source to EOI.
 */
static void
ukern_intr_eoi_source(struct intsrc *isrc)
{
}

/*
 * Enable and unmask the interrupt source.
 *
 * \param isrc  The interrupt source to enable.
 */
static void
ukern_intr_enable_intr(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc = (struct ukernisrc *)base_isrc;

	evtchn_unmask_port(isrc->xi_port);
}

/*------------------ Physical Interrupt Source PIC Functions -----------------*/
/*
 * Mask a level triggered interrupt source.
 *
 * \param isrc  The interrupt source to mask (if necessary).
 * \param eoi   If non-zero, perform any necessary end-of-interrupt
 *              acknowledgements.
 */
static void
ukern_intr_pirq_disable_source(struct intsrc *base_isrc, int eoi)
{
	struct ukernisrc *isrc;

	isrc = (struct ukernisrc *)base_isrc;
	evtchn_mask_port(isrc->xi_port);

	if (eoi == PIC_EOI)
		ukern_intr_pirq_eoi_source(base_isrc);
}

/*
 * Unmask a level triggered interrupt source.
 *
 * \param isrc  The interrupt source to unmask (if necessary).
 */
static void
ukern_intr_pirq_enable_source(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc;

	isrc = (struct ukernisrc *)base_isrc;
	evtchn_unmask_port(isrc->xi_port);
#ifdef notyet	
	pci_pass_apic_enable(isrc->xi_port);
#endif	
}

/*
 * Perform any necessary end-of-interrupt acknowledgements.
 *
 * \param isrc  The interrupt source to EOI.
 */
static void
ukern_intr_pirq_eoi_source(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc;

	/* XXX Use shared page of flags for this. */
	isrc = (struct ukernisrc *)base_isrc;
	if (test_bit(isrc->xi_pirq, ukern_intr_pirq_eoi_map)) {
		/* NOP */
	}
}

/*
 * Enable and unmask the interrupt source.
 *
 * \param isrc  The interrupt source to enable.
 */
static void
ukern_intr_pirq_enable_intr(struct intsrc *base_isrc)
{
#if 0	
	struct ukernisrc *isrc;
	struct evtchn_bind_pirq bind_pirq;
	struct physdev_irq_status_query irq_status;
	int error;

	isrc = (struct ukernisrc *)base_isrc;

	if (!ukern_intr_pirq_eoi_map_enabled) {
		irq_status.irq = isrc->xi_pirq;
		error = HYPERVISOR_physdev_op(PHYSDEVOP_irq_status_query,
		    &irq_status);
		if (error)
			panic("unable to get status of IRQ#%d", isrc->xi_pirq);

		if (irq_status.flags & XENIRQSTAT_needs_eoi) {
			/*
			 * Since the dynamic PIRQ EOI map is not available
			 * mark the PIRQ as needing EOI unconditionally.
			 */
			set_bit(isrc->xi_pirq, ukern_intr_pirq_eoi_map);
		}
	}

	bind_pirq.pirq = isrc->xi_pirq;
	bind_pirq.flags = isrc->xi_edgetrigger ? 0 : BIND_PIRQ__WILL_SHARE;
	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_pirq, &bind_pirq);
	if (error)
		panic("unable to bind IRQ#%d", isrc->xi_pirq);

	isrc->xi_port = bind_pirq.port;

	mtx_lock(&ukern_intr_isrc_lock);
	KASSERT((ukern_intr_port_to_isrc[bind_pirq.port] == NULL),
	    ("trying to override an already setup event channel port"));
	ukern_intr_port_to_isrc[bind_pirq.port] = isrc;
	mtx_unlock(&ukern_intr_isrc_lock);

	evtchn_unmask_port(isrc->xi_port);
#endif	
}

/*
 * Disable an interrupt source.
 *
 * \param isrc  The interrupt source to disable.
 */
static void
ukern_intr_pirq_disable_intr(struct intsrc *base_isrc)
{
	struct ukernisrc *isrc;
	int error;

	isrc = (struct ukernisrc *)base_isrc;

	evtchn_mask_port(isrc->xi_port);

#ifdef notyet	
	close.port = isrc->xi_port;
	error = HYPERVISOR_event_channel_op(EVTCHNOP_close, &close);
#endif	
	if (error)
		panic("unable to close event channel %d IRQ#%d",
		    isrc->xi_port, isrc->xi_pirq);

	mtx_lock(&ukern_intr_isrc_lock);
	ukern_intr_port_to_isrc[isrc->xi_port] = NULL;
	mtx_unlock(&ukern_intr_isrc_lock);

	isrc->xi_port = 0;
}

/**
 * Perform configuration of an interrupt source.
 *
 * \param isrc  The interrupt source to configure.
 * \param trig  Edge or level.
 * \param pol   Active high or low.
 *
 * \returns  0 if no events are pending, otherwise non-zero.
 */
static int
ukern_intr_pirq_config_intr(struct intsrc *base_isrc, enum intr_trigger trig,
    enum intr_polarity pol)
{
        struct ukernisrc *isrc = (struct ukernisrc *)base_isrc;
        isrc->xi_activehi = pol == INTR_POLARITY_HIGH ? 1 : 0;
        isrc->xi_edgetrigger = trig == INTR_TRIGGER_EDGE ? 1 : 0;

        return (0);
}


/*--------------------------- Public Functions -------------------------------*/
/*------- API comments for these methods can be found in ukern_intr.h -------*/

int
ukern_intr_register(enum evtchn_type type, int vector)
{
	struct ukernisrc *isrc;

	mtx_lock(&ukern_intr_isrc_lock);
	isrc = ukern_intr_alloc_isrc(type, vector);
	mtx_unlock(&ukern_intr_isrc_lock);

	if (isrc == NULL)
		return (ENOENT);
	return (0);
}

int
ukern_intr_bind(const char *name, int vector, driver_filter_t filter,
				driver_intr_t handler, void *arg, enum intr_type flags,
				void **cookiep, int cpu)
{
	int rc;

	rc = intr_add_handler(name, vector, filter, handler, arg, flags, cookiep);
	if (rc)
		return (rc);
	if (cpu != -1 && cpu < mp_ncpus)
		rc = intr_bind(vector, cpu);

	evtchn_unmask_port(vector);
	return (rc);
}

int
ukern_intr_pirq_setup(device_t dev, int vector, void **tag)
{
	int fd;
	struct pci_pass_setup_intr ppsi;

	ppsi.ppsi_trap = NULL;
	ppsi.ppsi_vcpuid = 0;
	ppsi.ppsi_vector = vector;
	if ((fd = device_get_fd(dev)) < 0)
		return (ENOENT);
	if ((fd = ioctl(fd, PCIPASSIOCSETUPINTR, &ppsi)) < 0)
		return (*__error());
	
	*tag = ppsi.ppsi_tag;
	return (0);
}
#if 0
int
ukern_intr_bind_local_port(device_t dev, evtchn_port_t local_port,
    driver_filter_t filter, driver_intr_t handler, void *arg,
    enum intr_type flags, ukern_intr_handle_t *port_handlep)
{
	struct ukernisrc *isrc;
	int error;

	error = ukern_intr_bind_isrc(&isrc, local_port, EVTCHN_TYPE_PORT, dev,
		    filter, handler, arg, flags, port_handlep);
	if (error != 0)
		return (error);

	/*
	 * The Event Channel API didn't open this port, so it is not
	 * responsible for closing it automatically on unbind.
	 */
	isrc->xi_close = 0;
	return (0);
}

int 
ukern_intr_bind_virq(device_t dev, u_int virq, u_int cpu,
    driver_filter_t filter, driver_intr_t handler, void *arg,
    enum intr_type flags, ukern_intr_handle_t *port_handlep)
{

	int vcpu_id = pcpu_find(cpu)->pc_vcpu_id;
	struct ukernisrc *isrc;
	struct evtchn_bind_virq bind_virq = { .virq = virq, .vcpu = vcpu_id };
	int error;

	/* Ensure the target CPU is ready to handle evtchn interrupts. */
	ukern_intr_intrcnt_add(cpu);

	isrc = NULL;
	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_virq, &bind_virq);
	if (error != 0) {
		/*
		 * XXX Trap Hypercall error code Linuxisms in
		 *     the HYPERCALL layer.
		 */
		return (-error);
	}

	error = ukern_intr_bind_isrc(&isrc, bind_virq.port, EVTCHN_TYPE_VIRQ, dev,
				 filter, handler, arg, flags, port_handlep);

#ifdef SMP
	if (error == 0)
		error = intr_event_bind(isrc->xi_intsrc.is_event, cpu);
#endif

	if (error != 0) {
		evtchn_close_t close = { .port = bind_virq.port };

		ukern_intr_unbind(*port_handlep);
		if (HYPERVISOR_event_channel_op(EVTCHNOP_close, &close))
			panic("EVTCHNOP_close failed");
		return (error);
	}

#ifdef SMP
	if (isrc->xi_cpu != cpu) {
		/*
		 * Too early in the boot process for the generic interrupt
		 * code to perform the binding.  Update our event channel
		 * masks manually so events can't fire on the wrong cpu
		 * during AP startup.
		 */
		ukern_intr_assign_cpu(&isrc->xi_intsrc, cpu_apic_ids[cpu]);
	}
#endif

	/*
	 * The Event Channel API opened this port, so it is
	 * responsible for closing it automatically on unbind.
	 */
	isrc->xi_close = 1;
	isrc->xi_virq = virq;

	return (0);
}

int
ukern_intr_alloc_and_bind_ipi(device_t dev, u_int cpu,
    driver_filter_t filter, enum intr_type flags,
    ukern_intr_handle_t *port_handlep)
{
#ifdef SMP
	int vcpu_id = pcpu_find(cpu)->pc_vcpu_id;
	struct ukernisrc *isrc;
	struct evtchn_bind_ipi bind_ipi = { .vcpu = vcpu_id };
	int error;

	/* Ensure the target CPU is ready to handle evtchn interrupts. */
	ukern_intr_intrcnt_add(cpu);

	isrc = NULL;
	error = HYPERVISOR_event_channel_op(EVTCHNOP_bind_ipi, &bind_ipi);
	if (error != 0) {
		/*
		 * XXX Trap Hypercall error code Linuxisms in
		 *     the HYPERCALL layer.
		 */
		return (-error);
	}

	error = ukern_intr_bind_isrc(&isrc, bind_ipi.port, EVTCHN_TYPE_IPI,
	                           dev, filter, NULL, NULL, flags,
	                           port_handlep);
	if (error == 0)
		error = intr_event_bind(isrc->xi_intsrc.is_event, cpu);

	if (error != 0) {
		evtchn_close_t close = { .port = bind_ipi.port };

		ukern_intr_unbind(*port_handlep);
		if (HYPERVISOR_event_channel_op(EVTCHNOP_close, &close))
			panic("EVTCHNOP_close failed");
		return (error);
	}

	if (isrc->xi_cpu != cpu) {
		/*
		 * Too early in the boot process for the generic interrupt
		 * code to perform the binding.  Update our event channel
		 * masks manually so events can't fire on the wrong cpu
		 * during AP startup.
		 */
		ukern_intr_assign_cpu(&isrc->xi_intsrc, cpu_apic_ids[cpu]);
	}

	/*
	 * The Event Channel API opened this port, so it is
	 * responsible for closing it automatically on unbind.
	 */
	isrc->xi_close = 1;
	return (0);
#else
	return (EOPNOTSUPP);
#endif
}

int
xen_register_pirq(int vector, enum intr_trigger trig, enum intr_polarity pol)
{
	struct physdev_map_pirq map_pirq;
	struct physdev_irq alloc_pirq;
	struct ukernisrc *isrc;
	int error;

	if (vector == 0)
		return (EINVAL);

	if (bootverbose)
		printf("xen: register IRQ#%d\n", vector);

	map_pirq.domid = DOMID_SELF;
	map_pirq.type = MAP_PIRQ_TYPE_GSI;
	map_pirq.index = vector;
	map_pirq.pirq = vector;

	error = HYPERVISOR_physdev_op(PHYSDEVOP_map_pirq, &map_pirq);
	if (error) {
		printf("xen: unable to map IRQ#%d\n", vector);
		return (error);
	}

	alloc_pirq.irq = vector;
	alloc_pirq.vector = 0;
	error = HYPERVISOR_physdev_op(PHYSDEVOP_alloc_irq_vector, &alloc_pirq);
	if (error) {
		printf("xen: unable to alloc PIRQ for IRQ#%d\n", vector);
		return (error);
	}

	mtx_lock(&ukern_intr_isrc_lock);
	isrc = ukern_intr_alloc_isrc(EVTCHN_TYPE_PIRQ, vector);
	mtx_unlock(&ukern_intr_isrc_lock);
	KASSERT((isrc != NULL), ("xen: unable to allocate isrc for interrupt"));
	isrc->xi_pirq = vector;
	isrc->xi_activehi = pol == INTR_POLARITY_HIGH ? 1 : 0;
	isrc->xi_edgetrigger = trig == INTR_TRIGGER_EDGE ? 1 : 0;

	return (0);
}

int
xen_register_msi(device_t dev, int vector, int count)
{
	struct physdev_map_pirq msi_irq;
	struct ukernisrc *isrc;
	int ret;

	memset(&msi_irq, 0, sizeof(msi_irq));
	msi_irq.domid = DOMID_SELF;
	msi_irq.type = count == 1 ?
	    MAP_PIRQ_TYPE_MSI_SEG : MAP_PIRQ_TYPE_MULTI_MSI;
	msi_irq.index = -1;
	msi_irq.pirq = -1;
	msi_irq.bus = pci_get_bus(dev) | (pci_get_domain(dev) << 16);
	msi_irq.devfn = (pci_get_slot(dev) << 3) | pci_get_function(dev);
	msi_irq.entry_nr = count;

	ret = HYPERVISOR_physdev_op(PHYSDEVOP_map_pirq, &msi_irq);
	if (ret != 0)
		return (ret);
	if (count != msi_irq.entry_nr) {
		panic("unable to setup all requested MSI vectors "
		    "(expected %d got %d)", count, msi_irq.entry_nr);
	}

	mtx_lock(&ukern_intr_isrc_lock);
	for (int i = 0; i < count; i++) {
		isrc = ukern_intr_alloc_isrc(EVTCHN_TYPE_PIRQ, vector + i);
		KASSERT(isrc != NULL,
		    ("xen: unable to allocate isrc for interrupt"));
		isrc->xi_pirq = msi_irq.pirq + i;
	}
	mtx_unlock(&ukern_intr_isrc_lock);

	return (0);
}

int
xen_release_msi(int vector)
{
	struct physdev_unmap_pirq unmap;
	struct ukernisrc *isrc;
	int ret;

	isrc = (struct ukernisrc *)intr_lookup_source(vector);
	if (isrc == NULL)
		return (ENXIO);

	unmap.pirq = isrc->xi_pirq;
	ret = HYPERVISOR_physdev_op(PHYSDEVOP_unmap_pirq, &unmap);
	if (ret != 0)
		return (ret);

	ukern_intr_release_isrc(isrc);

	return (0);
}

int
ukern_intr_describe(ukern_intr_handle_t port_handle, const char *fmt, ...)
{
	char descr[MAXCOMLEN + 1];
	struct ukernisrc *isrc;
	va_list ap;

	isrc = ukern_intr_isrc(port_handle);
	if (isrc == NULL)
		return (EINVAL);

	va_start(ap, fmt);
	vsnprintf(descr, sizeof(descr), fmt, ap);
	va_end(ap);
	return (intr_describe(isrc->xi_vector, port_handle, descr));
}

void
ukern_intr_unbind(ukern_intr_handle_t *port_handlep)
{
	struct intr_handler *handler;
	struct ukernisrc *isrc;

	handler = *port_handlep;
	*port_handlep = NULL;
	isrc = ukern_intr_isrc(handler);
	if (isrc == NULL)
		return;

	intr_remove_handler(handler);
	ukern_intr_release_isrc(isrc);
}

void
ukern_intr_signal(ukern_intr_handle_t handle)
{
	struct ukernisrc *isrc;

	isrc = ukern_intr_isrc(handle);
	if (isrc != NULL) {
		KASSERT(isrc->xi_type == EVTCHN_TYPE_PORT ||
			isrc->xi_type == EVTCHN_TYPE_IPI,
			("evtchn_signal on something other than a local port"));
		struct evtchn_send send = { .port = isrc->xi_port };
		(void)HYPERVISOR_event_channel_op(EVTCHNOP_send, &send);
	}
}

evtchn_port_t
ukern_intr_port(ukern_intr_handle_t handle)
{
	struct ukernisrc *isrc;

	isrc = ukern_intr_isrc(handle);
	if (isrc == NULL)
		return (0);
	
	return (isrc->xi_port);
}
#endif
#ifdef DDB
static const char *
ukern_intr_print_type(enum evtchn_type type)
{
	static const char *evtchn_type_to_string[EVTCHN_TYPE_COUNT] = {
		[EVTCHN_TYPE_UNBOUND]	= "UNBOUND",
		[EVTCHN_TYPE_PIRQ]	= "PIRQ",
		[EVTCHN_TYPE_VIRQ]	= "VIRQ",
		[EVTCHN_TYPE_IPI]	= "IPI",
		[EVTCHN_TYPE_PORT]	= "PORT",
	};

	if (type >= EVTCHN_TYPE_COUNT)
		return ("UNKNOWN");

	return (evtchn_type_to_string[type]);
}

static void
ukern_intr_dump_port(struct ukernisrc *isrc)
{
	struct ukern_intr_pcpu_data *pcpu;
	struct pass_status_page *s = status_page;
	int i;

	db_printf("Port %d Type: %s\n",
	    isrc->xi_port, ukern_intr_print_type(isrc->xi_type));
	if (isrc->xi_type == EVTCHN_TYPE_PIRQ) {
		db_printf("\tPirq: %d ActiveHi: %d EdgeTrigger: %d "
		    "NeedsEOI: %d Shared: %d\n",
		    isrc->xi_pirq, isrc->xi_activehi, isrc->xi_edgetrigger,
		    !!test_bit(isrc->xi_pirq, ukern_intr_pirq_eoi_map),
		    isrc->xi_shared);
	}
	if (isrc->xi_type == EVTCHN_TYPE_VIRQ)
		db_printf("\tVirq: %d\n", isrc->xi_virq);

	db_printf("\tMasked: %d Pending: %d\n",
	    !!test_bit(isrc->xi_port, &s->evtchn_mask[0]),
	    !!test_bit(isrc->xi_port, &s->evtchn_pending[0]));

	db_printf("\tPer-CPU Masks: ");
	CPU_FOREACH(i) {
		pcpu = DPCPU_ID_PTR(i, ukern_intr_pcpu);
		db_printf("cpu#%d: %d ", i,
		    !!test_bit(isrc->xi_port, pcpu->evtchn_enabled));
	}
	db_printf("\n");
}

DB_SHOW_COMMAND(ukern_evtchn, db_show_ukern_evtchn)
{
	int i;

	for (i = 0; i < NR_EVENT_CHANNELS; i++) {
		struct ukernisrc *isrc;

		isrc = ukern_intr_port_to_isrc[i];
		if (isrc == NULL)
			continue;

		ukern_intr_dump_port(isrc);
	}
}
#endif /* DDB */
