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


#ifndef _SYS_PCI_PASS_H_
#define	_SYS_PCI_PASS_H_
#include <sys/ioccom.h>


#define PCI_PASS_INTR_STK_SIZE	3*PAGE_SIZE
#define PCI_PASS_MAX_VCPUS 32

#define PCI_PASS_C_TRAPFRAME 0x1
struct dev_pass_tidvcpumap {
	/* physical cpuid */
	uint8_t	dpt_cpuid;
	/* kernel thread id (thr_self()) */
	lwpid_t	dpt_tid;
	/* stack to handle traps on */
	caddr_t dpt_stk;
};

struct dev_pass_vcpumap {
	/* number of virtual CPUs, can be greater than physical
	 * threads but performance will suffer greatly
	 */
	uint32_t dpv_nvcpus;
	/* default trap handler */
	caddr_t dpv_trap;
	int dpv_flags;
	/* vcpus are assumed to be contiguous */
	struct dev_pass_tidvcpumap dpv_map[PCI_PASS_MAX_VCPUS];
};


struct dev_pass_timer {
	/* if NULL will use the default */
	caddr_t dpt_trap;
	/* requested frequency - actual frequency is returned
	 * cannot be higher than the frequency in the Ring 0
	 * kernel
	 */
	uint32_t dpt_hz;
	/* return result is the vector for the timer interrupt */
	uint32_t dpt_vector;
};

/* check for presence of an IRQ vector */
#define	DEVPASSIOCCHKIRQ		_IOWR('y', 1, int)
/* Map a kernel irq status page */
#define	DEVPASSIOCSTATUSPAGE	_IOWR('y', 2, caddr_t)
/* get system call number for re-enabling the apic */
#define	DEVPASSIOCSYS			_IOWR('y', 3, int)
/* bind the corresponding threads to the listed physical cores
 * and track the mapping for interrupt scheduling
 */
#define	DEVPASSIOCVCPUMAP		_IOWR('y', 4, struct dev_pass_vcpumap)
/*
 * Install timer at frequency hz
 */
#define DEVPASSIOCTIMER			_IOWR('y', 5, struct dev_pass_timer)
/*
 * Install executable page for trap handling glue
 */
#define	DEVPASSIOCCODEPAGE		_IOWR('y', 6, caddr_t)

struct pci_pass_setup_intr {
	void	*ppsi_trap;
	int		ppsi_vcpuid;

	int		ppsi_vector;
	void	*ppsi_tag;
	void	*ppsi_cookie;
};

struct pci_pass_teardown_intr {
	void	*ppti_tag;
};

struct pci_pass_post_filter {
	int pppf_write_reg;
	int pppf_write_val;
};


#define PCIPASSIOCSETUPINTR    _IOWR('y', 11, struct pci_pass_setup_intr)
#define PCIPASSIOCTEARDOWNINTR _IOWR('y', 12, void *)
#define PCIPASSIOCPOSTFILTER   _IOWR('y', 13, struct pci_pass_post_filter)

#define PCI_PASS_APIC_ENABLE 1
#define PCI_PASS_IPI		 2

struct ppae_args {
	void *cookie;
	int vector;
};

struct ppi_args {
	int vcpuid;
	int vector;
};

struct dps_args {
	int sycall;
	union {
		struct ppae_args ppae;
		struct ppi_args ppi;
	} u;
};

/* The interrupt handling / masking problem has already been tackled by Xen.
 * Here we (mostly) re-use their structures. The time is already updated in
 * the system-wide shared page and the architecture specific information
 * doesn't apply.
 */


/******************************************************************************
 * xen.h
 * 
 * Guest OS interface to Xen.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (c) 2004, K A Fraser
 */

struct pass_vcpu_info {
    /*
     * 'evtchn_upcall_pending' is written non-zero by Xen to indicate
     * a pending notification for a particular VCPU. It is then cleared
     * by the guest OS /before/ checking for pending work, thus avoiding
     * a set-and-check race. Note that the mask is only accessed by Xen
     * on the CPU that is currently hosting the VCPU. This means that the
     * pending and mask flags can be updated by the guest without special
     * synchronisation (i.e., no need for the x86 LOCK prefix).
     * This may seem suboptimal because if the pending flag is set by
     * a different CPU then an IPI may be scheduled even when the mask
     * is set. However, note:
     *  1. The task of 'interrupt holdoff' is covered by the per-event-
     *     channel mask bits. A 'noisy' event that is continually being
     *     triggered can be masked at source at this very precise
     *     granularity.
     *  2. The main purpose of the per-VCPU mask is therefore to restrict
     *     reentrant execution: whether for concurrency control, or to
     *     prevent unbounded stack usage. Whatever the purpose, we expect
     *     that the mask will be asserted only for short periods at a time,
     *     and so the likelihood of a 'spurious' IPI is suitably small.
     * The mask is read before making an event upcall to the guest: a
     * non-zero mask therefore guarantees that the VCPU will not receive
     * an upcall activation. The mask is cleared when the VCPU requests
     * to block: this avoids wakeup-waiting races.
     */
    uint8_t evtchn_upcall_pending;
    uint8_t evtchn_upcall_mask;
	/* The bits set are indices in to the global evtchn_pending */
    unsigned long evtchn_pending_sel;
}; /* 64 bytes (x86) */


struct pass_status_page {
    struct pass_vcpu_info vcpu_info[PCI_PASS_MAX_VCPUS];

    /*
     * A domain can create "event channels" on which it can send and receive
     * asynchronous event notifications. There are three classes of event that
     * are delivered by this mechanism:
     *  1. Bi-directional inter- and intra-domain connections. Domains must
     *     arrange out-of-band to set up a connection (usually by allocating
     *     an unbound 'listener' port and avertising that via a storage service
     *     such as xenstore).
     *  2. Physical interrupts. A domain with suitable hardware-access
     *     privileges can bind an event-channel port to a physical interrupt
     *     source.
     *  3. Virtual interrupts ('events'). A domain can bind an event-channel
     *     port to a virtual interrupt source, such as the virtual-timer
     *     device or the emergency console.
     *
     * Event channels are addressed by a "port index". Each channel is
     * associated with two bits of information:
     *  1. PENDING -- notifies the domain that there is a pending notification
     *     to be processed. This bit is cleared by the guest.
     *  2. MASK -- if this bit is clear then a 0->1 transition of PENDING
     *     will cause an asynchronous upcall to be scheduled. This bit is only
     *     updated by the guest. It is read-only within Xen. If a channel
     *     becomes pending while the channel is masked then the 'edge' is lost
     *     (i.e., when the channel is unmasked, the guest must manually handle
     *     pending notifications as no upcall will be scheduled by Xen).
     *
     * To expedite scanning of pending notifications, any 0->1 pending
     * transition on an unmasked channel causes a corresponding bit in a
     * per-vcpu selector word to be set. Each bit in the selector covers a
     * 'C long' in the PENDING bitfield array.
     */
    unsigned long evtchn_pending[sizeof(unsigned long) * 8];
    unsigned long evtchn_mask[sizeof(unsigned long) * 8];
};
#endif
