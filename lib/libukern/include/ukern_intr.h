/******************************************************************************
 * ukern_intr.h
 * 
 * APIs for managing Xen event channel, virtual IRQ, and physical IRQ
 * notifications.
 * 
 * Copyright (c) 2004, K A Fraser
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
 *
 * $FreeBSD$
 */
#ifndef _UKERN_INTR_H_
#define _UKERN_INTR_H_
#include <machine/xen/synch_bitops.h>
typedef uint32_t evtchn_port_t;


/* XXX this doesn't belong here */
#ifdef __amd64__
/* these don't look like but check later */
#define LOCK_PREFIX ""
#define LOCK ""
#define ADDR (*(volatile long *) addr)

static __inline int constant_test_bit(int nr, const volatile void * addr)
{
    return ((1UL << (nr & 31)) & (((const volatile unsigned int *) addr)[nr >> 5])) != 0;
}

static __inline int variable_test_bit(int nr, volatile void * addr)
{
    int oldbit;
    
    __asm__ __volatile__(
        "btl %2,%1\n\tsbbl %0,%0"
        :"=r" (oldbit)
        :"m" (ADDR),"Ir" (nr));
    return oldbit;
}

#define test_bit(nr,addr) \
(__builtin_constant_p(nr) ? \
 constant_test_bit((nr),(addr)) : \
 variable_test_bit((nr),(addr)))

/**
 * set_bit - Atomically set a bit in memory
 * @nr: the bit to set
 * @addr: the address to start counting from
 *
 * This function is atomic and may not be reordered.  See __set_bit()
 * if you do not require the atomic guarantees.
 * Note that @nr may be almost arbitrarily large; this function is not
 * restricted to acting on a single-word quantity.
 */
static __inline__ void set_bit(int nr, volatile void * addr)
{
        __asm__ __volatile__( LOCK_PREFIX
                "btsl %1,%0"
                :"=m" (ADDR)
                :"Ir" (nr));
}

/**
 * clear_bit - Clears a bit in memory
 * @nr: Bit to clear
 * @addr: Address to start counting from
 *
 * clear_bit() is atomic and may not be reordered.  However, it does
 * not contain a memory barrier, so if it is used for locking purposes,
 * you should call smp_mb__before_clear_bit() and/or smp_mb__after_clear_bit()
 * in order to ensure changes are visible on other processors.
 */
static __inline__ void clear_bit(int nr, volatile void * addr)
{
        __asm__ __volatile__( LOCK_PREFIX
                "btrl %1,%0"
                :"=m" (ADDR)
                :"Ir" (nr));
}
#endif

#define NR_EVENT_CHANNELS (sizeof(unsigned long) * sizeof(unsigned long) * 64)
enum evtchn_type {
	EVTCHN_TYPE_UNBOUND,
	EVTCHN_TYPE_PIRQ,
	EVTCHN_TYPE_VIRQ,
	EVTCHN_TYPE_IPI,
	EVTCHN_TYPE_PORT,
	EVTCHN_TYPE_COUNT
};
#define	NUM_MSI_INTS	512
#define	FIRST_MSI_INT	256
#define	FIRST_EVTCHN_INT (FIRST_MSI_INT + NUM_MSI_INTS)


/**
 * Disable signal delivery for an event channel port, returning its
 * previous mask state.
 *
 * \param port  The event channel port to query and mask.
 *
 * \returns  1 if event delivery was previously disabled.  Otherwise 0.
 */
static inline int
evtchn_test_and_set_mask(evtchn_port_t port)
{
	struct pass_status_page *s = status_page;

	return synch_test_and_set_bit(port, s->evtchn_mask);
}

/**
 * Clear any pending event for the given event channel port.
 *
 * \param port  The event channel port to clear.
 */
static inline void 
evtchn_clear_port(evtchn_port_t port)
{	
	struct pass_status_page *s = status_page;

	synch_clear_bit(port, &s->evtchn_pending[0]);
}

/**
 * Disable signal delivery for an event channel port.
 *
 * \param port  The event channel port to mask.
 */
static inline void
evtchn_mask_port(evtchn_port_t port)
{
	struct pass_status_page *s = status_page;

	synch_set_bit(port, &s->evtchn_mask[0]);
}

/**
 * Enable signal delivery for an event channel port.
 *
 * \param port  The event channel port to enable.
 */
static inline void
evtchn_unmask_port(evtchn_port_t port)
{
	struct pass_status_page *s = status_page;

	synch_clear_bit(port, &s->evtchn_mask[0]);
}


int ukern_intr_register(enum evtchn_type type, int vector);
int ukern_intr_bind(const char *name, int vector, driver_filter_t filter,
					driver_intr_t handler, void *arg, enum intr_type flags,
					void **cookiep, int cpu);
int ukern_intr_pirq_setup(device_t dev, int vector, void **tag);
#endif /* _UKERN_INTR_H_ */
