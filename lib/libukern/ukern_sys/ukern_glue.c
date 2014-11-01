/*-
 * Copyright (c) 2014 Matthew Macy <kmacy@freebsd.org>
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
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <sys/event.h>
#include <sys/jail.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/refcount.h>
#include <sys/resourcevar.h>
#include <sys/sysctl.h>
#include <sys/sysent.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/priv.h>
#include <sys/time.h>
#include <sys/ucred.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_object.h>
#include <vm/vm_map.h>
#include <vm/vm_extern.h>

#include <time.h>

int bootverbose;
vm_paddr_t lapic_paddr;

SYSCTL_STRING(_kern, OID_AUTO, compiler_version, CTLFLAG_RD|CTLFLAG_MPSAFE,
    compiler_version, 0, "Version of compiler used to compile kernel");

static MALLOC_DEFINE(M_CRED, "cred", "credentials");
static MALLOC_DEFINE(M_PLIMIT, "plimit", "plimit structures");
MALLOC_DEFINE(M_PARGS, "proc-args", "Process arguments");

extern void abort(void);
volatile int	ticks;
int	cpu_disable_deep_sleep;
u_long ps_arg_cache_limit = PAGE_SIZE / 16;

/* This is used in modules that need to work in both SMP and UP. */
cpuset_t all_cpus;

int workaround_erratum383;

u_int mp_maxid;

long first_page = 0;

struct vmmeter cnt;

struct vm_object kernel_object_store;
struct vm_object kmem_object_store;

struct filterops fs_filtops;
struct filterops sig_filtops;


/*
 * Variable panicstr contains argument to first call to panic; used as flag
 * to indicate that the kernel has already called panic.
 */
const char *panicstr;

struct pidhashhead *pidhashtbl;
u_long pidhash;
struct pgrphashhead *pgrphashtbl;
u_long pgrphash;
int cold;
int kdb_active;

#if 0
int
prison_if(struct ucred *cred, struct sockaddr *sa)
{

	return (0);
}

int
prison_check_af(struct ucred *cred, int af)
{

	return (0);
}

int
prison_check_ip4(const struct ucred *cred, const struct in_addr *ia)
{

	return (0);
}


int
prison_equal_ip4(struct prison *pr1, struct prison *pr2)
{

	return (1);
}

int
prison_check_ip6(struct ucred *cred, struct in6_addr *ia)
{

	return (0);
}

int
prison_equal_ip6(struct prison *pr1, struct prison *pr2)
{

	return (1);
}

/*
 * See if a prison has the specific flag set.
 */
int
prison_flag(struct ucred *cred, unsigned flag)
{

	/* This is an atomic read, so no locking is necessary. */
	return (flag & PR_HOST);
}

int
prison_get_ip4(struct ucred *cred, struct in_addr *ia)
{

	return (0);
}

int
prison_local_ip4(struct ucred *cred, struct in_addr *ia)
{

	return (0);
}

int
prison_remote_ip4(struct ucred *cred, struct in_addr *ia)
{

	return (0);
}

int
prison_get_ip6(struct ucred *cred, struct in6_addr *ia)
{

	return (0);
}

int
prison_local_ip6(struct ucred *cred, struct in6_addr *ia, int other)
{

	return (0);
}

int
prison_remote_ip6(struct ucred *cred, struct in6_addr *ia)
{

	return (0);
}

int 
prison_saddrsel_ip4(struct ucred *cred, struct in_addr *ia)
{

	/* not jailed */
	return (1);
}

int 
prison_saddrsel_ip6(struct ucred *cred, struct in6_addr *ia)
{

	/* not jailed */
	return (1);

}

struct prison *
prison_find_child(struct prison *mypr, int prid)
{
	return (NULL);
}

int
jailed(struct ucred *cred)
{

	return (0);
}

/*
 * Return 1 if the passed credential is in a jail and that jail does not
 * have its own virtual network stack, otherwise 0.
 */
int
jailed_without_vnet(struct ucred *cred)
{

	return (0);
}
#endif
int
priv_check(struct thread *td, int priv)
{

	return (0);
}

int
priv_check_cred(struct ucred *cred, int priv, int flags)
{

	return (0);
}


#if 0
int
vslock(void *addr, size_t len)
{

	return (0);
}

void
vsunlock(void *addr, size_t len)
{

}


/*
 * Check that a proposed value to load into the .it_value or
 * .it_interval part of an interval timer is acceptable, and
 * fix it to have at least minimal value (i.e. if it is less
 * than the resolution of the clock, round it up.)
 */
int
itimerfix(struct timeval *tv)
{

	if (tv->tv_sec < 0 || tv->tv_usec < 0 || tv->tv_usec >= 1000000)
		return (EINVAL);
	if (tv->tv_sec == 0 && tv->tv_usec != 0 && tv->tv_usec < tick)
		tv->tv_usec = tick;
	return (0);
}

/*
 * Decrement an interval timer by a specified number
 * of microseconds, which must be less than a second,
 * i.e. < 1000000.  If the timer expires, then reload
 * it.  In this case, carry over (usec - old value) to
 * reduce the value reloaded into the timer so that
 * the timer does not drift.  This routine assumes
 * that it is called in a context where the timers
 * on which it is operating cannot change in value.
 */
int
itimerdecr(struct itimerval *itp, int usec)
{

	if (itp->it_value.tv_usec < usec) {
		if (itp->it_value.tv_sec == 0) {
			/* expired, and already in next interval */
			usec -= itp->it_value.tv_usec;
			goto expire;
		}
		itp->it_value.tv_usec += 1000000;
		itp->it_value.tv_sec--;
	}
	itp->it_value.tv_usec -= usec;
	usec = 0;
	if (timevalisset(&itp->it_value))
		return (1);
	/* expired, exactly at end of interval */
expire:
	if (timevalisset(&itp->it_interval)) {
		itp->it_value = itp->it_interval;
		itp->it_value.tv_usec -= usec;
		if (itp->it_value.tv_usec < 0) {
			itp->it_value.tv_usec += 1000000;
			itp->it_value.tv_sec--;
		}
	} else
		itp->it_value.tv_usec = 0;		/* sec is already 0 */
	return (0);
}

static void
timevalfix(struct timeval *t1)
{

	if (t1->tv_usec < 0) {
		t1->tv_sec--;
		t1->tv_usec += 1000000;
	}
	if (t1->tv_usec >= 1000000) {
		t1->tv_sec++;
		t1->tv_usec -= 1000000;
	}
}

/*
 * Add and subtract routines for timevals.
 * N.B.: subtract routine doesn't deal with
 * results which are before the beginning,
 * it just gets very confused in this case.
 * Caveat emptor.
 */
void
timevaladd(struct timeval *t1, const struct timeval *t2)
{

	t1->tv_sec += t2->tv_sec;
	t1->tv_usec += t2->tv_usec;
	timevalfix(t1);
}

void
timevalsub(struct timeval *t1, const struct timeval *t2)
{

	t1->tv_sec -= t2->tv_sec;
	t1->tv_usec -= t2->tv_usec;
	timevalfix(t1);
}
/*
 * ratecheck(): simple time-based rate-limit checking.
 */
int
ratecheck(struct timeval *lasttime, const struct timeval *mininterval)
{
	struct timeval tv, delta;
	int rv = 0;

	getmicrouptime(&tv);		/* NB: 10ms precision */
	delta = tv;
	timevalsub(&delta, lasttime);

	/*
	 * check for 0,0 is so that the message will be seen at least once,
	 * even if interval is huge.
	 */
	if (timevalcmp(&delta, mininterval, >=) ||
	    (lasttime->tv_sec == 0 && lasttime->tv_usec == 0)) {
		*lasttime = tv;
		rv = 1;
	}

	return (rv);
}

/*
 * ppsratecheck(): packets (or events) per second limitation.
 *
 * Return 0 if the limit is to be enforced (e.g. the caller
 * should drop a packet because of the rate limitation).
 *
 * maxpps of 0 always causes zero to be returned.  maxpps of -1
 * always causes 1 to be returned; this effectively defeats rate
 * limiting.
 *
 * Note that we maintain the struct timeval for compatibility
 * with other bsd systems.  We reuse the storage and just monitor
 * clock ticks for minimal overhead.  
 */
int
ppsratecheck(struct timeval *lasttime, int *curpps, int maxpps)
{
	int now;

	/*
	 * Reset the last time and counter if this is the first call
	 * or more than a second has passed since the last update of
	 * lasttime.
	 */
	now = ticks;
	if (lasttime->tv_sec == 0 || (u_int)(now - lasttime->tv_sec) >= hz) {
		lasttime->tv_sec = now;
		*curpps = 1;
		return (maxpps != 0);
	} else {
		(*curpps)++;		/* NB: ignore potential overflow */
		return (maxpps < 0 || *curpps < maxpps);
	}
}

/*
 * Compute number of ticks in the specified amount of time.
 */
int
tvtohz(tv)
	struct timeval *tv;
{
	register unsigned long ticks;
	register long sec, usec;

	/*
	 * If the number of usecs in the whole seconds part of the time
	 * difference fits in a long, then the total number of usecs will
	 * fit in an unsigned long.  Compute the total and convert it to
	 * ticks, rounding up and adding 1 to allow for the current tick
	 * to expire.  Rounding also depends on unsigned long arithmetic
	 * to avoid overflow.
	 *
	 * Otherwise, if the number of ticks in the whole seconds part of
	 * the time difference fits in a long, then convert the parts to
	 * ticks separately and add, using similar rounding methods and
	 * overflow avoidance.  This method would work in the previous
	 * case but it is slightly slower and assumes that hz is integral.
	 *
	 * Otherwise, round the time difference down to the maximum
	 * representable value.
	 *
	 * If ints have 32 bits, then the maximum value for any timeout in
	 * 10ms ticks is 248 days.
	 */
	sec = tv->tv_sec;
	usec = tv->tv_usec;
	if (usec < 0) {
		sec--;
		usec += 1000000;
	}
	if (sec < 0) {
#ifdef DIAGNOSTIC
		if (usec > 0) {
			sec++;
			usec -= 1000000;
		}
		printf("tvotohz: negative time difference %ld sec %ld usec\n",
		       sec, usec);
#endif
		ticks = 1;
	} else if (sec <= LONG_MAX / 1000000)
		ticks = (sec * 1000000 + (unsigned long)usec + (tick - 1))
			/ tick + 1;
	else if (sec <= LONG_MAX / hz)
		ticks = sec * hz
			+ ((unsigned long)usec + (tick - 1)) / tick + 1;
	else
		ticks = LONG_MAX;
	if (ticks > INT_MAX)
		ticks = INT_MAX;
	return ((int)ticks);
}
#endif

int
copyin(const void *uaddr, void *kaddr, size_t len)
{

	memcpy(kaddr, uaddr, len);

	return (0);
}

int
copyout(const void *kaddr, void *uaddr, size_t len)
{
	
	memcpy(uaddr, kaddr, len);

	return (0);
}


int
copystr(const void *kfaddr, void *kdaddr, size_t len, size_t *done)
{
	size_t bytes;
	
	bytes = strlcpy(kdaddr, kfaddr, len);
	if (done != NULL)
		*done = bytes;

	return (0);
}

int
copyinstr(const void *uaddr, void *kaddr, size_t len, size_t *done)
{	
	size_t bytes;
	
	bytes = strlcpy(kaddr, uaddr, len);
	if (done != NULL)
		*done = bytes;

	return (0);
}

#if 0
int
copyiniov(const struct iovec *iovp, u_int iovcnt, struct iovec **iov, int error)
{
	u_int iovlen;

	*iov = NULL;
	if (iovcnt > UIO_MAXIOV)
		return (error);
	iovlen = iovcnt * sizeof (struct iovec);
	*iov = malloc(iovlen, M_IOV, M_WAITOK);
	error = copyin(iovp, *iov, iovlen);
	if (error) {
		free(*iov, M_IOV);
		*iov = NULL;
	}
	return (error);
}

/*
 * Change the total socket buffer size a user has used.
 */
int
chgsbsize(uip, hiwat, to, max)
	struct	uidinfo	*uip;
	u_int  *hiwat;
	u_int	to;
	rlim_t	max;
{
	int diff;

	diff = to - *hiwat;
	if (diff > 0) {
		if (atomic_fetchadd_long(&uip->ui_sbsize, (long)diff) + diff > max) {
			atomic_subtract_long(&uip->ui_sbsize, (long)diff);
			return (0);
		}
	} else {
		atomic_add_long(&uip->ui_sbsize, (long)diff);
		if (uip->ui_sbsize < 0)
			printf("negative sbsize for uid = %d\n", uip->ui_uid);
	}
	*hiwat = to;
	return (1);
}

/*
 * Allocate a new resource limits structure and initialize its
 * reference count and mutex pointer.
 */
struct plimit *
lim_alloc()
{
	struct plimit *limp;

	limp = malloc(sizeof(struct plimit), M_PLIMIT, M_WAITOK);
	refcount_init(&limp->pl_refcnt, 1);
	return (limp);
}

struct plimit *
lim_hold(limp)
	struct plimit *limp;
{

	refcount_acquire(&limp->pl_refcnt);
	return (limp);
}

/*
 * Return the current (soft) limit for a particular system resource.
 * The which parameter which specifies the index into the rlimit array
 */
rlim_t
lim_cur(struct proc *p, int which)
{
	struct rlimit rl;

	lim_rlimit(p, which, &rl);
	return (rl.rlim_cur);
}

/*
 * Return a copy of the entire rlimit structure for the system limit
 * specified by 'which' in the rlimit structure pointed to by 'rlp'.
 */
void
lim_rlimit(struct proc *p, int which, struct rlimit *rlp)
{

	KASSERT(which >= 0 && which < RLIM_NLIMITS,
	    ("request for invalid resource limit"));
	*rlp = p->p_limit->pl_rlimit[which];
	if (p->p_sysent->sv_fixlimit != NULL)
		p->p_sysent->sv_fixlimit(rlp, which);
}

int
useracc(void *addr, int len, int rw)
{
	return (1);
}

struct proc *
zpfind(pid_t pid)
{

	return (NULL);
}


struct proc *
pfind(pid_t pid)
{

	return (NULL);
}
struct uidinfo uid0;


struct uidinfo *
uifind(uid_t uid)
{

	return (&uid0);
}

/*
 * Allocate a zeroed cred structure.
 */
struct ucred *
crget(void)
{
	register struct ucred *cr;

	cr = malloc(sizeof(*cr), M_CRED, M_WAITOK | M_ZERO);
	refcount_init(&cr->cr_ref, 1);

	return (cr);
}

/*
 * Claim another reference to a ucred structure.
 */
struct ucred *
crhold(struct ucred *cr)
{

	refcount_acquire(&cr->cr_ref);
	return (cr);
}

/*
 * Free a cred structure.  Throws away space when ref count gets to 0.
 */
void
crfree(struct ucred *cr)
{

	KASSERT(cr->cr_ref > 0, ("bad ucred refcount: %d", cr->cr_ref));
	KASSERT(cr->cr_ref != 0xdeadc0de, ("dangling reference to ucred"));
	if (refcount_release(&cr->cr_ref)) {

		free(cr, M_CRED);
	}
}

/*
 * Fill in a struct xucred based on a struct ucred.
 */

void
cru2x(struct ucred *cr, struct xucred *xcr)
{
#if 0
	int ngroups;

	bzero(xcr, sizeof(*xcr));
	xcr->cr_version = XUCRED_VERSION;
	xcr->cr_uid = cr->cr_uid;

	ngroups = MIN(cr->cr_ngroups, XU_NGROUPS);
	xcr->cr_ngroups = ngroups;
	bcopy(cr->cr_groups, xcr->cr_groups,
	    ngroups * sizeof(*cr->cr_groups));
#endif
}


int
cr_cansee(struct ucred *u1, struct ucred *u2)
{

	return (0);
}

int
cr_canseesocket(struct ucred *cred, struct socket *so)
{

	return (0);
}

int
cr_canseeinpcb(struct ucred *cred, struct inpcb *inp)
{

	return (0);
}


int
p_cansee(struct thread *td, struct proc *p)
{
	return (cr_cansee(td->td_ucred, p->p_ucred));
}


int
p_cansignal(struct thread *td, struct proc *p, int signum)
{

	return (0);
}

int
p_cansched(struct thread *td, struct proc *p)
{

	return (0);
}

int
securelevel_gt(struct ucred *cr, int level)
{

	return (0);
}


/**
 * @brief Send a 'notification' to userland, using standard ways
 */
void
devctl_notify(const char *system, const char *subsystem, const char *type,
    const char *data)
{
	;	
}
#endif

#if 0
vm_offset_t
kmem_alloc_contig(struct vmem *map, vm_size_t size, int flags, vm_paddr_t low,
    vm_paddr_t high, unsigned long alignment, unsigned long boundary,
    vm_memattr_t memattr)
{
	return (kmem_malloc(map, size, flags));
}
#endif

void
DELAY(int delay)
{
	struct timespec rqt;

	if (delay < 1000)
		return;
	
	rqt.tv_nsec = 1000*((unsigned long)delay);
	rqt.tv_sec = 0;
	nanosleep(&rqt, NULL);
}
#if 0
void
kick_proc0(void)
{
	;
}
#endif
#if 0
/*
 * Exit: deallocate address space and other resources, change proc state to
 * zombie, and unlink proc from allproc and parent's lists.  Save exit status
 * and rusage for wait().  Check for child processes and orphan them.
 */
void
exit1(struct thread *td, int rv)
{

	panic("notyet!");
}
/*
 * Make process 'parent' the new parent of process 'child'.
 * Must be called with an exclusive hold of proctree lock.
 */
void
proc_reparent(struct proc *child, struct proc *parent)
{
	panic("notyet!");
}
#endif

void
knlist_init(struct knlist *knl, void *lock,
    void (*kl_lock)(void *), void (*kl_unlock)(void *),
    void (*kl_assert_locked)(void *), void (*kl_assert_unlocked)(void *))
{}

void
knlist_add(struct knlist *knl, struct knote *kn, int islocked)
{}

void
knlist_cleardel(struct knlist *knl, struct thread *td, int islocked, int killkn)
{}


int
knlist_empty(struct knlist *knl)
{
	return (1);
}

void
knlist_destroy(struct knlist *knl)
{}

void
knlist_init_mtx(struct knlist *knl, struct mtx *lock)
{}

void
knlist_remove(struct knlist *knl, struct knote *kn, int islocked)
{
}

void
knote(struct knlist *list, long hint, int lockflags)
{}

void
knote_fdclose(struct thread *td, int fd)
{}

void
knote_fork(struct knlist *list, int pid)
{}

int 
kqfd_register(int fd, struct kevent *kev, struct thread *td, int waitok)
{
	return (0);
}

int
kqueue_add_filteropts(int filt, struct filterops *filtops)
{
	return (0);
}

#if 0
int
fget(struct thread *td, int fd, cap_rights_t *rightsp, struct file **fpp)
{
	return (0);
}

int
fget_read(struct thread *td, int fd, cap_rights_t *rightsp, struct file **fpp)
{
	return (0);
}

int
fget_write(struct thread *td, int fd, cap_rights_t *rightsp, struct file **fpp)
{
	return (0);
}


int
invfo_rdwr(struct file *fp, struct uio *uio, struct ucred *active_cred,
    int flags, struct thread *td)
{

	return (EOPNOTSUPP);
}

int
invfo_truncate(struct file *fp, off_t length, struct ucred *active_cred,
    struct thread *td)
{

	return (EINVAL);
}

int
invfo_ioctl(struct file *fp, u_long com, void *data,
    struct ucred *active_cred, struct thread *td)
{

	return (ENOTTY);
}

int
invfo_poll(struct file *fp, int events, struct ucred *active_cred,
    struct thread *td)
{

	return (poll_no_poll(events));
}

int
invfo_kqfilter(struct file *fp, struct knote *kn)
{

	return (EINVAL);
}

int
invfo_chmod(struct file *fp, mode_t mode, struct ucred *active_cred,
    struct thread *td)
{

	return (EINVAL);
}

int
invfo_chown(struct file *fp, uid_t uid, gid_t gid, struct ucred *active_cred,
    struct thread *td)
{

	return (EINVAL);
}

int
invfo_sendfile(struct file *fp, int sockfd, struct uio *hdr_uio,
    struct uio *trl_uio, off_t offset, size_t nbytes, off_t *sent, int flags,
    int kflags, struct sendfile_sync *sfs, struct thread *td)
{

	return (EINVAL);
}

int
kern_open(struct thread *td, char *path, enum uio_seg pathseg, int flags,
    int mode)
{

	return (ESRCH);
}
int
pget(pid_t pid, int flags, struct proc **pp)
{

	return (ESRCH);
}
#endif

void
timekeep_push_vdso(void)
{
}

void
addupc_intr(struct thread *td, uintfptr_t pc, u_int ticks)
{
}


struct shmfd;
int
shm_mmap(struct shmfd *shmfd, vm_size_t objsize, vm_ooffset_t foff,
		 vm_object_t *obj)
{
	return (ENOTSUP);
}

/* called from kern_exit.c */
void
shmexit(struct vmspace *vm)
{}

void
shmfork(struct proc *p1, struct proc *p2)
{ }

void
dump_add_page(vm_paddr_t pa)
{}

void
dump_drop_page(vm_paddr_t pa)
{}

struct dumperinfo;

int
dumpsys(struct dumperinfo *di)
{
	/* we don't need to dump ourself we've got the real kernel */
	return (0);
}

int
cpu_ptrace(struct thread *td, int req, void *addr, int data)
{
	panic("huh? no user!");
	return (0);
}


void
cpu_initclocks(void)
{
	/* .... */
}
struct uma_zone;
typedef struct uma_zone *uma_zone_t;

int
uma_zone_reserve_kva(uma_zone_t zone, int count)
{
	return (0);
}

int
uiomove_fromphys(vm_page_t ma[], vm_offset_t offset, int n, struct uio *uio)
{
	panic("unimplemented!");
}

/* XXX need clock */
int	clkintr_pending;
int hintmode;
char static_hints[1];
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <machine/intr_machdep.h>
#include <x86/apicreg.h>
#include <x86/apicvar.h>
struct apic_ops apic_ops;

void
mca_resume(void)
{

}

int
acpi_table_quirks(int *quirks)
{
	return (0);
}
