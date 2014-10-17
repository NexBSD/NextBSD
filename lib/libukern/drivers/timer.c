#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/time.h>

#include <stdio.h>
#include <signal.h>


int	setitimer(int, const struct itimerval *, struct itimerval *);

extern void *malloc(size_t);
static int vtimer_module_event_handler(module_t mod, int what, void *arg);


static moduledata_t vtimer_mod = {
	"vtimer",
	vtimer_module_event_handler,
	NULL
};


DECLARE_MODULE(vtimer, vtimer_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);

static void
timer_handler(int sig, siginfo_t *info, void *env)
{
	hardclock(0, (uintfptr_t)info->si_addr);
}

static int
vtimer_module_init(void)
{
	struct sigaction vtsa;
	stack_t sstk;
	struct itimerval itv;

	sstk.ss_sp = malloc(SIGSTKSZ);
	sstk.ss_size = SIGSTKSZ;
	sstk.ss_flags = 0;

	if (sigaltstack(&sstk, NULL) < 0)
		perror("sigaltstack");
	
	bzero(&vtsa, sizeof(vtsa));
	vtsa.__sigaction_u.__sa_sigaction = timer_handler;
	vtsa.sa_flags = SA_ONSTACK|SA_RESTART|SA_SIGINFO;
	
	sigaction(SIGALRM, &vtsa, NULL);
	itv.it_interval.tv_sec = 0;
	itv.it_interval.tv_usec = 10;
	itv.it_value.tv_sec = 4;
	itv.it_value.tv_usec = 0;
	setitimer(ITIMER_REAL, &itv, NULL);
	return (0);
}

static int
vtimer_module_event_handler(module_t mod, int what, void *arg)
{
	int err;

	switch (what) {
	case MOD_LOAD:
		if ((err = vtimer_module_init()) != 0)
			return (err);
		break;
	case MOD_UNLOAD:
		return (EBUSY);
	default:
		return (EOPNOTSUPP);
	}

	return (0);
}
