#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/pci_pass.h>

#include <stdio.h>
#include <signal.h>

#include <ukern_intr.h>
#include <machine/cpu.h>

extern int ioctl(int fd, unsigned long request, ...);
extern int devpass_fd;

static void *timercookie;
static int vtimer_module_event_handler(module_t mod, int what, void *arg);


static moduledata_t vtimer_mod = {
	"vtimer",
	vtimer_module_event_handler,
	NULL
};


DECLARE_MODULE(vtimer, vtimer_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);

static int
timer_intr(void *arg)
{
	struct trapframe *fp = arg;

	hardclock(TRAPF_USERMODE(fp), TRAPF_PC(fp));
	return (FILTER_HANDLED);
}

static int
vtimer_module_init(void)
{
	struct dev_pass_timer dpt;

	dpt.dpt_trap = NULL;
	dpt.dpt_hz = hz;
	dpt.dpt_vector = 0;
	if (ioctl(devpass_fd, DEVPASSIOCTIMER, &dpt) ||
		dpt.dpt_vector == 0) {
		printf("timer setup failed!!!\n");
		return (ENXIO);
	}
	ukern_intr_register(EVTCHN_TYPE_VIRQ, dpt.dpt_vector);
	ukern_intr_bind("vtimer", dpt.dpt_vector, timer_intr, NULL, NULL,
					INTR_TYPE_CLK|INTR_MPSAFE, &timercookie, curcpu);

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

int
timer_spkr_acquire(void)
{

	return (0);
}

int
timer_spkr_release(void)
{

	return (0);
}

void
timer_spkr_setfreq(int freq)
{

}
