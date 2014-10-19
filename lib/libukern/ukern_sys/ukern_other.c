
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <machine/intr_machdep.h>
#include <x86/apicreg.h>
#include <x86/apicvar.h>

static SLIST_HEAD(, apic_enumerator) enumerators =
	SLIST_HEAD_INITIALIZER(enumerators);

void
apic_register_enumerator(struct apic_enumerator *enumerator)
{
#ifdef INVARIANTS
	struct apic_enumerator *apic_enum;

	SLIST_FOREACH(apic_enum, &enumerators, apic_next) {
		if (apic_enum == enumerator)
			panic("%s: Duplicate register of %s", __func__,
			    enumerator->apic_name);
	}
#endif
	SLIST_INSERT_HEAD(&enumerators, enumerator, apic_next);
}
