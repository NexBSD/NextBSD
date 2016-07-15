#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/em/iflib_if_em.h>
#else
#include <dev/em/legacy_if_em.h>
#endif
