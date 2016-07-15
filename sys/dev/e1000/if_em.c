#ifndef IFLIB
#include "opt_iflib.h"
#endif

#ifdef IFLIB
#include <dev/em/iflib_if_em.c>
#else
#include <dev/em/legacy_if_em.c>
#endif
