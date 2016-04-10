
#include "opt_iflib.h"

#ifdef IFLIB
#include "iflib_ixlvc.c"
#else
#include "legacy_ixlvc.c"
#endif
