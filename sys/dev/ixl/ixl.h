/* $FreeBSD */
#include "opt_iflib.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/malloc.h>


#ifdef IFLIB
#include "iflib_ixl.h"
#else
#include "legacy_ixl.h"
#endif
