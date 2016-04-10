#include "opt_iflib.h"

#ifndef _IXL_H_
#define _IXL_H_

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/malloc.h>
MALLOC_DECLARE(M_IXL);

#ifdef IFLIB
#include "iflib_ixl.h"
#else
#include "legacy_ixl.h"
#endif

#endif
