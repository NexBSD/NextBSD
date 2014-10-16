/*-
 * Copyright (c) 2010 Kip Macy
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

#undef _KERNEL
#define _WANT_UCRED
#include <sys/param.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/refcount.h>
#include <sys/ucred.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/stdint.h>
#include <sys/uio.h>

#define _KERNEL
#include <sys/proc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/linker.h>
#undef _KERNEL
#include <signal.h>
#include <unistd.h>

#include <stdio.h>
#include <pthread.h>

#include <machine/elf.h>
#include <machine/md_var.h>

void	bzero(void *buf, size_t len) __nonnull(1);

struct malloc_type;
__thread struct thread *pcurthread;

TAILQ_HEAD(prisonlist, prison);

struct cdev;
struct vnode *rootvnode;
extern struct	thread thread0;
extern struct	proc	proc0;
struct proclist allproc;
struct sx allproc_lock;
struct	sx allprison_lock;
struct	prisonlist allprison;

int async_io_version;

#define	M_ZERO		0x0100		/* bzero the allocation */


int
_pthread_create(pthread_t *thread, const pthread_attr_t *attr,
	       void *(*start_routine)(void *), void *arg);

#if 0
vm_offset_t kmem_malloc(void * map, int bytes, int wait);
void kmem_free(void *map, vm_offset_t addr, vm_size_t size);

vm_offset_t
kmem_malloc(void * map, int bytes, int wait)
{

	return ((vm_offset_t)mmap(NULL, bytes, PROT_READ|PROT_WRITE, MAP_ANON, -1, 0));
}


void
kmem_free(void *map, vm_offset_t addr, vm_size_t size)
{

	munmap((void *)addr, size);
}
#endif

void *
plebnet_malloc(unsigned long size, struct malloc_type *type, int flags)
{
	void *alloc;
	alloc = malloc(size);

	if ((flags & M_ZERO) && alloc != NULL)
		bzero(alloc, size);
	return (alloc);
}

void
plebnet_free(void *addr, struct malloc_type *type)
{

	free(addr);
}

#if 0
void
panic(const char *fmt, ...)
{

	abort();
}


void
nanotime(struct timespec *ts)
{

	clock_gettime(CLOCK_REALTIME_PRECISE, ts);
}
#endif

void
resettodr(void)
{
	
}

void
pn_init_thread0(void)
{

	pcurthread = &thread0;
}

struct pthread_start_args 
{
	struct thread *psa_td;
	void (*psa_start_routine)(void *);
	void *psa_arg;
};

static void *
pthread_start_routine(void *arg)
{
	struct pthread_start_args *psa = arg;

	pcurthread = psa->psa_td;
	pcurthread->td_proc = &proc0;
	psa->psa_start_routine(psa->psa_arg);
	free(psa->psa_td);
	free(psa);

	return (NULL);
}
#if 0
dev_t
tty_udev(struct tty *tp)
{

	return (NODEV);
}
#endif

uint64_t
racct_get_limit(struct proc *p, int resource)
{

	return (UINT64_MAX);
}


/* Process one elf relocation with addend. */
static int
elf_reloc_internal(linker_file_t lf, Elf_Addr relocbase, const void *data,
    int type, int local, elf_lookup_fn lookup)
{
	Elf64_Addr *where, val;
	Elf32_Addr *where32, val32;
	Elf_Addr addr;
	Elf_Addr addend;
	Elf_Size rtype, symidx;
	const Elf_Rel *rel;
	const Elf_Rela *rela;

	switch (type) {
	case ELF_RELOC_REL:
		rel = (const Elf_Rel *)data;
		where = (Elf_Addr *) (relocbase + rel->r_offset);
		rtype = ELF_R_TYPE(rel->r_info);
		symidx = ELF_R_SYM(rel->r_info);
		/* Addend is 32 bit on 32 bit relocs */
		switch (rtype) {
		case R_X86_64_PC32:
		case R_X86_64_32S:
			addend = *(Elf32_Addr *)where;
			break;
		default:
			addend = *where;
			break;
		}
		break;
	case ELF_RELOC_RELA:
		rela = (const Elf_Rela *)data;
		where = (Elf_Addr *) (relocbase + rela->r_offset);
		addend = rela->r_addend;
		rtype = ELF_R_TYPE(rela->r_info);
		symidx = ELF_R_SYM(rela->r_info);
		break;
	default:
		printf("unknown reloc type %d\n", type);
		abort();
	}

	switch (rtype) {

		case R_X86_64_NONE:	/* none */
			break;

		case R_X86_64_64:		/* S + A */
			addr = lookup(lf, symidx, 1);
			val = addr + addend;
			if (addr == 0)
				return -1;
			if (*where != val)
				*where = val;
			break;

		case R_X86_64_PC32:	/* S + A - P */
			addr = lookup(lf, symidx, 1);
			where32 = (Elf32_Addr *)where;
			val32 = (Elf32_Addr)(addr + addend - (Elf_Addr)where);
			if (addr == 0)
				return -1;
			if (*where32 != val32)
				*where32 = val32;
			break;

		case R_X86_64_32S:	/* S + A sign extend */
			addr = lookup(lf, symidx, 1);
			val32 = (Elf32_Addr)(addr + addend);
			where32 = (Elf32_Addr *)where;
			if (addr == 0)
				return -1;
			if (*where32 != val32)
				*where32 = val32;
			break;

		case R_X86_64_COPY:	/* none */
			/*
			 * There shouldn't be copy relocations in kernel
			 * objects.
			 */
			printf("kldload: unexpected R_COPY relocation\n");
			return -1;
			break;

		case R_X86_64_GLOB_DAT:	/* S */
		case R_X86_64_JMP_SLOT:	/* XXX need addend + offset */
			addr = lookup(lf, symidx, 1);
			if (addr == 0)
				return -1;
			if (*where != addr)
				*where = addr;
			break;

		case R_X86_64_RELATIVE:	/* B + A */
			addr = relocbase + addend;
			val = addr;
			if (*where != val)
				*where = val;
			break;

		default:
			printf("kldload: unexpected relocation type %ld\n",
			       rtype);
			return -1;
	}
	return(0);
}

int
elf_reloc(linker_file_t lf, Elf_Addr relocbase, const void *data, int type,
    elf_lookup_fn lookup)
{

	return (elf_reloc_internal(lf, relocbase, data, type, 0, lookup));
}

int
elf_reloc_local(linker_file_t lf, Elf_Addr relocbase, const void *data,
    int type, elf_lookup_fn lookup)
{

	return (elf_reloc_internal(lf, relocbase, data, type, 1, lookup));
}

int
elf_cpu_load_file(linker_file_t lf __unused)
{

	return (0);
}

int
elf_cpu_unload_file(linker_file_t lf __unused)
{

	return (0);
}

int
casuword32(volatile uint32_t *addr, uint32_t old, uint32_t new)
{

	return (atomic_cmpset_int(addr, old, new));
}

#include <link.h>
#include <dlfcn.h>

const void *
ukern_get_dynamic(void)
{
	struct link_map *map;
	int err;

	if ((err = dlinfo(RTLD_SELF, RTLD_DI_LINKMAP, (void *)&map)))
		return (NULL);

	return (map->l_ld);
}

const void *
ukern_get_address(void)
{
	struct link_map *map;
	int err;

	if ((err = dlinfo(RTLD_SELF, RTLD_DI_LINKMAP, (void *)&map)))
		return (NULL);

	return (map->l_addr);
}
