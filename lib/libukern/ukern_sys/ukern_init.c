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

#define	_SYS_LIBKERN_H_

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/pcpu.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/lock.h>
#include <sys/sx.h>

#include <pn_private.h>

#include <vm/uma.h>
#include <vm/uma_int.h>
#include <pthread.h>
#include <spawn.h>

char *     getenv(const char *name);
pid_t     getpid(void);
char *strndup(const char *str, size_t len);
unsigned int     sleep(unsigned int seconds);
size_t strlen(const char *s);

int putchar(int c);

extern void mi_startup(void);
extern void uma_startup(void *, int);
extern void uma_startup2(void);

struct sx proctree_lock;
struct pcpu *pcpup;

extern void pn_init_thread0(void);
extern void mutex_init(void);
extern void pmap_bootstrap(void *firstaddr);

static int pn_init(void) __attribute__((constructor));
pthread_mutex_t init_lock;
pthread_cond_t init_cond;
void *__malloc(int size);
extern uint64_t phys_avail[];
early_putc_t * early_putc = (early_putc_t *)putchar;

static int
pn_init(void)
{
	struct thread *td;
	int needconfig, error;
	char *plebconf, *rcconf;
	char buf[512];
	char *envp[3];
	char *argv[3];

	plebconf = getenv("PLEBCONF_PATH");
	rcconf = getenv("RC_CONF");
	
	needconfig = 1;
	if (plebconf == NULL || rcconf == NULL ||
	    strlen(plebconf) == 0 || strlen(rcconf) == 0) {
		printf("WARNING: PLEBCONF_PATH and RC_CONF need "
		    "to be set to configure the virtual interface automatically\n");
		needconfig = 0;
	}
	pcpup = __malloc(sizeof(struct pcpu));
	pcpu_init(pcpup, 0, sizeof(struct pcpu));

#if 0		
	kern_timeout_callwheel_alloc(__malloc(512*1024, M_DEVBUF, M_ZERO));
	kern_timeout_callwheel_init();
#endif		
	pn_init_thread0();
	pmap_bootstrap(__malloc(40*4096));
#if 0	
	uma_startup(__malloc(40*4096), 40);
	uma_startup2();
#endif	
	/* XXX fix this magic 64 to something a bit more dynamic & sensible */
	uma_page_slab_hash = __malloc(sizeof(struct uma_page)*64);
	uma_page_mask = 64-1;
	mutex_init();
	phys_avail[0] = (uint64_t)__malloc(4);
	phys_avail[1] = phys_avail[0] + 100*1024*1024;
	mi_startup();
	sx_init(&proctree_lock, "proctree");
	td = curthread;
	fdused_range(td->td_proc->p_fd, 16);
	start_server_syscalls();
	if (needconfig) {
		posix_spawnattr_t pattr;
		pid_t targetpid, shpid;

		pthread_mutex_lock(&init_lock);
		pthread_cond_wait(&init_cond, &init_lock);
		pthread_mutex_unlock(&init_lock);
		targetpid = getpid();
		sprintf(buf, "TARGET_PID=%d", targetpid);
		envp[0] = strndup(buf, 128);
		sprintf(buf, "LD_PRELOAD=%s/libplebconf.so", plebconf);
		envp[1] = strndup(buf, 128);
		envp[2] = NULL;

		argv[0] = "/bin/sh";
		argv[1] = rcconf;
		argv[2] = NULL;
		posix_spawnattr_init(&pattr);
		posix_spawnattr_setflags(&pattr, POSIX_SPAWN_SETPGROUP);
		error = posix_spawn(&shpid, "/bin/sh", NULL, &pattr, argv, envp);
		if (error)
			printf("posix_spawn failed %d\n", error);
	}
	/* give all configuration threads time to complete initialization
	 * before continuing
	 */
	sleep(1);
	return (0);
}
