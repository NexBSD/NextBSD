/*-
 * Copyright (c) 2014 Matthew Macy
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
#include <sys/pci_pass.h>
#include <sys/thr.h>
#include <sys/mman.h>

#include <pn_private.h>

#include <vm/uma.h>
#include <vm/uma_int.h>
#include <pthread.h>
#include <spawn.h>

#include <fcntl.h>
char *     getenv(const char *name);
pid_t     getpid(void);
char *strndup(const char *str, size_t len);
unsigned int     sleep(unsigned int seconds);
size_t strlen(const char *s);
int open(const char *path, int flags, ...);
int ioctl(int fd, unsigned long request, ...);
void *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t offset);

int thr_self(long *id);

void exit(int status);
int putchar(int c);

extern void mi_startup(void);
extern void uma_startup(void *, int);
extern void uma_startup2(void);

struct sx proctree_lock;
__thread struct pcpu pcpu;
__thread struct pcpu *pcpup;

extern void pn_init_thread0(void);
extern void pmap_bootstrap(void *firstaddr);

static int ukern_init(void) __attribute__((constructor));
pthread_mutex_t init_lock;
pthread_cond_t init_cond;
extern uint64_t phys_avail[];
early_putc_t * early_putc = (early_putc_t *)putchar;
extern void *ukern_init_secondary(void *arg);
u_int64_t hammer_time(u_int64_t modulep, u_int64_t physfree);
int *	__error(void);
void ukern_intr(struct trapframe *trap_frame);

int io_fd;
int devpass_fd;
int dev_pass_sys;
struct pass_status_page *status_page;
int nAPs = 0;
#define STK_SIZE (PAGE_SIZE*3)


static int
ukern_init(void)
{
	struct thread *td;
	int i, needconfig, error;
	char *plebconf, *rcconf;
	char buf[512];
	char *envp[3];
	char *argv[3];
	pthread_t tid;
	volatile int cputid;
	unsigned long selftid;
	struct dev_pass_vcpumap *dpv;
	void *code_page;
	caddr_t stack_pages;
	void *segbase;
	void *directpcpup;
#if 0
	plebconf = getenv("PLEBCONF_PATH");
	rcconf = getenv("RC_CONF");
	
	needconfig = 1;
	if (plebconf == NULL || rcconf == NULL ||
	    strlen(plebconf) == 0 || strlen(rcconf) == 0) {
		printf("WARNING: PLEBCONF_PATH and RC_CONF need "
		    "to be set to configure the virtual interface automatically\n");
		needconfig = 0;
	}
#endif	
	if ((io_fd = open("/dev/io", O_RDWR)) < 0) {
		printf("failed to open /dev/io - check permissions %d", *__error());
		exit(1);
	}
	if ((devpass_fd = open("/dev/devpass", O_RDWR)) < 0) {
		printf("failed to open /dev/io - check permissions %d", *__error());
		exit(1);
	}
	if ((status_page = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE,
						  MAP_PREFAULT_READ|MAP_PRIVATE|MAP_ANON, -1, 0)) ==
		MAP_FAILED)  {
		printf("status_page mmap failed %d", *__error());
		exit(1);
	}
	if (ioctl(devpass_fd, DEVPASSIOCSTATUSPAGE, &status_page) < 0) {
		printf("failed to enable status_page - check permissions %d", *__error());
		exit(1);
	}
	if ((code_page = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_EXEC,
						  MAP_PREFAULT_READ|MAP_PRIVATE|MAP_ANON, -1, 0)) ==
		MAP_FAILED)  {
		printf("code_page mmap failed %d", *__error());
		exit(1);
	}
	if (ioctl(devpass_fd, DEVPASSIOCCODEPAGE, &code_page) < 0) {
		printf("failed to enable code_page - check permissions %d", *__error());
		exit(1);
	}
	if (ioctl(devpass_fd, DEVPASSIOCSYS, &dev_pass_sys) < 0) {
		printf("failed to obtain devpass syscall - %d", *__error());
		exit(1);
	}
	if ((stack_pages = mmap(NULL, STK_SIZE*(nAPs+1), PROT_READ|PROT_WRITE,
						  MAP_PREFAULT_READ|MAP_PRIVATE|MAP_ANON, -1, 0)) ==
		MAP_FAILED)  {
		printf("status_page mmap failed %d", *__error());
		exit(1);
	}

	dpv = __malloc(sizeof(struct dev_pass_vcpumap));
	dpv->dpv_nvcpus = nAPs + 1;
	dpv->dpv_trap = (void *)ukern_intr;
	dpv->dpv_flags = PCI_PASS_C_TRAPFRAME;
	dpv->dpv_map[0].dpt_cpuid = 1;
	dpv->dpv_map[0].dpt_stk = (caddr_t)(stack_pages + STK_SIZE);
	thr_self(&selftid);
	dpv->dpv_map[0].dpt_tid = selftid;
	printf("ncpus=%d stack_pages=%p dpt_stk=%p\n", nAPs +1, stack_pages, stack_pages + STK_SIZE);
	for (i = 0; i < nAPs; i++) {
		cputid = i + 1;
		if (pthread_create(&tid, NULL, ukern_init_secondary, (void *)(uintptr_t)&cputid) < 0) {
			
			printf("failed to create AP pthread - %d\n", *__error());
			exit(1);
		}
		while (cputid == i + 1)
			;
		dpv->dpv_map[i + 1].dpt_cpuid = 2 + i;
		dpv->dpv_map[i + 1].dpt_tid = cputid;
		dpv->dpv_map[i + 1].dpt_stk = stack_pages + (i+1)*STK_SIZE;
	}

	for (i = 0; i < nAPs + 1; i++)
		printf("cpu%d: tid:%d stk: %p \n", i, dpv->dpv_map[i].dpt_tid, dpv->dpv_map[i].dpt_stk);
	if (ioctl(devpass_fd, DEVPASSIOCVCPUMAP, dpv) < 0) {
		printf("failed to setup vcpu mapping - %d\n", *__error());
		exit(1);
	}

	pcpup = &pcpu;
	__asm __volatile("movq %%fs:0, %0" : "=r" (segbase));
	printf("&pcpu=%p segbase=%p\n",&pcpu, segbase);
	__asm __volatile("movq %%fs:-0x480, %0" : "=r" (directpcpup));
	printf("pcpu=%p directpcpup=%p pcpup=%p\n",pcpu.pc_curthread, directpcpup, pcpup); 
	pcpu_init(&pcpu, 0, sizeof(struct pcpu));

	pn_init_thread0();
	pmap_bootstrap(__malloc(40*4096));
	hammer_time(0, (uint64_t)__malloc(40*4096));

	/* XXX fix this magic 64 to something a bit more dynamic & sensible */
	uma_page_slab_hash = __malloc(sizeof(struct uma_page)*64);
	uma_page_mask = 64-1;
	for (i = 0; i < 64; i++)
		LIST_INIT(&uma_page_slab_hash[i]);

	phys_avail[0] = (uint64_t)__malloc(4);
	phys_avail[1] = phys_avail[0] + 100*1024*1024;
	mi_startup();
	td = curthread;
#if 0	
	fdused_range(td->td_proc->p_fd, 16);
	start_server_syscalls();
#endif	
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
