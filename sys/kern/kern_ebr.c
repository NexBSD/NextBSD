/*
 * Copyright (c) 2016 Matt Macy (mmacy@nextbsd.org)
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


#include <sys/types.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/smp.h>
#include <sys/taskqueue.h>

#include <machine/atomic.h>
#include <sys/queue.h>
#include <sys/ebr.h>

#include <ck_epoch.h>

struct ebr_epoch {
	ck_epoch_t ee_epoch;
};

struct ebr_entry {
	ck_epoch_entry_t ee_entry;
	ck_epoch_record_t *ee_record;
	ebr_callback_t ee_fn;
	void *ee_cookie;
	struct task ee_task;
};


#if 0

static ck_epoch_t global_epoch;
static DPCPU_DEFINE(ck_epoch_record_t *, epoch_record);

static void
ebr_runtime_init(void *arg __unused)
{
	ck_epoch_record_t *record, **pcpu_record;
	int i;

	ck_epoch_init(&global_epoch);

	CPU_FOREACH(i) {
		record = malloc(sizeof *record, M_EBR, M_WAITOK|M_ZERO);
		ck_epoch_register(&global_epoch, record);
		pcpu_record = DPCPU_ID_PTR(i, epoch_record);
		*pcpu_record = record;
	}

	for (i = 0; i < 2*mp_ncpus; i++) {
		record = malloc(sizeof *record, M_EBR, M_WAITOK|M_ZERO);
		ck_epoch_register(&global_epoch, record);
		ck_epoch_unregister(record);
	}
}
SYSINIT(ebr, SI_SUB_KTHREAD_PAGE, SI_ORDER_SECOND, ebr_runtime_init, NULL);

#endif

static MALLOC_DEFINE(M_EBR, "EBR", "CK EBR");

CK_EPOCH_CONTAINER(struct ebr_entry, ee_entry, ebr_entry_container)

static int ms_delayed;

static void
ebr_cleaner_func(void *context, int pending __unused)
{
	ebr_entry_t entry = context;
	ck_epoch_record_t *record = entry->ee_record;

	ck_epoch_barrier(record);
}

static void
ebr_destroy_entry(ck_epoch_entry_t *e)
{
	ck_epoch_record_t *record;
	ebr_entry_t entry = ebr_entry_container(e);

	record = entry->ee_record;
	entry->ee_record = NULL;
	entry->ee_fn(entry->ee_cookie);
	ck_epoch_unregister(record);
}

ebr_epoch_t
ebr_epoch_alloc(int pcpu_count)
{
	ebr_epoch_t epoch;
	ck_epoch_record_t *record;
	int i;

	epoch = malloc(sizeof(struct ebr_epoch), M_EBR, M_WAITOK|M_ZERO);
	ck_epoch_init(&epoch->ee_epoch);
	
	for (i = 0; i < pcpu_count*mp_ncpus; i++) {
		record = malloc(sizeof *record, M_EBR, M_WAITOK|M_ZERO);
		ck_epoch_register(&epoch->ee_epoch, record);
		ck_epoch_unregister(record);
	}
	return (epoch);
}

ebr_entry_t
ebr_epoch_entry_alloc(int flags)
{

	return (malloc(sizeof(struct ebr_entry), M_EBR, flags|M_ZERO));
}


void
ebr_epoch_entry_free(ebr_entry_t entry)
{

	free(entry, M_EBR);
}

int
ebr_epoch_entry_init(ebr_epoch_t eepoch, ebr_entry_t entry, void *cookie, bool cansleep)
{
	ck_epoch_t *epoch;
	ck_epoch_record_t *record;

	epoch = &eepoch->ee_epoch;
	entry->ee_cookie = cookie;
	if ((entry->ee_record = ck_epoch_recycle(epoch)) != NULL)
		return (0);
	if (!cansleep)
		return (ENOMEM);
	
	record = malloc(sizeof *record, M_EBR, M_WAITOK|M_ZERO);
	ck_epoch_register(epoch, record);
	entry->ee_record = record;
	return (0);
}

void *
ebr_epoch_read_lock(ebr_epoch_t eepoch)
{
	ck_epoch_record_t *record;

	while ((record = ck_epoch_recycle(&eepoch->ee_epoch)) == NULL) {
		DELAY(1);
		atomic_add_int(&ms_delayed, 1);
	}
	ck_epoch_begin(record, NULL);
	return (record);
}

void
ebr_epoch_read_unlock(void *cookie)
{
	ck_epoch_record_t *record;

	record = cookie;
	ck_epoch_end(record, NULL);
	ck_epoch_unregister(record);
}

void
ebr_epoch_defer(ebr_epoch_t eepoch, ebr_entry_t entry, ebr_callback_t fn)
{
	ck_epoch_record_t *record;

	while ((record = ck_epoch_recycle(&eepoch->ee_epoch)) == NULL) {
		DELAY(1);
		atomic_add_int(&ms_delayed, 1);
	}

	critical_enter();
	entry->ee_fn = fn;
	entry->ee_record = record;
	ck_epoch_call(record, &entry->ee_entry, ebr_destroy_entry);
	TASK_INIT(&entry->ee_task, 0, ebr_cleaner_func, entry);
	taskqueue_enqueue(taskqueue_fast, &entry->ee_task);
	critical_exit();
}

void
ebr_epoch_synchronize(ebr_epoch_t eepoch)
{
	ck_epoch_record_t *record;

	while ((record = ck_epoch_recycle(&eepoch->ee_epoch)) == NULL) {
		pause("eesync", 1);
		atomic_add_int(&ms_delayed, 1);
	}
	ck_epoch_synchronize(record);
	ck_epoch_unregister(record);
}
