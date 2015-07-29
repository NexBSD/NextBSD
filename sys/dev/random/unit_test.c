/*-
 * Copyright (c) 2000-2015 Mark R V Murray
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
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
 *
 * $FreeBSD$
 */

/*
 Build this by going:

cc -g -O0 -pthread -DRANDOM_<alg> -DRANDOM_DEBUG -I../.. -lstdthreads -Wall \
	unit_test.c \
	yarrow.c \
	fortuna.c \
	hash.c \
	../../crypto/rijndael/rijndael-api-fst.c \
	../../crypto/rijndael/rijndael-alg-fst.c \
	../../crypto/sha2/sha2.c \
        -lz \
	-o unit_test
./unit_test

Where <alg> is YARROW or FORTUNA.
*/

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <threads.h>
#include <unistd.h>
#include <zlib.h>

#include "unit_test.h"

#ifdef RANDOM_YARROW
#include "dev/random/yarrow.h"
#endif
#ifdef RANDOM_FORTUNA
#include "dev/random/fortuna.h"
#endif

#define	NUM_THREADS	  3
#define	DEBUG

static volatile int stopseeding = 0;

static __inline void
check_err(int err, const char *func)
{
	if (err != Z_OK) {
		fprintf(stderr, "Compress error in %s: %d\n", func, err);
		exit(0);
	}
}

void *
myalloc(void *q, unsigned n, unsigned m)
{
	q = Z_NULL;
	return (calloc(n, m));
}

void myfree(void *q, void *p)
{
	q = Z_NULL;
	free(p);
}

size_t
block_deflate(uint8_t *uncompr, uint8_t *compr, const size_t len)
{
	z_stream c_stream;
	int err;

	if (len == 0)
		return (0);

	c_stream.zalloc = myalloc;
	c_stream.zfree = myfree;
	c_stream.opaque = NULL;

	err = deflateInit(&c_stream, Z_DEFAULT_COMPRESSION);
	check_err(err, "deflateInit");

	c_stream.next_in  = uncompr;
	c_stream.next_out = compr;
	c_stream.avail_in = len;
	c_stream.avail_out = len*2u +512u;

	while (c_stream.total_in != len && c_stream.total_out < (len*2u + 512u)) {
		err = deflate(&c_stream, Z_NO_FLUSH);
#ifdef DEBUG
		printf("deflate: len = %zd  total_in = %lu  total_out = %lu\n", len, c_stream.total_in, c_stream.total_out);
#endif
		check_err(err, "deflate(..., Z_NO_FLUSH)");
	}

	for (;;) {
		err = deflate(&c_stream, Z_FINISH);
#ifdef DEBUG
		printf("deflate: len = %zd  total_in = %lu  total_out = %lu\n", len, c_stream.total_in, c_stream.total_out);
#endif
		if (err == Z_STREAM_END) break;
		check_err(err, "deflate(..., Z_STREAM_END)");
	}

	err = deflateEnd(&c_stream);
	check_err(err, "deflateEnd");

	return ((size_t)c_stream.total_out);
}

void
random_adaptor_unblock(void)
{

#if 0
	if (mtx_trylock(&random_reseed_mtx) == thrd_busy)
		printf("Mutex held. Good.\n");
	else {
		printf("Mutex not held. PANIC!!\n");
		thrd_exit(0);
	}
#endif
	printf("random: unblocking device.\n");
}

static int
RunHarvester(void *arg __unused)
{
	int i, r;
	struct harvest_event e;

	for (i = 0; ; i++) {
		if (stopseeding)
			break;
		if (i % 1000 == 0)
			printf("Harvest: %d\n", i);
		r = random()%10;
		e.he_somecounter = i;
		*((uint64_t *)e.he_entropy) = random();
		e.he_size = 8;
		e.he_bits = random()%4;
		e.he_destination = i;
		e.he_source = (i + 3)%7;
		e.he_next = NULL;
#ifdef RANDOM_YARROW
		random_yarrow_process_event(&e);
#endif
#ifdef RANDOM_FORTUNA
		random_fortuna_process_event(&e);
#endif
		usleep(r);
	}

	printf("Thread #0 ends\n");

	thrd_exit(0);

	return (0);
}

static int
WriteCSPRNG(void *threadid)
{
	uint8_t *buf;
	int i;

	printf("Thread #1 starts\n");

	for (i = 0; ; i++) {
		if (stopseeding)
			break;
		buf = malloc(4096);
		if (i % 1000 == 0)
			printf("Thread write 1 - %d\n", i);
		if (buf != NULL) {
			printf("Thread 1 writing.\n");
#ifdef RANDOM_YARROW
			random_yarrow_write(buf, i);
#endif
#ifdef RANDOM_FORTUNA
			random_fortuna_write(buf, i);
#endif
			free(buf);
		}
		usleep(1000000);
	}

	printf("Thread #1 ends\n");

	thrd_exit(0);

	return (0);
}

static int
ReadCSPRNG(void *threadid)
{
	size_t tid, zsize;
	uint8_t *buf, *zbuf;
	int i;
#ifdef DEBUG
	int j;
#endif

	tid = (size_t)threadid;
	printf("Thread #%zd starts\n", tid);

#ifdef RANDOM_YARROW
	while (!random_yarrow_seeded())
#endif
#ifdef RANDOM_FORTUNA
	while (!random_fortuna_seeded())
#endif
	{
#ifdef RANDOM_YARROW
		random_yarrow_pre_read();
		random_yarrow_post_read();
#endif
#ifdef RANDOM_FORTUNA
		random_fortuna_pre_read();
		random_fortuna_post_read();
#endif
		usleep(100);
	}

	for (i = 0; i < 100000; i++) {
		buf = malloc(i);
		zbuf = malloc(2*i + 1024);
		if (i % 1000 == 0)
			printf("Thread read %zd - %d\n", tid, i);
		if (buf != NULL && zbuf != NULL) {
#ifdef RANDOM_YARROW
			random_yarrow_pre_read();
			random_yarrow_read(buf, i);
			random_yarrow_post_read();
#endif
#ifdef RANDOM_FORTUNA
			random_fortuna_pre_read();
			random_fortuna_read(buf, i);
			random_fortuna_post_read();
#endif
			zsize = block_deflate(buf, zbuf, i);
			if (zsize < i)
				printf("ERROR!! Compressible RNG output!\n");
#ifdef DEBUG
			printf("RNG output:\n");
			for (j = 0; j < i; j++) {
				printf(" %02X", buf[j]);
				if (j % 32 == 31 || j == i - 1)
					printf("\n");
			}
			printf("Compressed output:\n");
			for (j = 0; j < zsize; j++) {
				printf(" %02X", zbuf[j]);
				if (j % 32 == 31 || j == zsize - 1)
					printf("\n");
			}
#endif
			free(zbuf);
			free(buf);
		}
		usleep(100);
	}

	printf("Thread #%zd ends\n", tid);

	thrd_exit(0);

	return (0);
}

int
main(int argc, char *argv[])
{
	thrd_t threads[NUM_THREADS];
	int rc;
	long t;

#ifdef RANDOM_YARROW
	random_yarrow_init_alg();
#endif
#ifdef RANDOM_FORTUNA
	random_fortuna_init_alg();
#endif

	for (t = 0; t < NUM_THREADS; t++) {
		printf("In main: creating thread %ld\n", t);
		rc = thrd_create(&threads[t], (t == 0 ? RunHarvester : (t == 1 ? WriteCSPRNG : ReadCSPRNG)), NULL);
		if (rc != thrd_success) {
			printf("ERROR; return code from thrd_create() is %d\n", rc);
			exit(-1);
		}
	}

	for (t = 2; t < NUM_THREADS; t++)
		thrd_join(threads[t], &rc);

	stopseeding = 1;

	thrd_join(threads[1], &rc);
	thrd_join(threads[0], &rc);

#ifdef RANDOM_YARROW
	random_yarrow_deinit_alg();
#endif
#ifdef RANDOM_FORTUNA
	random_fortuna_deinit_alg();
#endif

	/* Last thing that main() should do */
	thrd_exit(0);

	return (0);
}
