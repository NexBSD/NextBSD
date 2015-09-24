/*
 * Copyright 2008 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

/*
 * BSD 3 Clause License
 *
 * Copyright (c) 2007, The Storage Networking Industry Association.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 	- Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in
 *	  the documentation and/or other materials provided with the
 *	  distribution.
 *
 *	- Neither the name of The Storage Networking Industry Association (SNIA)
 *	  nor the names of its contributors may be used to endorse or promote
 *	  products derived from this software without specific prior written
 *	  permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Provides encode/decode routines for all door servers/clients.
 */

#include <jansson.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <libndmp.h>
#include <unistd.h>
#include <ndmpd_door.h>


#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>


static int
ndmp_json_dump_callback(const char *buf __unused, size_t size, void *data)
{
	int len = *(int *)data;

	printf ("adding %zu to json dump \n", size);
	*(int *)data = len + size;
	return (0);
}

ndmp_door_ctx_t *
ndmp_door_decode_start(char *ptr, int size)
{
	ndmp_door_ctx_t *ctx = malloc(sizeof (ndmp_door_ctx_t));
	json_error_t error;

	if (ctx) {
		ctx->idx = 0;
		ctx->ptr = ptr;
		ctx->root = json_loadb(ptr, size, 0, &error);
		ctx->count = json_array_size(ctx->root);
		ctx->status = 0;
	}
	return (ctx);
}

int
ndmp_door_decode_finish(ndmp_door_ctx_t *ctx)
{
	int status = ctx->status;
	if ((status == 0) && (ctx->idx < ctx->count)) {
		status = ENOTEMPTY;
	}
	free(ctx);
	return (status);
}

ndmp_door_ctx_t *
ndmp_door_encode_start(char *buf, int len)
{
	ndmp_door_ctx_t *ctx = malloc(sizeof (ndmp_door_ctx_t));
	if (ctx) {
		if ((ctx->root = json_array()) == NULL) {
			free(ctx);
			return (NULL);
		}
		ctx->status = ctx->idx = ctx->count = 0;
		ctx->ptr = buf;
	}
	return (ctx);
}

int
ndmp_door_encode_finish(ndmp_door_ctx_t *ctx, unsigned int *used)
{
	int status = ctx->status;
	char *ptr;

	*used = 0;
#if 0
	/*
	 * XXX we need a _working_ means of determing the length of
	 * the string returned without calling strlen
	 */
	json_dump_callback(ctx->root, ndmp_json_dump_callback, used, 0);
#endif
	ptr = json_dumps(ctx->root, 0);
	*used = strlen(ptr);
	memcpy(ctx->ptr, ptr, *used);
	free(ptr);
	json_decref(ctx->root);
	free(ctx);
	return (status);
}

int32_t
ndmp_door_get_int32(ndmp_door_ctx_t *ctx)
{
	int32_t num = 0;
	json_t *data;

	if (ctx->status == 0) {
		data = json_array_get(ctx->root, ctx->idx);
		ctx->idx++;
		num = (int32_t)json_integer_value(data);
	}
	return (num);
}

uint32_t
ndmp_door_get_uint32(ndmp_door_ctx_t *ctx)
{
	return ((uint32_t)ndmp_door_get_int32(ctx));
}

char *
ndmp_door_get_string(ndmp_door_ctx_t *ctx)
{
	json_t *data;
	char *buf = NULL;
	const char *str;
	int len = ndmp_door_get_int32(ctx);

	if (ctx->status == 0) {
		if (len == -1)
			return (buf);

		buf = malloc(len +1);
		if (buf) {
			data = json_array_get(ctx->root, ctx->idx);
			ctx->idx++;
			str = json_string_value(data);
			if (len == 0) {
				(void) strcpy(buf, "");
			} else {
				(void) memcpy(buf, str, len);
				*(buf + len) = '\0';
			}
		} else {
			ctx->status = errno;
		}
	}
	return (buf);
}

void
ndmp_door_put_int32(ndmp_door_ctx_t *ctx, int32_t num)
{
	json_t *data;

	if (ctx->status == 0) {
		data = json_integer(num);
		json_array_append(ctx->root, data);
		ctx->count++;
	}
}

void
ndmp_door_put_uint32(ndmp_door_ctx_t *ctx, uint32_t num)
{
	ndmp_door_put_int32(ctx, (int32_t)num);
}

void
ndmp_door_put_string(ndmp_door_ctx_t *ctx, char *buf)
{
	int len;
	json_t *data;

	if (!buf)
		len = -1;
	else
		len = strlen(buf);

	if (ctx->status == 0) {
		ndmp_door_put_int32(ctx, len);
		if (buf)
			data = json_string(buf);
		else
			data = json_string("");
		json_array_append(ctx->root, data);
		ctx->count++;
	}
}

void
ndmp_door_free_string(char *buf)
{
	free(buf);
}

int64_t
ndmp_door_get_int64(ndmp_door_ctx_t *ctx)
{
	int64_t num = 0;
	json_t *data;

	if (ctx->status == 0) {
		data = json_array_get(ctx->root, ctx->idx);
		ctx->idx++;
		num = (int64_t)json_integer_value(data);
	}
	return (num);
}

uint64_t
ndmp_door_get_uint64(ndmp_door_ctx_t *ctx)
{
	return ((uint64_t)ndmp_door_get_int64(ctx));
}


static void
ndmp_door_put_int64(ndmp_door_ctx_t *ctx, int64_t num)
{
	json_t *data;

	if (ctx->status == 0) {
		data = json_integer(num);
		json_array_append(ctx->root, data);
		ctx->count++;
	}
}

void
ndmp_door_put_uint64(ndmp_door_ctx_t *ctx, uint64_t num)
{
	ndmp_door_put_int64(ctx, (int64_t)num);
}

void
ndmp_door_put_short(ndmp_door_ctx_t *ctx, short num)
{
	json_t *data;

	if (ctx->status == 0) {
		data = json_integer(num);
		json_array_append(ctx->root, data);
		ctx->count++;
	}
}

short
ndmp_door_get_short(ndmp_door_ctx_t *ctx)
{

	return ((short) ndmp_door_get_int32(ctx));
}

void
ndmp_door_put_ushort(ndmp_door_ctx_t *ctx, unsigned short num)
{
	ndmp_door_put_short(ctx, (short)num);
}

unsigned short
ndmp_door_get_ushort(ndmp_door_ctx_t *ctx)
{
	return ((unsigned short)ndmp_door_get_short(ctx));
}

void
ndmp_door_put_buf(ndmp_door_ctx_t *ctx, unsigned char *start, int len)
{
	json_t *data;

	ndmp_door_put_int32(ctx, len);
	if (ctx->status == 0) {
		data = json_string_nocheck(start);
		json_array_append(ctx->root, data);
		ctx->count++;
	}
}

int
ndmp_door_get_buf(ndmp_door_ctx_t *ctx, unsigned char *buf, int bufsize)
{
	int rc, len = -1;
	json_t *data;

	if (!buf)
		return (-1);

	len = ndmp_door_get_int32(ctx);
	if (ctx->status == 0) {
		if (bufsize < len) {
			ctx->status = ENOSPC;
			return (-2);
		}
		data = json_array_get(ctx->root, ctx->idx);
		ctx->idx++;
		rc = json_string_setn_nocheck(data, buf, bufsize);

	}

	return (len);
}

__thread int doorfd;

int
json_door_call(int fd, json_door_arg_t *arg)
{
	int rc;

	rc = write(fd, arg->data_ptr, arg->data_size);
	if (rc < 0)
		return (errno);
	rc = read(fd, arg->rbuf, NDMP_DOOR_SIZE);
	if (rc < 0)
		return (errno);
	arg->data_size = arg->rsize = rc;
	return (0);
}

int
json_door_return(caddr_t buf, int size, void *arg0 __unused, int arg1 __unused)
{

	return (write(doorfd, buf, size));
}

struct json_door_request_arg {
	int fd;
	size_t size;
	void (*func)(char *, size_t);
	char *ptr;
	char buf[NDMP_DOOR_SIZE];
};

struct json_door_server_arg {
	int fd;
	void (*func)(char *, size_t);
};

static void *
json_door_request(void *arg)
{
	struct json_door_request_arg *jdra = arg;

	doorfd = jdra->fd;
	while (1) {
		if ((jdra->size = read(jdra->fd, jdra->ptr, NDMP_DOOR_SIZE)) <= 0) {
			free(jdra);
			close(doorfd);
			pthread_exit(NULL);
		}
		jdra->func(jdra->ptr, jdra->size);
	}

}

static void *
json_door_server(void *arg)
{
	int newfd, fd;
	pthread_t thread;
	struct json_door_server_arg *jdsa = arg;
	struct json_door_request_arg *jdra;
	fd = jdsa->fd;

	listen(fd, -1);
	while ((newfd = accept(fd, NULL, 0)) > 0) {
		if ((jdra = malloc(sizeof(struct json_door_request_arg))) == NULL)
			perror("json_door_server failed to allocate request");
		jdra->fd = newfd;
		jdra->func = jdsa->func;
		jdra->ptr = jdra->buf;
		if (pthread_create(&thread, NULL, json_door_request, jdra))
			perror("json_door_server failed to create a thread:\n");
	}
	return (NULL);
}

int
json_door_open(int port)
{
	int fd;
	struct sockaddr_in lsin;

	lsin.sin_port = htons(port);
	lsin.sin_family = AF_INET;
	lsin.sin_addr.s_addr = htonl((u_long)INADDR_LOOPBACK);
	if ((fd = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
		return (fd);
	if (connect(fd, (struct sockaddr *)&lsin, sizeof(struct sockaddr)) < 0)
		return (-1);
	return (fd);
}

int
json_door_create(void (*server_procedure) (char *argp, size_t arg_size), int port)
{
	int fd, rc;
	struct sockaddr_in lsin;
	struct json_door_server_arg *jdsa;
	pthread_t thread;

	jdsa = malloc(sizeof(*jdsa));
	bzero(&lsin, sizeof(lsin));
	lsin.sin_port = htons(port);
	lsin.sin_family = AF_INET;
	lsin.sin_addr.s_addr = htonl((u_long)INADDR_LOOPBACK);
	if (jdsa == NULL)
		return (ENOMEM);
	if ((fd = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
		return (errno);
	rc = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &rc, sizeof(rc));
	if (bind(fd, (struct sockaddr *)&lsin, sizeof(lsin)) < 0) {
		perror("bind failed");
		return (errno);
	}
	jdsa->fd = fd;
	jdsa->func = server_procedure;
	rc = pthread_create(&thread, NULL, json_door_server, jdsa);
	if (rc)
		printf("pthread_create failed %s\n", strerror(rc));
	return (rc);
}
