#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <netdb.h>
#include <mach/mach.h>
#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <dispatch/dispatch.h>
#include <servers/bootstrap.h>

struct msg_send
{
	mach_msg_header_t hdr;
	char body[1024];
};

struct msg_recv
{
	mach_msg_header_t hdr;
	char body[1024];
	mach_msg_trailer_t trailer;
};

char *
contact_fingerd(const char *username)
{
        int sock;
        char buf[1024];
	char *input;
        char *ret;
        struct addrinfo *ai;
        size_t n;

	asprintf(&input, "%s\n", username);

        if (getaddrinfo("localhost", "finger", NULL, &ai) == -1) {
                syslog(LOG_ERR, "Cannot resolve localhost:finger: %m");
		free(input);
                return (NULL);
        }

        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == -1) {
                syslog(LOG_ERR, "Cannot create socket: %m");
                return (NULL);
        }

        if (connect(sock, ai->ai_addr, ai->ai_addrlen) == -1) {
                syslog(LOG_ERR, "Cannot connect to fingerd: %m");
		goto fail2;
        }

        if (write(sock, input, strlen(input)) != strlen(input)) {
                syslog(LOG_ERR, "Write failed: %m");
		goto fail1;
        }

        n = read(sock, buf, sizeof(buf) - 1);
        if (n == 0) {
                syslog(LOG_ERR, "Got empty response from fingerd");
		goto fail1;
        }

        shutdown(sock, SHUT_RDWR);
        close(sock);
	free(input);
        return strdup(buf);

fail1:
	shutdown(sock, SHUT_RDWR);
fail2:
	close(sock);
	free(input);
	return (NULL);

}

int main()
{
	__block kern_return_t kr;
	mach_port_t bport, port;
	dispatch_source_t source;

	task_get_special_port(mach_task_self(), TASK_BOOTSTRAP_PORT, &bport);
	syslog(LOG_ERR, "bootstrap port: %d", bport);

	kr = bootstrap_check_in(bootstrap_port, "org.freebsd.demoserver", &port);
	if (kr != KERN_SUCCESS) {
		syslog(LOG_ERR, "bootstrap_check_in: kr=%d", kr);
		exit(1);
	}

	syslog(LOG_ERR, "service port: %d", port);

	source = dispatch_source_create(
	    DISPATCH_SOURCE_TYPE_MACH_RECV, port, 0,
	    dispatch_get_main_queue());

	dispatch_source_set_event_handler(source, ^{
		struct msg_recv message;
		struct msg_send reply;
		char *result;

		message.hdr.msgh_local_port = port;
		message.hdr.msgh_size = sizeof(struct msg_recv);

		kr = mach_msg_receive((mach_msg_header_t *)&message);
		if (kr != KERN_SUCCESS)
			syslog(LOG_ERR, "mach_msg_receive failure: kr=%d", kr);
		else
			syslog(LOG_ERR, "received message on port %d: body=%s", message.hdr.msgh_remote_port, message.body);

		result = contact_fingerd(message.body);

		memset(&reply, 0, sizeof(struct msg_send));
		strcpy(&reply.body[0], result);
		reply.hdr.msgh_local_port = MACH_PORT_NULL;
		reply.hdr.msgh_remote_port = message.hdr.msgh_remote_port;
		reply.hdr.msgh_size = sizeof(struct msg_send);
		reply.hdr.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0);
		kr = mach_msg_send((mach_msg_header_t *)&reply);
		if (kr != KERN_SUCCESS)
			syslog(LOG_ERR, "mach_msg_send failure: kr=%d", kr);

		free(result);
	});

	dispatch_resume(source);
	dispatch_main();
}

