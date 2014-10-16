
all:	test tcpconnect

test:	test.c
	$(CC) -g -pthread -o test test.c -L.. -lukern  -I../include

tcpconnect:	tcpconnect.c
	$(CC) -g -pthread -o tcpconnect tcpconnect.c -L.. -lukern  -I../include

clean:
	rm -f test test.core
