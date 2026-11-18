/* SPDX-License-Identifier: MIT */
/* Host test of main/SocketEvents.c (patch 0006): which command arms which
 * event, and the table.  Built and run by tests/run.sh against the patched
 * source checkout (firmware/nina-fw/source).
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "SocketEvents.h"

static int failures;

#define CHECK(cond)                                                                \
	do {                                                                       \
		if (!(cond)) {                                                     \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
			failures++;                                                \
		}                                                                  \
	} while (0)

#define FIRST 54 /* LWIP_SOCKET_OFFSET with 10 sockets */
#define COUNT 10

static uint8_t cmd[16];
static uint8_t rsp[32];

/* A command with the socket at [4] (8-bit parameter lengths). */
static void command(uint8_t c, uint8_t sock)
{
	memset(cmd, 0, sizeof(cmd));
	memset(rsp, 0, sizeof(rsp));
	cmd[0] = 0xe0;
	cmd[1] = c;
	cmd[2] = 1;
	cmd[3] = 1;
	cmd[4] = sock;
}

/* send and sendto: 16-bit lengths, the socket at [5]. */
static void wide_command(uint8_t c, uint8_t sock)
{
	command(c, 0);
	cmd[3] = 0;
	cmd[4] = 1;
	cmd[5] = sock;
}

static struct sockev_action after(int err)
{
	return sockev_after_command(cmd, rsp, err);
}

static void test_index(void)
{
	CHECK(sockev_index(FIRST, FIRST, COUNT) == 0);
	CHECK(sockev_index(FIRST + COUNT - 1, FIRST, COUNT) == COUNT - 1);
	CHECK(sockev_index(FIRST - 1, FIRST, COUNT) == -1);
	CHECK(sockev_index(FIRST + COUNT, FIRST, COUNT) == -1);
	CHECK(sockev_index(-1, FIRST, COUNT) == -1);
	CHECK(sockev_index(255, FIRST, COUNT) == -1);
	CHECK(sockev_index(0, 0, 33) == -1);
}

static void test_command_socket(void)
{
	command(0x71, 57);
	CHECK(sockev_command_socket(cmd) == 57);
	command(0x7c, 58);
	CHECK(sockev_command_socket(cmd) == 58);
	wide_command(0x77, 59);
	CHECK(sockev_command_socket(cmd) == 59);
	wide_command(0x79, 60);
	CHECK(sockev_command_socket(cmd) == 60);
	command(0x70, 61);
	CHECK(sockev_command_socket(cmd) == -1);
	command(0x72, 61);
	CHECK(sockev_command_socket(cmd) == -1);
	command(0x27, 61);
	CHECK(sockev_command_socket(cmd) == -1);
	command(0x6a, 61);
	CHECK(sockev_command_socket(cmd) == -1);
}

static void test_poll(void)
{
	struct sockev_action a;

	command(0x7c, 55);
	rsp[4] = 0x02; /* writable only */
	a = after(0);
	CHECK(a.fd == 55 && a.arm == SOCKEV_READ && !a.forget);

	rsp[4] = 0x00; /* connecting */
	a = after(0);
	CHECK(a.fd == 55 && a.arm == (SOCKEV_READ | SOCKEV_WRITE));

	rsp[4] = 0x03;
	a = after(0);
	CHECK(a.fd == -1 && a.arm == 0);

	rsp[4] = 0x01;
	a = after(0);
	CHECK(a.fd == 55 && a.arm == SOCKEV_WRITE);

	rsp[4] = 0x80; /* the poll failed */
	a = after(EBADF);
	CHECK(a.fd == -1 && a.arm == 0);

	rsp[4] = 0x84;
	a = after(0);
	CHECK(a.arm == 0);
}

static void test_recv(void)
{
	struct sockev_action a;

	command(0x78, 56);
	a = after(EAGAIN);
	CHECK(a.fd == 56 && a.arm == SOCKEV_READ);
	a = after(EWOULDBLOCK);
	CHECK(a.arm == SOCKEV_READ);
	a = after(0); /* the end of the stream */
	CHECK(a.fd == -1 && a.arm == 0);
	a = after(ECONNRESET);
	CHECK(a.arm == 0);
	rsp[3] = 0;
	rsp[4] = 10; /* data */
	a = after(EAGAIN);
	CHECK(a.arm == 0);
	rsp[3] = 1;
	rsp[4] = 0;
	a = after(EAGAIN);
	CHECK(a.arm == 0);

	command(0x7a, 56);
	a = after(EAGAIN);
	CHECK(a.fd == 56 && a.arm == SOCKEV_READ);
	rsp[14] = 48;
	a = after(EAGAIN);
	CHECK(a.arm == 0);
	rsp[14] = 0;
	rsp[13] = 1;
	a = after(EAGAIN);
	CHECK(a.arm == 0);
}

static void test_accept(void)
{
	struct sockev_action a;

	command(0x75, 54);
	rsp[4] = 255;
	a = after(EAGAIN);
	CHECK(a.fd == 54 && a.arm == SOCKEV_READ);
	a = after(ECONNABORTED);
	CHECK(a.arm == 0);
	rsp[4] = 58; /* a new connection */
	a = after(0);
	CHECK(a.fd == -1 && a.arm == 0);
}

static void test_send(void)
{
	struct sockev_action a;

	wide_command(0x77, 57);
	a = after(EAGAIN);
	CHECK(a.fd == 57 && a.arm == SOCKEV_WRITE);
	a = after(ENOMEM);
	CHECK(a.arm == SOCKEV_WRITE);
	a = after(EPIPE);
	CHECK(a.arm == 0);
	rsp[5] = 1; /* took one byte */
	a = after(EAGAIN);
	CHECK(a.arm == 0);
	rsp[5] = 0;
	rsp[4] = 1;
	a = after(EAGAIN);
	CHECK(a.arm == 0);

	wide_command(0x79, 58);
	a = after(ENOMEM);
	CHECK(a.fd == 58 && a.arm == SOCKEV_WRITE);
}

static void test_connect(void)
{
	struct sockev_action a;

	command(0x76, 59);
	a = after(EINPROGRESS);
	CHECK(a.fd == 59 && a.arm == SOCKEV_WRITE);
	a = after(EALREADY);
	CHECK(a.arm == SOCKEV_WRITE);
	a = after(ECONNREFUSED);
	CHECK(a.arm == 0);
	rsp[4] = 1;
	a = after(0);
	CHECK(a.arm == 0);
}

static void test_socket_and_others(void)
{
	struct sockev_action a;

	command(0x70, 0);
	rsp[4] = 60;
	a = after(0);
	CHECK(a.fd == 60 && a.forget && a.arm == 0);
	rsp[4] = 255;
	a = after(EMFILE);
	CHECK(a.fd == -1 && !a.forget);

	/* close is handled before the command runs. */
	command(0x71, 60);
	a = after(0);
	CHECK(a.fd == -1 && a.arm == 0 && !a.forget);

	command(0x7d, 60); /* setsockopt */
	a = after(EAGAIN);
	CHECK(a.fd == -1 && a.arm == 0);

	command(0x20, 0); /* getConnStatus */
	a = after(EAGAIN);
	CHECK(a.fd == -1 && a.arm == 0);
}

static void test_table(void)
{
	struct sockev_table t = {0, 0};

	CHECK(sockev_arm(&t, 3, SOCKEV_READ));
	CHECK(!sockev_arm(&t, 3, SOCKEV_READ));
	CHECK(sockev_arm(&t, 3, SOCKEV_READ | SOCKEV_WRITE));
	CHECK(!sockev_arm(&t, 3, SOCKEV_WRITE));
	CHECK(sockev_armed(&t, 3) == (SOCKEV_READ | SOCKEV_WRITE));
	CHECK(sockev_armed(&t, 4) == 0);
	CHECK(sockev_arm(&t, 9, SOCKEV_WRITE));

	/* Readable fires READ only, and once. */
	CHECK(sockev_fire(&t, 3, true, false, false) == SOCKEV_READ);
	CHECK(sockev_armed(&t, 3) == SOCKEV_WRITE);
	CHECK(sockev_fire(&t, 3, true, false, false) == 0);
	/* An exception fires everything armed. */
	CHECK(sockev_arm(&t, 3, SOCKEV_READ));
	CHECK(sockev_fire(&t, 3, false, false, true) == (SOCKEV_READ | SOCKEV_WRITE));
	CHECK(sockev_armed(&t, 3) == 0);
	/* Nothing reported: nothing fired. */
	CHECK(sockev_fire(&t, 9, false, false, false) == 0);
	CHECK(sockev_armed(&t, 9) == SOCKEV_WRITE);
	/* Not armed: readiness is not an event. */
	CHECK(sockev_fire(&t, 5, true, true, true) == 0);
	CHECK(sockev_fire(&t, 9, false, true, false) == SOCKEV_WRITE);
	CHECK(t.read == 0 && t.write == 0);

	CHECK(sockev_arm(&t, 0, SOCKEV_READ | SOCKEV_WRITE));
	CHECK(sockev_arm(&t, 31, SOCKEV_READ));
	sockev_disarm(&t, 0);
	CHECK(sockev_armed(&t, 0) == 0);
	CHECK(sockev_armed(&t, 31) == SOCKEV_READ);
	sockev_disarm(&t, 31);
	CHECK(t.read == 0 && t.write == 0);
}

int main(void)
{
	test_index();
	test_command_socket();
	test_poll();
	test_recv();
	test_accept();
	test_send();
	test_connect();
	test_socket_and_others();
	test_table();
	if (failures) {
		printf("%d failure(s)\n", failures);
		return 1;
	}
	printf("socket_events_test: all passed\n");
	return 0;
}
