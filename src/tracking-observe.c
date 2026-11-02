// SPDX-License-Identifier: BSL-1.0
// Device-side read-only subscriber; writes only packets received from the producer.
#include <qiyu/head/a081_tracking_client.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static volatile sig_atomic_t interrupted;
static void stop(int sig) { (void)sig; interrupted = 1; }
static uint64_t now_ns(void)
{
	struct timespec t;
	if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
	return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}

int main(int argc, char **argv)
{
	if (argc < 3 || argc > 4) {
		fprintf(stderr, "Usage: a081-tracking-observe /absolute/socket /absolute/new-output.bin [seconds:1..60]\n");
		return 2;
	}
	char *end = NULL;
	long seconds = argc == 4 ? strtol(argv[3], &end, 10) : 10;
	if (seconds < 1 || seconds > 60 || (end && *end) || argv[2][0] != '/') return 2;
	int output = open(argv[2], O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
	if (output < 0) { perror("output"); return 1; }
	struct sigaction sa = {0}; sa.sa_handler = stop;
	sigemptyset(&sa.sa_mask); sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL);
	uint64_t deadline = now_ns() + (uint64_t)seconds * UINT64_C(1000000000);
	uint64_t packets = 0, raw_pose = 0, quality_zero = 0, quality_one = 0;
	uint64_t missing_pose = 0, invalid_clock = 0, stale = 0, gaps = 0;
	uint64_t session = 0, sequence = 0, last_gyro = 0, first_gyro = 0;
	int fd = -1, failed = 0;
	while (!interrupted && now_ns() < deadline) {
		if (fd < 0) {
			fd = a081_tracking_connect(argv[1]);
			if (fd < 0) { poll(NULL, 0, 20); continue; }
		}
		struct pollfd p = {fd, POLLIN, 0};
		int ready = poll(&p, 1, 50);
		if (ready < 0 && errno == EINTR) continue;
		if (ready < 0 || (p.revents & (POLLERR | POLLNVAL))) { failed = 1; break; }
		if (!ready) continue;
		struct a081_tracking_packet packet;
		int rc = a081_tracking_receive(fd, &packet);
		if (rc == 0) continue;
		if (rc < 0) {
			if (errno != ECONNRESET) { perror("tracking receive"); failed = 1; }
			break;
		}
		if (session && (packet.session_id != session || packet.sequence <= sequence || packet.gyro_boottime_ns <= last_gyro)) {
			fprintf(stderr, "Tracking session or timestamps regressed\n"); failed = 1; break;
		}
		if (session && packet.sequence != sequence + 1) gaps += packet.sequence - sequence - 1;
		session = packet.session_id; sequence = packet.sequence; last_gyro = packet.gyro_boottime_ns;
		if (!first_gyro) first_gyro = last_gyro;
		const unsigned char *bytes = (const unsigned char *)&packet;
		size_t written = 0;
		while (written < sizeof(packet)) {
			ssize_t n = write(output, bytes + written, sizeof(packet) - written);
			if (n < 0 && errno == EINTR) continue;
			if (n <= 0) { failed = 1; break; }
			written += (size_t)n;
		}
		if (failed) break;
		++packets;
		uint64_t now = now_ns();
		if (!(packet.flags & A081_TRACKING_CLOCK)) ++invalid_clock;
		else if (packet.sample_monotonic_ns > now || now - packet.sample_monotonic_ns >= A081_TRACKING_MAX_AGE_NS) ++stale;
		if (packet.flags & A081_TRACKING_RAW_POSE) {
			++raw_pose;
			quality_zero += packet.pose_quality == 0.0f;
			quality_one += packet.pose_quality == 1.0f;
		} else ++missing_pose;
	}
	if (fd >= 0) close(fd);
	if (close(output) != 0) failed = 1;
	printf("{\"packets\":%" PRIu64 ",\"raw_pose\":%" PRIu64 ",\"quality_zero\":%" PRIu64
	       ",\"quality_one\":%" PRIu64 ",\"missing_pose\":%" PRIu64 ",\"invalid_clock\":%" PRIu64
	       ",\"stale\":%" PRIu64 ",\"sequence_gaps\":%" PRIu64 ",\"session_id\":%" PRIu64
	       ",\"first_gyro_boottime_ns\":%" PRIu64 ",\"last_gyro_boottime_ns\":%" PRIu64
	       ",\"interrupted\":%s,\"error\":%s,\"openxr_position_valid\":false}\n",
	       packets, raw_pose, quality_zero, quality_one, missing_pose, invalid_clock, stale, gaps,
	       session, first_gyro, last_gyro, interrupted ? "true" : "false", failed ? "true" : "false");
	return failed || !packets ? 1 : 0;
}
