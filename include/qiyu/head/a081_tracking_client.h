// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <qiyu/head/a081_tracking_wire.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// Never opens a sensor, changes a service, or reconnects behind the caller.
static inline int a081_tracking_connect(const char *path)
{
	struct sockaddr_un addr = {0};
	if (!path || path[0] != '/' || strlen(path) >= sizeof(addr.sun_path)) {
		errno = EINVAL;
		return -1;
	}
	addr.sun_family = AF_UNIX;
	memcpy(addr.sun_path, path, strlen(path) + 1);
	int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
	if (fd < 0) return -1;
	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		int saved = errno; close(fd); errno = saved; return -1;
	}
	struct ucred peer;
	socklen_t len = sizeof(peer);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &peer, &len) != 0 ||
	    len != sizeof(peer) || (peer.uid != geteuid() && peer.uid != 0)) {
		close(fd); errno = EACCES; return -1;
	}
	return fd;
}

// 1: one complete packet, 0: would block/interrupted, -1: disconnect/protocol error.
static inline int a081_tracking_receive(int fd, struct a081_tracking_packet *out)
{
	struct a081_tracking_packet p;
	ssize_t n = recv(fd, &p, sizeof(p), MSG_DONTWAIT | MSG_TRUNC);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
	if (n <= 0) { if (n == 0) errno = ECONNRESET; return -1; }
	if (n != sizeof(p) || p.magic != A081_TRACKING_MAGIC || p.version != A081_TRACKING_VERSION ||
	    p.bytes != sizeof(p) || !p.session_id || !p.sequence ||
	    (p.flags & ~A081_TRACKING_KNOWN_FLAGS) || !(p.flags & A081_TRACKING_IMU) ||
	    !p.gyro_boottime_ns || !p.accel_boottime_ns || p.accel_boottime_ns > p.gyro_boottime_ns ||
	    p.gyro_boottime_ns - p.accel_boottime_ns > A081_TRACKING_MAX_AGE_NS ||
	    p.reserved[0] || p.reserved[1] || p.reserved[2] ||
	    p.pose_read_status > A081_POSE_TIMESTAMP_MISMATCH) {
		errno = EBADMSG; return -1;
	}
	for (int i = 0; i < 3; ++i) {
		if (!isfinite(p.gyro[i]) || !isfinite(p.accel[i])) { errno = EBADMSG; return -1; }
	}
	if ((p.flags & A081_TRACKING_CLOCK) && (!p.sample_monotonic_ns ||
	    p.sample_monotonic_ns > p.send_monotonic_ns || p.clock_bracket_ns > A081_TRACKING_MAX_BRACKET_NS)) {
		errno = EBADMSG; return -1;
	}
	if ((p.flags & A081_TRACKING_RAW_POSE) && (p.pose_read_status != A081_POSE_COPIED ||
	    p.pose_index >= 80 || p.pose_boottime_ns != p.gyro_boottime_ns)) {
		errno = EBADMSG; return -1;
	}
	*out = p;
	return 1;
}
