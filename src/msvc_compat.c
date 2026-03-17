#ifdef _MSC_VER

#include "msvc_compat.h"

// fcntl implementation for Windows
int fcntl(int fd, int cmd, ...) {
    if (cmd == F_GETFL) {
        return 0;  // Assume blocking by default
    } else if (cmd == F_SETFL) {
        // For F_SETFL, we ignore it on Windows (socket options set elsewhere)
        return 0;
    }
    return -1;
}

// ioctl implementation for Windows (maps to ioctlsocket for sockets)
int ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    int *argp = va_arg(args, int*);
    va_end(args);
    return ioctlsocket(fd, request, (u_long*)argp);
}

// poll implementation for Windows (uses select internally)
int poll(struct pollfd *fds, int nfds, int timeout) {
    fd_set readfds, writefds, exceptfds;
    struct timeval tv, *tvp = NULL;
    int maxfd = -1;

    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_ZERO(&exceptfds);

    for (int i = 0; i < nfds; i++) {
        if (fds[i].fd < 0) continue;
        if (fds[i].events & POLLIN) FD_SET(fds[i].fd, &readfds);
        if (fds[i].events & POLLOUT) FD_SET(fds[i].fd, &writefds);
        FD_SET(fds[i].fd, &exceptfds);
        if (fds[i].fd > maxfd) maxfd = fds[i].fd;
    }

    if (timeout >= 0) {
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        tvp = &tv;
    }

    int ret = select(maxfd + 1, &readfds, &writefds, &exceptfds, tvp);
    if (ret <= 0) return ret;

    for (int i = 0; i < nfds; i++) {
        fds[i].revents = 0;
        if (fds[i].fd < 0) continue;
        if (FD_ISSET(fds[i].fd, &readfds)) fds[i].revents |= POLLIN;
        if (FD_ISSET(fds[i].fd, &writefds)) fds[i].revents |= POLLOUT;
        if (FD_ISSET(fds[i].fd, &exceptfds)) fds[i].revents |= POLLERR;
    }

    return ret;
}

// nanosleep implementation for Windows (uses Sleep)
int nanosleep(const struct timespec *req, struct timespec *rem) {
    if (req == NULL) return -1;

    DWORD ms = (DWORD)((req->tv_sec * 1000) + (req->tv_nsec / 1000000));
    Sleep(ms);

    if (rem != NULL) {
        rem->tv_sec = 0;
        rem->tv_nsec = 0;
    }

    return 0;
}

#endif // _MSC_VER
