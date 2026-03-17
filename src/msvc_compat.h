#ifndef MSVC_COMPAT_H
#define MSVC_COMPAT_H

#ifdef _MSC_VER

// Prevent winsock.h from being included (use winsock2.h instead)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// MSVC doesn't have __attribute__
#define __attribute__(x)

// MSVC doesn't have __builtin_expect
#define __builtin_expect(expr, expected) (expr)

// MSVC __builtin_isnan
#include <float.h>
#define __builtin_isnan _isnan

// MSVC __builtin_ctz (count trailing zeros)
#include <intrin.h>
static inline int __builtin_ctz(unsigned int x) {
    unsigned long index;
    _BitScanForward(&index, x);
    return (int)index;
}

// Unistd.h replacement for MSVC
#include <io.h>
#include <process.h>
#include <time.h>

// Socket operations (Winsock) - MUST be before windows.h
#pragma comment(lib, "ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#define usleep(x) Sleep((x)/1000)
#define sleep(x) Sleep((x)*1000)

// clock_gettime replacement for MSVC
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#define CLOCK_REALTIME 0

static inline int clock_gettime(int clk_id, struct timespec *tp) {
    static LARGE_INTEGER frequency = {0};
    LARGE_INTEGER counter;

    if (frequency.QuadPart == 0) {
        QueryPerformanceFrequency(&frequency);
    }

    QueryPerformanceCounter(&counter);

    tp->tv_sec = (time_t)(counter.QuadPart / frequency.QuadPart);
    tp->tv_nsec = (long)(((counter.QuadPart % frequency.QuadPart) * 1000000000LL) / frequency.QuadPart);

    return 0;
}
#endif

// POSIX strdup
#define strdup _strdup

// POSIX file operations
#define read _read
#define write _write
#define close _close
#define lseek _lseek
#define ftruncate _chsize
typedef long off_t;

// access() mode constants
#ifndef F_OK
#define F_OK 0
#define X_OK 1
#define W_OK 2
#define R_OK 4
#endif

// useconds_t for usleep
typedef unsigned int useconds_t;

// POSIX compatibility for sockets
#define socklen_t int
typedef int ssize_t;

// fcntl flags
#define F_GETFL 3
#define F_SETFL 4
#define O_NONBLOCK 1

// Socket flags
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0
#endif

// fcntl, ioctl, poll, nanosleep - implemented in msvc_compat.c
int fcntl(int fd, int cmd, ...);
int ioctl(int fd, unsigned long request, ...);
int nanosleep(const struct timespec *req, struct timespec *rem);

// poll structure for Windows
#ifndef POLLIN
#define POLLIN  0x0001
#define POLLOUT 0x0004
#define POLLERR 0x0008

struct pollfd {
    int fd;
    short events;
    short revents;
};

int poll(struct pollfd *fds, int nfds, int timeout);
#endif

// strncasecmp and strcasecmp
#define strncasecmp _strnicmp
#define strcasecmp _stricmp

// popen/pclose
#define popen _popen
#define pclose _pclose

// fseeko (Windows uses _fseeki64 for large file support)
#define fseeko _fseeki64

// SIGPIPE doesn't exist on Windows
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

// signal() stub - does nothing on Windows for SIGPIPE
#include <signal.h>
#ifndef SIG_IGN
#define SIG_IGN ((void (*)(int))1)
#endif

// getopt implementation for MSVC
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern char *optarg;
extern int optind, opterr, optopt;

static char *optarg = NULL;
static int optind = 1;
static int opterr = 1;
static int optopt = 0;

static inline int getopt(int argc, char *const argv[], const char *optstring) {
    static int optpos = 1;
    const char *arg;

    optarg = NULL;

    if (optind >= argc || argv[optind][0] != '-' || argv[optind][1] == '\0')
        return -1;

    if (argv[optind][1] == '-' && argv[optind][2] == '\0') {
        optind++;
        return -1;
    }

    optopt = argv[optind][optpos];
    const char *p = strchr(optstring, optopt);

    if (p == NULL) {
        if (opterr && *optstring != ':')
            fprintf(stderr, "%s: illegal option -- %c\n", argv[0], optopt);
        if (argv[optind][++optpos] == '\0') {
            optind++;
            optpos = 1;
        }
        return '?';
    }

    if (p[1] == ':') {
        if (argv[optind][optpos + 1] != '\0') {
            optarg = &argv[optind][optpos + 1];
            optind++;
            optpos = 1;
        } else if (optind + 1 < argc) {
            optarg = argv[++optind];
            optind++;
            optpos = 1;
        } else {
            if (opterr && *optstring != ':')
                fprintf(stderr, "%s: option requires an argument -- %c\n", argv[0], optopt);
            if (argv[optind][++optpos] == '\0') {
                optind++;
                optpos = 1;
            }
            return *optstring == ':' ? ':' : '?';
        }
    } else {
        if (argv[optind][++optpos] == '\0') {
            optind++;
            optpos = 1;
        }
    }

    return optopt;
}

#endif // _MSC_VER

#endif // MSVC_COMPAT_H
