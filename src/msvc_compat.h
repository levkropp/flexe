/*
 * msvc_compat.h — POSIX compatibility shims for MSVC/Windows builds
 *
 * Provides usleep, clock_gettime, nanosleep, file operations, getopt,
 * GCC builtins, and other POSIX functions unavailable on MSVC.
 *
 * Include this BEFORE any standard headers in files that use POSIX APIs.
 * Guarded by _MSC_VER so it's a no-op on GCC/Clang.
 */
#ifndef MSVC_COMPAT_H
#define MSVC_COMPAT_H

#ifdef _MSC_VER

/* Suppress MSVC warnings for standard POSIX function names */
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#pragma warning(disable: 4996)  /* deprecated POSIX names (_fileno, etc.) */
#pragma warning(disable: 4100)  /* unreferenced formal parameter */
#pragma warning(disable: 4244)  /* possible loss of data (int64 -> long) */
#pragma warning(disable: 4267)  /* size_t -> int conversion */

/* Prevent winsock.h from being included (use winsock2.h instead) */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ---- GCC builtins ---- */

#define __attribute__(x)
#define __builtin_expect(expr, expected) (expr)

#include <float.h>
#define __builtin_isnan _isnan

#include <intrin.h>
static inline int __builtin_ctz(unsigned int x) {
    unsigned long index;
    _BitScanForward(&index, x);
    return (int)index;
}

/* ---- Types ---- */

#ifndef _SSIZE_T_DEFINED
#define _SSIZE_T_DEFINED
typedef intptr_t ssize_t;
#endif

typedef unsigned int useconds_t;

/* ---- usleep / sleep ---- */

static inline void usleep(unsigned int usec)
{
    DWORD ms = usec / 1000;
    if (ms == 0 && usec > 0) ms = 1;
    Sleep(ms);
}

#define sleep(x) Sleep((x)*1000)

/* ---- nanosleep ---- */

static inline int nanosleep(const struct timespec *req, struct timespec *rem)
{
    (void)rem;
    DWORD ms = (DWORD)(req->tv_sec * 1000 + req->tv_nsec / 1000000);
    if (ms == 0 && (req->tv_sec > 0 || req->tv_nsec > 0)) ms = 1;
    Sleep(ms);
    return 0;
}

/* ---- clock_gettime ---- */

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME  0
#endif
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

static inline int clock_gettime(int clk_id, struct timespec *tp)
{
    if (clk_id == CLOCK_MONOTONIC) {
        static LARGE_INTEGER frequency = {0};
        LARGE_INTEGER counter;
        if (frequency.QuadPart == 0)
            QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&counter);
        tp->tv_sec  = (long)(counter.QuadPart / frequency.QuadPart);
        tp->tv_nsec = (long)((counter.QuadPart % frequency.QuadPart) * 1000000000LL
                             / frequency.QuadPart);
    } else {
        /* CLOCK_REALTIME: Windows FILETIME -> Unix epoch */
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        uint64_t t = ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
        t -= 116444736000000000ULL; /* 100-ns intervals from 1601 to 1970 */
        tp->tv_sec  = (long)(t / 10000000ULL);
        tp->tv_nsec = (long)((t % 10000000ULL) * 100);
    }
    return 0;
}

/* ---- File operations ---- */

#include <io.h>
#include <direct.h>
#include <sys/stat.h>

#define ftruncate(fd, sz)          _chsize_s((fd), (long long)(sz))
#define fileno(f)                  _fileno(f)
#define fseeko(f, off, whence)     _fseeki64((f), (long long)(off), (whence))
#define mkdir(path, mode)          _mkdir(path)
#define strdup                     _strdup
#define popen                      _popen
#define pclose                     _pclose

/* access() mode constants */
#ifndef F_OK
#define F_OK 0
#define X_OK 1
#define W_OK 2
#define R_OK 4
#endif

/* ---- NORETURN ---- */

#ifndef NORETURN
#define NORETURN __declspec(noreturn)
#endif

/* ---- String functions ---- */

#define strncasecmp  _strnicmp
#define strcasecmp   _stricmp

/* ---- Signals ---- */

#include <signal.h>
#ifndef SIGPIPE
#define SIGPIPE 13
#endif

/* ---- Socket compat (declarations only — wifi_stubs.c handles includes) ---- */

#define MSG_NOSIGNAL  0
#define MSG_DONTWAIT  0
#define F_GETFL       3
#define F_SETFL       4
#define O_NONBLOCK    1

/* ---- getopt (supports optional args via "T::") ---- */

static char *optarg;
static int   optind = 1;
static int   opterr = 1;
static int   optopt;

static inline int getopt(int argc, char *const argv[], const char *optstring)
{
    static int sp = 1;

    if (optind >= argc || argv[optind][0] != '-' || argv[optind][1] == '\0')
        return -1;
    if (argv[optind][0] == '-' && argv[optind][1] == '-' && argv[optind][2] == '\0') {
        optind++;
        return -1;
    }

    int c = argv[optind][sp];
    const char *cp = NULL;

    for (const char *p = optstring; *p; p++) {
        if (*p == c) { cp = p; break; }
    }

    if (!cp || c == ':') {
        optopt = c;
        if (opterr) fprintf(stderr, "%s: unknown option '-%c'\n", argv[0], c);
        if (argv[optind][++sp] == '\0') { optind++; sp = 1; }
        return '?';
    }

    if (cp[1] == ':') {
        if (cp[2] == ':') {
            /* Optional argument (e.g., "T::") */
            if (argv[optind][sp + 1] != '\0')
                optarg = &argv[optind][sp + 1];
            else
                optarg = NULL;
            optind++;
            sp = 1;
        } else {
            /* Required argument */
            if (argv[optind][sp + 1] != '\0') {
                optarg = &argv[optind][sp + 1];
            } else if (++optind < argc) {
                optarg = argv[optind];
            } else {
                optopt = c;
                if (opterr)
                    fprintf(stderr, "%s: option '-%c' requires an argument\n", argv[0], c);
                sp = 1;
                optind++;
                return optstring[0] == ':' ? ':' : '?';
            }
            optind++;
            sp = 1;
        }
    } else {
        if (argv[optind][++sp] == '\0') {
            optind++;
            sp = 1;
        }
        optarg = NULL;
    }

    return c;
}

#else /* !_MSC_VER */

/* On GCC/Clang, define NORETURN using __attribute__ */
#ifndef NORETURN
#define NORETURN __attribute__((noreturn))
#endif

#endif /* _MSC_VER */

#endif /* MSVC_COMPAT_H */
