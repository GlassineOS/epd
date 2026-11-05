/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-profpath: where glassine-epdd-desktop (root) may write files a control
 * line names, and how it opens them.
 *
 * The control FIFO and eink.conf's "@" lines reach commands that take a path
 * (main_prof_csv=, drive_prof_csv=) or write fixed names (shot, dcmap,
 * srccheck). A path from there is not trusted: it must be a plain file name
 * directly under EPD_PROF_DIR, a root-owned directory the daemon creates in
 * /run. Nothing else is accepted (no other directory, no "..", no
 * subdirectory), and the file is opened with O_NOFOLLOW at mode 0640, so a
 * symlink in the directory is not followed either.
 *
 * Header only: epd-drm.c (drive_prof), glassine-epdd-desktop.c and the host
 * test (src/test/epd-profpath-test.c) include it. The checks take the
 * directory as an argument so the test can use a scratch one. */
#ifndef EPD_PROFPATH_H
#define EPD_PROFPATH_H
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef EPD_PROF_DIR            /* (a build for a test may name a scratch directory) */
#define EPD_PROF_DIR "/run/glassine-epdd-prof"
#endif
#define EPD_PROF_MODE 0640

/* 1 if path is EPD_PROF_DIR-style `dir` + "/" + one file name: no other
 * slash, not "." or "..", no control character, at most 200 bytes of name.
 * A pure string check; the open below does the rest. */
static inline int epd_prof_path_ok(const char *dir, const char *path)
{
    size_t n = strlen(dir);
    if (!path || strncmp(path, dir, n) || path[n] != '/') return 0;
    const char *name = path + n + 1;
    size_t k = strlen(name);
    if (!k || k > 200 || !strcmp(name, ".") || !strcmp(name, "..")) return 0;
    for (size_t i = 0; i < k; i++)
        if (name[i] == '/' || (unsigned char)name[i] < 0x20 || name[i] == 0x7f) return 0;
    return 1;
}

/* the directory is one we can rely on: a real directory (not a link) owned by
 * the daemon's user and not writable by anyone else */
static inline int epd_prof_dir_ok(const char *dir)
{
    struct stat st;
    return !lstat(dir, &st) && S_ISDIR(st.st_mode) && st.st_uid == geteuid() && !(st.st_mode & (S_IWGRP | S_IWOTH));
}

/* create `dir` if it is missing (mode 0750, group `gid` unless -1) and check
 * it. 0 = usable, -1 = not (errno set; nothing is written there then) */
static inline int epd_prof_dir_make(const char *dir, gid_t gid)
{
    if (mkdir(dir, 0750) && errno != EEXIST) return -1;
    if (epd_prof_dir_ok(dir) && gid != (gid_t)-1) {
        if (chown(dir, (uid_t)-1, gid) == 0) chmod(dir, 02750);     /* (group video reads what the daemon writes) */
    }
    if (epd_prof_dir_ok(dir)) return 0;
    errno = EPERM;
    return -1;
}

/* open `path` for writing: truncated, created at 0640, a symlink as the last
 * component refused. No check of where it is (see epd_prof_fopen_in). */
static inline FILE *epd_prof_open_file(const char *path)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC, EPD_PROF_MODE);
    if (fd < 0) return NULL;
    fchmod(fd, EPD_PROF_MODE);      /* (not the umask's) */
    FILE *f = fdopen(fd, "w");
    if (!f) { int e = errno; close(fd); errno = e; }
    return f;
}

/* open `path` if it is a file name under `dir` (made if missing); NULL and
 * errno set when refused or failed. A refused path creates nothing. */
static inline FILE *epd_prof_fopen_in(const char *dir, const char *path)
{
    if (!epd_prof_path_ok(dir, path)) { errno = EACCES; return NULL; }
    if (epd_prof_dir_make(dir, (gid_t)-1)) return NULL;
    return epd_prof_open_file(path);
}

static inline FILE *epd_prof_fopen(const char *path) { return epd_prof_fopen_in(EPD_PROF_DIR, path); }

/* EPD_PROF_DIR "/" name, for the files the daemon names itself */
static inline const char *epd_prof_name(char *buf, size_t n, const char *name)
{
    snprintf(buf, n, "%s/%s", EPD_PROF_DIR, name);
    return buf;
}
#endif
