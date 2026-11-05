/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* epd-profpath-test: the path check and the open for files a control line names
 * (src/epd-profpath.h). A scratch directory stands in for EPD_PROF_DIR. Refused
 * paths must create and truncate nothing; an accepted one is created at 0640
 * whatever the umask, and a symlink in the directory is not followed. */
#include "../epd-profpath.h"
#include <stdlib.h>
#include <sys/stat.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static int exists(const char *p) { struct stat st; return !lstat(p, &st); }

static long size_of(const char *p) { struct stat st; return stat(p, &st) ? -1 : (long)st.st_size; }

static void put(const char *p, const char *s) { FILE *f = fopen(p, "w"); if (f) { fputs(s, f); fclose(f); } }

int main(void)
{
    char base[256], dir[300], outside[300], p[600], q[600];
    snprintf(base, sizeof base, "%s/epd-profpath-XXXXXX", getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp");
    if (!mkdtemp(base)) { perror("mkdtemp"); return 1; }
    snprintf(dir, sizeof dir, "%s/prof", base);
    snprintf(outside, sizeof outside, "%s/outside", base);
    CHECK(!mkdir(outside, 0755));

    /* the string check, no filesystem */
    CHECK(epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/main.csv"));
    CHECK(epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/a.b-c_1.freq"));
    CHECK(epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/.hidden"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", NULL));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", ""));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/etc/shadow"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/tmp/x"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "x.csv"));                              /* relative */
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof"));            /* the directory */
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/"));           /* no name */
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/."));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/.."));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/../../etc/shadow"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/../glassine-epdd-prof/x"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/sub/x"));      /* one level only */
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof//x"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-profx/y"));          /* a sibling with the prefix */
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/a\nb"));
    CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", "/run/glassine-epdd-prof/a\x7f"));
    {   /* the name's length */
        char long_name[300], ok_name[300];
        strcpy(long_name, "/run/glassine-epdd-prof/");
        strcpy(ok_name, long_name);
        memset(long_name + strlen(long_name), 'a', 201);
        long_name[strlen("/run/glassine-epdd-prof/") + 201] = 0;
        memset(ok_name + strlen(ok_name), 'a', 200);
        ok_name[strlen("/run/glassine-epdd-prof/") + 200] = 0;
        CHECK(!epd_prof_path_ok("/run/glassine-epdd-prof", long_name));
        CHECK(epd_prof_path_ok("/run/glassine-epdd-prof", ok_name));
    }

    /* the directory: made 0750, ours; one writable by others is not trusted */
    CHECK(!epd_prof_dir_make(dir, (gid_t)-1));
    CHECK(epd_prof_dir_ok(dir));
    {
        struct stat st;
        CHECK(!stat(dir, &st) && (st.st_mode & 0777) == 0750);
    }
    CHECK(epd_prof_dir_ok(outside));                /* (ours, 0755: fine) */
    CHECK(!chmod(outside, 0777) && !epd_prof_dir_ok(outside));
    CHECK(!chmod(outside, 0755));
    CHECK(!epd_prof_dir_ok("/nonexistent-epd-dir"));
    snprintf(p, sizeof p, "%s/link-to-dir", base);
    CHECK(!symlink(dir, p) && !epd_prof_dir_ok(p));                 /* a link to a good directory is not one */
    CHECK(epd_prof_dir_make(p, (gid_t)-1) != 0);

    /* the open: inside the directory */
    umask(0);
    snprintf(p, sizeof p, "%s/main.csv", dir);
    FILE *f = epd_prof_fopen_in(dir, p);
    CHECK(f != NULL);
    if (f) { fputs("t_s\n", f); fclose(f); }
    {
        struct stat st;
        CHECK(!stat(p, &st) && (st.st_mode & 0777) == 0640);        /* not 0666 though the umask is 0 */
    }
    f = epd_prof_fopen_in(dir, p);                                  /* truncates what is there */
    CHECK(f != NULL);
    if (f) fclose(f);
    CHECK(size_of(p) == 0);
    umask(077);                                                     /* a strict umask: still 0640, group video reads it */
    snprintf(q, sizeof q, "%s/strict.csv", dir);
    f = epd_prof_fopen_in(dir, q);
    CHECK(f != NULL);
    if (f) fclose(f);
    {
        struct stat st;
        CHECK(!stat(q, &st) && (st.st_mode & 0777) == 0640);
    }
    umask(0);

    /* refused: outside, ".." and sub-paths create and truncate nothing */
    snprintf(q, sizeof q, "%s/created", outside);
    CHECK(!epd_prof_fopen_in(dir, q) && !exists(q));
    snprintf(q, sizeof q, "%s/../outside/created", dir);
    CHECK(!epd_prof_fopen_in(dir, q) && !exists(q));
    snprintf(q, sizeof q, "%s/../x", dir);
    CHECK(!epd_prof_fopen_in(dir, q));
    snprintf(q, sizeof q, "%s/precious", outside);
    put(q, "keep");
    CHECK(!epd_prof_fopen_in(dir, q) && size_of(q) == 4);
    snprintf(q, sizeof q, "%s/../outside/precious", dir);
    CHECK(!epd_prof_fopen_in(dir, q) && size_of(q) == 4);
    errno = 0;
    CHECK(!epd_prof_fopen("/etc/shadow") && errno == EACCES);       /* the real directory's check, no write */
    CHECK(!epd_prof_fopen("/tmp/x") && !epd_prof_fopen("main_prof.csv"));

    /* a symlink in the directory (planted, or left behind) is not followed */
    snprintf(q, sizeof q, "%s/precious", outside);
    snprintf(p, sizeof p, "%s/planted", dir);
    CHECK(!symlink(q, p));
    CHECK(!epd_prof_fopen_in(dir, p) && size_of(q) == 4);
    snprintf(p, sizeof p, "%s/dangling", dir);
    snprintf(q, sizeof q, "%s/never", outside);
    CHECK(!symlink(q, p));
    CHECK(!epd_prof_fopen_in(dir, p) && !exists(q));                /* (not created through the link) */

    /* a directory that is not trusted refuses a good name */
    CHECK(!chmod(dir, 0777));
    snprintf(p, sizeof p, "%s/other.csv", dir);
    CHECK(!epd_prof_fopen_in(dir, p) && !exists(p));
    CHECK(!chmod(dir, 0750));

    /* epd_prof_name builds the daemon's own names inside the real directory */
    char nb[128];
    CHECK(epd_prof_path_ok(EPD_PROF_DIR, epd_prof_name(nb, sizeof nb, "glassine-shot-panel.pgm")));

    snprintf(p, sizeof p, "rm -rf '%s'", base);
    if (system(p)) fprintf(stderr, "(cleanup of %s failed)\n", base);
    if (fails) { fprintf(stderr, "epd-profpath: %d FAILED\n", fails); return 1; }
    printf("epd-profpath: ok\n");
    return 0;
}
