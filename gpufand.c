/*
 * gpufand — a small fan-curve daemon for NVIDIA cards on Linux, keyed to the
 * GDDR6X memory-junction temperature (which the driver does not expose on
 * GeForce) with the core temperature as a floor.
 *
 *   fan% = max( mem_curve(memory °C), core_curve(core °C) ), rate-limited.
 *
 * Dependencies: none beyond libc and libdl. NVML (libnvidia-ml.so.1, shipped
 * with the driver) is loaded at run time, so no CUDA toolkit or nvml.h is
 * needed to build. The memory temperature is read the way gddr6 does it —
 * one 32-bit register in the card's BAR0 — but through sysfs
 * (/sys/bus/pci/devices/<addr>/resource0) first, which needs no iomem=relaxed,
 * and through /dev/mem only as a fallback. Needs root.
 *
 * Build:   cc -O2 -Wall -o gpufand gpufand.c -ldl
 *
 * Daemon:  sudo gpufand [--mem CURVE] [--core CURVE] [--min P] [--max P]
 *                       [--interval S] [--step P] [--gpu N] [--quiet]
 * One instance per machine. If one is already running, a second invocation
 * does not start; instead it talks to the running one over a Unix socket:
 *
 *   sudo gpufand --status              what it is doing right now
 *   sudo gpufand --mem ... --core ...  change the curves/limits in the running daemon
 *   sudo gpufand --pause [SECONDS]     hand the fans back to the driver (for a while)
 *   sudo gpufand --resume              take them back
 *   sudo gpufand --stop                exit gracefully (fans handed back)
 *   sudo gpufand --reset               no daemon needed: hand the fans back to the driver now
 *   sudo gpufand --once                one reading, no daemon needed, fans untouched
 *
 * The device table and the register decode are taken from gddr6
 * (https://github.com/olealgoritme/gddr6, MIT licence) — credit to its authors;
 * the mistakes in this file are mine.
 *
 * Safety: a manual fan speed set through NVML PERSISTS until control is handed
 * back explicitly — it does not lapse when the setter dies. So the daemon hands
 * back on SIGINT/SIGTERM/SIGHUP/--stop, during a pause, on an NVML write error,
 * and after MAX_READ_FAILURES consecutive failed reads; and `gpufand --reset`
 * hands back after an unclean exit (SIGKILL, crash, suspend). Run it as a
 * systemd service with Restart=on-failure so a crashed controller is replaced.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define SOCK_PATH "/run/gpufand.sock"
#define PID_PATH  "/run/gpufand.pid"
#define MAX_READ_FAILURES 10

/* ------------------------------------------------------------------ NVML --- */
typedef void *nvmlDevice_t;
typedef int nvmlReturn_t;                      /* 0 == NVML_SUCCESS            */
typedef struct {                               /* nvmlPciInfo_t, v3 layout      */
    char busIdLegacy[16];
    unsigned int domain, bus, device, pciDeviceId, pciSubSystemId;
    char busId[32];
} nvmlPciInfo_t;

static nvmlReturn_t (*p_nvmlInit)(void);
static nvmlReturn_t (*p_nvmlShutdown)(void);
static nvmlReturn_t (*p_GetHandle)(unsigned int, nvmlDevice_t *);
static nvmlReturn_t (*p_GetName)(nvmlDevice_t, char *, unsigned int);
static nvmlReturn_t (*p_GetTemp)(nvmlDevice_t, int, unsigned int *);
static nvmlReturn_t (*p_GetNumFans)(nvmlDevice_t, unsigned int *);
static nvmlReturn_t (*p_GetFanSpeed)(nvmlDevice_t, unsigned int, unsigned int *);
static nvmlReturn_t (*p_SetFanSpeed)(nvmlDevice_t, unsigned int, unsigned int);
static nvmlReturn_t (*p_SetDefaultFan)(nvmlDevice_t, unsigned int);
static nvmlReturn_t (*p_GetPciInfo)(nvmlDevice_t, nvmlPciInfo_t *);
static const char *(*p_ErrorString)(nvmlReturn_t);

static int nvml_load(void)
{
    void *h = dlopen("libnvidia-ml.so.1", RTLD_NOW);
    if (!h) h = dlopen("libnvidia-ml.so", RTLD_NOW);
    if (!h) { fprintf(stderr, "cannot load libnvidia-ml.so (%s) — is the NVIDIA driver installed?\n", dlerror()); return -1; }
#define L(var, name) do { *(void **)&var = dlsym(h, name); \
        if (!var) { fprintf(stderr, "NVML lacks %s (driver too old? needs >= 520)\n", name); return -1; } } while (0)
    L(p_nvmlInit,      "nvmlInit_v2");
    L(p_nvmlShutdown,  "nvmlShutdown");
    L(p_GetHandle,     "nvmlDeviceGetHandleByIndex_v2");
    L(p_GetName,       "nvmlDeviceGetName");
    L(p_GetTemp,       "nvmlDeviceGetTemperature");
    L(p_GetNumFans,    "nvmlDeviceGetNumFans");
    L(p_GetFanSpeed,   "nvmlDeviceGetFanSpeed_v2");
    L(p_SetFanSpeed,   "nvmlDeviceSetFanSpeed_v2");
    L(p_SetDefaultFan, "nvmlDeviceSetDefaultFanSpeed_v2");
    L(p_GetPciInfo,    "nvmlDeviceGetPciInfo_v3");
    L(p_ErrorString,   "nvmlErrorString");
#undef L
    return 0;
}

/* -------------------------------------------------- memory sensor (gddr6) --- */
enum decode { DECODE_ADA = 0, DECODE_GDDR_MRCODE = 1 };
struct devinfo { uint32_t offset; enum decode decode; uint16_t dev_id; const char *name; };

/* From gddr6's table (MIT). Ampere/Ada: temperature = (reg & 0xfff) / 32. */
static const struct devinfo dev_table[] = {
    { 0x0000E2A8, DECODE_ADA, 0x2684, "RTX 4090" },
    { 0x0000E2A8, DECODE_ADA, 0x2685, "RTX 4090 D" },
    { 0x0000E2A8, DECODE_ADA, 0x2702, "RTX 4080 Super" },
    { 0x0000E2A8, DECODE_ADA, 0x2704, "RTX 4080" },
    { 0x0000E2A8, DECODE_ADA, 0x2705, "RTX 4070 Ti Super" },
    { 0x0000E2A8, DECODE_ADA, 0x2782, "RTX 4070 Ti" },
    { 0x0000E2A8, DECODE_ADA, 0x2783, "RTX 4070 Super" },
    { 0x0000E2A8, DECODE_ADA, 0x2786, "RTX 4070" },
    { 0x0000E2A8, DECODE_ADA, 0x2860, "RTX 4070 Max-Q / Mobile" },
    { 0x0000E2A8, DECODE_ADA, 0x28e0, "RTX 4060 Max-Q / Mobile" },
    { 0x0000E2A8, DECODE_ADA, 0x28a0, "RTX 4060 Laptop" },
    { 0x0000E2A8, DECODE_ADA, 0x2203, "RTX 3090 Ti" },
    { 0x0000E2A8, DECODE_ADA, 0x2204, "RTX 3090" },
    { 0x0000E2A8, DECODE_ADA, 0x2208, "RTX 3080 Ti" },
    { 0x0000E2A8, DECODE_ADA, 0x2206, "RTX 3080" },
    { 0x0000E2A8, DECODE_ADA, 0x2216, "RTX 3080 LHR" },
    { 0x0000EE50, DECODE_ADA, 0x2484, "RTX 3070" },
    { 0x0000EE50, DECODE_ADA, 0x2488, "RTX 3070 LHR" },
    { 0x0000E2A8, DECODE_ADA, 0x2531, "RTX A2000" },
    { 0x0000E2A8, DECODE_ADA, 0x2571, "RTX A2000" },
    { 0x0000E2A8, DECODE_ADA, 0x24b0, "RTX A4000" },
    { 0x0000E2A8, DECODE_ADA, 0x2232, "RTX A4500" },
    { 0x0000E2A8, DECODE_ADA, 0x2231, "RTX A5000" },
    { 0x0000E2A8, DECODE_ADA, 0x26B1, "RTX 6000 Ada" },
    { 0x0000E2A8, DECODE_ADA, 0x27b8, "L4" },
    { 0x0000E2A8, DECODE_ADA, 0x26b9, "L40S" },
    { 0x0000E2A8, DECODE_ADA, 0x2236, "A10" },
    { 0x009A24C0, DECODE_GDDR_MRCODE, 0x2b85, "RTX 5090" },   /* GDDR7, hotspot module 0 */
};

struct memsensor {
    int ok, fd;
    volatile uint8_t *map;
    size_t pg;
    uint32_t in_page;
    enum decode decode;
    const char *name, *how;
};

static int decode_temp(enum decode d, uint32_t raw)
{
    if (d == DECODE_GDDR_MRCODE) {
        int code = (raw >> 16) & 0xFF; if (code > 80) code = 80;
        return code > 19 ? (code - 20) * 2 : -(40 - code * 2);
    }
    return (int)((raw & 0xfff) / 0x20);
}

/* Map the register page for the card at this PCI address ("0000:43:00.0"). */
static int memsensor_open(struct memsensor *s, const char *pci_addr, uint16_t dev_id)
{
    memset(s, 0, sizeof *s); s->fd = -1;
    const struct devinfo *di = NULL;
    for (size_t i = 0; i < sizeof dev_table / sizeof *dev_table; i++)
        if (dev_table[i].dev_id == dev_id) { di = &dev_table[i]; break; }
    if (!di) { fprintf(stderr, "device 0x%04x is not in the memory-sensor table (add its line from gddr6's); core curve only\n", dev_id); return -1; }
    s->decode = di->decode; s->name = di->name;
    s->pg = (size_t)sysconf(_SC_PAGE_SIZE);
    uint64_t page_off = di->offset & ~((uint64_t)s->pg - 1);
    s->in_page = di->offset - (uint32_t)page_off;

    char path[128]; int err1 = 0, err2 = 0;
    snprintf(path, sizeof path, "/sys/bus/pci/devices/%s/resource0", pci_addr);
    s->fd = open(path, O_RDONLY | O_SYNC);
    if (s->fd >= 0) {
        s->map = mmap(NULL, s->pg, PROT_READ, MAP_SHARED, s->fd, (off_t)page_off);
        if (s->map != MAP_FAILED) { s->ok = 1; s->how = "sysfs resource0"; return 0; }
        err1 = errno; close(s->fd); s->fd = -1;
    } else err1 = errno;
    /* fallback: /dev/mem at the BAR0 physical address */
    snprintf(path, sizeof path, "/sys/bus/pci/devices/%s/resource", pci_addr);
    FILE *f = fopen(path, "r"); unsigned long long bar0 = 0;
    if (f) { if (fscanf(f, "%llx", &bar0) != 1) bar0 = 0; fclose(f); }
    if (bar0) {
        s->fd = open("/dev/mem", O_RDONLY | O_SYNC);
        if (s->fd >= 0) {
            s->map = mmap(NULL, s->pg, PROT_READ, MAP_SHARED, s->fd, (off_t)(bar0 + page_off));
            if (s->map != MAP_FAILED) { s->ok = 1; s->how = "/dev/mem"; return 0; }
            err2 = errno; close(s->fd); s->fd = -1;
        } else err2 = errno;
    }
    fprintf(stderr, "cannot read the memory-temperature register: resource0: %s; /dev/mem: %s\n",
            strerror(err1), bar0 ? strerror(err2) : "BAR0 address unknown");
    if (geteuid() != 0)
        fprintf(stderr, "  you are not root. Both halves need it: the register read (CAP_SYS_RAWIO) and the fan\n"
                        "  override (NVML refuses it unprivileged). Run with sudo, or install the systemd unit.\n");
    else if (err2 == EPERM)
        fprintf(stderr, "  /dev/mem is locked down (CONFIG_STRICT_DEVMEM); the sysfs route should have worked —\n"
                        "  check that %s exists and that no other process holds it exclusively.\n", pci_addr);
    fprintf(stderr, "  continuing on the core curve only.\n");
    return -1;
}

static int memsensor_read(struct memsensor *s)
{
    if (!s->ok) return -1;
    uint32_t raw = *(volatile uint32_t *)(s->map + s->in_page);
    if (raw == 0xffffffffu) return -1;
    int t = decode_temp(s->decode, raw);
    return (t > 0 && t < 130) ? t : -1;
}

static void memsensor_close(struct memsensor *s)
{
    if (s->ok) munmap((void *)s->map, s->pg);
    if (s->fd >= 0) close(s->fd);
    s->ok = 0; s->fd = -1;
}

/* ------------------------------------------------------------- the curve --- */
struct curve { int n; double t[16], f[16]; char spec[200]; };

static int parse_curve(struct curve *c, const char *spec)
{
    struct curve tmp; memset(&tmp, 0, sizeof tmp); const char *p = spec;
    while (*p && tmp.n < 16) {
        char *end; double t = strtod(p, &end);
        if (end == p || *end != ':') return -1;
        double f = strtod(end + 1, &end);
        if (tmp.n && t <= tmp.t[tmp.n - 1]) return -1;        /* must be increasing */
        if (f < 0 || f > 100) return -1;
        tmp.t[tmp.n] = t; tmp.f[tmp.n] = f; tmp.n++;
        if (*end == ',') end++; else if (*end) return -1;
        p = end;
    }
    if (tmp.n < 2) return -1;
    snprintf(tmp.spec, sizeof tmp.spec, "%s", spec);
    *c = tmp; return 0;
}

static double interp(const struct curve *c, double t)
{
    if (t <= c->t[0]) return c->f[0];
    if (t >= c->t[c->n - 1]) return c->f[c->n - 1];
    for (int i = 0; i + 1 < c->n; i++)
        if (t >= c->t[i] && t <= c->t[i + 1])
            return c->f[i] + (c->f[i + 1] - c->f[i]) * (t - c->t[i]) / (c->t[i + 1] - c->t[i]);
    return c->f[c->n - 1];
}

/* ------------------------------------------------------------- settings --- */
struct settings { struct curve mem, core; int fmin, fmax, step; double interval; };

static int settings_valid(const struct settings *s)
{
    return s->fmin >= 0 && s->fmax <= 100 && s->fmin <= s->fmax && s->step >= 1 && s->interval >= 0.5;
}

/* Parse "--mem X --core Y --min a --max b --step c --interval d" from an argv-like list.
   Used for the command line and for the text a client sends to the daemon. Returns the
   number of settings consumed, -1 on a bad value. Unknown words are left for the caller. */
static int apply_settings(struct settings *s, int argc, char **argv, int *consumed)
{
    int n = 0;
    for (int i = 0; i < argc; i++) {
        const char *a = argv[i], *v = (i + 1 < argc) ? argv[i + 1] : NULL;
        consumed[i] = 0;
        if (!v) continue;
        if      (!strcmp(a, "--mem"))      { if (parse_curve(&s->mem, v))  return -1; }
        else if (!strcmp(a, "--core"))     { if (parse_curve(&s->core, v)) return -1; }
        else if (!strcmp(a, "--min"))      s->fmin = atoi(v);
        else if (!strcmp(a, "--max"))      s->fmax = atoi(v);
        else if (!strcmp(a, "--step"))     s->step = atoi(v);
        else if (!strcmp(a, "--interval")) s->interval = atof(v);
        else continue;
        consumed[i] = consumed[i + 1] = 1; i++; n++;
    }
    return settings_valid(s) ? n : -1;
}

/* ------------------------------------------------------------ the socket --- */
static int control_connect(void)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0); if (fd < 0) return -1;
    struct sockaddr_un a; memset(&a, 0, sizeof a); a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", SOCK_PATH);
    if (connect(fd, (struct sockaddr *)&a, sizeof a)) { close(fd); return -1; }
    return fd;
}

/* client: send one line, print the reply */
static int control_send(const char *line)
{
    int fd = control_connect();
    if (fd < 0) { fprintf(stderr, "no running gpufand (%s: %s)\n", SOCK_PATH, strerror(errno)); return 1; }
    if (write(fd, line, strlen(line)) < 0 || write(fd, "\n", 1) < 0) { perror("write"); close(fd); return 1; }
    char buf[1024]; ssize_t n;
    while ((n = read(fd, buf, sizeof buf)) > 0) fwrite(buf, 1, (size_t)n, stdout);
    close(fd); return 0;
}

static int control_listen(void)
{
    unlink(SOCK_PATH);
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0); if (fd < 0) return -1;
    struct sockaddr_un a; memset(&a, 0, sizeof a); a.sun_family = AF_UNIX;
    snprintf(a.sun_path, sizeof a.sun_path, "%s", SOCK_PATH);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) || listen(fd, 4)) { close(fd); return -1; }
    chmod(SOCK_PATH, 0600);                       /* root only, like the fans */
    return fd;
}

/* single instance: an flock on the pidfile, held for the daemon's life */
static int pidfile_fd = -1;
static int take_pidfile(void)
{
    pidfile_fd = open(PID_PATH, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (pidfile_fd < 0) return -1;
    if (flock(pidfile_fd, LOCK_EX | LOCK_NB)) { close(pidfile_fd); pidfile_fd = -1; return -1; }
    if (ftruncate(pidfile_fd, 0) == 0) dprintf(pidfile_fd, "%d\n", getpid());
    return 0;
}

/* ---------------------------------------------------------------- main --- */
static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

static nvmlDevice_t g_dev; static unsigned int g_nfans;
static void hand_back(void) { for (unsigned int i = 0; i < g_nfans; i++) p_SetDefaultFan(g_dev, i); }

static void usage(const char *a0)
{
    printf("usage: %s [options]            start the daemon (or, if one runs, talk to it)\n"
           "  --mem  T:F,T:F,... memory-temperature curve, °C:fan%%  (default 70:40,80:52,88:62,94:72,100:85)\n"
           "  --core T:F,T:F,... core-temperature curve, a floor       (default 50:38,60:48,68:60,75:72,85:85)\n"
           "  --min P --max P    clamp fan%% (default 30..90)\n"
           "  --interval S       seconds between updates (default 3)\n"
           "  --step P           max fan%% change per update (default 4)\n"
           "  --gpu N            NVML index of the card (default 0; daemon start only)\n"
           "  --quiet            no per-loop status line (for a service)\n"
           "  --once             print temperatures and what the curve would set, then exit\n"
           "control of a running daemon:\n"
           "  --status           show temperatures, fan, curves, pause state\n"
           "  --pause [SECONDS]  hand the fans back to the driver, indefinitely or for a while\n"
           "  --resume           take them back\n"
           "  --stop             exit gracefully (fans handed back)\n"
           "  --reset            hand the fans back to the driver now (after an unclean exit; no daemon needed)\n", a0);
}

static void format_status(char *out, size_t n, const struct settings *s, unsigned core, int mem,
                          int fan, int paused, time_t pause_until, const char *name, const struct memsensor *ms)
{
    char p[96] = "running";
    if (paused && pause_until) {
        long left = (long)(pause_until - time(NULL)); if (left < 0) left = 0;
        snprintf(p, sizeof p, "PAUSED (fans with the driver), resumes in %ld s", left);
    } else if (paused) snprintf(p, sizeof p, "PAUSED (fans with the driver) until --resume");
    snprintf(out, n, "%s  sensor: %s%s%s\ncore %u°C  mem %d°C  fan %d%%  %s\nmem curve  %s\ncore curve %s\nmin %d  max %d  step %d  interval %.1f\n",
             name, ms->ok ? ms->name : "none", ms->ok ? " via " : "", ms->ok ? ms->how : "",
             core, mem, fan, p, s->mem.spec, s->core.spec, s->fmin, s->fmax, s->step, s->interval);
}

int main(int argc, char **argv)
{
    struct settings st; memset(&st, 0, sizeof st);
    parse_curve(&st.mem,  "70:40,80:52,88:62,94:72,100:85");
    parse_curve(&st.core, "50:38,60:48,68:60,75:72,85:85");
    st.fmin = 30; st.fmax = 90; st.step = 4; st.interval = 3.0;

    /* settings first (both for a fresh start and for passing to a running daemon) */
    int consumed[256] = {0}; if (argc > 256) argc = 256;
    int nset = apply_settings(&st, argc - 1, argv + 1, consumed + 1);
    if (nset < 0) { fprintf(stderr, "bad curve or limits (curves: increasing °C:fan%% pairs, fan 0..100)\n"); return 2; }

    unsigned int gpu = 0; int once = 0, quiet = 0, want_status = 0, want_pause = 0, want_resume = 0, want_stop = 0, want_reset = 0;
    long pause_secs = 0;
    for (int i = 1; i < argc; i++) {
        if (consumed[i]) continue;
        const char *a = argv[i], *v = (i + 1 < argc) ? argv[i + 1] : NULL;
        if      (!strcmp(a, "--gpu") && v) { gpu = (unsigned)atoi(v); i++; }
        else if (!strcmp(a, "--once"))   once = 1;
        else if (!strcmp(a, "--quiet"))  quiet = 1;
        else if (!strcmp(a, "--status")) want_status = 1;
        else if (!strcmp(a, "--resume")) want_resume = 1;
        else if (!strcmp(a, "--stop"))   want_stop = 1;
        else if (!strcmp(a, "--reset"))  want_reset = 1;
        else if (!strcmp(a, "--pause"))  { want_pause = 1; if (v && v[0] != '-') { pause_secs = atol(v); i++; } }
        else { usage(argv[0]); return !!strcmp(a, "--help") && !!strcmp(a, "-h"); }
    }

    /* ---- --reset: hand the fans back, daemon or no daemon ---- */
    if (want_reset) {
        int probe = control_connect();
        if (probe >= 0) { close(probe); fprintf(stderr, "a daemon is running; stopping it hands the fans back:\n"); return control_send("stop"); }
        if (nvml_load()) return 1;
        nvmlReturn_t r = p_nvmlInit(); if (r) { fprintf(stderr, "nvmlInit: %s\n", p_ErrorString(r)); return 1; }
        if ((r = p_GetHandle(gpu, &g_dev))) { fprintf(stderr, "no GPU %u: %s\n", gpu, p_ErrorString(r)); return 1; }
        if ((r = p_GetNumFans(g_dev, &g_nfans))) { fprintf(stderr, "fans: %s\n", p_ErrorString(r)); return 1; }
        int bad = 0;
        for (unsigned int i = 0; i < g_nfans; i++)
            if ((r = p_SetDefaultFan(g_dev, i))) { fprintf(stderr, "fan %u: %s%s\n", i, p_ErrorString(r), geteuid() ? " (needs root)" : ""); bad = 1; }
        if (!bad) printf("fan control returned to the driver (%u fan%s)\n", g_nfans, g_nfans == 1 ? "" : "s");
        p_nvmlShutdown(); return bad;
    }

    /* ---- client mode: a daemon is already running ---- */
    int probe = control_connect();
    if (probe >= 0) {
        close(probe);
        if (once) { fprintf(stderr, "a daemon is running; use --status\n"); return 1; }
        char line[512] = ""; int rc = 0;
        if (nset > 0) {                                  /* pass the new settings */
            snprintf(line, sizeof line, "set --mem %s --core %s --min %d --max %d --step %d --interval %.2f",
                     st.mem.spec, st.core.spec, st.fmin, st.fmax, st.step, st.interval);
            rc |= control_send(line);
        }
        if (want_pause)  { snprintf(line, sizeof line, "pause %ld", pause_secs); rc |= control_send(line); }
        if (want_resume) rc |= control_send("resume");
        if (want_stop)   rc |= control_send("stop");
        if (want_status || (!nset && !want_pause && !want_resume && !want_stop)) {
            if (!nset && !want_pause && !want_resume && !want_stop && !want_status)
                fprintf(stderr, "gpufand is already running; not starting a second one. Its state:\n");
            rc |= control_send("status");
        }
        return rc;
    }
    if (want_status || want_pause || want_resume || want_stop) {
        fprintf(stderr, "no running gpufand to control (%s not answering)\n", SOCK_PATH); return 1;
    }

    /* ---- daemon (or --once) ---- */
    if (nvml_load()) return 1;
    nvmlReturn_t r = p_nvmlInit();
    if (r) { fprintf(stderr, "nvmlInit: %s\n", p_ErrorString(r)); return 1; }
    if ((r = p_GetHandle(gpu, &g_dev))) { fprintf(stderr, "no GPU %u: %s\n", gpu, p_ErrorString(r)); return 1; }
    char name[96] = "?"; p_GetName(g_dev, name, sizeof name);
    nvmlPciInfo_t pci; memset(&pci, 0, sizeof pci);
    if ((r = p_GetPciInfo(g_dev, &pci))) { fprintf(stderr, "PCI info: %s\n", p_ErrorString(r)); return 1; }
    char pci_addr[32]; snprintf(pci_addr, sizeof pci_addr, "%04x:%02x:%02x.0", pci.domain & 0xffff, pci.bus, pci.device);
    uint16_t dev_id = (uint16_t)(pci.pciDeviceId >> 16);
    if ((r = p_GetNumFans(g_dev, &g_nfans)) || g_nfans == 0) { fprintf(stderr, "no controllable fans\n"); return 1; }

    struct memsensor ms; memsensor_open(&ms, pci_addr, dev_id);
    unsigned int core = 0, fan0 = 0; p_GetFanSpeed(g_dev, 0, &fan0);

    if (once) {
        p_GetTemp(g_dev, 0, &core); int mem = memsensor_read(&ms);
        double want = interp(&st.core, core); if (mem > 0 && interp(&st.mem, mem) > want) want = interp(&st.mem, mem);
        printf("%s (0x%04x, pci %s)  sensor: %s\ncore %u°C  mem %d°C  fan now %u%%  curve asks %d%%\n",
               name, dev_id, pci_addr, ms.ok ? ms.how : "none (core only)", core, mem, fan0, (int)(want + 0.5));
        if (geteuid() != 0) printf("(not root: the fan override would be refused; --once touches nothing)\n");
        memsensor_close(&ms); p_nvmlShutdown(); return 0;
    }

    if (geteuid() != 0) { fprintf(stderr, "the fan override needs root (NVML refuses it unprivileged); run with sudo\n"); return 1; }
    if (take_pidfile()) { fprintf(stderr, "another gpufand holds %s but does not answer on %s — kill it or remove the stale files\n", PID_PATH, SOCK_PATH); return 1; }
    int lfd = control_listen();
    if (lfd < 0) { fprintf(stderr, "cannot listen on %s: %s\n", SOCK_PATH, strerror(errno)); return 1; }

    if (!quiet) printf("%s (0x%04x, pci %s)  fans=%u  memory sensor: %s%s%s  starting at %u%%\n", name, dev_id, pci_addr,
                       g_nfans, ms.ok ? ms.name : "none (core curve only)", ms.ok ? " via " : "", ms.ok ? ms.how : "", fan0);

    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGHUP, on_signal); signal(SIGPIPE, SIG_IGN);
    int failures = 0, cur = (int)fan0, paused = 0, mem = -1; time_t pause_until = 0;
    struct timespec last = {0, 0};
    while (!stop_flag) {
        /* ---- control: wait up to the interval for a client, serve it, then continue ---- */
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        double since = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1e9;
        double wait = st.interval - since; if (wait < 0) wait = 0;
        struct pollfd pf = { lfd, POLLIN, 0 };
        if (poll(&pf, 1, (int)(wait * 1000)) > 0) {
            int c = accept(lfd, NULL, NULL);
            if (c >= 0) {
                char buf[600] = {0}; ssize_t n = read(c, buf, sizeof buf - 1); char reply[700] = "";
                if (n > 0) {
                    buf[strcspn(buf, "\n")] = 0;
                    if (!strncmp(buf, "set ", 4)) {
                        char *words[40]; int wc = 0; char *tok = strtok(buf + 4, " ");
                        while (tok && wc < 40) { words[wc++] = tok; tok = strtok(NULL, " "); }
                        struct settings ns = st; int cons[40] = {0};
                        if (apply_settings(&ns, wc, words, cons) < 0) snprintf(reply, sizeof reply, "rejected: bad curve or limits\n");
                        else { st = ns; snprintf(reply, sizeof reply, "settings updated\n");
                               if (quiet) fprintf(stderr, "settings updated: mem %s core %s min %d max %d step %d interval %.1f\n",
                                                  st.mem.spec, st.core.spec, st.fmin, st.fmax, st.step, st.interval); }
                    } else if (!strncmp(buf, "pause", 5)) {
                        long secs = atol(buf + 5); paused = 1; pause_until = secs > 0 ? time(NULL) + secs : 0;
                        hand_back();
                        snprintf(reply, sizeof reply, secs > 0 ? "paused for %ld s; fans with the driver\n" : "paused until --resume; fans with the driver\n", secs);
                        if (quiet) fputs(reply, stderr);
                    } else if (!strcmp(buf, "resume")) {
                        paused = 0; pause_until = 0; p_GetFanSpeed(g_dev, 0, &fan0); cur = (int)fan0;
                        snprintf(reply, sizeof reply, "resumed\n"); if (quiet) fputs(reply, stderr);
                    } else if (!strcmp(buf, "stop")) {
                        stop_flag = 1; snprintf(reply, sizeof reply, "stopping; fans handed back\n");
                    } else {                                  /* status */
                        format_status(reply, sizeof reply, &st, core, mem, paused ? -1 : cur, paused, pause_until, name, &ms);
                        if (paused) { unsigned f; if (!p_GetFanSpeed(g_dev, 0, &f)) { char *p = strstr(reply, "fan -1%"); if (p) { char t[16]; snprintf(t, sizeof t, "fan %2u%%", f); memcpy(p, t, strlen(t)); } } }
                    }
                }
                if (write(c, reply, strlen(reply)) < 0) { /* client gone */ }
                close(c);
            }
            if (since < st.interval) continue;             /* served early; not yet time for a fan step */
        }
        clock_gettime(CLOCK_MONOTONIC, &last);

        /* ---- the fan step ---- */
        if (paused) {
            if (pause_until && time(NULL) >= pause_until) { paused = 0; pause_until = 0; p_GetFanSpeed(g_dev, 0, &fan0); cur = (int)fan0; if (quiet) fputs("pause elapsed; resumed\n", stderr); }
            else { if (!quiet) { printf("\rpaused (driver curve)                 "); fflush(stdout); } continue; }
        }
        mem = memsensor_read(&ms);
        if (p_GetTemp(g_dev, 0, &core)) { if (++failures >= MAX_READ_FAILURES) { fprintf(stderr, "\ntemperature unreadable; handing back\n"); break; } continue; }
        failures = 0;
        double want = interp(&st.core, core);
        if (mem > 0) { double w = interp(&st.mem, mem); if (w > want) want = w; }
        int target = (int)(want + 0.5); if (target < st.fmin) target = st.fmin; if (target > st.fmax) target = st.fmax;
        if (target > cur) cur = (cur + st.step < target) ? cur + st.step : target;
        else if (target < cur) cur = (cur - st.step > target) ? cur - st.step : target;
        for (unsigned int i = 0; i < g_nfans; i++)
            if ((r = p_SetFanSpeed(g_dev, i, (unsigned)cur))) {
                fprintf(stderr, "\nset fan %u: %s — handing back\n", i, p_ErrorString(r)); stop_flag = 1; break;
            }
        if (!quiet) { if (mem > 0) printf("\rcore %3u°C  mem %3d°C  fan %3d%%   ", core, mem, cur);
                      else         printf("\rcore %3u°C  mem  --°C  fan %3d%%   ", core, cur);
                      fflush(stdout); }
    }
    hand_back();
    if (!quiet) printf("\nfan control returned to the driver\n");
    close(lfd); unlink(SOCK_PATH); if (pidfile_fd >= 0) { unlink(PID_PATH); close(pidfile_fd); }
    memsensor_close(&ms); p_nvmlShutdown();
    return 0;
}
