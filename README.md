# gpufand

----
Written by Claude (Anthropic). I take the responsibility for the code and hold the intellectual
property rights. The credit however goes to Claude, deservedly - Grigor Gatchev.
----

A ~300-line C daemon that runs a fan curve on an NVIDIA card keyed to the **memory-junction
temperature** (GDDR6X on the 3080/3090/40-series), which the driver does not expose on GeForce
cards and which is usually the hottest thing on the board. The core temperature acts as a floor.

    fan% = max( mem_curve(memory °C), core_curve(core °C) ),  rate-limited (default 4 %/3 s)

* No dependencies beyond libc. NVML (`libnvidia-ml.so.1`, part of the driver) is loaded at run
  time, so no CUDA toolkit is needed to build. Driver ≥ 520.
* The memory temperature is read from one register in the card's BAR0, the way
  [gddr6](https://github.com/olealgoritme/gddr6) does it (device table and decode borrowed from
  there, MIT) — but via `/sys/bus/pci/devices/<addr>/resource0`, which needs no `iomem=relaxed`
  kernel parameter. `/dev/mem` is the fallback.
* Needs root (both halves). Works headless; no X, no Coolbits.
* Fails safe: on SIGINT/SIGTERM, on an NVML write error, or after 10 consecutive failed
  temperature reads, it hands the fans back to the driver. If it is killed outright, the driver's
  curve resumes on its own, because the override only persists while something keeps writing it.

## Build and try

    cc -O2 -Wall -o gpufand gpufand.c -ldl
    sudo ./gpufand --once          # prints core/memory temperature and what the curve would set
    sudo ./gpufand                 # runs; Ctrl-C hands back

## One instance, controlled from the command line

Only one daemon runs per machine (an flock on `/run/gpufand.pid`). While it runs, any further
`gpufand` invocation talks to it over `/run/gpufand.sock` instead of starting a second copy:

    sudo gpufand --status                       temperatures, fan, curves, pause state
    sudo gpufand --mem 70:40,84:48,90:56,96:68,100:85   new curve/limits, applied live
    sudo gpufand --pause 1800                   fans back to the driver for 30 min (then resumes)
    sudo gpufand --pause                        ... until --resume
    sudo gpufand --resume
    sudo gpufand --stop                         graceful exit, fans handed back

A bare `sudo gpufand` while one is running prints its status and does not start another.
If the memory sensor cannot be read, the start-up message says why (not root, locked-down
/dev/mem, unknown card) and what to do; the daemon then runs on the core curve alone.

## Options

    --gpu N              NVML index (default 0)
    --mem  T:F,T:F,...   memory curve, °C:fan%   default 70:40,80:52,88:62,94:72,100:85
    --core T:F,T:F,...   core curve (floor)      default 50:38,60:48,68:60,75:72,85:85
    --min P --max P      clamp (default 30..90)
    --interval S         seconds between updates (default 3)
    --step P             max change per update (default 4)
    --once / --quiet

A quieter curve for daytime, e.g. `--mem 70:40,84:48,90:56,96:68,100:85`, holds a 3090's memory
around 88 °C at little more than the stock fan speed; the default holds it in the mid-80s.

## Install as a service

    sudo cp gpufand /usr/local/sbin/
    sudo cp gpufand.service /etc/systemd/system/
    sudo systemctl daemon-reload && sudo systemctl enable --now gpufand
    journalctl -u gpufand -n 20

Cards not in the table run on the core curve alone (the daemon says so at start). To add one,
copy its line from gddr6's table into `dev_table[]`.

## Authorship

The code was written by Claude (Anthropic's AI model) in collaboration with Grigor Gatchev, who
specified the tool, tested it on the hardware, found the bugs, and proposed the single-instance
and live-control design. The memory-sensor method, device table and decode come from
[gddr6](https://github.com/olealgoritme/gddr6) (MIT). Licence: MIT, see `LICENSE`.
