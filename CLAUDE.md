
# CLAUDE.md

## Project
This is a PX4 (fork) horizontal drone, this drone is only moving in the x,y direction (meaning it does not have up/down since it's hangs on a rope).

The main goal of the project is for precice landing and stabilization.
I am using it in a GPS denied area, so we are using optical flow.


## Stack
I am running on micoair h743 v2

## Commands

- Build: `make micoair_h743-v2_default`
- Flash: `make micoair_h743-v2_default upload`

### Gotcha: adding a NEW parameters file needs a CMake reconfigure

The parameter scanner globs `*params*.c` at **CMake configure time**, not build
time. Create a new `..._params.c` and the build fails with:

```
error: 'DF_MY_PARAM' is not a member of 'px4::params'
```

The file is fine — CMake just has not re-globbed. Force it:

```
touch src/modules/mc_pos_control/CMakeLists.txt   # any CMakeLists in that module
make micoair_h743-v2_default
```

Adding a param to an **existing** params file does not need this. Only new files.
Confirm it landed with:
`grep -c DF_MY_PARAM build/micoair_h743-v2_default/src/lib/parameters/px4_parameters.hpp`

## Log locations

Do not search for logs. They are always in one of these two places:

- **QGroundControl downloads** (default, `~` = `/Users/sshakuf`):
  `~/Documents/QGroundControl Daily/Logs/`
  Named `log_<n>_<YYYY-M-D-HH-MM-SS>.ulg`, timestamped from the host clock.
- **webui downloads** (pulled over MAVLink by `TestScripts/webui/server.py`):
  `TestScripts/webui/downloads/`
  Named `log_<id>_<utc>.ulg`. The date reads `2000-01-01` because the vehicle is
  GPS-denied so `time_utc` is ~0 — use the file mtime for the real time.

"the latest log" means the newest file by mtime across both directories:
`ls -t ~/Documents/QGroundControl\ Daily/Logs/*.ulg TestScripts/webui/downloads/*.ulg | head -1`

Analysis: `pyulog` is installed; `ulog_info` is at `~/.local/bin/ulog_info`.
Read with `pyulog.ULog(path)`; parameters via `u.initial_parameters` and
`u.changed_parameters` (the latter matters — features get toggled mid-flight).

## Architecture


## Rules


## Workflow



## Out of scope
- [Things Claude should not touch]
- [Files that are manually maintained]
- [Integrations Claude shouldn't modify]

# The lines with highest impact:

- IMPORTANT: run type check after every code change
  (prevents  from shipping broken types)

- Make minimal changes, don't refactor unrelated code
  (prevents  from rewriting your entire file)

- Create separate commits per logical change
  (prevents the 47-file monster commit)

- When unsure, explain both approaches and let me choose
  (prevents  from making architectural decisions for you)

