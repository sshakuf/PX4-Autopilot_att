
# CLAUDE.md

## Project
This is a PX4 (fork) horizontal drone, this drone is only moving in the x,y direction (meaning it does not have up/down since it's hangs on a rope).

The main goal of the project is for precice landing and stabilization.
I am using it in a GPS denied area, so we are using optical flow.


## Stack
I am running on micoair h743 v2

## Commands

- Build: `make micoair_h743-v2_default`

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

