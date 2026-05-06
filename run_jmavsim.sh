#!/usr/bin/env bash
set -euo pipefail

export JAVA_HOME=$(/usr/libexec/java_home -v 11 -a x86_64)
export PATH="$JAVA_HOME/bin:$PATH"

arch -x86_64 make px4_sitl jmavsim
