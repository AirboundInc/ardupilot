#!/usr/bin/env bash

# script to build cygwin binaries for plane to upload to Google Drive
# the contents of artifacts directory is uploaded to:
# https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/

# the script assumes you start in the root of the ardupilot git tree

set -x
set -e

# TOOLCHAIN=i686-pc-cygwin
TOOLCHAIN=x86_64-pc-cygwin
GPP_COMPILER="${TOOLCHAIN}-g++"

$GPP_COMPILER -print-sysroot

SYS_ROOT=$($GPP_COMPILER -print-sysroot)
echo "SYS_ROOT=$SYS_ROOT"

rm -rf artifacts
mkdir artifacts

# cygwin doesn't work out the parallelism properly
WAF_OPTIONS="-j8"

(
    python ./waf --color yes --toolchain $TOOLCHAIN --board sitl configure 2>&1
    python ./waf plane $WAF_OPTIONS 2>&1
) | tee artifacts/build.txt

cp -v build/sitl/bin/arduplane artifacts/arduplane.exe

# Find all cyg*.dll files returned by cygcheck for each exe in artifacts
# and copy them over
for exe in artifacts/*.exe; do
    echo $exe
    cygcheck $exe | grep -oP 'cyg[^\s\\/]+\.dll' | while read -r line; do
        cp -v /usr/bin/$line artifacts/
    done
done

git log -1 > artifacts/git.txt
ls -l artifacts/