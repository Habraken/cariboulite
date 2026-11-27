#!/bin/bash
set -eu
cd /home/pi/src/cariboulite
expected=a997ec4b92e28fc79fe38b320654d334dae61619
for attempt in $(seq 1 240); do
    if [ "$(git rev-parse origin/dev-fpga-pa-control)" = "$expected" ]; then break; fi
    sleep 5
done
[ "$(git rev-parse origin/dev-fpga-pa-control)" = "$expected" ]
backup=build/pre-github-update-20261127
mkdir -p "$backup"
git status --porcelain=v1 > "$backup/status.txt"
git diff --binary > "$backup/local.patch"
if [ -n "$(git status --porcelain=v1)" ]; then
    git stash push -u -m 'Preserve bench changes before GitHub update 2026-11-27'
    git rev-parse refs/stash > "$backup/stash.txt"
fi
git merge --ff-only "$expected"
[ "$(git rev-parse HEAD)" = "$expected" ]
for f in build/radio-backend build/cariboulite_test_app build/cariboulite_util; do
    if [ -e "$f" ]; then cp -a "$f" "$backup/"; fi
done
cmake -S software/libcariboulite -B build/github-latest-20261127 -DCMAKE_BUILD_TYPE=Release
cmake --build build/github-latest-20261127 -j4
make -j4 -C software/libcariboulite/tools/radio_backend LIBDIR=/home/pi/src/cariboulite/build/github-latest-20261127
if [ -e /usr/local/lib/libcariboulite.so ]; then cp -L /usr/local/lib/libcariboulite.so "$backup/installed-libcariboulite.so"; fi
sudo -n cmake --install build/github-latest-20261127
sudo -n ldconfig
for name in cariboulite_test_app cariboulite_util; do
    if [ -e "build/github-latest-20261127/$name" ]; then ln -sfn "github-latest-20261127/$name" "build/$name"; fi
done
build/radio-backend/cariboulite_radio_backend --help > "$backup/backend-help.txt" 2>&1
git rev-parse HEAD > "$backup/updated-revision.txt"
printf 'UPDATE_COMPLETE %s\n' "$expected"
