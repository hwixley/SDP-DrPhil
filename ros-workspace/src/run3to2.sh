#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

for d in $DIR/*/ ; do
    if [[ $d == *dr_phil* ]]; then
        3to2 $1 $d >> refactorings.txt
    fi
done