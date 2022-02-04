#!/bin/bash
for i in $(datamash mean 1 median 1 max 1 < "$1")
do
    echo -n "|$(awk '{printf "%.3f", $1}' <<< $i)"
done
