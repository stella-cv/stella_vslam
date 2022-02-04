#!/bin/bash
result=$(evo_rpe "$@")
max=$(grep max <<< "$result" | awk '{printf "%.3f", $2}')
mean=$(grep mean <<< "$result" | awk '{printf "%.3f", $2}')
median=$(grep median <<< "$result" | awk '{printf "%.3f", $2}')
rmse=$(grep rmse <<< "$result" | awk '{printf "%.3f", $2}')
sse=$(grep sse <<< "$result" | awk '{printf "%.3f", $2}')
std=$(grep std <<< "$result" | awk '{printf "%.3f", $2}')
echo -n "| $mean | $median | $max | $rmse | $sse | $std "
