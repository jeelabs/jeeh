#!/bin/sh

# Collect all the IRQ handler names in all the installed CMSIS packages in PIO.
# This will only be a complete list if all CMSIS families are actually present.
# The output should be saved as "irqs.h" header and is included from "mcu.cpp".

for i in ~/.platformio/packages/framework-cmsis-*/Source/Templates/gcc/
do
    echo $i >&2
    cd $i
    grep -h '\.word.*Handler' *.s | awk '{ print "IRQ(", $2, ")" }'
done | sort -u
