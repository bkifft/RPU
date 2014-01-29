#!/bin/bash

rm rpu 2>/dev/null

gcc -std=c99 main.c -o rpu

clear

stdbuf -e0 -o0 ./rpu | tee /boot/RPU_log.txt



