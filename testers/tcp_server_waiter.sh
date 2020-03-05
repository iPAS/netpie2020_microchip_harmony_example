#!/bin/bash

i=0; while true; do netcat -l -p 1883 -w 1; i=$((i+1)); echo $i $(date +'%F %T'); done
