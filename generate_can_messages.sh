#!/bin/bash

DBC="../dbc/can2.dbc"
MESSAGE="./src/can_management/"

if [[ -n "$1" ]]; then
    DBC="$1"
fi

dbc-codegen $DBC $MESSAGE
