#!/bin/bash

message="FF"
register=20000
topic="%40msg%2Fdev_rtu%2Fupdate%2F${register}"
client_id="** Find it in NETPIE-2020 dashboard **"
token="** Find it in NETPIE-2020 dashboard **"

curl  \
    -X PUT "https://api.netpie.io/v2/device/message?topic=${topic}"  \
    -H "Authorization: Basic ${client_id}:${token}"  \
    -H "Content-Type: text/plain" -d "${message}"
