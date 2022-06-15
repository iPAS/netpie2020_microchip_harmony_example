#!/bin/bash

# sudo iptables-save | uniq | iptables-restore
# sudo iptables -A INPUT -i eth5 -j ACCEPT

WLAN=wlan0
LAN=eth2

sudo iptables -t nat -A POSTROUTING        -o ${WLAN}                                     -j MASQUERADE
sudo iptables        -A FORWARD -i ${WLAN} -o ${LAN} -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables        -A FORWARD -i ${LAN}                                                 -j ACCEPT

sudo iptables -L --line-number


# https://realtechtalk.com/iptables_how_to_log_ALL_dropped_incoming_packets-2133-articles
# sudo iptables -N LOGGING
# sudo iptables -A INPUT -i eth5 -j LOGGING

# sudo iptables -A LOGGING -j LOG --log-prefix  "ipt denied: " --log-level 4
# sudo iptables -A LOGGING -j DROP

# https://svennd.be/rsyslog-separate-file-for-logging/
