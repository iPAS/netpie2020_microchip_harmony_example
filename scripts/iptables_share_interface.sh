#!/bin/bash

LAN=$1
WAN=$2
[[ "${LAN}" == "" || "${WAN}" == "" ]] && echo "$0 LAN WAN -- e.g. -- $0 eth1 wlan0" && exit 255

ip link show ${LAN} >/dev/null
ret=$?
[[ "$ret" -ne 0 ]] && exit $ret
ip link show ${WAN} >/dev/null
ret=$?
[[ "$ret" -ne 0 ]] && exit $ret


# Without -t <> --> -t filter
# -m --match
# -j --jump
# -i --in-interface
# -o --out-interface

rule_1="POSTROUTING -t nat -o ${WAN} -j MASQUERADE"
rule_2="FORWARD -i ${WAN} -o ${LAN} -j ACCEPT -m state --state RELATED,ESTABLISHED"
rule_3="FORWARD -i ${LAN} -j ACCEPT"

rules=("${rule_1}" "${rule_2}" "${rule_3}")

for r in "${rules[@]}"; do
    echo "Rule: ${r}"
    sudo iptables -C ${r}
    # echo $?
    if [ $? -ne 0 ]; then
        sudo iptables -A ${r}
        echo "  Set! -- $?"
    else
        echo "  Already exists!"
    fi
    echo
done


ip_forward=/proc/sys/net/ipv4/ip_forward
[[ `cat $ip_forward` -eq 0 ]] && Echo "Please set $ip_forward to 1"

