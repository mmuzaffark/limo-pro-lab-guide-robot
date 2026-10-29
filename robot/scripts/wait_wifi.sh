#!/bin/bash
# wait up to 120 s for wlan0 to get an IPv4 address (Cyclone DDS is pinned to wlan0)
for i in $(seq 60); do ip -4 addr show wlan0 | grep -q inet && exit 0; sleep 2; done
exit 0
