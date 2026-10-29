#!/bin/bash
# install_autostart.sh : service limo-guide = guide_start.sh at power-on (robot on the home mark).
# Installs only; switch on with: sudo systemctl enable limo-guide
set -e
sudo tee /etc/systemd/system/limo-guide.service > /dev/null <<'UNIT'
[Unit]
Description=Limo lab guide robot (base, AMCL, Nav2, guide manager)
After=network-online.target
Wants=network-online.target

[Service]
User=agilex
Environment=HOME=/home/agilex PYTHONUNBUFFERED=1
ExecStartPre=/bin/bash /home/agilex/guide_ws/scripts/wait_wifi.sh
ExecStart=/bin/bash /home/agilex/guide_ws/scripts/guide_start.sh
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=15

[Install]
WantedBy=multi-user.target
UNIT
C=""; for b in /usr/bin/systemctl /bin/systemctl; do for a in start stop restart; do C="$C${C:+, }$b $a limo-guide"; done; done
echo "agilex ALL=(root) NOPASSWD: $C" | sudo tee /etc/sudoers.d/limo-guide > /dev/null
sudo chmod 440 /etc/sudoers.d/limo-guide
sudo visudo -cf /etc/sudoers.d/limo-guide
sudo systemctl daemon-reload
echo "limo-guide installed (not enabled). Start now: sudo systemctl start limo-guide   At boot: sudo systemctl enable limo-guide"
