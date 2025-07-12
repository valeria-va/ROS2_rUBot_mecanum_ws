#!/bin/bash

set -e

echo "=== Creant servei systemd: my_robot.service ==="

SERVICE_PATH="/etc/systemd/system/my_robot.service"
SCRIPT_PATH="/home/ubuntu/start_robot.sh"

# Crear fitxer del servei amb suport per pantalla gràfica
sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=rubot_bringup_service
Wants=network-online.target
After=network-online.target
Requires=sys-subsystem-net-devices-wlan0.device
After=sys-subsystem-net-devices-wlan0.device

[Service]
ExecStart=$SCRIPT_PATH
WorkingDirectory=/home/ubuntu
User=ubuntu
Restart=on-failure
Environment="DISPLAY=:0"
Environment="XDG_RUNTIME_DIR=/run/user/1000"

[Install]
WantedBy=multi-user.target
EOF

echo "=== Assegurant permisos del script robot ==="
chmod +x "$SCRIPT_PATH"

echo "=== Recarregant i activant el servei ==="
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable my_robot.service

echo "✅ Servei creat i activat correctament!"
echo "Pots iniciar-lo ara amb: sudo systemctl start my_robot.service"
echo "Pots veure l'estat amb: systemctl status my_robot.service"
