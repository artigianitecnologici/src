#!/bin/bash

# Abilita il debug per vedere eventuali errori
set -e

# Installa XRDP
sudo apt update
sudo apt install -y xrdp

# Modifica la configurazione per consentire accesso da console
sudo sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config

# Crea il file di configurazione per Color Manager Policy
sudo bash -c 'cat > /etc/polkit-1/localauthority/50-local.d/45-allow.colord.pkla <<EOL
[Allow Colord all Users]
Identity=unix-user:*
Action=org.freedesktop.color-manager.create-device;org.freedesktop.color-manager.create-profile;org.freedesktop.color-manager.delete-device;org.freedesktop.color-manager.delete-profile;org.freedesktop.color-manager.modify-device;org.freedesktop.color-manager.modify-profile
ResultAny=no
ResultInactive=no
ResultActive=yes
EOL'

# Crea il file di configurazione per Package Manager Policy
sudo bash -c 'cat > /etc/polkit-1/localauthority/50-local.d/46-allow-update-repo.pkla <<EOL
[Allow Package Management all Users]
Identity=unix-user:*
Action=org.freedesktop.packagekit.system-sources-refresh
ResultAny=yes
ResultInactive=yes
ResultActive=yes
EOL'

# Crea il file di configurazione per Network Manager WiFi Scan Policy
sudo bash -c 'cat > /etc/polkit-1/localauthority/50-local.d/47-allow-wifi-scan.pkla <<EOL
[Allow WiFi Scan all Users]
Identity=unix-user:*
Action=org.freedesktop.NetworkManager.wifi.scan
ResultAny=no
ResultInactive=no
ResultActive=yes
EOL'

# Configura la sessione Xorg per MATE
echo "mate-session" > ~/.xsession
chmod +x ~/.xsession

# Configura variabili di ambiente per MATE
cat > ~/.xsessionrc <<EOL
export XDG_SESSION_DESKTOP=mate
export XDG_DATA_DIRS=${XDG_DATA_DIRS}
export XDG_CONFIG_DIRS=/etc/xdg/xdg-mate:/etc/xdg
EOL

# Aggiusta il file xrdp-sesman per leggere variabili d'ambiente
sudo sed -i '1i session required pam_env.so readenv=1 user_readenv=0' /etc/pam.d/xrdp-sesman

# Configura lo script startwm.sh
sudo cp /etc/xrdp/startwm.sh /etc/xrdp/startwm.sh.bak
sudo bash -c 'cat > /etc/xrdp/startwm.sh <<EOL
#!/bin/sh

if test -r /etc/profile; then
    . /etc/profile
fi

if test -r /home/\${USER}/.profile; then
    . /home/\${USER}/.profile
fi

if test -r /etc/default/locale; then
    . /etc/default/locale
    test -z "\${LANG+x}" || export LANG
    test -z "\${LANGUAGE+x}" || export LANGUAGE
    test -z "\${LC_ALL+x}" || export LC_ALL
fi

unset DBUS_SESSION_BUS_ADDRESS
unset XDG_RUNTIME_DIR
mate-session
EOL'

sudo chmod +x /etc/xrdp/startwm.sh

# Riavvia il servizio XRDP
sudo systemctl restart xrdp

# Messaggio di completamento
echo "Configurazione XRDP per MATE su Ubuntu 22.04 completata con successo!"
