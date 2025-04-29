echo "=Disabilita risparmio energetico e screen saver"
echo "============================================================================================"

gsettings set org.mate.power-manager sleep-display-ac 0
gsettings set org.mate.power-manager sleep-display-battery 0
gsettings set org.mate.power-manager sleep-computer-ac 0
gsettings set org.mate.power-manager sleep-computer-battery 0
gsettings set org.mate.power-manager idle-dim-ac false
gsettings set org.mate.power-manager idle-dim-battery false
gsettings set org.mate.screensaver idle-activation-enabled false
gsettings set org.mate.screensaver lock-enabled false
xset s off
xset -dpms
xset s noblank
