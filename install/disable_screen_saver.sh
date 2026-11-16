#!/bin/bash

# Disabilita il risparmio energetico
gsettings set org.mate.power-manager sleep-display-ac 0
gsettings set org.mate.power-manager sleep-display-battery 0
gsettings set org.mate.power-manager sleep-computer-ac 0
gsettings set org.mate.power-manager sleep-computer-battery 0
gsettings set org.mate.power-manager idle-dim-ac false
gsettings set org.mate.power-manager idle-dim-battery false

# Disabilita lo screen saver
gsettings set org.mate.screensaver idle-activation-enabled false
gsettings set org.mate.screensaver lock-enabled false

# Disabilita il Power Management e lo screen saver via X
xset s off
xset -dpms
xset s noblank