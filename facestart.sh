#!/bin/sh 
# echo "Launch Browser"
#DISPLAY=:0 midori -p -e Fullscreen -a http://localhost:8080/social/marrtina.html 
rm /home/marrtino/snap/chromium/common/chromium/SingletonLock 
chromium-browser --app=http://localhost/social/marrtina01.html --kiosk -start-maximized
