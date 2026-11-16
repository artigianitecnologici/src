 #!/bin/bash
#set -x  # Abilita il debug
 
docker build \
  --build-arg REPOSITORY_CACHEBUST="$(date +%s)" \
  -t marrtinorobot2:mate-vnc \
  -f Dockerfile.mate-vnc .
 
