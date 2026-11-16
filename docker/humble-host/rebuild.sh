 #!/bin/bash
#set -x  # Abilita il debug
 
#docker build -t marrtinorobot2:mate-vnc -f Dockerfile.mate-vnc .
docker build \
  --no-cache \
  --build-arg REPOSITORY_CACHEBUST="$(date +%s)" \
  -t marrtinorobot2:mate-vnc \
  -f Dockerfile.mate-vnc .
