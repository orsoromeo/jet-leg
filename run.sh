
DOCKER_VOLUMES="
--volume="./src/:/app/src/" \
"

docker run -i -t ${DOCKER_VOLUMES} jet-leg:latest /bin/bash
