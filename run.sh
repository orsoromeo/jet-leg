
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume="./src/:/app/src/" \
"

DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
"

DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

docker run -i -t ${DOCKER_ARGS} jet-leg:latest /bin/bash
