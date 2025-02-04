
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume="./examples/:/app/examples/" \
--volume="./jet_leg/:/app/jet_leg/" \
--volume="./resources/:/app/resources/" \
--volume="./unit_tests/:/app/unit_tests/" \
"

DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="PYTHONPATH="/app/ \
"

DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

docker run -i -t ${DOCKER_ARGS} jet-leg:latest /bin/bash
