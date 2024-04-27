FROM ghcr.io/ad-sdl/wei

LABEL org.opencontainers.image.source=https://github.com/byeongdulee/UR_12idb
LABEL org.opencontainers.image.description="Drivers and REST API's for the UR5 robots of APS-XSD-CMS group"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p ur_12idb

COPY ./common ur_12idb/common
COPY ./urxe ur_12idb/urxe
COPY ./rtde ur_12idb/rtde
COPY ./urscripts ur_12idb/urscripts
COPY ./images ur_12idb/images
COPY ./README.md ur_12idb/README.md
COPY ./pyproject.toml ur_12idb/pyproject.toml
COPY ./robot12idb.py ur_12idb/robot12idb.py
COPY ./ur_rest_node.py ur_12idb/ur_rest_node.py

# RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./ur_12idb

CMD ["python", "ur_12idb/ur_rest_node.py"]

#########################################
