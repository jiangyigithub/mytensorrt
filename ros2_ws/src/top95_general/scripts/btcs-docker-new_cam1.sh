#!/usr/bin/env bash

function show_help() {
    cat <<END_OF_HELP
Usage: run-btcs-docker.sh [--gpu <index>] [-v <video dir>] <config file>

    -h, --help          display this help and exit
    -v <video dir>      when analyzing video files, specify the directory containing video files
    --gpu <index>		select GPU (0..N)

END_OF_HELP
    exit 0
}

# BTCS_IMAGE_INT=${BTCS_IMAGE:-btcs_cam1}
# BTCS_VERSION_INT=${BTCS_VERSION_CAM1:-0.0.0.0}
BTCS_IMAGE_INT=${BTCS_IMAGE:-btcs}
BTCS_VERSION_INT=${BTCS_VERSION:-1.0.0.1}
BTCS_PROJECT_NAME_INT=${BTCS_PROJECT_NAME:-btcs_cam1}
NVIDIA_DRIVER_VERSION=$(modinfo -F version nvidia)

# parse arguments
OPTIONS=$(getopt -o "v:" -l "help,gpu:" -- "$@")

eval set -- "${OPTIONS}"
while true; do
	case "$1" in
		-v)
			BTCS_VIDEO_DIR="$2"
			shift
			;;
		--gpu)
			BTCS_GPU_INDEX="$2"
			shift
			;;
		--help | -h)
			show_help
			;;
		--)
			shift
			break
			;;
	esac
	shift
done

# parse config file
BTCS_CONFIG_FILE=$(realpath -q $@ 2>/dev/null)
if [ ! -f "${BTCS_CONFIG_FILE}" ]; then
    echo -e "Error: No config file specified or config file not found!\n"
    show_help
fi

# create Docker configuration
if [ -s "${BTCS_VIDEO_DIR}" ]; then
    DOCKER_VIDEO_MAPPING="- ${BTCS_VIDEO_DIR}:/video"
else
    DOCKER_VIDEO_MAPPING=""
fi

if [ -z "${BTCS_GPU_INDEX+x}" ]; then
	DOCKER_GPU_SELECT=""
else
	DOCKER_GPU_SELECT="- CUDA_VISIBLE_DEVICES=${BTCS_GPU_INDEX}"
fi

echo "${DOCKER_GPU_SELECT}"
echo "${DOCKER_VIDEO_MAPPING}"
echo "${DOCKER_COMPOSE_YML}" 
echo "${BTCS_CONFIG_FILE}"
echo "${BTCS_PROJECT_NAME_INT}"
echo "Running BTCS (${BTCS_IMAGE_INT}:${BTCS_VERSION_INT}) ..."
#echo "${DOCKER_COMPOSE_YML}" | docker-compose -p ${BTCS_PROJECT_NAME_INT} -f - up
docker-compose -p my.yml -f - up
docker-compose -f my.yml -p "camera_01_01" up

