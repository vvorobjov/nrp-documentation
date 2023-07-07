#!/bin/bash

set -e
set -x

MAKE_TARGET=doc
if [ "$1" == "true" ]; then
    MAKE_TARGET=doc-release
fi

export USER_SCRIPTS_DIR="${USER_SCRIPTS_DIR:-nrp-user-scripts}"
export DOCS_DIR="${DOCS_DIR:-nrp-documentation}"

# Set dev mode and switch HBP to Jenkins workspace
export NRP_INSTALL_MODE=dev
export PYTHONPATH=""
# shellcheck source=/dev/null
source "${HBP}/${USER_SCRIPTS_DIR}/nrp_variables"

export BUILD_NUMBER

# VIRTUAL_ENV is a variable with path to virtualenv
export VIRTUAL_ENV_PATH=$VIRTUAL_ENV

# create docs
make "${MAKE_TARGET}"
