#!/bin/bash

set -e
set -x
cd ${WORKSPACE}

MAKE_TARGET=doc
if [ $1 == "true" ]; then
    MAKE_TARGET=doc-release
fi

# Set dev mode and switch HBP to Jenkins workspace
export NRP_INSTALL_MODE=dev
export PYTHONPATH=""
source ${USER_SCRIPTS_DIR}/nrp_variables
export HBP=${WORKSPACE}

export BUILD_NUMBER

# VIRTUAL_ENV is a variable with path to virtualenv
export VIRTUAL_ENV_PATH=$VIRTUAL_ENV

# install modules listed in nrp-documentation Makefile
cd ${GIT_CHECKOUT_DIR}

# create docs
make "${MAKE_TARGET}"