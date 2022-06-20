#!/bin/bash

set -e
set -x
cd ${WORKSPACE}

whoami
env | sort
pwd

## Instal python modules
########################
if [ -z ${PYTHON_VERSION_MAJOR_MINOR} ]; then
    export PYTHON_VERSION_MAJOR=$(python -c "import sys; print(sys.version_info.major)")
    export PYTHON_VERSION_MAJOR_MINOR=$(python -c "import sys; print('{}.{}'.format(sys.version_info.major, sys.version_info.minor))")
fi

MAKE_TARGET=doc
if [ $1 == "true" ]; then
    MAKE_TARGET=doc-release
fi

# Set dev mode and switch HBP to Jenkins workspace
export NRP_INSTALL_MODE=dev
source ${USER_SCRIPTS_DIR}/nrp_variables
export HBP=${WORKSPACE}

# TODO: make it more elegant with makefile
export PYTHONPATH=
export PYTHONPATH=$HBP/CLE/hbp_nrp_cle:${PYTHONPATH}
export PYTHONPATH=$HBP/ExperimentControl/hbp_nrp_excontrol:${PYTHONPATH}
export PYTHONPATH=$HBP/ExDBackend/hbp-flask-restful-swagger-master:${PYTHONPATH}
export PYTHONPATH=$HBP/ExDBackend/hbp_nrp_backend:${PYTHONPATH}
export PYTHONPATH=$HBP/ExDBackend/hbp_nrp_cleserver:${PYTHONPATH}
export PYTHONPATH=$HBP/ExDBackend/hbp_nrp_commons:${PYTHONPATH}
export PYTHONPATH=$HBP/ExDBackend/hbp_nrp_watchdog:${PYTHONPATH}
export PYTHONPATH=$HBP/VirtualCoach/hbp_nrp_virtual_coach:${PYTHONPATH}
export PYTHONPATH=$HBP/BrainSimulation/hbp_nrp_distributed_nest:${PYTHONPATH}
export PYTHONPATH=$NRP_INSTALL_DIR/lib/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages:${PYTHONPATH}
# VIRTUAL_ENV is a variable with path to virtualenv
export VIRTUAL_ENV_PATH=$VIRTUAL_ENV

# Checkout config.ini.sample from user-scripts
cp ${HBP}/${USER_SCRIPTS_DIR}/config_files/CLE/config.ini.sample ${HBP}/${CLE_DIR}/hbp_nrp_cle/hbp_nrp_cle/config.ini

# install ExDBackend and schemas 
cd ${EXDBACKEND_DIR}
env | sort
# Obtain schemas
# This variable is needed for common_makefile during schemas generation
export DEFAULT_CO_BRANCH=${DEFAULT_BRANCH}
export TOPIC_BRANCH
make schemas-install

# return to Jenkins workspace
cd ${WORKSPACE}

# install modules listed in nrp-documentation Makefile
cd ${GIT_CHECKOUT_DIR}
env | sort
make devinstall



# create docs
. $VIRTUAL_ENV_PATH/bin/activate \
    && source /opt/ros/noetic/setup.bash \
    && source /home/bbpnrsoa/nrp/src/GazeboRosPackages/devel/setup.bash \
    && make "${MAKE_TARGET}"