#documentation to build
DOC_MODULES=$(HBP)/ExDBackend/hbp_nrp_backend/doc \
$(HBP)/ExDBackend/hbp_nrp_cleserver/doc \
$(HBP)/ExDBackend/hbp_nrp_commons/doc \
$(HBP)/ExDBackend/hbp_nrp_watchdog/doc \
$(HBP)/CLE/hbp_nrp_cle/doc \
$(HBP)/VirtualCoach/hbp_nrp_virtual_coach/doc \
$(HBP)/BrainSimulation/hbp_nrp_distributed_nest/doc \
$(HBP)/ExperimentControl/hbp_nrp_excontrol/doc


# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

VERSION       = $(shell git -C $(HBP)/ExDBackend describe --tags --abbrev=0)

## Include common makefile from adminscripts
CI_REPO?=git@bitbucket.org:hbpneurorobotics/admin-scripts.git
CI_DIR?=$(HBP)/admin-scripts/ContinuousIntegration
THIS_DIR:=$(PWD)

FETCH_CI := $(shell \
		if [ ! -d $(CI_DIR) ]; then \
				cd $(HBP) && git clone $(CI_REPO) > /dev/null && cd $(THIS_DIR);\
		fi;\
		echo $(CI_DIR) )

COMMON_PY_MAKEFILE:=$(FETCH_CI)/python/common_makefile

include $(COMMON_PY_MAKEFILE)

COPY_PY_DOCS=$(addprefix cp_, $(DOC_MODULES))

doc: $(DOCS) $(COPY_PY_DOCS)
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b html -D version=$(VERSION) "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

$(COPY_PY_DOCS): cp_$(HBP)/%:
	mkdir -p $(THIS_DIR)/nrp/modules/$*/../../
	cp -rf $(HBP)/$*/source/* $(THIS_DIR)/nrp/modules/$*/../
	
doc-clean-full: doc-clean
	rm -rf _build nrp/modules 
