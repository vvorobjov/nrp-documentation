#documentation to build
DOC_MODULES=$(HBP)/BrainSimulation/hbp_nrp_distributed_nest/doc \
$(HBP)/ExperimentControl/hbp_nrp_excontrol/doc \
$(HBP)/VirtualCoach/hbp_nrp_virtual_coach/doc \
$(HBP)/ExDBackend/hbp_nrp_backend/doc \
$(HBP)/ExDBackend/hbp_nrp_cleserver/doc \
$(HBP)/ExDBackend/hbp_nrp_commons/doc \
$(HBP)/ExDBackend/hbp_nrp_watchdog/doc \
$(HBP)/CLE/hbp_nrp_cle/doc


# The installation order is important
INSTALL_MODULES=$(HBP)/BrainSimulation/hbp_nrp_distributed_nest \
$(HBP)/ExperimentControl/hbp_nrp_excontrol \
$(HBP)/VirtualCoach/hbp_nrp_virtual_coach \
$(HBP)/ExDBackend/hbp-flask-restful-swagger-master \
$(HBP)/ExDBackend/hbp_nrp_backend \
$(HBP)/ExDBackend/hbp_nrp_cleserver \
$(HBP)/ExDBackend/hbp_nrp_commons \
$(HBP)/ExDBackend/hbp_nrp_watchdog \
$(HBP)/CLE/hbp_nrp_cle


# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= -t fulldocs
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = src
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

doc: $(DOCS) $(COPY_PY_DOCS) copy-frontend doc-fast

doc-fast: copy-nrp copy-frontend
	$(file > version,$(VERSION))
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b html -D version=$(VERSION) -D release=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/html" $(O)

doc-release: SPHINXOPTS += -D todo_include_todos=0 -D show_authors=0
doc-release: doc;

$(COPY_PY_DOCS): cp_$(HBP)/%:
	mkdir -p $(SOURCEDIR)/nrp/modules/$*/../../
	cp -rf $(HBP)/$*/source/* $(SOURCEDIR)/nrp/modules/$*/../

copy-frontend:
	mkdir -p $(SOURCEDIR)/nrp/modules
	cp -r static_rst/ExDFrontend $(SOURCEDIR)/nrp/modules/
	
doc-clean-full: doc-clean
	rm -rf _build $(SOURCEDIR)/nrp/modules 

copy-nrp:
	cp -rf $(HBP)/neurorobotics-platform/*.md $(SOURCEDIR)/nrp/
	sed -i -E 's/^        (.*)/    ```bash\n    \1\n    ```/' src/nrp/*.md
	sed -i -E '/    ```/N;/```\n    ```bash/d' src/nrp/*.md

linkcheck: doc-fast
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b linkcheck -D version=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/linkcheck" $(O)

