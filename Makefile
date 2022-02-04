# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= -t fulldocs
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = src
BUILDDIR      = _build

VERSION  = $(shell bash -c "git describe --tags --always | sed 's/-[^-]*$$//'")

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

doc: nrp-core-unzip doc_dep
	$(info $$VERSION is [${VERSION}])
	$(file > version,$(VERSION))
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b html -D version=$(VERSION) -D release=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/html" $(O)

doc-release: VERSION = $(shell git describe --tags --always --abbrev=0)
doc-release: doc;

nrp-core-unzip:
	unzip -o -q nrp-core-docs.zip -d src/nrp-core/

	
doc-clean-full: doc-clean
	rm -rf _build $(SOURCEDIR)/nrp-core

linkcheck: doc-fast
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b linkcheck -D version=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/linkcheck" $(O)

