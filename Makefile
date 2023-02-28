#documentation to build
DOC_MODULES=$(HBP)/nrp-backend/hbp_nrp_backend/doc

INSTALL_MODULES=$(HBP)/nrp-backend/hbp_nrp_backend

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= -t fulldocs
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = src
BUILDDIR      = _build

VERSION  = $(shell git describe --tags --always --abbrev=0)-$(BUILD_NUMBER)

## Include common makefile from nrp-backend
FETCH_CI?=$(HBP)/nrp-backend
COMMON_PY_MAKEFILE:=$(FETCH_CI)/user_makefile
include $(COMMON_PY_MAKEFILE)


COPY_PY_DOCS=$(addprefix cp_, $(DOC_MODULES))

doc: nrp-core-unzip doc_dep $(COPY_PY_DOCS)
	$(info $$VERSION is [${VERSION}])
	$(file > version,$(VERSION))
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b html -D version=$(VERSION) -D release=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/html" $(O)

doc-release: VERSION = $(shell git describe --tags --always --abbrev=0)
doc-release: doc;

$(COPY_PY_DOCS): cp_$(HBP)/%:
	mkdir -p $(SOURCEDIR)/$*/../../
	cp -rf $(HBP)/$*/source/* $(SOURCEDIR)/$*/../

nrp-core-unzip:
	unzip -o -q nrp-core-docs.zip -d src/nrp-core/

	
doc-clean-full: doc-clean
	rm -rf _build $(SOURCEDIR)/nrp-core

linkcheck: doc-fast
	. $(PLATFORM_VENV)/bin/activate; $(SPHINXBUILD) -b linkcheck -D version=$(VERSION) $(SPHINXOPTS) -d "$(BUILDDIR)/doctrees" -w sphinx_w.txt  "$(SOURCEDIR)" "$(BUILDDIR)/linkcheck" $(O)

