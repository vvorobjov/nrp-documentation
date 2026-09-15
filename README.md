# Buildng

The NRP source code should be installed to `$HBP`. Then just run

    make doc

for building of API and documentation pages.

# RST source outline

## Top level

The order of commands: `sectionauthor`, link-name, title. 

`sectionauthor` and/or link-name might be omitted, if not needed. Example:

    .. sectionauthor:: Page author <author@example.de>

    .. _link-name:

    Chapter Name
    ============

# CI

`.github/workflows/docs.yml` (EBR2-52) builds the Sphinx HTML on every
push and pull request targeting `master`, and deploys the result to
GitHub Pages from the canonical `vvorobjov/nrp-documentation` repo. The
published site lives at `https://vvorobjov.github.io/nrp-documentation/`
(activate Pages with source "GitHub Actions" in the repo settings on
first use). The nrp-core API section is included only when
`.ci/get-nrp-core-docs.py` can fetch `nrp-core-docs.zip` (default: the
latest `vvorobjov/nrp-core` GitHub Release asset, overridable through
the `NRP_CORE_DOCS_URL` Actions variable); a failed fetch downgrades
the build to a site without that section instead of failing it. The
`Jenkinsfile` and `ansible/` playbook are kept as references for the
legacy Bitbucket-side deploy.
