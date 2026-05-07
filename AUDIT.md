# nrp-documentation — Stale-content audit (EBR2-34)

Baseline audit of the `nrp-documentation` repository, captured
**2026-05-07**. No content changes here — this report is the input
for Phase 4 follow-up Stories (EBR2-49, EBR2-50, EBR2-51, EBR2-52).

## Method

- `grep -rn` across `src/`, `.ci/`, `Jenkinsfile`, `ansible/` for
  retired infrastructure references.
- Inventory of external URLs in `src/`.
- Inspection of `conf.py`, `.ci/requirements_docs.txt`,
  `nrp-core-docs.zip`, and the Ansible deploy playbook.
- `make linkcheck` was **not** run from this audit environment (no
  Sphinx 3.4 install on hand). EBR2-49 will run it after the Sphinx
  bump and feed the results back in.

## 1. Toolchain (EOL exposure)

`/home/vvorobev/git-tum/nrp-documentation/.ci/requirements_docs.txt`:

```
docutils==0.16
sphinx==3.4.3            # EOL — last 3.x release was 2021-11
sphinx-rtd-theme==0.5.1  # 2021
sphinxcontrib-apidoc==0.3.0
sphinxcontrib-httpdomain
recommonmark==0.7.1      # unmaintained; replacement is MyST-Parser
sphinx-copybutton==0.3.1
sphinxcontrib-images==0.9.3
Jinja2<3.1               # cap blocks Sphinx ≥ 4.4
```

`src/conf.py:23` registers `recommonmark` as the Markdown extension.
`conf.py:7` carries an explicit `# TODO: install doxyrest properly`
that has not been resolved.

**Action (EBR2-49):** bump to Sphinx 7.x, swap `recommonmark` →
`MyST-Parser`, drop the Jinja2 cap, decide doxyrest fate (CI install
vs. pre-built artifact).

## 2. Checked-in build artifact

`nrp-documentation/nrp-core-docs.zip` — **1,128,905 bytes** of
pre-built C++/Python API documentation. Currently extracted into
`src/nrp-core/` by the `nrp-core-unzip` Makefile target. Should be
fetched at build time, not tracked in git.

**Action (EBR2-50):** publish the zip as a GitHub Release artifact on
`vvorobjov/nrp-core` and have `.ci/get-nrp-core-docs.py` fetch it.
Remove the tracked file.

## 3. Retired infrastructure references

### 3a. Nexus (registry credential, retired)

`Jenkinsfile:70`
```
registryCredentialsId 'nexusadmin'
```
`Jenkinsfile:118`
```
usernamePassword(credentialsId: 'nexusadmin',
                 usernameVariable: 'USER',
                 passwordVariable: 'PASSWORD')
```

The Nexus instance these credentials reference is no longer reachable.
The whole Jenkins pipeline relies on it for both `pull` and `push`
of the rendered docs.

**Action (EBR2-52):** replace this pipeline with a GitHub Actions
workflow on `vvorobjov/nrp-documentation` that builds via Sphinx and
deploys to GitHub Pages (or a configured docs subdomain).

### 3b. Bitbucket clone URLs hardcoded in Jenkinsfile

`Jenkinsfile:101-102`
```groovy
cloneRepoTopic(env.USER_SCRIPTS_DIR,
               'git@bitbucket.org:hbpneurorobotics/nrp-user-scripts.git',
               env.TOPIC_BRANCH, env.DEFAULT_BRANCH)
cloneRepoTopic(env.NRP_BACKEND_DIR,
               'git@bitbucket.org:hbpneurorobotics/nrp-backend.git',
               env.TOPIC_BRANCH, env.DEFAULT_BRANCH)
```

The Bitbucket repos still exist; not strictly broken, but the new
GitHub Actions workflow (EBR2-52) should clone via `actions/checkout`
or with parameterized URLs (see EBR2-54).

### 3c. Ansible deploy target

`ansible/deploy_docs.yml` targets the `docs_vm` host group with the
`nrp-docs` role. Two playbooks (release vs. staging deploy_dir).
This entire flow is replaced by GitHub Pages in EBR2-52; the role
and inventory should be archived.

`ansible/` contains:
- `deploy_docs.yml`
- `group_vars/`, `hosts`, `roles/` (full inventory).

## 4. Content references that may be stale

### 4a. Bitbucket clone snippets in install pages

`src/installation-source.rst:55`
```
git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git
```

`src/installation-docker.rst:46`
```
git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git
```

These should reference whichever upstream the user picks via
`NRP_UPSTREAM` (see EBR2-54). For the rewrite, prefer documenting the
default (Bitbucket) and showing the GitHub override.

### 4b. EBRAINS Docker registry reference

`src/installation-docker.rst:72-73`
```
# image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-opensim-tvb-ubuntu20${NRP_IMAGE_TAG}
image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-gazebo-nest-ubuntu20${NRP_IMAGE_TAG}
```

The EBRAINS registry's status is unconfirmed; the revival baseline
moves images to **Docker Hub** (`hbpneurorobotics/nrp-*`, see EBR2-35).
Page must be rewritten when EBR2-35 lands.

### 4c. Default branch name

Both install pages clone `-b master`. `nrp-user-scripts` default
branch is `development`, not `master`. The instruction is currently
wrong on a fresh clone.

## 5. External URL inventory

Hosts referenced from `src/**/*.rst` and `src/**/*.md`:

| Count | Host | Likely status |
| --- | --- | --- |
| 5 | `localhost` (in code samples) | n/a — local dev paths |
| 3 | `docs.docker.com` | live |
| 2 | `nest-desktop.readthedocs.io` | live |
| 2 | `bitbucket.org` | live (clone URLs in install pages) |
| 1 | `www.tum.de`, `www.epfl.ch`, `www.santannapisa.it`, `www.ugr.es`, `www.fortiss.org`, `www.nest-simulator.org`, `nest-simulator.readthedocs.io`, `raw.githubusercontent.com`, `learn.microsoft.com` | likely live |
| 1 | `www.neurorobotics.net` | **probably retired** (HBP project funding ended 2023) |
| 1 | `support.humanbrainproject.eu`, `forum.humanbrainproject.eu` | **probably retired** |

Run `make linkcheck` (EBR2-49 prerequisite) to confirm each. The
neurorobotics.net domain in particular routes through HBP infra that
is being phased out.

## 6. Pages and rewrite candidates

`src/` top-level pages:

| Page | Status | Notes |
| --- | --- | --- |
| `welcome.rst` | review | likely OK; verify HBP funding statement still applies |
| `before-you-start.rst` | review | check for retired URLs |
| `installation-docker.rst` | **rewrite** | EBRAINS registry refs, `-b master`, EBR2-35 baseline |
| `installation-source.rst` | **rewrite** | bitbucket URL, `-b master`, must reflect EBR2-54 NRP_UPSTREAM |
| `installation-implementations.rst` | review | likely overlaps with the above |
| `nrp-backend.rst` | review | confirm REST surface still matches code |
| `glossary.rst` | review | terminology drift (CLE/ExDBackend/etc. names) |
| `contact-and-support.rst` | **rewrite** | links to support.humanbrainproject.eu / forum |
| `index.rst` | review | toctree of the above |
| `tutorials/*` | per-page review | EBR2-51 will split into per-page Stories |

Subdirectories: `_images/`, `_static/`, `_templates/` — no content
audit performed; rebuild after Sphinx bump (EBR2-49) will surface
asset issues.

## 7. Build outputs and tracked artifacts

- `nrp-core-docs.zip` — see §2; remove (EBR2-50).
- `gitversion.rst_tochide` — present at repo root, references unclear;
  verify before deleting (low priority).

## 8. Summary — open work for Phase 4

| Audit finding | Closing Story |
| --- | --- |
| Sphinx 3.4 / recommonmark / Jinja cap | EBR2-49 |
| Doxyrest TODO in conf.py | EBR2-49 |
| Tracked `nrp-core-docs.zip` | EBR2-50 |
| Per-page rewrite list (§6) | EBR2-51 (umbrella) — split per page |
| Jenkins/Nexus/Ansible deploy chain | EBR2-52 |
| Bitbucket clone snippet in install pages | EBR2-54 (selectable URL) + page rewrite |
| `git clone -b master` on `development`-default repo | page rewrite (EBR2-51 child) |
| EBRAINS Docker registry refs | rewrite after EBR2-35 lands |
| Probable dead links to neurorobotics.net / humanbrainproject.eu | run `make linkcheck` (EBR2-49) |

This audit closes EBR2-34. Concrete fixes are tracked in the linked
Stories above.
