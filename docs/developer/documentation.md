# Maintaining this manual

## Documentation ownership

| Location | Responsibility |
| --- | --- |
| `openwiki/` | Generated technical reference, indexes and evidence managed by OpenWiki |
| `docs/` | User-facing entry pages, maintenance instructions and website assets |
| `scripts/docs_openwiki.py` | Build-time integration of public OpenWiki pages and media |
| `mkdocs.yml` | Navigation, theme, extensions and link validation |
| `requirements-docs.txt` | Pinned documentation tools, separate from simulator dependencies |
| `site/` | Ignored build output; uploaded as the GitHub Pages artifact |

The build hook exposes non-hidden OpenWiki files under `openwiki/` in the site. It preserves the directory hierarchy for relative links and images, and converts generated directory links to explicit `index.md` targets in memory. It does not rewrite the source documentation. Hidden Claims, manifests and run state are excluded from the published files.

All existing Markdown pages are included in the navigation. When OpenWiki adds or moves a page, update the navigation and user-facing links; strict builds flag missing targets, anchors and pages omitted from navigation.

## Build and preview locally

Use a separate documentation environment. Python 3.13 matches CI; the commands below assume that interpreter is available:

```sh
python3.13 -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r requirements-docs.txt
mkdocs build --strict
mkdocs serve
```

Open the local URL printed by `mkdocs serve`. Check navigation, search, both color themes, code-copy controls and the Mermaid lifecycle diagram. A strict build validates internal Markdown links and resources; it does not execute the simulator, check remote websites, or prove browser-side diagram/math rendering.

Mermaid fences use Material's native diagram support. Mathematical expressions use `pymdownx.arithmatex` and MathJax; the MathJax runtime is loaded from a pinned CDN URL and needs internet access in the reader's browser. Keep figures next to the relevant OpenWiki page or under `docs/` and use relative references. There are no embedded image assets in the current OpenWiki pages.

## Regenerate the technical reference

1. Update the application source or its authoritative documentation.
2. Ask an OpenWiki-enabled coding agent to update this repository's OpenWiki from the current source and tests. Alternatively, use the existing OpenWiki CLI integration described below.
3. Review the generated changes and evidence. Preserve OpenWiki-managed metadata; do not hand-edit Claims or run-state files.
4. Update `docs/` introductions and `mkdocs.yml` if the topic structure changed.
5. Run `mkdocs build --strict`, preview the result and submit the changes for review.

The existing `.github/workflows/openwiki-update.yml` runs on a daily schedule or manual dispatch, uses `openwiki@0.9.0`, and opens an update pull request. It uses the repository's configured provider/model and secret environment variables. OpenWiki generation is separate from website building: the documentation workflow only publishes the reviewed Markdown already in the repository and requires no model-provider key.

With OpenWiki installed and its provider credentials configured, the repository workflow's update command can also be run locally:

```sh
openwiki code --update --print
```

Review the existing workflow for its provider, model and optional tracing configuration before using that automation. This documentation change does not configure those credentials or verify a remote regeneration run.

## Deployment and review

`.github/workflows/docs.yml` builds on documentation-related pull requests and deploys only from `main` after a successful strict build. It also supports manual dispatch; dispatching from another branch builds without deploying. Pushes to `main` affecting the wiki, manual, build hook, MkDocs configuration, dependency file, workflow or README trigger publication. Source-only changes first need a reviewed OpenWiki refresh.

The workflow uses the official GitHub Pages configure, artifact-upload and deployment actions. Build jobs have read-only repository access. Only the deployment job receives `pages: write` and `id-token: write`, with the `github-pages` environment. Production runs share a concurrency group and do not cancel an active deployment.

Before the first deployment, a repository maintainer must:

1. In **Settings → Pages → Build and deployment**, set **Source** to **GitHub Actions**.
2. Allow GitHub Actions and the actions used by `docs.yml` under **Settings → Actions → General**, if repository or organization policy currently blocks them.
3. Ensure the **github-pages** environment permits deployment from `main`. Add required reviewers there if each publication should require approval, where supported by the repository's plan.
4. Review and merge the documentation branch. A qualifying push to `main`, or a manual dispatch on `main`, will deploy to [the manual's configured URL](https://jakendarth.github.io/GNC_simulator/).

No `gh-pages` branch or personal access token is needed for the website workflow. A local build does not publish anything.

References: [GitHub Pages custom workflows](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages), [Material diagrams](https://squidfunk.github.io/mkdocs-material/reference/diagrams/), and [Material mathematics](https://squidfunk.github.io/mkdocs-material/reference/math/).
