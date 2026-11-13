"""Sphinx configuration. Build through scripts/build_docs.py to stage wiki/media."""

project = "GNC Simulator"
author = "Nikos Kougiatsos"
language = "en"
root_doc = "index"
extensions = [
    "myst_parser",
    "sphinx.ext.mathjax",
    "sphinx.ext.githubpages",
    "sphinxcontrib.mermaid",
    "sphinx_copybutton",
]
source_suffix = {".md": "markdown"}
myst_enable_extensions = ["colon_fence", "dollarmath", "amsmath"]
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 6
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "collapse_navigation": False,
    "navigation_depth": 3,
    "titles_only": True,
    "style_nav_header_background": "#f4f1ea",
}
html_baseurl = "https://jakendarth.github.io/GNC_simulator/"
html_logo = "assets/GNC_sim_logo.png"
html_favicon = "assets/GNC_sim_logo.png"
html_static_path = ["assets/stylesheets", "assets/javascripts"]
html_css_files = ["extra.css"]
html_js_files = ["theme.js"]
html_extra_path = ["_public"]
html_show_sourcelink = False
html_copy_source = False
html_context = {
    "display_github": True,
    "github_user": "Jakendarth",
    "github_repo": "GNC_simulator",
    "github_version": "main",
    "conf_py_path": "/docs/",
}
exclude_patterns = ["_public", "assets"]
mermaid_version = "11.16.0"
mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3.2.2/es5/tex-mml-chtml.js"
nitpicky = True

# Retain linked reference material without listing it in the public sidebar.
unlisted_reference_pages = {
    "openwiki/concepts/guidance-control",
    "openwiki/architecture/simulation-state",
    "openwiki/testing/validation",
    "developer/documentation",
    "openwiki/index",
    "openwiki/architecture/index",
    "openwiki/concepts/index",
    "openwiki/testing/index",
    "openwiki/workflows/index",
}


def unlisted_reference_metadata(app, docname, source):
    if docname not in unlisted_reference_pages:
        return
    # Annotate the disposable build input; never edit generated OpenWiki files.
    if source[0].startswith("---\n"):
        source[0] = source[0].replace("---\n", "---\norphan: true\n", 1)
    else:
        source[0] = "---\norphan: true\n---\n\n" + source[0]


def source_links(app, pagename, templatename, context, doctree):
    if pagename.startswith("openwiki/"):
        context["conf_py_path"] = "/"
    elif pagename == "reference/functions":
        # This page is generated from many source files, not docs/reference/.
        context["display_github"] = False


def setup(app):
    app.connect("html-page-context", source_links)
    app.connect("source-read", unlisted_reference_metadata)
