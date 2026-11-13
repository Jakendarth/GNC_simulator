"""Expose public OpenWiki content to MkDocs without changing generated sources."""

import ast
from pathlib import Path
import re
import warnings
from urllib.parse import urlsplit, urlunsplit

from mkdocs.exceptions import PluginError
from mkdocs.structure.files import File


def function_catalog(root):
    """Read definitions without importing GUI, solvers or application modules."""
    lines = [
        "# Function catalog", "",
        "Signatures and source links below are generated from the current Python source at build time. "
        "They describe available definitions, not a guarantee that every legacy path is integrated or runnable.", "",
        "For behavior and composition, start with [semantics](../topics/semantics.md), "
        "[guidance](../topics/guidance.md), [control](../topics/control.md), "
        "[dynamics and environment](../topics/dynamics.md), and [pipeline examples](../pipelines.md).", "",
    ]
    paths = sorted((root / "src").rglob("*.py")) + [root / "main.py", root / "plot_results.py"]
    for source in paths:
        relative = source.relative_to(root).as_posix()
        # Existing source docstrings contain legacy backslash escapes. Parsing
        # definitions does not execute them; keep those unrelated warnings out
        # of the manual build while still failing on invalid Python syntax.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", SyntaxWarning)
            tree = ast.parse(source.read_text(encoding="utf-8"), filename=relative)
        lines.extend([f"## `{relative}`", ""])
        owners = [(None, tree.body)] + [
            (node.name, node.body) for node in tree.body if isinstance(node, ast.ClassDef)
        ]
        for owner, body in owners:
            functions = [node for node in body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]
            if not functions:
                continue
            if owner:
                lines.extend([f"### Class `{owner}`", ""])
            for node in functions:
                name = f"{owner}.{node.name}" if owner else node.name
                signature = f"{name}({ast.unparse(node.args)})"
                source_url = f"https://github.com/Jakendarth/GNC_simulator/blob/main/{relative}#L{node.lineno}-L{node.end_lineno}"
                lines.extend([f"#### `{name}` · line {node.lineno}", "", "```python", signature, "```", ""])
                if any(later.name == node.name and later.lineno > node.lineno for later in functions):
                    lines.extend(["**Shadowed:** a later definition with the same name replaces this one in the same scope.", ""])
                lines.extend([f"[View implementation]({source_url})", ""])
    return "\n".join(lines)


def on_files(files, config):
    root = Path(config.config_file_path).parent
    wiki = root / "openwiki"
    if not (wiki / "quickstart.md").is_file():
        raise PluginError("Missing openwiki/quickstart.md; regenerate OpenWiki before building.")

    # Keep the hierarchy intact so relative page links and media remain valid.
    # Hidden evidence, provenance and run-state files are never public assets.
    for source in sorted(wiki.rglob("*")):
        relative = source.relative_to(wiki)
        if not source.is_file() or any(part.startswith(".") for part in relative.parts):
            continue
        files.append(File.generated(config, f"openwiki/{relative.as_posix()}", abs_src_path=str(source)))

    media = {"assets/GNC_sim_logo.png": "assets/GNC_sim_logo.png"}
    for name in ("PID_navigation_video.mp4", "NMPC_navigation_video.mp4",
                 "multi_environment_crossing.mp4", "multi_environment_head_on.mp4"):
        media[f"assets/videos/{name}"] = f"output/{name}"
    for destination, relative in media.items():
        source = root / relative
        if not source.is_file():
            raise PluginError(f"Missing manual media: {relative}")
        files.append(File.generated(config, destination, abs_src_path=str(source)))
    files.append(File.generated(config, "reference/functions.md", content=function_catalog(root)))
    return files


def on_page_markdown(markdown, page, config, files):
    if not page.file.src_uri.startswith("openwiki/"):
        return markdown

    # OpenWiki indexes use directory links; MkDocs validates explicit Markdown
    # targets and then rewrites them to the correct project-site URLs.
    def directory_link(match):
        target = urlsplit(match.group(1))
        if target.scheme or target.netloc or not target.path.endswith("/"):
            return match.group(0)
        candidate = (Path(page.file.src_uri).parent / target.path / "index.md").as_posix()
        if files.get_file_from_path(candidate) is None:
            return match.group(0)
        return "](" + urlunsplit(target._replace(path=target.path + "index.md")) + ")"

    return re.sub(r"\]\(([^\s)]+)\)", directory_link, markdown)
