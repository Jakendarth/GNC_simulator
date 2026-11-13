"""Build the Sphinx wiki in temporary staging; leave maintained/generated sources intact."""

import argparse
import ast
from html.parser import HTMLParser
from pathlib import Path
import re
import shutil
import tempfile
from urllib.parse import unquote, urlsplit
import warnings

from sphinx.cmd.build import build_main


def function_catalog(root):
    """Read definitions without importing GUI, solvers or application modules."""
    lines = [
        "# Function catalog", "",
        "Signatures and source links below are generated from the current Python source at build time. "
        "They describe available definitions, not a guarantee that every legacy path is integrated or runnable.", "",
        "For behavior and composition, start with [semantics](../topics/semantics.md), "
        "[guidance](../topics/guidance.md), [control](../topics/control.md), "
        "[navigation](../navigation/index.md), "
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
                level = "####" if owner else "###"
                lines.extend([f"{level} `{name}` · line {node.lineno}", "", "```python", signature, "```", ""])
                if any(later.name == node.name and later.lineno > node.lineno for later in functions):
                    lines.extend(["**Shadowed:** a later definition with the same name replaces this one in the same scope.", ""])
                lines.extend([f"[View implementation]({source_url})", ""])
    return "\n".join(lines)


def prepare_markdown(text, source):
    """Adapt generated OpenWiki syntax only in the disposable Sphinx input."""
    lines = text.splitlines(keepends=True)
    result = []
    in_fence = False
    for line in lines:
        if line.startswith("```"):
            in_fence = not in_fence
        if not in_fence:
            line = "".join(
                part if part.startswith("`") else re.sub(r"\\\((.+?)\\\)", r"$\1$", part)
                for part in re.split(r"(`+[^`]*`+)", line)
            )
            # Generated directory indexes use links to folders, not documents.
            def directory_link(match):
                target = urlsplit(match.group(1))
                if not target.scheme and target.path.endswith("/"):
                    candidate = source.parent / target.path / "index.md"
                    if candidate.is_file():
                        return "](" + match.group(1).rstrip("/") + "/index.md)"
                return match.group(0)
            line = re.sub(r"\]\(([^\s)]+)\)", directory_link, line)
        result.append(line)
    return "".join(result)


class HTMLLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.links = []

    def handle_starttag(self, tag, attributes):
        attributes = dict(attributes)
        if "id" in attributes:
            self.ids.add(attributes["id"])
        for name in ("href", "src", "poster"):
            if name in attributes:
                self.links.append(attributes[name])


def check_html(output):
    """Validate local resources and fragments, including raw video markup."""
    pages = {}
    for path in output.rglob("*.html"):
        page = HTMLLinks()
        page.feed(path.read_text(encoding="utf-8"))
        pages[path] = page
    errors = []
    checked = 0
    for path, page in pages.items():
        for link in page.links:
            url = urlsplit(link)
            if url.scheme or url.netloc:
                continue
            part = unquote(url.path)
            if part.startswith("/GNC_simulator/"):
                target = output / part.removeprefix("/GNC_simulator/")
            elif part.startswith("/"):
                target = output / part.lstrip("/")
            else:
                target = path.parent / part if part else path
            target = target.resolve()
            if target.is_dir():
                target /= "index.html"
            checked += 1
            if not target.exists():
                errors.append(f"{path.relative_to(output)}: missing {link}")
            elif url.fragment and target in pages and unquote(url.fragment) not in pages[target].ids:
                errors.append(f"{path.relative_to(output)}: missing fragment {link}")
    if errors:
        raise RuntimeError("\n".join(errors))
    print(f"Validated {len(pages)} HTML pages and {checked} local links/resources.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("site"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    # Only site/ is a generated output inside this repository. Outside it,
    # require an empty directory or a marker from an earlier successful build.
    if output.is_relative_to(root) and output != root / "site":
        parser.error("Choose site/ or an output directory outside the repository.")
    if output.exists() and (not output.is_dir() or (
        output != root / "site" and any(output.iterdir()) and not (output / ".gnc-docs-output").is_file()
    )):
        parser.error("The external output must be empty or created by this script.")
    with tempfile.TemporaryDirectory(prefix="gnc-sphinx-") as temporary:
        staged = Path(temporary) / "source"
        built = Path(temporary) / "html"
        shutil.copytree(root / "docs", staged, dirs_exist_ok=True)
        wiki = root / "openwiki"
        if not (wiki / "quickstart.md").is_file():
            raise FileNotFoundError("Regenerate openwiki/quickstart.md before building.")
        for source in wiki.rglob("*"):
            relative = source.relative_to(wiki)
            if source.is_file() and not any(p.startswith(".") for p in relative.parts):
                target = staged / "openwiki" / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
        reference = staged / "reference/functions.md"
        reference.parent.mkdir(parents=True, exist_ok=True)
        reference.write_text(function_catalog(root), encoding="utf-8")
        for page in staged.rglob("*.md"):
            page.write_text(prepare_markdown(page.read_text(encoding="utf-8"), page), encoding="utf-8")
        media = {
            "GNC_sim_logo.png": root / "assets/GNC_sim_logo.png",
            "branding/safe-net.png": root / "assets/safenet.png",
            "legal/LICENSE.txt": root / "LICENSE",
            "legal/NOTICE.txt": root / "NOTICE",
        }
        for name in ("PID_navigation_video.mp4", "NMPC_navigation_video.mp4", "multi_environment_crossing.mp4", "multi_environment_head_on.mp4"):
            media[f"videos/{name}"] = root / "output" / name
        for name, source in media.items():
            target = staged / "assets" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        shutil.copytree(staged / "assets", staged / "_public/assets")
        status = build_main([
            "-b", "dirhtml", "-W", "--keep-going", "-n",
            "-d", str(Path(temporary) / "doctrees"), str(staged), str(built),
        ])
        if status:
            raise SystemExit(status)
        check_html(built)
        # Replace generated output only after the build and link audit pass.
        if output.exists():
            shutil.rmtree(output)
        shutil.copytree(built, output)
        (output / ".gnc-docs-output").touch()
        print(f"Preview: python -m http.server 8000 --directory {output}")


if __name__ == "__main__":
    main()
