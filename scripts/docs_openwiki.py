"""Expose public OpenWiki content to MkDocs without changing generated sources."""

from pathlib import Path
import re
from urllib.parse import urlsplit, urlunsplit

from mkdocs.exceptions import PluginError
from mkdocs.structure.files import File


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
