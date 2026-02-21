#!/usr/bin/env python3
"""
Scrape the YAGSL documentation from docs.yagsl.com and save it as markdown files.
Output goes to docs/yagsl/ by default.
"""

import re
import time
import argparse
from pathlib import Path
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup

BASE_URL = "https://docs.yagsl.com"
SITEMAP_URL = "https://docs.yagsl.com/sitemap-pages.xml"
DEFAULT_OUT = Path(__file__).parent.parent / "docs" / "yagsl"
DELAY = 0.5  # seconds between requests


def get_page_urls() -> list[str]:
    """Parse the sitemap to get all page URLs."""
    resp = requests.get(SITEMAP_URL, timeout=15)
    resp.raise_for_status()
    soup = BeautifulSoup(resp.text, "xml")
    urls = [loc.text.strip() for loc in soup.find_all("loc")]
    return urls


def html_to_markdown(soup: BeautifulSoup) -> str:
    """Extract main content from a GitBook page and convert to rough markdown."""
    # GitBook renders content inside <main> or article-like containers
    main = (
        soup.find("main")
        or soup.find("article")
        or soup.find("div", class_=re.compile(r"page-content|content|markdown"))
        or soup.body
    )

    lines = []

    def walk(el, depth=0):
        if el is None:
            return
        if isinstance(el, str):
            text = el.strip()
            if text:
                lines.append(text)
            return

        tag = el.name if hasattr(el, "name") else None

        if tag in ("script", "style", "nav", "footer", "aside"):
            return

        if tag in ("h1", "h2", "h3", "h4", "h5", "h6"):
            level = int(tag[1])
            lines.append(f"\n{'#' * level} {el.get_text(strip=True)}\n")
        elif tag == "p":
            lines.append(f"\n{el.get_text(strip=True)}\n")
        elif tag in ("ul", "ol"):
            for i, li in enumerate(el.find_all("li", recursive=False)):
                prefix = f"{i+1}." if tag == "ol" else "-"
                lines.append(f"{prefix} {li.get_text(strip=True)}")
            lines.append("")
        elif tag == "pre":
            code = el.get_text()
            lang = ""
            code_el = el.find("code")
            if code_el:
                cls = " ".join(code_el.get("class", []))
                m = re.search(r"language-(\w+)", cls)
                if m:
                    lang = m.group(1)
                code = code_el.get_text()
            lines.append(f"\n```{lang}\n{code}\n```\n")
        elif tag == "code" and el.parent and el.parent.name != "pre":
            lines.append(f"`{el.get_text(strip=True)}`")
        elif tag == "a":
            href = el.get("href", "")
            text = el.get_text(strip=True)
            if href:
                full = urljoin(BASE_URL, href)
                lines.append(f"[{text}]({full})")
            else:
                lines.append(text)
        elif tag in ("strong", "b"):
            lines.append(f"**{el.get_text(strip=True)}**")
        elif tag in ("em", "i"):
            lines.append(f"*{el.get_text(strip=True)}*")
        elif tag == "blockquote":
            for line in el.get_text(strip=True).splitlines():
                lines.append(f"> {line}")
            lines.append("")
        elif tag in ("table",):
            rows = el.find_all("tr")
            for r_idx, row in enumerate(rows):
                cells = row.find_all(["th", "td"])
                lines.append("| " + " | ".join(c.get_text(strip=True) for c in cells) + " |")
                if r_idx == 0:
                    lines.append("| " + " | ".join("---" for _ in cells) + " |")
            lines.append("")
        elif tag in ("br",):
            lines.append("")
        else:
            for child in el.children:
                walk(child)
            return

    walk(main)

    # Clean up excessive blank lines
    text = "\n".join(lines)
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip()


def url_to_path(url: str, out_dir: Path) -> Path:
    """Convert a URL to a local file path inside out_dir."""
    parsed = urlparse(url)
    parts = [p for p in parsed.path.strip("/").split("/") if p]
    if not parts:
        parts = ["index"]
    rel = Path(*parts).with_suffix(".md")
    return out_dir / rel


def scrape(out_dir: Path, verbose: bool = True):
    out_dir.mkdir(parents=True, exist_ok=True)

    if verbose:
        print(f"Fetching sitemap from {SITEMAP_URL} ...")
    urls = get_page_urls()
    if verbose:
        print(f"Found {len(urls)} pages.")

    for i, url in enumerate(urls, 1):
        dest = url_to_path(url, out_dir)
        if dest.exists():
            if verbose:
                print(f"[{i}/{len(urls)}] SKIP (exists) {url}")
            continue

        if verbose:
            print(f"[{i}/{len(urls)}] {url}")

        try:
            resp = requests.get(url, timeout=15)
            resp.raise_for_status()
        except Exception as e:
            print(f"  ERROR fetching {url}: {e}")
            continue

        soup = BeautifulSoup(resp.text, "html.parser")

        # Page title for the top of the file
        title_tag = soup.find("title")
        title = title_tag.get_text(strip=True) if title_tag else ""

        md = html_to_markdown(soup)
        if title and not md.startswith("#"):
            md = f"# {title}\n\n{md}"

        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(md, encoding="utf-8")

        time.sleep(DELAY)

    if verbose:
        print(f"\nDone. Files written to: {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Scrape YAGSL docs to markdown")
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help=f"Output directory (default: {DEFAULT_OUT})",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress progress output")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-download pages even if they already exist",
    )
    args = parser.parse_args()

    if args.force and args.out.exists():
        import shutil
        shutil.rmtree(args.out)

    scrape(args.out, verbose=not args.quiet)


if __name__ == "__main__":
    main()
