import os
import requests
from bs4 import BeautifulSoup
from markdownify import markdownify as md
from urllib.parse import urljoin, urlparse
import time
import sys

# Configuration
BASE_URL = "https://yagsl.gitbook.io/yams"
OUTPUT_DIR = os.path.join("docs", "yams_scraped")
visited_urls = set()

def sanitize_filename(filename):
    return "".join([c if c.isalnum() or c in "._- " else "_" for c in filename])

def get_page_content(url):
    """Fetches and parses the page content."""
    try:
        response = requests.get(url)
        response.raise_for_status()
        return BeautifulSoup(response.text, 'html.parser')
    except Exception as e:
        print(f"Failed to fetch {url}: {e}")
        return None

def process_page(url, relative_path=""):
    """Scrapes a single page and saves it as markdown."""
    if url in visited_urls:
        return
    visited_urls.add(url)
    
    print(f"Processing: {url}")
    soup = get_page_content(url)
    if not soup:
        return

    # 1. Extract Main Content
    # GitBook structure often has a <main> tag or specific classes. 
    # We'll look for standard GitBook content wrappers.
    content = soup.find('main')
    if not content:
        # Fallback for some themes
        content = soup.find('div', {'class': 'gitbook-page'})
    
    if not content:
        print(f"  Warning: No content found for {url}")
        return

    # 2. Convert to Markdown
    # Remove some navigation elements if they exist inside main
    for nav in content.find_all('nav'):
        nav.decompose()
        
    markdown_text = md(str(content), heading_style="ATX")

    # 3. Determine File Path
    # Parse URL to create folder structure
    parsed_url = urlparse(url)
    path_parts = parsed_url.path.strip("/").split("/")
    
    # Remove 'yams' prefix if it's the base path to avoid double nesting
    if path_parts and path_parts[0] == "yams":
        path_parts = path_parts[1:]
        
    if not path_parts:
        filename = "README.md"
        current_dir = OUTPUT_DIR
    else:
        # If the URL ends with a slash or looks like a directory, index it as README
        filename = path_parts[-1] + ".md"
        current_dir = os.path.join(OUTPUT_DIR, *path_parts[:-1])

    os.makedirs(current_dir, exist_ok=True)
    file_path = os.path.join(current_dir, filename)

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(markdown_text)
        
    # 4. Find Links for Recursion
    # We only follow links within the documentation domain
    for a in soup.find_all('a', href=True):
        href = a['href']
        full_url = urljoin(url, href)
        parsed_href = urlparse(full_url)
        
        # Only internal links
        if parsed_href.netloc == urlparse(BASE_URL).netloc:
            # Check if it's a page (not a fragment identifier or asset)
            if not parsed_href.path.endswith(('.png', '.jpg', '.jpeg', '.gif', '.pdf')):
                # Recursive call
                # Simple recursion might revisit; visited_urls handles loops
                if full_url not in visited_urls and full_url.startswith(BASE_URL):
                     process_page(full_url)

def main():
    print(f"Starting crawl of {BASE_URL}...")
    print(f"Saving to {OUTPUT_DIR}")
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    process_page(BASE_URL)
    print("Done.")

if __name__ == "__main__":
    # Check dependencies
    try:
        import requests
        import markdownify
        import bs4
    except ImportError as e:
        print("Missing dependencies! Please run:")
        print("pip install requests beautifulsoup4 markdownify")
        sys.exit(1)
        
    main()
