"""Docusaurus site crawler for documentation extraction."""

import logging
import time
from typing import List, Set, Tuple
from urllib.parse import urljoin, urlparse
from xml.etree import ElementTree as ET

import requests
from bs4 import BeautifulSoup

from utils.errors import CrawlError, CrawlNetworkError, CrawlTimeoutError, CrawlValidationError

logger = logging.getLogger(__name__)


class DocusaurusCrawler:
    """Crawls Docusaurus documentation sites and extracts page URLs and content."""

    def __init__(
        self,
        base_url: str,
        max_pages: int = 1000,
        timeout_seconds: int = 10,
        follow_external_links: bool = False,
        crawl_delay_ms: int = 500,
        max_retries: int = 3,
    ):
        """Initialize the Docusaurus crawler.

        Args:
            base_url: Root URL of the documentation site
            max_pages: Maximum number of pages to crawl
            timeout_seconds: Timeout per page request
            follow_external_links: Whether to follow external links
            crawl_delay_ms: Delay between requests (milliseconds)
            max_retries: Maximum retry attempts for failed requests

        Raises:
            CrawlValidationError: If base_url is invalid
        """
        self.base_url = base_url.rstrip("/")
        self.max_pages = max_pages
        self.timeout_seconds = timeout_seconds
        self.follow_external_links = follow_external_links
        self.crawl_delay_ms = crawl_delay_ms
        self.max_retries = max_retries

        # Parse base URL
        parsed = urlparse(self.base_url)
        if not parsed.scheme or not parsed.netloc:
            raise CrawlValidationError(f"Invalid base URL: {base_url}")

        self.base_domain = parsed.netloc
        self.base_scheme = parsed.scheme
        self.session = requests.Session()
        self.session.headers.update(
            {"User-Agent": "DocusaurusBot/1.0 (+http://www.example.com/bot)"}
        )

    def crawl(self) -> List[Tuple[str, str]]:
        """Crawl the documentation site and return list of (url, html_content) tuples.

        Returns:
            List of tuples containing (page_url, html_content)

        Raises:
            CrawlError: If crawling fails
        """
        logger.info(f"Starting crawl of {self.base_url}")

        try:
            # Try to get sitemap URLs first
            urls = self._get_urls_from_sitemap()
            if not urls:
                # Fallback to breadth-first search
                logger.info("No sitemap found, using BFS crawl")
                urls = self._bfs_crawl()
        except Exception as e:
            raise CrawlError(f"Crawling failed: {str(e)}") from e

        logger.info(f"Crawl complete: discovered {len(urls)} pages")
        return urls

    def _get_urls_from_sitemap(self) -> List[Tuple[str, str]]:
        """Extract URLs from sitemap.xml.

        Returns:
            List of (url, html_content) tuples from sitemap
        """
        try:
            sitemap_url = urljoin(self.base_url, "/sitemap.xml")
            logger.debug(f"Attempting to fetch sitemap: {sitemap_url}")

            response = self._fetch_url(sitemap_url)
            if response is None:
                return []

            # Parse sitemap
            root = ET.fromstring(response)
            urls = []

            # Extract loc elements (namespace aware)
            for loc in root.findall(".//{http://www.sitemaps.org/schemas/sitemap/0.9}loc"):
                urls.append(loc.text)

            logger.info(f"Found {len(urls)} URLs in sitemap")

            # Fetch content for each URL
            results = []
            for i, url in enumerate(urls[: self.max_pages]):
                if i % 10 == 0:
                    logger.debug(f"Progress: {i}/{len(urls)} pages fetched")

                html = self._fetch_url(url)
                if html:
                    results.append((url, html))

                time.sleep(self.crawl_delay_ms / 1000.0)

            return results
        except Exception as e:
            logger.warning(f"Failed to use sitemap: {str(e)}")
            return []

    def _bfs_crawl(self) -> List[Tuple[str, str]]:
        """Breadth-first search crawl of documentation site.

        Returns:
            List of (url, html_content) tuples from BFS crawl
        """
        visited: Set[str] = set()
        queue: List[str] = [self.base_url]
        results: List[Tuple[str, str]] = []

        while queue and len(visited) < self.max_pages:
            url = queue.pop(0)

            if url in visited:
                continue

            visited.add(url)
            logger.debug(f"Crawling: {url} ({len(visited)}/{self.max_pages})")

            html = self._fetch_url(url)
            if html:
                results.append((url, html))

                # Extract links from HTML
                try:
                    soup = BeautifulSoup(html, "html.parser")
                    for link in soup.find_all("a", href=True):
                        href = link["href"]
                        absolute_url = urljoin(url, href)

                        # Filter to same domain
                        parsed = urlparse(absolute_url)
                        if parsed.netloc == self.base_domain or (
                            self.follow_external_links and parsed.scheme in ("http", "https")
                        ):
                            # Remove fragments
                            absolute_url = absolute_url.split("#")[0]
                            if absolute_url not in visited and absolute_url not in queue:
                                queue.append(absolute_url)
                except Exception as e:
                    logger.warning(f"Failed to extract links from {url}: {str(e)}")

            time.sleep(self.crawl_delay_ms / 1000.0)

        logger.info(f"BFS crawl complete: {len(results)} pages fetched")
        return results

    def _fetch_url(self, url: str) -> str | None:
        """Fetch URL content with retry logic.

        Args:
            url: URL to fetch

        Returns:
            HTML content or None if fetch fails

        Raises:
            CrawlNetworkError: If all retries are exhausted
        """
        for attempt in range(self.max_retries):
            try:
                response = self.session.get(
                    url, timeout=self.timeout_seconds, allow_redirects=True
                )
                response.raise_for_status()
                return response.text
            except requests.Timeout:
                logger.warning(f"Timeout fetching {url} (attempt {attempt + 1}/{self.max_retries})")
                if attempt == self.max_retries - 1:
                    raise CrawlTimeoutError(f"Timeout fetching {url} after {self.max_retries} retries")
            except requests.RequestException as e:
                if response.status_code >= 500:
                    logger.warning(
                        f"Server error {response.status_code} for {url} (attempt {attempt + 1}/{self.max_retries})"
                    )
                    if attempt == self.max_retries - 1:
                        raise CrawlNetworkError(f"Failed to fetch {url}: {str(e)}")
                else:
                    # Don't retry client errors
                    logger.error(f"Client error {response.status_code} for {url}: {str(e)}")
                    return None
            except Exception as e:
                logger.error(f"Unexpected error fetching {url}: {str(e)}")
                return None

        return None
