"""HTML to text extraction and cleaning."""

import logging
import re
from typing import Optional

from bs4 import BeautifulSoup, NavigableString

from utils.errors import CleaningError, ExtractionError

logger = logging.getLogger(__name__)


class TextCleaner:
    """Extracts and cleans text from HTML content."""

    # Tags to completely remove (content and tag)
    REMOVE_TAGS = {
        "script",
        "style",
        "nav",
        "footer",
        "header",
        "aside",
        "noscript",
        "meta",
        "link",
        "svg",
    }

    # Tags that should be treated as block elements with spacing
    BLOCK_TAGS = {"p", "div", "section", "article", "h1", "h2", "h3", "h4", "h5", "h6"}

    def extract_text(self, html: str, url: str = "") -> tuple[str, dict]:
        """Extract and clean text from HTML.

        Args:
            html: Raw HTML content
            url: Source URL (for metadata)

        Returns:
            Tuple of (cleaned_text, metadata_dict)

        Raises:
            ExtractionError: If extraction fails
        """
        try:
            soup = BeautifulSoup(html, "html.parser")

            # Remove unwanted tags
            for tag in self.REMOVE_TAGS:
                for element in soup.find_all(tag):
                    element.decompose()

            # Find main content (prefer main, article, or largest content div)
            main_content = self._find_main_content(soup)
            if main_content is None:
                main_content = soup.body or soup

            # Extract text with structure preservation
            text = self._extract_text_recursive(main_content)

            # Clean the text
            cleaned_text = self._clean_text(text)

            # Extract metadata
            metadata = self._extract_metadata(soup, url, cleaned_text)

            logger.debug(f"Extracted {len(cleaned_text)} characters from {url}")

            return cleaned_text, metadata

        except Exception as e:
            raise ExtractionError(f"Failed to extract text from HTML: {str(e)}") from e

    def _find_main_content(self, soup: BeautifulSoup) -> Optional[BeautifulSoup]:
        """Find the main content area in the document.

        Returns:
            Main content element or None
        """
        # Priority order: main > article > div.main > div.container > div.content
        selectors = [
            "main",
            "article",
            "div.main",
            "div[role='main']",
            "div.container",
        ]

        for selector in selectors:
            element = soup.select_one(selector)
            if element:
                return element

        return None

    def _extract_text_recursive(self, element) -> str:
        """Recursively extract text from HTML element, preserving structure.

        Args:
            element: BeautifulSoup element

        Returns:
            Extracted text with structure
        """
        text_parts = []

        for child in element.children:
            if isinstance(child, NavigableString):
                text = str(child).strip()
                if text:
                    text_parts.append(text)
            else:
                # Add spacing before block elements
                if child.name in self.BLOCK_TAGS:
                    if text_parts and text_parts[-1] != "\n":
                        text_parts.append("\n")

                # Recursively extract from child
                child_text = self._extract_text_recursive(child)
                if child_text:
                    text_parts.append(child_text)

                # Add spacing after block elements
                if child.name in self.BLOCK_TAGS:
                    if text_parts and text_parts[-1] != "\n":
                        text_parts.append("\n")

        return "".join(text_parts)

    def _clean_text(self, text: str) -> str:
        """Clean and normalize text.

        Args:
            text: Raw extracted text

        Returns:
            Cleaned text
        """
        try:
            # Normalize whitespace
            text = re.sub(r"\s+", " ", text)  # Multiple spaces to single
            text = re.sub(r"\n\s+\n", "\n\n", text)  # Multiple newlines to double

            # Remove leading/trailing whitespace
            text = text.strip()

            # Remove common HTML entities that weren't decoded
            text = text.replace("&nbsp;", " ")
            text = text.replace("&amp;", "&")
            text = text.replace("&lt;", "<")
            text = text.replace("&gt;", ">")

            # Remove zero-width characters
            text = text.replace("\u200b", "")
            text = text.replace("\u200c", "")
            text = text.replace("\u200d", "")

            return text
        except Exception as e:
            raise CleaningError(f"Failed to clean text: {str(e)}") from e

    def _extract_metadata(self, soup: BeautifulSoup, url: str, text: str) -> dict:
        """Extract metadata from HTML and text.

        Args:
            soup: BeautifulSoup object
            url: Source URL
            text: Extracted text

        Returns:
            Dictionary of metadata
        """
        metadata = {"url": url}

        # Extract title
        title_tag = soup.find("title")
        if title_tag:
            metadata["title"] = title_tag.get_text().strip()

        # Extract meta description
        meta_desc = soup.find("meta", attrs={"name": "description"})
        if meta_desc:
            metadata["description"] = meta_desc.get("content", "")

        # Extract headings hierarchy
        headings = []
        for heading in soup.find_all(["h1", "h2", "h3"]):
            heading_text = heading.get_text().strip()
            if heading_text:
                headings.append(
                    {
                        "level": int(heading.name[1]),
                        "text": heading_text,
                    }
                )

        metadata["headings"] = headings[:10]  # Limit to top 10
        metadata["character_count"] = len(text)
        metadata["word_count"] = len(text.split())

        return metadata
