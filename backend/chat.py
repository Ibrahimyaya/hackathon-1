"""Interactive RAG chatbot CLI interface."""

import sys
from typing import Optional

try:
    from rich.console import Console
    from rich.markdown import Markdown
    from rich.panel import Panel
    from rich.table import Table
except ImportError:
    print("Error: 'rich' library is required. Install it with: pip install rich")
    sys.exit(1)

import anthropic

from ingestion.embeddings.cohere_embedder import CohereEmbedder
from ingestion.storage.qdrant_store import QdrantStore
from rag.query_engine import RAGQueryEngine
from utils.config import load_config
from utils.logging import setup_logging


class RAGChatbot:
    """Interactive RAG chatbot with REPL interface."""

    def __init__(self):
        """Initialize chatbot with configuration and clients."""
        self.console = Console()

        try:
            # Load configuration
            self.config = load_config()
            setup_logging(self.config)

            # Initialize clients
            self.embedder = CohereEmbedder(
                api_key=self.config.cohere_api_key,
                model=self.config.cohere_model,
                batch_size=self.config.cohere_batch_size,
            )

            self.store = QdrantStore(
                url=self.config.qdrant_url,
                api_key=self.config.qdrant_api_key,
                collection_name=self.config.qdrant_collection_name,
                vector_size=self.config.qdrant_vector_size,
            )

            self.claude_client = anthropic.Anthropic(api_key=self.config.claude_api_key)

            # Initialize query engine
            self.engine = RAGQueryEngine(
                config=self.config,
                embedder=self.embedder,
                store=self.store,
                claude_client=self.claude_client,
            )

            self.show_sources = True
            self.is_running = True

        except Exception as e:
            self.console.print(f"[red]Failed to initialize chatbot: {str(e)}[/red]")
            sys.exit(1)

    def run(self) -> None:
        """Start the interactive REPL loop."""
        self._print_banner()

        while self.is_running:
            try:
                query = self.console.input("\n[bold cyan]You:[/bold cyan] ").strip()

                if not query:
                    continue

                if query.startswith("/"):
                    self._handle_command(query)
                else:
                    self._process_query(query)

            except KeyboardInterrupt:
                self.console.print("\n[yellow]Goodbye![/yellow]")
                break
            except EOFError:
                self.console.print("[yellow]Goodbye![/yellow]")
                break

    def _print_banner(self) -> None:
        """Print welcome banner."""
        banner = Panel.fit(
            "[bold blue]🤖 RAG Chatbot[/bold blue]\n"
            "[white]Powered by Claude + Qdrant[/white]\n"
            "[dim]Type /help for commands or /quit to exit[/dim]",
            border_style="blue",
            padding=(1, 2),
        )
        self.console.print(banner)

    def _handle_command(self, command: str) -> None:
        """Handle special commands starting with /.

        Args:
            command: Command string starting with /
        """
        cmd = command.lower().split()[0]

        if cmd == "/quit":
            self.is_running = False
        elif cmd == "/help":
            self._show_help()
        elif cmd == "/clear":
            self._clear_history()
        elif cmd == "/history":
            self._show_history()
        elif cmd == "/sources":
            self._toggle_sources()
        else:
            self.console.print(f"[red]Unknown command: {command}[/red]")
            self.console.print("[dim]Type /help for available commands[/dim]")

    def _show_help(self) -> None:
        """Display help for available commands."""
        help_text = Table(title="Available Commands", show_header=True)
        help_text.add_column("Command", style="cyan")
        help_text.add_column("Description", style="white")

        help_text.add_row("/help", "Show this help message")
        help_text.add_row("/clear", "Clear conversation history")
        help_text.add_row("/history", "Show conversation history")
        help_text.add_row("/sources", "Toggle source display")
        help_text.add_row("/quit", "Exit the chatbot")

        self.console.print(help_text)

    def _clear_history(self) -> None:
        """Clear conversation history."""
        self.engine.clear_history()
        self.console.print("[green]✓ Conversation history cleared[/green]")

    def _show_history(self) -> None:
        """Display conversation history."""
        history = self.engine.get_history()

        if not history:
            self.console.print("[dim]No conversation history yet[/dim]")
            return

        for i, turn in enumerate(history, 1):
            self.console.print(f"\n[bold]Turn {i}[/bold]")
            self.console.print(f"[cyan]User:[/cyan] {turn['question']}")
            self.console.print(f"[blue]Assistant:[/blue] {turn['answer'][:200]}...")

    def _toggle_sources(self) -> None:
        """Toggle source display."""
        self.show_sources = not self.show_sources
        status = "[green]enabled[/green]" if self.show_sources else "[red]disabled[/red]"
        self.console.print(f"Source display {status}")

    def _process_query(self, query: str) -> None:
        """Process user query and display response.

        Args:
            query: User's question
        """
        try:
            # Show thinking indicator
            with self.console.status("[yellow]🔍 Searching knowledge base...[/yellow]"):
                response = self.engine.query(query)

            # Display response
            self.console.print(f"\n[bold blue]🤖 Assistant:[/bold blue]")

            # Show confidence and answer
            confidence_pct = int(response.confidence * 100)
            confidence_bar = "█" * (confidence_pct // 10) + "░" * (10 - confidence_pct // 10)
            self.console.print(
                f"[dim][Confidence: {confidence_bar} {confidence_pct}%][/dim]\n"
            )

            # Display answer as markdown
            self.console.print(Markdown(response.answer))

            # Display sources if enabled
            if self.show_sources and response.sources:
                self.console.print("\n[bold dim]📚 Sources:[/bold dim]")
                for chunk in response.sources:
                    score_pct = int(chunk.score * 100)
                    section = chunk.section or "Unknown"
                    self.console.print(
                        f"  [cyan]•[/cyan] {section} "
                        f"[dim](score: {score_pct}%)[/dim]"
                    )
                    self.console.print(f"    [underline]{chunk.source_url}[/underline]")

        except anthropic.APIError as e:
            self.console.print(f"[red]Claude API error: {str(e)}[/red]")
        except Exception as e:
            self.console.print(f"[red]Error: {str(e)}[/red]")


def main() -> int:
    """Main entry point.

    Returns:
        Exit code (0 for success, 1 for error)
    """
    try:
        chatbot = RAGChatbot()
        chatbot.run()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as e:
        print(f"Fatal error: {str(e)}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
