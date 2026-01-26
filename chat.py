#!/usr/bin/env python
"""Run the RAG chatbot from project root."""

import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

# Run the chatbot
if __name__ == '__main__':
    from backend.chat import RAGChatbot

    chatbot = RAGChatbot()
    chatbot.run()
