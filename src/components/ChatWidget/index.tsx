/**
 * Chat Widget Component
 *
 * A floating chat bubble that opens a chat window for interacting with the RAG chatbot.
 * Features:
 * - Floating bubble (bottom-right corner)
 * - Click to expand/collapse chat window
 * - Message history within session
 * - Typing indicators
 * - Source citations
 * - Responsive design (full-screen on mobile)
 * - Light/dark theme support
 */

import React, { useState, useRef, useEffect } from 'react';
import ChatBubble from './ChatBubble';
import ChatWindow from './ChatWindow';
import './chat-widget.css';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{
    text: string;
    url: string;
    section: string;
  }>;
  confidence?: number;
}

const ChatWidget: React.FC = () => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [error, setError] = useState<string | null>(null);
  const [hasError, setHasError] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatApiUrl = process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000';

  // Initialize session ID on mount
  useEffect(() => {
    try {
      const storedSessionId = localStorage.getItem('chat-widget-session-id');
      if (storedSessionId) {
        setSessionId(storedSessionId);
      } else {
        const newSessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
        localStorage.setItem('chat-widget-session-id', newSessionId);
        setSessionId(newSessionId);
      }
      console.log('[ChatWidget] Initialized successfully', { chatApiUrl });
    } catch (err) {
      console.error('[ChatWidget] Initialization error:', err);
      setHasError(true);
    }
  }, []);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  /**
   * Toggle chat window expand/collapse
   */
  const handleToggleBubble = () => {
    setIsExpanded(!isExpanded);
    setError(null);
  };

  /**
   * Send a message to the chat API
   */
  const handleSendMessage = async (query: string) => {
    if (!query.trim()) return;

    // Add user message to state
    const userMessage: Message = {
      id: `msg-${Date.now()}-user`,
      role: 'user',
      content: query,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Prepare API request
      const requestPayload = {
        query: query,
        session_id: sessionId,
        history: messages.map((msg) => ({
          role: msg.role,
          content: msg.content,
        })),
      };

      // Call API
      const response = await fetch(`${chatApiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestPayload),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message to state
      const assistantMessage: Message = {
        id: `msg-${Date.now()}-assistant`,
        role: 'assistant',
        content: data.answer,
        timestamp: new Date(),
        sources: data.sources || [],
        confidence: data.confidence || 0,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);
      console.error('Chat API error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Clear chat history
   */
  const handleClearChat = () => {
    setMessages([]);
    setError(null);
  };

  // If component failed to initialize, show minimal chat bubble
  if (hasError) {
    return (
      <div className="chat-widget">
        <button
          className="chat-bubble"
          onClick={() => setIsExpanded(!isExpanded)}
          title="Chat widget (error mode)"
          style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999 }}
        >
          💬
        </button>
        {isExpanded && (
          <div
            style={{
              position: 'fixed',
              bottom: '80px',
              right: '0',
              width: '350px',
              height: '500px',
              background: 'white',
              borderRadius: '12px',
              boxShadow: '0 4px 24px rgba(0,0,0,0.15)',
              padding: '16px',
              zIndex: 9999,
              color: 'red',
              fontSize: '12px',
              overflow: 'auto',
            }}
          >
            <p>Chat widget error. Check console for details.</p>
            <p>{error}</p>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className="chat-widget">
      {/* Chat Bubble */}
      <ChatBubble
        isExpanded={isExpanded}
        onToggle={handleToggleBubble}
        unreadCount={messages.filter((m) => m.role === 'assistant' && !m.id.includes('read')).length}
      />

      {/* Chat Window */}
      {isExpanded && (
        <ChatWindow
          messages={messages}
          isLoading={isLoading}
          error={error}
          onSendMessage={handleSendMessage}
          onClearChat={handleClearChat}
          onClose={handleToggleBubble}
          messagesEndRef={messagesEndRef}
        />
      )}
    </div>
  );
};

export default ChatWidget;
