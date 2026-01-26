/**
 * Chat Window Component
 *
 * Main chat interface with:
 * - Message list
 * - Input field
 * - Send button
 * - Clear button
 * - Error display
 */

import React, { useState } from 'react';
import MessageList from './MessageList';
import { Message } from './index';

interface ChatWindowProps {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  onSendMessage: (message: string) => void;
  onClearChat: () => void;
  onClose: () => void;
  messagesEndRef: React.RefObject<HTMLDivElement>;
}

const ChatWindow: React.FC<ChatWindowProps> = ({
  messages,
  isLoading,
  error,
  onSendMessage,
  onClearChat,
  onClose,
  messagesEndRef,
}) => {
  const [inputValue, setInputValue] = useState('');

  /**
   * Handle form submission
   */
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim()) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  return (
    <div className="chat-window">
      {/* Header */}
      <div className="chat-window-header">
        <h2 className="chat-window-title">Documentation Assistant</h2>
        <button
          className="chat-window-close"
          onClick={onClose}
          aria-label="Close chat"
          title="Close"
        >
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        </button>
      </div>

      {/* Messages */}
      <div className="chat-window-messages">
        {messages.length === 0 ? (
          <div className="chat-empty-state">
            <p>Ask me anything about the documentation!</p>
            <p className="chat-empty-state-hint">Example: "What is ROS 2?"</p>
          </div>
        ) : (
          <MessageList messages={messages} messagesEndRef={messagesEndRef} />
        )}

        {/* Typing Indicator */}
        {isLoading && (
          <div className="chat-message chat-message-assistant">
            <div className="chat-message-avatar">🤖</div>
            <div className="chat-message-content">
              <div className="chat-typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        {/* Error Message */}
        {error && (
          <div className="chat-message chat-message-error">
            <div className="chat-message-avatar">⚠️</div>
            <div className="chat-message-content">
              <p className="chat-error-text">{error}</p>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <form className="chat-window-input-form" onSubmit={handleSubmit}>
        <input
          type="text"
          className="chat-window-input"
          placeholder="Ask a question..."
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          disabled={isLoading}
          aria-label="Chat message input"
        />
        <button
          type="submit"
          className="chat-window-send"
          disabled={isLoading || !inputValue.trim()}
          aria-label="Send message"
          title="Send message (Ctrl+Enter)"
        >
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="22" y1="2" x2="11" y2="13" />
            <polygon points="22 2 15 22 11 13 2 9 22 2" />
          </svg>
        </button>

        {messages.length > 0 && (
          <button
            type="button"
            className="chat-window-clear"
            onClick={onClearChat}
            disabled={isLoading}
            aria-label="Clear chat history"
            title="Clear chat history"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <polyline points="3 6 5 6 21 6" />
              <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
              <line x1="10" y1="11" x2="10" y2="17" />
              <line x1="14" y1="11" x2="14" y2="17" />
            </svg>
          </button>
        )}
      </form>
    </div>
  );
};

export default ChatWindow;
