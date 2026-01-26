/**
 * Chat Bubble Component
 *
 * Floating chat bubble in bottom-right corner.
 * Displays unread message count and expands on click.
 */

import React from 'react';

interface ChatBubbleProps {
  isExpanded: boolean;
  onToggle: () => void;
  unreadCount: number;
}

const ChatBubble: React.FC<ChatBubbleProps> = ({ isExpanded, onToggle, unreadCount }) => {
  return (
    <button
      className={`chat-bubble ${isExpanded ? 'expanded' : ''}`}
      onClick={onToggle}
      aria-label="Open chat widget"
      aria-expanded={isExpanded}
      title="Ask about the documentation"
    >
      {/* Chat Icon */}
      <svg
        className="chat-bubble-icon"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
      >
        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
      </svg>

      {/* Unread Badge */}
      {unreadCount > 0 && (
        <span className="chat-bubble-badge" aria-label={`${unreadCount} unread messages`}>
          {unreadCount > 9 ? '9+' : unreadCount}
        </span>
      )}

      {/* Tooltip */}
      <span className="chat-bubble-tooltip">Ask a question</span>
    </button>
  );
};

export default ChatBubble;
