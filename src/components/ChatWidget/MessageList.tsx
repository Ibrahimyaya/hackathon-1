/**
 * Message List Component
 *
 * Displays chat messages with:
 * - User messages (right-aligned)
 * - Assistant messages (left-aligned)
 * - Source citations
 * - Confidence scores
 */

import React from 'react';
import { Message } from './index';

interface MessageListProps {
  messages: Message[];
  messagesEndRef: React.RefObject<HTMLDivElement>;
}

const MessageList: React.FC<MessageListProps> = ({ messages, messagesEndRef }) => {
  return (
    <>
      {messages.map((message) => (
        <div key={message.id} className={`chat-message chat-message-${message.role}`}>
          {/* Avatar */}
          <div className="chat-message-avatar">
            {message.role === 'user' ? '👤' : '🤖'}
          </div>

          {/* Content */}
          <div className="chat-message-content">
            {/* Message Text */}
            <p className="chat-message-text">{message.content}</p>

            {/* Sources */}
            {message.sources && message.sources.length > 0 && (
              <div className="chat-message-sources">
                <details className="chat-sources-details">
                  <summary className="chat-sources-summary">
                    📚 Sources ({message.sources.length})
                  </summary>
                  <ul className="chat-sources-list">
                    {message.sources.map((source, idx) => (
                      <li key={idx} className="chat-source-item">
                        <a
                          href={source.url}
                          target="_blank"
                          rel="noopener noreferrer"
                          className="chat-source-link"
                          title={source.section}
                        >
                          {source.section || 'Documentation'}
                        </a>
                        {source.text && (
                          <p className="chat-source-excerpt">{source.text.substring(0, 100)}...</p>
                        )}
                      </li>
                    ))}
                  </ul>
                </details>
              </div>
            )}

            {/* Confidence Score */}
            {message.confidence !== undefined && (
              <div className="chat-message-confidence">
                <span
                  className={`chat-confidence-badge confidence-${Math.round(message.confidence * 10) * 10}`}
                  title={`Confidence: ${Math.round(message.confidence * 100)}%`}
                >
                  {Math.round(message.confidence * 100)}%
                </span>
              </div>
            )}

            {/* Timestamp */}
            <span className="chat-message-timestamp">
              {message.timestamp.toLocaleTimeString([], {
                hour: '2-digit',
                minute: '2-digit',
              })}
            </span>
          </div>
        </div>
      ))}
      <div ref={messagesEndRef} />
    </>
  );
};

export default MessageList;
