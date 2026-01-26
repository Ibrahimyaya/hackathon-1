/**
 * Custom Layout Component - Responsive Main Layout
 * Feature: 005-docusaurus-ui-upgrade
 * Date: 2026-01-09
 * User Story 1: Module Discovery & Navigation
 *
 * Renders the main layout with:
 * - Sidebar navigation (desktop/tablet)
 * - Hamburger menu toggle (mobile)
 * - Main content area
 * - Responsive grid layout
 */

import React, { useState } from 'react';
import Sidebar from '../Sidebar';
import ChatWidget from '../../components/ChatWidget';
import './layout.css';

interface LayoutProps {
  children: React.ReactNode;
  /**
   * Optional. If provided, will be displayed as the main content.
   * Otherwise, children will be used.
   */
}

/**
 * Layout Component
 *
 * Provides responsive layout with:
 * - Mobile: Full-width content, sidebar hidden (hamburger toggle)
 * - Tablet (768px+): Sidebar visible as column, content side-by-side
 * - Desktop (1024px+): Full 3-column layout with content
 * - Large (1440px+): Max-width constraints, wide sidebar
 *
 * Touch targets: All interactive elements ≥44x44px
 * Keyboard navigation: Tab, Enter, Escape support
 * Accessibility: ARIA labels, focus indicators, semantic HTML
 */
const Layout: React.FC<LayoutProps> = ({ children }) => {
  const [sidebarOpen, setSidebarOpen] = useState(false);

  /**
   * Handle hamburger menu toggle
   */
  const handleToggleSidebar = () => {
    setSidebarOpen(!sidebarOpen);
  };

  /**
   * Handle sidebar close (for mobile)
   */
  const handleCloseSidebar = () => {
    setSidebarOpen(false);
  };

  return (
    <div className="layout-root">
      {/* Main Layout Grid */}
      <div className="layout-main">
        {/* Sidebar Navigation */}
        <Sidebar isOpen={sidebarOpen} onClose={handleCloseSidebar} />

        {/* Main Content Area */}
        <main className="layout-content" role="main">
          {/* Mobile Hamburger Toggle */}
          <div className="layout-header-mobile">
            <button
              className={`sidebar-toggle ${sidebarOpen ? 'open' : ''}`}
              onClick={handleToggleSidebar}
              aria-label="Toggle navigation sidebar"
              aria-expanded={sidebarOpen}
              aria-controls="sidebar-nav"
              data-testid="hamburger-menu"
            >
              <div className="sidebar-toggle-icon">
                <span className="sidebar-toggle-line"></span>
                <span className="sidebar-toggle-line"></span>
                <span className="sidebar-toggle-line"></span>
              </div>
            </button>
            <h1 className="layout-header-title">Documentation</h1>
          </div>

          {/* Sidebar Overlay (Mobile) */}
          {sidebarOpen && (
            <div
              className="sidebar-overlay open"
              onClick={handleCloseSidebar}
              role="presentation"
              aria-hidden="true"
            ></div>
          )}

          {/* Main Content */}
          <div className="main-content-wrapper">
            {children}
          </div>
        </main>

        {/* Optional Right Sidebar for TOC (Future) */}
        <aside className="layout-toc" role="complementary" aria-label="Table of contents" />
      </div>

      {/* Chat Widget */}
      <ChatWidget />
    </div>
  );
};

export default Layout;
