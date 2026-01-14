/**
 * Custom Sidebar Component - Module Discovery & Navigation
 * Feature: 005-docusaurus-ui-upgrade
 * Date: 2026-01-09
 * User Story 1: Module Discovery & Navigation
 *
 * Renders a module-wise navigation sidebar with expand/collapse,
 * keyboard navigation, ARIA labels, and responsive behavior.
 */

import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import '../../../css/components/sidebar.css';

interface Chapter {
  id: string;
  title: string;
  documentId: string;
}

interface Module {
  id: string;
  name: string;
  chapters: Chapter[];
}

interface SidebarProps {
  modules?: Module[];
  isOpen?: boolean;
  onClose?: () => void;
}

/**
 * Sidebar Component
 *
 * Displays modules with expandable chapters for navigation.
 * Supports:
 * - Keyboard navigation (Tab, Enter, Escape)
 * - ARIA labels for accessibility
 * - Mobile hamburger menu integration
 * - Responsive design (320px - 1440px+)
 * - Current page highlighting
 */
const Sidebar: React.FC<SidebarProps> = ({ modules = [], isOpen = false, onClose = () => {} }) => {
  const location = useLocation();
  const [expandedModules, setExpandedModules] = useState<Set<string>>(new Set(['module-1'])); // Module 1 open by default
  const sidebarRef = useRef<HTMLDivElement>(null);

  // Parse current document ID from URL
  const currentDocId = location.pathname
    .split('/docs/')
    .pop()
    ?.replace(/\/$/, '') || '';

  /**
   * Handle module expand/collapse
   */
  const toggleModule = (moduleId: string) => {
    const newExpanded = new Set(expandedModules);
    if (newExpanded.has(moduleId)) {
      newExpanded.delete(moduleId);
    } else {
      newExpanded.add(moduleId);
    }
    setExpandedModules(newExpanded);
  };

  /**
   * Handle chapter navigation
   */
  const handleNavigateToChapter = (documentId: string) => {
    // Navigate to the chapter URL
    const url = `/docs/${documentId}`;
    window.location.href = url;

    // Close mobile sidebar
    onClose();
  };

  /**
   * Handle keyboard navigation
   */
  const handleKeyDown = (e: React.KeyboardEvent, moduleId: string, isModule: boolean = false) => {
    if (isModule) {
      switch (e.key) {
        case 'Enter':
        case ' ':
          e.preventDefault();
          toggleModule(moduleId);
          break;
        case 'Escape':
          onClose();
          break;
      }
    } else {
      switch (e.key) {
        case 'Escape':
          onClose();
          break;
      }
    }
  };

  /**
   * Close sidebar on Escape key globally
   */
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
      // Prevent body scroll on mobile when sidebar is open
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }

    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = '';
    };
  }, [isOpen, onClose]);

  /**
   * Default modules if none provided
   * Based on documentation structure
   */
  const defaultModules: Module[] = [
    {
      id: 'module-1',
      name: '1. Humanoid Control',
      chapters: [
        { id: 'ch1-1', title: 'ROS 2 Overview', documentId: 'part1-foundations/ros2-overview' },
        { id: 'ch1-2', title: 'DDS Concepts', documentId: 'part1-foundations/dds-concepts' },
        { id: 'ch1-3', title: 'Why Humanoids', documentId: 'part1-foundations/why-humanoids' },
      ],
    },
    {
      id: 'module-2',
      name: '2. Perception & SLAM',
      chapters: [
        { id: 'ch2-1', title: 'Nodes & Lifecycle', documentId: 'part2-communication/nodes-and-lifecycle' },
        { id: 'ch2-2', title: 'Topics & PubSub', documentId: 'part2-communication/topics-pubsub' },
        { id: 'ch2-3', title: 'Services & ReqRep', documentId: 'part2-communication/services-reqrep' },
        { id: 'ch2-4', title: 'Actions & Async', documentId: 'part2-communication/actions-async' },
        { id: 'ch2-5', title: 'Agent Controller Example', documentId: 'part2-communication/agent-controller-example' },
      ],
    },
    {
      id: 'module-3',
      name: '3. Isaac Brain',
      chapters: [
        { id: 'ch3-1', title: 'URDF Fundamentals', documentId: 'part3-robot-structure/urdf-fundamentals' },
        { id: 'ch3-2', title: 'Humanoid URDF Example', documentId: 'part3-robot-structure/humanoid-urdf-example' },
        { id: 'ch3-3', title: 'RViz Gazebo Integration', documentId: 'part3-robot-structure/rviz-gazebo-integration' },
      ],
    },
    {
      id: 'module-4',
      name: '4. Vision-Language-Action',
      chapters: [
        { id: 'ch4-1', title: 'Gazebo Fundamentals', documentId: 'part4-gazebo-simulation/gazebo-fundamentals' },
        { id: 'ch4-2', title: 'Physics Simulation', documentId: 'part4-gazebo-simulation/physics-simulation' },
        { id: 'ch4-3', title: 'Humanoid Gazebo World', documentId: 'part4-gazebo-simulation/humanoid-gazebo-world' },
        { id: 'ch4-4', title: 'Gazebo Sensors', documentId: 'part4-gazebo-simulation/gazebo-sensors' },
        { id: 'ch4-5', title: 'Sensor Streaming ROS2', documentId: 'part4-gazebo-simulation/sensor-streaming-ros2' },
        { id: 'ch4-6', title: 'Advanced Physics', documentId: 'part4-gazebo-simulation/gazebo-advanced-physics' },
      ],
    },
  ];

  const displayModules = modules.length > 0 ? modules : defaultModules;

  return (
    <nav
      ref={sidebarRef}
      className={`sidebar ${isOpen ? 'open' : ''}`}
      role="navigation"
      aria-label="Documentation modules and chapters"
      data-testid="sidebar-nav"
    >
      {/* Sidebar Header */}
      <div className="sidebar-header">
        <h2 className="sidebar-header-title">Modules</h2>
        <p className="sidebar-header-subtitle">Select a module to explore</p>
      </div>

      {/* Sidebar Content - Modules & Chapters */}
      <div className="sidebar-content" role="region" aria-label="Navigation content">
        {displayModules.map((module) => (
          <div key={module.id} className={`sidebar-module ${expandedModules.has(module.id) ? 'expanded' : ''}`}>
            {/* Module Header (Category) */}
            <button
              className="sidebar-module-header"
              onClick={() => toggleModule(module.id)}
              onKeyDown={(e) => handleKeyDown(e, module.id, true)}
              aria-expanded={expandedModules.has(module.id)}
              aria-controls={`chapters-${module.id}`}
              data-testid={`module-${module.id}`}
            >
              <span className="sidebar-module-icon">
                {/* Expand/Collapse Arrow */}
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 16 16"
                  fill="currentColor"
                  aria-hidden="true"
                >
                  <path d="M6 3l4 5-4 5z" />
                </svg>
              </span>
              <span>{module.name}</span>
            </button>

            {/* Chapters List */}
            <ul
              className="sidebar-chapters"
              id={`chapters-${module.id}`}
              role="list"
              aria-label={`Chapters in ${module.name}`}
            >
              {module.chapters.map((chapter) => (
                <li key={chapter.id} className="sidebar-chapter" role="listitem">
                  <a
                    href={`/docs/${chapter.documentId}`}
                    className={`sidebar-chapter-link ${currentDocId === chapter.documentId ? 'active' : ''}`}
                    onClick={(e) => {
                      e.preventDefault();
                      handleNavigateToChapter(chapter.documentId);
                    }}
                    onKeyDown={(e) => handleKeyDown(e, chapter.id)}
                    aria-current={currentDocId === chapter.documentId ? 'page' : undefined}
                    data-testid={`chapter-${chapter.id}`}
                  >
                    {chapter.title}
                  </a>
                </li>
              ))}
            </ul>
          </div>
        ))}
      </div>

      {/* Sidebar Footer */}
      <div className="sidebar-footer">
        <p>
          4 modules <span aria-hidden="•">•</span> 20+ chapters
        </p>
      </div>
    </nav>
  );
};

export default Sidebar;
