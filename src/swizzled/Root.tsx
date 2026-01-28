/**
 * Root Component - Top-level wrapper for all pages
 * Feature: 005-docusaurus-ui-upgrade
 *
 * This component wraps all pages with the custom Layout that includes
 * the sidebar navigation. It's the outermost component in the page hierarchy.
 */

import React from 'react';
import Layout from './Layout';
import ChatWidget from '../components/ChatWidget';

interface RootProps {
  children: React.ReactNode;
}

const Root: React.FC<RootProps> = ({ children }) => {
  return (
    <>
      <Layout>{children}</Layout>
      <ChatWidget />
    </>
  );
};

export default Root;
