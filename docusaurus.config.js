/** @type {import('@docusaurus/types').Config} */
// Vercel deployment - fixed baseUrl to / for correct asset paths
const config = {
  title: 'ROS 2 as the Robotic Nervous System for Humanoid Robots (Vercel)',
  tagline: 'Understanding middleware, communication patterns, and robot structure for humanoid robotics',
  favicon: 'img/favicon.ico',

  url: 'https://final-sandy-psi.vercel.app',
  baseUrl: '/',
  trailingSlash: false,

  // ✅ REMOVE GitHub Pages config (VERY IMPORTANT)
  // organizationName: 'anthropics',
  // projectName: 'ros2-humanoid-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  customFields: {
    chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // ✅ Correct edit URL
          editUrl: 'https://github.com/anthropics/ros2-humanoid-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'ROS 2 Humanoid Book',
      logo: {
        alt: 'ROS 2 Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          href: 'https://github.com/anthropics/ros2-humanoid-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Chapter 1: Foundations',
              to: '/docs/part1-foundations/ros2-overview',
            },
            {
              label: 'Chapter 2: Communication',
              to: '/docs/part2-communication/nodes-and-lifecycle',
            },
            {
              label: 'Chapter 3: Robot Structure',
              to: '/docs/part3-robot-structure/urdf-fundamentals',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Anthropic.`,
    },
  },
};

module.exports = config;
