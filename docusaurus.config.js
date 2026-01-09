// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are two ways to add type annotations for your Docusaurus config.
// 1. Inline type annotations
//    /**
//     * @type {import('@docusaurus/types').Config}
//     */
//    const config = { ... };
// 2. Create a separate `docusaurus.config.d.ts` file and add `const config: Config = ...;`

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'ROS 2 as the Robotic Nervous System for Humanoid Robots',
  tagline: 'Understanding middleware, communication patterns, and robot structure for humanoid robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://anthropics.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/ros2-humanoid-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'anthropics', // Usually your GitHub org/username.
  projectName: 'ros2-humanoid-book', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/anthropics/ros2-humanoid-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
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
                to: '/docs/part1-foundations/01-ros2-overview',
              },
              {
                label: 'Chapter 2: Communication',
                to: '/docs/part2-communication/04-nodes-and-lifecycle',
              },
              {
                label: 'Chapter 3: Robot Structure',
                to: '/docs/part3-robot-structure/09-urdf-fundamentals',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Official Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'DDS Specification',
                href: 'https://www.omg.org/spec/DDS/',
              },
              {
                label: 'GitHub Repository',
                href: 'https://github.com/anthropics/ros2-humanoid-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Anthropic. Built with Docusaurus. Book content follows official ROS 2 documentation (Humble/Jazzy LTS).`,
      },
      prism: {
        additionalLanguages: [],
      },
    }),
};

module.exports = config;
