/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a set of docs in the sidebar
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

/** @type {import('@docusaurus/types').SidebarsConfig} */
const sidebars = {
  // But you can create a sidebar manually
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'index',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Part 1: Humanoid Control',
      items: [
        'part1-foundations/ros2-overview',
        'part1-foundations/dds-concepts',
        'part1-foundations/why-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Perception & SLAM',
      items: [
        'part2-communication/nodes-and-lifecycle',
        'part2-communication/topics-pubsub',
        'part2-communication/services-reqrep',
        'part2-communication/actions-async',
        'part2-communication/agent-controller-example',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Isaac Brain',
      items: [
        'part3-robot-structure/urdf-fundamentals',
        'part3-robot-structure/humanoid-urdf-example',
        'part3-robot-structure/rviz-gazebo-integration',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Simulation & Control',
      items: [
        'part4-gazebo-simulation/gazebo-fundamentals',
        'part4-gazebo-simulation/physics-simulation',
        'part4-gazebo-simulation/humanoid-gazebo-world',
        'part4-gazebo-simulation/gazebo-sensors',
        'part4-gazebo-simulation/sensor-streaming-ros2',
        'part4-gazebo-simulation/gazebo-advanced-physics',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/intro',
        'module-4-vla/chapter-voice-recognition',
      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'setup-guide',
        'glossary',
        'references',
        'known-issues',
        'quickstart',
      ],
    },
  ],
};

module.exports = sidebars;
