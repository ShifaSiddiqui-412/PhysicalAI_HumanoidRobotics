import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/ros2-fundamentals/index',
        'modules/ros2-fundamentals/chapter-1-ros2-fundamentals',
        'modules/ros2-fundamentals/chapter-2-python-agents',
        'modules/ros2-fundamentals/chapter-3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/digital-twin/index',
        'modules/digital-twin/chapter-1-physics-simulation',
        'modules/digital-twin/chapter-2-digital-twins-hri',
        'modules/digital-twin/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/isaac-ai-brain/index',
        'modules/isaac-ai-brain/chapter-1-isaac-sim',
        'modules/isaac-ai-brain/chapter-2-isaac-ros',
        'modules/isaac-ai-brain/chapter-3-nav2-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/vla-integration/index',
        'modules/vla-integration/chapter-1-voice-to-action',
        'modules/vla-integration/chapter-2-cognitive-planning',
        'modules/vla-integration/chapter-3-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
