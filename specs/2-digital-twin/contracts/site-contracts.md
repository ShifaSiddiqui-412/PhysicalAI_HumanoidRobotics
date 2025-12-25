# API Contracts: Docusaurus Site for Digital Twin Educational Module

**Feature**: 2-digital-twin
**Created**: 2025-12-24
**Status**: Draft

## Build System Contracts

### Site Configuration Contract
```javascript
// docusaurus.config.js
{
  title: String,           // Site title (required)
  tagline: String,         // Site tagline (required)
  url: String,             // Site URL (required)
  baseUrl: String,         // Base path (required)
  onBrokenLinks: String,   // 'throw', 'warn', or 'ignore'
  onBrokenMarkdownLinks: String, // 'warn', 'throw', or 'ignore'
  favicon: String,         // Path to favicon
  organizationName: String, // GitHub org name (for deployment)
  projectName: String,     // GitHub project name (for deployment)
  deploymentBranch: String, // Branch for GitHub Pages deployment
  i18n: {
    defaultLocale: String, // Default language code
    locales: String[]      // Supported locales
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: String,    // Path to sidebar config
          editUrl: String,        // URL for edit links
          showLastUpdateTime: Boolean, // Show last update time
          showLastUpdateAuthor: Boolean // Show last update author
        },
        blog: false,              // Disable blog for educational content
        theme: {
          customCss: String[]     // Custom CSS files
        },
        gtag: {                   // Google Analytics (optional)
          trackingID: String,
          anonymizeIP: Boolean
        }
      }
    ]
  ],
  themeConfig: {
    navbar: {
      title: String,              // Navbar title
      logo: {
        alt: String,              // Alt text for logo
        src: String               // Path to logo
      },
      items: Array                // Navigation items
    },
    footer: {
      style: String,              // 'dark' or 'light'
      links: Array,               // Footer links
      copyright: String           // Copyright text
    },
    prism: {
      theme: Object,              // Prism theme
      darkTheme: Object           // Dark theme version
    }
  }
}
```

## Navigation Contracts

### Sidebar Configuration Contract
```javascript
// src/components/sidebars/2-digital-twin.js
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        {
          type: 'doc',
          id: 'modules/2-digital-twin/index', // Module overview
          label: 'Overview'
        },
        {
          type: 'category',
          label: 'Chapters',
          items: [
            {
              type: 'doc',
              id: 'modules/2-digital-twin/chapter-1-gazebo-physics',
              label: 'Chapter 1: Physics Simulation with Gazebo'
            },
            {
              type: 'doc',
              id: 'modules/2-digital-twin/chapter-2-unity-digital-twins',
              label: 'Chapter 2: Digital Twins and HRI using Unity'
            },
            {
              type: 'doc',
              id: 'modules/2-digital-twin/chapter-3-sensor-simulation',
              label: 'Chapter 3: Sensor Simulation & Validation'
            }
          ]
        }
      ]
    }
  ]
};
```

## Content File Contracts

### Chapter Content Structure Contract
```markdown
---
title: Chapter Title
description: Brief description of the chapter
tags: [tag1, tag2, tag3]
sidebar_position: number
---

# Chapter Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Introduction
Brief introduction to the chapter topic...

## Main Content
Detailed content with appropriate headings, code examples, and diagrams...

## Summary
Key takeaways from the chapter...

## Exercises/Assessment
Questions or tasks to test understanding...
```

## Build and Deployment Contracts

### Package.json Dependencies Contract
```json
{
  "name": "hackathon-book-digital-twin-module",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "serve": "docusaurus serve",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids"
  },
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.3.0",
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.0.0",
    "@docusaurus/types": "^3.0.0"
  },
  "browserslist": {
    "production": [
      ">0.5%",
      "not dead",
      "not op_mini all"
    ],
    "development": [
      "last 1 chrome version",
      "last 1 firefox version",
      "last 1 safari version"
    ]
  },
  "engines": {
    "node": ">=18.0"
  }
}
```

## Content Integration Contracts

### Code Example Component Contract
```javascript
// For use in MDX files
import CodeBlock from '@theme/CodeBlock';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

// Standard code block usage
<CodeBlock language="cpp">{`// Gazebo example
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class CustomModelPlugin : public gazebo::ModelPlugin
{
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Plugin initialization code
  }
};`}</CodeBlock>

// Tabbed code example usage
<Tabs>
<TabItem value="gazebo" label="Gazebo">
{`// Gazebo implementation`}
</TabItem>
<TabItem value="unity" label="Unity">
{`// Unity implementation`}
</TabItem>
</Tabs>
```

## Search and Metadata Contracts

### SEO Metadata Contract
Each page must include:
- Page title (h1) that describes the content
- Meta description (150-160 characters)
- Proper heading hierarchy (h1, h2, h3, etc.)
- Alt text for all images
- Structured content for search indexing