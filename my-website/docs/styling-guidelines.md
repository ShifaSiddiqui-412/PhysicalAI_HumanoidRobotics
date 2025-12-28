---
title: Styling Guidelines
sidebar_position: 999
---

# Styling Guidelines

This document outlines the styling guidelines and design system used in the ROS 2 Educational Modules website.

## Color Palette

### Primary Colors
- Primary: `#4f46e5` (Indigo)
- Primary Dark: `#4338ca`
- Primary Light: `#6366f1`
- Primary Lighter: `#818cf8`

### Secondary Colors
- Secondary: `#64748b` (Gray)
- Secondary Dark: `#475569`
- Secondary Light: `#94a3b8`

### Status Colors
- Success: `#10b981` (Green)
- Warning: `#f59e0b` (Amber)
- Danger: `#ef4444` (Red)

## Typography

### Font Stack
- Primary: `'Inter', system-ui, -apple-system, sans-serif`
- Code: Default Docusaurus monospace font

### Scales
- Base font size: 16px
- Line height: 1.6
- Global radius: 8px

## Spacing System

The design system uses a consistent spacing scale based on 0.25rem increments:
- 0.25rem, 0.5rem, 0.75rem, 1rem, 1.25rem, 1.5rem, 2rem, 2.5rem, 3rem, 4rem, 5rem, 8rem

## Responsive Breakpoints

- Mobile: Up to 640px
- Tablet: 641px to 768px
- Small Desktop: 769px to 1024px
- Desktop: 1025px and above

## Accessibility Features

### Focus Management
- Focus rings use primary color with 2px width
- Focus rings have 2px offset
- All interactive elements have visible focus states

### Touch Targets
- Minimum 44px touch targets for all interactive elements
- Buttons, links, and form elements meet this requirement

### Reduced Motion
- Animations and transitions are disabled when user prefers reduced motion
- Uses `prefers-reduced-motion` media query

### High Contrast Mode
- Color values adjust when user prefers high contrast
- Uses `prefers-contrast: high` media query

## Component Guidelines

### Buttons
- Use consistent padding and border-radius
- Primary buttons use primary color
- Secondary buttons use secondary color
- All buttons have hover and focus states

### Cards
- Use consistent shadows and border-radius
- Cards have proper spacing from surrounding elements
- Cards are responsive and adapt to screen size

### Navigation
- Navbar has consistent styling across all pages
- Sidebar navigation is collapsible on mobile
- Footer links are organized in columns appropriately