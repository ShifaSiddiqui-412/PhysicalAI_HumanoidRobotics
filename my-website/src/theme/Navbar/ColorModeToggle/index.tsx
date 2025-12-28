import React from 'react';
import clsx from 'clsx';
import {useColorMode, useThemeConfig} from '@docusaurus/theme-common';
import type {ColorModeToggleProps} from '@theme/Navbar/ColorModeToggle';
import type {ThemeConfig} from '@docusaurus/preset-classic';

import styles from './styles.module.css';

export default function ColorModeToggle({
  className,
}: ColorModeToggleProps): JSX.Element {
  const {colorMode, setColorMode} = useColorMode();
  const {navbar: {hideColorModeSwitch = false} = {}} =
    useThemeConfig() as ThemeConfig;

  if (hideColorModeSwitch) {
    return <div className={clsx('navbar__color-mode-dropdown', className)} />;
  }

  const isDarkTheme = colorMode === 'dark';
  const sunIcon = '☀️';
  const moonIcon = '🌙';

  return (
    <div className={clsx('navbar__color-mode-dropdown', styles.colorModeToggle, className)}>
      <button
        aria-label={`Switch to ${isDarkTheme ? 'light' : 'dark'} mode`}
        className={clsx(
          'navbar__color-mode-toggle',
          styles.colorModeButton,
          isDarkTheme ? styles.darkMode : styles.lightMode
        )}
        onClick={() => setColorMode(isDarkTheme ? 'light' : 'dark')}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            setColorMode(isDarkTheme ? 'light' : 'dark');
          }
        }}
        type="button"
        role="switch"
        aria-checked={isDarkTheme}
      >
        <span className={styles.icon}>
          {isDarkTheme ? moonIcon : sunIcon}
        </span>
      </button>
    </div>
  );
}