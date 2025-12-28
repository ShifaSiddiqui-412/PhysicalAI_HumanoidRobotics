import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import {useThemeConfig} from '@docusaurus/theme-common';
import {useWindowSize} from '@docusaurus/theme-common';
import DocSidebarItems from '@theme/DocSidebarItems';
import type {Props} from '@theme/DocSidebar';

import styles from './styles.module.css';

function DocSidebarDesktop({path, sidebar, className}: Props) {
  const {navbar: {hideOnScroll} = {}} = useThemeConfig();

  return (
    <div
      className={clsx(
        'navbar-sidebar--show',
        styles.sidebar,
        hideOnScroll && styles.sidebarWithHideableNavbar,
        className,
      )}>
      <div className={styles.sidebarInner}>
        <nav
          role="navigation"
          className={styles.sidebarNav}
          aria-label="Docs sidebar">
          <div className={styles.sidebarItemsWrapper}>
            <DocSidebarItems
              items={sidebar.map((item) => ({
                ...item,
                collapsible: true,
              }))}
              activePath={path}
              level={1}
              className={styles.docSidebarMenu}
            />
          </div>
        </nav>
      </div>
    </div>
  );
}

export default function DocSidebar(props: Props): JSX.Element {
  const windowSize = useWindowSize();
  const [showResponsiveSidebar, setShowResponsiveSidebar] = useState(false);

  useEffect(() => {
    if (windowSize === 'desktop') {
      setShowResponsiveSidebar(false);
    }
  }, [windowSize]);

  return (
    <>
      <DocSidebarDesktop {...props} />
    </>
  );
}