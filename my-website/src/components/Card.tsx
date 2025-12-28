import React from 'react';
import clsx from 'clsx';
import type {PropsWithChildren} from 'react';

import styles from './Card.module.css';

interface CardProps {
  title?: string;
  description?: string;
  icon?: React.ReactNode;
  className?: string;
  href?: string;
  to?: string;
  variant?: 'default' | 'elevated' | 'outlined';
}

export default function Card({
  title,
  description,
  icon,
  children,
  className,
  href,
  to,
  variant = 'default',
}: PropsWithChildren<CardProps>): JSX.Element {
  const cardClasses = clsx(
    styles.card,
    styles[`card--${variant}`],
    className,
  );

  const content = (
    <>
      {icon && <div className={styles.cardIcon}>{icon}</div>}
      {title && <h3 className={styles.cardTitle}>{title}</h3>}
      {description && <p className={styles.cardDescription}>{description}</p>}
      {children}
    </>
  );

  if (href) {
    return (
      <a href={href} className={cardClasses}>
        {content}
      </a>
    );
  }

  if (to) {
    return (
      <div className={cardClasses}>
        <a href={to} className={styles.cardLink}>
          {content}
        </a>
      </div>
    );
  }

  return (
    <div className={cardClasses}>
      {content}
    </div>
  );
}