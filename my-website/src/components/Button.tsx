import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import type {PropsWithChildren} from 'react';

import styles from './Button.module.css';

interface ButtonProps {
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost';
  size?: 'sm' | 'md' | 'lg';
  href?: string;
  to?: string;
  className?: string;
  disabled?: boolean;
  onClick?: () => void;
}

export default function Button({
  variant = 'primary',
  size = 'md',
  href,
  to,
  children,
  className,
  disabled = false,
  onClick,
}: PropsWithChildren<ButtonProps>): JSX.Element {
  const buttonClasses = clsx(
    styles.button,
    styles[`button--${variant}`],
    styles[`button--${size}`],
    {
      [styles['button--disabled']]: disabled,
    },
    className,
  );

  const commonProps = {
    className: buttonClasses,
    onClick: disabled ? undefined : onClick,
    'aria-disabled': disabled,
  };

  if (href) {
    return (
      <a href={href} {...commonProps}>
        {children}
      </a>
    );
  }

  if (to) {
    return (
      <Link to={to} {...commonProps}>
        {children}
      </Link>
    );
  }

  return (
    <button
      type="button"
      {...commonProps}
      disabled={disabled}>
      {children}
    </button>
  );
}