'use client';

import React from 'react';
import Link from 'next/link';
import { isLinkValid } from '@/data/contact';
import { cn } from '@/lib/utils';

interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  href?: string;
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost';
  size?: 'sm' | 'md' | 'lg';
  isExternal?: boolean;
  children: React.ReactNode;
  icon?: React.ReactNode;
}

export function Button({
  href,
  variant = 'primary',
  size = 'md',
  isExternal = false,
  children,
  icon,
  className,
  ...props
}: ButtonProps) {
  // Hide invalid links cleanly
  if (href && !isLinkValid(href)) {
    return null;
  }

  const baseStyles =
    'inline-flex items-center justify-center gap-2 font-mono text-xs uppercase tracking-wider transition-all duration-150 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[#8A3F32] active:scale-[0.98] select-none rounded-[2px] border cursor-pointer';

  const sizeStyles = {
    sm: 'px-3 py-1.5 text-[11px] min-h-[32px]',
    md: 'px-4 py-2 text-xs min-h-[38px]',
    lg: 'px-6 py-3 text-sm min-h-[44px]',
  };

  const variantStyles = {
    primary:
      'bg-[#211914] text-[#FCF9F2] border-[#211914] hover:bg-[#8A3F32] hover:border-[#8A3F32] shadow-xs',
    secondary:
      'bg-[#E7DECF] text-[#211914] border-[#C9BBA6] hover:bg-[#C9BBA6] hover:text-[#211914]',
    outline:
      'bg-transparent text-[#211914] border-[#211914] hover:bg-[#211914] hover:text-[#FCF9F2]',
    ghost:
      'bg-transparent text-[#4A3C34] border-transparent hover:text-[#211914] hover:bg-[#E7DECF]/60',
  };

  const content = (
    <>
      <span className="whitespace-nowrap">{children}</span>
      {icon && <span className="transition-transform group-hover:translate-x-0.5 shrink-0">{icon}</span>}
    </>
  );

  const combinedClasses = cn(
    baseStyles,
    sizeStyles[size],
    variantStyles[variant],
    'group',
    className
  );

  if (href) {
    // External or protocol or asset links (e.g. PDFs)
    if (isExternal || href.endsWith('.pdf') || href.startsWith('http') || href.startsWith('mailto:') || href.startsWith('tel:')) {
      const shouldOpenNewTab = isExternal || href.endsWith('.pdf') || href.startsWith('http');
      return (
        <a
          href={href}
          target={shouldOpenNewTab ? '_blank' : undefined}
          rel={shouldOpenNewTab ? 'noopener noreferrer' : undefined}
          className={combinedClasses}
        >
          {content}
        </a>
      );
    }

    // In-page hash anchor links (boringly reliable native <a> + instant scrollIntoView fallback)
    if (href.startsWith('#')) {
      const handleHashClick = (e: React.MouseEvent<HTMLAnchorElement>) => {
        const targetId = href.replace('#', '');
        const targetEl = document.getElementById(targetId);
        if (targetEl) {
          e.preventDefault();
          targetEl.scrollIntoView({ behavior: 'smooth', block: 'start' });
          if (typeof window !== 'undefined' && window.history.pushState) {
            window.history.pushState(null, '', href);
          }
        }
      };

      return (
        <a
          href={href}
          onClick={handleHashClick}
          className={combinedClasses}
        >
          {content}
        </a>
      );
    }

    // Internal router page navigation
    return (
      <Link href={href} className={combinedClasses}>
        {content}
      </Link>
    );
  }

  return (
    <button className={combinedClasses} {...props}>
      {content}
    </button>
  );
}
