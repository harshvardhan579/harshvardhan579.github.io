import React from 'react';
import { cn } from '@/lib/utils';

interface BadgeProps {
  children: React.ReactNode;
  variant?: 'default' | 'outline' | 'accent' | 'bracket';
  size?: 'sm' | 'md';
  className?: string;
}

export function Badge({
  children,
  variant = 'default',
  size = 'md',
  className,
}: BadgeProps) {
  if (variant === 'bracket') {
    return (
      <span
        className={cn(
          'inline-flex items-center font-mono text-[#75665B] tracking-wider select-none',
          size === 'sm' ? 'text-[10px]' : 'text-xs',
          className
        )}
      >
        <span className="text-[#C9BBA6] mr-0.5">[</span>
        <span className="uppercase text-[#211914] font-medium">{children}</span>
        <span className="text-[#C9BBA6] ml-0.5">]</span>
      </span>
    );
  }

  const baseStyles =
    'inline-flex items-center font-mono uppercase tracking-wider select-none rounded-[2px] transition-colors';

  const sizeStyles = {
    sm: 'px-1.5 py-0.5 text-[10px]',
    md: 'px-2 py-1 text-xs',
  };

  const variantStyles = {
    default: 'bg-[#E7DECF] text-[#211914] border border-[#C9BBA6]',
    outline: 'bg-transparent text-[#4A3C34] border border-[#C9BBA6]',
    accent: 'bg-[#8A3F32]/10 text-[#8A3F32] border border-[#8A3F32]/30',
  };

  return (
    <span
      className={cn(
        baseStyles,
        sizeStyles[size],
        variantStyles[variant],
        className
      )}
    >
      {children}
    </span>
  );
}
