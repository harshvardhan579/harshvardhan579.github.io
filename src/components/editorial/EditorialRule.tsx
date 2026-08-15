import React from 'react';
import { cn } from '@/lib/utils';

interface EditorialRuleProps {
  variant?: 'single' | 'double' | 'dashed' | 'heavy';
  orientation?: 'horizontal' | 'vertical';
  className?: string;
  label?: string;
}

export function EditorialRule({
  variant = 'single',
  orientation = 'horizontal',
  className,
  label,
}: EditorialRuleProps) {
  if (orientation === 'vertical') {
    return (
      <div
        className={cn(
          'w-px self-stretch bg-[#BEB09C]',
          variant === 'heavy' && 'w-[2px] bg-[#302019]',
          variant === 'dashed' && 'border-r border-dashed border-[#BEB09C] bg-transparent',
          className
        )}
        aria-hidden="true"
      />
    );
  }

  if (label) {
    return (
      <div className={cn('flex items-center gap-3 my-6 select-none', className)}>
        <div className="flex-1 h-px bg-[#BEB09C]" />
        <span className="font-mono text-[10px] tracking-widest uppercase text-[#857267]">
          {label}
        </span>
        <div className="flex-1 h-px bg-[#BEB09C]" />
      </div>
    );
  }

  if (variant === 'double') {
    return (
      <div className={cn('my-4 space-y-[3px]', className)} aria-hidden="true">
        <div className="h-[2px] bg-[#302019] w-full" />
        <div className="h-px bg-[#BEB09C] w-full" />
      </div>
    );
  }

  return (
    <div
      className={cn(
        'h-px bg-[#BEB09C] my-4 w-full',
        variant === 'heavy' && 'h-[2px] bg-[#302019]',
        variant === 'dashed' && 'border-b border-dashed border-[#BEB09C] bg-transparent h-0',
        className
      )}
      aria-hidden="true"
    />
  );
}
