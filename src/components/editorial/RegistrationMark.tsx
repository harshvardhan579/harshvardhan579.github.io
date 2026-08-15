import React from 'react';
import { cn } from '@/lib/utils';

export function RegistrationMark({
  className,
  position = 'top-left',
}: {
  className?: string;
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';
}) {
  const posStyles = {
    'top-left': 'top-2 left-2',
    'top-right': 'top-2 right-2',
    'bottom-left': 'bottom-2 left-2',
    'bottom-right': 'bottom-2 right-2',
  };

  return (
    <div
      className={cn(
        'absolute pointer-events-none text-[#857267]/40 font-mono text-[10px] select-none leading-none',
        posStyles[position],
        className
      )}
      aria-hidden="true"
    >
      +
    </div>
  );
}
