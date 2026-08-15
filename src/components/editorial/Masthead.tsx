import React from 'react';
import { cn } from '@/lib/utils';
import { EditorialRule } from './EditorialRule';

interface MastheadProps {
  className?: string;
  title?: string;
  issue?: string;
  subline?: string;
}

export function Masthead({
  className,
  title = 'THE HARSHVARDHAN SINGH ARCHIVE',
  issue = 'VOL. 01 — 2026 EDITION',
  subline = 'ENGINEERING FOLIO',
}: MastheadProps) {
  return (
    <div className={cn('w-full select-none', className)}>
      <div className="flex flex-wrap items-center justify-between text-[11px] font-mono tracking-widest text-[#75665B] uppercase py-1 gap-2 border-b border-[#C9BBA6]">
        <div>{title}</div>
        <div className="hidden sm:block">{issue}</div>
        <div>{subline}</div>
      </div>
      <EditorialRule variant="double" className="my-2" />
    </div>
  );
}
