import React from 'react';
import { cn } from '@/lib/utils';

interface SectionHeaderProps {
  index?: string;
  tag?: string;
  title: string;
  description?: string;
  className?: string;
  align?: 'left' | 'center';
}

export function SectionHeader({
  index,
  tag,
  title,
  description,
  className,
  align = 'left',
}: SectionHeaderProps) {
  return (
    <div
      className={cn(
        'mb-12 md:mb-16',
        align === 'center' ? 'text-center mx-auto max-w-2xl' : 'max-w-3xl',
        className
      )}
    >
      {(index || tag) && (
        <div className="flex items-center gap-2 font-mono text-xs uppercase tracking-wider text-[#7C8CFF] mb-3 select-none">
          {index && <span className="text-[#6F7682]">{index}</span>}
          {index && tag && <span className="text-[#6F7682]">/</span>}
          {tag && <span>{tag}</span>}
        </div>
      )}
      <h2 className="text-3xl md:text-4xl lg:text-5xl font-bold tracking-tight text-[#F4F6F8]">
        {title}
      </h2>
      {description && (
        <p className="mt-4 text-base md:text-lg text-[#A1A7B3] leading-relaxed">
          {description}
        </p>
      )}
    </div>
  );
}
