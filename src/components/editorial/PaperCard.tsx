import React from 'react';
import { cn } from '@/lib/utils';
import { RegistrationMark } from './RegistrationMark';

interface PaperCardProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode;
  className?: string;
  stacked?: boolean;
  inverted?: boolean;
  marks?: boolean;
}

export function PaperCard({
  children,
  className,
  stacked = true,
  inverted = false,
  marks = true,
  ...props
}: PaperCardProps) {
  return (
    <div className="relative group w-full">
      {/* Layered Paper Sheets Behind */}
      {stacked && !inverted && (
        <>
          <div
            className="absolute inset-0 bg-[#EEE8DC] border border-[#C9BBA6]/80 translate-x-2 translate-y-2 rounded-[2px] pointer-events-none hidden sm:block"
            aria-hidden="true"
          />
          <div
            className="absolute inset-0 bg-[#E7DECF] border border-[#C9BBA6] translate-x-1 translate-y-1 rounded-[2px] pointer-events-none"
            aria-hidden="true"
          />
        </>
      )}

      {/* Main Top Paper Sheet */}
      <div
        className={cn(
          'relative z-10 rounded-[2px] border transition-all duration-300 p-6 sm:p-8',
          inverted
            ? 'bg-[#211914] border-[#302019] text-[#FCF9F2] shadow-paper-lg'
            : 'bg-[#FCF9F2] border-[#C9BBA6] text-[#211914] shadow-paper',
          className
        )}
        {...props}
      >
        {marks && (
          <>
            <RegistrationMark position="top-left" />
            <RegistrationMark position="top-right" />
            <RegistrationMark position="bottom-left" />
            <RegistrationMark position="bottom-right" />
          </>
        )}
        {children}
      </div>
    </div>
  );
}
