'use client';

import React, { useEffect, useState } from 'react';
import { cn } from '@/lib/utils';
import { useReducedMotion } from 'framer-motion';

interface SystemTraceProps {
  steps: string[];
  className?: string;
  speedMs?: number;
}

export function SystemTrace({
  steps,
  className,
  speedMs = 2200,
}: SystemTraceProps) {
  const [activeStep, setActiveStep] = useState(0);
  const shouldReduceMotion = useReducedMotion();

  useEffect(() => {
    if (shouldReduceMotion || steps.length <= 1) return;
    const interval = setInterval(() => {
      setActiveStep((prev) => (prev + 1) % steps.length);
    }, speedMs);
    return () => clearInterval(interval);
  }, [steps.length, speedMs, shouldReduceMotion]);

  return (
    <div
      className={cn(
        'inline-flex flex-wrap items-center gap-1.5 p-1.5 px-3 rounded-lg border border-white/[0.06] bg-[#0A0C10]/80 backdrop-blur-sm select-none',
        className
      )}
      aria-label={`System trace: ${steps.join(' -> ')}`}
    >
      <div className="flex items-center gap-1.5 text-[11px] font-mono text-[#6F7682] mr-1">
        <span className="w-1.5 h-1.5 rounded-full bg-[#7C8CFF] animate-pulse" />
        <span className="uppercase text-[10px] tracking-wider text-[#A1A7B3]">Trace</span>
      </div>

      {steps.map((step, idx) => {
        const isActive = activeStep === idx && !shouldReduceMotion;
        return (
          <React.Fragment key={step}>
            <div
              className={cn(
                'px-2 py-0.5 rounded text-[11px] font-mono transition-all duration-300',
                isActive
                  ? 'bg-[#7C8CFF]/20 text-[#7C8CFF] border border-[#7C8CFF]/40 shadow-sm shadow-[#7C8CFF]/20'
                  : 'bg-white/[0.02] text-[#A1A7B3] border border-white/[0.04]'
              )}
            >
              {step}
            </div>
            {idx < steps.length - 1 && (
              <span
                className={cn(
                  'text-[10px] font-mono transition-colors duration-300',
                  activeStep === idx && !shouldReduceMotion
                    ? 'text-[#7C8CFF]'
                    : 'text-[#6F7682]/60'
                )}
              >
                →
              </span>
            )}
          </React.Fragment>
        );
      })}
    </div>
  );
}
