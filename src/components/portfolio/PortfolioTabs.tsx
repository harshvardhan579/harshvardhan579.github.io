'use client';

import React from 'react';
import { cn } from '@/lib/utils';

export type PortfolioTabId = 'work' | 'experience' | 'skills' | 'education' | 'profile';

interface TabItem {
  id: PortfolioTabId;
  index: string;
  label: string;
  desc: string;
}

export const PORTFOLIO_TABS: TabItem[] = [
  { id: 'work', index: '01', label: 'WORK', desc: 'Flagship Systems & Schematics' },
  { id: 'experience', index: '02', label: 'EXPERIENCE', desc: 'Verified Industry Track Record' },
  { id: 'skills', index: '03', label: 'SKILLS', desc: 'AI, Backend & Infra Matrix' },
  { id: 'education', index: '04', label: 'EDUCATION', desc: 'RIT (MS) & Penn State (BS)' },
  { id: 'profile', index: '05', label: 'PROFILE', desc: 'Biography & Engineering Tenets' },
];

interface PortfolioTabsProps {
  activeTab: PortfolioTabId;
  onSelectTab: (tab: PortfolioTabId) => void;
  orientation?: 'vertical' | 'horizontal';
}

export function PortfolioTabs({
  activeTab,
  onSelectTab,
  orientation = 'vertical',
}: PortfolioTabsProps) {
  const handleKeyDown = (e: React.KeyboardEvent, index: number) => {
    let nextIndex = index;
    if (e.key === 'ArrowDown' || e.key === 'ArrowRight') {
      e.preventDefault();
      nextIndex = (index + 1) % PORTFOLIO_TABS.length;
    } else if (e.key === 'ArrowUp' || e.key === 'ArrowLeft') {
      e.preventDefault();
      nextIndex = (index - 1 + PORTFOLIO_TABS.length) % PORTFOLIO_TABS.length;
    } else if (e.key === 'Home') {
      e.preventDefault();
      nextIndex = 0;
    } else if (e.key === 'End') {
      e.preventDefault();
      nextIndex = PORTFOLIO_TABS.length - 1;
    }

    if (nextIndex !== index) {
      onSelectTab(PORTFOLIO_TABS[nextIndex].id);
    }
  };

  if (orientation === 'horizontal') {
    return (
      <div
        role="tablist"
        aria-label="Portfolio Sections"
        className="flex items-center gap-2 overflow-x-auto pb-2 scrollbar-none scroll-smooth w-full border-b border-[#BEB09C]"
      >
        {PORTFOLIO_TABS.map((tab, idx) => {
          const isActive = activeTab === tab.id;
          return (
            <button
              key={tab.id}
              role="tab"
              id={`tab-${tab.id}`}
              aria-selected={isActive}
              aria-controls={`panel-${tab.id}`}
              tabIndex={isActive ? 0 : -1}
              onClick={() => onSelectTab(tab.id)}
              onKeyDown={(e) => handleKeyDown(e, idx)}
              className={cn(
                'flex items-center gap-1.5 px-3 py-2 font-mono text-xs uppercase tracking-wider rounded-[2px] transition-all shrink-0 cursor-pointer select-none border',
                isActive
                  ? 'bg-[#1D1511] text-[#FAF6ED] border-[#1D1511] font-semibold'
                  : 'bg-[#E8DDCA]/60 text-[#5A463B] border-[#BEB09C] hover:bg-[#DFD3BF] hover:text-[#1D1511]'
              )}
            >
              <span className="text-[10px] opacity-75">{tab.index}</span>
              <span>{tab.label}</span>
            </button>
          );
        })}
      </div>
    );
  }

  return (
    <div
      role="tablist"
      aria-label="Portfolio Sections"
      aria-orientation="vertical"
      className="flex flex-col space-y-1.5 w-full"
    >
      <div className="font-mono text-[10px] text-[#857267] tracking-widest uppercase mb-2 px-1 border-b border-[#BEB09C]/60 pb-1">
        THE INDEX
      </div>

      {PORTFOLIO_TABS.map((tab, idx) => {
        const isActive = activeTab === tab.id;
        return (
          <button
            key={tab.id}
            role="tab"
            id={`tab-${tab.id}`}
            aria-selected={isActive}
            aria-controls={`panel-${tab.id}`}
            tabIndex={isActive ? 0 : -1}
            onClick={() => onSelectTab(tab.id)}
            onKeyDown={(e) => handleKeyDown(e, idx)}
            className={cn(
              'group w-full text-left p-3 rounded-[2px] border transition-all cursor-pointer select-none font-mono relative',
              isActive
                ? 'bg-[#1D1511] text-[#FAF6ED] border-[#1D1511] shadow-xs'
                : 'bg-[#FAF6ED]/70 text-[#5A463B] border-[#BEB09C] hover:bg-[#FAF6ED] hover:text-[#1D1511] hover:border-[#302019]'
            )}
          >
            <div className="flex items-baseline justify-between text-xs">
              <span className="font-semibold uppercase tracking-wider">
                {tab.index} {tab.label}
              </span>
              <span
                className={cn(
                  'text-[10px]',
                  isActive ? 'text-[#FAF6ED]/70' : 'text-[#857267]'
                )}
              >
                FILE 0{idx + 1}
              </span>
            </div>
            <div
              className={cn(
                'text-[11px] font-sans mt-0.5 line-clamp-1',
                isActive ? 'text-[#FAF6ED]/80' : 'text-[#857267]'
              )}
            >
              {tab.desc}
            </div>
          </button>
        );
      })}
    </div>
  );
}
