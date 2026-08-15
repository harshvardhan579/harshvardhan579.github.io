'use client';

import React from 'react';
import { PortfolioTabId } from '../portfolio/PortfolioTabs';

interface MobilePortfolioDockProps {
  activeTab: PortfolioTabId;
  onSelectTab: (tab: PortfolioTabId) => void;
}

interface DockItem {
  id: PortfolioTabId;
  label: string;
  index: string;
}

const DOCK_ITEMS: DockItem[] = [
  { id: 'work', label: 'WORK', index: '01' },
  { id: 'experience', label: 'EXP', index: '02' },
  { id: 'skills', label: 'SKILLS', index: '03' },
  { id: 'education', label: 'EDU', index: '04' },
  { id: 'profile', label: 'ME', index: '05' },
];

export function MobilePortfolioDock({
  activeTab,
  onSelectTab,
}: MobilePortfolioDockProps) {
  return (
    <div className="lg:hidden fixed bottom-0 left-0 right-0 z-40 bg-[#FCF9F2] border-t border-[#8F7D6B] shadow-paper-elevated pb-[env(safe-area-inset-bottom,0px)]">
      <nav
        role="tablist"
        aria-label="Mobile Portfolio Sections"
        className="flex items-center justify-around h-14 max-w-md mx-auto px-2"
      >
        {DOCK_ITEMS.map((item) => {
          const isActive = activeTab === item.id;
          return (
            <button
              key={item.id}
              id={`mobile-tab-${item.id}`}
              role="tab"
              aria-selected={isActive}
              aria-controls={`mobile-panel-${item.id}`}
              onClick={() => onSelectTab(item.id)}
              className={`flex-1 flex flex-col items-center justify-center h-full py-1 font-mono text-xs whitespace-nowrap cursor-pointer transition-all select-none ${isActive
                  ? 'text-[#211914] font-semibold border-t-2 border-[#211914] -mt-[2px] bg-[#E7DECF]/40'
                  : 'text-[#75665B] hover:text-[#211914]'
                }`}
            >
              <span className="text-[9px] opacity-70 leading-tight">{item.index}</span>
              <span className="tracking-wider uppercase text-[11px] leading-tight">{item.label}</span>
            </button>
          );
        })}
      </nav>
    </div>
  );
}
