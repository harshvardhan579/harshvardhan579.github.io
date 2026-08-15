'use client';

import React, { useState } from 'react';
import { DossierTabId } from '../project/ProjectDossier';
import { ChevronDown } from 'lucide-react';

interface TabItem {
  id: DossierTabId;
  index: string;
  label: string;
}

interface MobileCaseStudySelectorProps {
  tabs: TabItem[];
  activeTab: DossierTabId;
  onSelectTab: (tab: DossierTabId) => void;
}

export function MobileCaseStudySelector({
  tabs,
  activeTab,
  onSelectTab,
}: MobileCaseStudySelectorProps) {
  const [isOpen, setIsOpen] = useState(false);
  const currentTab = tabs.find((t) => t.id === activeTab) || tabs[0];

  return (
    <div className="lg:hidden relative w-full mb-6 font-mono text-xs">
      {/* Trigger Button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-label="Select case study section"
        className="w-full flex items-center justify-between bg-[#FCF9F2] border-2 border-[#211914] p-3 rounded-[2px] shadow-paper font-semibold text-[#211914] cursor-pointer active:bg-[#E7DECF]"
      >
        <span className="flex items-center gap-2">
          <span className="text-[#8A3F32] font-bold">SECTION {currentTab.index} /</span>
          <span className="uppercase">{currentTab.label}</span>
        </span>
        <ChevronDown
          className={`w-4 h-4 text-[#211914] transition-transform ${
            isOpen ? 'rotate-180' : ''
          }`}
        />
      </button>

      {/* Popover / Dropdown Menu */}
      {isOpen && (
        <>
          <div
            className="fixed inset-0 z-40 bg-black/20"
            onClick={() => setIsOpen(false)}
          />
          <div className="absolute top-full left-0 right-0 z-50 mt-1 bg-[#FCF9F2] border-2 border-[#211914] rounded-[2px] shadow-paper-elevated overflow-hidden divide-y divide-[#C9BBA6]/60">
            {tabs.map((tab) => {
              const isActive = activeTab === tab.id;
              return (
                <button
                  key={tab.id}
                  onClick={() => {
                    onSelectTab(tab.id);
                    setIsOpen(false);
                  }}
                  className={`w-full flex items-center justify-between p-3.5 text-left transition-colors cursor-pointer ${
                    isActive
                      ? 'bg-[#211914] text-[#FCF9F2] font-semibold'
                      : 'text-[#211914] hover:bg-[#E7DECF]'
                  }`}
                >
                  <span className="flex items-center gap-2">
                    <span className={isActive ? 'text-[#C9BBA6]' : 'text-[#8A3F32]'}>
                      {tab.index}
                    </span>
                    <span className="uppercase">{tab.label}</span>
                  </span>
                  {isActive && <span className="text-[10px] text-[#C9BBA6]">ACTIVE</span>}
                </button>
              );
            })}
          </div>
        </>
      )}
    </div>
  );
}
