'use client';

import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { PortfolioTabs, PortfolioTabId, PORTFOLIO_TABS } from './PortfolioTabs';
import { ProjectDeck } from './ProjectDeck';
import { ExperienceFile } from './ExperienceFile';
import { SkillsIndex } from './SkillsIndex';
import { EducationFile } from './EducationFile';
import { ProfileFile } from './ProfileFile';
import { PaperCard } from '@/components/editorial/PaperCard';
import { MobilePortfolioDock } from '@/components/mobile/MobilePortfolioDock';
import { MobileProjectDeck } from '@/components/mobile/MobileProjectDeck';

export function PortfolioDesk() {
  const [activeTab, setActiveTab] = useState<PortfolioTabId>('work');
  const [savedProjectIndex, setSavedProjectIndex] = useState(0);
  const prefersReduced = useReducedMotion();

  // Deterministic hydration: subscribe to external URL state changes after mount
  useEffect(() => {
    const syncFromUrl = () => {
      const urlParams = new URLSearchParams(window.location.search);
      const tabParam = (urlParams.get('tab') || urlParams.get('section')) as PortfolioTabId;
      const hash = window.location.hash.replace('#', '') as PortfolioTabId;

      const validTab = (t: string): t is PortfolioTabId =>
        PORTFOLIO_TABS.some((tab) => tab.id === t);

      if (validTab(tabParam)) {
        setActiveTab(tabParam);
      } else if (validTab(hash)) {
        setActiveTab(hash);
      }
    };

    window.addEventListener('popstate', syncFromUrl);
    window.addEventListener('hashchange', syncFromUrl);
    const timer = setTimeout(syncFromUrl, 0);

    return () => {
      window.removeEventListener('popstate', syncFromUrl);
      window.removeEventListener('hashchange', syncFromUrl);
      clearTimeout(timer);
    };
  }, []);

  const handleTabChange = (tab: PortfolioTabId) => {
    setActiveTab(tab);
    if (typeof window !== 'undefined') {
      const url = new URL(window.location.href);
      url.searchParams.set('section', tab);
      url.searchParams.set('tab', tab);
      window.history.replaceState({}, '', url.toString());
    }
  };

  const renderDesktopPanel = () => {
    switch (activeTab) {
      case 'work':
        return (
          <ProjectDeck
            initialIndex={savedProjectIndex}
            onIndexChange={setSavedProjectIndex}
          />
        );
      case 'experience':
        return <ExperienceFile />;
      case 'skills':
        return <SkillsIndex />;
      case 'education':
        return <EducationFile />;
      case 'profile':
        return <ProfileFile />;
      default:
        return (
          <ProjectDeck
            initialIndex={savedProjectIndex}
            onIndexChange={setSavedProjectIndex}
          />
        );
    }
  };

  const renderMobilePanel = () => {
    switch (activeTab) {
      case 'work':
        return (
          <MobileProjectDeck
            initialIndex={savedProjectIndex}
            onIndexChange={setSavedProjectIndex}
          />
        );
      case 'experience':
        return <ExperienceFile />;
      case 'skills':
        return <SkillsIndex />;
      case 'education':
        return <EducationFile />;
      case 'profile':
        return <ProfileFile />;
      default:
        return (
          <MobileProjectDeck
            initialIndex={savedProjectIndex}
            onIndexChange={setSavedProjectIndex}
          />
        );
    }
  };

  return (
    <section id="work" className="w-full py-10 sm:py-16 px-4 sm:px-6 lg:px-8 bg-[#EEE8DC] scroll-mt-24 pb-28 lg:pb-16 relative">
      <div id="index" className="absolute -top-24 left-0 pointer-events-none" />
      <div className="max-w-[1280px] mx-auto">
        {/* Section Headline */}
        <div className="flex flex-col sm:flex-row sm:items-end justify-between gap-2 border-b border-[#C9BBA6] pb-4 mb-6 sm:mb-8">
          <div>
            <div className="font-mono text-xs text-[#8A3F32] font-semibold tracking-widest uppercase">
              PORTFOLIO INDEX
            </div>
            <h2 className="font-display text-4xl sm:text-5xl text-[#211914] font-normal tracking-tight mt-1">
              Selected Files & Dossiers
            </h2>
          </div>
          <div className="font-mono text-xs text-[#75665B] hidden sm:block">
            SWITCH TABS TO EXPLORE ARCHIVE IN PLACE
          </div>
        </div>

        {/* Mobile Viewport Engine (0px - 1023px) */}
        <div className="lg:hidden">
          <PaperCard stacked={false} marks={true} className="bg-[#FCF9F2] p-4 sm:p-6 border-[#C9BBA6] min-h-[480px]">
            <AnimatePresence mode="wait" initial={false}>
              <motion.div
                key={activeTab}
                role="tabpanel"
                id={`mobile-panel-${activeTab}`}
                aria-labelledby={`mobile-tab-${activeTab}`}
                initial={
                  prefersReduced
                    ? { opacity: 0 }
                    : { opacity: 0, x: 12, rotateY: 1.5 }
                }
                animate={{ opacity: 1, x: 0, rotateY: 0 }}
                exit={
                  prefersReduced
                    ? { opacity: 0 }
                    : { opacity: 0, x: -12, rotateY: -1.5 }
                }
                transition={{ duration: 0.22, ease: 'easeOut' }}
              >
                {renderMobilePanel()}
              </motion.div>
            </AnimatePresence>
          </PaperCard>

          {/* Sticky Mobile Portfolio Dock */}
          <MobilePortfolioDock activeTab={activeTab} onSelectTab={handleTabChange} />
        </div>

        {/* Desktop 2-Column Grid (>= 1024px) */}
        <div className="hidden lg:grid grid-cols-12 gap-8 items-start">
          {/* Desktop Left Index Column (3 cols / 25%) */}
          <div className="col-span-3 sticky top-20">
            <PortfolioTabs
              activeTab={activeTab}
              onSelectTab={handleTabChange}
              orientation="vertical"
            />
          </div>

          {/* Right Active Paper Card Panel (9 cols / 75%) with 3D Archival Reordering */}
          <div className="col-span-9 perspective-1400">
            <PaperCard stacked={true} marks={true} className="min-h-[580px] bg-[#FCF9F2]">
              <AnimatePresence mode="wait" initial={false}>
                <motion.div
                  key={activeTab}
                  role="tabpanel"
                  id={`panel-${activeTab}`}
                  aria-labelledby={`tab-${activeTab}`}
                  initial={
                    prefersReduced
                      ? { opacity: 0 }
                      : {
                          opacity: 0,
                          rotateY: 2.5,
                          x: 16,
                          scale: 0.99,
                          z: -8,
                        }
                  }
                  animate={{
                    opacity: 1,
                    rotateY: 0,
                    x: 0,
                    scale: 1,
                    z: 0,
                  }}
                  exit={
                    prefersReduced
                      ? { opacity: 0 }
                      : {
                          opacity: 0,
                          rotateY: -2.5,
                          x: -16,
                          scale: 0.99,
                          z: -8,
                        }
                  }
                  transition={{
                    duration: 0.28,
                    ease: 'easeOut',
                  }}
                  style={{ transformOrigin: 'left center' }}
                >
                  {renderDesktopPanel()}
                </motion.div>
              </AnimatePresence>
            </PaperCard>
          </div>
        </div>
      </div>
    </section>
  );
}
