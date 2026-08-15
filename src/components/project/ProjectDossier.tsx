'use client';

import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { Project } from '@/types/portfolio';
import { Badge } from '@/components/ui/Badge';
import { Button } from '@/components/ui/Button';
import { PaperCard } from '@/components/editorial/PaperCard';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import { CodeBlock } from '@/components/project/CodeBlock';
import { ProjectNav } from '@/components/project/ProjectNav';
import { GithubIcon } from '@/components/ui/Icons';
import { isLinkValid } from '@/data/contact';
import { MobileCaseStudySelector } from '@/components/mobile/MobileCaseStudySelector';
import { ArrowLeft, ExternalLink } from 'lucide-react';
import Link from 'next/link';

export type DossierTabId =
  | 'overview'
  | 'architecture'
  | 'aiml'
  | 'decisions'
  | 'challenges'
  | 'results';

interface DossierTab {
  id: DossierTabId;
  index: string;
  label: string;
}

export const DOSSIER_TABS: DossierTab[] = [
  { id: 'overview', index: '01', label: 'OVERVIEW' },
  { id: 'architecture', index: '02', label: 'ARCHITECTURE' },
  { id: 'aiml', index: '03', label: 'AI / ML' },
  { id: 'decisions', index: '04', label: 'DECISIONS' },
  { id: 'challenges', index: '05', label: 'CHALLENGES' },
  { id: 'results', index: '06', label: 'RESULTS' },
];

interface ProjectDossierProps {
  project: Project;
  prevProject: { slug: string; title: string } | null;
  nextProject: { slug: string; title: string } | null;
}

export function ProjectDossier({
  project,
  prevProject,
  nextProject,
}: ProjectDossierProps) {
  const [activeTab, setActiveTab] = useState<DossierTabId>('overview');
  const prefersReduced = useReducedMotion();

  // Deterministic hydration: subscribe to external URL state changes after mount
  useEffect(() => {
    const syncFromUrl = () => {
      const urlParams = new URLSearchParams(window.location.search);
      const tabParam = urlParams.get('tab') as DossierTabId;
      if (DOSSIER_TABS.some((t) => t.id === tabParam)) {
        setActiveTab(tabParam);
      }
    };

    window.addEventListener('popstate', syncFromUrl);
    const timer = setTimeout(syncFromUrl, 0);

    return () => {
      window.removeEventListener('popstate', syncFromUrl);
      clearTimeout(timer);
    };
  }, []);

  const handleTabChange = (tab: DossierTabId) => {
    setActiveTab(tab);
    if (typeof window !== 'undefined') {
      const url = new URL(window.location.href);
      url.searchParams.set('tab', tab);
      window.history.replaceState({}, '', url.toString());
    }
  };

  return (
    <div className="space-y-8">
      {/* Top Header / Metadata Masthead */}
      <div className="space-y-4 border-b border-[#C9BBA6] pb-6">
        <div className="flex items-center justify-between font-mono text-xs text-[#75665B]">
          <Link
            href="/#index"
            className="inline-flex items-center gap-1.5 text-[#211914] hover:text-[#8A3F32] transition-colors"
          >
            <ArrowLeft className="w-3.5 h-3.5" />
            <span>RETURN TO PORTFOLIO INDEX</span>
          </Link>
          <span>DOSSIER: {project.index}</span>
        </div>

        <div className="flex flex-col lg:flex-row lg:items-end justify-between gap-6">
          <div className="space-y-2">
            <div className="flex items-center gap-2 flex-wrap font-mono text-xs text-[#75665B]">
              <Badge variant="bracket">{project.category}</Badge>
              <span>/</span>
              <span>{project.year}</span>
            </div>
            <h1 className="font-display text-4xl sm:text-5xl md:text-6xl text-[#211914] font-normal tracking-tight">
              {project.title}
            </h1>
            <p className="font-mono text-xs sm:text-sm text-[#75665B]">
              {project.subtitle}
            </p>
          </div>

          <div className="flex items-center gap-3">
            {isLinkValid(project.github) && (
              <Button
                href={project.github}
                isExternal
                variant="outline"
                size="md"
                icon={<GithubIcon className="w-4 h-4 fill-current" />}
              >
                Repository ↗
              </Button>
            )}
            {isLinkValid(project.live) && (
              <Button
                href={project.live}
                isExternal
                variant="primary"
                size="md"
                icon={<ExternalLink className="w-4 h-4" />}
              >
                Live Deployment ↗
              </Button>
            )}
          </div>
        </div>
      </div>

      {/* Mobile Section Selector (Replaces horizontal tab wrap) */}
      <MobileCaseStudySelector
        tabs={DOSSIER_TABS}
        activeTab={activeTab}
        onSelectTab={handleTabChange}
      />

      {/* Main Dossier 2-Column Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-8 items-start">
        {/* Left Column (3 cols / 25%): Desktop Dossier Navigation & Quick Summary */}
        <div className="hidden lg:block lg:col-span-3 sticky top-20 space-y-6">
          <div role="tablist" className="space-y-1.5 font-mono text-xs">
            <div className="text-[10px] text-[#75665B] tracking-widest uppercase mb-2 px-1 border-b border-[#C9BBA6]/60 pb-1">
              DOSSIER SECTIONS
            </div>
            {DOSSIER_TABS.map((tab) => {
              const isActive = activeTab === tab.id;
              return (
                <button
                  key={tab.id}
                  role="tab"
                  aria-selected={isActive}
                  onClick={() => handleTabChange(tab.id)}
                  className={`w-full text-left p-2.5 rounded-[2px] border transition-all cursor-pointer select-none flex items-center justify-between ${
                    isActive
                      ? 'bg-[#211914] text-[#FCF9F2] border-[#211914] font-semibold'
                      : 'bg-[#FCF9F2] text-[#4A3C34] border-[#C9BBA6] hover:bg-[#E7DECF] hover:text-[#211914]'
                  }`}
                >
                  <span>
                    {tab.index} {tab.label}
                  </span>
                  <span className="text-[10px] opacity-60">→</span>
                </button>
              );
            })}
          </div>

          {/* Quick Technical Specs Card */}
          <div className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-3 font-mono text-xs">
            <div className="text-[10px] text-[#75665B] uppercase tracking-wider border-b border-[#C9BBA6]/60 pb-1">
              SYSTEM PROFILE
            </div>
            <div>
              <span className="text-[#75665B] block text-[10px]">CATEGORY</span>
              <span className="text-[#211914] font-medium">{project.category}</span>
            </div>
            <div>
              <span className="text-[#75665B] block text-[10px]">STACK</span>
              <div className="text-[#211914] flex flex-wrap gap-1 text-[11px] mt-0.5">
                {project.technologies.map((t) => (
                  <span key={t} className="bg-[#E7DECF]/80 px-1 py-0.5 rounded-[2px]">
                    {t}
                  </span>
                ))}
              </div>
            </div>
          </div>
        </div>

        {/* Right Column (9 cols / 75%): Active Dossier Content */}
        <div className="lg:col-span-9 perspective-1400">
          <PaperCard stacked={true} marks={true} className="min-h-[580px] bg-[#FCF9F2]">
            <AnimatePresence mode="wait" initial={false}>
              <motion.div
                key={activeTab}
                initial={
                  prefersReduced
                    ? { opacity: 0 }
                    : { opacity: 0, rotateY: 2, x: 16, scale: 0.99 }
                }
                animate={{ opacity: 1, rotateY: 0, x: 0, scale: 1 }}
                exit={
                  prefersReduced
                    ? { opacity: 0 }
                    : { opacity: 0, rotateY: -2, x: -16, scale: 0.99 }
                }
                transition={{ duration: 0.15 }}
                style={{ transformOrigin: 'left center' }}
                className="space-y-6"
              >
                {/* 01 OVERVIEW TAB */}
                {activeTab === 'overview' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 01 / PROBLEM & MOTIVATION
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        The Core Problem & Architecture Motivation
                      </h2>
                    </div>

                    <div className="bg-[#E7DECF]/50 border-l-2 border-[#8A3F32] p-4 rounded-r-[2px] space-y-1">
                      <div className="font-mono text-xs text-[#8A3F32] font-semibold uppercase">
                        The Challenge
                      </div>
                      <p className="text-sm text-[#211914] font-sans leading-relaxed">
                        {project.problem}
                      </p>
                    </div>

                    <div className="space-y-2">
                      <h3 className="font-mono text-xs uppercase tracking-wider text-[#75665B]">
                        Design Motivation
                      </h3>
                      <p className="text-sm sm:text-base text-[#4A3C34] font-sans leading-relaxed">
                        {project.motivation}
                      </p>
                    </div>

                    <div className="space-y-3 pt-2">
                      <h3 className="font-mono text-xs uppercase tracking-wider text-[#75665B]">
                        What It Accomplishes
                      </h3>
                      <ul className="space-y-2 text-sm text-[#211914] font-sans">
                        {project.whatItDoes.map((item, idx) => (
                          <li key={idx} className="flex items-start gap-2.5">
                            <span className="text-[#8A3F32] font-mono text-xs mt-0.5">▸</span>
                            <span>{item}</span>
                          </li>
                        ))}
                      </ul>
                    </div>
                  </div>
                )}

                {/* 02 ARCHITECTURE TAB */}
                {activeTab === 'architecture' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 02 / SYSTEM TOPOLOGY
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        System Architecture Topology
                      </h2>
                    </div>

                    <div className="space-y-4">
                      {project.architectureLayers && project.architectureLayers.length > 0 ? (
                        project.architectureLayers.map((layer, idx) => (
                          <div
                            key={layer.name}
                            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-2 shadow-xs"
                          >
                            <div className="flex items-baseline justify-between font-mono text-xs border-b border-[#C9BBA6]/60 pb-1.5">
                              <span className="font-semibold text-[#211914] uppercase">
                                LAYER 0{idx + 1}: {layer.name}
                              </span>
                              <span className="text-[#75665B]">L0{idx + 1}</span>
                            </div>

                            <p className="text-xs sm:text-sm text-[#4A3C34] font-sans leading-relaxed">
                              {layer.description}
                            </p>

                            <div className="font-mono text-xs text-[#211914] flex flex-wrap gap-1.5 pt-1">
                              {layer.components.map((c) => (
                                <span
                                  key={c}
                                  className="bg-[#E7DECF]/80 border border-[#C9BBA6] px-2 py-0.5 rounded-[2px]"
                                >
                                  {c}
                                </span>
                              ))}
                            </div>
                          </div>
                        ))
                      ) : (
                        <p className="text-sm text-[#4A3C34] font-sans">
                          Architecture specifications documented directly in code artifacts.
                        </p>
                      )}
                    </div>
                  </div>
                )}

                {/* 03 AI / ML TAB */}
                {activeTab === 'aiml' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 03 / AI & ML GUARDRAILS
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        Model Architecture & Validation Schemas
                      </h2>
                    </div>

                    {project.aiDecisions && project.aiDecisions.length > 0 ? (
                      <div className="space-y-4">
                        {project.aiDecisions.map((ai, idx) => (
                          <div
                            key={idx}
                            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-2 shadow-xs"
                          >
                            <h3 className="font-display text-xl text-[#211914]">
                              {ai.title}
                            </h3>
                            <p className="text-xs sm:text-sm text-[#4A3C34] font-sans leading-relaxed">
                              {ai.rationale}
                            </p>
                            {ai.tradeOff && (
                              <div className="font-mono text-xs bg-[#E7DECF]/60 p-2 rounded-[2px] border border-[#C9BBA6]/60 text-[#8A3F32]">
                                <strong>Trade-off:</strong> {ai.tradeOff}
                              </div>
                            )}
                          </div>
                        ))}
                      </div>
                    ) : (
                      <p className="text-sm text-[#4A3C34] font-sans">
                        This system focuses primarily on deterministic systems programming and algorithmic performance.
                      </p>
                    )}

                    {project.codeSnippets && project.codeSnippets.length > 0 && (
                      <div className="space-y-4 pt-2">
                        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                          Implementation Artifacts:
                        </div>
                        {project.codeSnippets.map((cs, idx) => (
                          <CodeBlock
                            key={idx}
                            code={cs.code}
                            language={cs.language}
                            filename={cs.filename}
                            caption={cs.caption}
                          />
                        ))}
                      </div>
                    )}
                  </div>
                )}

                {/* 04 DECISIONS TAB */}
                {activeTab === 'decisions' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 04 / SYSTEM DESIGN
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        Architectural Decisions & Trade-offs
                      </h2>
                    </div>

                    <div className="space-y-4">
                      {project.systemDesignDecisions && project.systemDesignDecisions.length > 0 ? (
                        project.systemDesignDecisions.map((sd, idx) => (
                          <div
                            key={idx}
                            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] shadow-xs space-y-2"
                          >
                            <div className="font-display text-xl text-[#211914]">
                              {sd.title}
                            </div>
                            <p className="text-xs sm:text-sm text-[#4A3C34] font-sans leading-relaxed">
                              {sd.rationale}
                            </p>
                            {sd.tradeOff && (
                              <div className="bg-[#E7DECF]/60 p-2.5 rounded-[2px] font-mono text-xs text-[#8A3F32]">
                                <span className="block text-[10px] uppercase text-[#75665B]">
                                  Trade-off Accepted
                                </span>
                                {sd.tradeOff}
                              </div>
                            )}
                          </div>
                        ))
                      ) : (
                        <p className="text-sm text-[#4A3C34] font-sans">
                          Decisions documented alongside architecture topology layers.
                        </p>
                      )}
                    </div>
                  </div>
                )}

                {/* 05 CHALLENGES TAB */}
                {activeTab === 'challenges' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 05 / ENGINEERING CHALLENGES
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        Failure Modes & Overcome Obstacles
                      </h2>
                    </div>

                    <div className="space-y-4">
                      {project.challenges && project.challenges.length > 0 ? (
                        project.challenges.map((ch, idx) => (
                          <div
                            key={idx}
                            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-2.5 shadow-xs"
                          >
                            <div className="font-display text-xl text-[#211914]">
                              {ch.title}
                            </div>
                            <div className="space-y-1.5 text-xs sm:text-sm font-sans">
                              <div className="text-[#8A3F32]">
                                <strong className="font-mono text-[10px] uppercase tracking-wider block">
                                  Failure Mode / Problem:
                                </strong>
                                {ch.problem}
                              </div>
                              <div className="text-[#211914]">
                                <strong className="font-mono text-[10px] uppercase tracking-wider block text-[#75665B]">
                                  Engineered Solution:
                                </strong>
                                {ch.solution}
                              </div>
                              <div className="bg-[#E7DECF]/60 p-2.5 rounded-[2px] border border-[#C9BBA6]/60 text-[#4A3C34] font-mono text-xs">
                                <strong>Key Takeaway:</strong> {ch.takeaway}
                              </div>
                            </div>
                          </div>
                        ))
                      ) : (
                        <p className="text-sm text-[#4A3C34] font-sans">
                          Standard deterministic assertions and testing harnesses verified all test cases.
                        </p>
                      )}
                    </div>
                  </div>
                )}

                {/* 06 RESULTS TAB */}
                {activeTab === 'results' && (
                  <div className="space-y-6">
                    <div className="border-b border-[#C9BBA6] pb-3">
                      <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                        SECTION 06 / TELEMETRY & ROADMAP
                      </div>
                      <h2 className="font-display text-3xl text-[#211914] font-normal mt-1">
                        Verified Telemetry & Future Scaling
                      </h2>
                    </div>

                    {project.metrics && project.metrics.length > 0 && (
                      <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
                        {project.metrics.map((m, idx) => (
                          <div
                            key={idx}
                            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] shadow-xs space-y-1"
                          >
                            <div className="font-display text-3xl text-[#211914] font-normal">
                              {m.value}
                            </div>
                            <div className="font-mono text-xs text-[#8A3F32] font-semibold">
                              {m.label}
                            </div>
                            <div className="font-mono text-[11px] text-[#75665B]">
                              {m.context}
                            </div>
                          </div>
                        ))}
                      </div>
                    )}

                    {project.improvements && project.improvements.length > 0 && (
                      <div className="space-y-3 pt-3 border-t border-[#C9BBA6]/60">
                        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
                          Engineering Roadmap & Next Improvements:
                        </div>
                        <ul className="space-y-2 text-sm text-[#211914] font-sans">
                          {project.improvements.map((item, idx) => (
                            <li key={idx} className="flex items-start gap-2.5">
                              <span className="text-[#8A3F32] font-mono text-xs mt-0.5">▸</span>
                              <span>{item}</span>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}
                  </div>
                )}
              </motion.div>
            </AnimatePresence>
          </PaperCard>
        </div>
      </div>

      {/* Prev / Next Navigation Footer */}
      <EditorialRule variant="single" className="my-8" />
      <ProjectNav prevProject={prevProject} nextProject={nextProject} />
    </div>
  );
}
