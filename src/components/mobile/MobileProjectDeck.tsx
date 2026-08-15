'use client';

import React, { useState } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { projectsData } from '@/data/projects';
import { SchematicFigure } from '@/components/portfolio/SchematicFigure';
import { Button } from '@/components/ui/Button';
import { Badge } from '@/components/ui/Badge';
import { ChevronLeft, ChevronRight, ChevronDown, Check, ArrowRight, ExternalLink } from 'lucide-react';
import { GithubIcon } from '@/components/ui/Icons';
import { isLinkValid } from '@/data/contact';
import Link from 'next/link';

interface MobileProjectDeckProps {
  initialIndex?: number;
  onIndexChange?: (index: number) => void;
}

export function MobileProjectDeck({ initialIndex = 0, onIndexChange }: MobileProjectDeckProps) {
  const featuredProjects = projectsData.filter((p) => p.featured);
  const [activeIndex, setActiveIndex] = useState(initialIndex);
  const [isSelectorOpen, setIsSelectorOpen] = useState(false);
  const prefersReduced = useReducedMotion();

  const currentProject = featuredProjects[activeIndex] || featuredProjects[0];

  const changeIndex = (newIndex: number) => {
    if (newIndex < 0 || newIndex >= featuredProjects.length) return;
    setActiveIndex(newIndex);
    setIsSelectorOpen(false);
    onIndexChange?.(newIndex);
  };

  const handleNext = () => {
    if (activeIndex < featuredProjects.length - 1) {
      changeIndex(activeIndex + 1);
    }
  };

  const handlePrev = () => {
    if (activeIndex > 0) {
      changeIndex(activeIndex - 1);
    }
  };

  return (
    <div className="space-y-4">
      {/* Top Header: Archival Project Selector Dropdown + Prev/Next Controls */}
      <div className="space-y-2 border-b border-[#C9BBA6] pb-3">
        <div className="flex items-center justify-between gap-2">
          {/* Accessible Project Selector Button */}
          <button
            onClick={() => setIsSelectorOpen(!isSelectorOpen)}
            aria-expanded={isSelectorOpen}
            aria-label="Select project file"
            className="flex-1 flex items-center justify-between px-3 py-2 bg-[#E7DECF] border border-[#C9BBA6] rounded-[2px] font-mono text-xs text-[#211914] hover:bg-[#DED3C1] active:bg-[#D5C9B5] transition-colors cursor-pointer"
          >
            <div className="flex items-center gap-2 truncate">
              <span className="font-bold text-[#8A3F32]">{currentProject.index}</span>
              <span className="font-semibold uppercase truncate">{currentProject.title}</span>
            </div>
            <ChevronDown
              className={`w-4 h-4 text-[#75665B] transition-transform duration-200 flex-shrink-0 ${
                isSelectorOpen ? 'rotate-180 text-[#8A3F32]' : ''
              }`}
            />
          </button>

          {/* Prev / Next controls */}
          <div className="flex items-center gap-1">
            <button
              onClick={handlePrev}
              disabled={activeIndex === 0}
              aria-label="Previous project"
              className={`w-9 h-9 flex items-center justify-center border rounded-[2px] transition-colors cursor-pointer ${
                activeIndex === 0
                  ? 'bg-[#E7DECF]/40 text-[#75665B]/40 border-[#C9BBA6]/50 cursor-not-allowed'
                  : 'bg-[#E7DECF] hover:bg-[#C9BBA6] border-[#C9BBA6] text-[#211914]'
              }`}
            >
              <ChevronLeft className="w-4 h-4" />
            </button>
            <button
              onClick={handleNext}
              disabled={activeIndex === featuredProjects.length - 1}
              aria-label="Next project"
              className={`w-9 h-9 flex items-center justify-center border rounded-[2px] transition-colors cursor-pointer ${
                activeIndex === featuredProjects.length - 1
                  ? 'bg-[#E7DECF]/40 text-[#75665B]/40 border-[#C9BBA6]/50 cursor-not-allowed'
                  : 'bg-[#E7DECF] hover:bg-[#C9BBA6] border-[#C9BBA6] text-[#211914]'
              }`}
            >
              <ChevronRight className="w-4 h-4" />
            </button>
          </div>
        </div>

        {/* Expandable Project List Dropdown */}
        <AnimatePresence>
          {isSelectorOpen && (
            <motion.div
              initial={{ opacity: 0, height: 0 }}
              animate={{ opacity: 1, height: 'auto' }}
              exit={{ opacity: 0, height: 0 }}
              transition={{ duration: 0.18 }}
              className="bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] shadow-paper divide-y divide-[#C9BBA6]/50 overflow-hidden"
            >
              {featuredProjects.map((project, idx) => {
                const isCurrent = idx === activeIndex;
                return (
                  <button
                    key={project.slug}
                    onClick={() => changeIndex(idx)}
                    className={`w-full px-3.5 py-2.5 flex items-center justify-between text-left font-mono text-xs transition-colors cursor-pointer ${
                      isCurrent
                        ? 'bg-[#211914] text-[#FCF9F2] font-semibold'
                        : 'text-[#4A3C34] hover:bg-[#E7DECF]/60 hover:text-[#211914]'
                    }`}
                  >
                    <div className="flex items-center gap-2.5">
                      <span className={isCurrent ? 'text-[#E07A5F]' : 'text-[#75665B]'}>
                        {project.index}
                      </span>
                      <span>{project.title}</span>
                    </div>
                    {isCurrent && <Check className="w-3.5 h-3.5 text-[#E07A5F]" />}
                  </button>
                );
              })}
            </motion.div>
          )}
        </AnimatePresence>

        {/* Compact Number Rail for Instant Tapping */}
        <div className="flex items-center gap-1 pt-1">
          {featuredProjects.map((p, idx) => {
            const isActive = idx === activeIndex;
            return (
              <button
                key={p.slug}
                onClick={() => changeIndex(idx)}
                aria-label={`Switch to project ${p.title}`}
                className={`flex-1 h-7 rounded-[2px] flex items-center justify-center font-mono text-[11px] transition-all border cursor-pointer ${
                  isActive
                    ? 'bg-[#211914] text-[#FCF9F2] border-[#211914] font-bold shadow-xs'
                    : 'bg-[#E7DECF]/70 text-[#75665B] border-[#C9BBA6] hover:bg-[#E7DECF]'
                }`}
              >
                {p.index}
              </button>
            );
          })}
        </div>
      </div>

      {/* Active Project Sheet with Optional Touch Swipe */}
      <AnimatePresence mode="wait" initial={false}>
        <motion.div
          key={currentProject.slug}
          drag={prefersReduced ? false : 'x'}
          dragConstraints={{ left: 0, right: 0 }}
          dragElastic={0.2}
          onDragEnd={(_, info) => {
            if (info.offset.x < -40) handleNext();
            if (info.offset.x > 40) handlePrev();
          }}
          initial={prefersReduced ? { opacity: 0 } : { opacity: 0, x: 16 }}
          animate={{ opacity: 1, x: 0 }}
          exit={prefersReduced ? { opacity: 0 } : { opacity: 0, x: -16 }}
          transition={{ duration: 0.22 }}
          className="space-y-4 touch-pan-y"
        >
          {/* Category & Year */}
          <div className="flex items-center justify-between font-mono text-xs text-[#75665B]">
            <Badge variant="bracket">{currentProject.category}</Badge>
            <span>{currentProject.year}</span>
          </div>

          {/* Project Title & Subtitle */}
          <div>
            <h3 className="font-display text-3xl text-[#211914] font-normal tracking-tight">
              {currentProject.title}
            </h3>
            <p className="font-mono text-xs text-[#75665B] mt-0.5">
              {currentProject.subtitle}
            </p>
          </div>

          {/* Problem Statement Box */}
          <div className="bg-[#E7DECF]/50 border-l-2 border-[#8A3F32] p-3 rounded-r-[2px] text-xs text-[#211914] font-sans leading-relaxed">
            <strong className="font-mono text-[10px] text-[#8A3F32] block uppercase tracking-wider mb-0.5">
              The Problem
            </strong>
            {currentProject.problem}
          </div>

          {/* Summary */}
          <p className="text-sm text-[#4A3C34] leading-relaxed font-sans">
            {currentProject.summary}
          </p>

          {/* Technical Highlights */}
          <div className="space-y-1 pt-1">
            <div className="font-mono text-[10px] uppercase tracking-wider text-[#75665B]">
              Key Architecture Decisions:
            </div>
            <ul className="space-y-1.5 text-xs text-[#211914] font-sans">
              {currentProject.highlights.slice(0, 3).map((h, i) => (
                <li key={i} className="flex items-start gap-2">
                  <span className="text-[#8A3F32] font-mono text-[10px] mt-0.5">▸</span>
                  <span>{h}</span>
                </li>
              ))}
            </ul>
          </div>

          {/* Technology Line */}
          <div className="pt-2 border-t border-[#C9BBA6]/60">
            <div className="font-mono text-[10px] text-[#75665B] uppercase tracking-wider mb-1">
              Stack:
            </div>
            <div className="font-mono text-xs text-[#211914] truncate">
              {currentProject.technologies.join(' / ')}
            </div>
          </div>

          {/* Responsive Schematic Figure */}
          <SchematicFigure slug={currentProject.slug} className="mt-2" />

          {/* Action Buttons */}
          <div className="flex flex-wrap items-center gap-2 pt-2">
            <Button
              href={`/projects/${currentProject.slug}`}
              variant="primary"
              size="md"
              icon={<ArrowRight className="w-3.5 h-3.5" />}
              className="flex-1 justify-center"
            >
              Case Study
            </Button>

            {isLinkValid(currentProject.github) && (
              <Button
                href={currentProject.github}
                isExternal
                variant="secondary"
                size="md"
                icon={<GithubIcon className="w-3.5 h-3.5 fill-current" />}
              >
                Source
              </Button>
            )}

            {isLinkValid(currentProject.live) && (
              <Button
                href={currentProject.live}
                isExternal
                variant="outline"
                size="md"
                icon={<ExternalLink className="w-3.5 h-3.5" />}
              >
                Live
              </Button>
            )}
          </div>
        </motion.div>
      </AnimatePresence>

      {/* Footer link to table archive & academic archive */}
      <div className="pt-3 border-t border-[#C9BBA6] flex flex-col gap-1.5 font-mono text-xs text-[#75665B]">
        <div className="flex items-center justify-between">
          <span>FILE 0{activeIndex + 1} / 0{featuredProjects.length}</span>
          <Link
            href="/projects#academic"
            className="text-[#8A3F32] font-semibold underline underline-offset-4 decoration-[#8A3F32]/50"
          >
            Academic Archive (12) →
          </Link>
        </div>
      </div>
    </div>
  );
}
