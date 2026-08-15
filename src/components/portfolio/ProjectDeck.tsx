'use client';

import React, { useState } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import { projectsData } from '@/data/projects';
import { Button } from '@/components/ui/Button';
import { Badge } from '@/components/ui/Badge';
import { SchematicFigure } from './SchematicFigure';
import { ArrowRight, ChevronLeft, ChevronRight, ExternalLink } from 'lucide-react';
import { GithubIcon } from '@/components/ui/Icons';
import { isLinkValid } from '@/data/contact';
import Link from 'next/link';

interface ProjectDeckProps {
  initialIndex?: number;
  onIndexChange?: (index: number) => void;
}

export function ProjectDeck({ initialIndex = 0, onIndexChange }: ProjectDeckProps) {
  const featuredProjects = projectsData.filter((p) => p.featured);
  const [activeIndex, setActiveIndex] = useState(initialIndex);
  const [direction, setDirection] = useState<'next' | 'prev'>('next');
  const prefersReduced = useReducedMotion();

  const currentProject = featuredProjects[activeIndex] || featuredProjects[0];

  const jumpToIndex = (newIndex: number) => {
    if (newIndex === activeIndex) return;
    if (newIndex < 0 || newIndex >= featuredProjects.length) return;

    setDirection(newIndex > activeIndex ? 'next' : 'prev');
    setActiveIndex(newIndex);
    onIndexChange?.(newIndex);
  };

  const goNext = () => {
    if (activeIndex < featuredProjects.length - 1) {
      jumpToIndex(activeIndex + 1);
    }
  };

  const goPrev = () => {
    if (activeIndex > 0) {
      jumpToIndex(activeIndex - 1);
    }
  };

  // Card slide variants
  const cardVariants = {
    initial: (dir: 'next' | 'prev') => {
      if (prefersReduced) return { opacity: 0 };
      return {
        opacity: 0,
        x: dir === 'next' ? 24 : -24,
        rotateY: dir === 'next' ? 2 : -2,
        z: -8,
        scale: 0.99,
      };
    },
    animate: {
      opacity: 1,
      x: 0,
      rotateY: 0,
      z: 0,
      scale: 1,
      transition: {
        duration: prefersReduced ? 0.15 : 0.36,
        ease: [0.22, 1, 0.36, 1] as const,
      },
    },
    exit: (dir: 'next' | 'prev') => {
      if (prefersReduced) return { opacity: 0 };
      return {
        opacity: 0,
        x: dir === 'next' ? -28 : 28,
        rotateY: dir === 'next' ? -3 : 3,
        z: 8,
        scale: 0.99,
        transition: {
          duration: prefersReduced ? 0.15 : 0.32,
          ease: [0.22, 1, 0.36, 1] as const,
        },
      };
    },
  };

  return (
    <div className="space-y-6 select-none focus:outline-none" tabIndex={0} aria-label="Selected Projects Dossier Desk">
      {/* Project Selector Header Bar */}
      <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-4 border-b border-[#C9BBA6] pb-4">
        {/* Horizontal Project Selector Buttons */}
        <div className="flex flex-wrap items-center gap-1.5 font-mono text-xs">
          {featuredProjects.map((p, idx) => {
            const isActive = idx === activeIndex;
            return (
              <button
                key={p.slug}
                onClick={() => jumpToIndex(idx)}
                className={`px-3 py-1.5 rounded-[2px] transition-all cursor-pointer select-none border text-left ${
                  isActive
                    ? 'bg-[#211914] text-[#FCF9F2] border-[#211914] font-semibold shadow-xs'
                    : 'bg-[#E7DECF]/70 text-[#4A3C34] border-[#C9BBA6] hover:bg-[#E7DECF] hover:text-[#211914]'
                }`}
                aria-label={`Select project ${p.title}`}
                aria-pressed={isActive}
              >
                <span>{p.index}</span> <span className="uppercase">{p.title}</span>
              </button>
            );
          })}
        </div>

        {/* Counter & Prev/Next Arrows */}
        <div className="flex items-center gap-3 font-mono text-xs text-[#75665B] self-end sm:self-auto">
          <span>
            FILE <strong className="text-[#211914]">0{activeIndex + 1}</strong> / 0{featuredProjects.length}
          </span>
          <div className="flex items-center gap-1">
            <button
              onClick={goPrev}
              disabled={activeIndex === 0}
              className={`p-1.5 border rounded-[2px] transition-colors cursor-pointer ${
                activeIndex === 0
                  ? 'bg-[#E7DECF]/40 text-[#75665B]/40 border-[#C9BBA6]/50 cursor-not-allowed'
                  : 'bg-[#E7DECF] hover:bg-[#C9BBA6] border-[#C9BBA6] text-[#211914]'
              }`}
              aria-label="Previous project"
            >
              <ChevronLeft className="w-4 h-4" />
            </button>
            <button
              onClick={goNext}
              disabled={activeIndex === featuredProjects.length - 1}
              className={`p-1.5 border rounded-[2px] transition-colors cursor-pointer ${
                activeIndex === featuredProjects.length - 1
                  ? 'bg-[#E7DECF]/40 text-[#75665B]/40 border-[#C9BBA6]/50 cursor-not-allowed'
                  : 'bg-[#E7DECF] hover:bg-[#C9BBA6] border-[#C9BBA6] text-[#211914]'
              }`}
              aria-label="Next project"
            >
              <ChevronRight className="w-4 h-4" />
            </button>
          </div>
        </div>
      </div>

      {/* Active Project Dossier Sheet with Physical Layering */}
      <div className="relative perspective-1400">
        {/* Visible Underlying Sheet Edges (Physical Horizontal Stack Metaphor) */}
        <div
          aria-hidden="true"
          className="absolute inset-0 bg-[#E7DECF] border border-[#C9BBA6] rounded-[2px] translate-x-[8px] translate-y-[8px] pointer-events-none -z-10"
        />
        <div
          aria-hidden="true"
          className="absolute inset-0 bg-[#EEE8DC] border border-[#C9BBA6]/80 rounded-[2px] translate-x-[16px] translate-y-[16px] pointer-events-none -z-20 hidden md:block"
        />

        <AnimatePresence mode="wait" custom={direction} initial={false}>
          <motion.div
            key={currentProject.slug}
            custom={direction}
            variants={cardVariants}
            initial="initial"
            animate="animate"
            exit="exit"
            className="bg-[#FCF9F2] border border-[#C9BBA6] p-6 sm:p-8 rounded-[2px] shadow-paper grid grid-cols-1 lg:grid-cols-12 gap-8 items-start"
          >
            {/* Left Column (7 cols): Editorial Information */}
            <div className="lg:col-span-7 space-y-4">
              <div className="flex items-center gap-2 flex-wrap">
                <Badge variant="bracket">{currentProject.category}</Badge>
                <span className="font-mono text-xs text-[#75665B]">/ {currentProject.year}</span>
              </div>

              <div>
                <h3 className="font-display text-3xl sm:text-4xl text-[#211914] font-normal tracking-tight">
                  {currentProject.title}
                </h3>
                <div className="font-mono text-xs text-[#75665B] mt-0.5">
                  {currentProject.subtitle}
                </div>
              </div>

              {/* Problem Statement Box */}
              <div className="bg-[#E7DECF]/50 border-l-2 border-[#8A3F32] p-3 rounded-r-[2px] text-xs text-[#211914] font-sans leading-relaxed">
                <strong className="font-mono text-[10px] text-[#8A3F32] block uppercase tracking-wider mb-0.5">
                  The Problem
                </strong>
                {currentProject.problem}
              </div>

              <p className="text-sm text-[#4A3C34] leading-relaxed font-sans">
                {currentProject.summary}
              </p>

              {/* Key Engineering Highlights */}
              <div className="space-y-1.5 pt-1">
                <div className="font-mono text-[10px] uppercase tracking-wider text-[#75665B]">
                  Key Architecture Decisions:
                </div>
                <ul className="space-y-1 text-xs text-[#211914] font-sans">
                  {currentProject.highlights.map((h, i) => (
                    <li key={i} className="flex items-start gap-2">
                      <span className="text-[#8A3F32] font-mono text-[10px] mt-0.5">▸</span>
                      <span>{h}</span>
                    </li>
                  ))}
                </ul>
              </div>

              {/* Technologies Row */}
              <div className="pt-2 border-t border-[#C9BBA6]/60">
                <div className="font-mono text-[10px] text-[#75665B] uppercase tracking-wider mb-1">
                  Engineering Stack:
                </div>
                <div className="font-mono text-xs text-[#211914] flex flex-wrap gap-x-2 gap-y-1">
                  {currentProject.technologies.map((t, i) => (
                    <span key={t} className="inline-flex items-center">
                      <span>{t}</span>
                      {i < currentProject.technologies.length - 1 && (
                        <span className="text-[#C9BBA6] ml-2">/</span>
                      )}
                    </span>
                  ))}
                </div>
              </div>

              {/* Action Buttons */}
              <div className="flex flex-wrap items-center gap-3 pt-4">
                <Button
                  href={`/projects/${currentProject.slug}`}
                  variant="primary"
                  size="md"
                  icon={<ArrowRight className="w-3.5 h-3.5" />}
                >
                  Open Case Study Dossier ↗
                </Button>

                {isLinkValid(currentProject.github) && (
                  <Button
                    href={currentProject.github}
                    isExternal
                    variant="secondary"
                    size="md"
                    icon={<GithubIcon className="w-3.5 h-3.5 fill-current" />}
                  >
                    Repository ↗
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
                    Live Demo ↗
                  </Button>
                )}
              </div>
            </div>

            {/* Right Column (5 cols): Printable Engineering Schematic Figure */}
            <div className="lg:col-span-5 space-y-4">
              <SchematicFigure slug={currentProject.slug} />

              {/* Verified Metric Callout */}
              {currentProject.metrics && currentProject.metrics.length > 0 && (
                <div className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-1">
                  <div className="font-mono text-[10px] text-[#75665B] uppercase tracking-wider">
                    Verified Telemetry:
                  </div>
                  <div className="font-display text-2xl text-[#211914] font-normal">
                    {currentProject.metrics[0].value}
                  </div>
                  <div className="font-mono text-xs text-[#4A3C34]">
                    {currentProject.metrics[0].label} — {currentProject.metrics[0].context}
                  </div>
                </div>
              )}
            </div>
          </motion.div>
        </AnimatePresence>
      </div>

      {/* Footer link to complete project table & academic archive */}
      <div className="pt-4 border-t border-[#C9BBA6] flex flex-col sm:flex-row items-start sm:items-center justify-between gap-2 font-mono text-xs text-[#75665B]">
        <span>ALL 6 CANONICAL PRODUCTION SYSTEMS & EXPERIMENTS</span>
        <div className="flex items-center gap-4">
          <Link
            href="/projects#academic"
            className="text-[#8A3F32] hover:text-[#211914] font-semibold underline underline-offset-4 decoration-[#8A3F32]/50 transition-colors"
          >
            Explore Academic Archive (12 Earlier Builds) →
          </Link>
          <Link
            href="/projects"
            className="text-[#211914] hover:text-[#8A3F32] font-semibold underline underline-offset-4 decoration-[#C9BBA6] transition-colors"
          >
            View Full Catalog →
          </Link>
        </div>
      </div>
    </div>
  );
}
