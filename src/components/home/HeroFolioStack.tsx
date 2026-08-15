'use client';

import React, { useState, useRef, useEffect, useCallback } from 'react';
import { motion, AnimatePresence, useReducedMotion } from 'framer-motion';
import Link from 'next/link';
import { ChevronUp, ChevronDown } from 'lucide-react';

interface FolioSheet {
  slug: string;
  index: string;
  title: string;
  field: string;
  year: string;
  summary: string;
  schematic: string;
}

const HERO_FOLIO_SHEETS: FolioSheet[] = [
  {
    slug: 'apiloom',
    index: '01',
    title: 'APILoom',
    field: 'AI INFRASTRUCTURE · APIS',
    year: '2026',
    summary: 'Plan separated from execution · One-use mutation token',
    schematic: 'PLAN → VALIDATE → REVIEW → EXECUTE',
  },
  {
    slug: 'civicpulse',
    index: '02',
    title: 'CivicPulse',
    field: 'APPLIED AI · NLP',
    year: '2026',
    summary: 'Bounded 5-stage workflow · Deterministic claim checks',
    schematic: 'INGEST → CRITIQUE → CHECK → REVIEW',
  },
  {
    slug: 'arcade-game',
    index: '03',
    title: 'Pocket Arcade',
    field: 'FRONTEND · CANVAS',
    year: '2026',
    summary: '5 original games · 0 KB assets · Web Audio synthesizer',
    schematic: 'PHASER 3 → AUDIO SYNTH → 60 FPS',
  },
  {
    slug: 'news-verify',
    index: '04',
    title: 'NewsVerify',
    field: 'MULTIMODAL AI · RAG',
    year: '2026',
    summary: 'Vision OCR claim extraction · Hard citation grounding',
    schematic: 'GPT-4O → DECOMPOSE → SERPAPI',
  },
  {
    slug: 'form-eval-app',
    index: '05',
    title: 'AI Form Evaluator',
    field: 'EDGE COMPUTER VISION · REAL-TIME',
    year: '2026',
    summary: 'Client-side pose landmarks · Zero video upload privacy',
    schematic: 'WEBCAM → MEDIAPIPE → WEBSOCKETS',
  },
  {
    slug: 'hybrid-ml-scheduler',
    index: '06',
    title: 'Hybrid ML Scheduler',
    field: 'REINFORCEMENT LEARNING · DISTRIBUTED',
    year: '2025',
    summary: 'Dueling DQN & Random Forest raced live vs optimal baseline',
    schematic: 'DQN → SIMULATE → EVALUATE',
  },
];

const GESTURE_THRESHOLD = 60; // Pixels of deltaY accumulation to trigger a switch
const GESTURE_LOCK_MS = 400;  // Cooldown lock between card animations
const GESTURE_SETTLE_MS = 160; // Inactivity ms to declare a gesture complete and clear boundary latch

export function HeroFolioStack() {
  const [activeIndex, setActiveIndex] = useState(0);
  const [direction, setDirection] = useState<'next' | 'prev'>('next');
  const [rotateX, setRotateX] = useState(0);
  const [rotateY, setRotateY] = useState(0);
  const [isRailHovered, setIsRailHovered] = useState(false);
  const prefersReduced = useReducedMotion();

  const containerRef = useRef<HTMLDivElement>(null);
  const railTrackRef = useRef<HTMLDivElement>(null);
  const accumDeltaYRef = useRef(0);
  const isLockedRef = useRef(false);
  const boundaryLatchRef = useRef<'bottom' | 'top' | null>(null);
  const settleTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const unlockTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const activeIndexRef = useRef(activeIndex);
  useEffect(() => {
    activeIndexRef.current = activeIndex;
  }, [activeIndex]);

  const currentSheet = HERO_FOLIO_SHEETS[activeIndex] || HERO_FOLIO_SHEETS[0];

  const switchProject = useCallback((targetIndex: number) => {
    if (targetIndex === activeIndexRef.current) return;
    if (targetIndex < 0 || targetIndex >= HERO_FOLIO_SHEETS.length) return;

    setDirection(targetIndex > activeIndexRef.current ? 'next' : 'prev');
    setActiveIndex(targetIndex);

    // Latch boundary if arriving at terminal projects from this gesture
    if (targetIndex === HERO_FOLIO_SHEETS.length - 1) {
      boundaryLatchRef.current = 'bottom';
    } else if (targetIndex === 0) {
      boundaryLatchRef.current = 'top';
    } else {
      boundaryLatchRef.current = null;
    }

    // Activate cooldown lock
    isLockedRef.current = true;
    if (unlockTimeoutRef.current) clearTimeout(unlockTimeoutRef.current);
    unlockTimeoutRef.current = setTimeout(() => {
      isLockedRef.current = false;
      accumDeltaYRef.current = 0;
    }, GESTURE_LOCK_MS);
  }, []);

  const goNext = useCallback(() => {
    if (activeIndexRef.current < HERO_FOLIO_SHEETS.length - 1) {
      switchProject(activeIndexRef.current + 1);
    }
  }, [switchProject]);

  const goPrev = useCallback(() => {
    if (activeIndexRef.current > 0) {
      switchProject(activeIndexRef.current - 1);
    }
  }, [switchProject]);

  // Scoped Wheel Interaction with Boundary Latch & Edge Release
  useEffect(() => {
    const containerEl = containerRef.current;
    if (!containerEl) return;

    const handleWheel = (e: WheelEvent) => {
      const deltaY = e.deltaY;
      const curIdx = activeIndexRef.current;

      // Always reset settling timer on every wheel event in the stream
      if (settleTimeoutRef.current) clearTimeout(settleTimeoutRef.current);
      settleTimeoutRef.current = setTimeout(() => {
        accumDeltaYRef.current = 0;
        boundaryLatchRef.current = null;
      }, GESTURE_SETTLE_MS);

      // BOUNDARY LATCH:
      // If we just arrived at the bottom (06) from this gesture, consume remaining downward momentum
      if (boundaryLatchRef.current === 'bottom' && deltaY > 0) {
        e.preventDefault();
        return;
      }
      // If we just arrived at the top (01) from this gesture, consume remaining upward momentum
      if (boundaryLatchRef.current === 'top' && deltaY < 0) {
        e.preventDefault();
        return;
      }

      // EDGE RELEASE ON NEW GESTURES (boundaryLatch === null):
      // At top project (01) scrolling up -> DO NOT INTERCEPT, let page scroll naturally
      if (curIdx === 0 && deltaY < 0) {
        return;
      }
      // At bottom project (06) scrolling down -> DO NOT INTERCEPT, let page scroll naturally
      if (curIdx === HERO_FOLIO_SHEETS.length - 1 && deltaY > 0) {
        return;
      }

      // Within active card navigation bounds: intercept and navigate projects
      e.preventDefault();

      if (isLockedRef.current) return;

      accumDeltaYRef.current += deltaY;

      if (accumDeltaYRef.current >= GESTURE_THRESHOLD) {
        accumDeltaYRef.current = 0;
        goNext();
      } else if (accumDeltaYRef.current <= -GESTURE_THRESHOLD) {
        accumDeltaYRef.current = 0;
        goPrev();
      }
    };

    containerEl.addEventListener('wheel', handleWheel, { passive: false });

    return () => {
      containerEl.removeEventListener('wheel', handleWheel);
      if (settleTimeoutRef.current) clearTimeout(settleTimeoutRef.current);
      if (unlockTimeoutRef.current) clearTimeout(unlockTimeoutRef.current);
    };
  }, [goNext, goPrev]);

  // Subtle 3D Pointer Parallax
  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    if (prefersReduced || !containerRef.current) return;
    const rect = containerRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left - rect.width / 2;
    const y = e.clientY - rect.top - rect.height / 2;

    const rY = (x / (rect.width / 2)) * 1.5;
    const rX = -(y / (rect.height / 2)) * 1.0;

    setRotateX(rX);
    setRotateY(rY);
  };

  const handleMouseLeave = () => {
    setRotateX(0);
    setRotateY(0);
  };

  // Rail Click Handler
  const handleTrackClick = (e: React.MouseEvent<HTMLDivElement>) => {
    const trackEl = railTrackRef.current;
    if (!trackEl) return;
    const rect = trackEl.getBoundingClientRect();
    const clickY = e.clientY - rect.top;
    const ratio = Math.max(0, Math.min(1, clickY / rect.height));
    const targetIdx = Math.round(ratio * (HERO_FOLIO_SHEETS.length - 1));
    switchProject(targetIdx);
  };

  // Keyboard navigation on rail
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      goNext();
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      goPrev();
    } else if (e.key === 'Home') {
      e.preventDefault();
      switchProject(0);
    } else if (e.key === 'End') {
      e.preventDefault();
      switchProject(HERO_FOLIO_SHEETS.length - 1);
    }
  };

  // Motion variants for physical 3D reshuffle
  const cardVariants = {
    initial: (dir: 'next' | 'prev') => {
      if (prefersReduced) return { opacity: 0 };
      return {
        opacity: 0.6,
        x: dir === 'next' ? 14 : -14,
        y: 8,
        z: -8,
        rotateY: dir === 'next' ? 2 : -2,
        rotateZ: dir === 'next' ? 0.7 : -0.7,
        scale: 0.985,
      };
    },
    animate: {
      opacity: 1,
      x: 0,
      y: 0,
      z: 24,
      rotateY: 0,
      rotateZ: 0,
      scale: 1,
      transition: {
        duration: prefersReduced ? 0.15 : 0.38,
        ease: [0.22, 1, 0.36, 1] as const,
      },
    },
    exit: (dir: 'next' | 'prev') => {
      if (prefersReduced) return { opacity: 0 };
      return {
        opacity: 0,
        x: dir === 'next' ? -20 : 20,
        y: 6,
        z: 8,
        rotateY: dir === 'next' ? -3 : 3,
        rotateZ: dir === 'next' ? -1 : 1,
        scale: 0.99,
        transition: {
          duration: prefersReduced ? 0.15 : 0.32,
          ease: [0.22, 1, 0.36, 1] as const,
        },
      };
    },
  };

  return (
    <div
      ref={containerRef}
      onMouseMove={handleMouseMove}
      onMouseLeave={handleMouseLeave}
      className="relative flex items-center gap-4 select-none perspective-1400 hidden lg:flex"
      aria-label="Interactive Hero Engineering Dossier Stack"
    >
      {/* 3D Dossier Card Stack Container */}
      <div className="relative w-[380px] xl:w-[420px] h-[320px]">
        {/* Layer 3 Background Sheet (Deepest) */}
        <div
          aria-hidden="true"
          className="absolute inset-0 bg-[#EEE8DC] border border-[#C9BBA6]/80 rounded-[2px] translate-x-[16px] translate-y-[16px] rotate-[2.5deg] shadow-xs pointer-events-none -z-30"
        />

        {/* Layer 2 Background Sheet */}
        <div
          aria-hidden="true"
          className="absolute inset-0 bg-[#E7DECF] border border-[#C9BBA6] rounded-[2px] translate-x-[8px] translate-y-[8px] rotate-[-1.5deg] shadow-paper pointer-events-none -z-20"
        />

        {/* Top Active Interactive Dossier Sheet */}
        <AnimatePresence mode="wait" custom={direction} initial={false}>
          <motion.div
            key={currentSheet.slug}
            custom={direction}
            variants={cardVariants}
            initial="initial"
            animate="animate"
            exit="exit"
            className="absolute inset-0 z-10"
            style={{
              rotateX,
              rotateY,
              transformStyle: 'preserve-3d',
            }}
          >
            <Link
              href={`/projects/${currentSheet.slug}`}
              className="block w-full h-full p-6 rounded-[2px] bg-[#FCF9F2] text-[#211914] border border-[#8F7D6B] shadow-paper-elevated hover:border-[#211914] transition-colors group cursor-pointer"
            >
              {/* Dossier Top Header */}
              <div className="flex items-center justify-between font-mono text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-2 mb-3">
                <span>FILE {currentSheet.index} / ARCHIVE DOSSIER</span>
                <span>{currentSheet.year}</span>
              </div>

              {/* Title & Field */}
              <div className="space-y-1">
                <div className="font-mono text-[10px] text-[#8A3F32] font-semibold tracking-wider uppercase">
                  {currentSheet.field}
                </div>
                <h3 className="font-display text-3xl text-[#211914] tracking-tight font-normal group-hover:text-[#8A3F32] transition-colors">
                  {currentSheet.title}
                </h3>
              </div>

              {/* Summary */}
              <p className="font-sans text-xs text-[#4A3C34] mt-2.5 line-clamp-2 leading-relaxed">
                {currentSheet.summary}
              </p>

              {/* Schematic Micro-Line */}
              <div className="mt-5 pt-3 border-t border-[#C9BBA6]/50 flex items-center justify-between font-mono text-[9px] text-[#75665B]">
                <span className="truncate max-w-[210px]">{currentSheet.schematic}</span>
                <span className="text-[#8A3F32] font-semibold flex-shrink-0 group-hover:underline">
                  VIEW DOSSIER ↗
                </span>
              </div>
            </Link>
          </motion.div>
        </AnimatePresence>
      </div>

      {/* HeroProjectRail: Vertical 3D Archive Depth Rail */}
      <div
        tabIndex={0}
        role="slider"
        aria-label="Hero Project Depth Rail"
        aria-valuemin={1}
        aria-valuemax={6}
        aria-valuenow={activeIndex + 1}
        aria-valuetext={`Project 0${activeIndex + 1} of 06: ${currentSheet.title}`}
        onKeyDown={handleKeyDown}
        onMouseEnter={() => setIsRailHovered(true)}
        onMouseLeave={() => setIsRailHovered(false)}
        className="relative flex flex-col items-center justify-between h-[300px] w-8 py-2 font-mono text-[9px] text-[#75665B] select-none focus:outline-none focus:ring-1 focus:ring-[#8A3F32] rounded-[2px]"
      >
        {/* Subtle Up Arrow */}
        <button
          onClick={goPrev}
          disabled={activeIndex === 0}
          aria-label="Previous project in stack"
          className={`p-0.5 rounded-[1px] transition-colors ${
            activeIndex === 0 ? 'opacity-30 cursor-not-allowed' : 'hover:text-[#211914] cursor-pointer'
          }`}
        >
          <ChevronUp className="w-3.5 h-3.5" />
        </button>

        {/* Top Bound Label */}
        <span className="font-semibold text-[#8A3F32]">01</span>

        {/* Vertical Track with Clickable Position Ticks */}
        <div
          ref={railTrackRef}
          onClick={handleTrackClick}
          className="relative flex-1 w-4 my-2 flex flex-col justify-between items-center cursor-pointer py-1"
        >
          {/* Track Line */}
          <div className="absolute inset-y-0 w-px bg-[#C9BBA6] left-1/2 -translate-x-1/2" />

          {/* 6 Discrete Position Ticks */}
          {HERO_FOLIO_SHEETS.map((sheet, idx) => {
            const isCurrent = idx === activeIndex;
            return (
              <button
                key={sheet.slug}
                onClick={(e) => {
                  e.stopPropagation();
                  switchProject(idx);
                }}
                aria-label={`Jump to project 0${idx + 1} ${sheet.title}`}
                className={`relative z-10 w-3.5 h-2 flex items-center justify-center transition-all ${
                  isCurrent ? 'scale-125' : 'opacity-60 hover:opacity-100'
                }`}
              >
                <div
                  className={`h-px transition-all ${
                    isCurrent
                      ? 'w-3.5 bg-[#8A3F32] h-[2px]'
                      : 'w-2 bg-[#75665B] hover:w-3 hover:bg-[#211914]'
                  }`}
                />
              </button>
            );
          })}

          {/* Draggable 3D Archival Thumb Indicator */}
          <motion.div
            className="absolute left-1/2 -translate-x-1/2 w-4 h-6 rounded-[2px] bg-[#211914] border border-[#8F7D6B] shadow-paper flex items-center justify-center pointer-events-none z-20"
            style={{
              top: `${(activeIndex / (HERO_FOLIO_SHEETS.length - 1)) * 88}%`,
            }}
            animate={{
              scale: isRailHovered ? 1.08 : 1,
            }}
            transition={{
              type: 'spring',
              stiffness: 320,
              damping: 26,
            }}
          >
            {/* Ivory Center Ridge Mark */}
            <div className="w-1.5 h-px bg-[#FCF9F2]" />
          </motion.div>
        </div>

        {/* Bottom Bound Label */}
        <span className="font-semibold text-[#75665B]">06</span>

        {/* Subtle Down Arrow */}
        <button
          onClick={goNext}
          disabled={activeIndex === HERO_FOLIO_SHEETS.length - 1}
          aria-label="Next project in stack"
          className={`p-0.5 rounded-[1px] transition-colors ${
            activeIndex === HERO_FOLIO_SHEETS.length - 1
              ? 'opacity-30 cursor-not-allowed'
              : 'hover:text-[#211914] cursor-pointer'
          }`}
        >
          <ChevronDown className="w-3.5 h-3.5" />
        </button>

        {/* Floating Tooltip Label on Hover */}
        <AnimatePresence>
          {isRailHovered && (
            <motion.div
              initial={{ opacity: 0, x: -6 }}
              animate={{ opacity: 1, x: 0 }}
              exit={{ opacity: 0, x: -6 }}
              transition={{ duration: 0.15 }}
              className="absolute right-full mr-3 top-1/2 -translate-y-1/2 bg-[#211914] text-[#FCF9F2] px-2.5 py-1 rounded-[2px] whitespace-nowrap shadow-paper-elevated pointer-events-none font-mono text-[10px] flex items-center gap-1.5 z-50"
            >
              <span className="text-[#E07A5F] font-bold">0{activeIndex + 1}</span>
              <span>—</span>
              <span className="uppercase">{currentSheet.title}</span>
            </motion.div>
          )}
        </AnimatePresence>
      </div>
    </div>
  );
}
