'use client';

import React from 'react';
import { contactData } from '@/data/contact';
import { Button } from '@/components/ui/Button';
import { Masthead } from '@/components/editorial/Masthead';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import { HeroFolioStack } from './HeroFolioStack';
import { ArrowDown, Mail, FileText } from 'lucide-react';

export function EditorialHero() {
  return (
    <section className="w-full pt-6 pb-12 sm:pb-16 px-4 sm:px-6 lg:px-8 bg-[#EEE8DC] border-b border-[#C9BBA6]/80">
      <div className="max-w-[1280px] mx-auto space-y-6 sm:space-y-8">
        {/* Newspaper Top Masthead */}
        <Masthead
          title="THE HARSHVARDHAN SINGH ARCHIVE"
          issue="VOL. 01 — 2026 EDITION"
          subline="SYSTEMS & AI ENGINEERING FOLIO"
        />

        {/* Hero Main Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-8 lg:gap-12 items-center min-h-[55svh] lg:min-h-[500px]">
          {/* Left Column (7 cols): Editorial Typography & Intentional Linebreaks */}
          <div className="lg:col-span-7 space-y-6">
            {/* Monospace Role Label */}
            <div className="flex items-center gap-2 flex-wrap font-mono text-xs text-[#8A3F32] font-semibold tracking-widest uppercase">
              <span>[ AI ENGINEER · SOFTWARE ENGINEER ]</span>
            </div>

            {/* Controlled Name Headline */}
            <h1 className="font-display text-5xl sm:text-6xl md:text-7xl lg:text-8xl text-[#211914] font-normal tracking-tight leading-[0.95] select-none">
              <span className="block">Harshvardhan</span>
              <span className="block italic text-[#8F7D6B] font-light">Singh</span>
            </h1>

            {/* Value Proposition Statement */}
            <p className="font-sans text-base sm:text-lg text-[#4A3C34] leading-relaxed max-w-xl">
              I build LLM and computer-vision products end-to-end with the guardrails, deterministic execution boundaries, and failure semantics that make them safe to run in production.
            </p>

            {/* Primary Action Controls */}
            <div className="flex flex-wrap items-center gap-3 pt-2">
              <Button
                href="#work"
                variant="primary"
                size="lg"
                icon={<ArrowDown className="w-4 h-4" />}
              >
                View Work
              </Button>

              <Button
                href="#contact"
                variant="secondary"
                size="lg"
                icon={<Mail className="w-4 h-4 text-[#75665B]" />}
              >
                Contact
              </Button>

              <Button
                href={contactData.resumeUrl}
                isExternal
                variant="outline"
                size="lg"
                icon={<FileText className="w-4 h-4" />}
                className="hidden sm:inline-flex"
              >
                Résumé PDF ↗
              </Button>
            </div>

            {/* Desktop Quick Metadata Row */}
            <div className="hidden lg:flex items-center gap-6 pt-4 border-t border-[#C9BBA6]/60 font-mono text-xs text-[#75665B]">
              <div>
                <span className="text-[#8A3F32] mr-1.5">LOCATION:</span>
                <span className="text-[#211914]">{contactData.location}</span>
              </div>
              <span>/</span>
              <div>
                <span className="text-[#8A3F32] mr-1.5">EMAIL:</span>
                <a
                  href={contactData.emailHref}
                  className="text-[#211914] hover:text-[#8A3F32] transition-colors underline decoration-[#C9BBA6]"
                >
                  {contactData.email}
                </a>
              </div>
            </div>
          </div>

          {/* Right Column (5 cols): Desktop Signature 3D Folio Stack */}
          <div className="lg:col-span-5 flex justify-center lg:justify-end">
            <HeroFolioStack />

            {/* Mobile Fallback: Compact Issue Card (0-767px) */}
            <div className="lg:hidden w-full bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] shadow-paper font-mono text-xs space-y-2">
              <div className="flex justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1">
                <span>ARCHIVE STATUS</span>
                <span className="text-[#8A3F32] font-semibold">6 ACTIVE CANONICAL FILES</span>
              </div>
              <div className="flex justify-between text-[11px]">
                <span className="text-[#75665B]">CORE FOCUS</span>
                <span className="text-[#211914]">AI Infrastructure & Distributed Systems</span>
              </div>
              <div className="flex justify-between text-[11px]">
                <span className="text-[#75665B]">EDUCATION</span>
                <span className="text-[#211914]">RIT (MS CS) · Penn State (BS CS)</span>
              </div>
            </div>
          </div>
        </div>

        <EditorialRule variant="single" className="pt-2 opacity-60" />
      </div>
    </section>
  );
}
