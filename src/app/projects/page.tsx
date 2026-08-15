'use client';

import React, { useState } from 'react';
import { projectsData } from '@/data/projects';
import { academicProjectsData, academicYears } from '@/data/academicProjects';
import { AcademicProject } from '@/types/portfolio';
import { Masthead } from '@/components/editorial/Masthead';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import { Badge } from '@/components/ui/Badge';
import { ArrowLeft, ArrowUpRight, ChevronDown, ChevronUp } from 'lucide-react';
import { GithubIcon } from '@/components/ui/Icons';
import { isLinkValid } from '@/data/contact';
import Link from 'next/link';

const CATEGORIES = [
  'ALL',
  'AI / ML',
  'SYSTEMS',
  'DATA',
  'WEB',
  'HARDWARE',
] as const;

type CategoryFilter = (typeof CATEGORIES)[number];

export default function ProjectsArchivePage() {
  const [activeCategory, setActiveCategory] = useState<CategoryFilter>('ALL');
  const [expandedAcademicSlug, setExpandedAcademicSlug] = useState<string | null>(null);

  const toggleAcademicExpand = (slug: string) => {
    setExpandedAcademicSlug((prev) => (prev === slug ? null : slug));
  };

  const matchesCategory = (categoryStr: string, filter: CategoryFilter) => {
    if (filter === 'ALL') return true;
    const cat = categoryStr.toUpperCase();
    if (filter === 'AI / ML') {
      return cat.includes('AI') || cat.includes('ML') || cat.includes('VISION') || cat.includes('NLP') || cat.includes('DEEP LEARNING') || cat.includes('REINFORCEMENT');
    }
    if (filter === 'SYSTEMS') {
      return cat.includes('SYSTEMS') || cat.includes('OPERATING') || cat.includes('ARCHITECTURE') || cat.includes('FILE') || cat.includes('INFRASTRUCTURE') || cat.includes('BACKEND');
    }
    if (filter === 'DATA') {
      return cat.includes('DATA') || cat.includes('DATABASE') || cat.includes('ANALYTICS') || cat.includes('SQL');
    }
    if (filter === 'WEB') {
      return cat.includes('WEB') || cat.includes('FRONTEND') || cat.includes('FULL STACK') || cat.includes('BLOCKCHAIN') || cat.includes('INTERACTIVE');
    }
    if (filter === 'HARDWARE') {
      return cat.includes('HARDWARE') || cat.includes('PROCESSOR') || cat.includes('VERILOG') || cat.includes('COMPUTER ARCHITECTURE');
    }
    return true;
  };

  const filteredFlagship = projectsData.filter((project) =>
    matchesCategory(project.category, activeCategory)
  );

  const filteredAcademic = academicProjectsData.filter((project) =>
    matchesCategory(project.category, activeCategory)
  );

  const hasBothSections = filteredFlagship.length > 0 && filteredAcademic.length > 0;
  const hasNoMatches = filteredFlagship.length === 0 && filteredAcademic.length === 0;

  return (
    <div className="w-full py-8 sm:py-12 px-4 sm:px-6 lg:px-8 bg-[#EEE8DC]">
      <div className="max-w-[1280px] mx-auto space-y-10">
        {/* Back Link & Header */}
        <div className="flex items-center justify-between font-mono text-xs text-[#75665B]">
          <Link
            href="/#work"
            className="inline-flex items-center gap-1.5 text-[#211914] hover:text-[#8A3F32] transition-colors"
          >
            <ArrowLeft className="w-3.5 h-3.5" />
            <span>RETURN TO PORTFOLIO INDEX</span>
          </Link>
          <span>TOTAL CATALOG: 0{projectsData.length + academicProjectsData.length} RECORDS</span>
        </div>

        <Masthead
          title="THE HARSHVARDHAN SINGH ARCHIVE"
          issue="VOL. 01 — COMPLETE TECHNICAL RECORD"
          subline="FLAGSHIP & ACADEMIC CATALOG"
        />

        <div className="space-y-3">
          <div className="font-mono text-xs text-[#8A3F32] font-semibold tracking-widest uppercase">
            TECHNICAL ARCHIVE / 2020—2026
          </div>
          <h1 className="font-display text-4xl sm:text-5xl md:text-6xl text-[#211914] font-normal tracking-tight">
            Systems, Platforms & Research Archive
          </h1>
          <p className="font-sans text-sm sm:text-base text-[#4A3C34] max-w-3xl leading-relaxed">
            A complete historical catalog covering six primary flagship production architectures and twelve exploratory systems spanning deep learning, distributed runbooks, operating systems, and computer architecture.
          </p>
        </div>

        {/* Quick Navigation Jump Links & Category Filters */}
        <div className="flex flex-col md:flex-row md:items-center justify-between gap-4 pt-4 border-t border-[#C9BBA6]/80">
          {/* Category Filter Pills */}
          <div className="flex flex-wrap items-center gap-2">
            <span className="font-mono text-xs text-[#75665B] mr-2 uppercase">Filter:</span>
            {CATEGORIES.map((cat) => {
              const isActive = activeCategory === cat;
              return (
                <button
                  key={cat}
                  onClick={() => setActiveCategory(cat)}
                  className={`px-3 py-1 font-mono text-xs rounded-[2px] transition-all cursor-pointer border ${
                    isActive
                      ? 'bg-[#211914] text-[#FCF9F2] border-[#211914] font-semibold shadow-xs'
                      : 'bg-[#FCF9F2] text-[#4A3C34] border-[#C9BBA6] hover:bg-[#E7DECF] hover:text-[#211914]'
                  }`}
                >
                  {cat}
                </button>
              );
            })}
          </div>

          {/* Dynamic Section Anchors (Only present when corresponding section exists) */}
          <div className="flex items-center gap-4 font-mono text-xs text-[#75665B]">
            {filteredFlagship.length > 0 && (
              <a href="#selected" className="hover:text-[#211914] underline underline-offset-4 decoration-[#C9BBA6]">
                ↓ SELECTED BUILDS ({filteredFlagship.length < 10 ? `0${filteredFlagship.length}` : filteredFlagship.length})
              </a>
            )}
            {filteredAcademic.length > 0 && (
              <a href="#academic" className="hover:text-[#211914] underline underline-offset-4 decoration-[#C9BBA6]">
                ↓ ACADEMIC ARCHIVE ({filteredAcademic.length < 10 ? `0${filteredAcademic.length}` : filteredAcademic.length})
              </a>
            )}
          </div>
        </div>

        {/* ========================================================================= */}
        {/* SECTION 1: SELECTED BUILDS (Rendered strictly when filtered count > 0)    */}
        {/* ========================================================================= */}
        {filteredFlagship.length > 0 && (
          <section id="selected" className="space-y-6 pt-4 scroll-mt-20">
            <div className="flex items-baseline justify-between border-b-2 border-[#211914] pb-2">
              <div>
                <span className="font-mono text-[10px] text-[#8A3F32] font-semibold uppercase tracking-wider block">
                  FLAGSHIP ENGINEERING
                </span>
                <h2 className="font-display text-2xl sm:text-3xl text-[#211914]">
                  Selected Builds / 2025—2026
                </h2>
              </div>
              <span className="font-mono text-xs text-[#75665B]">
                0{filteredFlagship.length} OF 06 ENTRIES
              </span>
            </div>

            {/* Mobile View: Compact Record Cards (0px - 767px) */}
            <div className="md:hidden space-y-4">
              {filteredFlagship.map((project) => (
                <div
                  key={project.slug}
                  className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] shadow-paper space-y-3"
                >
                  <div className="flex items-center justify-between font-mono text-xs text-[#75665B]">
                    <span className="text-[#8A3F32] font-semibold">FILE {project.index}</span>
                    <span>{project.year}</span>
                  </div>

                  <div>
                    <h3 className="font-display text-2xl text-[#211914]">
                      {project.title}
                    </h3>
                    <div className="font-mono text-[11px] text-[#75665B] mt-0.5">
                      {project.category}
                    </div>
                  </div>

                  <p className="text-xs text-[#4A3C34] font-sans leading-relaxed">
                    {project.summary}
                  </p>

                  <div className="pt-2 border-t border-[#C9BBA6]/60 flex items-center justify-between font-mono text-xs">
                    <div className="text-[#75665B] truncate max-w-[180px]">
                      {project.technologies.slice(0, 3).join(' / ')}
                    </div>
                    <div className="flex items-center gap-3">
                      {isLinkValid(project.github) && (
                        <a
                          href={project.github}
                          target="_blank"
                          rel="noopener noreferrer"
                          className="text-[#211914] hover:text-[#8A3F32] flex items-center gap-1"
                        >
                          <span>SOURCE</span>
                          <ArrowUpRight className="w-3 h-3" />
                        </a>
                      )}
                      <Link
                        href={`/projects/${project.slug}`}
                        className="text-[#8A3F32] font-semibold flex items-center gap-1"
                      >
                        <span>DOSSIER</span>
                        <ArrowUpRight className="w-3 h-3" />
                      </Link>
                    </div>
                  </div>
                </div>
              ))}
            </div>

            {/* Desktop View: Archival Table (>= 768px) */}
            <div className="hidden md:block bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] shadow-paper overflow-hidden">
              <table className="w-full text-left font-mono text-xs border-collapse">
                <thead>
                  <tr className="bg-[#E7DECF]/80 border-b border-[#C9BBA6] text-[#75665B]">
                    <th className="py-3 px-4 font-semibold w-16">FILE</th>
                    <th className="py-3 px-4 font-semibold">PROJECT & OVERVIEW</th>
                    <th className="py-3 px-4 font-semibold">CATEGORY</th>
                    <th className="py-3 px-4 font-semibold">TECHNOLOGIES</th>
                    <th className="py-3 px-4 font-semibold text-right">ACCESS</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-[#C9BBA6]/60">
                  {filteredFlagship.map((project) => (
                    <tr
                      key={project.slug}
                      className="hover:bg-[#E7DECF]/40 transition-colors group"
                    >
                      <td className="py-4 px-4 align-top text-[#75665B]">
                        <span className="font-semibold text-[#211914]">{project.index}</span>
                        <span className="block text-[10px] text-[#75665B] mt-0.5">{project.year}</span>
                      </td>
                      <td className="py-4 px-4 align-top max-w-sm">
                        <Link
                          href={`/projects/${project.slug}`}
                          className="font-display text-xl text-[#211914] group-hover:text-[#8A3F32] transition-colors"
                        >
                          {project.title}
                        </Link>
                        <p className="text-xs text-[#4A3C34] font-sans mt-1 line-clamp-2">
                          {project.summary}
                        </p>
                      </td>
                      <td className="py-4 px-4 align-top">
                        <Badge variant="bracket">{project.category}</Badge>
                      </td>
                      <td className="py-4 px-4 align-top text-[#4A3C34]">
                        <div className="flex flex-wrap gap-1 text-[11px]">
                          {project.technologies.map((t, idx) => (
                            <span key={t}>
                              {t}
                              {idx < project.technologies.length - 1 ? ' /' : ''}
                            </span>
                          ))}
                        </div>
                      </td>
                      <td className="py-4 px-4 align-top text-right whitespace-nowrap">
                        <div className="flex items-center justify-end gap-2">
                          {isLinkValid(project.github) && (
                            <a
                              href={project.github}
                              target="_blank"
                              rel="noopener noreferrer"
                              className="p-1.5 text-[#75665B] hover:text-[#211914] transition-colors"
                              aria-label={`GitHub repo for ${project.title}`}
                            >
                              <GithubIcon className="w-4 h-4 fill-current" />
                            </a>
                          )}
                          <Link
                            href={`/projects/${project.slug}`}
                            className="inline-flex items-center gap-1 px-2.5 py-1 bg-[#211914] text-[#FCF9F2] rounded-[2px] hover:bg-[#8A3F32] transition-colors text-[11px] font-semibold"
                          >
                            <span>DOSSIER</span>
                            <ArrowUpRight className="w-3 h-3" />
                          </Link>
                        </div>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </section>
        )}

        {/* Section Divider (Rendered ONLY when BOTH sections exist) */}
        {hasBothSections && <EditorialRule variant="double" className="my-8 opacity-70" />}

        {/* ========================================================================= */}
        {/* SECTION 2: ACADEMIC ARCHIVE (Rendered strictly when filtered count > 0)   */}
        {/* ========================================================================= */}
        {filteredAcademic.length > 0 && (
          <section id="academic" className="space-y-6 pt-2 scroll-mt-20">
            <div className="flex items-baseline justify-between border-b-2 border-[#211914] pb-2">
              <div>
                <span className="font-mono text-[10px] text-[#8A3F32] font-semibold uppercase tracking-wider block">
                  FOUNDATIONAL & COURSEWORK SYSTEMS
                </span>
                <h2 className="font-display text-2xl sm:text-3xl text-[#211914]">
                  Academic Archive / 2020—2025
                </h2>
              </div>
              <span className="font-mono text-xs text-[#75665B]">
                {filteredAcademic.length < 10 ? `0${filteredAcademic.length}` : filteredAcademic.length} OF 12 RECORDS
              </span>
            </div>

            <p className="text-xs sm:text-sm text-[#4A3C34] font-sans max-w-3xl">
              Dense technical records representing foundational coursework in computer vision, natural language processing, database internals, operating systems, processor microarchitecture, and full-stack software. Click any row to expand technical highlights and implementation details.
            </p>

            {/* Year Groupings */}
            <div className="space-y-8">
              {academicYears.map((year) => {
                const yearProjects = filteredAcademic.filter((p) => p.year === year);
                if (yearProjects.length === 0) return null;

                return (
                  <div key={year} className="space-y-3">
                    {/* Year Header Line */}
                    <div className="flex items-center gap-3 font-mono text-xs text-[#75665B]">
                      <span className="font-semibold text-[#211914] bg-[#E7DECF] px-2 py-0.5 rounded-[2px]">
                        {year}
                      </span>
                      <div className="flex-1 h-px bg-[#C9BBA6]" />
                      <span>0{yearProjects.length} {yearProjects.length === 1 ? 'ENTRY' : 'ENTRIES'}</span>
                    </div>

                    {/* List of Expandable Records */}
                    <div className="bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] shadow-paper divide-y divide-[#C9BBA6]/60 overflow-hidden">
                      {yearProjects.map((project: AcademicProject) => {
                        const isExpanded = expandedAcademicSlug === project.slug;

                        return (
                          <div key={project.slug} className="transition-colors">
                            {/* Row Header / Click Target */}
                            <button
                              onClick={() => toggleAcademicExpand(project.slug)}
                              aria-expanded={isExpanded}
                              aria-controls={`academic-details-${project.slug}`}
                              className="w-full p-4 sm:p-5 flex flex-col md:flex-row md:items-center justify-between gap-3 text-left hover:bg-[#E7DECF]/30 transition-colors cursor-pointer select-none"
                            >
                              <div className="space-y-1">
                                <div className="flex items-center gap-2 flex-wrap font-mono text-[11px] text-[#75665B]">
                                  <span className="text-[#8A3F32] font-semibold">{project.period}</span>
                                  {project.institution && (
                                    <>
                                      <span>·</span>
                                      <span>{project.institution}</span>
                                    </>
                                  )}
                                  <span>·</span>
                                  <Badge variant="bracket" size="sm">{project.category}</Badge>
                                </div>
                                <h3 className="font-display text-xl sm:text-2xl text-[#211914]">
                                  {project.title}
                                </h3>
                              </div>

                              <div className="flex items-center justify-between md:justify-end gap-3 font-mono text-xs text-[#75665B]">
                                <div className="hidden lg:block max-w-xs truncate text-[11px]">
                                  {project.technologies.slice(0, 3).join(' / ')}
                                </div>
                                <div className="flex items-center gap-1.5 text-[#211914] font-semibold text-[11px]">
                                  <span>{isExpanded ? 'COLLAPSE' : 'DETAILS'}</span>
                                  {isExpanded ? (
                                    <ChevronUp className="w-3.5 h-3.5 text-[#8A3F32]" />
                                  ) : (
                                    <ChevronDown className="w-3.5 h-3.5 text-[#75665B]" />
                                  )}
                                </div>
                              </div>
                            </button>

                            {/* Expanded In-Place Fold */}
                            {isExpanded && (
                              <div
                                id={`academic-details-${project.slug}`}
                                className="px-4 sm:px-6 pb-6 pt-2 bg-[#E7DECF]/20 border-t border-[#C9BBA6]/40 space-y-4 animate-in slide-in-from-top-2 duration-150"
                              >
                                <p className="text-xs sm:text-sm text-[#4A3C34] font-sans leading-relaxed">
                                  {project.summary}
                                </p>

                                {/* Technical Highlights */}
                                <div className="space-y-1.5">
                                  <div className="font-mono text-[10px] uppercase tracking-wider text-[#75665B]">
                                    Key Engineering Details & Highlights:
                                  </div>
                                  <ul className="space-y-1 text-xs text-[#211914] font-sans">
                                    {project.highlights.map((h, idx) => (
                                      <li key={idx} className="flex items-start gap-2">
                                        <span className="text-[#8A3F32] font-mono text-[10px] mt-0.5">▸</span>
                                        <span>{h}</span>
                                      </li>
                                    ))}
                                  </ul>
                                </div>

                                {/* Technologies & Source Button */}
                                <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-3 pt-3 border-t border-[#C9BBA6]/40 font-mono text-xs">
                                  <div className="flex flex-wrap items-center gap-1.5 text-[11px] text-[#4A3C34]">
                                    <span className="text-[#75665B] uppercase mr-1">Stack:</span>
                                    {project.technologies.map((t) => (
                                      <span key={t} className="bg-[#E7DECF] px-1.5 py-0.5 rounded-[2px] text-[#211914]">
                                        {t}
                                      </span>
                                    ))}
                                  </div>

                                  {isLinkValid(project.github) && (
                                    <a
                                      href={project.github}
                                      target="_blank"
                                      rel="noopener noreferrer"
                                      className="inline-flex items-center gap-1.5 px-3 py-1.5 bg-[#211914] text-[#FCF9F2] hover:bg-[#8A3F32] rounded-[2px] transition-colors self-start sm:self-auto text-xs font-semibold"
                                    >
                                      <GithubIcon className="w-3.5 h-3.5 fill-current" />
                                      <span>VIEW REPOSITORY ↗</span>
                                    </a>
                                  )}
                                </div>
                              </div>
                            )}
                          </div>
                        );
                      })}
                    </div>
                  </div>
                );
              })}
            </div>
          </section>
        )}

        {/* Empty State (When zero records exist in both flagship and academic for a filter) */}
        {hasNoMatches && (
          <div className="bg-[#FCF9F2] border border-[#C9BBA6] p-8 text-center rounded-[2px] shadow-paper font-mono text-xs text-[#75665B] space-y-2 my-8">
            <span className="text-[#8A3F32] font-semibold block uppercase tracking-wider">
              CATALOG NOTICE
            </span>
            <p className="font-display text-2xl text-[#211914]">
              No records cataloged under {activeCategory}
            </p>
            <p className="text-xs text-[#4A3C34] max-w-md mx-auto">
              Please select another category filter above or reset to view the entire engineering catalog.
            </p>
            <button
              onClick={() => setActiveCategory('ALL')}
              className="mt-3 inline-flex items-center gap-1 px-3 py-1.5 bg-[#211914] text-[#FCF9F2] rounded-[2px] hover:bg-[#8A3F32] transition-colors font-semibold cursor-pointer"
            >
              RESET TO COMPLETE CATALOG (ALL) →
            </button>
          </div>
        )}

        <EditorialRule variant="single" className="my-6 opacity-60" />
      </div>
    </div>
  );
}
