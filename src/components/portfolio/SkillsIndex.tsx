'use client';

import React, { useState } from 'react';
import { skillsData } from '@/data/skills';
import { ChevronLeft, ChevronRight } from 'lucide-react';

export function SkillsIndex() {
  const [activeCategoryIdx, setActiveCategoryIdx] = useState(0);
  const currentCategory = skillsData[activeCategoryIdx] || skillsData[0];

  const handleNext = () => {
    setActiveCategoryIdx((prev) => (prev + 1) % skillsData.length);
  };

  const handlePrev = () => {
    setActiveCategoryIdx((prev) => (prev - 1 + skillsData.length) % skillsData.length);
  };

  return (
    <div className="space-y-6">
      <div className="border-b border-[#C9BBA6] pb-3">
        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
          FILE 03 / CAPABILITY INVENTORY
        </div>
        <h3 className="font-display text-3xl text-[#211914] font-normal mt-1">
          Technical Capabilities & Systems Craft
        </h3>
        <p className="text-xs text-[#75665B] font-mono mt-1">
          Curated engineering index across AI architectures, distributed backends, testing, and production tooling.
        </p>
      </div>

      {/* Mobile-Friendly Category Switcher (0 - 767px) */}
      <div className="md:hidden space-y-4">
        <div className="flex items-center justify-between bg-[#E7DECF]/60 border border-[#C9BBA6] p-2.5 rounded-[2px] font-mono text-xs">
          <button
            onClick={handlePrev}
            aria-label="Previous skill category"
            className="p-1 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] text-[#211914] active:bg-[#C9BBA6]"
          >
            <ChevronLeft className="w-4 h-4" />
          </button>

          <div className="text-center">
            <span className="text-[10px] text-[#8A3F32] font-semibold block uppercase">
              CATEGORY 0{activeCategoryIdx + 1} / 0{skillsData.length}
            </span>
            <span className="font-semibold text-[#211914] text-xs uppercase">
              {currentCategory.title}
            </span>
          </div>

          <button
            onClick={handleNext}
            aria-label="Next skill category"
            className="p-1 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] text-[#211914] active:bg-[#C9BBA6]"
          >
            <ChevronRight className="w-4 h-4" />
          </button>
        </div>

        <div className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-3">
          <p className="text-xs text-[#4A3C34] leading-tight font-sans">
            {currentCategory.description}
          </p>
          <div className="grid grid-cols-2 gap-2 font-mono text-xs text-[#211914]">
            {currentCategory.skills.map((skill) => (
              <div key={skill} className="flex items-center gap-1.5 py-0.5">
                <span className="text-[#8A3F32] text-[10px]">·</span>
                <span className="truncate">{skill}</span>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Desktop 3-Column Newspaper Grid (>= 768px) */}
      <div className="hidden md:grid md:grid-cols-2 lg:grid-cols-3 gap-5 pt-2">
        {skillsData.map((group) => (
          <div
            key={group.id}
            className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-2.5 shadow-xs"
          >
            <div className="flex items-baseline justify-between border-b border-[#C9BBA6]/60 pb-1.5 font-mono">
              <h4 className="text-xs font-semibold uppercase text-[#211914]">
                {group.title}
              </h4>
              <span className="text-[10px] text-[#75665B]">{group.skills.length} tools</span>
            </div>

            <p className="text-[11px] text-[#4A3C34] leading-tight font-sans">
              {group.description}
            </p>

            {/* List of Skills as Monospace Rows */}
            <div className="font-mono text-xs text-[#211914] space-y-1 pt-1">
              {group.skills.map((skill) => (
                <div key={skill} className="flex items-center gap-1.5 py-0.5">
                  <span className="text-[#8A3F32] text-[10px]">·</span>
                  <span>{skill}</span>
                </div>
              ))}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
