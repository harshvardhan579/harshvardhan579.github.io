import React from 'react';
import { experienceData } from '@/data/experience';
import { EditorialRule } from '@/components/editorial/EditorialRule';

export function ExperienceFile() {
  return (
    <div className="space-y-8">
      <div className="border-b border-[#C9BBA6] pb-3">
        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
          FILE 02 / PROFESSIONAL JOURNEY
        </div>
        <h3 className="font-display text-3xl text-[#211914] font-normal mt-1">
          Engineering Experience & Verified Impact
        </h3>
      </div>

      <div className="space-y-8">
        {experienceData.map((exp, idx) => (
          <div key={`${exp.company}-${exp.period}`} className="space-y-4">
            {/* Entry Header */}
            <div className="flex flex-col sm:flex-row sm:items-baseline justify-between gap-1 border-b border-[#C9BBA6]/60 pb-2">
              <div>
                <h4 className="font-display text-2xl text-[#211914] font-normal">{exp.role}</h4>
                <div className="font-mono text-xs text-[#8A3F32] font-semibold">
                  {exp.company} <span className="text-[#75665B]">· {exp.type}</span>
                </div>
              </div>
              <div className="font-mono text-xs text-[#75665B]">
                {exp.period} — {exp.location}
              </div>
            </div>

            <p className="text-sm text-[#4A3C34] leading-relaxed font-sans">
              {exp.summary}
            </p>

            {/* Verified Metrics in Editorial Pull-Quote Format */}
            {exp.verifiedMetrics && exp.verifiedMetrics.length > 0 && (
              <div className="flex flex-wrap gap-4 bg-[#E7DECF]/50 p-3 rounded-[2px] border border-[#C9BBA6]/60 my-3">
                {exp.verifiedMetrics.map((m, mIdx) => (
                  <div key={mIdx} className="space-y-0.5 min-w-[160px]">
                    <div className="font-display text-xl sm:text-2xl text-[#211914] font-medium leading-none">
                      {m.value}
                    </div>
                    <div className="font-mono text-[10px] text-[#75665B] uppercase tracking-wider">
                      {m.label}
                    </div>
                  </div>
                ))}
              </div>
            )}

            {/* Key Contributions */}
            <div className="space-y-1.5 pt-1">
              <div className="font-mono text-[10px] uppercase tracking-wider text-[#75665B]">
                Key Engineering Deliverables:
              </div>
              <ul className="space-y-1 text-xs text-[#211914] font-sans">
                {exp.bullets.map((b, bIdx) => (
                  <li key={bIdx} className="flex items-start gap-2">
                    <span className="text-[#8A3F32] font-mono text-[10px] mt-0.5">▸</span>
                    <span>{b}</span>
                  </li>
                ))}
              </ul>
            </div>

            {/* Technologies */}
            <div className="font-mono text-xs text-[#4A3C34] pt-2 flex flex-wrap gap-x-2">
              <span className="text-[#75665B] text-[10px] uppercase mr-1">Stack:</span>
              {exp.technologies.map((t, tIdx) => (
                <span key={t} className="inline-flex items-center">
                  <span>{t}</span>
                  {tIdx < exp.technologies.length - 1 && (
                    <span className="text-[#C9BBA6] ml-2">/</span>
                  )}
                </span>
              ))}
            </div>

            {idx < experienceData.length - 1 && <EditorialRule variant="dashed" className="my-6" />}
          </div>
        ))}
      </div>
    </div>
  );
}
