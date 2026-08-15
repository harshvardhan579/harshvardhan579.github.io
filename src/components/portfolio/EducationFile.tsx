import React from 'react';
import { educationData } from '@/data/education';

export function EducationFile() {
  return (
    <div className="space-y-6">
      <div className="border-b border-[#C9BBA6] pb-3">
        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
          FILE 04 / ACADEMIC FOUNDATION
        </div>
        <h3 className="font-display text-3xl text-[#211914] font-normal mt-1">
          Academic Credentials & Research Focus
        </h3>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 pt-2">
        {educationData.map((edu) => (
          <div
            key={edu.institution}
            className="bg-[#FCF9F2] border border-[#C9BBA6] p-5 sm:p-6 rounded-[2px] shadow-paper space-y-4"
          >
            <div className="flex items-center justify-between border-b border-[#C9BBA6]/60 pb-2 font-mono text-xs text-[#75665B]">
              <span>DEGREE RECORD</span>
              <span>{edu.period}</span>
            </div>

            <div>
              <h4 className="font-display text-2xl text-[#211914] font-normal leading-snug">{edu.institution}</h4>
              <div className="font-mono text-xs text-[#8A3F32] font-semibold mt-1">
                {edu.degree} in {edu.major}
              </div>
              <div className="font-mono text-xs text-[#75665B] mt-0.5">{edu.location}</div>
            </div>

            {edu.focus && (
              <p className="text-xs text-[#4A3C34] leading-relaxed font-sans border-t border-[#C9BBA6]/40 pt-3">
                {edu.focus}
              </p>
            )}

            {edu.coursework && edu.coursework.length > 0 && (
              <div className="space-y-1.5 pt-2">
                <div className="font-mono text-[10px] text-[#75665B] uppercase tracking-wider">
                  Core Coursework:
                </div>
                <div className="font-mono text-xs text-[#211914] flex flex-wrap gap-x-2 gap-y-1">
                  {edu.coursework.map((c, cIdx) => (
                    <span key={c} className="inline-flex items-center">
                      <span>{c}</span>
                      {cIdx < (edu.coursework?.length || 0) - 1 && (
                        <span className="text-[#C9BBA6] ml-2">/</span>
                      )}
                    </span>
                  ))}
                </div>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}
