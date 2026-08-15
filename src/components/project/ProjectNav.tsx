import React from 'react';
import Link from 'next/link';
import { ArrowLeft, ArrowRight } from 'lucide-react';

export function ProjectNav({
  prevProject,
  nextProject,
}: {
  prevProject?: { slug: string; title: string } | null;
  nextProject?: { slug: string; title: string } | null;
}) {
  return (
    <div className="flex flex-col sm:flex-row items-center justify-between gap-4 font-mono text-xs w-full">
      {prevProject ? (
        <Link
          href={`/projects/${prevProject.slug}`}
          className="group flex items-center gap-2 text-left p-3 rounded-[2px] border border-[#BEB09C] hover:border-[#302019] bg-[#FAF6ED] hover:bg-[#E8DDCA] transition-all w-full sm:w-auto"
        >
          <ArrowLeft className="w-4 h-4 text-[#8C4432] group-hover:-translate-x-0.5 transition-transform shrink-0" />
          <div>
            <span className="text-[10px] text-[#857267] uppercase block">
              PREVIOUS DOSSIER
            </span>
            <span className="font-serif text-sm text-[#1D1511]">
              {prevProject.title}
            </span>
          </div>
        </Link>
      ) : (
        <div className="hidden sm:block" />
      )}

      <Link
        href="/#index"
        className="px-3 py-1.5 bg-[#E8DDCA] hover:bg-[#DFD3BF] text-[#1D1511] border border-[#BEB09C] rounded-[2px] transition-colors uppercase tracking-wider"
      >
        PORTFOLIO INDEX
      </Link>

      {nextProject ? (
        <Link
          href={`/projects/${nextProject.slug}`}
          className="group flex items-center justify-end gap-2 text-right p-3 rounded-[2px] border border-[#BEB09C] hover:border-[#302019] bg-[#FAF6ED] hover:bg-[#E8DDCA] transition-all w-full sm:w-auto"
        >
          <div>
            <span className="text-[10px] text-[#857267] uppercase block">
              NEXT DOSSIER
            </span>
            <span className="font-serif text-sm text-[#1D1511]">
              {nextProject.title}
            </span>
          </div>
          <ArrowRight className="w-4 h-4 text-[#8C4432] group-hover:translate-x-0.5 transition-transform shrink-0" />
        </Link>
      ) : (
        <div className="hidden sm:block" />
      )}
    </div>
  );
}
