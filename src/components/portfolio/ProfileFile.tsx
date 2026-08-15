import React from 'react';
import { principlesData } from '@/data/principles';
import { contactData } from '@/data/contact';
import { Mail, Phone } from 'lucide-react';
import { LinkedinIcon, GithubIcon } from '@/components/ui/Icons';

export function ProfileFile() {
  const focusAreas = [
    {
      title: 'Applied AI & Agentic Systems',
      description: 'Bounded LLM workflows, multimodal evidence synthesis, and deterministic execution boundaries.',
    },
    {
      title: 'Backend & Distributed Systems',
      description: 'High-concurrency REST/WebSocket APIs, relational data modeling, Redis caching, and containerized microservices.',
    },
    {
      title: 'Machine Learning & Vision',
      description: 'Transformer NLP classification, deep metric learning with margin losses, real-time pose estimation, and reinforcement learning schedulers.',
    },
    {
      title: 'Product & Frontend Craft',
      description: 'Translating complex backend and AI pipelines into responsive, type-safe interfaces with zero layout shift and robust testing.',
    },
  ];

  return (
    <div className="space-y-8">
      <div className="border-b border-[#C9BBA6] pb-3">
        <div className="font-mono text-xs text-[#75665B] uppercase tracking-wider">
          FILE 05 / ARCHIVAL PROFILE & TENETS
        </div>
        <h3 className="font-display text-3xl text-[#211914] font-normal mt-1">
          Biography, Focus Areas & Engineering Principles
        </h3>
      </div>

      {/* Section 1: Biography */}
      <div className="space-y-3">
        <div className="font-mono text-xs text-[#8A3F32] font-semibold uppercase tracking-wider">
          01 / About Harshvardhan
        </div>
        <div className="space-y-3 text-sm text-[#4A3C34] font-sans leading-relaxed max-w-3xl">
          <p>
            I am an AI Engineer and Software Engineer with an M.S. in Computer Science from the Rochester Institute of Technology and a B.S. in Computer Science from Penn State University. My work bridges empirical machine-learning research with production backend architectures.
          </p>
          <p>
            I focus on building systems that don&apos;t just demo well, but operate deterministically in high-throughput production: designing validation guardrails for LLM agents, establishing regression evaluation harnesses, and architecting resilient asynchronous services.
          </p>
        </div>
      </div>

      {/* Section 2: Direct Contact Field */}
      <div className="bg-[#E7DECF]/60 border border-[#C9BBA6] p-4 rounded-[2px] space-y-2">
        <div className="font-mono text-[10px] text-[#8A3F32] font-semibold uppercase tracking-wider">
          DIRECT CORRESPONDENCE
        </div>
        <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-4 gap-3 font-mono text-xs text-[#211914]">
          <a
            href={contactData.emailHref}
            className="flex items-center gap-1.5 hover:text-[#8A3F32] transition-colors"
          >
            <Mail className="w-3.5 h-3.5 text-[#75665B]" />
            <span>EMAIL ↗</span>
          </a>
          <a
            href={contactData.phoneHref}
            className="flex items-center gap-1.5 hover:text-[#8A3F32] transition-colors"
          >
            <Phone className="w-3.5 h-3.5 text-[#75665B]" />
            <span>{contactData.phone}</span>
          </a>
          <a
            href={contactData.linkedin}
            target="_blank"
            rel="noopener noreferrer"
            className="flex items-center gap-1.5 hover:text-[#8A3F32] transition-colors"
          >
            <LinkedinIcon className="w-3.5 h-3.5 fill-current text-[#75665B]" />
            <span>LINKEDIN ↗</span>
          </a>
          <a
            href={contactData.github}
            target="_blank"
            rel="noopener noreferrer"
            className="flex items-center gap-1.5 hover:text-[#8A3F32] transition-colors"
          >
            <GithubIcon className="w-3.5 h-3.5 fill-current text-[#75665B]" />
            <span>GITHUB ↗</span>
          </a>
        </div>
      </div>

      {/* Section 3: Core Engineering Disciplines */}
      <div className="space-y-3 border-t border-[#C9BBA6]/60 pt-6">
        <div className="font-mono text-xs text-[#8A3F32] font-semibold uppercase tracking-wider">
          02 / Current Engineering Focus
        </div>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 pt-1">
          {focusAreas.map((f, i) => (
            <div
              key={f.title}
              className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] space-y-1.5"
            >
              <div className="font-mono text-xs font-semibold text-[#211914]">
                [0{i + 1}] {f.title}
              </div>
              <p className="text-xs text-[#4A3C34] leading-relaxed font-sans">
                {f.description}
              </p>
            </div>
          ))}
        </div>
      </div>

      {/* Section 4: Engineering Principles */}
      <div className="space-y-3 border-t border-[#C9BBA6]/60 pt-6">
        <div className="font-mono text-xs text-[#8A3F32] font-semibold uppercase tracking-wider">
          03 / Core Engineering Tenets
        </div>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 pt-1">
          {principlesData.map((p) => (
            <div
              key={p.number}
              className="bg-[#FCF9F2] border border-[#C9BBA6] p-4 rounded-[2px] shadow-xs space-y-1.5"
            >
              <div className="font-display text-lg text-[#211914] font-normal">
                {p.title}
              </div>
              <div className="font-mono text-xs text-[#8A3F32] font-semibold">
                &ldquo;{p.statement}&rdquo;
              </div>
              <p className="text-xs text-[#4A3C34] leading-relaxed font-sans">
                {p.elaboration}
              </p>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
