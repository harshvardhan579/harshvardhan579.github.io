'use client';

import React from 'react';
import { contactData, isLinkValid } from '@/data/contact';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import Link from 'next/link';

export function Footer() {
  const currentYear = 2026;

  return (
    <footer className="w-full bg-[#EEE8DC] border-t border-[#C9BBA6]/80 py-10 px-4 sm:px-6 lg:px-8 text-[#75665B] font-mono text-xs">
      <div className="max-w-[1280px] mx-auto space-y-6">
        <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
          <div>
            <div className="font-semibold text-[#211914] text-sm uppercase">
              {contactData.name}
            </div>
            <div className="text-[11px] text-[#75665B]">
              {contactData.role} — Folio Edition {currentYear}
            </div>
          </div>

          <div className="flex flex-wrap items-center gap-4 text-xs">
            <Link
              href="/#index"
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              INDEX
            </Link>
            <Link
              href="/projects"
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              ARCHIVE
            </Link>
            <a
              href={contactData.emailHref}
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              EMAIL
            </a>
            <a
              href={contactData.phoneHref}
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              TEL
            </a>
            <a
              href={contactData.linkedin}
              target="_blank"
              rel="noopener noreferrer"
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              LINKEDIN
            </a>
            <a
              href={contactData.github}
              target="_blank"
              rel="noopener noreferrer"
              className="text-[#211914] hover:text-[#8A3F32] transition-colors"
            >
              GITHUB
            </a>
            {isLinkValid(contactData.resumeUrl) && (
              <a
                href={contactData.resumeUrl}
                target="_blank"
                rel="noopener noreferrer"
                className="text-[#8A3F32] font-semibold hover:text-[#211914] transition-colors"
              >
                RÉSUMÉ
              </a>
            )}
          </div>
        </div>

        <EditorialRule variant="single" className="opacity-40" />

        <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-2 text-[10px] text-[#75665B]">
          <div>
            TYPESET IN NEWSREADER & IBM PLEX MONO / SANS
          </div>
          <div>
            EST. 2026 — ALL TECHNICAL CONTENT VERIFIED AGAINST SOURCE CODE
          </div>
        </div>
      </div>
    </footer>
  );
}
