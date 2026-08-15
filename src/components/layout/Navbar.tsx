'use client';

import React, { useState } from 'react';
import Link from 'next/link';
import { contactData, isLinkValid } from '@/data/contact';
import { FileText, Menu, X } from 'lucide-react';

export function Navbar() {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

  const navLinks = [
    { name: 'INDEX', href: '/#index' },
    { name: 'ARCHIVE', href: '/projects' },
    { name: 'CONTACT', href: '/#contact' },
  ];

  return (
    <header className="sticky top-0 z-50 bg-[#EEE8DC]/95 backdrop-blur-xs border-b border-[#C9BBA6] transition-all">
      <div className="max-w-[1280px] mx-auto px-4 sm:px-6 lg:px-8 h-14 flex items-center justify-between">
        {/* Left Monogram / Archive Title */}
        <Link
          href="/"
          className="flex items-center gap-2 font-mono text-xs tracking-widest text-[#211914] font-semibold uppercase hover:text-[#8A3F32] transition-colors focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-[#8A3F32]"
        >
          <span className="bg-[#211914] text-[#FCF9F2] px-1.5 py-0.5 rounded-[2px]">HS</span>
          <span className="text-[#75665B]">/</span>
          <span>ARCHIVE</span>
        </Link>

        {/* Desktop Navigation Links */}
        <nav className="hidden md:flex items-center gap-6 font-mono text-xs tracking-wider text-[#4A3C34]" aria-label="Main Navigation">
          {navLinks.map((link) => (
            <Link
              key={link.name}
              href={link.href}
              className="hover:text-[#211914] transition-colors relative py-1 hover:underline underline-offset-4 decoration-[#8A3F32]"
            >
              {link.name}
            </Link>
          ))}

          {isLinkValid(contactData.resumeUrl) && (
            <a
              href={contactData.resumeUrl}
              target="_blank"
              rel="noopener noreferrer"
              className="inline-flex items-center gap-1.5 px-2.5 py-1 bg-[#E7DECF] hover:bg-[#C9BBA6] text-[#211914] border border-[#C9BBA6] rounded-[2px] transition-colors"
            >
              <FileText className="w-3.5 h-3.5 text-[#75665B]" />
              <span>RÉSUMÉ</span>
            </a>
          )}
        </nav>

        {/* Mobile Menu Button */}
        <div className="md:hidden flex items-center gap-2">
          {isLinkValid(contactData.resumeUrl) && (
            <a
              href={contactData.resumeUrl}
              target="_blank"
              rel="noopener noreferrer"
              className="px-2.5 py-1 bg-[#E7DECF] text-[#211914] border border-[#C9BBA6] font-mono text-[11px] rounded-[2px]"
            >
              RÉSUMÉ
            </a>
          )}
          <button
            onClick={() => setMobileMenuOpen(!mobileMenuOpen)}
            className="p-2 text-[#211914] hover:bg-[#E7DECF] border border-[#C9BBA6] rounded-[2px] focus-visible:outline-none"
            aria-label="Toggle navigation menu"
            aria-expanded={mobileMenuOpen}
          >
            {mobileMenuOpen ? <X className="w-4 h-4" /> : <Menu className="w-4 h-4" />}
          </button>
        </div>
      </div>

      {/* Mobile Dropdown Sheet */}
      {mobileMenuOpen && (
        <div className="md:hidden bg-[#FCF9F2] border-b border-[#C9BBA6] px-4 py-4 space-y-3 font-mono text-xs animate-in slide-in-from-top-2 duration-200">
          {navLinks.map((link) => (
            <Link
              key={link.name}
              href={link.href}
              onClick={() => setMobileMenuOpen(false)}
              className="block py-2 text-[#211914] border-b border-[#C9BBA6]/40 hover:text-[#8A3F32]"
            >
              → {link.name}
            </Link>
          ))}
        </div>
      )}
    </header>
  );
}
