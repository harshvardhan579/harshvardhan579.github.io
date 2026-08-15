'use client';

import React from 'react';
import { contactData } from '@/data/contact';
import { Button } from '@/components/ui/Button';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import { Mail, Phone, FileText } from 'lucide-react';
import { GithubIcon, LinkedinIcon } from '@/components/ui/Icons';

export function EditorialContact() {
  return (
    <section id="contact" className="w-full py-12 sm:py-16 px-4 sm:px-6 lg:px-8 bg-[#EEE8DC] scroll-mt-14">
      <div className="max-w-[1280px] mx-auto">
        <EditorialRule variant="double" className="mb-8 sm:mb-10" />

        <div className="bg-[#FCF9F2] border border-[#C9BBA6] p-6 sm:p-10 md:p-12 rounded-[2px] shadow-paper space-y-6">
          <div className="flex flex-col lg:flex-row lg:items-end justify-between gap-6">
            <div className="space-y-3 max-w-2xl">
              <div className="font-mono text-xs text-[#8A3F32] font-semibold tracking-widest uppercase">
                [ DISPATCH & CORRESPONDENCE / 2026 ]
              </div>
              <h2 className="font-display text-3xl sm:text-4xl md:text-5xl text-[#211914] font-normal tracking-tight">
                Have something worth building?
              </h2>
              <p className="text-sm sm:text-base text-[#4A3C34] font-sans leading-relaxed">
                I am actively considering AI engineering and backend software roles where I can architect robust agent workflows, rigorous evaluation platforms, and reliable production systems.
              </p>
            </div>

            {/* Direct Communication Action Controls */}
            <div className="flex flex-wrap items-center gap-2.5 sm:gap-3">
              <Button
                href={contactData.emailHref}
                variant="primary"
                size="md"
                icon={<Mail className="w-4 h-4" />}
              >
                Send Dispatch ↗
              </Button>

              <Button
                href={contactData.phoneHref}
                variant="secondary"
                size="md"
                icon={<Phone className="w-4 h-4 text-[#75665B]" />}
              >
                {contactData.phone}
              </Button>

              <Button
                href={contactData.resumeUrl}
                isExternal
                variant="secondary"
                size="md"
                icon={<FileText className="w-4 h-4 text-[#75665B]" />}
              >
                Résumé ↗
              </Button>

              <Button
                href={contactData.linkedin}
                isExternal
                variant="outline"
                size="md"
                icon={<LinkedinIcon className="w-4 h-4 fill-current" />}
              >
                LinkedIn ↗
              </Button>

              <Button
                href={contactData.github}
                isExternal
                variant="outline"
                size="md"
                icon={<GithubIcon className="w-4 h-4 fill-current" />}
              >
                GitHub ↗
              </Button>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
