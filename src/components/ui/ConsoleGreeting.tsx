'use client';

import { useEffect } from 'react';

export function ConsoleGreeting() {
  useEffect(() => {
    console.log(
      '%cHarshvardhan Singh — AI Engineer · Software Engineer\n%cDesigned & engineered with Next.js, TypeScript, Tailwind CSS & custom Canvas topologies.\nThanks for inspecting under the hood!',
      'color: #7C8CFF; font-weight: bold; font-size: 13px; font-family: monospace;',
      'color: #A1A7B3; font-size: 11px; font-family: monospace;'
    );
  }, []);

  return null;
}
