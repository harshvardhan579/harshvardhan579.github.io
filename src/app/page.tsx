import React from 'react';
import { EditorialHero } from '@/components/home/EditorialHero';
import { PortfolioDesk } from '@/components/portfolio/PortfolioDesk';
import { EditorialContact } from '@/components/home/EditorialContact';

export default function HomePage() {
  return (
    <main className="w-full flex flex-col">
      {/* Screen 1: Hero & Identity */}
      <EditorialHero />

      {/* Screen 2: Interactive Portfolio Desk / Index */}
      <PortfolioDesk />

      {/* Screen 3: Compact Editorial Contact / Dispatch */}
      <EditorialContact />
    </main>
  );
}
