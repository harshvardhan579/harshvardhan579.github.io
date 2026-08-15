import type { Metadata } from 'next';
import { Newsreader, IBM_Plex_Mono, IBM_Plex_Sans } from 'next/font/google';
import './globals.css';
import { Navbar } from '@/components/layout/Navbar';
import { Footer } from '@/components/layout/Footer';
import { ConsoleGreeting } from '@/components/ui/ConsoleGreeting';

const newsreader = Newsreader({
  variable: '--font-newsreader',
  subsets: ['latin'],
  style: ['normal', 'italic'],
  display: 'swap',
});

const ibmPlexMono = IBM_Plex_Mono({
  variable: '--font-mono',
  weight: ['400', '500', '600'],
  subsets: ['latin'],
  display: 'swap',
});

const ibmPlexSans = IBM_Plex_Sans({
  variable: '--font-sans',
  weight: ['400', '500', '600'],
  subsets: ['latin'],
  display: 'swap',
});

export const metadata: Metadata = {
  title: 'Harshvardhan Singh — AI Engineer · Software Engineer',
  description:
    'The official engineering archive and technical folio of Harshvardhan Singh. Applied AI, agentic systems, LLM evaluation platforms, and distributed systems.',
  keywords: [
    'Harshvardhan Singh',
    'AI Engineer',
    'Software Engineer',
    'APILoom',
    'CivicPulse',
    'Pocket Arcade',
    'NewsVerify',
    'AI Form Evaluator',
    'Hybrid ML Scheduler',
    'FastAPI',
    'PostgreSQL',
    'Docker',
    'TypeScript',
  ],
  authors: [{ name: 'Harshvardhan Singh' }],
  creator: 'Harshvardhan Singh',
  metadataBase: new URL('https://harshvardhan579.github.io'),
  openGraph: {
    title: 'Harshvardhan Singh — AI Engineer · Software Engineer',
    description:
      'The official engineering archive and technical folio of Harshvardhan Singh. Applied AI, agentic systems, LLM evaluation platforms, and distributed systems.',
    type: 'website',
    locale: 'en_US',
    siteName: 'Harshvardhan Singh Archive',
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Harshvardhan Singh — AI Engineer · Software Engineer',
    description:
      'The official engineering archive and technical folio of Harshvardhan Singh. Applied AI, agentic systems, LLM evaluation platforms, and distributed systems.',
  },
  robots: {
    index: true,
    follow: true,
  },
  icons: {
    icon: '/icon.svg',
    shortcut: '/icon.svg',
    apple: '/icon.svg',
  },
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const jsonLd = {
    '@context': 'https://schema.org',
    '@type': 'Person',
    name: 'Harshvardhan Singh',
    jobTitle: 'AI Engineer · Software Engineer',
    alumniOf: [
      {
        '@type': 'EducationalOrganization',
        name: 'Rochester Institute of Technology',
      },
      {
        '@type': 'EducationalOrganization',
        name: 'Penn State University',
      },
    ],
    knowsAbout: [
      'Artificial Intelligence',
      'Large Language Models',
      'Distributed Systems',
      'APILoom',
      'Machine Learning',
      'Backend Engineering',
      'Software Architecture',
    ],
  };

  return (
    <html
      lang="en"
      className={`${newsreader.variable} ${ibmPlexMono.variable} ${ibmPlexSans.variable} scroll-smooth`}
    >
      <head>
        <script
          type="application/ld+json"
          dangerouslySetInnerHTML={{ __html: JSON.stringify(jsonLd) }}
        />
      </head>
      <body className="min-h-screen bg-[#EEE8DC] text-[#211914] font-sans antialiased selection:bg-[#8A3F32]/20 selection:text-[#211914] flex flex-col bg-canvas-grain">
        {/* Skip to main content for accessibility */}
        <a
          href="#main-content"
          className="sr-only focus:not-sr-only focus:fixed focus:top-4 focus:left-4 focus:z-50 focus:px-4 focus:py-2 focus:bg-[#211914] focus:text-[#FCF9F2] focus:font-mono focus:text-xs focus:rounded-sm shadow-md"
        >
          Skip to main content
        </a>

        <ConsoleGreeting />
        <Navbar />
        <div id="main-content" className="flex-1 flex flex-col">{children}</div>
        <Footer />
      </body>
    </html>
  );
}
