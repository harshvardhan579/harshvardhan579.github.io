# Harshvardhan Singh — Retro-Future Editorial Engineering Archive

The official personal portfolio and engineering case-study dossier platform of **Harshvardhan Singh** (**AI Engineer · Software Engineer**).

Designed in the **Retro-Future Editorial** aesthetic: a synthesis of 1980s technical engineering journals, archival dossiers, and print typesetting paired with 2026 spring physics, in-place archival card transitions, and strict zero-cliché minimalism.

---

## 🎨 Visual Identity: Paper & Ink

- **Primary Paper Base**: `#F3EBDD` (Warm off-white with subtle print grain)
- **Secondary Paper / Active Sheets**: `#E8DDCA` & `#FAF6ED`
- **Primary Ink**: `#1D1511` (Very dark warm brown)
- **Rules & Separators**: `#BEB09C` (Printed double and single rules)
- **Accents**: Restrained `#8C4432` (Aged rust) & `#96754E` (Aged brass)
- **Typography**: [Newsreader](https://fonts.google.com/specimen/Newsreader) (Editorial Serif) + [IBM Plex Mono](https://fonts.google.com/specimen/IBM+Plex+Mono) (Technical Typewriter Monospace) + [Inter](https://fonts.google.com/specimen/Inter)

---

## 🏛️ Website Architecture: Compact 3-Screen Flow

The homepage drastically reduces vertical scroll down to **~2.3 viewports** by introducing an in-place **Portfolio Desk**:

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│ SCREEN 1: HERO MASTHEAD                                                     │
│ THE HARSHVARDHAN SINGH ARCHIVE · VOL. 01 · 2026                             │
│ Large Serif Headline · Value Statement · Layered Archival Metadata Card    │
├─────────────────────────────────────────────────────────────────────────────┤
│ SCREEN 2: PORTFOLIO DESK (Interactive Archival Tab Folio)                   │
│ ┌───────────────┬─────────────────────────────────────────────────────────┐ │
│ │ 01 WORK       │ Flagship Project Deck (APILoom, CivicPulse, etc.)       │ │
│ │ 02 EXPERIENCE │ Big Vision (10K+ uploads/day) & Penn State              │ │
│ │ 03 SKILLS     │ 3-Column Typographical Capability Inventory             │ │
│ │ 04 EDUCATION  │ RIT (MS CS) & Penn State (BS CS) Academic Records       │ │
│ │ 05 PROFILE    │ Biography, 4 Focus Disciplines & 4 Core Tenets          │ │
│ └───────────────┴─────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────────────────┤
│ SCREEN 3: COMPACT DISPATCH / CONTACT CLOSING                                │
│ Direct correspondence actions (Email, Résumé PDF, LinkedIn, GitHub)         │
└─────────────────────────────────────────────────────────────────────────────┘
```

- **Dedicated Archive Route (`/projects`)**: Searchable full table archive of all 8 production systems and experiments with category filtering.
- **Deep-Dive Case Study Dossiers (`/projects/[slug]`)**: Tabbed dossier interface (`OVERVIEW`, `ARCHITECTURE`, `AI / ML`, `DECISIONS`, `CHALLENGES`, `RESULTS`) with printable monochrome schematics.

---

## ⚡ Tech Stack

- **Framework**: [Next.js 16 (App Router)](https://nextjs.org/) + React 19
- **Language**: [TypeScript (Strict Mode)](https://www.typescriptlang.org/)
- **Styling**: [Tailwind CSS v4](https://tailwindcss.com/) + CSS Custom Properties Design Tokens
- **Motion & 3D Physics**: [Framer Motion](https://www.framer.com/motion/) (with `prefers-reduced-motion` compliance)
- **Icons**: Restrained [Lucide React](https://lucide.dev/) + Custom SVG brand marks
- **Testing**: [Vitest](https://vitest.dev/) (Unit) + [Playwright](https://playwright.dev/) (E2E & Visual QA)

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
npm install
```

### 2. Run Development Server
```bash
npm run dev
```
Open [http://localhost:3000](http://localhost:3000) in your browser.

### 3. Run Production Build
```bash
npm run build
npm run start
```

---

## 🧪 Testing & Quality Assurance

```bash
# Run Vitest unit tests
npm run test

# Run ESLint check
npm run lint

# Run TypeScript typecheck
npx tsc --noEmit

# Run Playwright E2E and visual tests (Desktop 1440x900 & Mobile 390x844)
npx playwright test
```

---

## 🔒 Zero Fabrication Safety
All external links (GitHub repos, LinkedIn, Email, Résumé PDF) are configured in `src/data/social.ts` and `src/data/projects.ts`. Any link marked with `TODO_` is automatically and cleanly omitted from the UI to prevent broken links.

---

## 📄 License
MIT © Harshvardhan Singh
