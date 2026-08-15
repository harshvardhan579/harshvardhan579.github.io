import { ExperienceItem } from '@/types/portfolio';

export const experienceData: ExperienceItem[] = [
  {
    role: 'Gen AI & ML Engineering Intern',
    company: 'Big Vision',
    location: 'Remote – San Diego, CA',
    period: 'May 2023 — Jul 2023',
    type: 'Internship',
    summary:
      'Full-stack engineer for a GPT-powered document intelligence platform architected toward 10,000+ uploads/day linking PaddleOCR extraction to structured GPT document querying.',
    bullets: [
      'Full-stack engineer for a GPT-powered document platform architected toward 10,000+ uploads/day: built the React/TypeScript frontend and FastAPI backend linking PaddleOCR extraction to GPT document querying.',
      'Detection degraded on real scans varying in resolution, orientation, and layout; tuned PaddleOCR hyperparameters and reworked preprocessing to recover quality before documents reached the GPT step.',
      'Environment drift kept breaking deploys; standardized services under Docker Compose so a full stack came up in one command, and added Pytest/Jest coverage over the upload and extraction paths.',
    ],
    technologies: [
      'React',
      'TypeScript',
      'FastAPI',
      'Python',
      'PaddleOCR',
      'GPT Integration',
      'Docker Compose',
      'Pytest',
      'Jest',
    ],
    verifiedMetrics: [
      { label: 'Target Platform Capacity', value: '10,000+ uploads/day' },
    ],
  },
  {
    role: 'Learning Assistant',
    company: 'Penn State University',
    location: 'University Park, PA',
    period: 'Aug 2022 — May 2023',
    type: 'Academic',
    summary:
      'Supported undergraduate engineering students in systems programming, computer architecture, digital logic, and embedded design laboratories.',
    bullets: [
      'Mentored 100+ students weekly through debugging low-level C, assembly, memory management, and digital logic circuits.',
      'Developed automated grading scripts in Python/Bash to evaluate student lab submissions against unit test suites.',
      'Conducted weekly review workshops covering pointer arithmetic, concurrency primitives, and hardware description concepts.',
    ],
    technologies: ['C', 'Assembly', 'Digital Logic', 'Python', 'Bash', 'GDB'],
    verifiedMetrics: [
      { label: 'Students Mentored', value: '100+ / week' },
      { label: 'Lab Grading Automation', value: 'Python / Bash' },
    ],
  },
];
