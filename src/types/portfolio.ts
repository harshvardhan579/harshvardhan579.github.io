export interface ArchitectureLayer {
  name: string;
  description: string;
  components: string[];
}

export interface AIDecision {
  title: string;
  rationale: string;
  tradeOff?: string;
}

export interface SystemDesignDecision {
  title: string;
  rationale: string;
  tradeOff?: string;
}

export interface EngineeringChallenge {
  title: string;
  problem: string;
  solution: string;
  takeaway: string;
}

export interface ProjectMetric {
  label: string;
  value: string;
  context: string;
}

export interface CodeSnippet {
  language: string;
  filename: string;
  code: string;
  caption?: string;
}

export interface Project {
  slug: string;
  title: string;
  subtitle: string;
  category: string;
  index: string;
  year: number | string;
  featured: boolean;
  summary: string;
  problem: string;
  motivation: string;
  whatItDoes: string[];
  highlights: string[];
  systemTrace?: string[];
  architectureLayers?: ArchitectureLayer[];
  aiDecisions?: AIDecision[];
  systemDesignDecisions?: SystemDesignDecision[];
  challenges?: EngineeringChallenge[];
  metrics?: ProjectMetric[];
  improvements?: string[];
  technologies: string[];
  codeSnippets?: CodeSnippet[];
  github?: string;
  live?: string;
}

export interface AcademicProject {
  slug: string;
  title: string;
  year: number;
  period: string;
  institution?: string;
  category: string;
  summary: string;
  highlights: string[];
  technologies: string[];
  github?: string;
}

export interface ExperienceItem {
  role: string;
  company: string;
  location: string;
  period: string;
  type: string;
  summary: string;
  bullets: string[];
  technologies: string[];
  verifiedMetrics?: {
    label: string;
    value: string;
  }[];
}

export interface EducationItem {
  institution: string;
  degree: string;
  major: string;
  period: string;
  location: string;
  focus?: string;
  coursework?: string[];
  honors?: string[];
}

export interface SkillCategory {
  id: string;
  title: string;
  description: string;
  skills: string[];
}

export interface EngineeringPrinciple {
  number: string;
  title: string;
  statement: string;
  elaboration: string;
}

export interface SocialLinks {
  github: string;
  linkedin: string;
  email: string;
  resumeUrl: string;
  statusText: string;
  availableForHire: boolean;
}
