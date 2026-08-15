import { describe, it, expect } from 'vitest';
import { projectsData } from '@/data/projects';
import { academicProjectsData, academicYears, getAcademicProjectsByYear } from '@/data/academicProjects';
import { contactData, isLinkValid } from '@/data/contact';

describe('Portfolio Data Integrity & Phase 4 Content Truth', () => {
  it('must NOT contain EvalForge anywhere', () => {
    const slugs = projectsData.map((p) => p.slug.toLowerCase());
    const titles = projectsData.map((p) => p.title.toLowerCase());
    expect(slugs).not.toContain('evalforge');
    expect(titles).not.toContain('evalforge');
  });

  it('must have exact authoritative years for all 6 flagship projects', () => {
    const expectedYears: Record<string, number | string> = {
      'apiloom': 2026,
      'civicpulse': 2026,
      'arcade-game': 2026,
      'news-verify': 2026,
      'form-eval-app': 2026,
      'hybrid-ml-scheduler': 2025,
    };

    projectsData.forEach((project) => {
      expect(project.year).toBe(expectedYears[project.slug]);
    });
  });

  it('must contain APILoom as flagship project 01', () => {
    const apiloom = projectsData.find((p) => p.slug === 'apiloom');
    expect(apiloom).toBeDefined();
    expect(apiloom?.title).toBe('APILoom');
    expect(apiloom?.index).toBe('01');
    expect(apiloom?.year).toBe(2026);
    expect(apiloom?.github).toBe('https://github.com/harshvardhan579/apiloom');
  });

  it('must contain the canonical 6 projects in correct order', () => {
    expect(projectsData.length).toBe(6);
    const expectedSlugs = [
      'apiloom',
      'civicpulse',
      'arcade-game',
      'news-verify',
      'form-eval-app',
      'hybrid-ml-scheduler',
    ];
    expect(projectsData.map((p) => p.slug)).toEqual(expectedSlugs);
  });

  it('should have unique slugs and indices for every project', () => {
    const slugs = projectsData.map((p) => p.slug);
    const uniqueSlugs = new Set(slugs);
    expect(slugs.length).toBe(uniqueSlugs.size);

    const indices = projectsData.map((p) => p.index);
    const uniqueIndices = new Set(indices);
    expect(indices.length).toBe(uniqueIndices.size);
  });

  it('should have complete non-empty technical fields across all 6 projects', () => {
    projectsData.forEach((project) => {
      expect(project.title).toBeTruthy();
      expect(project.subtitle).toBeTruthy();
      expect(project.category).toBeTruthy();
      expect(project.summary).toBeTruthy();
      expect(project.problem).toBeTruthy();
      expect(project.motivation).toBeTruthy();
      expect(project.whatItDoes.length).toBeGreaterThan(0);
      expect(project.highlights.length).toBeGreaterThan(0);
      expect(project.technologies.length).toBeGreaterThan(0);
    });
  });

  describe('Academic & Earlier Projects Archive', () => {
    it('should contain all 12 academic projects', () => {
      expect(academicProjectsData.length).toBe(12);
    });

    it('should correctly group projects by year', () => {
      expect(academicYears).toEqual([2025, 2024, 2023, 2022, 2021, 2020]);
      expect(getAcademicProjectsByYear(2025).length).toBe(3);
      expect(getAcademicProjectsByYear(2024).length).toBe(1);
      expect(getAcademicProjectsByYear(2023).length).toBe(1);
      expect(getAcademicProjectsByYear(2022).length).toBe(3);
      expect(getAcademicProjectsByYear(2021).length).toBe(3);
      expect(getAcademicProjectsByYear(2020).length).toBe(1);
    });

    it('should share the correct repository for the 3 RIT ML projects', () => {
      const ritProjects = getAcademicProjectsByYear(2025);
      expect(ritProjects.length).toBe(3);
      ritProjects.forEach((p) => {
        expect(p.github).toBe('https://github.com/harshvardhan579/ml-text-vision-projects');
        expect(p.institution).toBe('Rochester Institute of Technology');
      });
    });

    it('should configure Room Scheduler with its verified repository and omit unhosted ones', () => {
      const roomScheduler = academicProjectsData.find((p) => p.slug === 'room-scheduler-java');
      expect(roomScheduler).toBeDefined();
      expect(roomScheduler?.github).toBe('https://github.com/harshvardhan579/RoomSchedulerJava');

      const dbProject = academicProjectsData.find((p) => p.slug === 'database-system-development');
      expect(dbProject?.github).toBeUndefined();

      const osProject = academicProjectsData.find((p) => p.slug === 'custom-os-simulation');
      expect(osProject?.github).toBeUndefined();
    });

    it('should have non-empty required fields across all 12 academic projects', () => {
      academicProjectsData.forEach((project) => {
        expect(project.slug).toBeTruthy();
        expect(project.title).toBeTruthy();
        expect(project.year).toBeGreaterThanOrEqual(2020);
        expect(project.period).toBeTruthy();
        expect(project.category).toBeTruthy();
        expect(project.summary).toBeTruthy();
        expect(project.highlights.length).toBeGreaterThan(0);
        expect(project.technologies.length).toBeGreaterThan(0);
      });
    });
  });

  describe('Experience Data Integrity', () => {
    it('should have exact authoritative Big Vision Gen AI & ML Engineering Intern experience', async () => {
      const { experienceData } = await import('@/data/experience');
      const bigVision = experienceData.find((e) => e.company === 'Big Vision');
      expect(bigVision).toBeDefined();
      expect(bigVision?.role).toBe('Gen AI & ML Engineering Intern');
      expect(bigVision?.period).toBe('May 2023 — Jul 2023');
      expect(bigVision?.location).toBe('Remote – San Diego, CA');
      expect(bigVision?.bullets.length).toBe(3);

      // Check key phrases in bullets
      expect(bigVision?.bullets[0]).toContain('10,000+ uploads/day');
      expect(bigVision?.bullets[0]).toContain('React/TypeScript');
      expect(bigVision?.bullets[0]).toContain('FastAPI');
      expect(bigVision?.bullets[1]).toContain('PaddleOCR hyperparameters');
      expect(bigVision?.bullets[2]).toContain('Docker Compose');
      expect(bigVision?.bullets[2]).toContain('Pytest/Jest');

      // Assert absence of stale claims
      const allText = JSON.stringify(bigVision);
      expect(allText).not.toContain('Software Engineer Intern');
      expect(allText).not.toContain('800ms');
      expect(allText).not.toContain('45m');
      expect(allText).not.toContain('~90%');
    });
  });

  it('should have canonical contact information with clickable protocols', () => {
    expect(contactData.name).toBe('Harshvardhan Singh');
    expect(contactData.email).toBe('harshvardhan1singh1@gmail.com');
    expect(contactData.emailHref).toBe('mailto:harshvardhan1singh1@gmail.com');
    expect(contactData.phone).toBe('+1 (814) 600-5794');
    expect(contactData.phoneHref).toBe('tel:+18146005794');
    expect(contactData.linkedin).toBe('https://linkedin.com/in/harshvardhan2');
    expect(contactData.github).toBe('https://github.com/harshvardhan579');
    expect(contactData.location).toBe('Dallas, TX (Open to Relocation)');
    expect(contactData.resumeUrl).toBe('/Harshvardhan_Resume_August.pdf');
  });

  it('should properly validate links with TODO_ placeholders', () => {
    expect(isLinkValid('https://github.com/TODO_GITHUB_URL')).toBe(false);
    expect(isLinkValid('https://github.com/harshvardhan579/apiloom')).toBe(true);
    expect(isLinkValid('/Harshvardhan_Resume_August.pdf')).toBe(true);
    expect(isLinkValid('')).toBe(false);
    expect(isLinkValid(undefined)).toBe(false);
  });
});
