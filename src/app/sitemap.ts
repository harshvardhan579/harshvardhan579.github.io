import { MetadataRoute } from 'next';
import { projectsData } from '@/data/projects';

export const dynamic = 'force-static';

export default function sitemap(): MetadataRoute.Sitemap {
  const baseUrl = 'https://harshvardhan579.github.io';

  const projectUrls = projectsData.map((project) => ({
    url: `${baseUrl}/projects/${project.slug}/`,
    lastModified: new Date('2026-08-15'),
    changeFrequency: 'monthly' as const,
    priority: 0.8,
  }));

  return [
    {
      url: `${baseUrl}/`,
      lastModified: new Date('2026-08-15'),
      changeFrequency: 'weekly',
      priority: 1.0,
    },
    {
      url: `${baseUrl}/projects/`,
      lastModified: new Date('2026-08-15'),
      changeFrequency: 'weekly',
      priority: 0.9,
    },
    ...projectUrls,
  ];
}
