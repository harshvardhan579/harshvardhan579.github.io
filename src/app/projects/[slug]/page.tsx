import React from 'react';
import { notFound } from 'next/navigation';
import type { Metadata } from 'next';
import { projectsData } from '@/data/projects';
import { ProjectDossier } from '@/components/project/ProjectDossier';

interface PageProps {
  params: Promise<{ slug: string }>;
}

export async function generateStaticParams() {
  return projectsData.map((project) => ({
    slug: project.slug,
  }));
}

export async function generateMetadata({ params }: PageProps): Promise<Metadata> {
  const { slug } = await params;
  const project = projectsData.find((p) => p.slug === slug);
  if (!project) return { title: 'Project Not Found' };

  return {
    title: `${project.title} — Technical Case Study | Harshvardhan Singh`,
    description: project.summary,
    openGraph: {
      title: `${project.title} — Engineering Dossier`,
      description: project.summary,
    },
  };
}

export default async function ProjectDetailPage({ params }: PageProps) {
  const { slug } = await params;
  const projectIndex = projectsData.findIndex((p) => p.slug === slug);

  if (projectIndex === -1) {
    notFound();
  }

  const project = projectsData[projectIndex];
  const prevProject =
    projectIndex > 0
      ? {
          slug: projectsData[projectIndex - 1].slug,
          title: projectsData[projectIndex - 1].title,
        }
      : null;

  const nextProject =
    projectIndex < projectsData.length - 1
      ? {
          slug: projectsData[projectIndex + 1].slug,
          title: projectsData[projectIndex + 1].title,
        }
      : null;

  return (
    <main className="w-full py-10 px-4 sm:px-6 lg:px-8 bg-[#EEE8DC] flex-1">
      <div className="max-w-[1280px] mx-auto">
        <ProjectDossier
          project={project}
          prevProject={prevProject}
          nextProject={nextProject}
        />
      </div>
    </main>
  );
}
