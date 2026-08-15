/**
 * Central canonical contact metadata for Harshvardhan Singh.
 * Do not duplicate these strings throughout components.
 */
export const contactData = {
  name: 'Harshvardhan Singh',
  role: 'AI Engineer · Software Engineer',
  email: 'harshvardhan1singh1@gmail.com',
  emailHref: 'mailto:harshvardhan1singh1@gmail.com',
  phone: '+1 (814) 600-5794',
  phoneHref: 'tel:+18146005794',
  linkedin: 'https://linkedin.com/in/harshvardhan2',
  linkedinDisplay: 'linkedin.com/in/harshvardhan2',
  github: 'https://github.com/harshvardhan579',
  githubDisplay: 'github.com/harshvardhan579',
  location: 'Dallas, TX (Open to Relocation)',
  resumeUrl: '/Harshvardhan_Resume_August.pdf',
} as const;

export function isLinkValid(url: string | undefined | null): boolean {
  if (!url) return false;
  if (url.startsWith('TODO_') || url.includes('TODO')) return false;
  return true;
}
