import { contactData, isLinkValid as checkValid } from './contact';

export const socialData = {
  github: contactData.github,
  linkedin: contactData.linkedin,
  email: contactData.emailHref,
  emailPlain: contactData.email,
  phone: contactData.phoneHref,
  phonePlain: contactData.phone,
  location: contactData.location,
  resumeUrl: contactData.resumeUrl,
};

export const isLinkValid = checkValid;
