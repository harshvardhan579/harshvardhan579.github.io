import React from 'react';
import { describe, it, expect } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { Badge } from '@/components/ui/Badge';
import { Button } from '@/components/ui/Button';
import { EditorialRule } from '@/components/editorial/EditorialRule';
import { PaperCard } from '@/components/editorial/PaperCard';
import { Masthead } from '@/components/editorial/Masthead';
import { ProjectDeck } from '@/components/portfolio/ProjectDeck';
import { HeroFolioStack } from '@/components/home/HeroFolioStack';

describe('Retro-Future Editorial UI Primitives', () => {
  it('renders Badge component with bracket format', () => {
    render(<Badge variant="bracket">Agentic AI</Badge>);
    expect(screen.getByText('Agentic AI')).toBeInTheDocument();
  });

  it('renders EditorialRule with label', () => {
    render(<EditorialRule label="Archive Index" />);
    expect(screen.getByText('Archive Index')).toBeInTheDocument();
  });

  it('renders Masthead banner with archive title', () => {
    render(<Masthead />);
    expect(screen.getByText('THE HARSHVARDHAN SINGH ARCHIVE')).toBeInTheDocument();
  });

  it('renders PaperCard with content and registration marks', () => {
    render(
      <PaperCard>
        <div>Dossier Content</div>
      </PaperCard>
    );
    expect(screen.getByText('Dossier Content')).toBeInTheDocument();
  });

  it('renders Button component and filters out TODO_ links', () => {
    render(
      <div>
        <Button href="https://github.com/realuser/realrepo">Valid Link</Button>
        <Button href="https://github.com/TODO_GITHUB_URL">Hidden Link</Button>
      </div>
    );
    expect(screen.getByText('Valid Link')).toBeInTheDocument();
    expect(screen.queryByText('Hidden Link')).not.toBeInTheDocument();
  });

  describe('HeroFolioStack Component', () => {
    it('renders hero folio stack with APILoom initially and displays 3D depth rail', () => {
      render(<HeroFolioStack />);
      expect(screen.getByText('APILoom')).toBeInTheDocument();
      expect(screen.getByText('01')).toBeInTheDocument();
      expect(screen.getByText('06')).toBeInTheDocument();
      expect(screen.getByRole('slider', { name: /Hero Project Depth Rail/i })).toBeInTheDocument();
    });

    it('navigates to next project via keyboard ArrowDown on the rail', async () => {
      render(<HeroFolioStack />);
      const rail = screen.getByRole('slider', { name: /Hero Project Depth Rail/i });

      fireEvent.keyDown(rail, { key: 'ArrowDown' });
      await waitFor(() => {
        expect(rail).toHaveAttribute('aria-valuenow', '2');
      });

      fireEvent.keyDown(rail, { key: 'End' });
      await waitFor(() => {
        expect(rail).toHaveAttribute('aria-valuenow', '6');
      });

      fireEvent.keyDown(rail, { key: 'Home' });
      await waitFor(() => {
        expect(rail).toHaveAttribute('aria-valuenow', '1');
      });
    });
  });

  describe('Lower ProjectDeck Component (Restored Horizontal Switcher)', () => {
    it('renders top horizontal project buttons for all 6 projects', () => {
      render(<ProjectDeck initialIndex={0} />);
      expect(screen.getByRole('button', { name: /Select project APILoom/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Select project CivicPulse/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Select project Pocket Arcade/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Select project NewsVerify/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Select project AI Form Evaluator/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Select project Hybrid ML Scheduler/i })).toBeInTheDocument();
    });

    it('switches active project when clicking top horizontal buttons', async () => {
      render(<ProjectDeck initialIndex={0} />);
      const civicBtn = screen.getByRole('button', { name: /Select project CivicPulse/i });
      fireEvent.click(civicBtn);

      await waitFor(() => {
        expect(civicBtn).toHaveAttribute('aria-pressed', 'true');
      });
    });

    it('does NOT render the Hero 3D Project Rail on the lower deck', () => {
      render(<ProjectDeck initialIndex={0} />);
      expect(screen.queryByRole('slider', { name: /Hero Project Depth Rail/i })).not.toBeInTheDocument();
    });
  });
});
