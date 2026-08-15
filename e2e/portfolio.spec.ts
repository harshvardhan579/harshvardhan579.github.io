import { test, expect } from '@playwright/test';

test.describe('Harshvardhan Singh — Portfolio Reliability & Content Accuracy E2E', () => {
  test('Favicon & Identity: Custom HS Favicon and Canonical Title', async ({ page }) => {
    await page.goto('/');

    // Assert SEO Title & Metadata
    await expect(page).toHaveTitle(/Harshvardhan Singh — AI Engineer · Software Engineer/);

    // Assert Favicon request returns 200 SVG
    const iconResponse = await page.request.get('/icon.svg');
    expect(iconResponse.status()).toBe(200);
    const iconText = await iconResponse.text();
    expect(iconText).toContain('viewBox="0 0 32 32"');
    expect(iconText).toContain('#211914');
  });

  test('Hero VIEW WORK CTA: Boringly Reliable Single Click & Scroll Margin', async ({ page }, testInfo) => {
    const folder = `e2e/screenshots/${testInfo.project.name}`;
    await page.goto('/');
    await page.waitForTimeout(300);

    const initialScrollY = await page.evaluate(() => window.scrollY);
    expect(initialScrollY).toBe(0);

    // Locate primary VIEW WORK button
    const viewWorkBtn = page.locator('a[href="#work"]').filter({ hasText: /VIEW WORK/i }).first();
    await expect(viewWorkBtn).toBeVisible();

    // Click ONCE
    await viewWorkBtn.click();
    await page.waitForTimeout(800);

    // Assert scrollY increased significantly
    const scrolledY = await page.evaluate(() => window.scrollY);
    expect(scrolledY).toBeGreaterThan(200);

    // Assert the #work section is in viewport
    const workSection = page.locator('#work');
    await expect(workSection).toBeInViewport();

    // Assert the heading "Selected Files & Dossiers" is visible
    await expect(page.getByRole('heading', { name: 'Selected Files & Dossiers' })).toBeVisible();

    await page.screenshot({ path: `${folder}/01-view-work-clicked.png` });
  });

  test('Hero Folio Stack: Interactive 3D Depth Rail, Bounded Wheel & Boundary Latch', async ({ page }, testInfo) => {
    const folder = `e2e/screenshots/${testInfo.project.name}`;
    await page.goto('/');

    const viewport = page.viewportSize();
    const isDesktop = viewport ? viewport.width >= 1024 : false;

    if (isDesktop) {
      await expect(page.getByText('Dallas, TX (Open to Relocation)').filter({ visible: true }).first()).toBeVisible();

      const heroSection = page.locator('div[aria-label="Interactive Hero Engineering Dossier Stack"]');
      await expect(heroSection).toBeVisible();
      await expect(heroSection.getByRole('heading', { name: 'APILoom' })).toBeVisible();

      const depthRail = heroSection.locator('div[role="slider"]');
      await expect(depthRail).toBeVisible();
      await expect(depthRail).toHaveAttribute('aria-valuenow', '1');

      // 1. Mouse Wheel DOWN over Hero Stack navigates to CivicPulse (02)
      await heroSection.hover();
      await page.mouse.wheel(0, 120);
      await page.waitForTimeout(450);

      await expect(heroSection.getByRole('heading', { name: 'CivicPulse' })).toBeVisible();
      await expect(depthRail).toHaveAttribute('aria-valuenow', '2');

      // 2. Click tick 05 (AI Form Evaluator)
      const tick05 = heroSection.locator('button[aria-label*="AI Form Evaluator"]');
      await tick05.click();
      await page.waitForTimeout(450);
      await expect(heroSection.getByRole('heading', { name: 'AI Form Evaluator' })).toBeVisible();

      // 3. Boundary Latch Test: High-momentum scroll DOWN from 05 -> 06
      const scrollBefore = await page.evaluate(() => window.scrollY);
      await heroSection.hover();

      // Send strong momentum burst (simulate multi-frame trackpad swipe)
      await page.mouse.wheel(0, 100);
      await page.waitForTimeout(20);
      await page.mouse.wheel(0, 80);
      await page.waitForTimeout(20);
      await page.mouse.wheel(0, 60);
      await page.waitForTimeout(100);

      // Hero must now be on Hybrid ML Scheduler (06)
      await expect(heroSection.getByRole('heading', { name: 'Hybrid ML Scheduler' })).toBeVisible();

      // During the momentum tail, scrollY must remain stationary
      const scrollAfterTail = await page.evaluate(() => window.scrollY);
      expect(Math.abs(scrollAfterTail - scrollBefore)).toBeLessThan(10);

      // 4. Wait for gesture-end settle timer (~200ms)
      await page.waitForTimeout(250);

      // 5. Send a NEW downward scroll gesture while on 06 -> page scrolls naturally!
      await page.mouse.wheel(0, 250);
      await page.waitForTimeout(400);

      const scrollAfterNewGesture = await page.evaluate(() => window.scrollY);
      expect(scrollAfterNewGesture).toBeGreaterThan(scrollBefore + 50);

      await page.screenshot({ path: `${folder}/01-hero-boundary-latch.png` });

      // Return to top
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.waitForTimeout(200);
    }
  });

  test('Archive Category Filters (/projects): Strict Conditional Rendering for DATA and HARDWARE', async ({ page }, testInfo) => {
    const folder = `e2e/screenshots/${testInfo.project.name}`;
    await page.goto('/projects');
    await page.waitForTimeout(300);

    // 1. ALL Filter: Both Selected Builds and Academic Archive visible
    await expect(page.getByRole('heading', { name: 'Selected Builds / 2025—2026' })).toBeVisible();
    await expect(page.getByRole('heading', { name: 'Academic Archive / 2020—2025' })).toBeVisible();
    await page.screenshot({ path: `${folder}/07-archive-all.png`, fullPage: true });

    // 2. DATA Filter: Selected Builds must NOT appear; Academic data project must appear
    const dataFilterBtn = page.locator('button').filter({ hasText: /^DATA$/ }).first();
    await dataFilterBtn.click();
    await page.waitForTimeout(300);

    await expect(page.getByRole('heading', { name: 'Selected Builds / 2025—2026' })).not.toBeVisible();
    await expect(page.getByRole('heading', { name: 'Academic Archive / 2020—2025' })).toBeVisible();
    await expect(page.getByText('Database System Development').filter({ visible: true }).first()).toBeVisible();
    await page.screenshot({ path: `${folder}/07-archive-data.png`, fullPage: true });

    // 3. HARDWARE Filter: Selected Builds must NOT appear; Academic hardware project must appear
    const hardwareFilterBtn = page.locator('button').filter({ hasText: /^HARDWARE$/ }).first();
    await hardwareFilterBtn.click();
    await page.waitForTimeout(300);

    await expect(page.getByRole('heading', { name: 'Selected Builds / 2025—2026' })).not.toBeVisible();
    await expect(page.getByRole('heading', { name: 'Academic Archive / 2020—2025' })).toBeVisible();
    await expect(page.getByText('Pipelined Processor with Branch Prediction and Caching').filter({ visible: true }).first()).toBeVisible();
    await page.screenshot({ path: `${folder}/07-archive-hardware.png`, fullPage: true });

    // 4. Return to ALL
    const allFilterBtn = page.locator('button').filter({ hasText: /^ALL$/ }).first();
    await allFilterBtn.click();
    await page.waitForTimeout(300);
    await expect(page.getByRole('heading', { name: 'Selected Builds / 2025—2026' })).toBeVisible();
  });

  test('Portfolio Desk: Authoritative Big Vision Experience (Gen AI & ML Engineering Intern)', async ({ page }, testInfo) => {
    const folder = `e2e/screenshots/${testInfo.project.name}`;
    await page.goto('/');

    const viewport = page.viewportSize();
    const isMobile = viewport ? viewport.width < 1024 : false;

    // Switch to EXPERIENCE tab
    const expTab = isMobile
      ? page.locator('#mobile-tab-experience')
      : page.locator('button[role="tab"]').filter({ hasText: /EXP/i, visible: true }).first();
    await expTab.click({ force: true });
    await page.waitForTimeout(400);

    // Assert Company & New Role Title
    await expect(page.getByText('Big Vision').filter({ visible: true }).first()).toBeVisible();
    await expect(page.getByRole('heading', { name: 'Gen AI & ML Engineering Intern' }).filter({ visible: true }).first()).toBeVisible();

    // Assert Period & Location
    await expect(page.getByText('May 2023 — Jul 2023').filter({ visible: true }).first()).toBeVisible();
    await expect(page.getByText('Remote – San Diego, CA').filter({ visible: true }).first()).toBeVisible();

    // Assert Exact New Bullets & Keywords
    await expect(page.getByText('10,000+ uploads/day').filter({ visible: true }).first()).toBeVisible();
    await expect(page.getByText('PaddleOCR hyperparameters').filter({ visible: true }).first()).toBeVisible();
    await expect(page.getByText('Docker Compose').filter({ visible: true }).first()).toBeVisible();

    // Assert Old Role is absent from Big Vision
    const panelText = await page.locator('#work').textContent();
    expect(panelText).not.toContain('Software Engineer Intern');

    await page.locator('#work').screenshot({ path: `${folder}/03-tab-experience.png` });
  });

  test('Hydration Integrity: Zero Console Hydration Errors Across All Routes & URL Parameters', async ({ page }) => {
    const hydrationErrors: string[] = [];

    page.on('console', (msg) => {
      const text = msg.text();
      if (
        msg.type() === 'error' &&
        /hydration|server rendered|client rendered|did not match|react\.dev\/link\/hydration-mismatch/i.test(text)
      ) {
        hydrationErrors.push(text);
      }
    });

    page.on('pageerror', (err) => {
      if (/hydration|server rendered|client rendered|did not match/i.test(err.message)) {
        hydrationErrors.push(err.message);
      }
    });

    const routesToTest = [
      '/',
      '/?section=experience&tab=experience',
      '/?section=skills&tab=skills',
      '/?section=education&tab=education',
      '/?section=profile&tab=profile',
      '/projects',
      '/projects/apiloom',
      '/projects/apiloom?tab=architecture',
      '/projects/apiloom?tab=results',
    ];

    for (const route of routesToTest) {
      await page.goto(route);
      await page.waitForLoadState('networkidle');
      await page.waitForTimeout(200);
    }

    expect(hydrationErrors).toEqual([]);
  });

  test('Visual Audit & Multi-Viewport Responsiveness (Zero Overflow)', async ({ page }, testInfo) => {
    const folder = `e2e/screenshots/${testInfo.project.name}`;

    await page.goto('/');
    await page.waitForTimeout(400);

    await page.screenshot({ path: `${folder}/01-hero.png` });
    await page.screenshot({ path: `${folder}/00-full-page.png`, fullPage: true });

    // Assert Zero Horizontal Overflow across viewport
    const hasHorizontalOverflow = await page.evaluate(() => {
      return document.documentElement.scrollWidth > window.innerWidth;
    });
    expect(hasHorizontalOverflow).toBe(false);
  });
});
