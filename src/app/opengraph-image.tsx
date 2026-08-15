import { ImageResponse } from 'next/og';

export const alt = 'Harshvardhan Singh — AI Engineer · Software Engineer';
export const size = {
  width: 1200,
  height: 630,
};
export const contentType = 'image/png';
export const dynamic = 'force-static';

export default async function Image() {
  return new ImageResponse(
    (
      <div
        style={{
          background: '#EEE8DC',
          width: '100%',
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'space-between',
          padding: '70px',
          fontFamily: 'serif',
          position: 'relative',
          border: '12px solid #E7DECF',
        }}
      >
        {/* Masthead Header Bar */}
        <div
          style={{
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            borderBottom: '2px solid #302019',
            paddingBottom: '16px',
            fontFamily: 'monospace',
            fontSize: '16px',
            color: '#857267',
            letterSpacing: '2px',
          }}
        >
          <span>THE HARSHVARDHAN SINGH ARCHIVE</span>
          <span>VOL. 01 — 2026 EDITION</span>
          <span>ENGINEERING FOLIO</span>
        </div>

        {/* Main Title & Subtitle */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: '14px' }}>
          <div
            style={{
              fontSize: '18px',
              fontFamily: 'monospace',
              color: '#8C4432',
              letterSpacing: '2px',
              fontWeight: 'bold',
            }}
          >
            [ AI ENGINEER · SOFTWARE ENGINEER ]
          </div>
          <h1
            style={{
              fontSize: '76px',
              fontWeight: 400,
              color: '#1D1511',
              letterSpacing: '-1px',
              margin: 0,
              lineHeight: 1.0,
            }}
          >
            Harshvardhan Singh
          </h1>
          <p
            style={{
              fontSize: '24px',
              color: '#5A463B',
              margin: 0,
              fontFamily: 'sans-serif',
            }}
          >
            Applied AI Systems · LLM Evaluation Harnesses · Distributed Backend Architecture
          </p>
        </div>

        {/* Footer */}
        <div
          style={{
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            borderTop: '1px solid #BEB09C',
            paddingTop: '20px',
            color: '#857267',
            fontSize: '16px',
            fontFamily: 'monospace',
          }}
        >
          <span>harshvardhansingh.dev</span>
          <span>RIT (MS CS) · PENN STATE (BS CS)</span>
        </div>
      </div>
    ),
    {
      ...size,
    }
  );
}
