'use client';

import React, { useState } from 'react';
import { Check, Copy, Terminal } from 'lucide-react';

interface CodeBlockProps {
  code: string;
  language?: string;
  filename?: string;
  caption?: string;
}

export function CodeBlock({
  code,
  language = 'python',
  filename,
  caption,
}: CodeBlockProps) {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(code);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy code', err);
    }
  };

  return (
    <div className="my-4 rounded-[2px] border border-[#302019] bg-[#251813] overflow-hidden shadow-paper-lg">
      {/* Code Header Bar */}
      <div className="flex items-center justify-between px-4 py-2 bg-[#1D1511] border-b border-[#302019]">
        <div className="flex items-center gap-2 text-xs font-mono text-[#BEB09C]">
          <Terminal className="w-3.5 h-3.5 text-[#8C4432]" />
          <span>{filename || `${language} snippet`}</span>
        </div>

        <button
          onClick={handleCopy}
          className="flex items-center gap-1.5 px-2 py-0.5 rounded-[2px] text-xs font-mono text-[#BEB09C] hover:text-[#FAF6ED] hover:bg-[#302019] transition-colors focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-[#8C4432] cursor-pointer"
          aria-label="Copy code to clipboard"
        >
          {copied ? (
            <>
              <Check className="w-3.5 h-3.5 text-[#8C4432]" />
              <span className="text-[#8C4432]">COPIED</span>
            </>
          ) : (
            <>
              <Copy className="w-3.5 h-3.5" />
              <span>COPY</span>
            </>
          )}
        </button>
      </div>

      {/* Code Content */}
      <pre className="p-4 overflow-x-auto text-xs font-mono text-[#F1E8D8] leading-relaxed selection:bg-[#8C4432]/40">
        <code>{code}</code>
      </pre>

      {caption && (
        <div className="px-4 py-1.5 bg-[#1D1511] border-t border-[#302019] text-[10px] font-mono text-[#BEB09C]">
          {caption}
        </div>
      )}
    </div>
  );
}
