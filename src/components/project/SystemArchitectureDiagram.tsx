import React from 'react';
import { ArchitectureLayer } from '@/types/portfolio';
import { Layers, ArrowDown, Database, Server, Cpu, Monitor } from 'lucide-react';

export function SystemArchitectureDiagram({
  layers,
  title,
}: {
  layers: ArchitectureLayer[];
  title: string;
}) {
  const layerIcons = [
    <Monitor key="1" className="w-4 h-4 text-[#7C8CFF]" />,
    <Server key="2" className="w-4 h-4 text-[#67E8F9]" />,
    <Cpu key="3" className="w-4 h-4 text-[#C6F36B]" />,
    <Database key="4" className="w-4 h-4 text-[#F4F6F8]" />,
  ];

  return (
    <div className="rounded-xl border border-white/10 bg-[#0A0C10] p-6 sm:p-8 my-8 shadow-xl">
      {/* Header */}
      <div className="flex items-center justify-between border-b border-white/[0.08] pb-4 mb-6">
        <div className="flex items-center gap-2.5">
          <Layers className="w-5 h-5 text-[#7C8CFF]" />
          <h4 className="text-base font-bold text-[#F4F6F8] font-sans">
            {title} — System Architecture Topology
          </h4>
        </div>
        <span className="text-xs font-mono text-[#6F7682] uppercase">
          {layers.length} Layers
        </span>
      </div>

      {/* Layer stack */}
      <div className="space-y-4">
        {layers.map((layer, index) => (
          <React.Fragment key={layer.name}>
            <div className="p-4 sm:p-5 rounded-lg border border-white/[0.08] bg-[#0E1117] hover:border-white/20 transition-colors">
              <div className="flex flex-col sm:flex-row sm:items-center justify-between gap-2 mb-2">
                <div className="flex items-center gap-2">
                  <div className="p-1.5 rounded bg-white/[0.04]">
                    {layerIcons[index % layerIcons.length]}
                  </div>
                  <span className="font-mono text-sm font-bold text-[#F4F6F8]">
                    Layer 0{index + 1}: {layer.name}
                  </span>
                </div>
                <span className="text-xs font-mono text-[#6F7682]">
                  {layer.components.length} components
                </span>
              </div>

              <p className="text-xs sm:text-sm text-[#A1A7B3] mb-3">
                {layer.description}
              </p>

              <div className="flex flex-wrap gap-1.5">
                {layer.components.map((comp) => (
                  <span
                    key={comp}
                    className="text-xs font-mono px-2.5 py-1 rounded bg-[#151820] text-[#7C8CFF] border border-[#7C8CFF]/20"
                  >
                    {comp}
                  </span>
                ))}
              </div>
            </div>

            {index < layers.length - 1 && (
              <div className="flex justify-center py-1">
                <div className="flex items-center gap-1.5 font-mono text-[11px] text-[#6F7682]">
                  <ArrowDown className="w-3.5 h-3.5 text-[#7C8CFF] animate-pulse" />
                  <span className="uppercase text-[10px] tracking-wider">
                    Data Flow / IPC
                  </span>
                </div>
              </div>
            )}
          </React.Fragment>
        ))}
      </div>
    </div>
  );
}
