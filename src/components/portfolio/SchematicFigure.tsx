'use client';

import React from 'react';
import { motion } from 'framer-motion';

interface SchematicFigureProps {
  slug: string;
  className?: string;
}

export function SchematicFigure({ slug, className = '' }: SchematicFigureProps) {
  const renderDiagram = () => {
    switch (slug) {
      case 'apiloom':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 1.0 — PLANNING VS EXECUTION BOUNDARY</span>
              <span>APILOOM</span>
            </div>

            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 text-center">
              <div className="p-2.5 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] space-y-1">
                <span className="text-[9px] text-[#8A3F32] font-semibold block uppercase">PROBABILISTIC</span>
                <span className="text-[#211914] font-medium block">[ OPENAI PLANNER ]</span>
                <span className="text-[10px] text-[#75665B] block">Structured Function Call</span>
              </div>

              <div className="p-2.5 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] space-y-1">
                <span className="text-[9px] text-[#8A3F32] font-semibold block uppercase">VALIDATION</span>
                <span className="text-[#211914] font-medium block">[ PYDANTIC RUNBOOK ]</span>
                <span className="text-[10px] text-[#75665B] block">Acyclic Plan Schema</span>
              </div>
            </div>

            <div className="flex items-center justify-center gap-2 text-[10px] text-[#75665B]">
              <span>↓</span>
              <span className="bg-[#E7DECF] px-2 py-0.5 rounded-[2px] text-[#8A3F32] font-semibold">
                ONE-USE MUTATION APPROVAL TOKEN
              </span>
              <span>↓</span>
            </div>

            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 text-center">
              <div className="p-2.5 bg-[#211914] text-[#FCF9F2] border border-[#211914] rounded-[2px] space-y-1">
                <span className="text-[9px] text-[#C9BBA6] font-semibold block uppercase">DETERMINISTIC</span>
                <span className="font-medium block">[ GUARDED HTTPX ]</span>
                <span className="text-[10px] text-[#C9BBA6] block">SSRF & Mutation Policy</span>
              </div>

              <div className="p-2.5 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] space-y-1">
                <span className="text-[9px] text-[#8A3F32] font-semibold block uppercase">IMMUTABLE</span>
                <span className="text-[#211914] font-medium block">[ POSTGRESQL AUDIT ]</span>
                <span className="text-[10px] text-[#75665B] block">UNKNOWN_OUTCOME Semantics</span>
              </div>
            </div>
          </div>
        );

      case 'civicpulse':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 2.0 — BOUNDED WORKFLOW TOPOLOGY</span>
              <span>CIVICPULSE</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-center">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">STAGE 01</span>
                <span className="text-[#211914]">[ GENERATE ]</span>
              </div>
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">STAGE 02</span>
                <span className="text-[#211914]">[ AI CRITIQUE ]</span>
              </div>
            </div>

            <div className="p-2 bg-[#E7DECF]/80 border border-[#8A3F32] text-center rounded-[2px] space-y-0.5">
              <span className="text-[9px] text-[#8A3F32] font-bold block uppercase">STAGE 03 / DETERMINISTIC GATE</span>
              <span className="text-[#211914] text-xs font-semibold">[ CLAIM CHECK (NO LLM) ]</span>
              <span className="text-[9px] text-[#75665B] block">Rejects ungrounded numerical claims</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-center">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">STAGE 04</span>
                <span className="text-[#211914]">[ REVISION ]</span>
              </div>
              <div className="p-2 bg-[#211914] text-[#FCF9F2] border border-[#211914] rounded-[2px]">
                <span className="text-[9px] text-[#C9BBA6] block">STAGE 05</span>
                <span>[ HUMAN REVIEW ]</span>
              </div>
            </div>
          </div>
        );

      case 'arcade-game':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 3.0 — ZERO-ASSET CANVAS ENGINE</span>
              <span>POCKET ARCADE</span>
            </div>

            <div className="grid grid-cols-3 gap-1.5 text-center text-[10px]">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="block text-[#8A3F32]">0 KB</span>
                <span className="text-[#211914] font-medium">Assets</span>
              </div>
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="block text-[#8A3F32]">60 FPS</span>
                <span className="text-[#211914] font-medium">Fixed Step</span>
              </div>
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="block text-[#8A3F32]">5 Games</span>
                <span className="text-[#211914] font-medium">Canvas</span>
              </div>
            </div>

            <div className="p-3 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px] space-y-1">
              <div className="flex justify-between text-[10px] text-[#75665B]">
                <span>WEB AUDIO SYNTHESIZER</span>
                <span>REAL-TIME ADSR</span>
              </div>
              <div className="text-[10px] text-[#211914]">
                Oscillator → Exponential Frequency Decay → Dynamic Waveforms
              </div>
            </div>
          </div>
        );

      case 'news-verify':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 4.0 — MULTIMODAL SYNTHESIS</span>
              <span>NEWSVERIFY</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-center">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">VISION & OCR</span>
                <span className="text-[#211914]">[ EXTRACT CLAIM ]</span>
              </div>
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">DECOMPOSITION</span>
                <span className="text-[#211914]">[ SUB-ASSERTIONS ]</span>
              </div>
            </div>

            <div className="p-2 bg-[#E7DECF]/80 border border-[#C9BBA6] text-center rounded-[2px]">
              <span className="text-[10px] text-[#8A3F32] font-semibold block">EVIDENCE GROUNDING ENGINE</span>
              <span className="text-[#211914] text-[10px]">SerpApi Retrieval → Strict Index-Constrained Citations</span>
            </div>
          </div>
        );

      case 'form-eval-app':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 5.0 — EDGE POSE KINEMATICS</span>
              <span>AI FORM EVAL</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-center">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#8A3F32] block">IN-BROWSER WASM</span>
                <span className="text-[#211914]">[ 33 LANDMARKS ]</span>
              </div>
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#75665B] block">TRANSPORT</span>
                <span className="text-[#211914]">[ WEBSOCKETS ]</span>
              </div>
            </div>

            <div className="p-2 bg-[#211914] text-[#FCF9F2] text-center rounded-[2px]">
              <span className="text-[10px] text-[#C9BBA6] block uppercase">PRIVACY GUARANTEE</span>
              <span className="text-xs">Raw video never leaves client device</span>
            </div>
          </div>
        );

      case 'hybrid-ml-scheduler':
        return (
          <div className="space-y-3 font-mono text-[11px]">
            <div className="flex items-center justify-between text-[10px] text-[#75665B] border-b border-[#C9BBA6]/60 pb-1 uppercase tracking-wider">
              <span>FIG. 6.0 — RL VS ORACLE BENCHMARK</span>
              <span>HYBRID SCHEDULER</span>
            </div>

            <div className="grid grid-cols-2 gap-2 text-center">
              <div className="p-2 bg-[#FCF9F2] border border-[#C9BBA6] rounded-[2px]">
                <span className="text-[9px] text-[#8A3F32] block">LEARNED POLICY</span>
                <span className="text-[#211914]">[ DUELING DQN ]</span>
              </div>
              <div className="p-2 bg-[#211914] text-[#FCF9F2] border border-[#211914] rounded-[2px]">
                <span className="text-[9px] text-[#C9BBA6] block">GROUND TRUTH</span>
                <span>[ ORACLE OPTIMUM ]</span>
              </div>
            </div>

            <div className="p-2 bg-[#E7DECF]/80 border border-[#C9BBA6] text-center rounded-[2px]">
              <span className="text-[10px] text-[#211914]">6 Scheduling Policies Raced Live Across Cluster Workloads</span>
            </div>
          </div>
        );

      default:
        return null;
    }
  };

  return (
    <motion.div
      initial={{ opacity: 0, y: 6 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.3 }}
      className={`bg-[#E7DECF]/50 border border-[#C9BBA6] p-4 rounded-[2px] ${className}`}
    >
      {renderDiagram()}
    </motion.div>
  );
}
