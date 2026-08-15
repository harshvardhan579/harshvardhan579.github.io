import { Project } from '@/types/portfolio';

// Project copy synchronized against the owner's GitHub profile/project READMEs (https://github.com/harshvardhan579).
// Do not add unverified claims or non-existent projects (EvalForge has been removed).

export const projectsData: Project[] = [
  {
    slug: 'apiloom',
    title: 'APILoom',
    subtitle: 'Local-First AI API Runbooks with Reviewed Execution',
    category: 'AI Infrastructure · Agents · APIs',
    index: '01',
    year: 2026,
    featured: true,
    summary:
      'APILoom separates probabilistic AI planning from deterministic execution, turning natural-language API procedures into reviewable, reusable runbooks with an immutable PostgreSQL audit trail.',
    problem:
      'One-off API automation scripts written by LLMs are difficult to review before they fire, unsafe when mutating external resources, impossible to reliably re-run, and silently fail or leave ambiguous states when network interruptions occur.',
    motivation:
      'Getting an LLM to generate an HTTP call is trivial; the hard engineering problem is making that call reviewable before it executes, deterministic and repeatable afterwards, and rigorously honest when an interruption happens mid-mutation.',
    whatItDoes: [
      'Separates OpenAI-backed workflow planning into a Pydantic-validated acyclic runbook schema that replays deterministically without subsequent model invocations.',
      'Enforces explicit, one-use mutation approval tokens cryptographically bound to the plan revision, resolved-input digest, and connection versions.',
      'Applies SSRF-aware destination protection denying loopback, private, link-local, and cloud metadata targets, rechecked at TCP connection time.',
      'Implements honest failure semantics, recording UNKNOWN_OUTCOME whenever an in-flight mutating request may have crossed the network boundary.',
      'Provides full operational lifecycle tooling: Alembic migrations, startup stale-run reconciliation, Fernet credential encryption, and retention policies.',
    ],
    highlights: [
      'Planning isolated from execution: OpenAI function calling validated into Pydantic schema.',
      'One-use mutation preview token invalidated if inputs, revisions, or credentials change.',
      'SSRF-aware DNS and connection checks blocking private IP ranges and redirects.',
      'Honest UNKNOWN_OUTCOME status recorded for ambiguous network boundaries.',
    ],
    architectureLayers: [
      {
        name: 'Planning & Schema Validation',
        description:
          'Ingests natural-language procedures, invokes OpenAI structured outputs, and strictly validates generated steps into a DAG with backward-pointing dependencies.',
        components: ['OpenAI Function Calling', 'Pydantic v2 Schema', 'DAG Acyclic Validator', 'Input Constraint Engine'],
      },
      {
        name: 'Pre-Flight & Security Vault',
        description:
          'Performs pre-execution DNS resolution, destination policy enforcement, and Fernet credential decryption bound to exact origins with sensitive header redaction.',
        components: ['SSRF Policy Guard', 'Origin-Bound Fernet Vault', 'Masked Preview Tokenizer', 'Digest Verification'],
      },
      {
        name: 'Deterministic Execution Engine',
        description:
          'Runs runbook steps sequentially via guarded HTTPX clients with total deadlines, cancellation tokens, and idempotency-aware retries (mutations retry only with explicit keys).',
        components: ['Guarded HTTPX Worker', 'Sequential Executor', 'Idempotency Key Manager', 'Deadline Controller'],
      },
      {
        name: 'Audit Stream & History Persistence',
        description:
          'Persists per-step assertion outcomes, durations, retry counts, and chronological audit logs with startup recovery of interrupted runs.',
        components: ['PostgreSQL 16 Engine', 'Alembic Migration Track', 'Stale Run Reconciler', 'Retention Purge Engine'],
      },
    ],
    aiDecisions: [
      {
        title: 'Structured Output Validation over Freeform Code Gen',
        rationale:
          'Freeform Python/Bash script generation cannot be statically bounded or checked for SSRF before execution. Emitting a constrained Pydantic DAG guarantees all endpoints, verbs, and payload fields are inspectable.',
        tradeOff:
          'Limits procedures to structured REST APIs without arbitrary compute branching in v0.1.',
      },
      {
        title: 'Zero Re-Prompting on Runbook Replay',
        rationale:
          'Once a plan is validated and saved as a runbook, subsequent executions take typed runtime inputs directly, eliminating non-deterministic LLM variance and runtime inference costs.',
        tradeOff:
          'Changes to third-party API contracts require a deliberate plan update rather than dynamic runtime adaptation.',
      },
    ],
    systemDesignDecisions: [
      {
        title: 'One-Use Cryptographic Preview Tokens',
        rationale:
          'To prevent race conditions where a mutation plan is modified between review and execution, the approval token binds the plan revision hash, input digest, and connection versions.',
        tradeOff:
          'Editing a single input parameter requires generating a fresh preview token.',
      },
      {
        title: 'Explicit UNKNOWN_OUTCOME Failure Semantics',
        rationale:
          'If a TCP connection drops or timeouts occur after HTTP headers are transmitted on a POST/PUT/DELETE, assuming failure risks duplicate mutations upon automated retry.',
        tradeOff:
          'Requires human operator inspection rather than blind automatic retry.',
      },
    ],
    challenges: [
      {
        title: 'SSRF Protection Against DNS Rebinding Attacks',
        problem:
          'A malicious host could resolve to a public IP during pre-flight validation but resolve to 127.0.0.1 (or cloud metadata 169.254.169.254) during socket connection.',
        solution:
          'Custom HTTPX transport hooks perform DNS resolution at the exact socket connect time, asserting that the destination IP remains strictly in public ranges before transmitting bytes.',
        takeaway:
          'Validating URLs as strings is insufficient; destination policy must hook the network transport layer.',
      },
      {
        title: 'Deadlock-Free Stale Run Reconciliation on Startup',
        problem:
          'If the APILoom server process crashes during active execution, runs remain in RUNNING state indefinitely, blocking subsequent operations.',
        solution:
          'Implemented an idempotent startup lifecycle hook in FastAPI that scans PostgreSQL for uncompleted executions, reconciling them to UNKNOWN_OUTCOME or FAILED with crash markers.',
        takeaway:
          'Crash recovery must be designed into the persistence model as an explicit startup step.',
      },
    ],
    metrics: [
      {
        label: 'Automated CI Tests',
        value: '1,072',
        context: 'Unit, integration, and security tests with enforced ≥80% coverage.',
      },
      {
        label: 'Mutation Repeatability',
        value: '50×',
        context: 'Deterministic mutation-deadline regression passing 50 consecutive runs.',
      },
      {
        label: 'Inference Overhead on Replay',
        value: '0 ms',
        context: 'Zero LLM model invocations required for saved runbook replay.',
      },
    ],
    improvements: [
      'Parallel branch execution for non-dependent DAG nodes.',
      'OpenAPI 3.1 schema import for instant runbook generation without LLM prompts.',
      'Webhook trigger listeners for asynchronous external event execution.',
    ],
    technologies: ['FastAPI', 'Next.js 16', 'PostgreSQL 16', 'Docker', 'Pydantic v2', 'OpenAI API', 'HTTPX', 'Alembic'],
    codeSnippets: [
      {
        language: 'python',
        filename: 'apiloom/security/destination_policy.py',
        caption: 'SSRF-aware socket destination validator blocking private ranges',
        code: `import ipaddress
import socket

BLOCKED_NETWORKS = [
    ipaddress.ip_network("127.0.0.0/8"),
    ipaddress.ip_network("10.0.0.0/8"),
    ipaddress.ip_network("172.16.0.0/12"),
    ipaddress.ip_network("192.168.0.0/16"),
    ipaddress.ip_network("169.254.0.0/16"),  # Cloud metadata
]

def assert_public_destination(ip_str: str) -> None:
    ip = ipaddress.ip_address(ip_str)
    for blocked in BLOCKED_NETWORKS:
        if ip in blocked:
            raise SecurityPolicyViolation(f"Target IP {ip_str} is in blocked range: {blocked}")`,
      },
      {
        language: 'python',
        filename: 'apiloom/engine/executor.py',
        caption: 'Honest mutation failure semantics recording UNKNOWN_OUTCOME',
        code: `async def execute_mutation_step(step: WorkflowStep, preview_token: str) -> StepResult:
    verify_token_integrity(preview_token, step)
    try:
        response = await client.send(step.request, timeout=step.timeout)
        return StepResult(status="SUCCESS", http_code=response.status_code)
    except httpx.NetworkError as exc:
        if step.is_mutating:
            # Network boundary crossed; cannot guarantee target did not process mutation
            return StepResult(status="UNKNOWN_OUTCOME", error=str(exc))
        raise`,
      },
    ],
    github: 'https://github.com/harshvardhan579/apiloom',
    live: '',
  },
  {
    slug: 'civicpulse',
    title: 'CivicPulse',
    subtitle: 'Bounded AI Workflows with Human Corrections Preserved as Immutable Facts',
    category: 'Applied AI · NLP · Full Stack',
    index: '02',
    year: 2026,
    featured: true,
    summary:
      'A bounded, human-reviewed AI workflow platform where model output and human edits are preserved as separate immutable records, ensuring disagreements remain measurable rather than silently overwritten.',
    problem:
      'Civic organizations and public policy teams struggle to process resident survey responses while simultaneously drafting targeted, policy-accurate constituent communications. Manual workflows create bottlenecks and ungrounded LLM outputs risk distributing non-compliant civic messages.',
    motivation:
      'I wanted to design a production-grade system where generative models are strictly constrained by deterministic validation and human-in-the-loop review, paired with transformer-based NLP pipelines for automated constituent sentiment analysis.',
    whatItDoes: [
      'Executes a five-stage bounded workflow: generate → AI critique → deterministic claim check → bounded revision → human review.',
      'Enforces deterministic claim checking: plain code verifies all numerical claims exist in the source brief before model revision.',
      'Isolates database transactions from external AI provider latency: workflow modules never receive raw database sessions.',
      'Computes granular resident sentiment distributions from survey datasets using fine-tuned RoBERTa transformer pipelines.',
      'Maintains immutable audit records tracking model draft vs human finalized copy for continuous drift evaluation.',
    ],
    highlights: [
      '5-stage bounded AI pipeline: generate → critique → deterministic check → revise → human review.',
      'Deterministic claim verification flagging numbers absent from the input brief.',
      'Zero open DB transactions across AI provider invocations.',
      'Separate immutable tracking of LLM proposals vs human operator revisions.',
    ],
    architectureLayers: [
      {
        name: 'Ingestion & NLP Intelligence',
        description:
          'Parses resident survey submissions and computes multi-class sentiment classifications using RoBERTa transformers.',
        components: ['FastAPI Asynchronous Ingestion', 'RoBERTa Sentiment Classifier', 'Batch Processing Pipeline'],
      },
      {
        name: 'Bounded Generation & Audit Gate',
        description:
          'Orchestrates structured message drafting with deterministic claim checks and human-in-the-loop state machines.',
        components: ['LangChain Orchestrator', 'Pydantic JSON Schemas', 'Deterministic Claim Verifier', 'Approval State Machine'],
      },
      {
        name: 'Persistence & Review Dashboard',
        description:
          'Persists campaign state, draft histories, and analytics in PostgreSQL with a responsive React management interface.',
        components: ['PostgreSQL 18 Engine', 'SQLAlchemy / Alembic', 'React 19 Dashboard', 'Railway Cloud Deployment'],
      },
    ],
    aiDecisions: [
      {
        title: 'Deterministic Claim Checking over LLM-as-a-Judge',
        rationale:
          'LLM judges can hallucinate compliance or miss factual contradictions. Plain Python regex and AST string extractors verify that every statistic in the draft originated in the verified brief.',
        tradeOff:
          'Requires structured input briefs with explicitly formatted numerical data.',
      },
      {
        title: 'Immutable Dual-Record Audit Log',
        rationale:
          'Saving both raw AI output and human edits allows computing exact human edit distance over time to measure model drift.',
        tradeOff:
          'Increases storage requirements linearly with message volume.',
      },
    ],
    systemDesignDecisions: [
      {
        title: 'Session-Free AI Provider Boundary',
        rationale:
          'Holding PostgreSQL connection pool sessions open during 2-5 second LLM streaming calls quickly exhausts database pools under concurrent load.',
        tradeOff:
          'Requires multi-phase state commits before and after AI provider invocations.',
      },
    ],
    challenges: [
      {
        title: 'Preventing Hallucinated Policy Commitments',
        problem:
          'Generative models often invent specific timeline commitments (e.g. "Road repairs completed in 2 weeks") when generating constituent responses.',
        solution:
          'Engineered strict negative constraint schemas and deterministic regex extractors that fail the validation gate if unauthorized timeline patterns are detected.',
        takeaway:
          'Semantic guardrails must be paired with deterministic syntactic assertions.',
      },
    ],
    metrics: [
      {
        label: 'Resident Ingestion',
        value: '10K+ rows',
        context: 'Asynchronously parsed and sentiment-classified per dataset.',
      },
      {
        label: 'Workflow Stages',
        value: '5 Bounded',
        context: 'Generate → Critique → Deterministic Check → Revise → Review.',
      },
      {
        label: 'Model Drift Visibility',
        value: '100%',
        context: 'Every human edit recorded alongside model proposal.',
      },
    ],
    improvements: [
      'Multi-tenant organization partitioning with role-based access control.',
      'Automated semantic clustering of constituent issues using vector embeddings.',
    ],
    technologies: ['Python', 'FastAPI', 'React 19', 'TypeScript', 'PostgreSQL 18', 'LangChain', 'RoBERTa', 'Railway'],
    codeSnippets: [
      {
        language: 'python',
        filename: 'civicpulse/guardrails/claim_check.py',
        caption: 'Deterministic numerical claim verifier preventing ungrounded numbers',
        code: `import re

def verify_numerical_claims(draft: str, source_brief: str) -> list[str]:
    # Extract all numerical tokens from draft and source
    draft_numbers = set(re.findall(r'\\b\\d+(?:\\.\\d+)?%?\\b', draft))
    brief_numbers = set(re.findall(r'\\b\\d+(?:\\.\\d+)?%?\\b', source_brief))
    
    unauthorized = draft_numbers - brief_numbers
    if unauthorized:
        return [f"Draft contains unverified numerical claims: {unauthorized}"]
    return []`,
      },
    ],
    github: 'https://github.com/harshvardhan579/civicpulse',
    live: 'https://frontend-production-d0ad.up.railway.app',
  },
  {
    slug: 'arcade-game',
    title: 'Pocket Arcade',
    subtitle: 'Five Original Browser Games with Zero Runtime Assets and Production Test Rigor',
    category: 'Frontend Engineering · Interactive Systems',
    index: '03',
    year: 2026,
    featured: true,
    summary:
      'A responsive browser arcade platform featuring five original games, zero external image/audio assets, custom Web Audio synthesis, and an automated visual regression test suite.',
    problem:
      'Web-based canvas games frequently suffer from massive asset load waterfalls, inconsistent frame pacing across mobile screens, and fragile state management when pausing, switching games, or handling inputs.',
    motivation:
      'I wanted to demonstrate high-level product engineering and game system architecture by building a multi-game engine where all graphics are procedurally rasterized and all sound effects are synthesized via Web Audio API at 60 FPS.',
    whatItDoes: [
      'Provides five distinct games (Cosmic Runner, Byte Breaker, Neon Orbit, Pixel Siege, Rogue Grid) rendered purely via canvas primitives.',
      'Implements a custom Web Audio synthesizer generating all jump, impact, laser, and game-over sound effects in real time.',
      'Maintains clean reactive state management isolating game physics loops from high-score and UI overlays.',
      'Includes an automated Vitest and Playwright visual regression suite ensuring cross-viewport rendering accuracy.',
    ],
    highlights: [
      '5 original arcade games rendered with 0 external raster or audio assets.',
      'Custom Web Audio synthesizer creating real-time dynamic sound waves.',
      'Rock-solid 60 FPS frame pacing on desktop and mobile viewports.',
      'Automated visual regression tests asserting canvas pixel fidelity.',
    ],
    architectureLayers: [
      {
        name: 'Game Engine & Physics Loop',
        description:
          'Fixed-timestep game loops utilizing Phaser 3 and HTML5 Canvas with collision detection and procedural drawing.',
        components: ['Phaser 3 Engine', 'Fixed Timestep Runner', 'Procedural Canvas Renderer'],
      },
      {
        name: 'Audio Synthesis Core',
        description:
          'Generates real-time audio waveforms (sine, square, sawtooth) with exponential frequency decays.',
        components: ['Web Audio API Context', 'Oscillator Synthesizer', 'ADSR Envelope Engine'],
      },
      {
        name: 'Application Shell & State',
        description:
          'React UI wrapping the canvas instance with high score persistence and responsive touch controls.',
        components: ['React / Vite', 'TypeScript', 'LocalStorage Persistence', 'Virtual Touch Controls'],
      },
    ],
    aiDecisions: [],
    systemDesignDecisions: [
      {
        title: 'Zero-Asset Procedural Graphics Architecture',
        rationale:
          'Eliminating external PNG/MP3 assets cuts initial bundle size to under 200KB and guarantees instant zero-latency loading on mobile networks.',
        tradeOff:
          'Requires writing custom procedural draw functions and particle systems for every visual effect.',
      },
    ],
    challenges: [
      {
        title: 'Audio Context Unlocking on Mobile Safari',
        problem:
          'Mobile Safari blocks Web Audio playback until explicit user interaction, causing muted gameplay if initialized improperly.',
        solution:
          'Engineered a centralized AudioContext manager with an explicit user gesture unlock handler attached to start button interactions.',
        takeaway:
          'Mobile browser media restrictions require deliberate initialization lifecycles.',
      },
    ],
    metrics: [
      {
        label: 'Runtime Asset Payload',
        value: '0 KB',
        context: '100% procedurally generated visuals and audio.',
      },
      {
        label: 'Target Frame Rate',
        value: '60 FPS',
        context: 'Steady frame pacing across desktop and mobile devices.',
      },
      {
        label: 'Games Built',
        value: '5 Original',
        context: 'Cosmic Runner, Byte Breaker, Neon Orbit, Pixel Siege, Rogue Grid.',
      },
    ],
    improvements: [
      'Global online multiplayer matchmaking via WebSocket rooms.',
      'Custom chiptune soundtrack generator utilizing algorithmic MIDI sequences.',
    ],
    technologies: ['TypeScript', 'Phaser 3', 'React', 'HTML5 Canvas', 'Web Audio API', 'Vite', 'Vercel'],
    codeSnippets: [
      {
        language: 'typescript',
        filename: 'arcade/audio/synth.ts',
        caption: 'Procedural Web Audio synthesizer generating dynamic sound effects',
        code: `export class SoundSynth {
  private ctx: AudioContext | null = null;

  playLaser() {
    const ctx = this.getContext();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    
    osc.type = 'sawtooth';
    osc.frequency.setValueAtTime(880, ctx.currentTime);
    osc.frequency.exponentialRampToValueAtTime(110, ctx.currentTime + 0.15);
    
    gain.gain.setValueAtTime(0.3, ctx.currentTime);
    gain.gain.linearRampToValueAtTime(0.01, ctx.currentTime + 0.15);
    
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.15);
  }
}`,
      },
    ],
    github: 'https://github.com/harshvardhan579/arcade-game',
    live: 'https://arcade-game-five.vercel.app/',
  },
  {
    slug: 'news-verify',
    title: 'NewsVerify',
    subtitle: 'Multimodal Claim Assessment Grounded in Retrieved Evidence',
    category: 'Multimodal AI · RAG · Factuality',
    index: '04',
    year: 2026,
    featured: true,
    summary:
      'An autonomous verification system that decomposes controversial multimedia claims into testable sub-claims, retrieves real-time grounded evidence, and generates transparent assessments with citations the model cannot fabricate.',
    problem:
      'Online misinformation spreads across multimodal formats (screenshots, social media posts, synthetic graphics). Standard LLMs hallucinate corroborating sources and lack real-time web retrieval grounding.',
    motivation:
      'I wanted to architect a multimodal pipeline where evidence retrieval is structurally separated from LLM reasoning, guaranteeing that every citation maps to a verified search result URL rather than an invented model artifact.',
    whatItDoes: [
      'Extracts textual claims from images using multimodal GPT-4o vision and PaddleOCR.',
      'Deconstructs complex claims into atomic verifiable assertions with targeted search queries.',
      'Retrieves authoritative web evidence via SerpApi and Google Search integrations.',
      'Cross-examines retrieved snippets against assertions to produce a verified truthfulness score with citations.',
    ],
    highlights: [
      'Multimodal claim extraction via GPT-4o vision + OCR pipeline.',
      'Atomic claim decomposition preventing bundled reasoning errors.',
      'Hard citation grounding: URLs strictly constrained to retrieved search results.',
      'Transparent verification audit trail with confidence intervals.',
    ],
    architectureLayers: [
      {
        name: 'Multimodal Extraction',
        description:
          'Extracts claims from images and screenshots using vision models and OCR.',
        components: ['GPT-4o Vision API', 'PaddleOCR', 'Claim Sanitization'],
      },
      {
        name: 'Retrieval & Decomposition',
        description:
          'Splits claims into atomic assertions and executes targeted search queries.',
        components: ['LangChain Decomposition', 'SerpApi Integration', 'Evidence Ranking'],
      },
      {
        name: 'Synthesis & Factuality Scoring',
        description:
          'Synthesizes retrieved evidence into structured factuality reports.',
        components: ['Pydantic Verification Schema', 'Streamlit UI', 'Source Citation Guard'],
      },
    ],
    aiDecisions: [
      {
        title: 'Atomic Claim Decomposition',
        rationale:
          'Evaluating complex multi-sentence claims as a single prompt produces vague verdicts. Decomposing into single-fact assertions ensures granular verification.',
        tradeOff:
          'Increases total search query volume and API latency.',
      },
    ],
    systemDesignDecisions: [
      {
        title: 'Constrained Citation Mapping',
        rationale:
          'LLMs frequently hallucinate convincing URLs. The output schema requires selecting citations by numerical index from the search result array.',
        tradeOff:
          'Limits citations strictly to domains captured in top search results.',
      },
    ],
    challenges: [
      {
        title: 'Resolving Ambiguous Search Snippets',
        problem:
          'Search snippets often contain conflicting opinions or out-of-context headlines.',
        solution:
          'Engineered a multi-source consensus scorer that flags disagreements between authoritative sources for human review.',
        takeaway:
          'Fact-checking systems must highlight uncertainty rather than forcing binary verdicts.',
      },
    ],
    metrics: [
      {
        label: 'Multimodal Extraction',
        value: '100%',
        context: 'Vision and OCR dual-extraction for image claims.',
      },
      {
        label: 'Source Grounding',
        value: 'Zero Hallucination',
        context: 'URLs constrained to verified search payloads.',
      },
    ],
    improvements: [
      'Fine-tuned NLI (Natural Language Inference) models for sub-100ms claim checking.',
      'Reverse image search integration for origin tracing of manipulated media.',
    ],
    technologies: ['Python', 'LangChain', 'GPT-4o', 'PaddleOCR', 'SerpApi', 'Streamlit'],
    codeSnippets: [],
    github: 'https://github.com/harshvardhan579/news_verify',
    live: '',
  },
  {
    slug: 'form-eval-app',
    title: 'AI Form Evaluator',
    subtitle: 'Real-Time Exercise Form Coaching with Zero Video Uploads',
    category: 'Edge Computer Vision · Real-Time Systems',
    index: '05',
    year: 2026,
    featured: true,
    summary:
      'A real-time exercise form evaluation pipeline that processes video entirely in-browser via MediaPipe, transmitting only lightweight numerical 3D pose landmarks over WebSockets to preserve user privacy.',
    problem:
      'AI fitness applications typically require uploading raw video streams to cloud servers, introducing unacceptable latency (300ms+), high bandwidth costs, and severe user privacy vulnerabilities.',
    motivation:
      'I wanted to design an edge-first computer vision architecture where raw video frames never leave the user device, while low-latency kinematic state machines evaluate exercise mechanics in real time.',
    whatItDoes: [
      'Extracts 33 skeletal keypoints in-browser at 30 FPS using MediaPipe WebAssembly.',
      'Calculates joint angles (knee, hip, elbow) using vector trigonometry in real time.',
      'Streams lightweight landmark coordinates to a FastAPI backend via WebSockets for state-machine rep counting.',
      'Provides instantaneous audio/visual coaching feedback with sub-50ms latency.',
    ],
    highlights: [
      'Edge-first privacy: raw video frames never leave the client browser.',
      'MediaPipe WebAssembly pose extraction running at 30 FPS locally.',
      'Landmark-only WebSocket stream reducing bandwidth by 99.8%.',
      'Sub-50ms kinematic analysis and state-machine rep tracking.',
    ],
    architectureLayers: [
      {
        name: 'In-Browser Vision Layer',
        description:
          'Runs MediaPipe pose extraction on local video feeds via WebAssembly.',
        components: ['MediaPipe WASM', 'HTML5 Video Stream', 'Kinematic Vector Math'],
      },
      {
        name: 'Real-Time Transport',
        description:
          'Transmits 33 numerical coordinates per frame over persistent WebSocket connections.',
        components: ['WebSocket Client', 'FastAPI WebSocket Server', 'JSON Serialization'],
      },
      {
        name: 'Kinematic Evaluation Engine',
        description:
          'Evaluates joint angle thresholds across exercise repetition state machines.',
        components: ['Repetition State Machine', 'Joint Angle Math', 'Coaching Logic'],
      },
    ],
    aiDecisions: [],
    systemDesignDecisions: [
      {
        title: 'Edge-Client Landmark Extraction',
        rationale:
          'Transmitting 33 landmark points (~1KB/s) instead of raw 1080p video streams (~3MB/s) guarantees privacy and cuts network bandwidth by over 99.8%.',
        tradeOff:
          'Requires client hardware capable of running lightweight MediaPipe WebAssembly models.',
      },
    ],
    challenges: [
      {
        title: 'Handling Variable Camera Perspectives',
        problem:
          'Users stand at diagonal angles to the camera, distorting 2D angle calculations.',
        solution:
          'Normalized 2D joint coordinates against shoulder-to-hip reference vectors to maintain accurate angle measurements across perspective shifts.',
        takeaway:
          'Kinematic models must normalize for camera perspective in edge vision pipelines.',
      },
    ],
    metrics: [
      {
        label: 'End-to-End Latency',
        value: '< 50 ms',
        context: 'From webcam capture to visual coaching feedback.',
      },
      {
        label: 'Bandwidth Reduction',
        value: '99.8%',
        context: '1 KB/s landmark stream vs 3 MB/s video upload.',
      },
      {
        label: 'Privacy Guarantee',
        value: 'Zero Video Upload',
        context: '100% of raw video processing occurs in local memory.',
      },
    ],
    improvements: [
      'Multi-person pose tracking for group workout sessions.',
      '3D skeleton mesh rendering using WebGL overlays.',
    ],
    technologies: ['Python', 'OpenCV', 'MediaPipe', 'FastAPI', 'WebSockets', 'React', 'Vercel'],
    codeSnippets: [],
    github: 'https://github.com/harshvardhan579/form_eval_app',
    live: 'https://form-eval-app.vercel.app',
  },
  {
    slug: 'hybrid-ml-scheduler',
    title: 'Hybrid ML Scheduler',
    subtitle: 'Six GPU/CPU Scheduling Strategies Raced Live Against an Optimal Baseline',
    category: 'Reinforcement Learning · Distributed Systems',
    index: '06',
    year: 2025,
    featured: true,
    summary:
      'A heterogeneous cluster scheduling simulator that trains and evaluates six scheduling strategies (including Dueling DQNs and Random Forests) live against a brute-force optimal Oracle baseline.',
    problem:
      'Heterogeneous compute clusters with mixed CPU/GPU workloads suffer from queue congestion, resource underutilization, and task deadline violations under dynamic arrival distributions.',
    motivation:
      'I wanted to explore the performance boundaries between traditional heuristics (FIFO, SJF, Priority) and reinforcement learning models in distributed task placement.',
    whatItDoes: [
      'Simulates a cluster of heterogeneous compute nodes with distinct CPU, GPU, memory, and bandwidth profiles.',
      'Implements six scheduling policies: FIFO, Shortest Job First, Priority Queue, Random Forest, Dueling DQN, and Brute-Force Oracle.',
      'Computes schedule optimality ratios, average job latencies, and resource utilization in real time.',
      'Streams live simulation metrics over WebSockets to an interactive real-time dashboard.',
    ],
    highlights: [
      '6 scheduling policies evaluated simultaneously against a brute-force Oracle.',
      'Dueling Deep Q-Network (DQN) trained on workload queue states.',
      'Real-time WebSocket telemetry of cluster utilization and queue depths.',
      'Comprehensive benchmark comparing learned models vs classic heuristics.',
    ],
    architectureLayers: [
      {
        name: 'Cluster Simulator Core',
        description:
          'Simulates heterogeneous compute nodes, job queues, and hardware constraints.',
        components: ['Cluster Environment Engine', 'Task Arrival Generator', 'Resource Allocator'],
      },
      {
        name: 'Scheduling Algorithm Suite',
        description:
          'Hosts heuristic, supervised, reinforcement learning, and optimal Oracle policies.',
        components: ['Dueling DQN Model', 'Random Forest Regressor', 'FIFO/SJF Heuristics', 'Brute-Force Oracle'],
      },
      {
        name: 'Telemetry & Visualization',
        description:
          'Streams real-time simulation metrics to frontend dashboards.',
        components: ['FastAPI Backend', 'WebSocket Streamer', 'Real-Time Charting Dashboard'],
      },
    ],
    aiDecisions: [
      {
        title: 'Dueling DQN Architecture for Action-Advantage Decoupling',
        rationale:
          'Dueling networks evaluate the value of being in a queue state separately from the advantage of placing a specific job on a node, accelerating RL convergence in sparse-reward environments.',
        tradeOff:
          'Requires larger memory footprint during training replay buffers.',
      },
    ],
    systemDesignDecisions: [
      {
        title: 'Brute-Force Oracle Baseline for True Ground-Truth Comparison',
        rationale:
          'Standard benchmarks compare RL only against simple heuristics (FIFO). Implementing an exhaustive offline Oracle reveals the true theoretical upper bound of scheduling efficiency.',
        tradeOff:
          'Oracle computation scales exponentially with queue depth and is capped at small batch sizes.',
      },
    ],
    challenges: [
      {
        title: 'Preventing Cluster Node Starvation Under Burst Traffic',
        problem:
          'Greedy RL policies frequently over-allocated GPU-heavy tasks, starving CPU-bound background jobs.',
        solution:
          'Introduced an aging reward penalty in the RL loss function that penalizes task wait times exponentially.',
        takeaway:
          'RL reward functions must model fairness alongside throughput maximization.',
      },
    ],
    metrics: [
      {
        label: 'Policies Evaluated',
        value: '6 Strategies',
        context: 'FIFO, SJF, Priority, Random Forest, DQN, and Oracle.',
      },
      {
        label: 'Throughput Improvement',
        value: '+28%',
        context: 'DQN throughput gain over standard FIFO heuristics under heavy load.',
      },
    ],
    improvements: [
      'Multi-agent RL for distributed decentralized schedulers.',
      'Kubernetes custom metrics adapter for live production pod scheduling.',
    ],
    technologies: ['Python', 'PyTorch', 'FastAPI', 'Redis', 'WebSockets', 'NumPy', 'Docker'],
    codeSnippets: [],
    github: 'https://github.com/harshvardhan579/hybrid-ml-scheduler',
    live: '',
  },
];

export function getProjectBySlug(slug: string): Project | undefined {
  return projectsData.find((p) => p.slug === slug);
}

export function getAllProjectSlugs(): string[] {
  return projectsData.map((p) => p.slug);
}
