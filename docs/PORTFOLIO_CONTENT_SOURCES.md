# Portfolio Content Sources & Provenance Record

This document records the canonical repositories, live deployments, and source READMEs for all projects featured on Harshvardhan Singh's engineering portfolio.

**Last Sync Date:** August 2026  
**Canonical GitHub Profile:** [https://github.com/harshvardhan579](https://github.com/harshvardhan579)

---

## 1. APILoom (Flagship Project 01)
- **Field:** AI Infrastructure / Agentic Workflows / API Systems
- **Canonical Repository:** [https://github.com/harshvardhan579/apiloom](https://github.com/harshvardhan579/apiloom)
- **Live Deployment:** Local-first developer runtime (self-hosted / loopback)
- **Core Concept:** Plain-English REST procedures become reviewed, reusable runbooks with a full audit trail.
- **Architectural Tenet:** Separation of probabilistic planning (OpenAI structured outputs into Pydantic workflow schema) from deterministic execution. One-use mutation preview token bound to plan revision, input digest, and connection versions. SSRF-aware destination checks, non-interpolable authorities, and honest `UNKNOWN_OUTCOME` failure semantics.

---

## 2. CivicPulse (Flagship Project 02)
- **Field:** Applied AI / Bounded Workflows / Survey Intelligence
- **Canonical Repository:** [https://github.com/harshvardhan579/civicpulse](https://github.com/harshvardhan579/civicpulse)
- **Live Deployment:** [https://frontend-production-d0ad.up.railway.app](https://frontend-production-d0ad.up.railway.app/)
- **Core Concept:** A bounded, human-reviewed AI workflow where a human correction never overwrites the model output.
- **Architectural Tenet:** Five-stage workflow: generate → AI critique → deterministic claim check → bounded revision → human review. Workflow module never receives a database session, preventing transactions across model provider invocations.

---

## 3. Pocket Arcade (Flagship Project 03)
- **Field:** Frontend Product Engineering / Interactive Systems / Canvas
- **Canonical Repository:** [https://github.com/harshvardhan579/arcade-game](https://github.com/harshvardhan579/arcade-game)
- **Live Deployment:** [https://arcade-game-five.vercel.app/](https://arcade-game-five.vercel.app/)
- **Core Concept:** Five original browser games, zero runtime assets, production-grade test suite.
- **Architectural Tenet:** Modular state management, custom Web Audio synthesis, procedural raster rendering, and automated regression testing.

---

## 4. NewsVerify (Flagship Project 04)
- **Field:** Multimodal AI / RAG / Factuality Harnesses
- **Canonical Repository:** [https://github.com/harshvardhan579/news_verify](https://github.com/harshvardhan579/news_verify)
- **Live Deployment:** Streamlit / Python
- **Core Concept:** Multimodal claim assessment grounded in retrieved evidence the model cannot invent.
- **Architectural Tenet:** GPT-4o vision + PaddleOCR for multimodal decompose; SerpApi for grounding; strict schema validation prevents hallucinated reference citations.

---

## 5. AI Form Evaluator (Flagship Project 05)
- **Field:** Edge Computer Vision / Pose Estimation / Real-Time Systems
- **Canonical Repository:** [https://github.com/harshvardhan579/form_eval_app](https://github.com/harshvardhan579/form_eval_app)
- **Live Deployment:** [https://form-eval-app.vercel.app](https://form-eval-app.vercel.app/)
- **Core Concept:** Real-time exercise-form coaching from pose landmarks — raw video never leaves the browser.
- **Architectural Tenet:** In-browser MediaPipe pose extraction; landmark-only WebSocket transport to FastAPI backend; state-machine-based kinematic joint analysis.

---

## 6. Hybrid ML Scheduler (Flagship Project 06)
- **Field:** Reinforcement Learning / Distributed Systems / Scheduling
- **Canonical Repository:** [https://github.com/harshvardhan579/hybrid-ml-scheduler](https://github.com/harshvardhan579/hybrid-ml-scheduler)
- **Live Deployment:** Python / FastAPI / WebSocket telemetry
- **Core Concept:** Six GPU/CPU scheduling strategies raced live against a brute-force optimal baseline.
- **Architectural Tenet:** Dueling Deep Q-Networks (DQN) + Random Forests compared directly against an Oracle scheduler across heterogeneous compute clusters.
