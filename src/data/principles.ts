import { EngineeringPrinciple } from '@/types/portfolio';

export const principlesData: EngineeringPrinciple[] = [
  {
    number: '01',
    title: 'Models need guardrails',
    statement: 'Surround probabilistic AI with strict deterministic software contracts.',
    elaboration:
      'Large language models are powerful reasoning engines but inherently non-deterministic. Production systems must constrain model outputs using strict schema validation, state machines, and bounded retry loops rather than hoping for well-formed text.',
  },
  {
    number: '02',
    title: 'Evals before vibes',
    statement: 'Quantify model behavior and regressions with repeatable evaluation suites.',
    elaboration:
      'Impressive single-turn demos do not equal reliable software. Every prompt or model upgrade requires automated regression benchmarks, calibrated rubrics, and continuous tracking of hallucination rates, latency, and token unit costs.',
  },
  {
    number: '03',
    title: 'Reliability is a feature',
    statement: 'Engineer for the inevitable failure paths: retries, timeouts, and state isolation.',
    elaboration:
      'Distributed services and external AI APIs will experience rate limits, transient drops, and malformed payloads. Architecting with idempotent endpoints, async job queues, and isolated mock harnesses makes systems resilient under pressure.',
  },
  {
    number: '04',
    title: 'The interface is part of the system',
    statement: 'Complex technical architectures deserve thoughtful, responsive interfaces.',
    elaboration:
      'Deep backend and AI engineering only delivers value when users and operators can intuitively monitor, inspect, and guide the system. Latency feedback, streaming states, and clear visual hierarchy turn complex pipelines into joyful tools.',
  },
];
