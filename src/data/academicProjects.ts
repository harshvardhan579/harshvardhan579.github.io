import { AcademicProject } from '@/types/portfolio';
export type { AcademicProject };

export const academicProjectsData: AcademicProject[] = [
  // 2025 Projects (RIT)
  {
    slug: 'image-similarity-convnext-arcface',
    title: 'Image Similarity Learning with ConvNeXt and ArcFace',
    year: 2025,
    period: 'Mar 2025 — Apr 2025',
    institution: 'Rochester Institute of Technology',
    category: 'Computer Vision / Deep Learning',
    github: 'https://github.com/harshvardhan579/ml-text-vision-projects',
    summary:
      'Built an image-similarity learning pipeline using ConvNeXt and ArcFace embeddings on Tiny ImageNet-200, experimenting with angular margins and cosine-similarity-based retrieval and classification.',
    highlights: [
      'Custom PyTorch dataset on Tiny ImageNet-200 with mixed-precision training',
      'Pretrained ConvNeXt backbone generating 512-dimensional normalized embeddings',
      'ArcFace loss function with additive angular margin penalty optimizing hyperspherical inter-class separability',
      'Cosine similarity retrieval benchmarks and margin experiments evaluated on classification accuracy',
    ],
    technologies: ['PyTorch', 'Torchvision', 'ConvNeXt', 'ArcFace', 'Hugging Face Datasets', 'Python'],
  },
  {
    slug: 'bert-yelp-review-curriculum',
    title: 'BERT-based Yelp Review Rating Prediction with Curriculum Learning',
    year: 2025,
    period: 'Feb 2025 — Mar 2025',
    institution: 'Rochester Institute of Technology',
    category: 'NLP / Machine Learning',
    github: 'https://github.com/harshvardhan579/ml-text-vision-projects',
    summary:
      'Studied curriculum learning for Yelp review rating prediction by ordering training examples using reading difficulty and comparing fine-tuned BERT models against classical TF-IDF baselines.',
    highlights: [
      'Automated easy-to-hard training curriculum based on Flesch Reading Ease readability scores',
      'Fine-tuned BERT-base encoder across 1MB, 10MB, and 100MB training subsets',
      'Final encoder-layer fine-tuning benchmarked against TF-IDF + Logistic Regression baselines',
      'Comprehensive multi-class evaluation: accuracy, precision, recall, macro F1, and confusion matrices',
    ],
    technologies: ['Python', 'PyTorch', 'Hugging Face Transformers', 'Pandas', 'scikit-learn', 'TextStat'],
  },
  {
    slug: 'spam-classification-naive-bayes',
    title: 'Spam Classification Using Bag-of-Words and Naïve Bayes',
    year: 2025,
    period: 'Jan 2025 — Feb 2025',
    institution: 'Rochester Institute of Technology',
    category: 'NLP / Machine Learning',
    github: 'https://github.com/harshvardhan579/ml-text-vision-projects',
    summary:
      'Implemented a Bag-of-Words and Naïve Bayes spam classifier from scratch and benchmarked it against scikit-learn’s MultinomialNB on SMS and email corpora.',
    highlights: [
      'Built Bag-of-Words vectorizer, tokenization pipeline, and vocabulary constructor from scratch',
      'Implemented Multinomial Naïve Bayes algorithm with configurable Laplace smoothing parameter',
      'Benchmarked scratch implementation against scikit-learn MultinomialNB on SMS Spam Collection and Kaggle datasets',
      'Evaluated accuracy, precision, recall, F1 score, and classification confusion matrices',
    ],
    technologies: ['Python', 'NumPy', 'Pandas', 'scikit-learn'],
  },

  // 2024 Projects (Penn State)
  {
    slug: 'database-system-development',
    title: 'Database System Development',
    year: 2024,
    period: 'Jan 2024 — Apr 2024',
    institution: 'Penn State University',
    category: 'Databases / Backend',
    summary:
      'Designed a normalized PostgreSQL database system with 15+ entity relationships and built a Python CLI for transactional data management.',
    highlights: [
      '3NF relational schema with 15+ entities, B-tree/hash indexes, triggers, and stored procedures',
      'Complex analytical SQL queries using CTEs, window functions, and multi-table joins on 10K+ records',
      'Python CLI with psycopg2 connection pooling, prepared statements, and parameterized queries',
      'Strict transaction isolation levels, rollback integrity, and pytest execution plan verification',
    ],
    technologies: ['PostgreSQL', 'Python', 'psycopg2', 'pytest', 'SQL'],
  },

  // 2023 Projects (Penn State)
  {
    slug: 'advanced-data-analytics-suite',
    title: 'Advanced Data Analytics and Predictive Modeling Suite',
    year: 2023,
    period: 'Jan 2023 — May 2023',
    institution: 'Penn State University',
    category: 'Machine Learning / Data Science',
    summary:
      'Implemented and analyzed classical machine-learning techniques including k-NN, k-means++, and PCA across standard healthcare and Iris datasets.',
    highlights: [
      'Pima Indians Diabetes classification using distance-weighted k-NN and cross-validation',
      'Unsupervised clustering with k-means++ centroid initialization on Iris flower morphology',
      'Dimensionality reduction and feature variance explanation using Principal Component Analysis (PCA)',
      'Data cleansing, feature scaling, correlation analysis, and ROC-AUC curve visualization',
    ],
    technologies: ['Python', 'Pandas', 'scikit-learn', 'Jupyter', 'NumPy'],
  },

  // 2022 Projects (Penn State)
  {
    slug: 'custom-os-simulation',
    title: 'Custom Operating Systems Simulation and Optimization',
    year: 2022,
    period: 'Aug 2022 — Dec 2022',
    institution: 'Penn State University',
    category: 'Operating Systems / Systems Programming',
    summary:
      'Implemented core operating-system mechanisms spanning CPU scheduling, thread synchronization, memory allocation, and virtual memory.',
    highlights: [
      'Preemptive CPU scheduling algorithms (Multi-Level Feedback Queue, Round Robin, Priority, SJF)',
      'Process synchronization using POSIX threads, semaphores, mutexes, and condition variables',
      'Custom Buddy allocator and Slab allocator for dynamic kernel memory management without fragmentation',
      'Virtual memory paging simulator with LRU, FIFO, and Clock page-replacement policies',
    ],
    technologies: ['C', 'Linux', 'POSIX Threads', 'Systems Programming', 'GDB'],
  },
  {
    slug: 'comprehensive-blockchain-infrastructure',
    title: 'Comprehensive Blockchain Infrastructure Development',
    year: 2022,
    period: 'Jan 2022 — May 2022',
    institution: 'Penn State University',
    category: 'Blockchain / Web',
    summary:
      'Built blockchain-oriented server/client applications and experimented with Ethereum smart contracts and decentralized storage workflows.',
    highlights: [
      'Node / Express server-client architecture for transaction broadcasting and block verification',
      'Interactive React SVG visualizer rendering live blockchain ledger states and merkle roots',
      'Solidity smart contracts for escrow and payments compiled and tested via Remix',
      'Decentralized storage integration exploring IPFS-based content-addressed persistence',
    ],
    technologies: ['Node.js', 'Express', 'React', 'Solidity', 'Ethereum', 'JavaScript'],
  },
  {
    slug: 'x86-file-system',
    title: 'File System on x86 Architecture',
    year: 2022,
    period: 'Jan 2022 — May 2022',
    institution: 'Penn State University',
    category: 'Systems / File Systems',
    summary:
      'Designed a C file-system implementation with caching, mount/read/write operations, and later extended it through a client-server interface.',
    highlights: [
      'Hierarchical disk file system with superblock, inode table, and data bitmap structures',
      'LRU block cache reducing disk I/O operations for frequent read and directory traversals',
      'Mount, unmount, directory navigation, file creation, and byte-level read/write primitives',
      'Client-server RPC extension allowing remote file operations over Unix network sockets',
    ],
    technologies: ['C', 'Linux', 'Unix Sockets', 'Systems Programming'],
  },

  // 2021 Projects (Penn State)
  {
    slug: 'pipelined-mips-processor',
    title: 'Pipelined Processor with Branch Prediction and Caching',
    year: 2021,
    period: 'Aug 2021 — Dec 2021',
    institution: 'Penn State University',
    category: 'Computer Architecture / Hardware',
    summary:
      'Designed a pipelined MIPS-style processor in Verilog with forwarding, branch prediction, caching, and support for core arithmetic/logic instructions.',
    highlights: [
      '5-stage pipelined datapath (IF, ID, EX, MEM, WB) modeled and simulated in Verilog',
      'Hazard detection unit with data forwarding and load-use stall insertion logic',
      '2-bit dynamic branch prediction unit minimizing pipeline flush penalties',
      'Direct-mapped instruction and data caches simulated and verified in Xilinx Vivado',
    ],
    technologies: ['Verilog', 'MIPS Assembly', 'Xilinx Vivado', 'Digital Logic'],
  },
  {
    slug: 'room-scheduler-java',
    title: 'Room Scheduler',
    year: 2021,
    period: 'Jun 2021 — Aug 2021',
    institution: 'Penn State University',
    category: 'Java / Database Application',
    github: 'https://github.com/harshvardhan579/RoomSchedulerJava',
    summary:
      'Built a Java scheduling application for allocating college rooms across faculty requests, reservations, dates, and waitlists.',
    highlights: [
      'Java GUI application structured with MVC architecture and modular DAO layer',
      'Relational database integration with Apache Derby for room, faculty, date, and reservation tables',
      'Automated room allocation algorithm handling capacity constraints, room amenities, and priority waitlists',
      'Conflict resolution and cancellation workflows with immediate waitlist promotion',
    ],
    technologies: ['Java', 'Apache Derby', 'SQL', 'Swing / AWT', 'JDBC'],
  },
  {
    slug: 'access-wheel-app',
    title: 'Access Wheel',
    year: 2021,
    period: 'May 2021 — Jun 2021',
    category: 'Accessibility / Full Stack',
    summary:
      'Built a web application for discovering wheelchair-accessibility information around locations using mapping and places APIs.',
    highlights: [
      'Interactive map interface displaying wheelchair-accessible venues, ramps, and entrances',
      'Integration with Google Places API and Mapbox for geocoding, address autocomplete, and map rendering',
      'User submission pipeline for reporting updated accessibility conditions and barrier descriptions',
      'Serverless backend on Node.js with CockroachDB distributed relational storage',
    ],
    technologies: ['React', 'Node.js', 'CockroachDB', 'Google Places API', 'Mapbox'],
  },

  // 2020 Projects
  {
    slug: 'healthcare-transparency-pricing',
    title: 'Healthcare Transparency and Pricing Web Platform',
    year: 2020,
    period: 'Aug 2020 — Dec 2020',
    category: 'Full Stack / Civic Healthcare',
    summary:
      'Created a healthcare pricing transparency application where users could report and discover healthcare pricing information through an interactive map-driven interface.',
    highlights: [
      'Crowdsourced and public healthcare procedure cost comparison across regional medical providers',
      'Google Maps API integration for location-based hospital and clinic search with cost pins',
      'Firebase Firestore real-time database for user-submitted procedure pricing data',
      'Client-side search and filtering by procedure code (CPT), insurance provider, and distance',
    ],
    technologies: ['JavaScript', 'HTML5', 'CSS3', 'Firebase', 'Google Maps API'],
  },
];

export const academicYears = [2025, 2024, 2023, 2022, 2021, 2020] as const;

export function getAcademicProjectsByYear(year: number): AcademicProject[] {
  return academicProjectsData.filter((p) => p.year === year);
}
