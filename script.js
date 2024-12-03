// Scene setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
camera.position.z = 20;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(300, 300); // Initial size; will adjust on window resize
const animationSection = document.querySelector('.animation-section');
animationSection.insertBefore(renderer.domElement, animationSection.firstChild);

// Lighting
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(10, 10, 10).normalize();
scene.add(light);

// Variables to hold objects
let obstacles = [];
let robots = [];
let awakeRobots = []; // Array to hold awake robots
let isAwakening = false;

// Variables to hold the initial state
let initialObstacles = [];
let initialRobots = [];

// Interaction mode
let mode = 'obstacle'; // 'obstacle', 'robot', or null

// Buttons
const addObstacleBtn = document.getElementById('addObstacleBtn');
const addRobotBtn = document.getElementById('addRobotBtn');
const startBtn = document.getElementById('startBtn');
const resetAnimationBtn = document.getElementById('resetAnimationBtn');
const resetBtn = document.getElementById('resetBtn');

// Algorithm selection
const algorithmSelect = document.getElementById('algorithmSelect');

// **Steps counter variable**
let steps = 0; // Initialize steps counter

// **Get the steps display element**
const stepsDisplay = document.getElementById('stepsDisplay');

// Grid dimensions
const GRID_SIZE = 40; // Define the size of the grid (increase for better resolution)
const grid = Array.from({ length: GRID_SIZE }, () => Array(GRID_SIZE).fill(0));

// Function to convert world coordinates to grid coordinates
function worldToGrid(position) {
    const x = Math.round(position.x + GRID_SIZE / 2 - 0.5);
    const y = Math.round(position.y + GRID_SIZE / 2 - 0.5);
    return { x, y };
}

// Function to convert grid coordinates to world coordinates
function gridToWorld(gridPos) {
    const x = gridPos.x - GRID_SIZE / 2 + 0.5;
    const y = gridPos.y - GRID_SIZE / 2 + 0.5;
    return new THREE.Vector3(x, y, 0);
}

// A* Pathfinding Implementation
class AStar {
    constructor(grid) {
        this.grid = grid;
    }

    heuristic(pos0, pos1) {
        // Euclidean distance
        const dx = pos1.x - pos0.x;
        const dy = pos1.y - pos0.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    findPath(start, goal) {
        const openList = [];
        const closedList = new Set();

        const startNode = {
            position: start,
            g: 0,
            h: this.heuristic(start, goal),
            f: 0,
            parent: null
        };
        startNode.f = startNode.g + startNode.h;
        openList.push(startNode);

        while (openList.length > 0) {
            // Sort the open list by f value
            openList.sort((a, b) => a.f - b.f);
            const currentNode = openList.shift();
            closedList.add(`${currentNode.position.x},${currentNode.position.y}`);

            // Check if we reached the goal
            if (currentNode.position.x === goal.x && currentNode.position.y === goal.y) {
                // Reconstruct path
                const path = [];
                let curr = currentNode;
                while (curr) {
                    path.push({ x: curr.position.x, y: curr.position.y });
                    curr = curr.parent;
                }
                return path.reverse();
            }

            // Get neighbors (including diagonals)
            const neighbors = [
                { x: currentNode.position.x + 1, y: currentNode.position.y },
                { x: currentNode.position.x - 1, y: currentNode.position.y },
                { x: currentNode.position.x, y: currentNode.position.y + 1 },
                { x: currentNode.position.x, y: currentNode.position.y - 1 },
                { x: currentNode.position.x + 1, y: currentNode.position.y + 1 },
                { x: currentNode.position.x - 1, y: currentNode.position.y - 1 },
                { x: currentNode.position.x + 1, y: currentNode.position.y - 1 },
                { x: currentNode.position.x - 1, y: currentNode.position.y + 1 },
            ];

            for (const neighborPos of neighbors) {
                const neighborKey = `${neighborPos.x},${neighborPos.y}`;

                // Check bounds and obstacles
                if (
                    neighborPos.x < 0 ||
                    neighborPos.x >= this.grid.length ||
                    neighborPos.y < 0 ||
                    neighborPos.y >= this.grid[0].length ||
                    this.grid[neighborPos.x][neighborPos.y] === 1 ||
                    closedList.has(neighborKey)
                ) {
                    continue;
                }

                // Determine movement cost
                const dx = neighborPos.x - currentNode.position.x;
                const dy = neighborPos.y - currentNode.position.y;
                const isDiagonal = dx !== 0 && dy !== 0;
                const movementCost = isDiagonal ? Math.SQRT2 : 1;

                const gScore = currentNode.g + movementCost;
                const hScore = this.heuristic(neighborPos, goal);
                const neighborNode = {
                    position: neighborPos,
                    g: gScore,
                    h: hScore,
                    f: gScore + hScore,
                    parent: currentNode
                };

                const existingNode = openList.find(
                    (node) =>
                        node.position.x === neighborPos.x && node.position.y === neighborPos.y
                );
                if (existingNode) {
                    if (gScore < existingNode.g) {
                        existingNode.g = gScore;
                        existingNode.f = gScore + existingNode.h;
                        existingNode.parent = currentNode;
                    }
                } else {
                    openList.push(neighborNode);
                }
            }
        }

        // No path found
        return null;
    }
}

// Initialize A* instance
const aStar = new AStar(grid);

// Set to keep track of targeted robots
let targetedRobots = new Set();

// Add Obstacles Button Event
addObstacleBtn.addEventListener('click', () => {
    if (mode === 'obstacle') {
        mode = null;
        addObstacleBtn.classList.remove('active');
    } else {
        mode = 'obstacle';
        addObstacleBtn.classList.add('active');
        addRobotBtn.classList.remove('active');
    }
});

// Add Robots Button Event
addRobotBtn.addEventListener('click', () => {
    if (mode === 'robot') {
        mode = null;
        addRobotBtn.classList.remove('active');
    } else {
        mode = 'robot';
        addRobotBtn.classList.add('active');
        addObstacleBtn.classList.remove('active');
    }
});

// Start Awakening Button Event
startBtn.addEventListener('click', () => {
    if (!isAwakening && selectedRobot) {
        steps = 0; // Reset steps counter when starting a new awakening
        stepsDisplay.textContent = steps;

        isAwakening = true;
        // Save the initial state before starting the awakening
        saveInitialState();

        // Find the robot object in the robots array corresponding to the selectedRobot mesh
        const startingRobot = robots.find((r) => r.mesh === selectedRobot);
        if (startingRobot) {
            startingRobot.isAwake = true;
            startingRobot.mesh.material.color.set(0x00ff00); // Change color to indicate "awake"
            awakeRobots.push(startingRobot);
        }
        selectedRobot.material.emissive.setHex(0x000000); // Remove highlight
        selectedRobot = null;
    }
});

// Reset Animation Button Event
resetAnimationBtn.addEventListener('click', () => {
    if (isAwakening || awakeRobots.length > 0) {
        resetToInitialState();
    }
});

// **Reset All Button Event**
resetBtn.addEventListener('click', () => {
    resetScene();
    steps = 0; // **Reset steps counter when resetting all**
    stepsDisplay.textContent = steps; // **Update steps display**
});

// Function to save the initial state
function saveInitialState() {
    // Deep copy obstacles
    initialObstacles = obstacles.map((obs) => {
        return {
            position: obs.position.clone(),
        };
    });

    // Deep copy robots
    initialRobots = robots.map((robot) => {
        return {
            position: robot.mesh.position.clone(),
        };
    });
}

// Function to save the initial state
function saveInitialState() {
    // Deep copy obstacles
    initialObstacles = obstacles.map((obs) => {
        return {
            position: obs.position.clone(),
        };
    });

    // Deep copy robots
    initialRobots = robots.map((robot) => {
        return {
            position: robot.mesh.position.clone(),
        };
    });
}

// Function to reset to the initial state
function resetToInitialState() {
    // Remove all obstacles from scene
    obstacles.forEach(obstacle => {
        scene.remove(obstacle);
    });
    obstacles = [];

    // Remove all robots from scene
    robots.forEach(robotObj => {
        scene.remove(robotObj.mesh);
    });
    robots = [];

    // Clear awakeRobots array
    awakeRobots = [];

    // Reset grid
    for (let x = 0; x < GRID_SIZE; x++) {
        for (let y = 0; y < GRID_SIZE; y++) {
            grid[x][y] = 0;
        }
    }

    // Reset clusters
    clusters = [];
    clustersInitialized = false;

    // Reset targetedRobots set
    targetedRobots = new Set();

    // Reset flags and selections
    isAwakening = false;

    // Reset selected robot
    if (selectedRobot) {
        selectedRobot.material.emissive.setHex(0x000000);
        selectedRobot = null;
    }

    // Reset interaction mode
    mode = null;
    addObstacleBtn.classList.remove('active');
    addRobotBtn.classList.remove('active');

    // Restore obstacles
    initialObstacles.forEach((obsData) => {
        addObstacle(obsData.position.x, obsData.position.y);
    });

    // Restore robots
    initialRobots.forEach((robotData) => {
        addRobot(robotData.position.x, robotData.position.y);
    });

    // Reset steps counter
    steps = 0;
    stepsDisplay.textContent = steps;

    // Re-initialize A* grid
    aStar.grid = grid;
}

// Raycaster for mouse interaction
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let selectedRobot = null;

// Event listener for clicks
renderer.domElement.addEventListener('click', onClick, false);

function onClick(event) {
    event.preventDefault();

    // Calculate mouse position in normalized device coordinates (-1 to +1)
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / renderer.domElement.clientWidth) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / renderer.domElement.clientHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    if (mode === 'obstacle' || mode === 'robot') {
        // Add an obstacle or robot at the clicked position
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const intersectionPoint = new THREE.Vector3();
        raycaster.ray.intersectPlane(plane, intersectionPoint);
        if (intersectionPoint) {
            if (mode === 'obstacle') {
                addObstacle(intersectionPoint.x, intersectionPoint.y);
            } else if (mode === 'robot') {
                addRobot(intersectionPoint.x, intersectionPoint.y);
            }
        }
    } else if (!isAwakening) {
        // Select a robot to start awakening
        const intersects = raycaster.intersectObjects(robots.map(r => r.mesh));
        if (intersects.length > 0) {
            if (selectedRobot) {
                selectedRobot.material.emissive.setHex(0x000000); // Deselect previous
            }
            selectedRobot = intersects[0].object;
            selectedRobot.material.emissive.setHex(0xffff00); // Highlight selected robot
        }
    }
}

// Functions to add obstacles and robots
function addObstacle(x, y) {
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshStandardMaterial({ color: 0x8b0000 });
    const obstacle = new THREE.Mesh(geometry, material);
    obstacle.position.set(x, y, 0);
    obstacles.push(obstacle);
    scene.add(obstacle);

    // Update grid
    const gridPos = worldToGrid(obstacle.position);
    if (gridPos.x >= 0 && gridPos.x < GRID_SIZE && gridPos.y >= 0 && gridPos.y < GRID_SIZE) {
        grid[gridPos.x][gridPos.y] = 1;
    }
}

function addRobot(x, y) {
    const geometry = new THREE.SphereGeometry(0.5, 32, 32);
    const material = new THREE.MeshStandardMaterial({ color: 0x0000ff });
    const robotMesh = new THREE.Mesh(geometry, material);
    robotMesh.position.set(x, y, 0);

    const robot = {
        mesh: robotMesh,
        isAwake: false,
        target: null,
        path: null,
        pathIndex: 0,
        cluster: null
    };

    robots.push(robot);
    scene.add(robotMesh);
}

// Function to reset the scene
function resetScene() {
    // Remove all obstacles from scene
    obstacles.forEach(obstacle => {
        scene.remove(obstacle);
    });
    obstacles = [];

    // Remove all robots from scene
    robots.forEach(robotObj => {
        scene.remove(robotObj.mesh);
    });
    robots = [];

    // Clear awakeRobots array
    awakeRobots = [];

    // Reset grid
    for (let x = 0; x < GRID_SIZE; x++) {
        for (let y = 0; y < GRID_SIZE; y++) {
            grid[x][y] = 0;
        }
    }

    // Reset clusters
    clusters = [];
    clustersInitialized = false;

    // Reset targetedRobots set
    targetedRobots = new Set();

    // Reset flags and selections
    isAwakening = false;

    if (selectedRobot) {
        selectedRobot.material.emissive.setHex(0x000000);
        selectedRobot = null;
    }

    // Reset interaction mode
    mode = null;
    addObstacleBtn.classList.remove('active');
    addRobotBtn.classList.remove('active');

    // Clear initial state
    initialObstacles = [];
    initialRobots = [];
}

// Assign paths using Greedy Static
function assignPathsGreedyStatic() {
    awakeRobots.forEach((robotObj) => {
        if (!robotObj.target && robots.some((r) => !r.isAwake)) {
            const asleepRobots = robots.filter((r) => !r.isAwake && !targetedRobots.has(r));
            let minDistance = Infinity;
            let closestRobot = null;
            let bestPath = null;

            asleepRobots.forEach((r) => {
                const startGridPos = worldToGrid(robotObj.mesh.position);
                const endGridPos = worldToGrid(r.mesh.position);
                const path = aStar.findPath(startGridPos, endGridPos);

                if (path && path.length < minDistance) {
                    minDistance = path.length;
                    closestRobot = r;
                    bestPath = path;
                }
            });

            if (closestRobot && bestPath) {
                robotObj.target = closestRobot;
                robotObj.path = bestPath.map((p) => gridToWorld(p));
                robotObj.pathIndex = 0;
                targetedRobots.add(closestRobot);
            }
        }
    });
}

// Assign paths using Greedy Dynamic
function assignPathsGreedyDynamic() {
    awakeRobots.forEach((robotObj) => {
        if (robots.some((r) => !r.isAwake)) {
            const asleepRobots = robots.filter((r) => !r.isAwake && !targetedRobots.has(r));
            if (asleepRobots.length === 0) {
                return;
            }
            let minDistance = Infinity;
            let closestRobot = null;
            let bestPath = null;

            asleepRobots.forEach((r) => {
                const startGridPos = worldToGrid(robotObj.mesh.position);
                const endGridPos = worldToGrid(r.mesh.position);
                const path = aStar.findPath(startGridPos, endGridPos);

                if (path && path.length < minDistance) {
                    minDistance = path.length;
                    closestRobot = r;
                    bestPath = path;
                }
            });

            if (closestRobot && bestPath) {
                const currentRemainingPathLength = robotObj.path ? robotObj.path.length - robotObj.pathIndex : Infinity;
                if (!robotObj.target || robotObj.target.isAwake || minDistance < currentRemainingPathLength) {
                    // Remove previous target from targetedRobots set
                    if (robotObj.target && !robotObj.target.isAwake) {
                        targetedRobots.delete(robotObj.target);
                    }
                    robotObj.target = closestRobot;
                    robotObj.path = bestPath.map((p) => gridToWorld(p));
                    robotObj.pathIndex = 0;
                    targetedRobots.add(closestRobot);
                }
            }
        }
    });
}

// Assign paths using Clustering
let clusters = [];
let clustersInitialized = false;

function assignPathsClustering() {
    if (!clustersInitialized) {
        clusters = clusterRobots();
        clustersInitialized = true;
    }

    awakeRobots.forEach((robotObj) => {
        if (!robotObj.cluster && clusters.length > 0) {
            // Assign the next cluster to this robot
            robotObj.cluster = clusters.shift();
        }

        if (!robotObj.target && robotObj.cluster) {
            const asleepRobotsInCluster = robotObj.cluster.filter((r) => !r.isAwake && !targetedRobots.has(r));
            if (asleepRobotsInCluster.length > 0) {
                const targetRobot = asleepRobotsInCluster[0];
                const startGridPos = worldToGrid(robotObj.mesh.position);
                const endGridPos = worldToGrid(targetRobot.mesh.position);
                const path = aStar.findPath(startGridPos, endGridPos);

                if (path) {
                    robotObj.target = targetRobot;
                    robotObj.path = path.map((p) => gridToWorld(p));
                    robotObj.pathIndex = 0;
                    targetedRobots.add(targetRobot);
                } else {
                    // If no path is found, remove the robot from the cluster to prevent infinite loops
                    robotObj.cluster = robotObj.cluster.filter((r) => r !== targetRobot);
                    if (robotObj.cluster.length === 0) {
                        robotObj.cluster = null;
                    }
                }
            }
        }
    });
}

// Function to cluster robots (improved)
function clusterRobots() {
    const clusters = [];
    const unclustered = robots.filter((r) => !r.isAwake);
    const threshold = 10; // Adjusted distance threshold for clustering
    while (unclustered.length > 0) {
        const cluster = [unclustered.pop()];
        for (let i = unclustered.length - 1; i >= 0; i--) {
            const dist = cluster[0].mesh.position.distanceTo(unclustered[i].mesh.position);
            if (dist < threshold) {
                cluster.push(unclustered.splice(i, 1)[0]);
            }
        }
        clusters.push(cluster);
    }
    return clusters;
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    if (isAwakening) {
        steps++; // **Increment steps counter**
        stepsDisplay.textContent = steps; // **Update the steps display**

        const algorithm = algorithmSelect.value;
        if (algorithm === 'greedyStatic') {
            assignPathsGreedyStatic();
        } else if (algorithm === 'greedyDynamic') {
            assignPathsGreedyDynamic();
        } else if (algorithm === 'clustering') {
            assignPathsClustering();
        }

        let allRobotsAwake = true; // **Flag to check if all robots are awake**

        awakeRobots.forEach((robotObj) => {
            if (robotObj.path && robotObj.pathIndex < robotObj.path.length) {
                const targetPos = robotObj.path[robotObj.pathIndex];
                robotObj.mesh.position.lerp(targetPos, 0.4); // Adjust the lerp factor for speed

                if (robotObj.mesh.position.distanceTo(targetPos) < 0.1) {
                    robotObj.pathIndex++;
                }
            } else if (robotObj.target) {
                if (robotObj.target.isAwake) {
                    // Target is already awake, so clear the target
                    robotObj.target = null;
                    robotObj.path = null;
                    robotObj.pathIndex = 0;
                } else {
                    // Awaken the target robot
                    robotObj.target.isAwake = true;
                    robotObj.target.mesh.material.color.set(0x00ff00); // Change color to indicate "awake"
                    awakeRobots.push(robotObj.target);
                    // Remove the awakened robot from targetedRobots set
                    targetedRobots.delete(robotObj.target);
                    // Remove the awakened robot from the cluster
                    if (robotObj.cluster) {
                        robotObj.cluster = robotObj.cluster.filter((r) => r !== robotObj.target);
                        if (robotObj.cluster.length === 0) {
                            robotObj.cluster = null;
                        }
                    }
                    // Reset target robot's properties
                    robotObj.target.target = null;
                    robotObj.target.path = null;
                    robotObj.target.pathIndex = 0;
                    robotObj.target.cluster = robotObj.cluster; // Assign the same cluster to the new awake robot

                    // Reset current robot's properties
                    robotObj.target = null;
                    robotObj.path = null;
                    robotObj.pathIndex = 0;
                }
            }
        });

        // **Check if all robots are awake**
        robots.forEach((robot) => {
            if (!robot.isAwake) {
                allRobotsAwake = false;
            }
        });

        if (allRobotsAwake) {
            isAwakening = false; // **Stop the awakening process**
            // **Display completion message**
            stepsDisplay.textContent = `${steps} (Awakening Complete)`;
        }
    }

    renderer.render(scene, camera);
}

// Start the animation loop
animate();

// Handle window resize
function onWindowResize() {
    const size = Math.min(animationSection.clientWidth, animationSection.clientHeight);
    renderer.setSize(size, size);
    camera.aspect = size / size;
    camera.updateProjectionMatrix();
}

window.addEventListener('resize', onWindowResize);
onWindowResize(); // Initial call to set size correctly
