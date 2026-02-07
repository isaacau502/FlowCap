import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const container = document.getElementById("canvas");
const infoEl = document.getElementById("info");

// Telemetry card DOM refs (populated at init)
const telemEls = {};
if (!container) {
  throw new Error("Canvas container not found.");
}

function hasWebGL() {
  try {
    const canvas = document.createElement("canvas");
    return !!(
      canvas.getContext("webgl2") ||
      canvas.getContext("webgl") ||
      canvas.getContext("experimental-webgl")
    );
  } catch (err) {
    return false;
  }
}

if (!hasWebGL()) {
  infoEl.textContent =
    "WebGL is not available in this browser.\n" +
    "Try enabling hardware acceleration or use a different browser.";
  throw new Error("WebGL not available.");
}

const state = {
  squatPct: 0,
  leanDeg: 0,
  torsoRot: 0,
  pitch: 0,
  roll: 0,
  slopeDeg: 0,
  boardAccelFwd: 0,
  isToeside: false,
};

const playback = {
  data: [],
  duration: 0,
  index: 0,
  lastT: 0,
  startTime: 0,
  playing: true,
  hasData: false,
  // Session playback extensions
  speed: 1.0,
  currentTime: 0,
  startWallTime: 0,
  sessionMode: false,
  activeSessionId: null,
  dimsBackup: null,
};

const liveWS = {
  ws: null,
  connected: false,
  reconnectTimer: null,
  enabled: true,
};

const recording = {
  active: false,
  frames: [],
  startTime: 0,
  dimsSnapshot: null,
  timerInterval: null,
  timedTimeout: null,
};

const trail = {
  enabled: true,
  points: [],           // [{x, y}] computed each frame for mesh rendering
  segments: [],         // [{curv, step}] per-frame curvature + distance history
  maxPoints: 600,       // ~20s at 30Hz
  baseSpeed: 0.018,     // units/frame forward speed
  ribbonWidth: 0.06,    // half-width of ribbon strip
  mesh: null,
  geometry: null,
  posAttr: null,
  colorAttr: null,
  _scrubbing: false,
  _lastTime: 0,
};

const dims = {
  boardWidth: 0.2,
  boardLength: 0.8,
  standingHeight: 0.9,
  minHeight: 0.4,
  torsoLength: 0.6,
  shinLength: 0.45,
  upperArmLen: 0.28,
  forearmLen: 0.25,
  shoulderSpread: 0.2,
  maxHipShift: 0.1,
  maxLeanDeg: 30,
};

const scene = new THREE.Scene();
scene.background = new THREE.Color("#0a0e14");
scene.up.set(0, 0, 1);

const camera = new THREE.PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  0.01,
  50
);
camera.position.set(1.4, -2.0, 1.1);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.3;
const initialRect = container.getBoundingClientRect();
renderer.setSize(
  Math.max(1, initialRect.width),
  Math.max(1, initialRect.height)
);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0, 0.6);
controls.update();

scene.fog = new THREE.FogExp2(0x0a0e14, 0.18);

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const hemiLight = new THREE.HemisphereLight(0x4a6fa5, 0x2d1b4e, 0.4);
scene.add(hemiLight);
const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
dirLight.position.set(2, -2, 3);
scene.add(dirLight);

const grid = new THREE.GridHelper(4, 20, 0x1a2540, 0x111827);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const axes = new THREE.AxesHelper(0.6);
axes.visible = false;
scene.add(axes);

// Cool cyberpunk color palette
const colors = {
  board: 0xd0d4d8,      // light metallic gray
  shin: 0x00cec9,       // teal cyan
  thigh: 0x0984e3,      // ocean blue
  torso: 0x6c5ce7,      // purple
  shoulder: 0xe84393,   // pink magenta
  upperArm: 0x0984e3,   // ocean blue (matches thigh)
  forearm: 0x00cec9,    // teal cyan (matches shin)
  hand: 0xdfe6e9,       // light silver
  head: 0xdfe6e9,       // light silver
  joints: 0x1e272e,     // dark charcoal
  pelvis: 0x0984e3,     // matches thigh
  feet: 0x1e272e,       // dark
  flow: 0x00ff88,       // neon green
};

const boardMaterial = new THREE.LineBasicMaterial({ color: colors.board });
const limbMaterial = new THREE.LineBasicMaterial({ color: colors.shin });
const thighMaterial = new THREE.LineBasicMaterial({ color: colors.thigh });
const torsoMaterial = new THREE.LineBasicMaterial({ color: colors.torso });
const shoulderMaterial = new THREE.LineBasicMaterial({ color: colors.shoulder });
const chestMaterial = new THREE.LineBasicMaterial({ color: colors.shoulder });

const solidBoardMaterial = new THREE.MeshStandardMaterial({
  color: colors.board,
  metalness: 0.55,
  roughness: 0.25,
});
const shinMaterial = new THREE.MeshStandardMaterial({ color: colors.shin, metalness: 0.2, roughness: 0.5, emissive: colors.shin, emissiveIntensity: 0.12 });
const thighSolidMaterial = new THREE.MeshStandardMaterial({ color: colors.thigh, metalness: 0.2, roughness: 0.5, emissive: colors.thigh, emissiveIntensity: 0.1 });
const torsoSolidMaterial = new THREE.MeshStandardMaterial({ color: colors.torso, metalness: 0.15, roughness: 0.6, emissive: colors.torso, emissiveIntensity: 0.1 });
const shoulderSolidMaterial = new THREE.MeshStandardMaterial({ color: colors.shoulder, metalness: 0.2, roughness: 0.5, emissive: colors.shoulder, emissiveIntensity: 0.1 });
const headMaterial = new THREE.MeshStandardMaterial({ color: colors.head, metalness: 0.1, roughness: 0.3, emissive: 0xffffff, emissiveIntensity: 0.05 });

const boardLine = new THREE.Line(
  new THREE.BufferGeometry(),
  boardMaterial
);
scene.add(boardLine);

const leftShin = new THREE.Line(new THREE.BufferGeometry(), limbMaterial);
const rightShin = new THREE.Line(new THREE.BufferGeometry(), limbMaterial);
const leftThigh = new THREE.Line(new THREE.BufferGeometry(), thighMaterial);
const rightThigh = new THREE.Line(new THREE.BufferGeometry(), thighMaterial);
const torsoLine = new THREE.Line(new THREE.BufferGeometry(), torsoMaterial);
const shoulderLine = new THREE.Line(
  new THREE.BufferGeometry(),
  shoulderMaterial
);
const chestLine = new THREE.Line(new THREE.BufferGeometry(), chestMaterial);

scene.add(leftShin, rightShin, leftThigh, rightThigh, torsoLine, shoulderLine, chestLine);
// Hide wireframe lines for clean demo — solid meshes are sufficient
const wireframeLines = [leftShin, rightShin, leftThigh, rightThigh, torsoLine, shoulderLine, boardLine, chestLine];
wireframeLines.forEach(l => l.visible = false);

const boardMesh = new THREE.Mesh(new THREE.BufferGeometry(), solidBoardMaterial);
scene.add(boardMesh);

// Create anatomically shaped calf geometry (bulge at top for gastrocnemius)
function createCalfGeometry() {
  const points = [];
  const segments = 12;
  for (let i = 0; i <= segments; i++) {
    const t = i / segments; // 0 = bottom (ankle), 1 = top (knee)
    // Calf muscle bulge peaks around 70-80% up from ankle
    const bulge = Math.sin(t * Math.PI * 0.85) * 0.6;
    const baseRadius = 0.025 + t * 0.015; // taper from ankle to knee
    const radius = baseRadius + bulge * 0.025;
    points.push(new THREE.Vector2(radius, t - 0.5));
  }
  return new THREE.LatheGeometry(points, 16);
}

// Create anatomically shaped thigh geometry (bulge in upper-mid for quadriceps)
function createThighGeometry() {
  const points = [];
  const segments = 12;
  for (let i = 0; i <= segments; i++) {
    const t = i / segments; // 0 = bottom (knee), 1 = top (hip)
    // Quad muscle bulge peaks around 50-70% up from knee
    const bulge = Math.sin(t * Math.PI * 0.9) * 0.7;
    const baseRadius = 0.035 + t * 0.02; // taper from knee toward hip
    const radius = baseRadius + bulge * 0.03;
    points.push(new THREE.Vector2(radius, t - 0.5));
  }
  return new THREE.LatheGeometry(points, 16);
}

// Create upper arm geometry (bicep/tricep bulge)
// y=-0.5 is at shoulder, y=+0.5 is at elbow (based on updateCylinder call order)
function createUpperArmGeometry() {
  const points = [];
  const segments = 10;
  for (let i = 0; i <= segments; i++) {
    const t = i / segments; // 0 = shoulder end (y=-0.5), 1 = elbow end (y=+0.5)
    // Bicep bulge peaks around 40-60% from shoulder (middle of upper arm)
    const bulge = Math.sin((0.3 + t * 0.7) * Math.PI) * 0.5;
    const baseRadius = 0.028 - t * 0.006; // thicker at shoulder, thinner at elbow
    const radius = baseRadius + bulge * 0.01;
    points.push(new THREE.Vector2(radius, t - 0.5));
  }
  return new THREE.LatheGeometry(points, 12);
}

// Create forearm geometry (thicker at elbow end which is at y=-0.5, tapers to wrist at y=+0.5)
function createForearmGeometry() {
  const points = [];
  const segments = 10;
  for (let i = 0; i <= segments; i++) {
    const t = i / segments; // 0 = elbow end (y=-0.5), 1 = wrist end (y=+0.5)
    // Forearm thicker at elbow (t=0), thinner at wrist (t=1)
    const tInv = 1 - t; // invert so bulge is at elbow
    const bulge = Math.sin(tInv * Math.PI * 0.9) * 0.5;
    const baseRadius = 0.015 + tInv * 0.012; // thicker at elbow, thinner at wrist
    const radius = baseRadius + bulge * 0.006;
    points.push(new THREE.Vector2(radius, t - 0.5));
  }
  return new THREE.LatheGeometry(points, 12);
}

const shinGeo = createCalfGeometry();
const thighGeo = createThighGeometry();
const upperArmGeo = createUpperArmGeometry();
const forearmGeo = createForearmGeometry();
const shoulderGeo = new THREE.CylinderGeometry(0.04, 0.04, 1, 16);
const leftShinSolid = new THREE.Mesh(shinGeo, shinMaterial);
const rightShinSolid = new THREE.Mesh(shinGeo, shinMaterial);
const leftThighSolid = new THREE.Mesh(thighGeo, thighSolidMaterial);
const rightThighSolid = new THREE.Mesh(thighGeo, thighSolidMaterial);
const shoulderSolid = new THREE.Mesh(shoulderGeo, shoulderSolidMaterial);

// Arm materials
const upperArmMaterial = new THREE.MeshStandardMaterial({ color: colors.upperArm, metalness: 0.2, roughness: 0.5, emissive: colors.upperArm, emissiveIntensity: 0.1 });
const forearmMaterial = new THREE.MeshStandardMaterial({ color: colors.forearm, metalness: 0.2, roughness: 0.5, emissive: colors.forearm, emissiveIntensity: 0.12 });
const handMaterial = new THREE.MeshStandardMaterial({ color: colors.hand, metalness: 0.1, roughness: 0.4, emissive: 0xffffff, emissiveIntensity: 0.05 });

// Arm meshes
const leftUpperArmSolid = new THREE.Mesh(upperArmGeo, upperArmMaterial);
const rightUpperArmSolid = new THREE.Mesh(upperArmGeo, upperArmMaterial);
const leftForearmSolid = new THREE.Mesh(forearmGeo, forearmMaterial);
const rightForearmSolid = new THREE.Mesh(forearmGeo, forearmMaterial);

// Hand meshes (small spheres)
const handGeo = new THREE.SphereGeometry(0.025, 10, 8);
const leftHandMesh = new THREE.Mesh(handGeo, handMaterial);
const rightHandMesh = new THREE.Mesh(handGeo, handMaterial);

// Create tapered torso geometry (wider at shoulders, narrower at waist)
// Local coords: Y = height (up), Z = width (left-right shoulders), X = depth (front-back chest)
function createTorsoGeometry() {
  const geo = new THREE.BufferGeometry();
  // Torso dimensions: waist at bottom, chest/shoulders at top - BEEFED UP
  const waistWidth = 0.16;   // narrower at waist (left-right)
  const waistDepth = 0.10;   // front-back
  const chestWidth = 0.24;   // wider at chest
  const chestDepth = 0.14;
  
  // 8 vertices: 4 at bottom (waist), 4 at top (chest)
  // X = depth (front-back), Y = height, Z = width (left-right)
  const vertices = new Float32Array([
    // Bottom face (waist) - y = -0.5
    -waistDepth/2, -0.5, -waistWidth/2,  // 0: back-left
    -waistDepth/2, -0.5,  waistWidth/2,  // 1: back-right
     waistDepth/2, -0.5,  waistWidth/2,  // 2: front-right
     waistDepth/2, -0.5, -waistWidth/2,  // 3: front-left
    // Top face (chest) - y = 0.5
    -chestDepth/2,  0.5, -chestWidth/2,  // 4: back-left
    -chestDepth/2,  0.5,  chestWidth/2,  // 5: back-right
     chestDepth/2,  0.5,  chestWidth/2,  // 6: front-right
     chestDepth/2,  0.5, -chestWidth/2,  // 7: front-left
  ]);
  
  // Indices for triangles (6 faces, 2 triangles each) - CCW winding for front faces
  const indices = [
    // Front face (+X)
    3, 7, 6,  3, 6, 2,
    // Back face (-X)
    1, 5, 4,  1, 4, 0,
    // Left face (-Z)
    0, 4, 7,  0, 7, 3,
    // Right face (+Z)
    2, 6, 5,  2, 5, 1,
    // Top face (+Y)
    4, 5, 6,  4, 6, 7,
    // Bottom face (-Y)
    0, 3, 2,  0, 2, 1,
  ];
  
  geo.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return geo;
}

const torsoGeo = createTorsoGeometry();
const torsoSolid = new THREE.Mesh(torsoGeo, torsoSolidMaterial);
scene.add(torsoSolid);

// Add a head (sphere) - proportional to torso
const headGeo = new THREE.SphereGeometry(0.07, 16, 12);
const headMesh = new THREE.Mesh(headGeo, headMaterial);
scene.add(headMesh);

// Add pelvis/hip mass
const pelvisMaterial = new THREE.MeshStandardMaterial({ color: colors.pelvis, metalness: 0.2, roughness: 0.5 });
const pelvisGeo = new THREE.SphereGeometry(0.09, 12, 8);
const pelvisMesh = new THREE.Mesh(pelvisGeo, pelvisMaterial);
pelvisMesh.scale.set(1.2, 0.6, 1.0); // flattened sphere for hip shape
scene.add(pelvisMesh);

// Add feet (box shapes)
const footMaterial = new THREE.MeshStandardMaterial({ color: colors.feet, metalness: 0.1, roughness: 0.7 });
const footGeo = new THREE.BoxGeometry(0.08, 0.12, 0.04); // length along board, width, height
const leftFootMesh = new THREE.Mesh(footGeo, footMaterial);
const rightFootMesh = new THREE.Mesh(footGeo, footMaterial);
scene.add(leftFootMesh, rightFootMesh);

scene.add(
  leftShinSolid,
  rightShinSolid,
  leftThighSolid,
  rightThighSolid,
  shoulderSolid,
  leftUpperArmSolid,
  rightUpperArmSolid,
  leftForearmSolid,
  rightForearmSolid,
  leftHandMesh,
  rightHandMesh
);

const jointGroup = new THREE.Group();
scene.add(jointGroup);
const jointMaterial = new THREE.MeshStandardMaterial({ color: colors.joints, metalness: 0.4, roughness: 0.3 });
const jointGeometry = new THREE.SphereGeometry(0.055, 16, 16);  // bigger joints
const kneeGeometry = new THREE.SphereGeometry(0.045, 16, 16);   // knees slightly smaller
const elbowGeometry = new THREE.SphereGeometry(0.028, 12, 12);  // elbow joints
const joints = Array.from({ length: 10 }, (_, i) => {
  // indices 2,3 are knees, 8,9 are elbows
  let geo = jointGeometry;
  if (i === 2 || i === 3) geo = kneeGeometry;
  if (i === 8 || i === 9) geo = elbowGeometry;
  const mesh = new THREE.Mesh(geo, jointMaterial);
  jointGroup.add(mesh);
  return mesh;
});

const arrowBoard = new THREE.ArrowHelper(
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(0, 0.4, 0.02),
  0.4,
  colors.flow
);
arrowBoard.visible = false;
scene.add(arrowBoard);

const arrowEye = new THREE.ArrowHelper(
  new THREE.Vector3(-1, 0, 0),
  new THREE.Vector3(-0.4, 0, 0.02),
  0.4,
  colors.shoulder
);
arrowEye.visible = false;
scene.add(arrowEye);

// Curved flow arrow showing board travel direction based on edge%
const flowMaterial = new THREE.LineBasicMaterial({ color: colors.flow, linewidth: 2 });
const flowLine = new THREE.Line(new THREE.BufferGeometry(), flowMaterial);
scene.add(flowLine);

// Arrowhead cone at end of flow path
const arrowheadGeo = new THREE.ConeGeometry(0.04, 0.12, 8);
const arrowheadMat = new THREE.MeshStandardMaterial({ color: colors.flow });
const flowArrowhead = new THREE.Mesh(arrowheadGeo, arrowheadMat);
scene.add(flowArrowhead);

// ── Carve Trail ───────────────────────────────────────────────────────────

function createTrailMesh() {
  const maxVerts = trail.maxPoints * 2;
  const maxTris = (trail.maxPoints - 1) * 2;

  trail.geometry = new THREE.BufferGeometry();

  const positions = new Float32Array(maxVerts * 3);
  trail.posAttr = new THREE.BufferAttribute(positions, 3);
  trail.posAttr.setUsage(THREE.DynamicDrawUsage);
  trail.geometry.setAttribute("position", trail.posAttr);

  const vertColors = new Float32Array(maxVerts * 3);
  trail.colorAttr = new THREE.BufferAttribute(vertColors, 3);
  trail.colorAttr.setUsage(THREE.DynamicDrawUsage);
  trail.geometry.setAttribute("color", trail.colorAttr);

  const indices = new Uint16Array(maxTris * 3);
  trail.geometry.setIndex(new THREE.BufferAttribute(indices, 1));

  const material = new THREE.MeshBasicMaterial({
    vertexColors: true,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.85,
    depthWrite: false,
  });

  trail.mesh = new THREE.Mesh(trail.geometry, material);
  trail.mesh.frustumCulled = false;
  scene.add(trail.mesh);
}

function updateTrail() {
  if (!trail.enabled || trail._scrubbing) return;

  const now = performance.now();
  const dt = trail._lastTime ? Math.min((now - trail._lastTime) / 1000, 0.1) : 1 / 30;
  trail._lastTime = now;
  const scale = dt * 30;

  // Curvature matched to flow arrow: buildFlowPath uses curvature = clamp(lean/maxLean) * 0.8
  const curvature = Math.max(-1, Math.min(1, state.leanDeg / dims.maxLeanDeg)) * 0.8;
  const curv = Math.abs(curvature) < 0.04 ? 0 : curvature; // dead zone for noise
  const pathLen = 1.2;

  let speed = trail.baseSpeed * scale;
  if (state.boardAccelFwd !== 0) {
    speed *= 1.0 + Math.max(-0.5, Math.min(0.5, state.boardAccelFwd / 6.0));
  }

  // Store this frame's curvature and step distance
  trail.segments.push({ curv: curv, step: speed });
  if (trail.segments.length > trail.maxPoints) trail.segments.shift();

  // Reconstruct trail path by integrating curvatures backward from the board
  const n = trail.segments.length;
  let heading = 0;
  let px = 0, py = 0;

  // Board position is the newest point
  if (!trail.points[n]) trail.points[n] = { x: 0, y: 0 };
  trail.points[n].x = 0;
  trail.points[n].y = 0;

  // Walk backward from board (newest segment to oldest)
  for (let i = n - 1; i >= 0; i--) {
    const seg = trail.segments[i];
    // Heading at past position: undo the turn that happened at this segment
    heading += (2 * seg.curv / pathLen) * seg.step;
    px -= Math.sin(heading) * seg.step;
    py -= Math.cos(heading) * seg.step;
    if (!trail.points[i]) trail.points[i] = { x: 0, y: 0 };
    trail.points[i].x = px;
    trail.points[i].y = py;
  }
  trail.points.length = n + 1;

  updateTrailMesh();
}

function updateTrailMesh() {
  const pts = trail.points;
  const n = pts.length;
  if (n < 2) { trail.geometry.setDrawRange(0, 0); return; }

  const posArr = trail.posAttr.array;
  const colArr = trail.colorAttr.array;
  const idxArr = trail.geometry.index.array;
  const Z = 0.005;
  const hw = trail.ribbonWidth;

  for (let i = 0; i < n; i++) {
    const p = pts[i];
    let tx, ty;
    if (i === 0) { tx = pts[1].x - pts[0].x; ty = pts[1].y - pts[0].y; }
    else if (i === n - 1) { tx = pts[n - 1].x - pts[n - 2].x; ty = pts[n - 1].y - pts[n - 2].y; }
    else { tx = pts[i + 1].x - pts[i - 1].x; ty = pts[i + 1].y - pts[i - 1].y; }

    const tLen = Math.sqrt(tx * tx + ty * ty);
    if (tLen > 1e-6) { tx /= tLen; ty /= tLen; }
    else { tx = 0; ty = 1; }

    const nx = ty, ny = -tx;
    const vi = i * 2;
    posArr[vi * 3] = p.x - nx * hw;     posArr[vi * 3 + 1] = p.y - ny * hw;     posArr[vi * 3 + 2] = Z;
    posArr[(vi+1)*3] = p.x + nx * hw;   posArr[(vi+1)*3+1] = p.y + ny * hw;     posArr[(vi+1)*3+2] = Z;

    const t = i / (n - 1);
    const fade = t * t;
    const r = 0.08 * (1 - fade);
    const g = 0.15 + 0.85 * fade;
    const b = 0.05 + 0.483 * fade;

    colArr[vi * 3] = r;     colArr[vi * 3 + 1] = g;     colArr[vi * 3 + 2] = b;
    colArr[(vi+1)*3] = r;   colArr[(vi+1)*3+1] = g;     colArr[(vi+1)*3+2] = b;
  }

  let idx = 0;
  for (let i = 0; i < n - 1; i++) {
    const a = i * 2, b = i * 2 + 1, c = (i+1) * 2, d = (i+1) * 2 + 1;
    idxArr[idx++] = a; idxArr[idx++] = c; idxArr[idx++] = b;
    idxArr[idx++] = b; idxArr[idx++] = c; idxArr[idx++] = d;
  }

  trail.geometry.setDrawRange(0, idx);
  trail.posAttr.needsUpdate = true;
  trail.colorAttr.needsUpdate = true;
  trail.geometry.index.needsUpdate = true;
}

createTrailMesh();

function buildFlowPath(leanDeg) {
  // leanDeg: raw board lean in degrees
  // Curvature: negative = curve left (heelside for goofy), positive = curve right (toeside)
  const curvature = Math.max(-1, Math.min(1, leanDeg / dims.maxLeanDeg)) * 0.8;
  const numPts = 30;
  const pathLen = 1.2;
  const pts = [];
  for (let i = 0; i <= numPts; i++) {
    const t = i / numPts;
    const y = 0.5 + t * pathLen; // start in front of board, go forward
    // x curves based on edge: positive edge = toeside = curve in -x direction (rider's left for goofy)
    const x = -curvature * t * t * pathLen;
    pts.push(new THREE.Vector3(x, y, 0.01));
  }
  return pts;
}

function updateFlowArrow(leanDeg) {
  const pts = buildFlowPath(leanDeg);
  flowLine.geometry.setFromPoints(pts);

  // Position arrowhead at end, orient along curve tangent
  const last = pts[pts.length - 1];
  const prev = pts[pts.length - 2];
  flowArrowhead.position.copy(last);
  const dir = new THREE.Vector3().subVectors(last, prev).normalize();
  // Cone points along +Y by default; rotate to point along dir
  const up = new THREE.Vector3(0, 1, 0);
  flowArrowhead.quaternion.setFromUnitVectors(up, dir);
}

function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

function getBodyPoints() {
  const squatPct = state.squatPct;
  const torsoRot = degToRad(state.torsoRot);
  const pitch = degToRad(state.pitch);
  const roll = degToRad(state.roll);

  const hipHeight =
    dims.standingHeight - (squatPct / 100) * (dims.standingHeight - dims.minHeight);

  // Normalize lean degrees to -1..+1 for proportional effects (hip shift, arms)
  const edgeNorm = Math.max(-1, Math.min(1, state.leanDeg / dims.maxLeanDeg));
  const hipShift = -edgeNorm * (squatPct / 100) * dims.maxHipShift;
  const edgeAngle = degToRad(state.leanDeg);

  const cosE = Math.cos(edgeAngle);
  const sinE = Math.sin(edgeAngle);
  const edgeRot = new THREE.Matrix3().set(
    cosE, 0, -sinE,
    0, 1, 0,
    sinE, 0, cosE
  );

  // Slope rotation: pitch around X axis (nose-down = positive)
  const slopeAngle = degToRad(state.slopeDeg);
  const cosS = Math.cos(slopeAngle);
  const sinS = Math.sin(slopeAngle);
  const slopeRot = new THREE.Matrix3().set(
    1, 0, 0,
    0, cosS, -sinS,
    0, sinS, cosS
  );

  const boardFlat = [
    new THREE.Vector3(-dims.boardWidth / 2, -dims.boardLength / 2, 0),
    new THREE.Vector3(dims.boardWidth / 2, -dims.boardLength / 2, 0),
    new THREE.Vector3(dims.boardWidth / 2, dims.boardLength / 2, 0),
    new THREE.Vector3(-dims.boardWidth / 2, dims.boardLength / 2, 0),
    new THREE.Vector3(-dims.boardWidth / 2, -dims.boardLength / 2, 0),
  ];
  // Combined board rotation: slope pitch then edge lean
  const boardRot = edgeRot.clone().multiply(slopeRot);
  const board = boardFlat.map((p) => p.clone().applyMatrix3(boardRot));
  const minZ = Math.min(...board.map((p) => p.z));
  board.forEach((p) => (p.z -= minZ));

  const carveRot = boardRot.clone();

  const leftFootLocal = new THREE.Vector3(0, -0.25, 0.02);
  const rightFootLocal = new THREE.Vector3(0, 0.25, 0.02);

  const kneeAngleDeg = 170 - (squatPct / 100) * 100;
  const kneeOffset = 0.12 + (squatPct / 100) * 0.08;
  const kneeHeightLocal = dims.shinLength * Math.cos(degToRad(90 - kneeAngleDeg / 2));

  const leftKneeLocal = new THREE.Vector3(hipShift - kneeOffset, -0.25, kneeHeightLocal);
  const rightKneeLocal = new THREE.Vector3(hipShift - kneeOffset, 0.25, kneeHeightLocal);
  const hipLocal = new THREE.Vector3(hipShift, 0, hipHeight);

  const leftFoot = leftFootLocal.clone().applyMatrix3(carveRot);
  const rightFoot = rightFootLocal.clone().applyMatrix3(carveRot);
  const leftKnee = leftKneeLocal.clone().applyMatrix3(carveRot);
  const rightKnee = rightKneeLocal.clone().applyMatrix3(carveRot);
  const hip = hipLocal.clone().applyMatrix3(carveRot);

  [leftFoot, rightFoot, leftKnee, rightKnee, hip].forEach((p) => (p.z -= minZ));

  const torsoVec = new THREE.Vector3(0, 0, dims.torsoLength);
  const pitchRot = new THREE.Matrix3().set(
    Math.cos(pitch), 0, Math.sin(pitch),
    0, 1, 0,
    -Math.sin(pitch), 0, Math.cos(pitch)
  );
  const yawRot = new THREE.Matrix3().set(
    Math.cos(torsoRot), -Math.sin(torsoRot), 0,
    Math.sin(torsoRot), Math.cos(torsoRot), 0,
    0, 0, 1
  );
  const rollRot = new THREE.Matrix3().set(
    1, 0, 0,
    0, Math.cos(roll), -Math.sin(roll),
    0, Math.sin(roll), Math.cos(roll)
  );

  const torsoRotMat = carveRot.clone().multiply(yawRot).multiply(pitchRot).multiply(rollRot);
  const torsoVecRot = torsoVec.clone().applyMatrix3(torsoRotMat);
  const head = hip.clone().add(torsoVecRot);

  const shoulderVec = new THREE.Vector3(0, dims.shoulderSpread, 0).applyMatrix3(torsoRotMat);
  const shoulderCenter = hip.clone().add(torsoVecRot.clone().multiplyScalar(0.85));
  const leftShoulder = shoulderCenter.clone().sub(shoulderVec);
  const rightShoulder = shoulderCenter.clone().add(shoulderVec);

  const chest = hip.clone().add(torsoVecRot.clone().multiplyScalar(0.6));
  const chestNormal = new THREE.Vector3(-1, 0, 0).applyMatrix3(torsoRotMat).multiplyScalar(0.3);

  // === PROCEDURAL ARM CALCULATIONS ===
  const squat01 = squatPct / 100;
  const edge01 = Math.max(-1, Math.min(1, state.leanDeg / dims.maxLeanDeg));
  const rotRad = torsoRot; // already in radians

  // Arm lengths from calibration profile
  const upperArmLen = dims.upperArmLen;
  const forearmLen = dims.forearmLen;

  // Helper to calculate arm points for one side
  function calcArmPoints(shoulder, isLeft) {
    const side = isLeft ? -1 : 1;
    
    // Base elbow direction in torso-local space (hanging down and slightly out/forward)
    let elbowDir = new THREE.Vector3(
      -0.1 - squat01 * 0.15,              // X: forward (more when squatting)
      side * (0.15 + squat01 * 0.2),      // Y: outward (more when squatting)
      -0.85 + squat01 * 0.5               // Z: down (arms rise when squatting)
    );
    
    // Edge counterbalance: arms shift opposite to lean direction
    elbowDir.x += edge01 * 0.2;
    // Asymmetric height adjustment: downhill arm drops, uphill arm rises
    elbowDir.z += (isLeft ? -1 : 1) * edge01 * 0.15;
    
    // Rotation-based arm flare: arms extend outward with rotation magnitude
    const rotMagnitude = Math.abs(rotRad);
    elbowDir.y += side * rotMagnitude * 0.15;
    
    // Counter-rotation: arms lag behind torso rotation (~40%)
    const counterRotAngle = -rotRad * 0.4;
    const cosR = Math.cos(counterRotAngle);
    const sinR = Math.sin(counterRotAngle);
    const rotX = elbowDir.x * cosR - elbowDir.y * sinR;
    const rotY = elbowDir.x * sinR + elbowDir.y * cosR;
    elbowDir.x = rotX;
    elbowDir.y = rotY;
    
    // Normalize and scale to upper arm length
    elbowDir.normalize().multiplyScalar(upperArmLen);
    
    // Transform to world space using torso rotation
    elbowDir.applyMatrix3(torsoRotMat);
    const elbow = shoulder.clone().add(elbowDir);
    
    // Hand direction from elbow (continues roughly same direction, slightly more down/forward)
    let handDir = new THREE.Vector3(
      -0.2 - squat01 * 0.2,               // forward
      side * (0.1 + squat01 * 0.15),      // outward
      -0.8 + squat01 * 0.3                // down
    );
    
    // Same edge and rotation adjustments
    handDir.x += edge01 * 0.15;
    handDir.z += (isLeft ? -1 : 1) * edge01 * 0.1;
    
    // Rotation-based arm flare for hands
    handDir.y += side * rotMagnitude * 0.12;
    
    const handRotX = handDir.x * cosR - handDir.y * sinR;
    const handRotY = handDir.x * sinR + handDir.y * cosR;
    handDir.x = handRotX;
    handDir.y = handRotY;
    
    handDir.normalize().multiplyScalar(forearmLen);
    handDir.applyMatrix3(torsoRotMat);
    const hand = elbow.clone().add(handDir);
    
    return { elbow, hand };
  }
  
  const leftArm = calcArmPoints(leftShoulder, true);
  const rightArm = calcArmPoints(rightShoulder, false);

  return {
    board,
    leftFoot,
    rightFoot,
    leftKnee,
    rightKnee,
    hip,
    head,
    shoulderCenter,
    leftShoulder,
    rightShoulder,
    leftElbow: leftArm.elbow,
    rightElbow: rightArm.elbow,
    leftHand: leftArm.hand,
    rightHand: rightArm.hand,
    chest,
    chestTip: chest.clone().add(chestNormal),
    kneeAngle: kneeAngleDeg,
    hipShift,
    edgeAngle,
    minZ,
    torsoRotMat,
  };
}

function updateLine(line, points) {
  line.geometry.setFromPoints(points);
}

function updateCylinder(mesh, start, end) {
  const dir = new THREE.Vector3().subVectors(end, start);
  const length = dir.length();
  if (length < 1e-6) {
    mesh.visible = false;
    return;
  }
  mesh.visible = true;
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    dir.normalize()
  );
  mesh.scale.set(1, length, 1);
}

function updateTorso(mesh, hip, shoulderCenter, torsoRotMat) {
  const dir = new THREE.Vector3().subVectors(shoulderCenter, hip);
  const length = dir.length();
  if (length < 1e-6) {
    mesh.visible = false;
    return;
  }
  mesh.visible = true;
  // Position at midpoint between hip and shoulder center
  mesh.position.copy(hip).add(shoulderCenter).multiplyScalar(0.5);
  
  // The torso geometry has Y as its length axis, but torsoRotMat assumes Z-up
  // We need to: 1) rotate geometry from Y-up to Z-up, 2) apply torsoRotMat
  // Rotation of +90° around X converts Y-up to Z-up (right-hand rule)
  const yToZ = new THREE.Matrix4().makeRotationX(Math.PI / 2);
  const mat4 = new THREE.Matrix4().setFromMatrix3(torsoRotMat);
  mat4.multiply(yToZ);
  mesh.quaternion.setFromRotationMatrix(mat4);
  
  // Scale to match torso length
  mesh.scale.set(1, length, 1);
}

function updateModel() {
  // Simple: toeside when positive, heelside when negative (no threshold hysteresis)
  state.isToeside = state.leanDeg > 0;
  const points = getBodyPoints();

  updateLine(boardLine, points.board);
  updateLine(leftShin, [points.leftFoot, points.leftKnee]);
  updateLine(rightShin, [points.rightFoot, points.rightKnee]);
  updateLine(leftThigh, [points.leftKnee, points.hip]);
  updateLine(rightThigh, [points.rightKnee, points.hip]);
  updateLine(torsoLine, [points.hip, points.head]);
  updateLine(shoulderLine, [points.leftShoulder, points.rightShoulder]);
  updateLine(chestLine, [points.chest, points.chestTip]);
  const boardVerts = new Float32Array([
    points.board[0].x, points.board[0].y, points.board[0].z,
    points.board[1].x, points.board[1].y, points.board[1].z,
    points.board[2].x, points.board[2].y, points.board[2].z,
    points.board[3].x, points.board[3].y, points.board[3].z,
  ]);
  boardMesh.geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(boardVerts, 3)
  );
  boardMesh.geometry.setIndex([0, 1, 2, 0, 2, 3]);
  boardMesh.geometry.computeVertexNormals();

  updateCylinder(leftShinSolid, points.leftFoot, points.leftKnee);
  updateCylinder(rightShinSolid, points.rightFoot, points.rightKnee);
  updateCylinder(leftThighSolid, points.leftKnee, points.hip);
  updateCylinder(rightThighSolid, points.rightKnee, points.hip);
  updateTorso(torsoSolid, points.hip, points.shoulderCenter, points.torsoRotMat);
  updateCylinder(shoulderSolid, points.leftShoulder, points.rightShoulder);
  
  // Update arms
  updateCylinder(leftUpperArmSolid, points.leftShoulder, points.leftElbow);
  updateCylinder(rightUpperArmSolid, points.rightShoulder, points.rightElbow);
  updateCylinder(leftForearmSolid, points.leftElbow, points.leftHand);
  updateCylinder(rightForearmSolid, points.rightElbow, points.rightHand);
  
  // Position hands
  leftHandMesh.position.copy(points.leftHand);
  rightHandMesh.position.copy(points.rightHand);
  
  // Position the head
  headMesh.position.copy(points.head);
  
  // Position the pelvis at hip
  pelvisMesh.position.copy(points.hip);
  
  // Position feet
  leftFootMesh.position.copy(points.leftFoot);
  leftFootMesh.position.z += 0.02; // lift slightly above board
  rightFootMesh.position.copy(points.rightFoot);
  rightFootMesh.position.z += 0.02;

  const jointPoints = [
    points.leftFoot,
    points.rightFoot,
    points.leftKnee,
    points.rightKnee,
    points.hip,
    points.head,
    points.leftShoulder,
    points.rightShoulder,
    points.leftElbow,
    points.rightElbow,
  ];
  jointPoints.forEach((p, idx) => joints[idx].position.copy(p));

  updateFlowArrow(state.leanDeg);

  // Update telemetry cards
  const carveDir = state.leanDeg > 0 ? "TOESIDE" : state.leanDeg < 0 ? "HEELSIDE" : "STRAIGHT";
  const carveLabel = carveDir === 'TOESIDE' ? 'TURNING LEFT' : carveDir === 'HEELSIDE' ? 'TURNING RIGHT' : 'STRAIGHT';
  const carveSub = carveDir === 'TOESIDE' ? 'Toe Edge' : carveDir === 'HEELSIDE' ? 'Heel Edge' : 'Flat Board';
  const tData = {
    carveState: carveLabel,
    tSquat: state.squatPct.toFixed(0),
    tLean: state.leanDeg.toFixed(1),
    tKnee: points.kneeAngle.toFixed(0),
    tRotation: state.torsoRot.toFixed(0),
    tHipShift: (points.hipShift * 100).toFixed(0),
    tPitch: state.pitch.toFixed(0),
    tSlope: state.slopeDeg.toFixed(1),
  };
  for (const [id, val] of Object.entries(tData)) {
    const el = telemEls[id];
    if (el) el.textContent = val;
  }
  // Tint hero card by carve direction + update subtitle
  const heroEl = telemEls.carveState;
  if (heroEl && heroEl.parentElement) {
    heroEl.parentElement.className = 'telem-hero' + (carveDir === 'TOESIDE' ? ' toeside' : carveDir === 'HEELSIDE' ? ' heelside' : '');
  }
  const subEl = telemEls.carveSubtitle;
  if (subEl) subEl.textContent = carveSub;
}

function syncUI() {
  const map = [
    ["squat", "squatPct"],
    ["edge", "leanDeg"],
    ["rotation", "torsoRot"],
    ["pitch", "pitch"],
    ["roll", "roll"],
    ["slope", "slopeDeg"],
  ];
  map.forEach(([id, key]) => {
    const input = document.getElementById(id);
    const valueEl = document.getElementById(`${id}Value`);
    input.value = String(state[key]);
    valueEl.textContent = String(state[key]);
  });
}

function applyStateFromSample(sample) {
  state.squatPct = sample.squatPct;
  state.leanDeg = sample.leanDeg;
  state.torsoRot = sample.torsoRot;
  state.pitch = sample.pitch;
  state.roll = sample.roll;
  if (sample.slopeDeg != null) state.slopeDeg = sample.slopeDeg;
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function sampleAt(timeS) {
  const data = playback.data;
  if (!data.length) {
    return null;
  }
  if (timeS <= data[0].time_s) {
    return data[0];
  }
  if (timeS >= data[data.length - 1].time_s) {
    return data[data.length - 1];
  }
  while (playback.index < data.length - 2 && data[playback.index + 1].time_s < timeS) {
    playback.index += 1;
  }
  const a = data[playback.index];
  const b = data[playback.index + 1];
  const span = b.time_s - a.time_s;
  const t = span > 0 ? (timeS - a.time_s) / span : 0;
  return {
    time_s: timeS,
    squat_pct: lerp(a.squat_pct, b.squat_pct, t),
    lean_deg: lerp(a.lean_deg, b.lean_deg, t),
    torso_rot: lerp(a.torso_rot, b.torso_rot, t),
    pitch: lerp(a.pitch, b.pitch, t),
    roll: lerp(a.roll, b.roll, t),
  };
}

function tickPlayback() {
  // Session playback mode
  if (playback.sessionMode && playback.hasData) {
    if (!playback.playing) return false;
    const now = performance.now();
    const wallDelta = now - playback.startWallTime;
    playback.startWallTime = now;
    playback.currentTime += wallDelta * playback.speed;

    // Loop
    if (playback.currentTime >= playback.duration) {
      playback.currentTime = 0;
      playback.index = 0;
      trail.points = []; trail.segments = []; trail._lastTime = 0;
    }

    const sample = sampleAtMs(playback.currentTime);
    if (!sample) return false;
    applyStateFromSample(sample);
    syncUI();
    updateModel();
    updatePlaybackUI();
    return true;
  }

  // Legacy CSV playback
  if (liveWS.connected) {
    return false;
  }
  if (!playback.hasData || !playback.playing) {
    return false;
  }
  const elapsed = (performance.now() - playback.startTime) / 1000;
  const duration = playback.duration || 1;
  const t = elapsed % duration;
  if (t < playback.lastT) {
    playback.index = 0;
  }
  playback.lastT = t;
  const sample = sampleAt(t);
  if (!sample) {
    return false;
  }
  applyStateFromSample({
    squatPct: sample.squat_pct,
    leanDeg: sample.lean_deg,
    torsoRot: sample.torso_rot,
    pitch: sample.pitch,
    roll: sample.roll,
  });
  syncUI();
  updateModel();
  return true;
}

// ── Session Playback ─────────────────────────────────────────────────────

function sampleAtMs(timeMs) {
  const data = playback.data;
  if (!data.length) return null;

  // Forward scan (frames sorted by t)
  while (playback.index < data.length - 2 && data[playback.index + 1].t < timeMs) {
    playback.index++;
  }

  const a = data[playback.index];
  const b = data[Math.min(playback.index + 1, data.length - 1)];
  const span = b.t - a.t;
  const frac = span > 0 ? (timeMs - a.t) / span : 0;

  return {
    squatPct: lerp(a.s[0], b.s[0], frac),
    leanDeg: lerp(a.s[1], b.s[1], frac),
    torsoRot: lerp(a.s[2], b.s[2], frac),
    pitch: lerp(a.s[3], b.s[3], frac),
    roll: lerp(a.s[4], b.s[4], frac),
    slopeDeg: lerp(a.s[5], b.s[5], frac),
  };
}

function loadSessionIntoPlayback(session, sessionId) {
  trail.points = []; trail.segments = []; trail._lastTime = 0;
  // Backup current dims for restoration later
  playback.dimsBackup = { ...dims };

  // Apply session dims
  if (session.config && session.config.dims) {
    Object.assign(dims, session.config.dims);
  }

  playback.data = session.frames;
  playback.duration = session.meta.durationMs;
  playback.index = 0;
  playback.currentTime = 0;
  playback.startWallTime = performance.now();
  playback.playing = true;
  playback.hasData = true;
  playback.speed = 1.0;
  playback.sessionMode = true;
  playback.activeSessionId = sessionId || true;

  // Show playback transport bar
  const pbBar = document.getElementById("playbackBar");
  if (pbBar) pbBar.style.display = "";

  // Update speed buttons
  document.querySelectorAll(".speed-btn").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.speed === "1");
  });

  updatePlaybackUI();
}

function exitSessionPlayback() {
  trail.points = []; trail.segments = []; trail._lastTime = 0;
  playback.sessionMode = false;
  playback.activeSessionId = null;
  playback.hasData = false;
  playback.playing = false;
  playback.data = [];

  // Restore dims
  if (playback.dimsBackup) {
    Object.assign(dims, playback.dimsBackup);
    playback.dimsBackup = null;
  }

  // Hide playback bar
  const pbBar = document.getElementById("playbackBar");
  if (pbBar) pbBar.style.display = "none";

  // Deselect active session in list
  document.querySelectorAll(".session-item.active").forEach((el) => el.classList.remove("active"));
}

function updatePlaybackUI() {
  const scrub = document.getElementById("pbScrub");
  const timeEl = document.getElementById("pbTime");
  const playPauseBtn = document.getElementById("pbPlayPause");

  if (scrub && playback.duration > 0) {
    scrub.value = Math.round((playback.currentTime / playback.duration) * 1000);
  }

  if (timeEl) {
    const cur = Math.round(playback.currentTime / 1000);
    const total = Math.round(playback.duration / 1000);
    const fmt = (s) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
    timeEl.textContent = `${fmt(cur)} / ${fmt(total)}`;
  }

  if (playPauseBtn) {
    playPauseBtn.textContent = playback.playing ? "\u23F8" : "\u25B6";
  }
}

function formatDuration(ms) {
  const sec = Math.round(ms / 1000);
  const min = Math.floor(sec / 60);
  const s = sec % 60;
  return min > 0 ? `${min}m ${s}s` : `${s}s`;
}

function renderSessionList(sessions) {
  const list = document.getElementById("sessionsList");
  if (!list) return;

  if (!sessions.length) {
    list.innerHTML = '<div class="session-empty">No recordings yet</div>';
    return;
  }

  list.innerHTML = sessions
    .map((s) => {
      const name = s.meta.name || "Untitled";
      const date = s.meta.createdAt
        ? new Date(s.meta.createdAt).toLocaleDateString()
        : "";
      const dur = formatDuration(s.meta.durationMs || 0);
      const frames = s.meta.frameCount || 0;
      const isActive = playback.activeSessionId === s.id;
      return `<div class="session-item${isActive ? " active" : ""}" data-id="${s.id}">
        <div class="session-info">
          <div class="session-name">${name}</div>
          <div class="session-meta">${date} &middot; ${dur} &middot; ${frames} frames</div>
        </div>
        <div class="session-actions">
          <button class="session-play" title="Play">&#9654;</button>
          <button class="session-delete" title="Delete">&times;</button>
        </div>
      </div>`;
    })
    .join("");

  // Bind play/delete buttons
  list.querySelectorAll(".session-play").forEach((btn) => {
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      const id = btn.closest(".session-item").dataset.id;
      // Mark active
      list.querySelectorAll(".session-item").forEach((el) => el.classList.remove("active"));
      btn.closest(".session-item").classList.add("active");
      requestLoadSession(id);
    });
  });

  list.querySelectorAll(".session-delete").forEach((btn) => {
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      const id = btn.closest(".session-item").dataset.id;
      if (confirm("Delete this recording?")) {
        if (playback.activeSessionId === id) exitSessionPlayback();
        requestDeleteSession(id);
      }
    });
  });
}

async function loadCsvPlayback(url) {
  try {
    const resp = await fetch(url, { cache: "no-store" });
    if (!resp.ok) {
      throw new Error(`CSV load failed: ${resp.status}`);
    }
    const text = await resp.text();
    const lines = text.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
    const rows = [];
    for (let i = 1; i < lines.length; i += 1) {
      const parts = lines[i].split(",").map((val) => val.trim());
      if (parts.length < 6) {
        continue;
      }
      const row = {
        time_s: Number(parts[0]),
        squat_pct: Number(parts[1]),
        lean_deg: Number(parts[2]),
        torso_rot: Number(parts[3]),
        pitch: Number(parts[4]),
        roll: Number(parts[5]),
      };
      if (Number.isFinite(row.time_s)) {
        rows.push(row);
      }
    }
    if (rows.length < 2) {
      throw new Error("CSV missing usable rows.");
    }
    rows.sort((a, b) => a.time_s - b.time_s);
    playback.data = rows;
    playback.duration = rows[rows.length - 1].time_s;
    playback.index = 0;
    playback.lastT = 0;
    playback.startTime = performance.now();
    playback.hasData = true;
    return true;
  } catch (err) {
    console.warn(err);
    playback.hasData = false;
    return false;
  }
}

function bindUI() {
  const bindRange = (id, key) => {
    const input = document.getElementById(id);
    const valueEl = document.getElementById(`${id}Value`);
    input.addEventListener("input", () => {
      state[key] = Number(input.value);
      valueEl.textContent = input.value;
      updateModel();
    });
  };

  bindRange("squat", "squatPct");
  bindRange("edge", "leanDeg");
  bindRange("rotation", "torsoRot");
  bindRange("pitch", "pitch");
  bindRange("roll", "roll");
  bindRange("slope", "slopeDeg");

  const resetBtn = document.getElementById("reset");
  resetBtn.addEventListener("click", () => {
    state.squatPct = 0;
    state.leanDeg = 0;
    state.torsoRot = 0;
    state.pitch = 0;
    state.roll = 0;
    state.slopeDeg = 0;
    state.isToeside = false;
    trail.points = []; trail.segments = []; trail._lastTime = 0;
    syncUI();
    updateModel();
  });

  const playToggle = document.getElementById("playToggle");
  playToggle.addEventListener("click", () => {
    playback.playing = !playback.playing;
    if (playback.playing) {
      playback.startTime = performance.now() - playback.lastT * 1000;
      playToggle.textContent = "Pause";
    } else {
      playToggle.textContent = "Play";
    }
  });

  // Camera presets
  const camTarget = new THREE.Vector3(0, 0, 0.6);
  const camPresets = {
    front: { pos: new THREE.Vector3(-2.5, 0, 0.8), target: camTarget },
    top: { pos: new THREE.Vector3(0, 0, 3.5), target: camTarget },
    behind: { pos: new THREE.Vector3(1.4, -1.6, 1.1), target: camTarget },
    pov: { pos: new THREE.Vector3(0.05, -0.05, 1.6), target: new THREE.Vector3(0, 0, 0) },
  };

  function setCameraPreset(preset) {
    const p = camPresets[preset];
    if (!p) return;
    camera.position.copy(p.pos);
    controls.target.copy(p.target);
    controls.update();
  }

  document.getElementById("camFront").addEventListener("click", () => setCameraPreset("front"));
  document.getElementById("camTop").addEventListener("click", () => setCameraPreset("top"));
  document.getElementById("camBehind").addEventListener("click", () => setCameraPreset("behind"));
  document.getElementById("camPov").addEventListener("click", () => setCameraPreset("pov"));

  // Auto-orbit toggle
  let autoOrbitActive = false;
  const camOrbitBtn = document.getElementById("camOrbit");
  if (camOrbitBtn) {
    camOrbitBtn.addEventListener("click", () => {
      autoOrbitActive = !autoOrbitActive;
      controls.autoRotate = autoOrbitActive;
      controls.autoRotateSpeed = 2.0;
      camOrbitBtn.classList.toggle("active", autoOrbitActive);
    });
  }
  controls.addEventListener("start", () => {
    if (autoOrbitActive) {
      autoOrbitActive = false;
      controls.autoRotate = false;
      if (camOrbitBtn) camOrbitBtn.classList.remove("active");
    }
  });

  const liveToggle = document.getElementById("liveToggle");
  if (liveToggle) {
    liveToggle.addEventListener("click", () => {
      liveWS.enabled = !liveWS.enabled;
      if (liveWS.enabled) {
        liveToggle.textContent = "Live: ON";
        connectLiveWS();
      } else {
        liveToggle.textContent = "Live: OFF";
        if (liveWS.ws) liveWS.ws.close();
        if (liveWS.reconnectTimer) {
          clearTimeout(liveWS.reconnectTimer);
          liveWS.reconnectTimer = null;
        }
        updateConnectionStatus("disconnected");
      }
    });
  }
}

function handleResize() {
  const rect = container.getBoundingClientRect();
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

window.addEventListener("resize", handleResize);

window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    const overlay = document.getElementById("sessionSummaryOverlay");
    if (overlay) {
      hideSessionSummary();
      _finishSessionSave();
      return;
    }
  }
  if (e.key === "c" || e.key === "C") {
    if (liveWS.ws && liveWS.ws.readyState === WebSocket.OPEN) {
      liveWS.ws.send(JSON.stringify({ action: "calibrate" }));
      const label = document.getElementById("liveLabel");
      if (label) {
        label.textContent = "CALIBRATING...";
        setTimeout(() => {
          if (liveWS.connected) label.textContent = "LIVE";
        }, 6000);
      }
    }
  }
  if (e.key === "t" || e.key === "T") {
    trail.enabled = !trail.enabled;
    if (trail.mesh) trail.mesh.visible = trail.enabled;
    if (!trail.enabled) { trail.points = []; trail.segments = []; trail._lastTime = 0; }
  }
});

// ── Live WebSocket Client ──────────────────────────────────────────────────

function updateConnectionStatus(status) {
  const indicator = document.getElementById("liveIndicator");
  const label = document.getElementById("liveLabel");
  if (!indicator || !label) return;

  if (status === "connected") {
    indicator.style.background = "#00ff88";
    indicator.style.boxShadow = "0 0 8px #00ff88";
    label.textContent = "LIVE";
  } else {
    indicator.style.background = "#ff4757";
    indicator.style.boxShadow = "none";
    label.textContent = "OFFLINE";
  }
}

function handleConfigMessage(msg) {
  if (msg.dims) {
    const d = msg.dims;
    if (d.standingHeight != null) dims.standingHeight = d.standingHeight;
    if (d.torsoLength != null) dims.torsoLength = d.torsoLength;
    if (d.minHeight != null) dims.minHeight = d.minHeight;
    if (d.shinLength != null) dims.shinLength = d.shinLength;
    if (d.upperArmLen != null) dims.upperArmLen = d.upperArmLen;
    if (d.forearmLen != null) dims.forearmLen = d.forearmLen;
    if (d.shoulderSpread != null) dims.shoulderSpread = d.shoulderSpread;
  }
  if (msg.ranges) {
    if (msg.ranges.maxLeanDeg != null) dims.maxLeanDeg = msg.ranges.maxLeanDeg;
  }
  updateModel();
}

function updateIMUStatus(boardConnected, bodyConnected) {
  const boardDot = document.getElementById("boardDot");
  const bodyDot = document.getElementById("bodyDot");
  if (boardDot) {
    boardDot.style.background = boardConnected ? "#00ff88" : "#ff4757";
    boardDot.style.boxShadow = boardConnected ? "0 0 6px #00ff88" : "none";
  }
  if (bodyDot) {
    bodyDot.style.background = bodyConnected ? "#00ff88" : "#ff4757";
    bodyDot.style.boxShadow = bodyConnected ? "0 0 6px #00ff88" : "none";
  }
}

function handleStateMessage(msg) {
  state.squatPct = msg.squatPct;
  state.leanDeg = msg.leanDeg;
  state.torsoRot = msg.torsoRot;
  state.pitch = msg.pitch;
  state.roll = msg.roll;
  if (msg.slopeDeg != null) state.slopeDeg = msg.slopeDeg;
  if (msg.boardAccelFwd != null) state.boardAccelFwd = msg.boardAccelFwd;
  if (msg.boardConnected !== undefined) {
    updateIMUStatus(msg.boardConnected, msg.bodyConnected);
  }
  if (recording.active) {
    recording.frames.push({
      t: Math.round(performance.now() - recording.startTime),
      s: [state.squatPct, state.leanDeg, state.torsoRot, state.pitch, state.roll, state.slopeDeg, state.boardAccelFwd],
    });
  }
  syncUI();
  updateModel();
}

// ── Recording Functions ──────────────────────────────────────────────────

function startRecording() {
  recording.frames = [];
  recording.startTime = performance.now();
  recording.dimsSnapshot = { ...dims };
  recording.active = true;

  // Update UI
  const recBtn = document.getElementById("recordBtn");
  const stopBtn = document.getElementById("stopRecordBtn");
  const recTimer = document.getElementById("recTimer");
  const recTimerText = document.getElementById("recTimerText");
  if (recBtn) recBtn.style.display = "none";
  if (stopBtn) stopBtn.style.display = "";
  if (recTimer) recTimer.style.display = "";

  recording.timerInterval = setInterval(() => {
    const elapsed = Math.round((performance.now() - recording.startTime) / 1000);
    const min = Math.floor(elapsed / 60);
    const sec = elapsed % 60;
    if (recTimerText) recTimerText.textContent = `${min}:${String(sec).padStart(2, "0")}`;
  }, 500);
}

function stopRecording() {
  recording.active = false;
  clearInterval(recording.timerInterval);
  if (recording.timedTimeout) {
    clearTimeout(recording.timedTimeout);
    recording.timedTimeout = null;
  }

  // Update UI
  const recBtn = document.getElementById("recordBtn");
  const stopBtn = document.getElementById("stopRecordBtn");
  const recTimer = document.getElementById("recTimer");
  if (recBtn) recBtn.style.display = "";
  if (stopBtn) stopBtn.style.display = "none";
  if (recTimer) recTimer.style.display = "none";

  if (recording.frames.length < 2) {
    console.warn("Recording too short, discarding");
    return null;
  }

  const lastFrame = recording.frames[recording.frames.length - 1];
  return {
    version: 1,
    meta: {
      name: "",
      createdAt: new Date().toISOString(),
      durationMs: lastFrame.t,
      frameCount: recording.frames.length,
    },
    config: { dims: recording.dimsSnapshot },
    frames: recording.frames,
  };
}

// ── Session Metrics ───────────────────────────────────────────────────────

function _mean(arr) { return arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0; }
function _stdDev(arr) {
  const m = _mean(arr);
  return Math.sqrt(arr.reduce((s, v) => s + (v - m) ** 2, 0) / arr.length);
}
function _pearsonCorr(x, y) {
  const mx = _mean(x), my = _mean(y);
  let num = 0, dx = 0, dy = 0;
  for (let i = 0; i < x.length; i++) {
    num += (x[i] - mx) * (y[i] - my);
    dx += (x[i] - mx) ** 2;
    dy += (y[i] - my) ** 2;
  }
  return dx && dy ? num / Math.sqrt(dx * dy) : 0;
}
function _percentile(sorted, p) {
  const idx = Math.floor(p / 100 * (sorted.length - 1));
  return sorted[Math.min(idx, sorted.length - 1)];
}

function computeSessionMetrics(session) {
  const frames = session.frames;
  const n = frames.length;
  if (n < 2) return null;

  const squats  = frames.map(f => f.s[0]);
  const leans   = frames.map(f => f.s[1]);
  const pitches = frames.map(f => f.s[3]);
  const rolls   = frames.map(f => f.s[4]);
  const slopes  = frames.map(f => f.s[5]);
  const durationS = (session.meta.durationMs || 0) / 1000;

  // 1. Peak Lean
  const absLeans = leans.map(Math.abs);
  const peakLean = Math.max(...absLeans);

  // 2. Total Turns — sign changes with 3° deadzone
  const inCarve = leans.filter(l => Math.abs(l) > 3.0);
  let turns = 0;
  for (let i = 1; i < inCarve.length; i++) {
    if (Math.sign(inCarve[i]) !== Math.sign(inCarve[i - 1]) && Math.sign(inCarve[i]) !== 0) turns++;
  }

  // 3. Squat-Carve Sync
  const sync = n > 10 ? _pearsonCorr(squats, absLeans) : 0;

  // 4. Form Score (0-100)
  const meanAbsLean = _mean(absLeans);
  const maxLean = session.config?.dims?.maxLeanDeg || 15;
  const carveScore = Math.min(100, meanAbsLean / maxLean * 100) * 0.30;

  const carvingIdx = [];
  leans.forEach((l, i) => { if (Math.abs(l) > 2) carvingIdx.push(i); });
  const sqInCarve = carvingIdx.map(i => squats[i]);
  const sqScore = (sqInCarve.length > 0 ? _mean(sqInCarve) : 0) * 0.30;

  const win = 30;
  const jitters = [];
  for (let i = 0; i + win <= n; i += win) {
    jitters.push(_stdDev(pitches.slice(i, i + win)) + _stdDev(rolls.slice(i, i + win)));
  }
  const stability = jitters.length > 0 ? Math.max(0, 100 - _mean(jitters) * 10) : 50;
  const stabScore = stability * 0.20;
  const syncScore = Math.max(0, sync * 100) * 0.20;
  const formScore = Math.min(100, Math.max(0, carveScore + sqScore + stabScore + syncScore));

  // 5. Ride Style Tag
  const avgSquat = _mean(squats);
  let style;
  if (formScore > 75) style = "FLOW STATE";
  else if (meanAbsLean > 7 && avgSquat > 25) style = "CARVER";
  else if (meanAbsLean > 4 && avgSquat < 15) style = "STIFF RIDER";
  else style = "CRUISER";

  // 6-7. Slope metrics
  const absSlopes = slopes.map(Math.abs);
  const avgSlopeDeg = _mean(absSlopes);
  const maxSlopeDeg = Math.max(...absSlopes);
  const toGrade = deg => Math.tan(deg * Math.PI / 180) * 100;
  const sorted = [...absSlopes].sort((a, b) => a - b);

  // 8-9. Speed & Acceleration from board accelerometer
  const accels = frames.map(f => f.s[6] || 0);
  const hasAccel = accels.some(a => a !== 0);
  let maxAccelMps2 = 0, maxSpeedMps = 0, maxSpeedKph = 0;
  if (hasAccel) {
    maxAccelMps2 = Math.max(...accels.map(Math.abs));
    // Speed via trapezoidal integration with friction damping
    const dt = 1 / 30;  // ~30Hz
    let speed = 0;
    let peakSpeed = 0;
    for (let i = 0; i < accels.length; i++) {
      speed += accels[i] * dt;
      speed *= 0.998;             // Light friction damping per frame
      if (speed < 0) speed = 0;   // Can't go negative
      if (speed > peakSpeed) peakSpeed = speed;
    }
    maxSpeedMps = peakSpeed;
    maxSpeedKph = peakSpeed * 3.6;
  }

  return {
    durationS:      Math.round(durationS * 10) / 10,
    peakLeanDeg:    Math.round(peakLean * 10) / 10,
    totalTurns:     turns,
    squatCarveSync: Math.round(sync * 100) / 100,
    formScore:      Math.round(formScore * 10) / 10,
    rideStyle:      style,
    avgSlopeDeg:    Math.round(avgSlopeDeg * 10) / 10,
    avgSlopePct:    Math.round(toGrade(avgSlopeDeg) * 10) / 10,
    maxSlopeDeg:    Math.round(maxSlopeDeg * 10) / 10,
    maxSlopePct:    Math.round(toGrade(maxSlopeDeg) * 10) / 10,
    slopeP50:       Math.round(_percentile(sorted, 50) * 10) / 10,
    slopeP75:       Math.round(_percentile(sorted, 75) * 10) / 10,
    slopeP90:       Math.round(_percentile(sorted, 90) * 10) / 10,
    maxAccelMps2:   Math.round(maxAccelMps2 * 100) / 100,
    maxAccelG:      Math.round(maxAccelMps2 / 9.81 * 100) / 100,
    maxSpeedMps:    Math.round(maxSpeedMps * 10) / 10,
    maxSpeedKph:    Math.round(maxSpeedKph * 10) / 10,
    hasAccelData:   hasAccel,
  };
}

// ── Session Summary Modal ────────────────────────────────────────────────

let _pendingSession = null;

function _scoreColor(score) {
  if (score >= 70) return "#00ff88";
  if (score >= 40) return "#fdcb6e";
  return "#ff4757";
}

function _styleColor(style) {
  const map = { "FLOW STATE": "#00ff88", "CARVER": "#00cec9", "STIFF RIDER": "#fdcb6e", "CRUISER": "#a8b2c2" };
  return map[style] || "#a8b2c2";
}

function _fmtDuration(s) {
  const m = Math.floor(s / 60);
  const sec = Math.round(s % 60);
  return m > 0 ? `${m}m ${sec}s` : `${sec}s`;
}

function showSessionSummary(metrics) {
  // Remove existing modal if any
  hideSessionSummary();

  const overlay = document.createElement("div");
  overlay.id = "sessionSummaryOverlay";
  overlay.className = "summary-overlay";

  const sc = _scoreColor(metrics.formScore);
  const stc = _styleColor(metrics.rideStyle);
  const syncPct = Math.round(metrics.squatCarveSync * 100);
  const syncLabel = syncPct >= 0 ? `+${syncPct}%` : `${syncPct}%`;

  overlay.innerHTML = `
    <div class="summary-card">
      <div class="summary-header">
        <div class="summary-title">SESSION SUMMARY</div>
        <span class="summary-badge" style="background:${stc}20;color:${stc};border-color:${stc}40">${metrics.rideStyle}</span>
      </div>

      <div class="summary-hero">
        <div class="summary-score" style="color:${sc}">${metrics.formScore}</div>
        <div class="summary-score-label">FORM SCORE</div>
      </div>

      <div class="summary-metrics">
        <div class="summary-metric">
          <div class="summary-metric-value">${metrics.peakLeanDeg}&deg;</div>
          <div class="summary-metric-label">Peak Lean</div>
        </div>
        <div class="summary-metric">
          <div class="summary-metric-value">${metrics.totalTurns}</div>
          <div class="summary-metric-label">Turns</div>
        </div>
        <div class="summary-metric">
          <div class="summary-metric-value">${syncLabel}</div>
          <div class="summary-metric-label">Squat-Carve Sync</div>
        </div>
      </div>

      <div class="summary-slope-section">
        <div class="summary-slope-title">SLOPE</div>
        <div class="summary-slope-row">
          <div class="summary-slope-item">
            <span class="summary-slope-val">${metrics.avgSlopeDeg}&deg;</span>
            <span class="summary-slope-sub">(${metrics.avgSlopePct}% grade)</span>
            <span class="summary-slope-label">avg</span>
          </div>
          <div class="summary-slope-item">
            <span class="summary-slope-val">${metrics.maxSlopeDeg}&deg;</span>
            <span class="summary-slope-sub">(${metrics.maxSlopePct}% grade)</span>
            <span class="summary-slope-label">max</span>
          </div>
        </div>
        <div class="summary-percentiles">
          P50: ${metrics.slopeP50}&deg; &middot; P75: ${metrics.slopeP75}&deg; &middot; P90: ${metrics.slopeP90}&deg;
        </div>
      </div>

      ${metrics.hasAccelData ? `
      <div class="summary-slope-section">
        <div class="summary-slope-title">SPEED &amp; ACCEL</div>
        <div class="summary-slope-row">
          <div class="summary-slope-item">
            <span class="summary-slope-val">${metrics.maxSpeedMps}</span>
            <span class="summary-slope-sub">m/s</span>
            <span class="summary-slope-label">est. max speed</span>
          </div>
          <div class="summary-slope-item">
            <span class="summary-slope-val">${metrics.maxAccelG}</span>
            <span class="summary-slope-sub">G</span>
            <span class="summary-slope-label">peak accel</span>
          </div>
        </div>
      </div>
      ` : ""}

      <div class="summary-footer">
        <span class="summary-duration">${_fmtDuration(metrics.durationS)}</span>
        <span class="summary-dismiss-hint">Press ESC to close</span>
      </div>
    </div>
  `;

  document.body.appendChild(overlay);

  // Click outside card to dismiss
  overlay.addEventListener("click", (e) => {
    if (e.target === overlay) {
      hideSessionSummary();
      _finishSessionSave();
    }
  });

  // Trigger animation on next frame
  requestAnimationFrame(() => overlay.classList.add("visible"));
}

function hideSessionSummary() {
  const overlay = document.getElementById("sessionSummaryOverlay");
  if (overlay) overlay.remove();
}

function _finishSessionSave() {
  if (!_pendingSession) return;
  const session = _pendingSession;
  _pendingSession = null;
  const defaultName = new Date().toLocaleString().replace(/[/:]/g, "-");
  const name = prompt("Name this recording:", defaultName);
  if (name !== null) {
    session.meta.name = name || defaultName;
    saveSession(session);
  }
}

function startTimedRecording(durationSec) {
  startRecording();
  recording.timedTimeout = setTimeout(() => {
    recording.timedTimeout = null;
    const session = stopRecording();
    if (session) {
      session.meta.name = `${durationSec}s recording`;
      saveSession(session);
    }
  }, durationSec * 1000);
}

function wsSend(obj) {
  if (liveWS.ws && liveWS.ws.readyState === WebSocket.OPEN) {
    liveWS.ws.send(JSON.stringify(obj));
  }
}

function saveSession(session) {
  wsSend({ action: "save_session", session });
}

function requestSessionList() {
  wsSend({ action: "list_sessions" });
}

function requestLoadSession(id) {
  wsSend({ action: "load_session", id });
}

function requestDeleteSession(id) {
  wsSend({ action: "delete_session", id });
}

function scheduleReconnect() {
  if (liveWS.reconnectTimer) return;
  liveWS.reconnectTimer = setTimeout(() => {
    liveWS.reconnectTimer = null;
    connectLiveWS();
  }, 2000);
}

function connectLiveWS() {
  if (!liveWS.enabled) return;
  if (liveWS.ws && liveWS.ws.readyState === WebSocket.OPEN) return;

  const host = window.location.hostname || "localhost";
  const url = `ws://${host}:9093`;

  try {
    liveWS.ws = new WebSocket(url);

    liveWS.ws.onopen = () => {
      liveWS.connected = true;
      updateConnectionStatus("connected");
      playback.playing = false;
      const playToggle = document.getElementById("playToggle");
      if (playToggle) playToggle.textContent = "Play";
      requestSessionList();
    };

    liveWS.ws.onmessage = (event) => {
      const msg = JSON.parse(event.data);
      if (msg.type === "config") {
        handleConfigMessage(msg);
      } else if (msg.type === "state") {
        if (!playback.activeSessionId) {
          handleStateMessage(msg);
        }
      } else if (msg.type === "session_saved") {
        requestSessionList();
      } else if (msg.type === "session_list") {
        renderSessionList(msg.sessions);
      } else if (msg.type === "session_data") {
        loadSessionIntoPlayback(msg.session, msg.id);
      } else if (msg.type === "session_deleted") {
        requestSessionList();
      }
    };

    liveWS.ws.onclose = () => {
      liveWS.connected = false;
      updateConnectionStatus("disconnected");
      scheduleReconnect();
    };

    liveWS.ws.onerror = () => {
      // onclose will fire after this
    };
  } catch (err) {
    updateConnectionStatus("disconnected");
    scheduleReconnect();
  }
}

// ── Initialization ─────────────────────────────────────────────────────────

// Cache telemetry card DOM refs
['carveState', 'carveSubtitle', 'tSquat', 'tLean', 'tKnee', 'tRotation', 'tHipShift', 'tPitch', 'tSlope'].forEach(id => {
  telemEls[id] = document.getElementById(id);
});

// Debug panel toggle
const debugToggle = document.getElementById('debugToggle');
const debugPanel = document.getElementById('debugPanel');
if (debugToggle && debugPanel) {
  debugToggle.addEventListener('click', () => {
    const opening = debugPanel.classList.contains('collapsed');
    debugPanel.classList.toggle('collapsed');
    debugToggle.classList.toggle('active');
    axes.visible = opening;
    arrowEye.visible = opening;
    wireframeLines.forEach(l => l.visible = opening);
  });
}

bindUI();
syncUI();
updateModel();

// ── Session Recording & Playback UI Bindings ──────────────────────────────

// Record / Stop buttons
const recordBtn = document.getElementById("recordBtn");
const stopRecordBtn = document.getElementById("stopRecordBtn");

if (recordBtn) {
  recordBtn.addEventListener("click", () => {
    startRecording();
  });
}

if (stopRecordBtn) {
  stopRecordBtn.addEventListener("click", () => {
    const session = stopRecording();
    if (session) {
      const metrics = computeSessionMetrics(session);
      if (metrics) {
        session.metrics = metrics;
        _pendingSession = session;
        showSessionSummary(metrics);
      } else {
        // Too short for metrics, go straight to save
        const defaultName = new Date().toLocaleString().replace(/[/:]/g, "-");
        const name = prompt("Name this recording:", defaultName);
        if (name !== null) {
          session.meta.name = name || defaultName;
          saveSession(session);
        }
      }
    }
  });
}

// Sessions panel toggle
const sessionsToggle = document.getElementById("sessionsToggle");
const sessionsPanel = document.getElementById("sessionsPanel");
if (sessionsToggle && sessionsPanel) {
  sessionsToggle.addEventListener("click", () => {
    const opening = sessionsPanel.classList.contains("collapsed");
    sessionsPanel.classList.toggle("collapsed");
    sessionsToggle.classList.toggle("active");
    if (opening) requestSessionList();
  });
}

// Playback transport: play/pause
const pbPlayPause = document.getElementById("pbPlayPause");
if (pbPlayPause) {
  pbPlayPause.addEventListener("click", () => {
    if (!playback.sessionMode) return;
    playback.playing = !playback.playing;
    if (playback.playing) {
      playback.startWallTime = performance.now();
    }
    updatePlaybackUI();
  });
}

// Playback transport: scrub bar
const pbScrub = document.getElementById("pbScrub");
if (pbScrub) {
  pbScrub.addEventListener("input", () => {
    if (!playback.sessionMode) return;
    playback.currentTime = (pbScrub.value / 1000) * playback.duration;
    playback.index = 0;
    playback.startWallTime = performance.now();
    const sample = sampleAtMs(playback.currentTime);
    if (sample) {
      applyStateFromSample(sample);
      syncUI();
      trail.points = []; trail.segments = []; trail._lastTime = 0;
      updateModel();
    }
    updatePlaybackUI();
  });
}

// Playback transport: speed buttons
document.querySelectorAll(".speed-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    playback.speed = parseFloat(btn.dataset.speed);
    playback.startWallTime = performance.now();
    document.querySelectorAll(".speed-btn").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
  });
});

// Back to Live button
const backToLive = document.getElementById("backToLive");
if (backToLive) {
  backToLive.addEventListener("click", () => {
    exitSessionPlayback();
  });
}

// Timed recording buttons (debug panel)
const rec10Btn = document.getElementById("rec10");
const rec30Btn = document.getElementById("rec30");
if (rec10Btn) rec10Btn.addEventListener("click", () => startTimedRecording(10));
if (rec30Btn) rec30Btn.addEventListener("click", () => startTimedRecording(30));

// ── Start ─────────────────────────────────────────────────────────────────

connectLiveWS();

loadCsvPlayback("/inputs/board_viz.csv").then((ok) => {
  if (!ok) {
    const playToggle = document.getElementById("playToggle");
    playToggle.textContent = "Play";
  }
});

function animate() {
  requestAnimationFrame(animate);
  tickPlayback();
  updateTrail();
  controls.update();
  renderer.render(scene, camera);
}

animate();
