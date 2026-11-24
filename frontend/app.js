/**
 * DARAR Coherent Radar Digital Twin Simulation Engine
 * Based on: "Distributed coherent detection method for rotating array radar"
 * Journal of Systems Engineering and Electronics, Vol. 37, No. 5, October 2026.
 */

// Global State
const state = {
  mode: 'guided', // 'guided' | 'lab'
  currentStage: 1,
  isRunning: true,
  autoTourActive: false,
  autoTourTimer: null,
  ws: null,
  
  // Simulation parameters
  params: {
    baseline_length: 60.0,
    drone_distance: 10000.0,
    drone_speed: 20.0,
    drone_altitude: 500.0,
    snr_db: 10.0,
    rcc_window_points: 5,
    rotation_speed_rpm: 10.0,
    carrier_freq_hz: 5.5e9,
    signal_bandwidth_hz: 5.0e6,
    ins_error_m: 0.5,
    phase_jitter_deg: 5.0
  },
  
  // Last received full cycle telemetry
  telemetry: null,
  
  // 3D scene objects
  three: {
    scene: null,
    camera: null,
    renderer: null,
    controls: null,
    radarGroups: [],
    radarTurrets: [],
    radarBeams: [],
    droneMesh: null,
    dronePropellers: [],
    wavefronts: [],
    showWavefronts: true,
    showBeams: true,
    targetTrajectoryLine: null
  },
  
  // Active oscilloscope view mode
  oscViewMode: 'rcc', // 'raw' | 'coarse' | 'rcc'
  selectedRccChannel: 'S22'
};

// Colors matching HUD theme
const PALETTE = {
  cyan: '#00f0ff',
  green: '#00ff66',
  amber: '#ffb700',
  rose: '#ff0055',
  purple: '#a855f7',
  blue: '#3b82f6',
  slate: '#64748b',
  bgDark: '#050b14',
  grid: '#142b4d'
};

const CHANNEL_COLORS = [
  '#00f0ff', '#38bdf8', '#0284c7', // S11, S12, S13 (Cyan hues)
  '#00ff66', '#4ade80', '#16a34a', // S21, S22, S23 (Emerald hues)
  '#ffb700', '#f59e0b', '#d97706'  // S31, S32, S33 (Amber hues)
];

// Initialize on DOM load
window.addEventListener('DOMContentLoaded', () => {
  initIcons();
  initThreeJS();
  initEventHandlers();
  initWebSocket();
  fetchLiveAdsbTargets();
  fetchAntennaPattern();
  fetchSimulationCycle();
  renderStageGuidance(state.currentStage);
});

function initIcons() {
  if (window.lucide) {
    window.lucide.createIcons();
  }
}

// ==========================================
// 1. THREE.JS 3D VIRTUAL RADAR ENVIRONMENT
// ==========================================
function initThreeJS() {
  const container = document.getElementById('threejs-container');
  const width = container.clientWidth || 800;
  const height = container.clientHeight || 520;

  // Scene
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050b14);
  scene.fog = new THREE.FogExp2(0x050b14, 0.0008);
  state.three.scene = scene;

  // Camera
  const camera = new THREE.PerspectiveCamera(45, width / height, 1, 50000);
  camera.position.set(0, 180, 360);
  state.three.camera = camera;

  // Renderer
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);
  state.three.renderer = renderer;

  // OrbitControls
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below ground
  controls.minDistance = 15;
  controls.maxDistance = 2500;
  controls.target.set(0, 20, 0);
  state.three.controls = controls;

  // Lighting
  const ambientLight = new THREE.AmbientLight(0x1e3d6b, 1.2);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(100, 300, 100);
  dirLight.castShadow = true;
  scene.add(dirLight);

  const cyanPoint = new THREE.PointLight(0x00f0ff, 2.0, 500);
  cyanPoint.position.set(0, 50, 0);
  scene.add(cyanPoint);

  // Terrain Grid
  buildTerrainGrid(scene);

  // Central Command Bunker
  buildCentralStation(scene);

  // 3 Radar Stations
  buildRadarStations(scene);

  // Target Drone (UAV)
  buildTargetDrone(scene);

  // Window Resize
  window.addEventListener('resize', onWindowResize);

  // Animation Loop
  animateThreeJS();
}

function buildTerrainGrid(scene) {
  // Dual-layer terrain grid
  const gridHelper = new THREE.GridHelper(1200, 60, 0x00f0ff, 0x142b4d);
  gridHelper.position.y = 0;
  scene.add(gridHelper);

  const subGrid = new THREE.GridHelper(1200, 240, 0x142b4d, 0x0a1628);
  subGrid.position.y = -0.1;
  scene.add(subGrid);

  // Baseline Axis Marker
  const baselineGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-450, 0.2, 0),
    new THREE.Vector3(450, 0.2, 0)
  ]);
  const baselineMat = new THREE.LineDashedMaterial({ color: 0x00f0ff, dashSize: 10, gapSize: 5 });
  const baselineLine = new THREE.Line(baselineGeo, baselineMat);
  baselineLine.computeLineDistances();
  scene.add(baselineLine);
}

function buildCentralStation(scene) {
  const centralGroup = new THREE.Group();
  centralGroup.position.set(0, 0, -20);

  // Command bunker base
  const bunkerGeo = new THREE.CylinderGeometry(8, 11, 4, 8);
  const bunkerMat = new THREE.MeshStandardMaterial({ color: 0x142b4d, roughness: 0.7, metalness: 0.3 });
  const bunker = new THREE.Mesh(bunkerGeo, bunkerMat);
  bunker.position.y = 2;
  centralGroup.add(bunker);

  // Central Processing Tower
  const towerGeo = new THREE.CylinderGeometry(2, 3, 10, 8);
  const towerMat = new THREE.MeshStandardMaterial({ color: 0x00f0ff, roughness: 0.2, metalness: 0.8 });
  const tower = new THREE.Mesh(towerGeo, towerMat);
  tower.position.y = 9;
  centralGroup.add(tower);

  // Pulsing beacon sphere
  const beaconGeo = new THREE.SphereGeometry(1.2, 16, 16);
  const beaconMat = new THREE.MeshBasicMaterial({ color: 0x00ff66 });
  const beacon = new THREE.Mesh(beaconGeo, beaconMat);
  beacon.position.y = 15;
  centralGroup.add(beacon);

  scene.add(centralGroup);

  // Optical communication link lines from Central Station to Radars
  const linkMat = new THREE.LineBasicMaterial({ color: 0x00ff66, transparent: true, opacity: 0.4 });
  const linkCoords = [
    new THREE.Vector3(0, 5, 0),    // Radar 1
    new THREE.Vector3(-60, 5, 0),  // Radar 2
    new THREE.Vector3(60, 5, 0)    // Radar 3
  ];

  linkCoords.forEach(pos => {
    const geo = new THREE.BufferGeometry().setFromPoints([centralGroup.position, pos]);
    const line = new THREE.Line(geo, linkMat);
    scene.add(line);
  });
}

function buildRadarStations(scene) {
  const radarConfigs = [
    { id: 1, name: "Radar 1 (Ref)", x: 0, color: 0x00f0ff, initialAngle: 0.0 },
    { id: 2, name: "Radar 2 (West)", x: -state.params.baseline_length, color: 0x00ff66, initialAngle: 30.0 },
    { id: 3, name: "Radar 3 (East)", x: state.params.baseline_length, color: 0xffb700, initialAngle: 330.0 }
  ];

  state.three.radarGroups = [];
  state.three.radarTurrets = [];
  state.three.radarBeams = [];

  radarConfigs.forEach(cfg => {
    const group = new THREE.Group();
    group.position.set(cfg.x, 0, 0);

    // Concrete Base Foundation
    const baseGeo = new THREE.CylinderGeometry(5.5, 6.5, 2.5, 16);
    const baseMat = new THREE.MeshStandardMaterial({ color: 0x1e3d6b, roughness: 0.9 });
    const base = new THREE.Mesh(baseGeo, baseMat);
    base.position.y = 1.25;
    group.add(base);

    // Rotating Pedestal Turret
    const turretGroup = new THREE.Group();
    turretGroup.position.set(0, 2.5, 0);

    const pedestalGeo = new THREE.CylinderGeometry(3.5, 3.5, 4, 16);
    const pedestalMat = new THREE.MeshStandardMaterial({ color: 0x0e1f38, metalness: 0.8, roughness: 0.3 });
    const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
    pedestal.position.y = 2;
    turretGroup.add(pedestal);

    // 32-Element Phased Array Antenna Panel
    const arrayGroup = new THREE.Group();
    arrayGroup.position.set(0, 7, 0);

    // Backplane chassis
    const chassisGeo = new THREE.BoxGeometry(14, 8, 1.5);
    const chassisMat = new THREE.MeshStandardMaterial({ color: 0x142b4d, metalness: 0.6, roughness: 0.4 });
    const chassis = new THREE.Mesh(chassisGeo, chassisMat);
    arrayGroup.add(chassis);

    // Antenna array face (32 elements visualized)
    const elementGeo = new THREE.BoxGeometry(0.8, 0.8, 0.4);
    const elementMat = new THREE.MeshBasicMaterial({ color: cfg.color });
    for (let r = 0; r < 4; r++) {
      for (let c = 0; c < 8; c++) {
        const elem = new THREE.Mesh(elementGeo, elementMat);
        elem.position.set((c - 3.5) * 1.5, (r - 1.5) * 1.5, 0.8);
        arrayGroup.add(elem);
      }
    }

    turretGroup.add(arrayGroup);

    // RF Beam Projection Frustum (Radiation Cone)
    const beamGeo = new THREE.ConeGeometry(35, 180, 16, 1, true);
    beamGeo.rotateX(Math.PI / 2); // Point along forward Z
    beamGeo.translate(0, 0, 90);
    const beamMat = new THREE.MeshBasicMaterial({
      color: cfg.color,
      transparent: true,
      opacity: 0.12,
      wireframe: true,
      side: THREE.DoubleSide
    });
    const beam = new THREE.Mesh(beamGeo, beamMat);
    beam.position.set(0, 7, 0);
    turretGroup.add(beam);

    group.add(turretGroup);
    scene.add(group);

    state.three.radarGroups.push(group);
    state.three.radarTurrets.push(turretGroup);
    state.three.radarBeams.push(beam);
  });
}

function buildTargetDrone(scene) {
  const droneGroup = new THREE.Group();
  // Scaled down position for 3D viewport (e.g. 100m represents 10km)
  droneGroup.position.set(0, 45, 140);

  // UAV Body
  const bodyGeo = new THREE.BoxGeometry(3, 0.8, 4);
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xff0055, roughness: 0.4, metalness: 0.7 });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  droneGroup.add(body);

  // UAV Nose
  const noseGeo = new THREE.ConeGeometry(1, 2, 8);
  noseGeo.rotateX(-Math.PI / 2);
  const noseMat = new THREE.MeshStandardMaterial({ color: 0xffffff });
  const nose = new THREE.Mesh(noseGeo, noseMat);
  nose.position.set(0, 0, 3);
  droneGroup.add(nose);

  // 4 Rotor Arms & Propellers
  const armGeo = new THREE.CylinderGeometry(0.2, 0.2, 5, 8);
  const propGeo = new THREE.BoxGeometry(3.5, 0.05, 0.3);
  const propMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, transparent: true, opacity: 0.7 });

  const armOffsets = [
    { x: -2.5, z: 2.5 },
    { x: 2.5, z: 2.5 },
    { x: -2.5, z: -2.5 },
    { x: 2.5, z: -2.5 }
  ];

  state.three.dronePropellers = [];

  armOffsets.forEach(pos => {
    const arm = new THREE.Mesh(armGeo, bodyMat);
    arm.rotation.z = Math.PI / 2;
    arm.rotation.y = Math.atan2(pos.z, pos.x);
    arm.position.set(pos.x / 2, 0, pos.z / 2);
    droneGroup.add(arm);

    const prop = new THREE.Mesh(propGeo, propMat);
    prop.position.set(pos.x, 0.6, pos.z);
    droneGroup.add(prop);
    state.three.dronePropellers.push(prop);
  });

  // Target Beacon Light
  const droneLight = new THREE.PointLight(0xff0055, 3.0, 50);
  droneLight.position.set(0, 2, 0);
  droneGroup.add(droneLight);

  scene.add(droneGroup);
  state.three.droneMesh = droneGroup;
}

// Wavefront pulse emission animation
function emitRFWavefront() {
  if (!state.three.showWavefronts || !state.three.scene) return;

  // Pulse ring traveling towards target
  state.three.radarGroups.forEach((group, idx) => {
    const ringGeo = new THREE.RingGeometry(2, 2.5, 32);
    ringGeo.rotateX(Math.PI / 2);
    const ringMat = new THREE.MeshBasicMaterial({
      color: idx === 0 ? 0x00f0ff : (idx === 1 ? 0x00ff66 : 0xffb700),
      transparent: true,
      opacity: 0.8,
      side: THREE.DoubleSide
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.position.copy(group.position);
    ring.position.y = 8;
    ring.userData = {
      progress: 0.0,
      startPos: group.position.clone().add(new THREE.Vector3(0, 8, 0)),
      isEcho: false,
      nodeIndex: idx
    };
    state.three.scene.add(ring);
    state.three.wavefronts.push(ring);
  });
}

function emitEchoWavefront(targetPos) {
  if (!state.three.showWavefronts || !state.three.scene) return;

  const echoGeo = new THREE.SphereGeometry(1.5, 16, 16);
  const echoMat = new THREE.MeshBasicMaterial({
    color: 0xff0055,
    wireframe: true,
    transparent: true,
    opacity: 0.9
  });
  const echo = new THREE.Mesh(echoGeo, echoMat);
  echo.position.copy(targetPos);
  echo.userData = {
    progress: 0.0,
    startPos: targetPos.clone(),
    isEcho: true
  };
  state.three.scene.add(echo);
  state.three.wavefronts.push(echo);
}

let lastWavefrontTime = 0;

function updateWavefronts(time) {
  if (time - lastWavefrontTime > 1.8) {
    emitRFWavefront();
    lastWavefrontTime = time;
  }

  const dronePos = state.three.droneMesh ? state.three.droneMesh.position : new THREE.Vector3(0, 45, 140);

  for (let i = state.three.wavefronts.length - 1; i >= 0; i--) {
    const wf = state.three.wavefronts[i];
    wf.userData.progress += 0.015;

    if (!wf.userData.isEcho) {
      // Outward transmit pulse
      wf.position.lerpVectors(wf.userData.startPos, dronePos, wf.userData.progress);
      const scale = 1.0 + wf.userData.progress * 12.0;
      wf.scale.set(scale, scale, scale);
      wf.material.opacity = Math.max(0, 0.8 - wf.userData.progress * 0.7);

      if (wf.userData.progress >= 0.95) {
        // Trigger reflection echo at target
        emitEchoWavefront(dronePos);
        state.three.scene.remove(wf);
        wf.geometry.dispose();
        wf.material.dispose();
        state.three.wavefronts.splice(i, 1);
      }
    } else {
      // Returning reflected echo
      const scale = 1.0 + wf.userData.progress * 25.0;
      wf.scale.set(scale, scale, scale);
      wf.material.opacity = Math.max(0, 0.9 - wf.userData.progress * 0.9);

      if (wf.userData.progress >= 1.0) {
        state.three.scene.remove(wf);
        wf.geometry.dispose();
        wf.material.dispose();
        state.three.wavefronts.splice(i, 1);
      }
    }
  }
}

function animateThreeJS(time = 0) {
  requestAnimationFrame(animateThreeJS);

  const tSeconds = time * 0.001;

  // Spin propellers
  state.three.dronePropellers.forEach(prop => {
    prop.rotation.y += 0.4;
  });

  // Bobbing target motion
  if (state.three.droneMesh) {
    state.three.droneMesh.position.x = Math.sin(tSeconds * 0.5) * 35;
    state.three.droneMesh.position.y = 45 + Math.sin(tSeconds * 1.2) * 3;
  }

  // Update RF pulse waves
  updateWavefronts(tSeconds);

  if (state.three.controls) {
    state.three.controls.update();
  }

  if (state.three.renderer && state.three.scene && state.three.camera) {
    state.three.renderer.render(state.three.scene, state.three.camera);
  }
}

function onWindowResize() {
  const container = document.getElementById('threejs-container');
  if (!container || !state.three.camera || !state.three.renderer) return;

  const width = container.clientWidth;
  const height = container.clientHeight;
  state.three.camera.aspect = width / height;
  state.three.camera.updateProjectionMatrix();
  state.three.renderer.setSize(width, height);
}

// Camera presets
function setCameraView(preset) {
  const cam = state.three.camera;
  const ctrl = state.three.controls;
  if (!cam || !ctrl) return;

  switch (preset) {
    case 'orbit':
      cam.position.set(0, 180, 360);
      ctrl.target.set(0, 20, 0);
      break;
    case 'top':
      cam.position.set(0, 420, 10);
      ctrl.target.set(0, 0, 60);
      break;
    case 'drone':
      if (state.three.droneMesh) {
        const p = state.three.droneMesh.position;
        cam.position.set(p.x, p.y + 15, p.z + 40);
        ctrl.target.copy(p);
      }
      break;
    case 'r1':
      cam.position.set(0, 20, 35);
      ctrl.target.set(0, 8, 0);
      break;
    case 'r2':
      cam.position.set(-state.params.baseline_length, 20, 35);
      ctrl.target.set(-state.params.baseline_length, 8, 0);
      break;
    case 'r3':
      cam.position.set(state.params.baseline_length, 20, 35);
      ctrl.target.set(state.params.baseline_length, 8, 0);
      break;
  }
}

// ==========================================
// 2. WEBSOCKET & REST SIMULATION ENGINE SYNC
// ==========================================
function initWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/simulation`;

  try {
    const ws = new WebSocket(wsUrl);
    state.ws = ws;

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      updateLivePhysicalTelemetry(data);
    };

    ws.onerror = () => {
      console.warn("WebSocket stream unavailable; falling back to REST interval.");
    };
  } catch (err) {
    console.warn("WebSocket init error:", err);
  }
}

function updateLivePhysicalTelemetry(data) {
  const surv = data.surveillance;
  const useAsync360 = surv && surv.mode === 'async_360';

  // Update turntable angles in 3D scene
  const angles = useAsync360 ? surv.radar_angles_deg : data.servo_angles;
  if (angles && state.three.radarTurrets.length === 3) {
    angles.forEach((angleDeg, idx) => {
      const turret = state.three.radarTurrets[idx];
      // Three.js Y-rotation in radians
      turret.rotation.y = -THREE.MathUtils.degToRad(angleDeg);
    });
  }

  // Update HUD pointing angles
  const displayAngles = useAsync360 ? surv.radar_angles_deg : data.world_pointing_angles;
  if (displayAngles) {
    document.getElementById('r1-pointing-angle').textContent = `${displayAngles[0].toFixed(1)}°`;
    document.getElementById('r2-pointing-angle').textContent = `${displayAngles[1].toFixed(1)}°`;
    document.getElementById('r3-pointing-angle').textContent = `${displayAngles[2].toFixed(1)}°`;
  }

  // Update spatial sync status badge
  const syncElem = document.getElementById('hud-sync-status');
  if (useAsync360) {
    syncElem.className = 'text-radar-cyan font-semibold flex items-center gap-1.5';
    syncElem.innerHTML = '<span class="w-2 h-2 rounded-full bg-radar-cyan animate-pulse"></span> 360° ASYNC SCAN';
  } else if (data.is_synchronized) {
    syncElem.className = 'text-emerald-400 font-semibold flex items-center gap-1.5';
    syncElem.innerHTML = '<span class="w-2 h-2 rounded-full bg-emerald-400"></span> SYNCHRONIZED (&lt;0.1°)';
  } else {
    syncElem.className = 'text-amber-400 font-semibold flex items-center gap-1.5';
    syncElem.innerHTML = `<span class="w-2 h-2 rounded-full bg-amber-400 animate-ping"></span> CONVERGING (${data.max_error_deg.toFixed(1)}°)`;
  }

  // Update Continuous Surveillance, Fusion & Rapid Alert Console
  if (surv) {
    if (surv.step_hits && surv.step_hits.length > 0 && state.three.droneMesh) {
      emitEchoWavefront(state.three.droneMesh.position);
    }
    updateSurveillanceConsole(surv);
  }
}

async function fetchSimulationCycle() {
  try {
    const res = await fetch('/api/simulation/full_cycle', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(state.params)
    });
    const telemetry = await res.json();
    state.telemetry = telemetry;
    updateSimulationUI(telemetry);
  } catch (err) {
    console.error("Failed to fetch simulation cycle:", err);
  }
}

// ==========================================
// 3. UI TELEMETRY & MULTI-STAGE UPDATER
// ==========================================
function updateSimulationUI(data) {
  if (!data) return;

  const s1 = data.stage1;
  const s2 = data.stage2;
  const s6 = data.stage6;

  // Top HUD
  document.getElementById('hud-target-info').textContent = `UAV @ ${(state.params.drone_distance / 1000).toFixed(1)} km`;
  document.getElementById('hud-gain-val').textContent = `+${s6.rcc_fine.gain_db.toFixed(2)} dB`;
  document.getElementById('hud-eff-val').textContent = `${s6.rcc_fine.efficiency_percent.toFixed(1)}%`;
  document.getElementById('hud-baseline-len').textContent = `${state.params.baseline_length}`;
  document.getElementById('hud-tau11').textContent = `${s2.delays_matrix_us[0][0].toFixed(2)} μs`;
  
  const diffDelay = Math.abs(s2.delays_matrix_us[1][1] - s2.delays_matrix_us[0][0]);
  document.getElementById('hud-diff-delay').textContent = `${diffDelay.toFixed(3)} μs`;

  // Performance Metrics Card
  document.getElementById('metric-single-snr').textContent = `${s6.single_snr_db.toFixed(1)} dB`;
  document.getElementById('metric-synth-snr').textContent = `${s6.rcc_fine.snr_db.toFixed(1)} dB`;
  document.getElementById('metric-actual-gain').textContent = `+${s6.rcc_fine.gain_db.toFixed(2)} dB`;
  document.getElementById('metric-efficiency').textContent = `${s6.rcc_fine.efficiency_percent.toFixed(1)}%`;
  document.getElementById('metric-eff-bar-text').textContent = `${s6.rcc_fine.efficiency_percent.toFixed(1)}%`;
  document.getElementById('metric-eff-bar').style.width = `${Math.min(100, Math.max(0, s6.rcc_fine.efficiency_percent))}%`;

  // Comparison Table
  document.getElementById('cmp-raw-gain').textContent = `+${s6.uncompensated.gain_db.toFixed(2)} dB`;
  document.getElementById('cmp-raw-eff').textContent = `${s6.uncompensated.efficiency_percent.toFixed(1)}%`;
  document.getElementById('cmp-pp-gain').textContent = `+${s6.peak_picking.gain_db.toFixed(2)} dB`;
  document.getElementById('cmp-pp-eff').textContent = `${s6.peak_picking.efficiency_percent.toFixed(1)}%`;
  document.getElementById('cmp-cc-gain').textContent = `+${s6.conventional_cc.gain_db.toFixed(2)} dB`;
  document.getElementById('cmp-cc-eff').textContent = `${s6.conventional_cc.efficiency_percent.toFixed(1)}%`;
  document.getElementById('cmp-rcc-gain').textContent = `+${s6.rcc_fine.gain_db.toFixed(2)} dB`;
  document.getElementById('cmp-rcc-eff').textContent = `${s6.rcc_fine.efficiency_percent.toFixed(1)}%`;

  // Update Oscilloscope Deck
  renderOscilloscopeCanvas(data);

  // Update RCC Diagnostic Plot
  renderRccCorrelationPlot(data);

  // Update Synthesis Comparison Chamber
  renderSynthesisComparisonPlot(data);

  // Update Advanced DSP Workstation (Phasors, Range-Doppler, 3D Loss Surface)
  if (data.analytics) {
    const an = data.analytics;
    const dopplerKhz = (an.doppler_shift_hz / 1000.0).toFixed(2);
    const dopplerSign = an.doppler_shift_hz >= 0 ? '+' : '';
    const dopplerEl = document.getElementById('hud-doppler-val');
    if (dopplerEl) dopplerEl.textContent = `${dopplerSign}${dopplerKhz} kHz`;

    const rdBadge = document.getElementById('rd-doppler-badge');
    if (rdBadge) rdBadge.textContent = `vr: ${an.radial_velocity_ms >= 0 ? '+' : ''}${an.radial_velocity_ms.toFixed(1)} m/s | fd: ${dopplerSign}${dopplerKhz} kHz`;

    const meta = an.target_metadata || {};
    const srcEl = document.getElementById('hud-data-source');
    if (srcEl) {
      if (meta.source === 'opensky_live') {
        srcEl.className = 'text-emerald-400 font-bold flex items-center gap-1';
        srcEl.innerHTML = `<span class="w-2 h-2 rounded-full bg-emerald-400 animate-pulse"></span> OPENSKY LIVE (${meta.callsign})`;
      } else if (meta.source === 'paper_outfield_uav') {
        srcEl.className = 'text-radar-cyan font-bold flex items-center gap-1';
        srcEl.innerHTML = `<span class="w-2 h-2 rounded-full bg-radar-cyan"></span> OUTFIELD UAV (${meta.callsign})`;
      } else if (meta.source === 'recorded_adsb_fallback') {
        srcEl.className = 'text-amber-400 font-bold flex items-center gap-1';
        srcEl.innerHTML = `<span class="w-2 h-2 rounded-full bg-amber-400"></span> ADS-B CACHED (${meta.callsign})`;
      } else {
        srcEl.className = 'text-slate-300 font-bold flex items-center gap-1';
        srcEl.innerHTML = `<span class="w-2 h-2 rounded-full bg-slate-300"></span> SIMULATED UAV`;
      }
    }

    if (meta.callsign) {
      document.getElementById('hud-target-info').textContent = `${meta.callsign} @ ${(state.params.drone_distance / 1000).toFixed(1)} km`;
    }

    renderPhasorPlots(an.iq_phasors);
    renderRangeDopplerPlots(an.range_doppler);
    renderLossSurface3D(an.loss_surface);
  }
}

// ==========================================
// 4. 9-CHANNEL MULTI-TRACE OSCILLOSCOPE DECK
// ==========================================
function renderOscilloscopeCanvas(data) {
  const canvas = document.getElementById('oscilloscope-canvas');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 700;
  const height = canvas.clientHeight || 280;

  canvas.width = width * dpr;
  canvas.height = height * dpr;
  ctx.scale(dpr, dpr);

  // Clear CRT screen
  ctx.fillStyle = '#050b14';
  ctx.fillRect(0, 0, width, height);

  // Draw Phosphor Reticle Grid
  ctx.strokeStyle = '#0e243d';
  ctx.lineWidth = 1;

  const xDivs = 10;
  const yDivs = 8;
  for (let x = 0; x <= width; x += width / xDivs) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += height / yDivs) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  // Target Range Cell Marker line
  const targetBin = data.stage3.center_sample || 30;
  const targetX = (targetBin / data.stage3.sample_points.length) * width;
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.35)';
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(targetX, 0);
  ctx.lineTo(targetX, height);
  ctx.stroke();
  ctx.setLineDash([]);

  // Select signal dataset according to view mode
  let channelsSource;
  if (state.oscViewMode === 'raw') {
    channelsSource = data.stage3.channels;
  } else if (state.oscViewMode === 'coarse') {
    channelsSource = data.stage4.channels;
  } else {
    channelsSource = data.stage5.channels;
  }

  const channelKeys = Object.keys(channelsSource);
  const legendGrid = document.getElementById('channel-legend-grid');
  if (legendGrid && legendGrid.children.length === 0) {
    legendGrid.innerHTML = '';
  }

  channelKeys.forEach((key, idx) => {
    const ch = channelsSource[key];
    const env = ch.envelope;
    const color = CHANNEL_COLORS[idx % CHANNEL_COLORS.length];

    // Draw trace
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.shadowColor = color;
    ctx.shadowBlur = 4;
    ctx.beginPath();

    const maxVal = 35.0; // Max expected amplitude
    const numPoints = env.length;

    for (let i = 0; i < numPoints; i++) {
      const px = (i / (numPoints - 1)) * width;
      const normalizedY = env[i] / maxVal;
      const py = height - (normalizedY * (height * 0.82)) - (height * 0.08);

      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Update legend button
    let legendItem = document.getElementById(`legend-${key}`);
    if (!legendItem && legendGrid) {
      legendItem = document.createElement('div');
      legendItem.id = `legend-${key}`;
      legendItem.className = 'p-1 rounded bg-radar-850/80 border border-radar-800 text-center cursor-pointer hover:border-radar-cyan';
      legendGrid.appendChild(legendItem);
    }
    if (legendItem) {
      legendItem.innerHTML = `
        <div class="font-bold flex items-center justify-center gap-1" style="color: ${color}">
          <span class="w-1.5 h-1.5 rounded-full" style="background: ${color}"></span> ${key}
        </div>
        <div class="text-[9px] text-slate-400">Peak: bin ${ch.peak_index}</div>
      `;
    }
  });
}

// ==========================================
// 5. RCC CORRELATION PLOT (STAGE 5 DEEP DIVE)
// ==========================================
function renderRccCorrelationPlot(data) {
  const plotDiv = document.getElementById('rcc-correlation-plot');
  if (!plotDiv || !data || !data.stage5) return;

  const chKey = state.selectedRccChannel || 'S22';
  const curve = data.stage5.correlation_curves[chKey];
  if (!curve) return;

  const w = data.stage5.search_window_points;
  document.getElementById('rcc-win-label').textContent = `${w}`;

  // Full correlation trace
  const traceFull = {
    x: curve.lags,
    y: curve.corr_abs,
    mode: 'lines',
    name: `|x_{${chKey}}(τ)| Full`,
    line: { color: '#64748b', width: 1.2 }
  };

  // Search region highlighted slice
  const regionLags = [];
  const regionVals = [];
  for (let i = 0; i < curve.lags.length; i++) {
    if (Math.abs(curve.lags[i]) <= w) {
      regionLags.push(curve.lags[i]);
      regionVals.push(curve.corr_abs[i]);
    }
  }

  const traceRCC = {
    x: regionLags,
    y: regionVals,
    mode: 'lines',
    name: `RCC Window [±${w} pts]`,
    line: { color: '#ffb700', width: 2.5 }
  };

  // Detected peak marker
  const tracePeak = {
    x: [curve.peak_lag],
    y: [curve.corr_abs[curve.lags.indexOf(curve.peak_lag)]],
    mode: 'markers+text',
    name: 'Detected RCC Peak',
    text: [`τ = ${curve.peak_lag} pts`],
    textposition: 'top center',
    textfont: { color: '#00f0ff', size: 10 },
    marker: { color: '#00f0ff', size: 8, symbol: 'circle' }
  };

  const layout = {
    margin: { l: 30, r: 10, t: 15, b: 25 },
    paper_bgcolor: '#050b14',
    plot_bgcolor: '#050b14',
    showlegend: false,
    xaxis: {
      title: 'Time Lag τ (sampling points)',
      titlefont: { size: 9, color: '#64748b' },
      tickfont: { size: 9, color: '#64748b' },
      gridcolor: '#142b4d',
      range: [-40, 40]
    },
    yaxis: {
      titlefont: { size: 9, color: '#64748b' },
      tickfont: { size: 9, color: '#64748b' },
      gridcolor: '#142b4d',
      showticklabels: false
    }
  };

  Plotly.react(plotDiv, [traceFull, traceRCC, tracePeak], layout, { responsive: true, displayModeBar: false });
}

// ==========================================
// 6. SYNTHESIS COMPARISON CHAMBER PLOT
// ==========================================
function renderSynthesisComparisonPlot(data) {
  const plotDiv = document.getElementById('synthesis-comparison-plot');
  if (!plotDiv || !data || !data.stage6) return;

  const s6 = data.stage6;
  const sampleIndices = Array.from({ length: s6.rcc_fine.waveform_abs.length }, (_, i) => i);

  const traceRaw = {
    x: sampleIndices,
    y: s6.uncompensated.waveform_abs,
    mode: 'lines',
    name: 'Before Comp (Destructive)',
    line: { color: '#ff0055', dash: 'dot', width: 1.5 }
  };

  const traceRCC = {
    x: sampleIndices,
    y: s6.rcc_fine.waveform_abs,
    mode: 'lines',
    name: 'After RCC (Coherent Addition)',
    line: { color: '#00ff66', width: 2.5 }
  };

  const layout = {
    margin: { l: 30, r: 10, t: 15, b: 25 },
    paper_bgcolor: '#050b14',
    plot_bgcolor: '#050b14',
    showlegend: true,
    legend: {
      x: 0.05,
      y: 0.95,
      font: { size: 9, color: '#cbd5e1' },
      bgcolor: 'rgba(5, 11, 20, 0.7)'
    },
    xaxis: {
      title: 'Range Sampling Points',
      titlefont: { size: 9, color: '#64748b' },
      tickfont: { size: 9, color: '#64748b' },
      gridcolor: '#142b4d'
    },
    yaxis: {
      title: 'Amplitude',
      titlefont: { size: 9, color: '#64748b' },
      tickfont: { size: 9, color: '#64748b' },
      gridcolor: '#142b4d'
    }
  };

  Plotly.react(plotDiv, [traceRaw, traceRCC], layout, { responsive: true, displayModeBar: false });
}

// ==========================================
// 7. FIGURE 13 LIVE RESEARCH BENCHMARK
// ==========================================
async function runFig13Benchmark() {
  const btn = document.getElementById('btn-run-fig13');
  btn.innerHTML = `<i data-lucide="loader-2" class="w-3.5 h-3.5 animate-spin"></i> Calculating Monte Carlo...`;
  initIcons();

  try {
    const res = await fetch('/api/benchmark/fig13');
    const benchmark = await res.json();
    renderFig13BenchmarkPlot(benchmark);
    btn.innerHTML = `<i data-lucide="check" class="w-3.5 h-3.5 text-emerald-400"></i> Benchmark Up to Date`;
  } catch (err) {
    console.error("Benchmark error:", err);
    btn.innerHTML = `<i data-lucide="refresh-cw" class="w-3.5 h-3.5"></i> Retry Benchmark`;
  }
  initIcons();
}

function renderFig13BenchmarkPlot(data) {
  const plotDiv = document.getElementById('fig13-benchmark-plot');
  if (!plotDiv || !data) return;

  const snrs = data.snr_values;

  const tracePP = {
    x: snrs,
    y: data.PP,
    mode: 'lines+markers',
    name: 'Peak Picking (PP) [5, 10]',
    line: { color: '#38bdf8', dash: 'dash', width: 2 },
    marker: { symbol: 'square', size: 6 }
  };

  const traceCC = {
    x: snrs,
    y: data.CC,
    mode: 'lines+markers',
    name: 'Conventional CC [5, 11]',
    line: { color: '#f59e0b', dash: 'dot', width: 2 },
    marker: { symbol: 'circle', size: 6 }
  };

  const traceRCC7 = {
    x: snrs,
    y: data.RCC_7,
    mode: 'lines+markers',
    name: 'Proposed RCC (7 sampling points)',
    line: { color: '#a855f7', width: 2 },
    marker: { symbol: 'x', size: 6 }
  };

  const traceRCC5 = {
    x: snrs,
    y: data.RCC_5,
    mode: 'lines+markers',
    name: 'Proposed RCC (5 sampling points)',
    line: { color: '#ffb700', width: 2.2 },
    marker: { symbol: 'triangle-up', size: 6 }
  };

  const traceRCC3 = {
    x: snrs,
    y: data.RCC_3,
    mode: 'lines+markers',
    name: 'Proposed RCC (3 sampling points)',
    line: { color: '#00ff66', width: 2.5 },
    marker: { symbol: 'diamond', size: 7 }
  };

  const traceTheo = {
    x: snrs,
    y: data.Theoretical,
    mode: 'lines',
    name: 'Theoretical Upper Bound (KL = 9)',
    line: { color: '#00f0ff', width: 1.8, dash: 'dashdot' }
  };

  const layout = {
    margin: { l: 45, r: 20, t: 25, b: 40 },
    paper_bgcolor: '#050b14',
    plot_bgcolor: '#050b14',
    xaxis: {
      title: 'Signal-to-Noise Ratio (SNR) / dB',
      titlefont: { size: 11, color: '#94a3b8' },
      tickfont: { size: 10, color: '#64748b' },
      gridcolor: '#142b4d',
      range: [4, 26]
    },
    yaxis: {
      title: 'Coherent Synthesis Efficiency η / %',
      titlefont: { size: 11, color: '#94a3b8' },
      tickfont: { size: 10, color: '#64748b' },
      gridcolor: '#142b4d',
      range: [0, 105]
    },
    legend: {
      x: 0.55,
      y: 0.25,
      font: { size: 10, color: '#cbd5e1' },
      bgcolor: 'rgba(5, 11, 20, 0.85)',
      bordercolor: '#1e3d6b',
      borderwidth: 1
    }
  };

  Plotly.react(plotDiv, [tracePP, traceCC, traceRCC7, traceRCC5, traceRCC3, traceTheo], layout, { responsive: true });
}

// ==========================================
// 8. 6-STAGE SCIENTIFIC METHODOLOGY GUIDANCE
// ==========================================
const STAGE_GUIDES = {
  1: {
    title: "Stage 1: Spatial Synchronization of Rotating Arrays",
    subtitle: "Parallel Control Model & Alpha-Filter Servo Synchronization (Section 3.1)",
    description: `To achieve coherent detection, the 3 rotating radars must illuminate the target simultaneously. Radars 1, 2, and 3 are initialized with mechanical INS offsets (0°, 30°, and 330°). The central controller broadcasts unified position commands $\\theta^k$, and each servo subsystem applies an $\\alpha$-filter to eliminate pointing error smoothly.`,
    math: `\\psi_i^k = \\alpha \\cdot \\theta_i^k + (1 - \\alpha) \\cdot \\psi_i^{k-1}`,
    action: "Observe the 3 radar arrays rotating and converging until pointing error &lt; 0.1° (loss &lt; 0.15 dB).",
    buttonText: "Verify Spatial Convergence",
    viewPreset: "orbit",
    oscMode: "raw"
  },
  2: {
    title: "Stage 2: Signal Transmission and Propagation Geometry",
    subtitle: "Distributed Electromagnetic Pulse Delay Modeling (Section 2.1)",
    description: `Each radar node emits orthogonal phase-encoded pulses $s_k(t)$ (C-band, 5 MHz bandwidth). The physical distance travelled from transmitting unit $k$ to the aerial drone and back to receiving unit $l$ governs the exact physical transit time $t = d/c$.`,
    math: `\\tau_{lk} = \\frac{R_k + R_l}{c}`,
    action: "Watch RF pulse wavefronts expand from the 3 radar stations, scatter off the drone, and return at different times.",
    buttonText: "Inspect Propagation Delays",
    viewPreset: "drone",
    oscMode: "raw"
  },
  3: {
    title: "Stage 3: Multi-Channel Signal Processing Laboratory",
    subtitle: "Matched-Filter Separation & Destructive Interference (Section 2.1 & 2.2)",
    description: `The central processor matches each received echo against the K transmitted orthogonal codes, producing $K \\times L = 9$ independent channels ($S_{11} \\dots S_{33}$). Notice the severe time misalignments (range bin spread) and random phase offsets. Directly adding these echoes leads to severe destructive cancellation.`,
    math: `s_{lk}^m(t) = g_{lk} A_{lk}(t - \\tau_{lk}) e^{-j 2\\pi f_c \\tau_{lk} - j\\varphi_{lk}} + n_{lk}(t)`,
    action: "Examine the 9 oscilloscope traces. Notice how arrival times and phases differ across channels.",
    buttonText: "Simulate Direct Sum (Failure)",
    viewPreset: "orbit",
    oscMode: "raw"
  },
  4: {
    title: "Stage 4: First-Level Coarse Time Compensation",
    subtitle: "Geometric Path Difference from INS Baseline & Beam Direction (Section 3.2)",
    description: `Using geodetic coordinates $(L_i, B_i, H_i)$ and target beam azimuth $\\theta$, the central system calculates the baseline distance $D_i$ (Eq. 13) and applies geometric delay compensation $\\Delta t_i = \\text{sgn}(L_i - L_1) D_i \\sin\\theta / c$. This slides all 9 envelopes into a small residual window of $\\pm 2$ to $\\pm 3$ sampling points.`,
    math: `\\Delta t_i = \\text{sgn}(L_i - L_1) \\cdot \\frac{D_i \\sin\\theta}{c}`,
    action: "Apply coarse compensation to shift the 9 waveforms into coarse alignment around bin 30.",
    buttonText: "Execute Coarse Time Shift",
    viewPreset: "orbit",
    oscMode: "coarse"
  },
  5: {
    title: "Stage 5: Regional Cross-Correlation (RCC) Fine Estimation",
    subtitle: "Small-Scale Restricted Search Window Peak Detection (Section 3.3)",
    description: `This is the core research contribution. Conventional cross-correlation and peak picking fail under low SNR (8-13 dB) because noise spikes outside the target gate exceed the true echo. RCC restricts peak searching to a small window ($n = \\pm 3, \\pm 5, \\pm 7$ points) around expected zero lag, rejecting false noise peaks and extracting sub-sample delay $\\Delta \\tau$ and phase $\\Delta \\phi$.`,
    math: `\\hat{\\Delta \\tau}_{lk} = \\arg\\max_{|\\tau| \\le n T_s} |x_{lk}(\\tau)|, \\quad \\hat{\\Delta \\phi}_{lk} = \\angle x_{lk}(\\hat{\\Delta \\tau}_{lk})`,
    action: "Inspect the RCC correlation curve and the restricted amber search window rejecting outside noise spikes.",
    buttonText: "Run RCC Fine Alignment",
    viewPreset: "orbit",
    oscMode: "rcc"
  },
  6: {
    title: "Stage 6: Coherent Signal Synthesis & Verification",
    subtitle: "Phase-Aligned Constructive Synthesis & Efficiency Analysis (Section 4 & 5)",
    description: `With time delays aligned and carrier phase shifts compensated, all 9 channels sum constructively: $y(t) = \\sum s_{lk}^{\\text{comp}}(t)$. The synthesized signal rises sharply above the noise floor, achieving over $+8.69\\text{ dB}$ SNR gain and $82.3\\%$ coherent synthesis efficiency, exactly validating the paper's experimental findings!`,
    math: `\\eta = \\frac{\\Delta \\text{SNR}_{s,\\text{actual}}}{KL} \\times 100\\%, \\quad \\Delta \\text{SNR}_{\\text{ideal}} = KL = 9.54\\text{ dB}`,
    action: "Observe the dramatic SNR gain and synthesis efficiency increase (+8.69 dB, 82.3% efficiency).",
    buttonText: "Benchmark vs Outfield Results",
    viewPreset: "drone",
    oscMode: "rcc"
  }
};

function renderStageGuidance(stageNum) {
  const guide = STAGE_GUIDES[stageNum];
  const deck = document.getElementById('stage-guidance-deck');
  if (!deck || !guide) return;

  deck.innerHTML = `
    <div class="flex items-center justify-between border-b border-radar-800 pb-1.5">
      <div class="text-xs font-bold text-radar-cyan uppercase">${guide.title}</div>
      <div class="text-[10px] text-slate-400">${guide.subtitle}</div>
    </div>
    <p class="text-slate-300 leading-relaxed">${guide.description}</p>
    <div id="katex-stage-formula" class="py-1 text-center bg-radar-900/60 rounded border border-radar-800/80 overflow-x-auto"></div>
    <div class="flex items-center justify-between pt-1">
      <span class="text-amber-400/90 text-[11px] flex items-center gap-1">
        <i data-lucide="info" class="w-3.5 h-3.5"></i> ${guide.action}
      </span>
      <button id="btn-stage-action" class="px-2.5 py-1 rounded bg-radar-cyan hover:bg-cyan-400 text-radar-950 font-bold text-[11px] shadow-sm">
        ${guide.buttonText}
      </button>
    </div>
  `;

  // Render KaTeX formula
  const formulaDiv = document.getElementById('katex-stage-formula');
  if (formulaDiv && window.katex) {
    try {
      window.katex.render(guide.math, formulaDiv, { displayMode: true, throwOnError: false });
    } catch (e) {
      formulaDiv.textContent = guide.math;
    }
  }

  // Update pipeline badges
  document.getElementById('pipeline-active-badge').textContent = `Stage ${stageNum}: ${guide.title.split(':')[1] || guide.title}`;
  document.getElementById('guided-step-counter').textContent = `${stageNum} / 6`;

  // Update stage tab active styles
  document.querySelectorAll('.stage-tab').forEach(tab => {
    const s = parseInt(tab.getAttribute('data-stage'));
    if (s === stageNum) {
      tab.classList.add('active');
    } else {
      tab.classList.remove('active');
    }
  });

  // Switch camera & oscilloscope view mode according to stage
  setCameraView(guide.viewPreset);
  state.oscViewMode = guide.oscMode;
  document.getElementById('osc-view-mode').value = guide.oscMode;

  // Bind stage action button
  const actionBtn = document.getElementById('btn-stage-action');
  if (actionBtn) {
    actionBtn.onclick = () => {
      fetchSimulationCycle();
      emitRFWavefront();
    };
  }

  initIcons();
}

// ==========================================
// 9. EVENT HANDLERS & PARAMETER CONTROLS
// ==========================================
function initEventHandlers() {
  // Mode switcher
  document.getElementById('btn-mode-guided').onclick = () => switchMode('guided');
  document.getElementById('btn-mode-lab').onclick = () => switchMode('lab');

  // Stage tab buttons
  document.querySelectorAll('.stage-tab').forEach(tab => {
    tab.onclick = () => {
      const s = parseInt(tab.getAttribute('data-stage'));
      state.currentStage = s;
      renderStageGuidance(s);
      fetchSimulationCycle();
    };
  });

  // Guided Stepper Controls
  document.getElementById('btn-next-stage').onclick = () => {
    if (state.currentStage < 6) {
      state.currentStage++;
      renderStageGuidance(state.currentStage);
      fetchSimulationCycle();
    }
  };

  document.getElementById('btn-prev-stage').onclick = () => {
    if (state.currentStage > 1) {
      state.currentStage--;
      renderStageGuidance(state.currentStage);
      fetchSimulationCycle();
    }
  };

  document.getElementById('btn-auto-tour').onclick = toggleAutoTour;

  // 3D Camera preset buttons
  document.getElementById('cam-orbit').onclick = () => setCameraView('orbit');
  document.getElementById('cam-top').onclick = () => setCameraView('top');
  document.getElementById('cam-drone').onclick = () => setCameraView('drone');
  document.getElementById('cam-r1').onclick = () => setCameraView('r1');
  document.getElementById('cam-r2').onclick = () => setCameraView('r2');
  document.getElementById('cam-r3').onclick = () => setCameraView('r3');

  // 3D toggles
  document.getElementById('btn-toggle-wavefronts').onclick = () => {
    state.three.showWavefronts = !state.three.showWavefronts;
    document.getElementById('wavefront-status').textContent = state.three.showWavefronts ? 'ON' : 'OFF';
  };

  document.getElementById('btn-toggle-beams').onclick = () => {
    state.three.showBeams = !state.three.showBeams;
    state.three.radarBeams.forEach(b => b.visible = state.three.showBeams);
    document.getElementById('beam-status').textContent = state.three.showBeams ? 'ON' : 'OFF';
  };

  document.getElementById('btn-sim-play').onclick = () => {
    fetchSimulationCycle();
    emitRFWavefront();
  };

  document.getElementById('btn-sim-reset').onclick = resetSimulation;

  // Oscilloscope view mode selector
  document.getElementById('osc-view-mode').onchange = (e) => {
    state.oscViewMode = e.target.value;
    if (state.telemetry) renderOscilloscopeCanvas(state.telemetry);
  };

  // RCC Channel selector
  document.getElementById('rcc-channel-select').onchange = (e) => {
    state.selectedRccChannel = e.target.value;
    if (state.telemetry) renderRccCorrelationPlot(state.telemetry);
  };

  // Sliders
  const baselineSlider = document.getElementById('param-baseline');
  baselineSlider.oninput = (e) => {
    const val = parseFloat(e.target.value);
    state.params.baseline_length = val;
    document.getElementById('val-baseline').textContent = `${val} m`;
    update3DRadarBaselines(val);
    fetchSimulationCycle();
  };

  const distSlider = document.getElementById('param-distance');
  distSlider.oninput = (e) => {
    const val = parseFloat(e.target.value) * 1000;
    state.params.drone_distance = val;
    document.getElementById('val-distance').textContent = `${(val / 1000).toFixed(1)} km`;
    fetchSimulationCycle();
  };

  const snrSlider = document.getElementById('param-snr');
  snrSlider.oninput = (e) => {
    const val = parseFloat(e.target.value);
    state.params.snr_db = val;
    document.getElementById('val-snr').textContent = `${val.toFixed(1)} dB`;
    fetchSimulationCycle();
  };

  // RCC Window Buttons (3, 5, 7)
  document.querySelectorAll('.rcc-btn').forEach(btn => {
    btn.onclick = () => {
      document.querySelectorAll('.rcc-btn').forEach(b => {
        b.className = 'rcc-btn py-1 px-2 rounded border border-radar-700 text-xs font-mono text-center hover:border-radar-cyan';
      });
      btn.className = 'rcc-btn active py-1 px-2 rounded border border-radar-cyan bg-radar-cyan/20 text-radar-cyan text-xs font-mono text-center font-bold';
      const pts = parseInt(btn.getAttribute('data-rcc'));
      state.params.rcc_window_points = pts;
      document.getElementById('val-rcc-win').textContent = `±${pts} points (${pts * 50} ns)`;
      fetchSimulationCycle();
    };
  });

  // Presets
  document.getElementById('preset-outfield').onclick = () => {
    setPreset({ baseline: 60, dist: 17000, snr: 10, rcc: 5 });
  };
  document.getElementById('preset-lowsnr').onclick = () => {
    setPreset({ baseline: 60, dist: 10000, snr: 8, rcc: 3 });
  };
  document.getElementById('preset-farfield').onclick = () => {
    setPreset({ baseline: 400, dist: 10000, snr: 12, rcc: 7 });
  };
  document.getElementById('preset-ideal').onclick = () => {
    setPreset({ baseline: 60, dist: 10000, snr: 22, rcc: 3 });
  };

  // Benchmark button
  document.getElementById('btn-run-fig13').onclick = runFig13Benchmark;

  // Live Airspace & Dataset controls
  document.getElementById('btn-refresh-adsb').onclick = fetchLiveAdsbTargets;
  document.getElementById('live-adsb-select').onchange = (e) => {
    const idx = e.target.value;
    if (idx !== '' && state.liveTargets && state.liveTargets[idx]) {
      lockAdsbTarget(state.liveTargets[idx]);
    }
  };
  document.getElementById('btn-load-outfield').onclick = loadOutfieldDataset;
  document.getElementById('outfield-cycle-slider').oninput = (e) => {
    const cyc = parseInt(e.target.value);
    document.getElementById('outfield-cycle-label').textContent = `${cyc} / 20`;
    loadOutfieldCycle(cyc);
  };
  document.getElementById('btn-export-telemetry').onclick = exportTelemetryJSON;

  // Continuous 360° Surveillance Mode Toggle
  document.getElementById('btn-surv-async').onclick = () => setSurveillanceMode('async_360');
  document.getElementById('btn-surv-sync').onclick = () => setSurveillanceMode('coherent_sync');
  document.getElementById('btn-reset-tracks').onclick = async () => {
    await fetch('/api/surveillance/reset', { method: 'POST' });
    document.getElementById('surv-obs-log').innerHTML = '<div class="text-slate-500 italic">Waiting for beam-target intersection...</div>';
    document.getElementById('surv-alerts-log').innerHTML = '<div class="text-slate-500 italic">No active threat alerts yet...</div>';
  };
}

function update3DRadarBaselines(baselineM) {
  // Update 3D node positions
  if (state.three.radarGroups.length === 3) {
    state.three.radarGroups[1].position.x = -baselineM;
    state.three.radarGroups[2].position.x = baselineM;
  }
}

function setPreset(p) {
  state.params.baseline_length = p.baseline;
  state.params.drone_distance = p.dist;
  state.params.snr_db = p.snr;
  state.params.rcc_window_points = p.rcc;

  document.getElementById('param-baseline').value = p.baseline;
  document.getElementById('val-baseline').textContent = `${p.baseline} m`;
  document.getElementById('param-distance').value = p.dist / 1000;
  document.getElementById('val-distance').textContent = `${(p.dist / 1000).toFixed(1)} km`;
  document.getElementById('param-snr').value = p.snr;
  document.getElementById('val-snr').textContent = `${p.snr.toFixed(1)} dB`;

  document.querySelectorAll('.rcc-btn').forEach(btn => {
    const pts = parseInt(btn.getAttribute('data-rcc'));
    if (pts === p.rcc) {
      btn.className = 'rcc-btn active py-1 px-2 rounded border border-radar-cyan bg-radar-cyan/20 text-radar-cyan text-xs font-mono text-center font-bold';
    } else {
      btn.className = 'rcc-btn py-1 px-2 rounded border border-radar-700 text-xs font-mono text-center hover:border-radar-cyan';
    }
  });
  document.getElementById('val-rcc-win').textContent = `±${p.rcc} points (${p.rcc * 50} ns)`;

  update3DRadarBaselines(p.baseline);
  fetchSimulationCycle();
}

function switchMode(mode) {
  state.mode = mode;
  const btnGuided = document.getElementById('btn-mode-guided');
  const btnLab = document.getElementById('btn-mode-lab');
  const guidedControls = document.getElementById('guided-mode-controls');

  if (mode === 'guided') {
    btnGuided.className = 'px-3 py-1 text-xs font-medium rounded-md transition-all bg-radar-cyan text-radar-950 font-bold shadow-sm flex items-center gap-1.5';
    btnLab.className = 'px-3 py-1 text-xs font-medium rounded-md transition-all text-slate-300 hover:text-white flex items-center gap-1.5';
    guidedControls.classList.remove('hidden');
    document.getElementById('control-panel-title').textContent = 'Guided Mission Pipeline';
  } else {
    btnLab.className = 'px-3 py-1 text-xs font-medium rounded-md transition-all bg-radar-cyan text-radar-950 font-bold shadow-sm flex items-center gap-1.5';
    btnGuided.className = 'px-3 py-1 text-xs font-medium rounded-md transition-all text-slate-300 hover:text-white flex items-center gap-1.5';
    guidedControls.classList.add('hidden');
    document.getElementById('control-panel-title').textContent = 'Experimental Laboratory';
  }
}

function toggleAutoTour() {
  state.autoTourActive = !state.autoTourActive;
  const btn = document.getElementById('btn-auto-tour');

  if (state.autoTourActive) {
    btn.innerHTML = `<i data-lucide="pause-circle" class="w-4 h-4 text-amber-400"></i> Pause Scientific Methodology Walkthrough`;
    state.autoTourTimer = setInterval(() => {
      state.currentStage = (state.currentStage % 6) + 1;
      renderStageGuidance(state.currentStage);
      fetchSimulationCycle();
      emitRFWavefront();
    }, 4500);
  } else {
    btn.innerHTML = `<i data-lucide="play-circle" class="w-4 h-4 text-emerald-400"></i> Auto-Run Complete Scientific Methodology`;
    clearInterval(state.autoTourTimer);
  }
  initIcons();
}

async function resetSimulation() {
  try {
    await fetch('/api/physics/reset', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(state.params)
    });
    fetchSimulationCycle();
  } catch (err) {
    console.error("Reset error:", err);
  }
}


// ==========================================
// 10. LIVE ADS-B & OUTFIELD DATASET ENGINE
// ==========================================
async function fetchLiveAdsbTargets() {
  const select = document.getElementById('live-adsb-select');
  if (!select) return;
  select.innerHTML = '<option value="">Fetching live OpenSky ADS-B feed...</option>';
  try {
    const res = await fetch('/api/datasets/adsb/live?limit=25');
    const data = await res.json();
    state.liveTargets = data.targets || [];
    select.innerHTML = '';

    if (state.liveTargets.length === 0) {
      select.innerHTML = '<option value="">No airborne targets found</option>';
      return;
    }

    state.liveTargets.forEach((t, idx) => {
      const opt = document.createElement('option');
      opt.value = idx;
      const tag = t.source === 'opensky_live' ? 'LIVE' : 'CACHED';
      opt.textContent = `[${tag}] ${t.callsign} (${t.origin_country}) • ${Math.round(t.altitude_m)}m • ${Math.round(t.velocity_ms)} m/s`;
      select.appendChild(opt);
    });

    // Automatically lock onto first live aircraft
    await lockAdsbTarget(state.liveTargets[0]);
  } catch (err) {
    console.error('Failed to fetch live ADS-B targets:', err);
    select.innerHTML = '<option value="">Live feed error - using simulated target</option>';
  }
}

async function lockAdsbTarget(target) {
  try {
    const res = await fetch('/api/datasets/adsb/lock', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(target)
    });
    const result = await res.json();
    if (result.target) {
      document.getElementById('outfield-cycle-bar').classList.add('hidden');
      document.getElementById('outfield-cycle-bar').classList.remove('flex');
      state.params.drone_distance = Math.round(result.target.slant_range_m);
      state.params.drone_speed = Math.round(result.target.speed_ms);
      state.params.drone_altitude = Math.round(result.target.altitude_m);
      document.getElementById('param-distance').value = (state.params.drone_distance / 1000).toFixed(1);
      document.getElementById('val-distance').textContent = `${(state.params.drone_distance / 1000).toFixed(1)} km`;
      fetchSimulationCycle();
    }
  } catch (err) {
    console.error('Failed to lock ADS-B target:', err);
  }
}

async function loadOutfieldDataset() {
  const bar = document.getElementById('outfield-cycle-bar');
  bar.classList.remove('hidden');
  bar.classList.add('flex');
  document.getElementById('outfield-cycle-slider').value = 1;
  document.getElementById('outfield-cycle-label').textContent = '1 / 20';
  await loadOutfieldCycle(1);
}

async function loadOutfieldCycle(cycleIdx) {
  try {
    const res = await fetch(`/api/datasets/outfield/cycle/${cycleIdx}`, { method: 'POST' });
    const data = await res.json();
    if (data.cycle) {
      state.params.baseline_length = 60.0;
      state.params.drone_distance = data.cycle.range_m;
      document.getElementById('param-baseline').value = 60;
      document.getElementById('val-baseline').textContent = '60 m';
      document.getElementById('param-distance').value = (data.cycle.range_m / 1000).toFixed(1);
      document.getElementById('val-distance').textContent = `${(data.cycle.range_m / 1000).toFixed(1)} km`;
      update3DRadarBaselines(60.0);
      fetchSimulationCycle();
    }
  } catch (err) {
    console.error('Outfield cycle load error:', err);
  }
}

function exportTelemetryJSON() {
  if (!state.telemetry) return;
  const blob = new Blob([JSON.stringify(state.telemetry, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `darar_iq_telemetry_${Date.now()}.json`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// ==========================================
// 11. ADVANCED DSP WORKSTATION PLOTS
// ==========================================
function renderPhasorPlots(iqData) {
  if (!iqData) return;

  const drawPhasor = (divId, phasors, resultant, color) => {
    const el = document.getElementById(divId);
    if (!el) return;

    const traces = [];
    // Individual channel vectors from origin (0, 0)
    phasors.forEach((p, idx) => {
      traces.push({
        x: [0, p.i],
        y: [0, p.q],
        mode: 'lines+markers',
        name: p.channel,
        line: { color: CHANNEL_COLORS[idx % CHANNEL_COLORS.length], width: 1.5 },
        marker: { size: 4 }
      });
    });

    // Resultant coherent vector sum (scaled down by 1/3 for visual clarity on same grid if needed, or plotted directly)
    traces.push({
      x: [0, resultant.i / 4.0],
      y: [0, resultant.q / 4.0],
      mode: 'lines+markers',
      name: `Sum/4 (|Σ|=${resultant.mag.toFixed(1)})`,
      line: { color: color, width: 3.5 },
      marker: { size: 8, symbol: 'diamond' }
    });

    const layout = {
      margin: { l: 25, r: 10, t: 10, b: 25 },
      paper_bgcolor: '#050b14',
      plot_bgcolor: '#050b14',
      showlegend: false,
      xaxis: { zeroline: true, zerolinecolor: '#1e3d6b', gridcolor: '#0e243d', range: [-45, 45], tickfont: { size: 8, color: '#64748b' } },
      yaxis: { zeroline: true, zerolinecolor: '#1e3d6b', gridcolor: '#0e243d', range: [-45, 45], scaleanchor: 'x', tickfont: { size: 8, color: '#64748b' } }
    };

    Plotly.react(el, traces, layout, { responsive: true, displayModeBar: false });
  };

  drawPhasor('phasor-before-plot', iqData.before, iqData.resultant_before, '#ff0055');
  drawPhasor('phasor-after-plot', iqData.after, iqData.resultant_after, '#00ff66');
}

function renderRangeDopplerPlots(rdData) {
  if (!rdData) return;

  const drawHeatmap = (divId, zMatrix, colorscale) => {
    const el = document.getElementById(divId);
    if (!el) return;

    const trace = {
      x: rdData.range_bins,
      y: rdData.velocity_bins,
      z: zMatrix,
      type: 'heatmap',
      colorscale: colorscale,
      zmin: 0,
      zmax: 24,
      showscale: false
    };

    const layout = {
      margin: { l: 38, r: 10, t: 10, b: 28 },
      paper_bgcolor: '#050b14',
      plot_bgcolor: '#050b14',
      xaxis: { title: 'Range Bin', titlefont: { size: 9, color: '#64748b' }, tickfont: { size: 8, color: '#64748b' } },
      yaxis: { title: 'Vel (m/s)', titlefont: { size: 9, color: '#64748b' }, tickfont: { size: 8, color: '#64748b' } }
    };

    Plotly.react(el, [trace], layout, { responsive: true, displayModeBar: false });
  };

  drawHeatmap('rd-single-plot', rdData.single_channel_db, 'Viridis');
  drawHeatmap('rd-synth-plot', rdData.coherent_synth_db, 'Jet');
}

function renderLossSurface3D(surfaceData) {
  const el = document.getElementById('loss-surface-3d-plot');
  if (!el || !surfaceData) return;

  const traceSurface = {
    x: surfaceData.time_error_axis,
    y: surfaceData.phase_error_axis,
    z: surfaceData.loss_surface_db,
    type: 'surface',
    colorscale: 'Portland',
    showscale: false,
    opacity: 0.85
  };

  const pt = surfaceData.current_point;
  const tracePoint = {
    x: [pt.time_err_norm],
    y: [pt.phase_err_deg],
    z: [pt.loss_db + 0.15],
    mode: 'markers',
    type: 'scatter3d',
    marker: { color: '#ff0055', size: 6, symbol: 'circle' },
    name: 'Live Operating Point'
  };

  const layout = {
    margin: { l: 0, r: 0, t: 5, b: 0 },
    paper_bgcolor: '#050b14',
    scene: {
      xaxis: { title: 'Time Err (1/B)', titlefont: { size: 9, color: '#94a3b8' }, tickfont: { size: 8, color: '#64748b' }, gridcolor: '#1e3d6b' },
      yaxis: { title: 'Phase Err (°)', titlefont: { size: 9, color: '#94a3b8' }, tickfont: { size: 8, color: '#64748b' }, gridcolor: '#1e3d6b' },
      zaxis: { title: 'Loss (dB)', titlefont: { size: 9, color: '#94a3b8' }, tickfont: { size: 8, color: '#64748b' }, gridcolor: '#1e3d6b' },
      bgcolor: '#050b14'
    }
  };

  Plotly.react(el, [traceSurface, tracePoint], layout, { responsive: true, displayModeBar: false });
}

async function fetchAntennaPattern() {
  const el = document.getElementById('antenna-pattern-plot');
  if (!el) return;
  try {
    const res = await fetch('/api/analytics/antenna_pattern');
    const data = await res.json();
    const trace = {
      x: data.angles_deg,
      y: data.gains_db,
      mode: 'lines',
      name: '32-Element Array Gain',
      line: { color: '#00f0ff', width: 2 },
      fill: 'tozeroy',
      fillcolor: 'rgba(0, 240, 255, 0.08)'
    };
    const layout = {
      margin: { l: 42, r: 15, t: 15, b: 35 },
      paper_bgcolor: '#050b14',
      plot_bgcolor: '#050b14',
      xaxis: { title: 'Normalized Target Angle θ (°)', titlefont: { size: 10, color: '#94a3b8' }, tickfont: { size: 9, color: '#64748b' }, gridcolor: '#142b4d' },
      yaxis: { title: 'Normalized Gain (dB)', titlefont: { size: 10, color: '#94a3b8' }, tickfont: { size: 9, color: '#64748b' }, gridcolor: '#142b4d', range: [-70, 2] }
    };
    Plotly.react(el, [trace], layout, { responsive: true, displayModeBar: false });
  } catch (err) {
    console.error('Antenna pattern error:', err);
  }
}


// ==========================================
// 12. CONTINUOUS 360° SURVEILLANCE & ALERT UI
// ==========================================
async function setSurveillanceMode(mode) {
  const btnAsync = document.getElementById('btn-surv-async');
  const btnSync = document.getElementById('btn-surv-sync');
  const badge = document.getElementById('surv-mode-badge');

  if (mode === 'async_360') {
    btnAsync.className = 'px-2.5 py-0.5 rounded bg-emerald-500 text-radar-950 font-bold';
    btnSync.className = 'px-2.5 py-0.5 rounded text-slate-300 hover:text-white';
    if (badge) badge.textContent = 'MODE: 360° ASYNC (ZERO-WAIT)';
  } else {
    btnSync.className = 'px-2.5 py-0.5 rounded bg-radar-cyan text-radar-950 font-bold';
    btnAsync.className = 'px-2.5 py-0.5 rounded text-slate-300 hover:text-white';
    if (badge) badge.textContent = 'MODE: COHERENT SYNC (α-FILTER)';
  }

  try {
    await fetch(`/api/surveillance/mode?mode=${mode}`, { method: 'POST' });
  } catch (err) {
    console.error('Failed to switch surveillance mode:', err);
  }
}

function updateSurveillanceConsole(surv) {
  if (!surv) return;

  // 1. Independent Radar Observations Log
  const obsLog = document.getElementById('surv-obs-log');
  if (obsLog && surv.recent_observations && surv.recent_observations.length > 0) {
    obsLog.innerHTML = surv.recent_observations.map(o => {
      const rColor = o.radar_id === 1 ? 'text-radar-cyan' : (o.radar_id === 2 ? 'text-emerald-400' : 'text-amber-400');
      return `<div class="flex justify-between border-b border-radar-900 pb-0.5">
        <span><span class="${rColor} font-bold">${o.radar_name}</span> HIT @ ${o.timestamp.toFixed(2)}s</span>
        <span class="text-slate-400">${(o.range_m / 1000).toFixed(2)}km | ${o.processing_latency_ms}ms</span>
      </div>`;
    }).join('');
  }

  // 2. Multi-Radar Fused Track Card
  if (surv.tracks && surv.tracks.length > 0) {
    const trk = surv.tracks[0];
    document.getElementById('trk-id').textContent = trk.track_id;
    document.getElementById('trk-conf').textContent = `${trk.confidence.toFixed(1)}%`;
    document.getElementById('trk-range').textContent = `${(trk.range_m / 1000).toFixed(2)} km`;
    document.getElementById('trk-az').textContent = `${trk.azimuth_deg.toFixed(1)}°`;
    document.getElementById('trk-speed').textContent = `${trk.speed_ms.toFixed(1)} m/s`;
    document.getElementById('trk-heading').textContent = `${trk.heading_deg.toFixed(0)}°`;

    [1, 2, 3].forEach(rid => {
      const b = document.getElementById(`trk-r${rid}-badge`);
      if (b) {
        if (trk.contributing_radars.includes(rid)) {
          b.className = 'px-1 rounded bg-emerald-500/20 text-emerald-400 border border-emerald-500/50 font-bold';
        } else {
          b.className = 'px-1 rounded bg-radar-800 text-slate-500';
        }
      }
    });
  }

  // 3. Rapid Threat Alerts Log
  const alertsLog = document.getElementById('surv-alerts-log');
  if (alertsLog && surv.alerts && surv.alerts.length > 0) {
    alertsLog.innerHTML = surv.alerts.map(a => {
      const sevClass = a.severity === 'CRITICAL'
        ? 'bg-rose-500/15 border-rose-500/40 text-rose-300'
        : (a.severity === 'HIGH'
          ? 'bg-amber-500/15 border-amber-500/40 text-amber-300'
          : 'bg-radar-cyan/10 border-radar-cyan/30 text-radar-cyan');
      return `<div class="p-1.5 rounded border ${sevClass}">
        <div class="flex justify-between font-bold">
          <span>[${a.severity}] ${a.track_id}</span>
          <span>${a.confidence.toFixed(0)}% • ${a.latency_ms}ms</span>
        </div>
        <div class="text-[9px] text-slate-300 mt-0.5">
          Rng ${a.range_km}km @ ${a.azimuth_deg}° | Vel ${a.speed_ms}m/s → ${a.heading_deg}° (${a.contributing_radars.length}/3 Radars)
        </div>
      </div>`;
    }).join('');
  }
}
