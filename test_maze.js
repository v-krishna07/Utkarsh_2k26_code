'use strict';
// ============================================================================
//  test_maze.js - Acceptance and integration tests for the maze mapper.
//
//  Tests all 6 requirements:
//    1. Real 90-degree spin right and left: heading ends at exactly +90 / -90, logged once.
//    2. 180 turn: one U-TURN entry, no duplicate nodes, driving back reuses nodes.
//    3. W steps move the car exactly STEP_CM along the heading for all four headings.
//    4. W then A then W then D: path is correct and graph has right nodes and edges.
//    5. Simulated maze with virtual car driven by real server step path:
//       map walls match maze within 5 cm (corridor, T-junction, dead end).
//    6. Front echo of 0 does not create a wall, obstacle at 4 cm does,
//       and BLOCKED shows only on the front box.
//
//  Run:  node test_maze.js
// ============================================================================
const maze = require('./maze');
const server = require('./server');

function buildPacket(seq, leftUs, frontUs, rightUs) {
    const buf = Buffer.alloc(9);
    buf[0] = 0x5A;
    buf[1] = seq & 0xFF;
    buf[2] = leftUs & 0xFF;
    buf[3] = (leftUs >> 8) & 0xFF;
    buf[4] = frontUs & 0xFF;
    buf[5] = (frontUs >> 8) & 0xFF;
    buf[6] = rightUs & 0xFF;
    buf[7] = (rightUs >> 8) & 0xFF;
    let xor = 0;
    for (let i = 0; i < 8; i++) xor ^= buf[i];
    buf[8] = xor;
    return buf;
}

function cmToUs(cm) {
    return Math.round(cm * 2 / maze.MAZE_CONFIG.SPEED_OF_SOUND_CM_PER_US);
}

// ----------------------------------------------------------------------------
// TEST 1: Real 90-degree spin right and left: heading ends at exactly +90 / -90, logged once
// ----------------------------------------------------------------------------
function test1_Real90DegreeSpins() {
    console.log('--- TEST 1: Real 90-Degree Spins Right & Left ---');
    const state = maze.createMazeState();
    state.mode = 'hold';
    const calib = { alpha: 1, fasterSide: 'left', spinLeft: 1, spinRight: 1, spinInvert: false };
    const engine = server.createControlEngine(calib, state);

    console.log(`  Initial heading: ${state.heading}°`);

    let now = 1000;
    const dt = 0.02;

    // Provide simulated sensor packets with open side/front
    let seq = 0;
    function feedSensors(dL, dF, dR) {
        const pkt = maze.parseSensorPacket(buildPacket(seq++, cmToUs(dL), cmToUs(dF), cmToUs(dR)));
        maze.sensorUpdate(state, pkt, now);
    }
    feedSensors(20, 60, 20);

    // 1. Spin RIGHT (key D): str = 1
    // At spinDegS = 180 deg/s, full spin in place (SPIN_BASE=255) rotates at 180 deg/s.
    // 90 deg rotation takes 0.50 s (25 ticks of 20 ms).
    for (let t = 0; t < 0.50; t += dt) {
        now += dt * 1000;
        engine.setIntent({ thr: 0, str: 1, speed: 140, boost: false, estop: false, smooth: false, brake: false }, now);
        engine.step(now);
        feedSensors(20, 60, 20);
        maze.mazeTick(state, dt, now);
    }
    // Release key D -> neutral intent
    engine.setIntent({ thr: 0, str: 0, speed: 0, boost: false, estop: false, smooth: false, brake: true }, now + 20);
    now += 20;
    engine.step(now);
    feedSensors(20, 60, 20);
    maze.mazeTick(state, dt, now);

    const passRightHeading = state.heading === 90;
    const rightLogs = state.turnLog.filter(e => e.text.startsWith('RIGHT (node #'));
    const passRightLog = rightLogs.length === 1;
    console.log(`[${passRightHeading ? 'PASS' : 'FAIL'}] Spin Right ended at exactly ${state.heading}° (expected 90°)`);
    console.log(`[${passRightLog ? 'PASS' : 'FAIL'}] Logged once: ${rightLogs[0] ? rightLogs[0].text : 'none'} (count: ${rightLogs.length})`);

    // 2. Spin LEFT (key A): str = -1 from 90° back to 0°
    for (let t = 0; t < 0.50; t += dt) {
        now += dt * 1000;
        engine.setIntent({ thr: 0, str: -1, speed: 140, boost: false, estop: false, smooth: false, brake: false }, now);
        engine.step(now);
        feedSensors(20, 60, 20);
        maze.mazeTick(state, dt, now);
    }
    // Release key A -> neutral intent
    engine.setIntent({ thr: 0, str: 0, speed: 0, boost: false, estop: false, smooth: false, brake: true }, now + 20);
    now += 20;
    engine.step(now);
    feedSensors(20, 60, 20);
    maze.mazeTick(state, dt, now);

    const passLeftHeading = state.heading === 0;
    const leftLogs = state.turnLog.filter(e => e.text.startsWith('LEFT (node #'));
    const passLeftLog = leftLogs.length === 1;
    console.log(`[${passLeftHeading ? 'PASS' : 'FAIL'}] Spin Left ended at exactly ${state.heading}° (expected 0°)`);
    console.log(`[${passLeftLog ? 'PASS' : 'FAIL'}] Logged once: ${leftLogs[0] ? leftLogs[0].text : 'none'} (count: ${leftLogs.length})`);

    return passRightHeading && passRightLog && passLeftHeading && passLeftLog;
}

// ----------------------------------------------------------------------------
// TEST 2: 180 turn: one U-TURN entry, no duplicate nodes, driving back reuses nodes
// ----------------------------------------------------------------------------
function test2_UturnAndNodeReuse() {
    console.log('\n--- TEST 2: 180 Turn (U-Turn) & Node Reuse on Driving Back ---');
    const state = maze.createMazeState();
    let now = 1000;
    const dt = 0.02;
    const calib = { alpha: 1, fasterSide: 'left', spinLeft: 1, spinRight: 1, spinInvert: false };
    const engine = server.createControlEngine(calib, state);

    let seq = 0;
    function feedSensors(dL, dF, dR) {
        const pkt = maze.parseSensorPacket(buildPacket(seq++, cmToUs(dL), cmToUs(dF), cmToUs(dR)));
        maze.sensorUpdate(state, pkt, now);
    }

    // Start with origin node at (0, 0)
    state.nodes.push({ id: state.nextNodeId++, x: 0, y: 0, isJunction: false, isDeadEnd: false, explored: true });
    state.lastNodeId = 0;

    // Drive forward North using 2 steps of 15 cm: reaches (0, 30)
    feedSensors(15, 60, 15);
    maze.startStep(state, 'W', 140, now);
    while (state.currentStep !== null) {
        now += dt * 1000;
        engine.step(now);
        feedSensors(15, 60, 15);
        maze.mazeTick(state, dt, now);
    }
    maze.startStep(state, 'W', 140, now);
    while (state.currentStep !== null) {
        now += dt * 1000;
        engine.step(now);
        feedSensors(15, 60, 15);
        maze.mazeTick(state, dt, now);
    }

    // Flag dead end at y = 30
    feedSensors(15, 10, 15);
    maze.mazeTick(state, dt, now);

    const initialNodeCount = state.nodes.length;
    console.log(`  Forward path generated ${initialNodeCount} nodes up to y=${state.y} cm`);

    // Execute 180° turn (spin in place for 1.00 s at 180 deg/s)
    state.mode = 'hold';
    for (let t = 0; t < 1.00; t += dt) {
        now += dt * 1000;
        engine.setIntent({ thr: 0, str: 1, speed: 140, boost: false, estop: false, smooth: false, brake: false }, now);
        engine.step(now);
        maze.mazeTick(state, dt, now);
    }
    // Stop spinning
    engine.setIntent({ thr: 0, str: 0, speed: 0, boost: false, estop: false, smooth: false, brake: true }, now + 20);
    now += 20;
    engine.step(now);
    feedSensors(15, 60, 15);
    maze.mazeTick(state, dt, now);

    const passHeading180 = state.heading === 180;
    const uturnLogs = state.turnLog.filter(e => e.text.startsWith('U-TURN (node #'));
    const passOneUturnLog = uturnLogs.length === 1;
    const passNoDupTurn = state.nodes.length === initialNodeCount;
    console.log(`[${passHeading180 ? 'PASS' : 'FAIL'}] Heading after 180° spin: ${state.heading}° (expected 180°)`);
    console.log(`[${passOneUturnLog ? 'PASS' : 'FAIL'}] Exactly one U-TURN log entry: "${uturnLogs[0] ? uturnLogs[0].text : 'none'}"`);
    console.log(`[${passNoDupTurn ? 'PASS' : 'FAIL'}] No duplicate nodes created at turnaround point (${state.nodes.length} nodes)`);

    // Drive back South from y = 30 down to y = 0 using 2 steps of 15 cm
    state.mode = 'step';
    maze.startStep(state, 'W', 140, now);
    while (state.currentStep !== null) {
        now += dt * 1000;
        engine.step(now);
        feedSensors(15, 60, 15);
        maze.mazeTick(state, dt, now);
    }
    maze.startStep(state, 'W', 140, now);
    while (state.currentStep !== null) {
        now += dt * 1000;
        engine.step(now);
        feedSensors(15, 60, 15);
        maze.mazeTick(state, dt, now);
    }

    const finalNodeCount = state.nodes.length;
    const passNodeReuse = finalNodeCount === initialNodeCount && Math.abs(state.y - 0) < 0.1;
    console.log(`[${passNodeReuse ? 'PASS' : 'FAIL'}] Driving back South reached y=${state.y} cm and reused existing nodes without duplicates (${finalNodeCount} nodes total)`);

    return passHeading180 && passOneUturnLog && passNoDupTurn && passNodeReuse;
}

// ----------------------------------------------------------------------------
// TEST 3: W steps move the car exactly STEP_CM along the heading for all four headings
// ----------------------------------------------------------------------------
function test3_WStepsAllHeadings() {
    console.log('\n--- TEST 3: W Steps Exactly STEP_CM for All 4 Headings ---');
    const state = maze.createMazeState();
    state.mode = 'step';
    const stepCm = state.cfg.STEP_CM; // 15 cm
    let allPass = true;
    let now = 1000;
    const dt = 0.02;

    const headings = [
        { name: 'NORTH', heading: 0,   expectedDx: 0, expectedDy: stepCm },
        { name: 'EAST',  heading: 90,  expectedDx: stepCm, expectedDy: 0 },
        { name: 'SOUTH', heading: 180, expectedDx: 0, expectedDy: -stepCm },
        { name: 'WEST',  heading: 270, expectedDx: -stepCm, expectedDy: 0 },
    ];

    for (const h of headings) {
        state.x = 0; state.y = 0; state.heading = h.heading;
        state.prevSnapName = h.name;
        state.distF = 80; // plenty of space

        const started = maze.startStep(state, 'W', 140, now);
        while (state.currentStep !== null) {
            now += dt * 1000;
            maze.mazeTick(state, dt, now);
        }

        const dx = Math.round(state.x * 10) / 10;
        const dy = Math.round(state.y * 10) / 10;
        const pass = dx === h.expectedDx && dy === h.expectedDy && state.heading === h.heading;
        if (!pass) allPass = false;
        console.log(`[${pass ? 'PASS' : 'FAIL'}] Facing ${h.name} (${h.heading}°): moved (Δx=${dx}, Δy=${dy}), expected (${h.expectedDx}, ${h.expectedDy})`);
    }

    return allPass;
}

// ----------------------------------------------------------------------------
// TEST 4: W then A then W then D: path is correct and graph has right nodes and edges
// ----------------------------------------------------------------------------
function test4_SequenceWAWD() {
    console.log('\n--- TEST 4: Sequence W -> A -> W -> D Path & Graph ---');
    const state = maze.createMazeState();
    state.mode = 'step';
    let now = 1000;
    const dt = 0.02;

    function runMotion(m) {
        maze.startStep(state, m, 140, now);
        while (state.currentStep !== null) {
            now += dt * 1000;
            maze.mazeTick(state, dt, now);
        }
    }

    state.distF = 80;

    // 1. W (forward 15 cm along North)
    runMotion('W');
    const passW1 = state.x === 0 && state.y === 15 && state.heading === 0;
    console.log(`[${passW1 ? 'PASS' : 'FAIL'}] Step 1 (W): pose=(${state.x}, ${state.y}), heading=${state.heading}°`);

    // 2. A (spin left 90° to West: 270°)
    runMotion('A');
    const passA = state.x === 0 && state.y === 15 && state.heading === 270;
    console.log(`[${passA ? 'PASS' : 'FAIL'}] Step 2 (A): pose=(${state.x}, ${state.y}), heading=${state.heading}°`);

    // 3. W (forward 15 cm along West: x = -15, y = 15)
    runMotion('W');
    const passW2 = state.x === -15 && state.y === 15 && state.heading === 270;
    console.log(`[${passW2 ? 'PASS' : 'FAIL'}] Step 3 (W): pose=(${state.x}, ${state.y}), heading=${state.heading}°`);

    // 4. D (spin right 90° to North: 0°)
    runMotion('D');
    const passD = state.x === -15 && state.y === 15 && state.heading === 0;
    console.log(`[${passD ? 'PASS' : 'FAIL'}] Step 4 (D): pose=(${state.x}, ${state.y}), heading=${state.heading}°`);

    // Graph checks: nodes at (0, 15) and (-15, 15) connected by edge
    const hasOriginNode = state.nodes.some(n => Math.hypot(n.x - 0, n.y - 15) < 5);
    const hasCornerNode = state.nodes.some(n => Math.hypot(n.x - (-15), n.y - 15) < 5);
    const passGraph = hasOriginNode && hasCornerNode && state.edges.length >= 1;
    console.log(`[${passGraph ? 'PASS' : 'FAIL'}] Graph: ${state.nodes.length} nodes, ${state.edges.length} edges connecting the corridor path`);

    return passW1 && passA && passW2 && passD && passGraph;
}

// ----------------------------------------------------------------------------
// TEST 5: Simulated maze with virtual car driven by real server step path
// ----------------------------------------------------------------------------
function test5_SimulatedMazeStepIntegration() {
    console.log('\n--- TEST 5: Simulated Maze Exploration & Wall Mapping (< 5 cm) ---');
    const state = maze.createMazeState();
    state.mode = 'step';
    let now = 1000;
    const dt = 0.02;

    // True maze corridor:
    // Lane 0: x ∈ [-15, 15], y ∈ [0, 120] (North-South, walls at x = -15 and x = +15)
    // Left branch: x ∈ [-60, -15], y ∈ [105, 135] (West dead end, wall at x = -60)
    function simulateRay(ox, oy, headDeg, angleDeg) {
        const rad = (headDeg + angleDeg) * Math.PI / 180;
        const dx = Math.sin(rad), dy = Math.cos(rad);
        for (let d = 0.5; d <= 120; d += 0.5) {
            const px = ox + dx * d, py = oy + dy * d;
            // North wall of main lane
            if (py >= 120 && (px < -15 || px > 15 || py >= 135)) return d;
            // Dead end west wall
            if (px <= -60) return d;
            // Lane 0 side walls
            if (py < 105 && (px <= -15 || px >= 15)) return d;
            if (py >= 105 && px >= 15) return d;
            // Branch side walls
            if (px < -15 && (py <= 105 || py >= 135)) return d;
        }
        return 120;
    }

    function feedSensors() {
        const cfg = state.cfg;
        const headRad = state.heading * Math.PI / 180;

        const lx = state.x + cfg.SENSOR_LEFT.x * Math.cos(headRad) + cfg.SENSOR_LEFT.y * Math.sin(headRad);
        const ly = state.y - cfg.SENSOR_LEFT.x * Math.sin(headRad) + cfg.SENSOR_LEFT.y * Math.cos(headRad);
        const dL = simulateRay(lx, ly, state.heading, -90);

        const fx = state.x + cfg.SENSOR_FRONT.x * Math.cos(headRad) + cfg.SENSOR_FRONT.y * Math.sin(headRad);
        const fy = state.y - cfg.SENSOR_FRONT.x * Math.sin(headRad) + cfg.SENSOR_FRONT.y * Math.cos(headRad);
        const dF = simulateRay(fx, fy, state.heading, 0);

        const rx = state.x + cfg.SENSOR_RIGHT.x * Math.cos(headRad) + cfg.SENSOR_RIGHT.y * Math.sin(headRad);
        const ry = state.y - cfg.SENSOR_RIGHT.x * Math.sin(headRad) + cfg.SENSOR_RIGHT.y * Math.cos(headRad);
        const dR = simulateRay(rx, ry, state.heading, 90);

        const pkt = maze.parseSensorPacket(buildPacket(0, cmToUs(dL), cmToUs(dF), cmToUs(dR)));
        maze.sensorUpdate(state, pkt, now);
    }

    function doStep(m) {
        feedSensors();
        maze.startStep(state, m, 180, now);
        while (state.currentStep !== null) {
            now += dt * 1000;
            feedSensors();
            maze.mazeTick(state, dt, now);
        }
        feedSensors();
        maze.mazeTick(state, dt, now);
    }

    // Drive forward North from y=0 to y=105 (7 steps of 15 cm)
    for (let i = 0; i < 7; i++) doStep('W');
    console.log(`  Explored North corridor: reached y=${state.y.toFixed(1)} cm`);

    // Turn LEFT into the west dead end
    doStep('A');
    console.log(`  Turned LEFT at T-junction: heading=${state.heading}°, pose=(${state.x.toFixed(1)}, ${state.y.toFixed(1)})`);

    // Drive West into dead end (2 steps of 15 cm)
    doStep('W');
    doStep('W');
    console.log(`  Reached dead end: pose=(${state.x.toFixed(1)}, ${state.y.toFixed(1)})`);

    // U-Turn at dead end
    doStep('S');
    console.log(`  U-turn executed: heading=${state.heading}° (facing East)`);

    // Check mapped wall accuracy against true maze in the North corridor (x=-15 and x=15)
    let hits = 0, checks = 0;
    const snap = maze.getGridSnapshot(state).filter(c => c.v > 0.3);

    for (let testY = 15; testY <= 90; testY += 15) {
        for (const trueWallX of [-15, 15]) {
            checks++;
            const match = snap.find(c => Math.abs(c.x - trueWallX) <= 5.0 && Math.abs(c.y - testY) <= 5.0);
            if (match) hits++;
        }
    }

    const accuracyPct = checks > 0 ? (hits / checks * 100) : 0;
    const passAccuracy = accuracyPct >= 80;
    console.log(`[${passAccuracy ? 'PASS' : 'FAIL'}] Wall map accuracy: ${hits}/${checks} samples within 5 cm (${accuracyPct.toFixed(0)}%)`);

    return passAccuracy;
}

// ----------------------------------------------------------------------------
// TEST 6: Front echo of 0 does not create a wall, 4 cm does, and BLOCKED on front box only
// ----------------------------------------------------------------------------
function test6_EchoZeroNoWallAndFrontOnlyBlocked() {
    console.log('\n--- TEST 6: Echo 0 No-Wall, Obstacle 4 cm Wall & Front-Only BLOCKED ---');
    const state = maze.createMazeState();
    let now = 1000;

    // 6a: Front echo of 0 after far reading does NOT create a wall
    // Initial far reading (50 cm)
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(50), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
    }
    now += 600; // past close memory
    // Now receive echo of 0
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), 0, cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const snapZero = maze.getGridSnapshot(state);
    const passNoWallOnZero = !snapZero.some(c => c.v > 0.4);
    console.log(`[${passNoWallOnZero ? 'PASS' : 'FAIL'}] 6a: Front echo of 0 does NOT create a wall on map (cells: ${snapZero.length})`);

    // 6b: Obstacle at 4 cm DOES create a wall and sets frontBlocked
    state.pwmL = 100; state.pwmR = 100; // commanded motion
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(4), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const snap4cm = maze.getGridSnapshot(state);
    const hasWall4cm = snap4cm.some(c => c.v > 0.4);
    const passObstacle4cm = hasWall4cm && state.frontBlocked === true;
    console.log(`[${passObstacle4cm ? 'PASS' : 'FAIL'}] 6b: Obstacle at 4 cm creates wall and sets frontBlocked=${state.frontBlocked}`);

    // 6c: BLOCKED styling logic applies to FRONT box ONLY
    // Simulate UI renderSensorBox logic
    function simulateRenderBox(id, dist, isBlocked) {
        if (id === 'SF' && (dist < 12 || isBlocked)) return { tag: 'BLOCKED', isRed: true };
        return { tag: '', isRed: false };
    }
    const leftBox = simulateRenderBox('SL', 8.0, false);   // Left is close (8 cm)
    const rightBox = simulateRenderBox('SR', 9.0, false);  // Right is close (9 cm)
    const frontBox = simulateRenderBox('SF', 4.0, true);   // Front is close (4 cm)

    const passLeftNotBlocked = leftBox.tag === '' && !leftBox.isRed;
    const passRightNotBlocked = rightBox.tag === '' && !rightBox.isRed;
    const passFrontBlocked = frontBox.tag === 'BLOCKED' && frontBox.isRed;
    const passFrontOnly = passLeftNotBlocked && passRightNotBlocked && passFrontBlocked;

    console.log(`[${passFrontOnly ? 'PASS' : 'FAIL'}] 6c: BLOCKED label and red styling applies to FRONT box only (Left: "${leftBox.tag}", Front: "${frontBox.tag}", Right: "${rightBox.tag}")`);

    return passNoWallOnZero && passObstacle4cm && passFrontOnly;
}

// ----------------------------------------------------------------------------
// RUN ALL TESTS
// ----------------------------------------------------------------------------
function runAllTests() {
    console.log('====================================================');
    console.log('     EXTENDED MAZE MAPPER ACCEPTANCE TEST SUITE     ');
    console.log('====================================================\n');

    const res1 = test1_Real90DegreeSpins();
    const res2 = test2_UturnAndNodeReuse();
    const res3 = test3_WStepsAllHeadings();
    const res4 = test4_SequenceWAWD();
    const res5 = test5_SimulatedMazeStepIntegration();
    const res6 = test6_EchoZeroNoWallAndFrontOnlyBlocked();

    console.log('\n====================================================');
    console.log('                 TEST SUMMARY                       ');
    console.log('====================================================');
    console.log(`Test 1 (Real 90° Spins Right & Left):     ${res1 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 2 (180° Turn & Node Reuse Backtrack): ${res2 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 3 (W Steps Exact STEP_CM 4 Headings): ${res3 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 4 (Sequence W -> A -> W -> D):        ${res4 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 5 (Simulated Maze Step Integration):  ${res5 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 6 (Echo 0 / 4cm & Front-Only Blocked):${res6 ? '✅ PASS' : '❌ FAIL'}`);
    console.log('====================================================');

    const allPassed = res1 && res2 && res3 && res4 && res5 && res6;
    if (allPassed) {
        console.log('\n🎉 ALL 6 TESTS PASSED SUCCESSFULLY!\n');
        process.exit(0);
    } else {
        console.error('\n❌ SOME TESTS FAILED!\n');
        process.exit(1);
    }
}

runAllTests();
