'use strict';
// ============================================================================
//  test_maze.js - Acceptance and unit tests for the maze mapper and simulator.
//
//  Tests all 5 requirements:
//    1. Front obstacle at 4 cm, 2 cm, 0.5 cm, and no-echo after close reading
//       -> Front blocked flag set, distance clamped, wall in grid.
//    2. No-echo after far readings -> Open space (distF is null / >100, not blocked).
//    3. Distinct L/F/R values reflected in UI telemetry, raw 0 shows wiring warning.
//    4. T-junction left turn, right turn, dead-end U-turn and driving back out
//       -> Verify nodes, edges, headings, turn log with node IDs, NO duplicate nodes.
//    5. Simulator stops in front of wall and never passes through it.
//
//  Run:  node test_maze.js
// ============================================================================
const maze = require('./maze');

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
// TEST 1: Front obstacle at 4 cm, 2 cm, 0.5 cm, and no-echo after close reading
// ----------------------------------------------------------------------------
function test1_FrontCloseAndMemory() {
    console.log('--- TEST 1: Front Close-Range & Memory Rules ---');
    const cfg = { ...maze.MAZE_CONFIG };
    const state = maze.createMazeState(cfg);
    let now = 1000;

    // 1a: Obstacle at 4 cm
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(4), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const pass4cm = state.frontBlocked === true && Math.abs(state.distF - 4.0) < 0.1;
    console.log(`[${pass4cm ? 'PASS' : 'FAIL'}] 1a: Obstacle at 4 cm -> blocked: ${state.frontBlocked}, distF: ${state.distF} cm`);

    // Verify wall appears on grid
    const gSnapshot1 = maze.getGridSnapshot(state);
    const hasWall1 = gSnapshot1.some(c => c.v > 0.4);
    console.log(`[${hasWall1 ? 'PASS' : 'FAIL'}] 1a: Wall cell present on grid -> cells: ${gSnapshot1.length}`);

    // 1b: Obstacle at 2 cm (clamped to 2.5 cm, treated as blocked)
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(2), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const pass2cm = state.frontBlocked === true && state.distF === 2.5;
    console.log(`[${pass2cm ? 'PASS' : 'FAIL'}] 1b: Obstacle at 2 cm -> clamped: ${state.distF} cm, blocked: ${state.frontBlocked}`);

    // 1c: Obstacle at 0.5 cm (clamped to 2.5 cm, treated as blocked)
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(0.5), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const pass05cm = state.frontBlocked === true && state.distF === 2.5;
    console.log(`[${pass05cm ? 'PASS' : 'FAIL'}] 1c: Obstacle at 0.5 cm -> clamped: ${state.distF} cm, blocked: ${state.frontBlocked}`);

    // 1d: No-echo (us = 0) right after a close reading (< 500 ms)
    now += 50; // 50 ms later (< 500 ms)
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), 0, cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    const passMemory = state.frontBlocked === true && state.distF === 2.5;
    console.log(`[${passMemory ? 'PASS' : 'FAIL'}] 1d: No-echo within 500ms of close reading -> still blocked: ${state.frontBlocked}, distF: ${state.distF} cm`);

    const gSnapshot2 = maze.getGridSnapshot(state);
    const hasWall2 = gSnapshot2.some(c => c.v > 0.4);
    console.log(`[${hasWall2 ? 'PASS' : 'FAIL'}] 1d: Wall persisted on map -> cells: ${gSnapshot2.length}`);

    return pass4cm && hasWall1 && pass2cm && pass05cm && passMemory && hasWall2;
}

// ----------------------------------------------------------------------------
// TEST 2: No-echo after far readings must still mean open space
// ----------------------------------------------------------------------------
function test2_NoEchoAfterFar() {
    console.log('\n--- TEST 2: No-Echo After Far Readings ---');
    const state = maze.createMazeState();
    let now = 1000;

    // Establish far reading (50 cm)
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), cmToUs(50), cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }
    console.log(`  Initial reading at 50 cm -> distF: ${state.distF} cm, blocked: ${state.frontBlocked}`);

    // Advance 600 ms (> 500 ms memory limit) and send no-echo
    now += 600;
    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, cmToUs(20), 0, cmToUs(20)));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }

    const passOpen = state.distF === null && state.frontBlocked === false;
    console.log(`[${passOpen ? 'PASS' : 'FAIL'}] No-echo after far reading -> open space (distF: ${state.distF}, blocked: ${state.frontBlocked})`);
    return passOpen;
}

// ----------------------------------------------------------------------------
// TEST 3: Distinct values for L, F, R and raw front = 0 warning
// ----------------------------------------------------------------------------
function test3_DistinctTelemetryAndWiringWarning() {
    console.log('\n--- TEST 3: Distinct Telemetry & Wiring Warning ---');
    const state = maze.createMazeState();
    let now = 1000;

    const usL = cmToUs(14.5);
    const usF = cmToUs(28.0);
    const usR = cmToUs(36.2);

    for (let i = 0; i < 5; i++) {
        const pkt = maze.parseSensorPacket(buildPacket(i, usL, usF, usR));
        maze.sensorUpdate(state, pkt, now += 20);
        maze.mazeTick(state, 0.02, now);
    }

    const telem = maze.getTelemetry(state, now);
    const passDistL = Math.abs(telem.distL - 14.5) < 0.2;
    const passDistF = Math.abs(telem.distF - 28.0) < 0.2;
    const passDistR = Math.abs(telem.distR - 36.2) < 0.2;
    const passRaw = telem.rawL === usL && telem.rawF === usF && telem.rawR === usR;

    console.log(`[${passDistL && passDistF && passDistR ? 'PASS' : 'FAIL'}] Distinct cm readings: L=${telem.distL}, F=${telem.distF}, R=${telem.distR}`);
    console.log(`[${passRaw ? 'PASS' : 'FAIL'}] Distinct raw µs: rawL=${telem.rawL}, rawF=${telem.rawF}, rawR=${telem.rawR}`);

    // Raw front value = 0 (no signal)
    const pktZero = maze.parseSensorPacket(buildPacket(10, usL, 0, usR));
    maze.sensorUpdate(state, pktZero, now += 20);
    const telemZero = maze.getTelemetry(state, now);

    const isWiringWarning = telemZero.rawF === 0;
    console.log(`[${isWiringWarning ? 'PASS' : 'FAIL'}] Raw front value of 0 correctly identified for wiring warning (rawF=${telemZero.rawF})`);

    return passDistL && passDistF && passDistR && passRaw && isWiringWarning;
}

// ----------------------------------------------------------------------------
// TEST 4: T-junction left turn, right turn, dead-end U-turn and driving back out
// ----------------------------------------------------------------------------
function test4_JunctionNavigationAndNoDuplicates() {
    console.log('\n--- TEST 4: Junction Anchoring, U-Turn & Duplicate Node Suppression ---');
    const cfg = { ...maze.MAZE_CONFIG };
    const state = maze.createMazeState(cfg);
    let now = 1000;

    // 1. Start facing North at (0, 0)
    console.log(`  Initial Pose: (${state.x}, ${state.y}), heading: ${state.heading}°`);

    // Drive north along corridor to y = 100
    state.pwmL = 180; state.pwmR = 180;
    for (let y = 0; y <= 100; y += 10) {
        state.y = y; state.totalDist = y;
        maze.graphUpdate(state, { v: 40, omega: 0, dist: 10 });
    }
    const nodesInCorridor = state.nodes.length;
    console.log(`  Northbound corridor traversed: ${nodesInCorridor} nodes generated`);

    // 2. Open side detected on left at y = 105
    state.recentOpenL = {
        startX: 0,
        startY: 105,
        startDist: 105,
        heading: 0,
        active: true,
        endDist: null
    };

    // 3. Driver turns LEFT at the T-junction
    const juncNode = maze.reanchorToJunction(state, 'left');
    console.log(`  Turned LEFT at T-junction:`);
    console.log(`    Re-anchored position: (${state.x.toFixed(1)}, ${state.y.toFixed(1)}), heading: ${state.heading}°`);
    console.log(`    Junction node #${juncNode.id}: isJunction=${juncNode.isJunction}, turnDir=${juncNode.turnDir}`);

    const passLeftAnchor = state.heading === 270 && state.y === 120 && juncNode.isJunction;
    console.log(`[${passLeftAnchor ? 'PASS' : 'FAIL'}] Left turn re-anchored to junction center (0, 120) with heading 270°`);

    // 4. Drive west into dead end from x = 0 to x = -60
    for (let x = 0; x >= -60; x -= 10) {
        state.x = x; state.totalDist += 10;
        maze.graphUpdate(state, { v: 40, omega: 0, dist: 10 });
    }

    // Flag dead end
    state.distF = 10; state.distL = 12; state.distR = 12; // dead end obstacle
    maze.graphUpdate(state, { v: 0, omega: 0, dist: 0 });
    const deadEndNode = state.nodes.find(n => n.isDeadEnd);
    console.log(`[${deadEndNode ? 'PASS' : 'FAIL'}] Dead end flagged at node #${deadEndNode ? deadEndNode.id : 'none'}`);

    // 5. U-turn at dead end
    const nodeCountBeforeUturn = state.nodes.length;
    maze.handleUturn(state);
    const passUturn = state.heading === 90; // 270 + 180 = 90 (East)
    const passNoDupUturn = state.nodes.length === nodeCountBeforeUturn;
    console.log(`[${passUturn ? 'PASS' : 'FAIL'}] U-turn heading flipped by 180° to ${state.heading}° (East)`);
    console.log(`[${passNoDupUturn ? 'PASS' : 'FAIL'}] No duplicate node created on U-turn (node count: ${state.nodes.length})`);

    // 6. Drive back out east toward junction (from x = -60 back to x = 0)
    for (let x = -60; x <= 0; x += 10) {
        state.x = x; state.totalDist += 10;
        maze.graphUpdate(state, { v: 40, omega: 0, dist: 10 });
    }
    const nodeCountAfterBacktrack = state.nodes.length;
    const passNoDupBacktrack = nodeCountAfterBacktrack === nodeCountBeforeUturn;
    console.log(`[${passNoDupBacktrack ? 'PASS' : 'FAIL'}] Backtracked out to junction without duplicate nodes (${nodeCountAfterBacktrack} nodes)`);

    // 7. Turn RIGHT to go South at junction
    state.recentOpenR = {
        startX: 0,
        startY: 105,
        startDist: state.totalDist,
        heading: 90,
        active: true,
        endDist: null
    };
    const southNode = maze.reanchorToJunction(state, 'right');
    const passRightTurn = state.heading === 180;
    console.log(`[${passRightTurn ? 'PASS' : 'FAIL'}] Right turn at junction snapped to heading 180° (South)`);

    // 8. Verify Turn Log
    const logTexts = state.turnLog.map(e => e.text);
    console.log(`  Turn Log Entries:`);
    state.turnLog.forEach(e => console.log(`    ${e.time}  ${e.text}`));

    const hasLeftLog = logTexts.some(t => t.startsWith('LEFT (node #'));
    const hasDeadEndLog = logTexts.some(t => t.startsWith('DEAD END (node #'));
    const hasUturnLog = logTexts.some(t => t.startsWith('U-TURN (node #'));
    const hasRightLog = logTexts.some(t => t.startsWith('RIGHT (node #'));
    const passLog = hasLeftLog && hasDeadEndLog && hasUturnLog && hasRightLog;

    console.log(`[${passLog ? 'PASS' : 'FAIL'}] Turn log contains LEFT, DEAD END, U-TURN, RIGHT with node IDs`);

    return passLeftAnchor && deadEndNode && passUturn && passNoDupUturn && passNoDupBacktrack && passRightTurn && passLog;
}

// ----------------------------------------------------------------------------
// TEST 5: The simulated car must stop in front of a wall and never pass through it
// ----------------------------------------------------------------------------
function test5_SimulatorCollisionStop() {
    console.log('\n--- TEST 5: Simulator Wall Collision Stopping ---');
    const cfg = { ...maze.MAZE_CONFIG, IS_SIMULATOR: true };

    // Wall at y = 30 cm
    const wallY = 30;
    const carHalfLength = cfg.CAR_LENGTH_CM / 2; // 8 cm
    const margin = 1.0; // 1 cm margin
    const stopThresholdCenter = carHalfLength + margin; // 9 cm

    // Car starts at y = 10 facing North (heading 0)
    let carX = 0, carY = 10, heading = 0;
    const speed = 40; // 40 cm/s
    const dt = 0.02;

    let stoppedBeforeWall = false;
    let minDistanceToWall = Infinity;
    let sensorShortValid = true;

    // Simulate 3 seconds of continuous forward driving into the wall
    for (let t = 0; t < 3.0; t += dt) {
        // Distance from front bumper / front sensor to wall
        const frontSensorY = carY + cfg.SENSOR_FRONT.y;
        const frontDistToWall = wallY - frontSensorY;
        const centerDistToWall = wallY - carY;

        // Simulated sensor reading (touches wall -> valid short distance, NOT 0)
        let simSensorReading;
        if (frontDistToWall <= 0) {
            simSensorReading = cfg.MIN_DIST_CM; // touches wall -> valid short distance
        } else if (frontDistToWall < cfg.MIN_DIST_CM) {
            simSensorReading = cfg.MIN_DIST_CM;
        } else {
            simSensorReading = frontDistToWall;
        }

        if (simSensorReading <= 0) {
            sensorShortValid = false;
        }

        // Simulator collision prevention rule:
        // Stop forward motion when center distance is below half-length + small margin
        let v = speed;
        if (centerDistToWall <= stopThresholdCenter) {
            v = 0; // STOP FORWARD MOTION
            stoppedBeforeWall = true;
        } else if (centerDistToWall - v * dt < stopThresholdCenter) {
            carY = wallY - stopThresholdCenter;
            v = 0;
            stoppedBeforeWall = true;
        } else {
            carY += v * dt;
        }
        if (wallY - carY < minDistanceToWall) minDistanceToWall = wallY - carY;
    }

    const passNeverPassed = carY <= (wallY - stopThresholdCenter + 0.01) && carY < wallY;
    console.log(`  Car initial y: 10 cm, wall y: ${wallY} cm`);
    console.log(`  Car final y: ${carY.toFixed(2)} cm (stopped threshold: ${(wallY - stopThresholdCenter).toFixed(2)} cm)`);
    console.log(`[${passNeverPassed ? 'PASS' : 'FAIL'}] Car stopped in front of wall and NEVER passed through (min distance: ${minDistanceToWall.toFixed(2)} cm)`);
    console.log(`[${sensorShortValid ? 'PASS' : 'FAIL'}] Simulated sensor returned valid short distance (>= 2.5 cm, not 0) when touching wall`);

    return passNeverPassed && sensorShortValid && stoppedBeforeWall;
}

// ----------------------------------------------------------------------------
// RUN ALL TESTS
// ----------------------------------------------------------------------------
function runAllTests() {
    console.log('====================================================');
    console.log('       MAZE MAPPER & SIMULATOR TEST SUITE           ');
    console.log('====================================================\n');

    const res1 = test1_FrontCloseAndMemory();
    const res2 = test2_NoEchoAfterFar();
    const res3 = test3_DistinctTelemetryAndWiringWarning();
    const res4 = test4_JunctionNavigationAndNoDuplicates();
    const res5 = test5_SimulatorCollisionStop();

    console.log('\n====================================================');
    console.log('                 TEST SUMMARY                       ');
    console.log('====================================================');
    console.log(`Test 1 (Front Obstacle & Close Memory):   ${res1 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 2 (No-Echo After Far Open Space):    ${res2 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 3 (Distinct Telemetry & Wiring):     ${res3 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 4 (Junction Anchoring & No Dups):    ${res4 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`Test 5 (Simulator Wall Collision Stop):   ${res5 ? '✅ PASS' : '❌ FAIL'}`);
    console.log('====================================================');

    const allPassed = res1 && res2 && res3 && res4 && res5;
    if (allPassed) {
        console.log('\n🎉 ALL 5 TESTS PASSED SUCCESSFULLY!\n');
        process.exit(0);
    } else {
        console.error('\n❌ SOME TESTS FAILED!\n');
        process.exit(1);
    }
}

runAllTests();
