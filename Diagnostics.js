let diagnosticFailures = 0;

function runSimulationDiagnostics() {
  diagnosticFailures = 0;
  console.log("=== PMSM simulation diagnostics ===");
  testCoordinateTransform();
  testManualCurrentStep();
  testManualBackEmfCompensation();
  testVectorCurrentStep();
  testSpeedLoopStep();
  testNegativeSpeedLoopStep();
  testCurrentAntiWindup();
  testGuiParameterReset();

  if (diagnosticFailures > 0) {
    console.log("PMSM diagnostics failed: " + diagnosticFailures);
    return diagnosticFailures;
  }
  console.log("All PMSM diagnostics passed.");
  return diagnosticFailures;
}

function testCoordinateTransform() {
  let testParameters = new MotorParameters();
  let testMotor = new PMSMModel(testParameters);
  testMotor.state.currentD = 7.0;
  testMotor.state.currentQ = -4.0;
  testMotor.state.mechanicalAngle = 1.1;
  testMotor.updateDerivedValues(0.0, 0.0, 0.0);

  let dqMagnitude = sqrt(7.0 * 7.0 + 4.0 * 4.0);
  let alphaBetaMagnitude = sqrt(
    testMotor.state.currentAlpha * testMotor.state.currentAlpha
    + testMotor.state.currentBeta * testMotor.state.currentBeta);
  diagnosticNear("Park transform preserves current magnitude",
    alphaBetaMagnitude, dqMagnitude, 0.0001);
}

function testManualCurrentStep() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  let testMotor = new PMSMModel(testParameters);
  let testController = new DriveController(testParameters, testSettings);
  testSettings.mode = MODE_MANUAL;
  testController.setManualCurrent(10.0, 0.0);

  runDiagnosticSteps(testMotor, testController, testSettings, 250);
  diagnosticNear("Manual alpha current step", testMotor.state.currentAlpha, 10.0, 0.05);
  diagnosticNear("Manual beta cross error", testMotor.state.currentBeta, 0.0, 0.02);
}

function testManualBackEmfCompensation() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  let testMotor = new PMSMModel(testParameters);
  let testController = new DriveController(testParameters, testSettings);
  testSettings.mode = MODE_MANUAL;
  testMotor.state.mechanicalSpeed = 100.0;
  testMotor.updateDerivedValues(0.0, 0.0, 0.0);

  runDiagnosticSteps(testMotor, testController, testSettings, 100);
  let currentMagnitude = sqrt(
    testMotor.state.currentAlpha * testMotor.state.currentAlpha
    + testMotor.state.currentBeta * testMotor.state.currentBeta);
  diagnosticLessThan("Manual back-EMF rejection", currentMagnitude, 0.15);
}

function testVectorCurrentStep() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  let testMotor = new PMSMModel(testParameters);
  let testController = new DriveController(testParameters, testSettings);
  testSettings.mode = MODE_VECTOR;
  testSettings.speedLoopEnabled = false;
  testSettings.currentQReference = 3.0;

  runDiagnosticSteps(testMotor, testController, testSettings, 500);
  diagnosticNear("Vector id regulation", testMotor.state.currentD, 0.0, 0.05);
  diagnosticNear("Vector iq regulation", testMotor.state.currentQ, 3.0, 0.08);
}

function testSpeedLoopStep() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  let testMotor = new PMSMModel(testParameters);
  let testController = new DriveController(testParameters, testSettings);
  testSettings.mode = MODE_VECTOR;
  testSettings.speedLoopEnabled = true;
  testSettings.speedReferenceRpm = 1000.0;

  let maximumRpm = 0.0;
  let maximumCurrentReference = 0.0;
  let maximumVoltage = 0.0;
  let peakTime = 0.0;
  for (let step = 0; step < 10000; step++) {
    runDiagnosticStep(testMotor, testController, testSettings);
    let currentRpm = rpmFromRadians(testMotor.state.mechanicalSpeed);
    if (currentRpm > maximumRpm) {
      maximumRpm = currentRpm;
      peakTime = testMotor.state.simulationTime;
    }
    maximumCurrentReference = max(maximumCurrentReference,
      abs(testController.currentQReference));
    maximumVoltage = max(maximumVoltage, testController.voltageCommand.magnitude());
  }
  let finalRpm = rpmFromRadians(testMotor.state.mechanicalSpeed);
  console.log("INFO: speed peak at " + peakTime + " s; max iq* = "
    + maximumCurrentReference + " A; max |u| = " + maximumVoltage + " V");
  diagnosticNear("Speed loop steady state", finalRpm, 1000.0, 10.0);
  diagnosticLessThan("Speed loop overshoot", maximumRpm, 1050.0);

  testSettings.loadTorque = 10.0;
  let minimumLoadedRpm = finalRpm;
  for (let step = 0; step < 10000; step++) {
    runDiagnosticStep(testMotor, testController, testSettings);
    minimumLoadedRpm = min(minimumLoadedRpm,
      rpmFromRadians(testMotor.state.mechanicalSpeed));
  }
  let loadedFinalRpm = rpmFromRadians(testMotor.state.mechanicalSpeed);
  console.log("INFO: 10 Nm load step minimum = " + minimumLoadedRpm
    + " rpm; final = " + loadedFinalRpm + " rpm");
  diagnosticLessThan("Speed load-step dip", 1000.0 - minimumLoadedRpm, 200.0);
  diagnosticNear("Speed load rejection", loadedFinalRpm, 1000.0, 10.0);
}

function testCurrentAntiWindup() {
  let testRegulator = new PIRegulator();
  let timeStep = 0.0001;
  for (let step = 0; step < 10000; step++) {
    let rawOutput = testRegulator.calculate(100.0, 4.0, 800.0, timeStep);
    let saturatedOutput = constrain(rawOutput, -10.0, 10.0);
    testRegulator.applyTracking(saturatedOutput - rawOutput, 4.0, 800.0, timeStep);
  }
  diagnosticLessThan("Current PI anti-windup", abs(testRegulator.integrator), 20.0);
}

function testNegativeSpeedLoopStep() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  let testMotor = new PMSMModel(testParameters);
  let testController = new DriveController(testParameters, testSettings);
  testSettings.mode = MODE_VECTOR;
  testSettings.speedLoopEnabled = true;
  testSettings.speedReferenceRpm = -1000.0;

  runDiagnosticSteps(testMotor, testController, testSettings, 10000);
  let finalRpm = rpmFromRadians(testMotor.state.mechanicalSpeed);
  diagnosticNear("Negative speed command", finalRpm, -1000.0, 10.0);
  if (formatSignedNumber(finalRpm, 0).startsWith("-")) {
    console.log("PASS: Negative speed display sign (" + formatSignedNumber(finalRpm, 0) + ")");
  } else {
    diagnosticFailures++;
    console.log("FAIL: Negative speed display sign (" + formatSignedNumber(finalRpm, 0) + ")");
  }
}

function testGuiParameterReset() {
  let testParameters = new MotorParameters();
  let testSettings = new ControlSettings(testParameters);
  testSettings.mode = MODE_VECTOR;
  testSettings.loadTorque = 17.0;
  testSettings.openLoopVoltage = 200.0;
  testSettings.openLoopFrequency = 42.0;
  testSettings.currentKp = 12.0;
  testSettings.speedReferenceRpm = -730.0;
  testSettings.speedLoopEnabled = true;
  testSettings.showVoltage = false;
  testSettings.lockDqFrame = true;

  testSettings.resetGuiParametersPreservingMode();
  diagnosticNear("Reset restores load", testSettings.loadTorque, 0.0, 0.0001);
  diagnosticNear("Reset restores open-loop voltage", testSettings.openLoopVoltage, 0.0, 0.0001);
  diagnosticNear("Reset restores open-loop frequency", testSettings.openLoopFrequency, 0.0, 0.0001);
  diagnosticNear("Reset restores current Kp", testSettings.currentKp, 4.0, 0.0001);
  diagnosticNear("Reset restores speed reference", testSettings.speedReferenceRpm, 0.0, 0.0001);
  if (testSettings.mode == MODE_VECTOR && !testSettings.speedLoopEnabled
      && testSettings.showVoltage && !testSettings.lockDqFrame) {
    console.log("PASS: Reset preserves mode and restores checkboxes");
  } else {
    diagnosticFailures++;
    console.log("FAIL: Reset mode/checkbox behavior");
  }
}

function runDiagnosticSteps(testMotor, testController,
                        testSettings, stepCount) {
  for (let step = 0; step < stepCount; step++) {
    runDiagnosticStep(testMotor, testController, testSettings);
  }
}

function runDiagnosticStep(testMotor, testController,
                       testSettings) {
  let timeStep = 0.0001;
  testController.update(testMotor.state, timeStep);
  testMotor.step(testController.voltageCommand.x, testController.voltageCommand.y,
    testSettings.loadTorque, timeStep);
}

function diagnosticNear(name, actual, expected, tolerance) {
  if (abs(actual - expected) <= tolerance) {
    console.log("PASS: " + name + " (" + actual + ")");
  } else {
    diagnosticFailures++;
    console.log("FAIL: " + name + ", expected " + expected + " ± " + tolerance
      + ", actual " + actual);
  }
}

function diagnosticLessThan(name, actual, limit) {
  if (actual <= limit) {
    console.log("PASS: " + name + " (" + actual + ")");
  } else {
    diagnosticFailures++;
    console.log("FAIL: " + name + ", expected <= " + limit + ", actual " + actual);
  }
}
