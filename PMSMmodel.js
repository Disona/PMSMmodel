let motorParameters;
let controlSettings;
let motor;
let driveController;
let simulator;
let sketchLayout;
let motorView;
let controlPanel;

let simulationPaused = false;
let diagnosticsMode = false;

function setup() {
  document.title = "PMSM — визуальная модель синхронной машины";

  if (new URLSearchParams(window.location.search).has("self-test")) {
    diagnosticsMode = true;
    noCanvas();
    const failures = runSimulationDiagnostics();
    showDiagnosticResult(failures);
    noLoop();
    return;
  }

  const runningInProcessing = typeof window.pde !== "undefined";
  const canvas = createCanvas(
    runningInProcessing ? 1280 : windowWidth,
    runningInProcessing ? 720 : windowHeight,
  );
  const browserContainer = document.getElementById("app");
  if (browserContainer) canvas.parent(browserContainer);
  frameRate(60);

  motorParameters = new MotorParameters();
  controlSettings = new ControlSettings(motorParameters);
  motor = new PMSMModel(motorParameters);
  driveController = new DriveController(motorParameters, controlSettings);
  simulator = new FixedStepSimulator(motor, driveController, controlSettings);
  sketchLayout = new SketchLayout();
  motorView = new MotorView(motorParameters, controlSettings);
  controlPanel = new ControlPanel(motorParameters, controlSettings, driveController);
}

function draw() {
  if (diagnosticsMode) return;

  sketchLayout.update(width, height);
  simulator.advanceFrame(simulationPaused);
  motorView.updateReferenceFrame(motor.state);

  background(16, 20, 28);
  motorView.draw(sketchLayout.motorArea, motor, driveController, simulator);
  controlPanel.draw(sketchLayout.panelArea, motor, simulator);
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
}

function mousePressed() {
  if (controlPanel.mousePressed(mouseX, mouseY)) return false;
  motorView.mousePressed(mouseX, mouseY, sketchLayout.motorArea, driveController);
  return false;
}

function mouseDragged() {
  if (controlPanel.mouseDragged(mouseX, mouseY)) return false;
  motorView.mouseDragged(mouseX, mouseY, sketchLayout.motorArea, driveController);
  return false;
}

function mouseReleased() {
  controlPanel.mouseReleased();
  motorView.mouseReleased();
  return false;
}

function keyPressed() {
  if (key === " ") {
    simulationPaused = !simulationPaused;
  } else if (key === "r" || key === "R" || key === "к" || key === "К") {
    resetSimulation();
  }
}

function resetSimulation() {
  controlPanel.mouseReleased();
  motorView.mouseReleased();
  controlSettings.resetGuiParametersPreservingMode();
  controlPanel.syncWidgetsFromSettings();
  motor.reset();
  driveController.reset();
  simulator.resetClock();
  motorView.resetReferenceFrame();
}

function showDiagnosticResult(failures) {
  const output = document.createElement("main");
  output.className = failures === 0 ? "diagnostics diagnostics--pass" : "diagnostics diagnostics--fail";
  output.textContent = failures === 0
    ? "Все диагностические тесты PMSM пройдены."
    : `Диагностические тесты PMSM: ошибок — ${failures}. Подробности в консоли.`;
  document.body.append(output);
}
