class FixedStepSimulator {
  fixedTimeStep = 0.0001;
  simulatedTimePerFrame = 0.001;

  motor;
  controller;
  settings;
  stepsLastFrame = 0;

  constructor(motor, controller, settings) {
    this.motor = motor;
    this.controller = controller;
    this.settings = settings;
  }

  advanceFrame(paused) {
    this.stepsLastFrame = 0;
    if (paused) return;

    let stepsPerFrame = round(this.simulatedTimePerFrame / this.fixedTimeStep);
    for (let step = 0; step < stepsPerFrame; step++) {
      this.controller.update(this.motor.state, this.fixedTimeStep);
      this.motor.step(this.controller.voltageCommand.x, this.controller.voltageCommand.y,
        this.settings.loadTorque, this.fixedTimeStep);
      this.stepsLastFrame++;
    }
  }

  resetClock() {
    this.stepsLastFrame = 0;
  }
}
