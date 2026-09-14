class MotorParameters {
  statorResistance = 1.2;
  inductanceD = 0.006;
  inductanceQ = 0.006;
  magnetFlux = 1.25;
  polePairs = 1;

  inertia = 0.006;
  viscousFriction = 0.0015;
  coulombFriction = 0.055;
  frictionSmoothingSpeed = 0.8;

  maximumVoltage = 311.0;
  maximumCurrent = 25.0;
  maximumLoadTorque = 50.0;
}

class MotorState {
  currentD;
  currentQ;
  mechanicalAngle;
  mechanicalSpeed;
  simulationTime;

  electricalAngle;
  electricalSpeed;
  currentAlpha;
  currentBeta;
  voltageAlpha;
  voltageBeta;
  emfAlpha;
  emfBeta;
  electromagneticTorque;
  frictionTorque;
  loadTorque;
}

class StateDerivative {
  currentD;
  currentQ;
  mechanicalAngle;
  mechanicalSpeed;
}

class PMSMModel {
  parameters;
  state = new MotorState();

  k1 = new StateDerivative();
  k2 = new StateDerivative();
  k3 = new StateDerivative();
  k4 = new StateDerivative();
  temporaryState = new MotorState();

  constructor(parameters) {
    this.parameters = parameters;
    this.reset();
  }

  reset() {
    this.state.currentD = 0.0;
    this.state.currentQ = 0.0;
    this.state.mechanicalAngle = 0.0;
    this.state.mechanicalSpeed = 0.0;
    this.state.simulationTime = 0.0;
    this.state.loadTorque = 0.0;
    this.updateDerivedValues(0.0, 0.0, 0.0);
  }

  step(voltageAlpha, voltageBeta, loadTorque, timeStep) {
    this.evaluateDerivative(this.state, voltageAlpha, voltageBeta, loadTorque, this.k1);

    this.makeTemporaryState(this.state, this.k1, timeStep * 0.5);
    this.evaluateDerivative(this.temporaryState, voltageAlpha, voltageBeta, loadTorque, this.k2);

    this.makeTemporaryState(this.state, this.k2, timeStep * 0.5);
    this.evaluateDerivative(this.temporaryState, voltageAlpha, voltageBeta, loadTorque, this.k3);

    this.makeTemporaryState(this.state, this.k3, timeStep);
    this.evaluateDerivative(this.temporaryState, voltageAlpha, voltageBeta, loadTorque, this.k4);

    const sixthStep = timeStep / 6.0;
    this.state.currentD += sixthStep * (this.k1.currentD + 2.0 * this.k2.currentD
      + 2.0 * this.k3.currentD + this.k4.currentD);
    this.state.currentQ += sixthStep * (this.k1.currentQ + 2.0 * this.k2.currentQ
      + 2.0 * this.k3.currentQ + this.k4.currentQ);
    this.state.mechanicalAngle += sixthStep * (this.k1.mechanicalAngle
      + 2.0 * this.k2.mechanicalAngle + 2.0 * this.k3.mechanicalAngle
      + this.k4.mechanicalAngle);
    this.state.mechanicalSpeed += sixthStep * (this.k1.mechanicalSpeed
      + 2.0 * this.k2.mechanicalSpeed + 2.0 * this.k3.mechanicalSpeed
      + this.k4.mechanicalSpeed);
    this.state.mechanicalAngle = wrapAngle(this.state.mechanicalAngle);
    this.state.simulationTime += timeStep;

    if (!this.stateIsFinite()) {
      this.reset();
      return;
    }
    this.updateDerivedValues(voltageAlpha, voltageBeta, loadTorque);
  }

  makeTemporaryState(source, derivative, timeOffset) {
    this.temporaryState.currentD = source.currentD + derivative.currentD * timeOffset;
    this.temporaryState.currentQ = source.currentQ + derivative.currentQ * timeOffset;
    this.temporaryState.mechanicalAngle = source.mechanicalAngle
      + derivative.mechanicalAngle * timeOffset;
    this.temporaryState.mechanicalSpeed = source.mechanicalSpeed
      + derivative.mechanicalSpeed * timeOffset;
  }

  evaluateDerivative(sample, voltageAlpha, voltageBeta, loadTorque, derivative) {
    const electricalAngle = this.parameters.polePairs * sample.mechanicalAngle;
    const electricalSpeed = this.parameters.polePairs * sample.mechanicalSpeed;
    const cosine = cos(electricalAngle);
    const sine = sin(electricalAngle);

    const voltageD = cosine * voltageAlpha + sine * voltageBeta;
    const voltageQ = -sine * voltageAlpha + cosine * voltageBeta;

    derivative.currentD = (voltageD
      - this.parameters.statorResistance * sample.currentD
      + electricalSpeed * this.parameters.inductanceQ * sample.currentQ)
      / this.parameters.inductanceD;
    derivative.currentQ = (voltageQ
      - this.parameters.statorResistance * sample.currentQ
      - electricalSpeed * (this.parameters.inductanceD * sample.currentD
      + this.parameters.magnetFlux)) / this.parameters.inductanceQ;

    const torque = this.electromagneticTorque(sample.currentD, sample.currentQ);
    const friction = this.frictionTorque(sample.mechanicalSpeed, torque - loadTorque);
    derivative.mechanicalSpeed = (torque - loadTorque - friction) / this.parameters.inertia;
    derivative.mechanicalAngle = sample.mechanicalSpeed;
  }

  electromagneticTorque(currentD, currentQ) {
    return 1.5 * this.parameters.polePairs
      * (this.parameters.magnetFlux * currentQ
      + (this.parameters.inductanceD - this.parameters.inductanceQ) * currentD * currentQ);
  }

  frictionTorque(mechanicalSpeed, torqueBeforeFriction) {
    if (abs(mechanicalSpeed) < 0.015
        && abs(torqueBeforeFriction) <= this.parameters.coulombFriction) {
      return torqueBeforeFriction;
    }
    return this.parameters.viscousFriction * mechanicalSpeed
      + this.parameters.coulombFriction
      * Math.tanh(mechanicalSpeed / this.parameters.frictionSmoothingSpeed);
  }

  updateDerivedValues(voltageAlpha, voltageBeta, loadTorque) {
    this.state.electricalAngle = wrapAngle(
      this.parameters.polePairs * this.state.mechanicalAngle,
    );
    this.state.electricalSpeed = this.parameters.polePairs * this.state.mechanicalSpeed;

    const cosine = cos(this.state.electricalAngle);
    const sine = sin(this.state.electricalAngle);
    this.state.currentAlpha = cosine * this.state.currentD - sine * this.state.currentQ;
    this.state.currentBeta = sine * this.state.currentD + cosine * this.state.currentQ;
    this.state.voltageAlpha = voltageAlpha;
    this.state.voltageBeta = voltageBeta;
    this.state.emfAlpha = -this.parameters.magnetFlux * this.state.electricalSpeed * sine;
    this.state.emfBeta = this.parameters.magnetFlux * this.state.electricalSpeed * cosine;
    this.state.electromagneticTorque = this.electromagneticTorque(
      this.state.currentD,
      this.state.currentQ,
    );
    this.state.frictionTorque = this.frictionTorque(
      this.state.mechanicalSpeed,
      this.state.electromagneticTorque - loadTorque,
    );
    this.state.loadTorque = loadTorque;
  }

  stateIsFinite() {
    return finiteValue(this.state.currentD)
      && finiteValue(this.state.currentQ)
      && finiteValue(this.state.mechanicalAngle)
      && finiteValue(this.state.mechanicalSpeed);
  }
}
