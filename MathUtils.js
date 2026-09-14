class Vec2 {
  x;
  y;

  constructor(x = 0.0, y = 0.0) {
    this.x = x;
    this.y = y;
  }

  set(x, y) {
    this.x = x;
    this.y = y;
  }

  magnitude() {
    return sqrt(this.x * this.x + this.y * this.y);
  }
}

function wrapAngle(angle) {
  angle %= TWO_PI;
  if (angle < 0.0) {
    angle += TWO_PI;
  }
  return angle;
}

function signedAngle(angle) {
  angle = wrapAngle(angle);
  return angle > PI ? angle - TWO_PI : angle;
}

function rpmFromRadians(radiansPerSecond) {
  return radiansPerSecond * 60.0 / TWO_PI;
}

function radiansFromRpm(rpm) {
  return rpm * TWO_PI / 60.0;
}

function formatSignedNumber(value, decimalPlaces) {
  let zeroThreshold = 0.5 * pow(10.0, -decimalPlaces);
  if (abs(value) < zeroThreshold) value = 0.0;
  let formatted = decimalPlaces == 0
    ? str(round(value))
    : nf(value, 1, decimalPlaces);
  return value > 0.0 ? "+" + formatted : formatted;
}

function clampMagnitude(value, maximum) {
  return constrain(value, -maximum, maximum);
}

function limitVector(vector, maximum) {
  let magnitudeSquared = vector.x * vector.x + vector.y * vector.y;
  if (magnitudeSquared > maximum * maximum) {
    let scale = maximum / sqrt(magnitudeSquared);
    vector.x *= scale;
    vector.y *= scale;
  }
}

function finiteValue(value) {
  return Number.isFinite(value);
}
