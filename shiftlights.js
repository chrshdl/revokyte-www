// SPDX-License-Identifier: GPL-3.0-or-later
// Derived from chrshdl/revokyte (GPL-3.0-or-later); this file stays
// under that license, unlike the rest of the site's assets.
//
// JS port of the real device's shift-light ECU (ecu.py): EngineModel,
// ShiftPointCalculator and ShiftLightController, plus the pair/alert
// rendering rules from shift_lights.py. Kept structurally 1:1 with the
// Python so behaviour (thresholds, hysteresis, blink timing) matches
// the hardware; only the debug plotting and TCS/ASM paths are omitted
// (the device code has them disabled too).

// EngineModel torque curve shape
const TORQUE_LOW_BLEND_BASE = 0.8; // fraction of max torque at rpm=0
const TORQUE_LOW_BLEND_SLOPE = 0.2; // additional fraction gained linearly to peak-torque rpm
const OVER_REV_TORQUE_DROP = 0.25; // fraction of peak-power torque lost by redline

// ShiftLightController gear-scale factors (window width multipliers per gear)
const GEAR_SCALE_1 = 1.15; // gear 1 gets more lead time
const GEAR_SCALE_2 = 1.05; // gear 2 gets slightly more lead time
const GEAR_SCALE_HIGH = 0.90; // gears >= 5 get a tighter window

// Default progressive shift-light activation fractions (fraction of RPM window)
const DEFAULT_SHIFT_FRACTIONS = [0.00, 0.35, 0.60, 0.75];

// Schmitt-trigger hysteresis to prevent RPM flicker around thresholds
const HYSTERESIS_RPM = 60.0;
const ALERT_EXIT_HYSTERESIS_RPM = 120.0;

// Shift-alert blink period in seconds
const BLINK_PERIOD_S = 0.10;

// RPM window size limits
const WINDOW_RPM_MIN = 800.0;
const WINDOW_RPM_MAX = 2000.0;

// Gear-ratio change tolerance — avoids recreating ShiftPointCalculator
// on floating-point noise
const RATIO_CHANGE_TOLERANCE = 1e-3;

// Default dt when none is provided or value is non-positive
const DEFAULT_DT_S = 0.016;

export const NUM_PIXELS = 8;

function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

// np.linspace: n points, both endpoints included
function linspace(a, b, n) {
  const out = new Array(n);
  const step = (b - a) / (n - 1);
  for (let i = 0; i < n; i++) out[i] = a + step * i;
  return out;
}

class EngineModel {
  constructor(maxPowerKw, maxPowerRpm, maxTorqueNm, maxTorqueRpm, redlineRpm) {
    this.redline = redlineRpm;
    this.maxPowerRpm = maxPowerRpm;
    this.maxTorqueRpm = maxTorqueRpm;
    this.maxTorqueNm = maxTorqueNm;

    this.torqueAtPowerPeak = (maxPowerKw * 9549) / maxPowerRpm;
  }

  /** Returns estimated torque in Nm. */
  getTorque(rpm) {
    if (rpm > this.redline) return 0.0;

    // if below peak torque do linear ramp up
    if (rpm < this.maxTorqueRpm) {
      const blend = TORQUE_LOW_BLEND_BASE + TORQUE_LOW_BLEND_SLOPE * (rpm / this.maxTorqueRpm);
      return this.maxTorqueNm * blend;
    }

    // ensure we are moving from max_torque down to torque_at_power_peak
    let dropRange = this.maxPowerRpm - this.maxTorqueRpm;
    if (dropRange <= 0) dropRange = 1.0;

    const dist = (rpm - this.maxTorqueRpm) / dropRange;
    const dropAmount = Math.max(0, this.maxTorqueNm - this.torqueAtPowerPeak);

    let torque = this.maxTorqueNm - dropAmount * dist ** 2;

    if (rpm > this.maxPowerRpm) {
      // force a drop of OVER_REV_TORQUE_DROP from peak power torque
      // by the time we hit redline
      const overRevRange = this.redline - this.maxPowerRpm;
      if (overRevRange > 0) {
        const pctPast = (rpm - this.maxPowerRpm) / overRevRange;
        torque *= 1.0 - OVER_REV_TORQUE_DROP * pctPast ** 2;
      }
    }

    return Math.max(0.0, torque);
  }
}

class ShiftPointCalculator {
  constructor(engine, gearRatios) {
    this.engine = engine;
    this.ratios = gearRatios;
    this.optimalShiftRpms = {};
    this._calculate();
  }

  _calculate() {
    // we scan the RPM range to find where wheel torque crosses over
    const scanRpms = linspace(this.engine.redline * 0.6, this.engine.redline, 250);

    for (let gearIdx = 0; gearIdx < this.ratios.length - 1; gearIdx++) {
      const currentRatio = this.ratios[gearIdx];
      const nextRatio = this.ratios[gearIdx + 1];

      let bestRpm = this.engine.redline;

      for (const rpm of scanRpms) {
        // Calculate what RPM we would land at in the next gear
        // Ratio of RPMs is inverse to Ratio of Gears
        const nextRpm = rpm * (nextRatio / currentRatio);

        // Torque at Wheels = Engine Torque * Gear Ratio
        // (We ignore final drive as it cancels out on both sides of equation)
        const torqueNow = this.engine.getTorque(rpm) * currentRatio;
        const torqueNext = this.engine.getTorque(nextRpm) * nextRatio;

        // The moment Next Gear gives more torque than Current Gear -> SHIFT!
        if (torqueNext > torqueNow) {
          bestRpm = rpm;
          break;
        }
      }

      // Map gear number (1-based) to the rpm
      this.optimalShiftRpms[gearIdx + 1] = bestRpm;
    }
  }

  getOptimalRpm(gear) {
    return this.optimalShiftRpms[gear] ?? this.engine.redline;
  }
}

export class ShiftLightController {
  constructor({
    name = 'Car Name',
    maxPowerKw = 400,
    maxPowerRpm = 8500,
    maxTorqueNm = 600,
    maxTorqueRpm = 6500,
    redlineRpm = 9000,
    shiftlightFractions = null,
    filterWindow = 3,
    targetCorridor = 1600.0,
  } = {}) {
    this.engine = new EngineModel(maxPowerKw, maxPowerRpm, maxTorqueNm, maxTorqueRpm, redlineRpm);
    this.calculator = null;
    this.lastGearRatios = null;

    // deque([0.0] * filter_window, maxlen=filter_window)
    this.filterWindow = filterWindow;
    this._rpmBuffer = new Array(filterWindow).fill(0.0);

    // Dynamic Window Config
    this.targetCorridor = targetCorridor;
    this.windowRpmMin = WINDOW_RPM_MIN;
    this.windowRpmMax = WINDOW_RPM_MAX;

    // Progressive fractions: each LED pair lights when RPM crosses
    // that fraction of the window below the shift point.
    this.fractions = shiftlightFractions || [...DEFAULT_SHIFT_FRACTIONS];

    // hysteresis: prevents flicker around thresholds
    this.hysRpm = HYSTERESIS_RPM;
    // alert hysteresis: prevents 1-frame overshoot
    this.alertExitHysRpm = ALERT_EXIT_HYSTERESIS_RPM;

    // blink timing in seconds
    this.blinkPeriod = BLINK_PERIOD_S;
    this._blinkT = 0.0;
    this._blinkOn = true;

    // state
    this._pairCount = 0;
    this._inAlert = false;
    this._targetRpm = 0.0;
    this._lastGear = 0;

    // cache
    this._thresholdsByGear = {};
    this._shiftRpmByGear = {};
  }

  _gearScale(gear) {
    if (gear === 1) return GEAR_SCALE_1;
    if (gear === 2) return GEAR_SCALE_2;
    if (gear >= 5) return GEAR_SCALE_HIGH;
    return 1.00;
  }

  /**
   * Window is 'targetCorridor' wide, widened for low gears
   * (more lead time) and tightened for high gears.
   */
  _computeWindowRpm(gear) {
    const base = this.targetCorridor * this._gearScale(gear);
    return clamp(base, this.windowRpmMin, this.windowRpmMax);
  }

  _computeThresholds(gear, shiftRpm) {
    const window = this._computeWindowRpm(gear);
    const startRpm = shiftRpm - window;
    // thresholds are increasing RPM points
    // where each additional pair turns on
    return this.fractions.map((f) => startRpm + f * window);
  }

  _resetStatesOnGearChange(gear) {
    this._pairCount = 0;
    this._inAlert = false;
    this._blinkT = 0.0;
    this._blinkOn = true;
    this._lastGear = gear;
  }

  _updateBlink(dt) {
    // toggles every blink_period; starts as "ON" on entry
    this._blinkT += dt;
    while (this._blinkT >= this.blinkPeriod) {
      this._blinkT = 0.0;
      this._blinkOn = !this._blinkOn;
    }
    return this._blinkOn;
  }

  /**
   * Schmitt-trigger state machine that maps
   * RPM to pair_count with hysteresis.
   */
  _updatePairCount(rpm, thresholds) {
    // this is the state (memory)
    let pc = this._pairCount;

    // step up as RPM crosses the upper threshold
    while (pc < thresholds.length && rpm >= thresholds[pc]) pc += 1;

    // step down only after dropping below lower threshold (threshold - hysteresis)
    while (pc > 0 && rpm < thresholds[pc - 1] - this.hysRpm) pc -= 1;

    this._pairCount = pc;
    return pc;
  }

  /**
   * frame: { currentGear, engineRpm, gearRatios, rpmAlertMax,
   *          revLimiterAlertActive }
   * Returns { leds: bool[NUM_PIXELS], isAlert }.
   */
  calculateLights(frame, dt = null) {
    dt = dt == null || dt <= 0.0 ? DEFAULT_DT_S : clamp(dt, 0.001, 0.05);

    const gear = frame.currentGear;

    // filter RPM to stop red bleeding (median of the last N samples)
    this._rpmBuffer.push(Number(frame.engineRpm));
    if (this._rpmBuffer.length > this.filterWindow) this._rpmBuffer.shift();
    const buf = [...this._rpmBuffer].sort((a, b) => a - b);
    const mid = buf.length >> 1;
    const rpm = buf.length % 2 ? buf[mid] : 0.5 * (buf[mid - 1] + buf[mid]);

    const revAlert = !!frame.revLimiterAlertActive;

    if (gear <= 0) {
      // neutral / reverse -> no shift lights
      this._pairCount = 0;
      this._inAlert = false;
      return { leds: new Array(NUM_PIXELS).fill(false), isAlert: false };
    }

    // reset on gear change
    if (gear !== this._lastGear) this._resetStatesOnGearChange(gear);

    // update redline from telemetry; keep the configured redline if the
    // frame carries no rev-limit (rpmAlertMax == 0)
    if (frame.rpmAlertMax > 0) this.engine.redline = frame.rpmAlertMax;

    // build calculator if ratios changed, with a tolerance check to
    // prevent recreating ShiftPointCalculator on floating-point noise
    if (frame.gearRatios) {
      let ratiosChanged = false;
      if (this.lastGearRatios === null) {
        ratiosChanged = true;
      } else if (frame.gearRatios.length !== this.lastGearRatios.length) {
        ratiosChanged = true;
      } else {
        for (let i = 0; i < frame.gearRatios.length; i++) {
          if (Math.abs(frame.gearRatios[i] - this.lastGearRatios[i]) > RATIO_CHANGE_TOLERANCE) {
            ratiosChanged = true;
            break;
          }
        }
      }

      if (ratiosChanged) {
        this.lastGearRatios = frame.gearRatios;
        this.calculator = new ShiftPointCalculator(this.engine, frame.gearRatios);
        this._thresholdsByGear = {};
        this._shiftRpmByGear = {};
      }
    }

    if (!this.calculator) {
      return { leds: new Array(NUM_PIXELS).fill(false), isAlert: false };
    }

    // shift RPM target for this gear
    const optimal = this.calculator.getOptimalRpm(gear);
    const shiftRpm = Math.min(optimal, this.engine.redline - 40.0);
    this._targetRpm = shiftRpm;

    if (
      !(gear in this._thresholdsByGear) ||
      Math.abs((this._shiftRpmByGear[gear] ?? 0) - shiftRpm) > 5.0
    ) {
      this._shiftRpmByGear[gear] = shiftRpm;
      this._thresholdsByGear[gear] = this._computeThresholds(gear, shiftRpm);
    }

    const thresholds = this._thresholdsByGear[gear];

    // enter alert if either rev limiter alert is active
    // or we exceeded shift_rpm
    const enterAlert = revAlert || rpm >= shiftRpm;

    // exit alert only if the flag is off
    // and RPM has fallen below the exit threshold
    const exitAlert = !revAlert && rpm <= shiftRpm - this.alertExitHysRpm;

    if (!this._inAlert) {
      if (enterAlert) {
        this._inAlert = true;
        this._blinkT = 0.0;
        this._blinkOn = true;
      }
    } else if (exitAlert) {
      this._inAlert = false;
      this._blinkT = 0.0;
      this._blinkOn = true;
    }

    if (this._inAlert) {
      const on = this._updateBlink(dt);
      return { leds: new Array(NUM_PIXELS).fill(on), isAlert: true };
    }

    const pairs = this._updatePairCount(rpm, thresholds);

    const leds = new Array(NUM_PIXELS).fill(false);
    for (let i = 0; i < pairs; i++) {
      leds[i] = leds[NUM_PIXELS - 1 - i] = true;
    }

    return { leds, isAlert: false };
  }

  get targetRpm() {
    return this._targetRpm;
  }
}
