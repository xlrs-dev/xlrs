/** Matches firmware rc_config_data_t (RCConfig.h) */
export interface RcConfig {
  channel_function: number[];  // 4: which function (0=A,1=E,2=R,3=T) gets physical axis i
  invert: boolean[];
  calib_min: number[];
  calib_max: number[];
  calib_center: number[];
  deadzone: number[];
  rate: number[];
  expo: number[];
  cutoff_min: number[];
  cutoff_max: number[];
  high_pass_filter: boolean;
}

export const NUM_AXES = 4;
export const NUM_CHANNELS = 8;
export const LABELS = ['Aileron', 'Elevator', 'Rudder', 'Throttle'] as const;
export const AXIS_LABELS = ['Axis 0', 'Axis 1', 'Axis 2', 'Axis 3'] as const;

export function defaultRcConfig(): RcConfig {
  return {
    channel_function: [0, 1, 2, 3],
    invert: [true, false, false, true],
    calib_min: [2917, 2917, 2917, 2917],
    calib_max: [23420, 23420, 23420, 23420],
    calib_center: [13199, 13199, 13199, 13199],
    deadzone: [0.05, 0.05, 0.05, 0],
    rate: [0.7, 0.7, 0.7, 0.7],
    expo: [0.3, 0.3, 0.3, 0.3],
    cutoff_min: Array(NUM_CHANNELS).fill(1000),
    cutoff_max: Array(NUM_CHANNELS).fill(2000),
    high_pass_filter: false,
  };
}
