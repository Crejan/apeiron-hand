# Error codes

Joints report their active error in `error_code_current` of every realtime entry and in
the per-joint register `error_status`. `0` means no active error. The table is provided
by `apeiron_protocol::error_codes` with title, description and remedy; `apeiron
telemetry` and the commissioning reports print names instead of numbers.

## Severity and clearing

| Severity | Effect |
|---|---|
| Warning | informational, motion continues |
| DeferredStop | controlled stop |
| ImmediateStop | motors cut |
| Error, Fatal | board-level fault |

| Clearing | How it clears |
|---|---|
| AutoClear | by itself once the condition is gone |
| ManualClear | `clear_fault` action |
| NonClearable | power cycle or service |

## Routine codes

- **6 `BusFrameLossHigh`** is present on all joints after boot and on some joints
  permanently; it is a warning about internal bus frame formatting and does not affect
  operation.
- **3 `Enc1BitRate`** may appear transiently on a joint under load.

## Frozen joint encoder

A state the firmware does not report: `od/pos_rad_jnt_enc2` reads exactly `0.0` while
`od/pos_rad_jnt_enc1` moves with the joint and `valid_enc2` stays 1. The reported
position then stays constant and the controller pushes against a phantom target with the
full current limit. A power cycle of the hand clears it. The driver checks all joints for
this signature before every `motor_enable` and refuses to drive an affected joint.

## Table

| Code | Name | Severity | Clearing | Title |
|---|---|---|---|---|
| 1 | TestWarning | Warning | AutoClear | Test-injected warning |
| 2 | TemperatureApproaching | Warning | AutoClear | Temperature approaching the limit |
| 3 | Enc1BitRate | Warning | AutoClear | Encoder 1 data-quality flag rate is high |
| 4 | Enc2BitRate | Warning | AutoClear | Encoder 2 data-quality flag rate is high |
| 5 | CommCrcRateHigh | Warning | AutoClear | Communication CRC error rate is high |
| 6 | BusFrameLossHigh | Warning | AutoClear | Bus frame-format error rate is high |
| 7 | Stall | Warning | AutoClear | Motor stall detected |
| 8 | Enc1NotCalibrated | Warning | AutoClear | Encoder 1 is not calibrated |
| 9 | BootConfigUsingDefaults | Warning | AutoClear | The device started using compile-time defaults |
| 16 | Enc1CalibFailed | Warning | AutoClear | Encoder 1 calibration failed |
| 17 | Enc2CalibFailed | Warning | AutoClear | Encoder 2 calibration failed |
| 18 | Enc2NotCalibrated | Warning | AutoClear | Encoder 2 is not calibrated |
| 19 | Enc1CrcRate | Warning | AutoClear | Encoder 1 CRC error rate is high |
| 20 | Enc2CrcRate | Warning | AutoClear | Encoder 2 CRC error rate is high |
| 21 | MotorParamNotMigrated | Warning | AutoClear | Motor parameters have not been restored to the unit-specific tuned values |
| 22 | Enc1AutocalFailed | Warning | AutoClear | Encoder 1 on-chip self-calibration failed |
| 257 | TestWarningManual | Warning | ManualClear | Test-injected warning that requires manual clearing |
| 258 | IwdgResetDetected | Warning | ManualClear | The previous reset was caused by the independent watchdog |
| 4097 | TestDeferredAuto | DeferredStop | AutoClear | Test-injected deferred-stop fault |
| 4098 | CommTimeout | DeferredStop | AutoClear | Host communication timed out |
| 4099 | OverTemperature | DeferredStop | AutoClear | Temperature limit exceeded |
| 4353 | TestDeferredManual | DeferredStop | ManualClear | Test-injected deferred-stop fault that requires manual clearing |
| 4354 | PreChargeTimeout | DeferredStop | ManualClear | DC-bus precharge timed out |
| 4355 | BusRecoveryFailed | DeferredStop | ManualClear | Bus communication failed to recover |
| 4356 | IwdgProtectiveMode | DeferredStop | ManualClear | Repeated watchdog resets; minimal-service mode is active |
| 8193 | TestImmediateAuto | ImmediateStop | AutoClear | Test-injected immediate-stop fault |
| 8194 | Undervoltage | ImmediateStop | AutoClear | DC-bus undervoltage |
| 8195 | Overvoltage | ImmediateStop | AutoClear | DC-bus overvoltage |
| 8449 | TestImmediateManual | ImmediateStop | ManualClear | Test-injected immediate-stop fault that requires manual clearing |
| 8450 | Overcurrent | ImmediateStop | ManualClear | Phase overcurrent |
| 8451 | GateDriverFault | ImmediateStop | ManualClear | Gate-driver fault |
| 8452 | AdcSensorFailure | ImmediateStop | ManualClear | Invalid ADC or sensor reading |
| 8453 | EnableRejectedNoCalib | ImmediateStop | ManualClear | Enable request rejected: calibration is incomplete |
| 8454 | Enc1CommStuck | ImmediateStop | ManualClear | Encoder 1 communication timed out |
| 8455 | Enc2CommStuck | ImmediateStop | ManualClear | Encoder 2 communication timed out |
| 8456 | Enc1CrcPersistent | ImmediateStop | ManualClear | Persistent Encoder 1 CRC failures |
| 8457 | Enc1BitPersistent | ImmediateStop | ManualClear | Encoder 1 error flag remains asserted |
| 8464 | Enc2CrcPersistent | ImmediateStop | ManualClear | Persistent Encoder 2 CRC failures |
| 8465 | Enc2BitPersistent | ImmediateStop | ManualClear | Encoder 2 error flag remains asserted |
| 8466 | FlashEccDoubleError | ImmediateStop | ManualClear | Uncorrectable Flash ECC error; persistent data is corrupted |
| 8467 | UnknownFaultTransition | ImmediateStop | ManualClear | Fault-state transition without a specific active error code |
| 8468 | MotorPhaseOpen | ImmediateStop | ManualClear | Open motor phase-current path |
| 8469 | PhaseCurrentImbalance | ImmediateStop | ManualClear | Abnormal three-phase current-sum residual |
| 8470 | InitAbortClock | ImmediateStop | ManualClear | Previous boot aborted during system-clock initialization |
| 8471 | InitAbortPeriph | ImmediateStop | ManualClear | Previous boot aborted during peripheral initialization |
| 8472 | InitAbortBus | ImmediateStop | ManualClear | Previous boot aborted during bus-slave initialization |
| 8473 | InitAbortSpawn | ImmediateStop | ManualClear | Previous boot aborted during task startup |
| 8480 | InitAbortUnknown | ImmediateStop | ManualClear | Previous boot aborted at an initialization stage without its own code |
| 8481 | InitAbortMotorAdc | ImmediateStop | ManualClear | Previous boot aborted during motor ADC initialization |
| 8482 | InitAbortMotorEncoderSpi | ImmediateStop | ManualClear | Previous boot aborted during encoder SPI initialization |
| 8720 | HardwareFault | Error | NonClearable | Internal hardware fault |
| 12801 | TestFatal | Fatal | NonClearable | Test-injected fatal fault |
| 12802 | MultipleOvercurrent | Fatal | NonClearable | Repeated-overcurrent lockout |
| 12803 | HardwareDamage | Fatal | NonClearable | Severe hardware fault |
| 12804 | InitFailed | Fatal | NonClearable | Initialization failed |
| 12805 | SelfCheckFail | Fatal | NonClearable | Self-check failed |
| 12806 | CalibPhaseSeqMismatch | Fatal | NonClearable | Stored calibration phase sequence does not match the current configuration |
| 12807 | InitGateDriverFault | Fatal | NonClearable | Gate-driver fault during initialization |
| 12808 | InitCurrentCalibFailed | Fatal | NonClearable | Current-sense offset calibration failed during initialization |
| 12809 | InitCurrentSampleTimeout | Fatal | NonClearable | The current-sampling path did not respond during initialization |

Codes are grouped by severity in their high byte: `0x00xx` warnings, `0x01xx` warnings
needing manual clearing, `0x10xx`/`0x11xx` deferred stops, `0x20xx`/`0x21xx` immediate
stops, `0x22xx` errors, `0x32xx` fatal.
