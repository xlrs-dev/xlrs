# Safety And Failsafe

Default failsafe mode is `NoPulses`.

When the link is not valid and `NoPulses` is active, the RX stops emitting CRSF
RC channel frames so the flight controller can enter RXLOSS and own aircraft
policy. `Hold` mode is available as an explicit configuration option.

```mermaid
stateDiagram-v2
    [*] --> Disconnected : Boot/Initialization

    state Connected {
        [*] --> LinkActive
        LinkActive --> StandardOutput : RC Frame Received
        StandardOutput --> Send_RC_And_Stats : outputActive = true
        Send_RC_And_Stats --> LinkActive
    }

    Disconnected --> Connected : Bind Successful / Link Sync

    Connected --> FailsafeTriggered : Missed Slots >= Failsafe Threshold (Debounced)

    state FailsafeTriggered {
        [*] --> EvaluateMode

        state EvaluateMode {
            state "Default: NoPulses Mode" as NoPulses {
                [*] --> StopRcFrame : Stop CRSF_RC_CHANNELS
                StopRcFrame --> FC_Enters_RXLOSS : Flight Controller triggers failsafe
                FC_Enters_RXLOSS --> GracePeriod : Emit CRSF_LINK_STATS (LQ=0) for grace window
                GracePeriod --> StopAllOutput : Silence all CRSF output after window
            }

            state "Opt-In: Hold Mode" as Hold {
                [*] --> UseHoldPositions : Load Preset Positions
                UseHoldPositions --> EmitHoldRc : Emit CRSF_RC_CHANNELS with presets
            }
        }
    }

    FailsafeTriggered --> Connected : Valid Uplink Frame Received (Recovery)
    Disconnected --> Disconnected : No Sync / Searching
```

Safety-sensitive areas:


- CRSF output gating.
- Failsafe thresholds and missed-uplink-slot accounting.
- Link-state transitions.
- RF timing and scheduler deadlines.
- RF region tables and TX power limits.
- Dynamic power emergency boost behavior.

Changes here should include native tests where possible and hardware validation
notes where native tests cannot prove the behavior.
