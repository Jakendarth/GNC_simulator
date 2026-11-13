# Simulation scenarios

The repository contains several JSON configurations and pipeline implementations. Their compatibility differs; the current default is the anti-grounding MPC scenario.

| Configuration or variant | Implementation | Current status |
| --- | --- | --- |
| `input/SimParam_anti_grounding.json` | `sim_ag_upg` | Default synthetic channel scenario using anti-grounding MPC |
| PID variant | `sim_ag_PID` | Active PID rudder control; select by changing the JSON import |
| `input/SimParam_WC.json` | `sim_colav` | Collision-avoidance scenario; guidance and drawing interfaces need reconciliation |
| `input/SimParam_ASKO.json` | `sim_ftc` | Older FTC scenario; four-argument entry point does not match current startup |
| `input/SimParam.json` | No import configured | Does not supply the `run_simulation` callable required by startup |

## Default anti-grounding scenario

The pipeline constructs an `Own` vessel and a sinusoidal channel route. LOS guidance provides heading references to MPC, which controls propeller speed and rudder angle with a soft clearance constraint. See [Guidance and anti-grounding control](openwiki/concepts/guidance-control.md) for the actual limits and model coupling.

## Selecting the PID variant

The JSON import mechanism can select the PID pipeline with:

```json
"imports": [
  { "from": "sim_ag_PID", "import": ["run_simulation"] }
]
```

This is a fragment of the configuration, not a complete input file. Keep the other required scenario parameters. The PID variant changes the active control call; this manual does not claim a runtime-validated comparison between the two algorithms.

## Traffic and older scenarios

The collision pipeline reads vessel traffic and navigation-map CSV assets. The ASKO configuration selects the older FTC implementation. Review [configuration and asset contracts](openwiki/workflows/scenarios-results.md) and [known interface gaps](openwiki/testing/validation.md) before attempting to use or extend these paths.
