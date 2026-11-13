# Simulation scenarios

Choose a scenario by its purpose and interface compatibility. A JSON file selects a pipeline through its `imports` entry; `main.py` selects which JSON file to read. The default scenario uses model predictive control (MPC) in a synthetic channel.

| Configuration or variant | Implementation | Current status |
| --- | --- | --- |
| `input/SimParam_anti_grounding.json` | `sim_ag_upg` | Default synthetic channel scenario using anti-grounding MPC |
| PID variant | `sim_ag_PID` | Active PID rudder control; select by changing the JSON import |
| `input/SimParam_WC.json` | `sim_colav` | Collision-avoidance scenario; guidance and drawing interfaces need reconciliation |
| `input/SimParam_ASKO.json` | `sim_ftc` | Older FTC scenario; four-argument entry point does not match current startup |
| `input/SimParam.json` | No import configured | Does not supply the `run_simulation` callable required by startup |

## Default anti-grounding scenario

Use this scenario to inspect the connection between route following, propeller/rudder optimization and shallow-water dynamics. It constructs an `Own` vessel and a sinusoidal channel route in Python.

Line-of-sight (LOS) guidance supplies heading references to MPC. The controller includes a soft under-keel clearance constraint: limited violation is allowed with a penalty. See [Guidance and anti-grounding control](openwiki/concepts/guidance-control.md) for limits and model coupling, and [Connecting the functions](pipelines.md) for the call sequence.

## Selecting the PID variant

The proportional–integral–derivative (PID) variant uses the same standard LOS function and a PID rudder command. To select it, change the `imports` entry in your anti-grounding input file to the following fragment:

```json
"imports": [
  { "from": "sim_ag_PID", "import": ["run_simulation"] }
]
```

Keep the other required scenario parameters and point `main.py` at the edited input file. This fragment is not a complete JSON document. The PID variant leaves the initialized propeller command in place; it does not run the anti-grounding MPC optimization. The [PID pipeline example](pipelines.md#example-2-substitute-the-existing-pid-controller) shows that difference.

## Traffic and older scenarios

The collision-avoidance pipeline reads vessel traffic and navigation-map CSV files, then attempts to combine encounter reasoning with route tracking. Its guidance and drawing calls use older interfaces. The ASKO input selects the older fault-tolerant control (FTC) pipeline, whose entry-point argument count also differs from current startup.

Use these paths as implementation material when extending the simulator. Review [configuration and asset contracts](openwiki/workflows/scenarios-results.md) and [known interface gaps](openwiki/testing/validation.md) before expecting a complete run. This documentation review did not execute a comparison of the scenarios.
