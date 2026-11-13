---
type: concept
title: Guidance and anti-grounding control
description: LOS waypoint tracking, encounter roles, controller state contracts and depth constraints.
tags: [guidance, mpc, grounding, channel]
verified:
  - by: openwiki/0.9.0
    at: 2026-11-13T18:28:11.946Z
sources:
  - id: openwiki-source-e9adf5f794c55aed12d6a76e
    resource: repo://src/modules/classes.py
  - id: openwiki-source-5044576d448aff1321f8847c
    resource: repo://src/modules/helper_utils.py
  - id: openwiki-source-fee98f614b96687de6862995
    resource: repo://src/modules/intel_guidance.py
  - id: openwiki-source-9f7804a99cffc12309ad827f
    resource: repo://src/modules/traffic_planner.py
generated: { by: "codex", at: "2026-11-13T18:28:11.946Z" }
---

# Guidance and anti-grounding control

Guidance produces a heading reference; controllers produce actuator commands; vessel dynamics advance physical state. The default anti-grounding scenario calls LOS directly with no avoidance-route offsets. Encounter reasoning is a separate path used by the collision-avoidance pipeline.

## LOS and encounter reasoning

`compute_LOS_ref` reads the latest measured ontology position and uses either the vessel route or route plus offsets. It skips degenerate segments and advances the waypoint index when the position projects beyond a segment. Cross-track error produces a heading correction with lookahead `5 * L`. Wrapped heading changes are limited to `(pi / 90) * vessel.timestep` per call. The vessel default timestep is 1; changing pipeline `Time step` does not automatically update that member.

`Guidance_Reasoner` computes CPA geometry for nearby vessels, records active regulation histories, classifies encounters and assigns Give-way/Stand-On roles. It switches between COLREGS and BPR branches around scaled x=600. Give-way behavior generates route offsets; Stand-On keeps zero offsets. This is explicit Python logic over ontology individuals, not a call to a generic OWL reasoner. Its six-value return contract includes offsets, speed, safe distance, scenario, nearby vessels and closest map segment.

The standalone global planner builds a visibility graph between anchors and safety vertices, then returns the weighted shortest path. If NetworkX reports no path it prints a message and returns an empty list, rather than the usual pair of coordinate lists. Its waypoint-spacing argument currently does not activate resampling.

## Actuator and optimization contracts

PID uses measured heading, yaw rate and accumulated error, computes gains from vessel parameters, and returns rudder angle clipped to ±35 degrees in radians. The implementation does not read the `PID Gains` JSON object.

Anti-grounding MPC constructs a continuous do-mpc/CasADi model with state `[x, y, psi, u, v, r]` and algebraic sinkage. Its inputs are propeller speed `Np` followed by rudder `delta`. `get_next_ag_step` repacks the seven-element physical state and calls `make_step`; the pipeline appends those two output components to the corresponding vessel histories.

The model is rebuilt on each default simulation step. It uses the MUMPS linear solver through IPOPT, holds the heading reference constant over the horizon, rewards speed while penalizing heading error and actuator changes, bounds propeller speed to 50/60–600/60 and rudder to ±35 degrees, and constrains speed to 5 knots. Clearance uses a soft constraint with a 0.05 m minimum plus 0.15 m buffer and at most 0.15 m permitted violation.

The vessel wrapper currently replaces caller-supplied weights with its own default array. Editing the weight vector in the pipeline therefore does not tune the controller through this wrapper. This is a source-level contract gap, not a validated tuning interface.

## Depth, geometry and coupling

`get_channel_depth` accepts screen coordinates, uses a sinusoidal centerline and perpendicular offset, and returns zero outside longitudinal bounds or supported channel depth regions. Its active bottom depth falls from 3.3 to 3.1 m over x=0–1200, with piecewise side slopes and two-decimal output. The declared `min_depth` and `max_depth` arguments do not drive that active formula.

Depth prediction samples straight-line future positions using current speed and measured heading, and takes their minimum depth for the clearance constraint. The numeric dynamics, history logger and controller do not share one scale conversion: `rhs` uses 0.2, history UKC uses 1, and prediction uses `sim_scale`. Review these sites together when changing geometry. The rendered channel also has its own width constant; visual banks are not sufficient evidence that every depth call uses identical geometry.

Bank-ray measurement stores starboard distance positive and port distance negative, with positive/negative infinity when intersections are absent. Physics and encounter rules consume these values, so preserve the sign convention when replacing bank detection.

See [state lifecycle](../architecture/simulation-state.md), [scenario workflow](../workflows/scenarios-results.md), [validation](../testing/validation.md), and [quickstart](../quickstart.md).
