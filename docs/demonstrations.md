# Recorded simulations

Use these recordings to relate the [pipeline explanations](pipelines.md) to visible vessel motion. Each player shows an existing repository video. Select Play to load it, or use the link below the player to open the MP4 directly.

The recordings are grouped by filename. Their exact inputs, gains and source revisions are not recorded here, so use them as qualitative examples rather than a controlled comparison or validation of current source.

## Navigation recordings

### PID navigation

Watch how the vessel follows its route while heading control adjusts the rudder. PID stands for proportional–integral–derivative control.

<video controls preload="none" playsinline poster="../assets/video-posters/PID_navigation_video.png" aria-label="PID navigation recording">
  <source src="../assets/videos/PID_navigation_video.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the PID video](assets/videos/PID_navigation_video.mp4)

Read the [PID controller explanation](topics/control.md#pid-background-and-implementation) for the feedback law, then follow the [PID pipeline example](pipelines.md#example-2-substitute-the-existing-pid-controller) to see where commands are applied.

### NMPC navigation

NMPC denotes nonlinear model predictive control. Use this clip alongside the predictive-control discussion to examine route-following behavior.

<video controls preload="none" playsinline poster="../assets/video-posters/NMPC_navigation_video.png" aria-label="NMPC navigation recording">
  <source src="../assets/videos/NMPC_navigation_video.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the NMPC video](assets/videos/NMPC_navigation_video.mp4)

See [predictive control](topics/control.md#predictive-control-background-and-implementation) for state and command conventions, and the [anti-grounding loop](pipelines.md#example-1-the-existing-anti-grounding-loop) for the current call sequence. The filenames alone do not establish matching experimental conditions between this clip and the PID recording.

## Encounter recordings

### Multi-environment crossing

This recording provides visual context for a crossing encounter. Read [encounter guidance](topics/guidance.md#rule-aware-path-offsets) to connect roles and reference-path changes.

<video controls preload="none" playsinline poster="../assets/video-posters/multi_environment_crossing.png" aria-label="Multi-environment crossing encounter recording">
  <source src="../assets/videos/multi_environment_crossing.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the crossing video](assets/videos/multi_environment_crossing.mp4)

### Multi-environment head-on

Use this recording with the [semantic role-assignment explanation](topics/semantics.md#regulation-activation-and-geometric-decisions) to inspect a head-on encounter.

<video controls preload="none" playsinline poster="../assets/video-posters/multi_environment_head_on.png" aria-label="Multi-environment head-on encounter recording">
  <source src="../assets/videos/multi_environment_head_on.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the head-on video](assets/videos/multi_environment_head_on.mp4)

The [collision-free planning publication](publications.md#rule-aware-collision-free-planning) discusses head-on and crossing encounters. The recordings do not carry enough metadata to identify them as particular published experiments. For the software interfaces behind encounter-aware navigation, return to [encounter guidance](topics/guidance.md#rule-aware-path-offsets) and [semantic role assignment](topics/semantics.md#regulation-activation-and-geometric-decisions).
