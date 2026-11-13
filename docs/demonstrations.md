# Recorded simulations

These videos are existing repository recordings, grouped by their filenames. They provide visual examples alongside the [pipeline explanations](pipelines.md); they are not newly generated runs or regression tests of the current source revision. The original MP4 files are served with the site and loaded only when needed.

## Navigation recordings

### PID navigation

<video controls preload="none" playsinline poster="../assets/video-posters/PID_navigation_video.png" aria-label="PID navigation recording">
  <source src="../assets/videos/PID_navigation_video.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the PID video](assets/videos/PID_navigation_video.mp4)

Follow the [PID controller explanation](topics/control.md#pid-background-and-implementation) and [PID pipeline example](pipelines.md#example-2-substitute-the-existing-pid-controller) while examining heading and route tracking. The filename alone does not establish the exact gains, inputs or source revision of this recording.

### NMPC navigation

<video controls preload="none" playsinline poster="../assets/video-posters/NMPC_navigation_video.png" aria-label="NMPC navigation recording">
  <source src="../assets/videos/NMPC_navigation_video.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the NMPC video](assets/videos/NMPC_navigation_video.mp4)

See [predictive control](topics/control.md#predictive-control-background-and-implementation) and the [anti-grounding loop](pipelines.md#example-1-the-existing-anti-grounding-loop) for the present implementation's interfaces. The video is provided as a recorded demonstration, not a verified quantitative comparison against the PID clip.

## Encounter recordings

### Multi-environment crossing

<video controls preload="none" playsinline poster="../assets/video-posters/multi_environment_crossing.png" aria-label="Multi-environment crossing encounter recording">
  <source src="../assets/videos/multi_environment_crossing.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the crossing video](assets/videos/multi_environment_crossing.mp4)

### Multi-environment head-on

<video controls preload="none" playsinline poster="../assets/video-posters/multi_environment_head_on.png" aria-label="Multi-environment head-on encounter recording">
  <source src="../assets/videos/multi_environment_head_on.mp4" type="video/mp4">
  Your browser does not support embedded MP4 playback.
</video>

[Open or download the head-on video](assets/videos/multi_environment_head_on.mp4)

Read [encounter guidance](topics/guidance.md#rule-aware-path-offsets), [semantic role assignment](topics/semantics.md#regulation-activation-and-geometric-decisions), and the [collision-free planning publication](publications.md#rule-aware-collision-free-planning). The publication discusses head-on and crossing encounters; the recordings do not include metadata proving their exact relationship to a published experiment.
