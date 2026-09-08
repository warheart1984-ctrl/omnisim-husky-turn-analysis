# OmniSim v8.3 four-Husky turn replay

This is a current replay of a +90 degree turn through the shipped HTTP bridge in
`omnilink_husky_swarm.omniworld`.

The bridge accepted 1.570796 rad, issued ten pulse/settle corrections, and the
settled pose moved only 0.162652 rad (9.319 degrees). The remaining error was
-1.408144 rad (-80.681 degrees). This does **not** reproduce the older 0.44 degree
mean-error measurement.

The replay also found a reporting defect: exhausting the correction limit was
returned as `settled: true`. The accompanying source fix makes this exit return
`settled: false` with `completion_reason: correction_limit`; its focused source
tests pass.

Environment: build `7d39130cf`, machine `9722d23d12a3`, Windows 11, RTX 3060
Laptop GPU, Newton/MuJoCo CPU solver. The engine log and exact replay client are
included. The test world was not edited.

