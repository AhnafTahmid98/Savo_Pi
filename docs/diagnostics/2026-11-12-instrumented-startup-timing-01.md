# Instrumented autonomous-mapping failure — capture 01

**2026-11-12 · read-only analysis · master `ae21661`**  
Bundle: `~/savo-diagnostics/startup_timing_instrumented_01/`. No source, configuration, safety policy, EKF frequency or Git-history changes.

## Conclusion

**This was a localization-consumer observation stall, not a 678 ms interruption in IMU sampling or producer-health publication.** Fresh IMU reports reached both the recorder and coordinator while localization health kept an older report. Wheel-health and filtered-odometry callbacks also stopped advancing across the same two evaluations. Only IMU crossed its 600 ms freshness limit because its cached callback was already older.

The strongest remaining hypothesis is **delay in the localization process's DDS/RMW receiving path**, potentially from thread scheduling, locking or transport/reader contention. The measurements do not identify which. Expensive health evaluation or IMU callback processing is not supported as the principal cause. The coordinator then demonstrably confirmed **one cached STALE report twice**, and correctly retained its existing terminal failure after recovery.

## Clock and evidence interpretation

Decoded **13,249 messages / 10 topics** with Jazzy and extracted MCAP `publishTime`/`logTime` with `libmcap`: publisher/source versus recorder receive/record times, not disk-write completion. Raw IMU headers provide measurement timestamps.

STALE evaluation **188** started at **1794442046.858539**, before its **1794442046.886555588** warning. Recovered evaluation **189** started at **1794442047.378443**, before its **1794442047.378716177** OK log. Freshness ages refer to evaluation starts.

For the tables, `t` is epoch seconds minus **1794442000**. `≈` denotes conversion of monotonic time using the median paired health-clock offset, **1794429056001566583 ns**; observed offsets span only 1.143 µs in this window. Monotonic event order is definitive; converted epoch values are approximate.

## Event reconstruction

| t (s) | Event and evidence |
| ---: | --- |
| 46.159835605 | IMU report **publish_count 1726** published; bag at **46.160287202**. This is the report later used by stale localization. |
| ≈46.180004407 | Localization IMU callback **389** runs; DDS receive timestamp **46.159997458**. |
| 46.358494 | Localization evaluation **187**, OK. Callback counts IMU/wheel/filtered = **389/435/1668**. |
| 46.414423347 | New IMU report **1730** published; bag **46.430332974**. Coordinator DDS receives it at **46.414522756**, processing it by ≈**46.425191119**. |
| 46.626876095 | New IMU report **1734** published; bag **46.627110153**. Coordinator DDS receives it at **46.631527564**, processing it by ≈**46.670240756**. |
| 46.858539 | Evaluation **188**, STALE: `imu_stale:IMU healthy`; age **678.529 ms**, rate **14.658297 Hz**. Callback counts remain **389/435/1668**. |
| 46.858762946 | STALE health published; bag **46.886005064**; coordinator DDS receives it at **46.886399327**. |
| ≈46.896715150 | Coordinator records localization receipt **125**, event **12824**. |
| ≈46.922411106 | Coordinator evaluation **317**, event **12825**: receipt 125, count **1/2**, blocked but not failed. |
| ≈47.104261131 | Evaluation **318**, event **12858**: same receipt 125, count **2/2**. Failed decision exists by ≈**47.107747131**. Its trace reaches the bag at **47.205493916**. |
| ≈47.328542561 | Evaluation **319**, event **12898**: still receipt 125 and failed. Its bag timestamp **47.426664928** is later than recovery publication, illustrating why bag ordering alone is insufficient. |
| 47.378443 | Evaluation **189**, OK, using IMU report **1738**. Health published **47.378613732**; coordinator DDS receives it **47.383411646**. |
| ≈47.452276076 | Coordinator records localization receipt **126**, event **12913**. |
| ≈47.545823983 | Evaluation **320**, event **12954**: `dependencies_recovered=true`, count **0/2**, `decision_failed=true`, `first_failure_evaluation=318`. |

`localization_rx` is a coordinator receive counter, distinct from localization evaluation sequence and IMU producer `publish_count`. Evaluations 317→318 are **181.850 ms** apart; recovery was **published after confirmation**. Evaluations 317–320 all have `recovery_eligible=false`: this was freshness loss, not isolated low EKF rate.

## IMU reports and continued sampling

There is no matching producer-health publication gap of 678 ms. During **46.180004–46.858539**, the bag received **nine new raw IMU messages**. Their maximum consecutive header gap was **108.665 ms**; the latest header was **46.735249674**, only **123.289 ms** old at the stale evaluation. During evaluations 187→188 alone, the recorder received **six raw IMUs and two newer IMU-health reports**.

Relevant producer payloads:

| Role | publish_count | Source t | Bag t | Producer monotonic publication ns | Self-reported health gap (ms) | Rate (Hz) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Cached report used by STALE | 1726 | 46.159835605 | 46.160287202 | 12990158192565 | 223.906185 | 14.658297 |
| Newer, not yet processed at STALE | 1730 | 46.414423347 | 46.430332974 | 12990412787363 | 254.594798 | 15.153332 |
| **Last recorded report before STALE** | **1734** | **46.626876095** | **46.627110153** | **12990625151425** | **212.364062** | **16.787097** |
| Report used by recovered evaluation 189 | 1738 | 47.287318697 | 47.287711275 | 12991285665806 | 389.873270 | 13.472918 |
| **First recorded report after localization recovery** | **1740** | **47.493931744** | **47.494028282** | **12991492288686** | **206.622880** | **13.313284** |

All five have `health_state=OK`, `reason="IMU healthy"`, `hardware_ok/data_valid/frame_valid/timestamp_valid/motion_ready=true`, `error_count=0`, `last_success_age_s=0`, and equal sample/publish counts. The two literal before/after payloads are preserved below. Report **1738**, which enabled recovery, was already published **before** the recovered aggregate health report; **1740** is the first producer report chronologically after it.

## Cross-topic comparison, 1794442044–1794442049

Maximum gaps use adjacent recorded messages within the window; the TF row includes only `odom→base_footprint`.

| Stream | Count | Maximum recorder gap (ms) | Maximum producer self-reported health gap (ms) | New records between evals 187→188 |
| --- | ---: | ---: | ---: | ---: |
| Raw IMU | 72 | 231.457 | — | 6 |
| IMU health | 21 | 390.160 | 389.873 | 2 |
| Wheel health | 21 | 300.595 | 300.522 | 2 |
| Filtered odometry | 64 | 251.531 | — | 8 |
| Odom→base TF | 64 | 251.376 | — | 8 |

At evaluation 188, health's cached ages were **678.529/526.128/503.143 ms** for IMU/wheel/filtered and **526.569 ms** for TF. In contrast, the latest recorder arrivals were only **231.429/81.183/13.560 ms** old; the latest recorded odom→base TF header was **88.555 ms** old. The health TF buffer did advance between evaluations, but lagged the independently recorded TF. Its listener has a separate thread, so a main-executor-only explanation is incomplete.

The producer's lifetime IMU `max_health_publish_gap_s=0.626616702` predates this failure and remains unchanged through the window. It must not be mistaken for this incident's callback gap.

## DDS, callbacks and publication timing

- Between evaluations **187 and 188**, all three callback counts and last DDS source/receive timestamps are identical, despite a normal **500.045 ms** evaluation interval. At recovery they advance to **393/439/1675**. IMU's completed maximum callback gap becomes **714.812 ms**. Before that callback completed, its recorded maximum still showed the older **622.641 ms**; maxima do not include an unfinished gap.
- For cached IMU report 1726: source→DDS receive **0.162 ms**, DDS receive→callback **20.007 ms**. For report 1738 used after recovery: **8.374 ms** and **10.163 ms**. Neither describes the missing intermediate deliveries.
- Across the stall/recovery, maximum source→callback delay grows to **480.393 ms** (IMU), **353.266 ms** (wheel) and **379.883 ms** (filtered). Their lifetime maximum receive→callback delays remain **184.688/56.970/57.177 ms**. Therefore, for the respective worst source-delay samples, **at least 295.705/296.296/322.706 ms occurred before the instrumented DDS receive point**. These are bounds, not paired per-message timings. They favor delayed receiving/delivery over solely slow callbacks after DDS receipt; interpretation still depends on the deployed RMW's receive-timestamp semantics.
- Callback work maxima by recovery are only **5.032/4.471/6.686 ms**; IMU callback CPU maximum is **0.136 ms**. Parsing/sensor-health computation cannot account for the observed half-second lack of callback progress.

Completed evaluation timings appear in the **next** health report:

| Evaluation | Start t | Total wall (ms) | Publication wall / thread CPU (ms) |
| ---: | ---: | ---: | ---: |
| 180, lifetime publication peak | 42.858505 | 239.674 | **239.398 / 0.319** |
| 186 | 45.867744 | 42.285 | 42.058 / 0.309 |
| 187, immediately before STALE | 46.358494 | 16.892 | **16.698 / 0.189** |
| 188, STALE | 46.858539 | 28.100 | **27.784 / 0.219** |
| 189, recovered | 47.378443 | 0.344 | 0.102 / 0.100 |

The **239.4 ms maximum was earlier**, not the immediately preceding evaluation. It shows blocking/descheduling inside the publication scope, not heavy CPU computation. That scope includes health, summary and diagnostics publishes, so it cannot identify an individual blocking call. The uninstrumented heartbeat callback, executor wait/take path and DDS internal threads remain possible sites. Low thread CPU alone cannot distinguish locks, I/O waits and runnable scheduling delay.

## Recorder overhead and ranked hypotheses

The MCAP is **11,077,683 bytes**, containing **10,412,628 serialized payload bytes over 139.235 s**. The focus window contains **526 messages / 552,788 payload bytes**: approximately **105 messages/s and 111 kB/s**. All **14 MCAP chunks are uncompressed**. The failure lies within the chunk spanning **2040.862611–2048.542707**, not at its message-time boundary; that boundary is not a disk-flush timestamp.

Recording adds subscribers, DDS fan-out, callbacks, allocation and storage activity. Reliable subscriptions can potentially add middleware pressure, depending on actual recorder QoS and RMW settings. Metadata lists publisher-offered QoS, not a complete recorder configuration. Modest bandwidth and no compression weaken a simple bandwidth/compression-overload explanation but **do not rule out recorder-induced contention or backpressure**. There is no recorder CPU/I/O trace or matched recorder-off run. At evaluation 180, source→bag delay (**239.296 ms**) closely matches publication wall time; that correlation establishes shared delay, not which process caused it.

| Confidence | Hypothesis / conclusion |
| --- | --- |
| **High: observed** | Localization's consumer view stalled while other subscribers received fresh data; cached double-confirmation caused terminal startup failure. |
| **Medium-high: mechanism class** | Localization-specific DDS/RMW receiving/participant contention or delayed receive-thread scheduling. Multi-topic stalls and ≥296 ms pre-receive bounds support this. |
| **Medium: possible contributor** | CPU scheduling pressure, blocking middleware operations or shared locks. Low CPU/wall ratios support waiting/descheduling, but no scheduler evidence identifies runnable versus blocked time. |
| **Low as sole cause** | Slow application callback/evaluation computation or main-executor saturation: measured work is short, timers continue, and TF also lags. An unmeasured blocking callback is not excluded. |
| **Low for this incident** | IMU sensor sampling or producer-health publication stopping for 600+ ms: directly contradicted by fresh raw messages, new healthy payloads and coordinator receipts. |
| **Unresolved** | Recorder overhead as a trigger/amplifier: plausible, not demonstrated by one recorded run. |

## Smallest justified next experiment

**No functional fix is justified yet.** Run a matched pair of instrumented startups with the recorder enabled/disabled, retaining identical EKF, freshness, QoS, executor, safety and arming settings. In both runs capture a short scheduler window around mapping-runtime startup, covering localization **main, TF and DDS threads**, coordinator, IMU and recorder—not only EKF. Record thread names/TIDs, `sched_switch` and wakeups, effective RMW/configuration, CPU pressure and recorder CPU/I/O. Use the existing transition logs for the recorder-off outcome; avoid extra high-rate JSON logging. Repeat the pair if results differ, since one pair cannot establish causality.

Runnability without CPU time would support scheduler contention; sleeping/blocking with late DDS receive would support middleware/locking waits. A repeatable recorder-on-only regression would justify investigating recorder QoS/storage or publication behavior before changing autonomy logic. If scheduler evidence remains ambiguous, the smallest subsequent **diagnostic-only** code change is separate timings for each publish call and heartbeat, plus bounded capture of the source/receive/callback timestamp tuple for the worst delay. Do not yet switch executors, relax freshness, alter EKF rate or add recovery.

## Integrity and verification

All messages decoded successfully and per-topic counts matched `metadata.yaml`. C++ MCAP source timestamps were joined to decoded messages by exact topic/log timestamp; the cached and recovery IMU source timestamps match the health node's DDS metadata. Original files were mounted read-only in offline, disposable Jazzy containers; no bag playback or robot access occurred. Temporary calculations are under `/tmp/savo-instrumented-analysis/`. This report is the only repository addition from this investigation; pre-existing untracked PC-validation logs were left alone.

MCAP SHA-256: `ef5f63bfdb35b64b23363628c57db58156c26d3138649c8c56854e3b01ea2a08`. Bundle hashes were checked before and after analysis. No source build or tests were run for this read-only analysis.

## Exact boundary payloads

<details>
<summary>Last IMU-health report recorded before STALE: publish_count 1734</summary>

Source **1794442046.626876095**, recorder **1794442046.627110153**. Original `std_msgs/String.data`:

```json
{"calibration_accel":3,"calibration_gyro":3,"calibration_mag":0,"calibration_system":0,"child_frame_id":"","chip_id":160,"data_valid":true,"error_count":0,"frame_id":"imu_link","frame_valid":true,"hardware_ok":true,"health_publish_gap_s":0.212364062,"health_publish_monotonic_ns":12990625151425,"health_state":"OK","illegal_transition_count":0,"isolated_gap_excluded":false,"last_success_age_s":0.0,"last_success_monotonic_ns":12990625151425,"max_health_publish_gap_s":0.626616702,"max_inter_publication_gap_s":0.125099009,"motion_ready":true,"node":"imu_node","parent_frame_id":"","producer_rate_available":true,"producer_rate_hz":16.787096787690448,"publish_count":1734,"rate_quality":"GOOD","rate_window_sample_count":30,"raw_window_rate_hz":16.787096787690448,"reason":"IMU healthy","sample_count":1734,"schema_version":1,"system_error":0,"system_status":5,"timestamp_valid":true}
```

</details>

<details>
<summary>First IMU-health report recorded after localization recovery: publish_count 1740</summary>

Source **1794442047.493931744**, recorder **1794442047.494028282**. Original `std_msgs/String.data`:

```json
{"calibration_accel":3,"calibration_gyro":3,"calibration_mag":0,"calibration_system":0,"child_frame_id":"","chip_id":160,"data_valid":true,"error_count":0,"frame_id":"imu_link","frame_valid":true,"hardware_ok":true,"health_publish_gap_s":0.20662288,"health_publish_monotonic_ns":12991492288686,"health_state":"OK","illegal_transition_count":0,"isolated_gap_excluded":false,"last_success_age_s":0.0,"last_success_monotonic_ns":12991492288686,"max_health_publish_gap_s":0.626616702,"max_inter_publication_gap_s":0.2311259,"motion_ready":true,"node":"imu_node","parent_frame_id":"","producer_rate_available":true,"producer_rate_hz":13.313283913385929,"publish_count":1740,"rate_quality":"MINIMUM","rate_window_sample_count":30,"raw_window_rate_hz":13.313283913385929,"reason":"IMU healthy","sample_count":1740,"schema_version":1,"system_error":0,"system_status":5,"timestamp_valid":true}
```

</details>
