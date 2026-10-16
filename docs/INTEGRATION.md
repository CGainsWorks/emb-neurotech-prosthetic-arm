# Integration with ml-semg-gesture-classifier

The classifier lives in
[`ml-semg-gesture-classifier`](https://github.com/CGainsWorks/ml-semg-gesture-classifier)
and this repository is everything on either side of it: the board that would
produce the signal and the arm that acts on the decision.

This document is the contract between them and the three places it does not
currently hold.

## The interface

The classifier produces one decision every 50 ms from a 200 ms window. This
repository consumes it as `arm_msgs/GestureCommand`:

```
uint8   gesture_id            Ninapro DB5 exercise B, 0 to 17
string  gesture_name
float32 confidence            softmax probability of the reported class
bool    valid                 false when no decision could be made at all
float32 inference_latency_ms  end of window to publication
```

`valid` is not the same as low confidence. A lifted electrode produces an
invalid decision and the mapper treats the two differently: a low-confidence
decision is ignored and an invalid one abandons the candidate it was
accumulating.

What the arm does with each class is in
[`arm_control/src/gesture_map.cpp`](../ros2_ws/src/arm_control/src/gesture_map.cpp)
and the gating that stands between the decision and the actuators is in
[`CONTROL_LOOP_TIMING.md`](CONTROL_LOOP_TIMING.md).

## The gesture table

Ninapro DB5 exercise B has seventeen movements plus rest. This mechanism has
six joints and a rigid terminal device, so it can perform some of them and not
others. Both cases are in the table and the ones it cannot perform are marked
rather than quietly mapped onto something else.

| Class | Movement | This arm |
|---|---|---|
| 0 | rest | returns to the resting pose |
| 1–8 | finger configurations: thumb up, pointing, fist, spread and so on | **reported, moves nothing.** There are no fingers |
| 9 | wrist supination (middle finger axis) | forearm rotation +1.2 rad |
| 10 | wrist pronation (middle finger axis) | forearm rotation −1.2 rad |
| 11 | wrist supination (little finger axis) | forearm rotation +1.6 rad, tool roll +0.4 |
| 12 | wrist pronation (little finger axis) | forearm rotation −1.6 rad, tool roll −0.4 |
| 13 | wrist flexion | wrist flexion +1.1 rad |
| 14 | wrist extension | wrist flexion −0.9 rad |
| 15 | wrist radial deviation | **reported, moves nothing** |
| 16 | wrist ulnar deviation | **reported, moves nothing** |
| 17 | wrist extension with closed hand | wrist flexion −0.9 rad |

Nine of the eighteen classes move nothing. That is a property of the
mechanism, not of the mapping:

- The finger configurations need fingers. The terminal device is modelled as a
  rigid payload and its own degrees of freedom are outside the six-axis chain.
- Radial and ulnar deviation are what the concurrent wrist cost. The three
  wrist axes were made to intersect so that the inverse kinematics has a
  closed form ([`dh_parameters.md`](dh_parameters.md) has the derivation) and
  deviation is the axis that went.

They are still published, with `moves_arm` false, so a terminal device can act
on them without the arm pretending to.

## Three places the contract does not hold

These are real and they are not worked around. Each one is a reason the two
repositories cannot simply be wired together today.

### 1. Eight channels against sixteen

Ninapro DB5 is recorded with two Thalmic Myo armbands, eight dry electrodes
each, so sixteen channels at 200 Hz. This board has **eight** differential
channels.

A model trained on sixteen channels cannot consume eight. There is no
adapter that fixes this: the convolution in the classifier runs across the
electrode axis and halving it changes the input shape and the receptive
field.

What has to happen is a retrain on an eight-channel subset of DB5. The natural
subset is one armband: eight electrodes evenly spaced around the forearm,
which is close to this board's layout. That costs accuracy and how much is an
experiment nobody has run.

### 2. The 200 Hz output rate aliases

DB5 is sampled at 200 Hz, so the classifier wants 200 Hz, so the board
decimates its 2 kHz acquisition by ten to produce it.

Nyquist is then 100 Hz and the band the classifier reads reaches 99 Hz.
There is no transition band left. Everything between 101 and 199 Hz folds
into the band and no filter can prevent that while keeping a passband that
reaches 99 Hz.

What the design does instead is bound it. A sixth-order Butterworth at 80 Hz
keeps the response within a third of a decibel to 65 Hz, puts everything above
160 Hz more than 37 dB down and leaves the 100 to 160 Hz region attenuated by
12 to 37 dB. That residual is the compromise, it is stated rather than hidden
and `the_anti_alias_filter_does_what_the_decimator_needs_and_no_more` in
[`firmware/test/test_dsp.cpp`](../firmware/test/test_dsp.cpp) asserts every
figure in that sentence.

It also means this board's 200 Hz stream is 11 dB down at 99 Hz. The Myo
armbands DB5 was recorded with have their own front end and filtering ahead
of their 200 Hz output, which is not the same response. That is a domain
shift and a model trained on one and fed the other will lose accuracy by an
amount nobody has measured.

The clean fix is to retrain the classifier at the acquisition rate rather than
decimate to meet it. 2 kHz needs no anti-alias compromise at all and the band
is wider; the cost is retraining on something other than DB5, because DB5 does
not exist at 2 kHz.

How much folded energy DB5 itself contains depends on the anti-alias
filtering inside the Myo ahead of its 200 Hz output, which this repository
does not model and makes no assumption about. Matching it would not be an
argument for doing it here either.

### 3. No recorded signal to test against

Every sEMG signal in this repository is synthetic. The simulated board
generates band-limited noise whose amplitude follows the active gesture
through a per-gesture electrode pattern, which is enough to prove the data
path carries frames and nothing at all about whether a classifier could read
them.

Personally recorded sEMG is personal data and is not committed here. That is a
deliberate constraint, not an oversight and it is why the classifier
repository uses a public dataset.

## What would connect them

In order and none of it has been done:

1. Fabricate a board and measure its noise floor
   ([`BRING_UP.md`](../firmware/docs/BRING_UP.md)).
2. Retrain the classifier on an eight-channel subset of DB5 and publish what
   that costs against the sixteen-channel figure.
3. Decide the rate. Either accept the 200 Hz path with its documented aliasing
   and measure the accuracy loss, or retrain at 2 kHz on new data and drop the
   decimation entirely.
4. Export the retrained model and run it on the host, against the board's
   frames, with the latency measured rather than estimated.
5. Only then wire the classifier's output into `gesture/command` and see what
   the arm does.

## What already connects

The parts that do not depend on any of the above are real and are tested:

- The message contract exists and the arm acts on it, with the confidence
  gating, the dwell and the staleness handling that make a noisy classifier
  usable on a limb.
- The interface is exercised from the sEMG side as well.
  `arm_bringup/classifier_stub` consumes `semg/frame` and publishes
  `gesture/command` at the classifier's rate from its 200 ms window.
  [`test_end_to_end.launch.py`](../ros2_ws/src/arm_bringup/test/test_end_to_end.launch.py)
  drives it with a synthetic sEMG replay and checks the arm follows. The stub
  matches electrode patterns that the synthetic signal was built from, so it
  is right by construction: it tests the wiring, not a classifier.
- The STM32H743 is the part the classifier repository recommends. Its README
  ("On target hardware") puts the deployed int8 network at 83 ms by
  arithmetic and 158 ms under Renode on a 100 MHz Cortex-M4F (STM32F411),
  which fits that part's flash and RAM but misses the 300 ms end-to-end
  budget and cannot keep up with the 50 ms stride. On this part the
  arithmetic estimate is 12.6 ms, inside both. Neither figure is a measurement
  on silicon and only the Cortex-M4F one has been run, on an emulator.
- The end-to-end latency budget in
  [`CONTROL_LOOP_TIMING.md`](CONTROL_LOOP_TIMING.md) is built on the
  classifier's published figures for the window, the inference and the
  decision stride and adds this repository's own measured contributions to
  them.
- The simulated run in
  [`analysis/sim/control_loop_sim.json`](../analysis/sim/control_loop_sim.json)
  drives the arm with a scripted decision stream at the classifier's real rate,
  with one decision in seven deliberately wrong at high confidence, which is
  roughly its real error rate. The gating absorbs them.
