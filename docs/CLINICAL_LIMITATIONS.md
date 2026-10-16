# Clinical limitations

**This is a research prototype. It is not a medical device, it has never been
worn by anyone and it must not be described as a prosthesis for clinical
use.**

That sentence is the whole document and the rest of it is the detail behind
each clause.

## It is not a medical device

Nothing in this repository has been developed under a medical device quality
management system. Specifically:

- There is no ISO 13485 quality system behind it and no design history file.
- There is no risk management file to ISO 14971. [`SAFETY.md`](SAFETY.md)
  records the hazards that were considered during design, which is not the
  same thing and is not a substitute for one.
- No biocompatibility assessment has been made of anything that would contact
  skin (ISO 10993). The electrodes are commodity parts used outside any
  qualified configuration.
- There is no electrical safety assessment to IEC 60601-1 and the arm is not
  isolated: the signal ground of the amplifier chain is the same ground the
  battery, the servos and any attached debugger share. A debug probe plugged
  into a laptop that is plugged into mains puts the wearer's skin and mains
  earth in the same circuit.
- No UKCA or CE marking applies, has been sought, or would be granted.

It must not be placed on a person. Nothing about the design is dangerous to
look at or to build on a bench; what is missing is everything that would make
it safe to attach to someone.

## It has never been worn, or built

As of the date in [STATUS.md](../STATUS.md):

- No board has been fabricated. The analogue front end, the servo driver and
  the power tree exist as a design and a set of calculations.
- No arm has been manufactured. The kinematic and inertial model is a design
  and the URDF is generated from it.
- No firmware has executed on silicon. It compiles for the STM32H743, passes
  its host-side tests and runs under Renode on an emulated STM32H743, which
  checks its logic against models of the peripherals and nothing about how
  it behaves on a real part attached to a person.
- No sEMG has been recorded by this hardware. Every sEMG signal anywhere in
  this repository is synthetic and is labelled as such where it appears.
- No user trial has taken place. There is no ethics approval and none has
  been sought, because there is nothing to approve.

Every performance figure in this repository is therefore either a calculation
or a simulation. The documents say which, on every number.

## The control scheme has known clinical shortcomings

Even if the hardware existed, the approach has limits worth stating plainly,
because a reader familiar with prosthetics will recognise them and a reader
who is not should not be misled.

**Classification accuracy is not good enough for unsupervised use.** The
gesture classifier this arm is designed around reports 82% accuracy on
Ninapro DB5 with a causal filter and 66% balanced accuracy, which means the
less common gestures are substantially worse than the headline. A false
activation rate of 7% at the raw decision level is a spurious movement every
fifteen seconds. The dwell and confidence gating in
[`arm_control`](../ros2_ws/src/arm_control) reduce that at the cost of
responsiveness and the residual is still not a number anybody should rely on
to hold a cup.

**It is trained on able-bodied subjects.** Ninapro DB5 records intact limbs
performing movements. Residual limb musculature after amputation is different
in volume, in innervation and in what a wearer can voluntarily recruit and
the literature is consistent that models trained on able-bodied data transfer
poorly. Any real use would need data from the wearer, which is why the
classifier repository has a calibration path and why this one does not pretend
otherwise.

**Electrode position shift is not handled.** Donning a socket places the
electrodes differently every time. Nothing here detects or compensates for
that and it is one of the two or three largest sources of real-world failure
in myoelectric control.

**There is no feedback to the wearer.** No force sensing, no proprioceptive
feedback, no indication of grip state other than looking at the hand. Users of
myoelectric prostheses consistently report this as a bigger problem than
classification accuracy.

**The mechanism is not a hand.** It has six joints and a rigid terminal
device. Nine of the eighteen gesture classes it can recognise are finger
configurations it cannot form; they are reported and deliberately not acted
on, which [`docs/INTEGRATION.md`](INTEGRATION.md) describes. Wrist radial and
ulnar deviation are also absent, because the three wrist axes were made
concurrent to obtain a closed-form inverse solution.

**The arm is heavier than a worn prosthesis should be.** The model totals
2.2 kg including the terminal device. A transhumeral prosthesis much above
that is not tolerated for long and the figure here does not include the
socket, the battery or the electronics.

## What it is for

It is a complete, honest engineering exercise: the kinematics, dynamics,
trajectory generation, control stack, wire protocol, firmware and analogue
design for a six-axis myoelectric arm, built so that every claim in it can be
checked by running something. It is useful as a basis for a device, as a
teaching artefact and as evidence of how the author works.

It is not useful as a prosthesis and no part of this repository should be
presented as one.

## If this ever became a device

The gap is not small and listing it is more honest than implying it is a
matter of tidying up:

1. A quality management system, a design history file and a risk management
   file, before any further design work rather than after.
2. Electrical isolation of the patient-connected part and an assessment to
   IEC 60601-1 with the arm treated as a type BF applied part.
3. Biocompatibility assessment of the electrode interface and anything else
   that contacts skin.
4. Clinical evaluation, with ethics approval, beginning with able-bodied
   subjects wearing a bypass socket and only then with the intended users.
5. Software developed to IEC 62304, which among much else means the control
   stack would need a hazard analysis driving its architecture rather than the
   other way round.
6. A mechanical redesign for weight, for sealing and for the fact that a
   limb is dropped, knocked and leaned on.

None of that is work this repository has done.
