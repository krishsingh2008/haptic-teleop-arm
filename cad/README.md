# Haptic Teleop Hand: full CAD

A five-finger hand built from rev B fingers. The design comes from about 4,500 simulated hands, a finger battle run inside the full hand, and a palm-shape battle:

- **Fingers:** rev B with re-tuned knuckle elastics. Three identical fingers (42.7 / 28.1 / 23.7 mm) plus a shorter little finger (37.0 / 24.4 / 20.6 mm).
- **Palm:** U-shaped cup, with rounded 8 mm ridges along both sides and across the heel. A soft raised pad at the thumb base covers the sixth servo, the way the thenar muscle sits on a real palm.
- **Thumb:** two rev B joints (45.3 / 28.8 mm) on a carrier that a sixth servo swings 0–40° into opposition.
- **Drive:** 6 × MG90S. S1–S5 sit in the forearm and pull tendons; S6 sits in the palm and swings the thumb.

Simulated holds with MG90S servos, measured as the push the grip survives in its weakest direction:

| Object | Hold |
|---|---|
| Tennis ball | 5.0 N (about 9× its weight) |
| Baseball | 3.8 N |
| Soda can | 3.2 N |
| Ping-pong ball held clear of the palm | 2.6 N |

## Files

| File | What it is |
|---|---|
| `parts/*.stl` | Print these |
| `parts/*.step`, `hand_assembly.step` | Open or edit in Fusion 360 |
| `hand_cad.py` + `hand_params.json` | Parametric source. Change a number, then run `python hand_cad.py hand_params.json` |
| `check_fit.py` | Collision check of the moving parts (and S6) in several poses |
| `verify_cad.py` | Checks every tendon channel is open end to end, the thumb pivot, S6 clearances and wall thickness |
| `test_rig.py` | One-finger test stand (one finger, one MG90S, one spool). Writes `parts/finger_test_rig` and checks the fit |
| `finger_revB.py` | Finger part generator. Set `PIN_TIGHT`, `PIN_FREE` and `SLOT_CLR` there from the coupon |

## Print list

All parts: PLA, 0.2 mm layers, 4 walls, 30% infill, back face down, no supports. Print the rev B coupon first, set the three numbers in `finger_revB.py`, then regenerate.

| Part | Qty | Notes |
|---|---|---|
| `palm` | 1 | Ridges and thumb-base pad are printed in. S6 drops into the pocket from the back |
| `s6_cover` | 1 | Holds S6 in from the back, 2 × M2 × 8 flat-head screws |
| `finger_base` | 5 | 4 + 1 spare. Screws into the palm, 2 × M2 × 16 flat-head each |
| `finger_proximal` / `finger_middle` / `finger_distal` | 3 each | index, middle, ring |
| `little_proximal` / `little_middle` / `little_distal` | 1 each | |
| `thumb_carrier` | 1 | Hub screws to the S6 round horn |
| `thumb_proximal` / `thumb_distal` | 1 each | |
| `forearm_tray` | 1 | Holds S1–S5. The wrist guide wall lines each tendon up with its palm channel |
| `tendon_comb` | 1 | Print lying flat. Glue it into the post notches over the front servo row |
| `spool_32` | 4 | Finger spools. Sit on the MG90S round horn (horn screw from the top, through the counterbore) |
| `spool_20` | 1 | Thumb spool |

That's about 165 g of plastic for the hand and 85 g for the tray and spools.

## Hardware

- 6 × MG90S servos, with round horns
- About 45 pieces of 1.75 mm filament, cut to 17 mm, for hinge pins and anchor pins
- One M2 flat-head self-tapping screw kit covers every screw in the hand:
  - 8 × M2 × 16 for the finger bases. They go in from the back of the palm, countersunk flush, and thread about 9 mm into 1.6 mm pilot holes in each base.
  - 2 × M2 × 8 for the S6 cover.
  - 14 × M2 × 10 for the rev C knuckle tensioners.
  - 4 × M2 for the wrist flange, with the length set by the wrist part.
- A #0 Phillips screwdriver
- M2 screws for the servo tabs
- 30 lb braided fishing line, about 2.5 m total
- Round elastic cord in three thicknesses (see "Elastics")
- Optional: FSR 400 sensors for the fingertip pockets, plus thin ones on the middle segments

## Assembly

1. **Fingers.** Build each finger exactly as in the rev B README: hinge pins, then the tendon, then one elastic per knuckle.
2. **Bases.** Drop each finger base into its palm socket and bolt it from the back. The tendon from each finger runs through a channel inside the palm and comes out at the wrist end.
3. **Thumb.**
   - Thread the thumb tendon through the bore in the 4 mm wall at the wrist end, along the swing axis, and up the carrier arm. Because it runs down the axis, swinging the thumb doesn't change the tendon length.
   - Put the carrier's pivot boss into that bore.
   - Fit a single-arm horn to S6, pointing toward the back of the palm.
   - Drop S6 into its pocket from the back of the palm, shaft toward the wrist, so the horn arm slides into the slot in the carrier hub. Its tabs sit in their slot.
   - Screw on `s6_cover` (2 × M2 × 8).
   - Build the thumb phalanges onto the carrier's knuckle the same way as a finger.
4. **Forearm.** Screw S1–S5 into the tray frames:

   | Row | Servos |
   |---|---|
   | Front | S1 index, S2 middle |
   | Middle | S5 thumb, S3 ring |
   | Back | S4 little |

   Each finger tendon leaves the wrist end of the palm at 15 mm below the palm face, and the thumb tendon leaves on the swing axis. Run each one through its hole in the wrist guide wall. Then:
   - Front-row tendons (S1, S2) go straight to their spools.
   - Back-row tendons (S3, S4, S5) go over the comb first, so they clear the front spools.
   - Tie each tendon through its spool's tie hole.

## Elastics (new tuning)

The hand battle found that **base-first** closing grips balls best and keeps opening lag to about 5 mm. Base-first means the base knuckle bends first and the tip bends last. This reverses the stiffness order in the rev B README:

| Knuckle | Stiffness | Cord |
|---|---|---|
| Base | softest, 0.03 N/mm | thin (about 1 mm), long loop |
| Middle | medium, 0.2 N/mm | about 1.5 mm |
| Tip | stiffest, 0.6 N/mm | about 2.5 mm |

Preload each elastic to about 0.6 N, just taut. To check it, pull a tendon slowly: the base knuckle should start bending first and the tip last.

## Glove calibration

Each glove finger drives one servo. Record your open hand, your fist and a thumb-to-index pinch, then map those readings to the robot's servo positions:

| Pose | S1–S5 | S6 |
|---|---|---|
| Open | 0° | 0° |
| Fist | 150° | 0° |
| Pinch | S1 ≈ 59°, S5 ≈ 23° | ≈ 20° |

The thumb-rotation reading for S6 can come from a flex sensor across the web of your thumb.

## Finger rev C (screw-tuned elastics)

`finger_revC.py` builds the reprint, and `verify_revC.py` checks it. Every knuckle uses the same 1 mm cord: a knot in a keyhole at one end, and a loop over an M2 × 10 screw at the other, turned with a screwdriver from the back of the hand. The fishing line runs 5.5, 4.5 and 3.5 mm from the base, middle and tip hinges, so the base leads the closing. `elastic_battle.py` is the 10-design comparison that picked this layout. The parts are in `parts_revC/`. The full-hand files still use rev B fingers until rev C is proven on the test stand.

## Checked, and not checked

**Checked in CAD (`check_fit.py`, `verify_cad.py`):**
- No part collides with another, or with S6, in the open, half-closed, fist and thumb poses.
- The thumb carrier clears the palm and S6 through its full 0–40° swing.
- Every tendon channel is open from the finger base to the wrist end, at least 5.5 mm from the S6 pocket.
- The thumb tendon is open along the swing axis and up the arm.
- The pivot wall is solid all round its bore.
- Both S6 tabs are enclosed.
- S6 is fully covered on the palm side. No part of the servo shows through the palm face.
- Every part is a single solid, and the outer socket walls are at least 2 mm.
- In a fist, the little fingertip rests on the side ridge. That's expected: the simulator stops it there too.

An independent review of the first version found 8 blockers, and all are fixed:
- the index tendon ran under the S6 tab
- the thumb tendon had no path into the carrier
- the carrier overlapped the servo
- the pivot wall was too thin
- one S6 tab had nothing to screw into
- the base bolts had nothing to thread into
- the thumb spool printed as two pieces
- the outer sockets broke through the palm sides

**Known limits:**
- The walls between neighbouring finger sockets are thin, 1.0–1.5 mm. That's set by the 1.9 mm finger gap the simulator chose.
- The thumb-side ridge starts about 22 mm from the wrist end, and the heel ridge stops short of the thumb, so the thumb carrier can swing through the gap.

**Not checked:** wrist motion, real friction, how the tendons wear, and the servo tab-hole positions on your exact MG90S batch.

Before printing the whole hand, print the palm, one finger and the thumb carrier. The palm is the biggest print (about 149 cm³). Check S6 and its horn in the pocket and slot before printing the rest.
