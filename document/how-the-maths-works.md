# How the maths works (plain language)

The whole project is one idea, tried in a few flying games: each drone has a picture in its head of how the world should look. When the real world does not match that picture, the drone gets a **surprise score**. It then moves in a way that makes the surprise smaller.

The drones are tiny Crazyflie quadcopters. The same maths runs in a simulator and on the real drones.

This note is a student-level walkthrough of the experiments in [EXPERIMENTS.md](../EXPERIMENTS.md). The numbers below match the YAML configs. If a config file and this note ever disagree, the YAML wins.

---

## The one formula

Almost every experiment uses the same surprise score:

$$
S = \frac{1}{2}\left(\frac{\text{error}}{\sigma}\right)^2
$$

Read it like this.

- **error** is how wrong things are right now. Too far, too close, too fast, too slow.
- **σ** (sigma) is "how wrong is still normal?" A small sigma means the drone is fussy. A big sigma means it shrugs off small mistakes.
- The square means a mistake twice as big feels **four times** as surprising. A tiny mistake is almost nothing. A huge mistake is a shock.
- The 1/2 is a tidy constant from the bell curve. It does not change the story.

Where this comes from: the drone believes "mistakes are usually small, like a bell-shaped hill of probability." Things near the top of the hill are ordinary. Things out in the tail are rare. Surprise is basically "how rare is this?" For a bell curve, rare-ness grows with the square of the error. The code drops an extra constant that never changes the drone's decision, and keeps only the squared part.

In symbols, a normal distribution with spread σ gives

$$
p(\text{error}) \propto \exp\left(-\frac{1}{2}\left(\frac{\text{error}}{\sigma}\right)^2\right)
$$

and the information-theoretic surprisal is \(-\log p\). Up to a constant that does not depend on the error, that is exactly \(S\).

**Worked example.** Two drones want to be **60 cm** apart. Sigma is **10 cm**, so 10 cm off is a normal-sized mistake.

| Real gap | Error | Error / 10 cm | Surprise \(S\) | Feeling |
|---|---|---|---|---|
| 60 cm | 0 | 0 | **0** | Perfect. Nothing to do. |
| 70 cm | 10 cm | 1 | **0.5** | A bit off. |
| 80 cm | 20 cm | 2 | **2** | Clearly wrong. |
| 100 cm | 40 cm | 4 | **8** | Very surprised. |

20 cm of error is only twice 10 cm, but the surprise jumps from 0.5 to 2. That is the square doing its job.

Once the drone has \(S\), it does not "think." It uses \(S\) in a simple rule: push harder, slow down, or walk downhill on the surprise. The experiments differ only in **what they expected** and **what they do with the score**.

---

## Experiment 1 — two drones keeping 60 cm while drifting to a goal

Command: `surprisal_goal_navigation`.

Code: `src/experiments/surprisal_goal_navigation_experiment.py`.

Config: `config/experiments/surprisal_goal_navigation.yaml`.

**The picture in their heads:** "My partner is 60 cm away."

**The game:** two drones also slide their midpoint half a metre to the right. They must fix their spacing and travel together.

Two starting stories:

- **Too far:** they begin 80 cm apart and must come closer.
- **Too close:** they begin 45 cm apart and must move apart.

**Numbers from the config.** Desired gap 0.60 m. Sigma 0.10 m. A calm correction strength `base_kp = 0.25`.

**Step by step, using the too-far start (80 cm).**

1. Error: \(e = 0.80 - 0.60 = +0.20\) metres. Positive means "too far."
2. Surprise:

$$
S = \frac{1}{2}\left(\frac{0.20}{0.10}\right)^2 = 2
$$

3. How hard to correct:
   - **Fixed** controller (the plain baseline): always \(k_p = 0.25\).
   - **Adaptive** controller (the idea being tested): the strength grows with surprise,

$$
k_p = \min\big(0.25 \times (1 + 1.5 \times S),\; 0.60\big)
$$

   With \(S = 2\): \(0.25 \times (1 + 3) = 1.0\), then the cap pulls it down to **0.60**. A big surprise makes the correction as stiff as the rules allow.

4. The closing speed is

$$
\text{correction} = k_p \times e + 0.25 \times (\text{how fast the gap is changing})
$$

   If they are sitting still, adaptive gives \(0.60 \times 0.20 = 0.12\) m/s of closing. Fixed gives only \(0.25 \times 0.20 = 0.05\) m/s. Each drone takes half of that, so they walk toward each other and meet in the middle.

5. The extra \(0.25 \times\) gap-rate term is a brake. If they are already rushing together, this term fights the rush so they do not shoot past 60 cm. Same idea as not slamming a door.

**Safety, separate from surprise.** Below 40 cm an extra push forces them apart. Below 30 cm the flight stops and they land.

**What we hope to see (hypothesis H-D).** The adaptive drones should reach 60 cm sooner and stay closer to it, while their closest moment stays just as safe as the fixed drones.

On the real drones (19 Oct), the fixed controller did the job: too-far went from about 87 cm to 60 cm, too-close from about 43 cm to 60 cm, both in roughly 11 seconds. The adaptive version on real drones is still to be flown.

---

## Experiment 2 — three drones flying to one point, no leader

Command: `target_swarm`.

Code: `src/controllers/target_swarm_controller.py`, flown by `src/experiments/target_swarm_experiment.py`.

Config: `config/experiments/target_swarm.yaml`.

**Two pictures at once, for each drone:**

1. "I should be standing on the target."
2. "Each neighbour should be 60 cm away."

They cannot all stand on the same dot and stay 60 cm apart. The maths finds the compromise: a triangle around the target.

Start positions are about \((-0.9, -0.6)\), \((-0.4, 0)\), \((-0.9, 0.6)\). The target is \((0.6, 0)\).

**Surprise of drone \(i\):**

$$
S_{\text{goal}} = \frac{1}{2}\left(\frac{\text{distance to target}}{0.30}\right)^2
$$

$$
S_{\text{neighbour}} = \frac{1}{2}\left(\frac{\text{gap} - 0.60}{0.10}\right)^2
$$

added up for every other drone, and

$$
F = S_{\text{goal}} + S_{\text{neighbours}}
$$

Sigma for the target is 30 cm (being a bit away from the goal is ok). Sigma for neighbours is 10 cm (spacing is fussier).

**Example.** A drone is 90 cm from the target:

$$
S_{\text{goal}} = \frac{1}{2}\left(\frac{0.90}{0.30}\right)^2 = \frac{1}{2} \times 9 = 4.5
$$

One neighbour is 80 cm away instead of 60 cm:

$$
S_{\text{neighbour}} = \frac{1}{2}\left(\frac{0.20}{0.10}\right)^2 = 2
$$

So this drone's total surprise is \(4.5 + 2 = 6.5\), plus whatever the third drone adds.

**How surprise becomes motion.** \(F\) is like a hill. The drone looks at the slope (the gradient) and walks downhill:

$$
\text{velocity} = -0.02 \times (\text{slope of } F)
$$

Speed is capped at 10 cm per second.

Downhill on the goal part means "walk toward the target." Downhill on a neighbour part means "if I am too far, walk toward them; if I am too close, walk away." All of that is added into one velocity.

The slope itself, for one neighbour gap, points along the line between the two drones and has size \((\text{gap} - 0.60) / \sigma^2\). Moving against that slope closes a gap that is too big and opens a gap that is too small.

**The comparison.**

- **Surprise:** both wishes. They should settle in a triangle with sides a little under 60 cm. The pull of the target squeezes the triangle slightly. An offline check gave about 58 cm, with the safety push never needed.
- **Goal only:** only "go to the target." They crowd the same point, and only a safety push (below 35 cm) keeps them from piling up. That offline check had them about 33 cm apart, with the safety push on about 83% of the time.

Below 20 cm the run aborts.

**What we hope to see (hypothesis H-T).** The surprise controller keeps a larger closest-pair distance and a much smaller spacing error, and arrives at a similar time.

---

## Experiment 3 — the convoy (a little train)

Commands: `surprise_minimization` (E1, with surprise) and `convoy_baseline` (E2, without it).

Code: `src/controllers/convoy_controller.py`.

Config: `config/experiments/convoy.yaml` (simulator) and `config/experiments/convoy_real.yaml` (real arena).

Three drones in a line. The leader flies forward at 15 cm/s, then stops half a metre before an imaginary wall (in sim the wall is at \(x = 5\) m). Each follower tries to stay **1 metre** behind the drone in front. In the simulator they start at \(x = 0\), \(-1\), and \(-2\).

**Ordinary following (both versions).** "My parking spot is 1 m behind the one ahead." If I am too far back, speed up. If I am too close, slow down:

$$
\text{base speed} = \text{clamp}\big(0.8 \times (\text{desired } x - \text{my } x),\; \pm 0.15\big)
$$

This is a proportional controller: bigger spacing error, stronger speed change, with a gain of 0.8, capped at 15 cm/s.

**The surprise, and this one is a real prediction.** The follower's belief is: "the drone ahead should be moving at my speed."

$$
\text{prediction error} = v_{\text{ahead}} - v_{\text{me}}
$$

$$
U = \frac{1}{2}\left(\frac{v_{\text{ahead}} - v_{\text{me}}}{N}\right)^2
$$

\(N\) (`noise_scale`) is sigma for speed. In the sim config, \(N = 0.20\) m/s, so a 20 cm/s speed difference is a normal-sized surprise.

Then the follower turns its speed command down when it is surprised:

$$
\text{factor} = e^{-1 \cdot U}, \qquad \text{command} = \text{base speed} \times \text{factor}
$$

\(e^{-U}\) is about 1 when \(U\) is 0 (no surprise, fly normally) and shrinks toward 0 when \(U\) is large (almost freeze).

**Example.** The leader has just stopped (\(v_{\text{ahead}} = 0\)). The follower is still doing 15 cm/s. \(N = 0.20\).

$$
U = \frac{1}{2}\left(\frac{-0.15}{0.20}\right)^2 = 0.28
$$

$$
\text{factor} = e^{-0.28} \approx 0.75
$$

The follower only uses about 75% of the speed it would have used. It hesitates because the leader did something unexpected. That hesitation is meant to stop the followers from piling into the leader after the sudden stop.

If \(N\) is tiny, say 0.05, the same 15 cm/s difference gives a huge \(U\) and the factor collapses to nearly 0. The followers freeze. If \(N\) is large, say 0.40, the factor stays near 1 and surprise barely matters. The experiment tries \(N\) in {0.05, 0.10, 0.20, 0.40} to find a middle where hesitation helps and does not freeze them.

**The comparison.** E1 multiplies by that factor. E2 uses the plain following speed and ignores surprise. Both still slam into reverse if the gap drops under 55 cm. That safety move is applied last, so surprise can never cancel it.

**What we hope to see.**

- **H1:** after the leader stops, E1 overshoots less and does not let a gap fall under the safe distance.
- **H2:** this depends on \(N\). There is a range of \(N\) where E1 helps without freezing.

---

## Experiment 4 — fixing a crooked line

Command: `formation_distance`.

Code: `src/controllers/formation_controller.py`.

Config: `config/experiments/formation_distance.yaml`.

Three drones on a line. They want **50 cm** between neighbours. They start badly placed:

- positions 0 m, 0.3 m, 1.1 m
- so the gaps are **30 cm** (too tight) and **80 cm** (too loose)

Call the gap errors

$$
S_{12} = 0.30 - 0.50 = -0.20
$$

$$
S_{23} = 0.80 - 0.50 = +0.30
$$

The badness of the whole line is

$$
U = \frac{1}{2}\big(S_{12}^2 + S_{23}^2\big) = \frac{1}{2}(0.04 + 0.09) = 0.065
$$

\(U = 0\) only when both gaps are exactly 50 cm. Every 0.3 seconds, each drone steps a little downhill on \(U\), with step size `gain = 0.15`, and no step bigger than 3 cm:

- Drone 1 only cares about the first gap. \(S_{12}\) is negative (too tight), so it steps left and opens the gap: \(0.15 \times (-0.20) = -3\) cm.
- Drone 3 only cares about the second gap. That gap is too big, so it steps left, toward the middle. The raw step is bigger than 3 cm, so it is capped at 3 cm.
- The middle drone feels both gaps. The tight gap pushes it right, the loose gap also pulls it right, so it steps right (again capped at 3 cm):

$$
\text{step}_2 = 0.15 \times (S_{23} - S_{12}) = 0.15 \times 0.50 = 0.075 \text{ m} \rightarrow \text{capped at } 0.03 \text{ m}
$$

Repeat. The gaps walk toward 50 cm and \(U\) walks toward 0. That is hypothesis **H3**.

This experiment uses the squared error itself as the hill. It does not divide by a sigma. Same shape of maths, slightly simpler.

---

## The toy physics simulation (no real drone)

File: `src/simulation/surprisal_euler_lagrange_sim.py`.

Run with `python -m src.simulation.surprisal_euler_lagrange_sim`.

This is a tiny computer world of two dots, each pretending to weigh 35 grams. It is **not** CrazySim and it does not fly a real drone. The dots are tied together by an invisible spring.

Spring energy (a virtual potential):

$$
V = \frac{1}{2} k \,(\text{distance} - 0.90)^2
$$

Kinetic energy is the usual \(\frac{1}{2}mv^2\). Force is the slope of \(V\) (Hooke's law): the spring pulls if they are too far and pushes if they are too close.

$$
F = k \times (\text{distance} - 0.90)
$$

Newton's second law turns force into motion, with a little friction:

$$
m \times \text{acceleration} = F + \text{outside push} - (\text{damping} \times \text{velocity})
$$

Here the spring stiffness \(k\) is the thing surprise changes.

- Fixed spring: \(k = 0.80\) always.
- Adaptive spring: \(k = 0.80 \times (1 + 0.65 \times S)\), never above 3.0.
- Sigma in this toy is 15 cm, and the wished distance is 90 cm.

At the 3-second mark, a short shove (0.12 N for 0.18 s) pushes one dot toward the other. The spring fights back. The adaptive spring gets stiffer exactly when the gap is badly wrong, so it fights harder. This file is a physics cartoon of the same idea as the two-drone flight.

"Euler–Lagrange" here means the motion comes from energy (kinetic plus potential) instead of a hand-written velocity rule. For this simple spring, that energy rule gives the same force as Hooke's law above.

---

## New experiments: second order and Fitts' law

These two experiments are written up in [EXPERIMENTS.md](../EXPERIMENTS.md) §7 and §8. The maths is in `src/controllers/second_order_controller.py`, the flight scripts are `second_order_pair_experiment.py` and `second_order_swarm_experiment.py`, and the plots come from `analysis/plot_second_order_pair_runs.py` and `analysis/plot_second_order_swarm_runs.py`.

### What changes: from "speed" to "acceleration"

Every controller above sets the drone's **speed** straight from the error: big error, big speed. That already means "fast when far, slow when near." But the speed can jump instantly, which is jerky.

The new controllers set the drone's **acceleration** instead. Acceleration is the second derivative of position (position → speed → acceleration), so this is called **second order**:

$$
a = -k_p\, e - k_d\, \dot{e}
$$

- \(-k_p\, e\) is a **spring**. It pulls toward the target, harder when farther away.
- \(-k_d\, \dot{e}\) is a **shock absorber** (damping). It brakes harder the faster the error is changing.

Think of a car's suspension. A soft absorber makes the car bounce. A hard one makes it move like it's in honey. The "just right" setting is **critical damping**:

$$
\omega = \sqrt{k_p}, \qquad k_d = 2\sqrt{k_p} = 2\omega
$$

\(\omega\) is "how quick" the drone is.

**Golden rule:** if surprise makes the spring stiffer, recompute \(k_d = 2\sqrt{k_p}\) every step. Then the brakes always match the spring, and a surprised drone pushes harder without bouncing. The old `adaptive` controller does not do this. It stiffens the spring but keeps the same brakes.

The drone only accepts speed commands, so every 0.1 s we turn acceleration into speed:

$$
v_{\text{next}} = v + a \times 0.1
$$

### What the motion looks like

Starting from rest, a distance \(D\) from the target, a critically damped move follows

$$
e(t) = D\,(1 + \omega t)\, e^{-\omega t}
$$

The speed starts at 0, rises to a peak of \(D\omega / e\) (here \(e \approx 2.718\)), then fades to 0 right at the target. The speed curve is a smooth hill. Humans reaching for a cup move the same way.

### Fitts' law

Fitts' law comes from studies of people pointing at targets:

$$
T = a + b \log_2\!\left(\frac{D}{W} + 1\right)
$$

- \(D\) is how far away the target is.
- \(W\) is how big the target is (how close counts as "arrived").
- \(T\) is how long the move takes.
- \(\log_2(D/W + 1)\) is called the **index of difficulty** (ID).

The **log** is the key. A target twice as far does *not* take twice as long, just a bit longer. That only works if you **move faster when far**.

For the spring-plus-absorber drone, the time to get within \(W/2\) is about

$$
T \approx \frac{1}{\omega}\ln\!\left(\frac{2D}{W}\right) + \text{a small extra}
$$

That has the same shape as Fitts' law. The drone obeys it automatically, **as long as it never hits its speed limit**. To test it, we fly several start distances, plot time against ID, and check whether the points make a straight line.

### New experiment A: two drones, second order

Same task as Experiment 1: keep 60 cm while drifting to a goal.

Each step:

1. Error \(e = d - 0.60\) and surprise \(S = \frac{1}{2}(e/0.10)^2\), as before.
2. Spring stiffness:
   - `second_order`: \(k_p = 0.64\), so \(\omega = 0.8\).
   - `second_order_adaptive`: \(k_p = \min(0.64(1 + 1.5S),\ 1.44)\), so \(\omega\) goes up to 1.2 when surprised.
3. Brakes: \(k_d = 2\sqrt{k_p}\).
4. Wanted acceleration of the gap: \(a = -k_p e - k_d \times (\text{how fast the gap changes})\).
5. Add \(a \times 0.1\) to the gap-speed command, and split it: each drone does half, along the line between them.

**Example.** The gap is 20 cm too big, and \(\omega = 0.8\). "Arrived" means within ±5 cm, so \(W = 0.10\) m.

- The peak closing speed is \(0.20 \times 0.8 / 2.718 \approx 0.06\) m/s.
- Arrival takes about **3.4 s**.

Now the gap is 40 cm too big:

- The peak speed is about 0.12 m/s. It is twice as far, so it goes twice as fast.
- Arrival takes about **4.5 s**, not 6.8 s. That's Fitts' law.

**The sweep:** start gaps of 70, 80, 90, and 100 cm. All of them stay under the speed limit, so the time-versus-ID points should form a straight line.

**What we hope to see:**

- No overshoot past 60 cm.
- Smoother commands than `fixed` and `adaptive`.
- The surprise-adaptive version is faster without extra wobble.

### New experiment B: three drones, second order

Same task as Experiment 2: three drones, one target, settle in a triangle.

The surprise \(F\) is exactly the same. The only difference is that each drone now **accelerates** downhill on \(F\) instead of setting its speed from the slope.

Each drone has two kinds of springs:

- **Goal spring** (soft): pulls toward the target. Its absorber brakes the drone's own speed.
- **Neighbour springs** (stiff): one per other drone, keeping 60 cm. Their absorbers only brake how fast that pair's distance changes. Braking the drone's own speed here would stop the whole group flying together.

Why two kinds of absorbers? The goal spring is slow (\(\omega \approx 0.5\)) and the neighbour springs are fast (\(\omega \approx 2.1\)). One brake setting can't be "just right" for both, so each spring gets its own.

**Where the drones end up.** They stop where all the pushes cancel and the speed is zero. That is the same place as in Experiment 2: a triangle with sides of about 58 cm. Only the path there is smoother.

**The speed limit catch.** The peak speed is about \(D \times 0.5 / 2.718\). It hits the 0.15 m/s limit when the target is about 0.8 m away.

- Closer than about 0.8 m: Fitts' law should hold (a straight line against ID).
- Farther: the drones cruise at the limit, and time grows in a straight line with distance instead.

The sweep tests start distances of 0.4, 0.6, 0.8, 1.2, and 1.5 m, so we can see where the law breaks.

---

## How a flight actually uses the maths

Every tenth of a second (10 times a second; the line experiment is a bit slower, every 0.3 s):

1. Read where each drone is, from the simulator or from the Lighthouse cameras.
2. Optionally add a little random wobble (3 cm and 3 cm/s) to what the **controller** sees, so repeated simulator runs are not identical. Safety still uses the true position.
3. Compute the error, then \(S\), then a velocity.
4. Send that velocity to the drone.
5. If data stops arriving for half a second, or two drones get inside the critical distance, everybody lands.

The controller files only do arithmetic: positions in, velocities out. They do not know whether the drones are real.

---

## One honest note

In the two-drone flight and in the triangle flight, surprise is computed only from **how wrong the distance is right now**. A bigger error makes a stronger push, and because of the square, the push grows faster than a straight line. Comparing adaptive against fixed shows the effect of that curve.

The convoy is the experiment where surprise is a **prediction**: "I thought you would keep moving at my speed, and you did not."

A planned next step is to make the other flights surprised by a wrong prediction too, for example "the gap is not where I predicted it would be," so the tests can show that the prediction itself changes how they fly.
