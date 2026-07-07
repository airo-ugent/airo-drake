# airo-drake
Python package to simplify working with [Drake](https://drake.mit.edu/) in combination with  [`airo-mono`](https://github.com/airo-ugent/airo-mono).

**Key motivation:**
  - 🔋**Batteries included:** Drake is a powerful robotics toolbox, but it can have a steep learning curve.
If you've worked with Drake, you likely ended up deep in the [C++ documentation](https://drake.mit.edu/doxygen_cxx/index.html) or in Russ Tedrake's [manipulation repo](https://github.com/RussTedrake/manipulation) looking for guidance. `airo-drake` aims to be a *batteries included* Python package to get you up and running quickly with your own robot scenes in Drake!

## Overview 🧾
**Use cases** - we currently use Drake mainly for:
  - 🎨 Visualization
  - 💥 Collision checking
  - ⏱️ Time parameterization of paths

**Features:**
  - 🏗️ Help building scenes
  - 📈 Visualization functions for TCP poses, IK solutions, robot arm trajectories
  - 🔄 Converting `airo-mono` types to Drake types

**Design choices:**
 - 🍃 **Lightweight:** We try to limit duplicating or wrapping Drake, and prefer adding examples over convenience functions.
  - 🔓 **Opt-in:** drake can function as full blown physics simulator, but for many use cases you dont need it, so we make sure this is opt-in.

## Calibration (UR only) 🎯
Every physical UR arm has its own *calibrated* DH parameters, which differ slightly
from the nominal DH parameters baked into the `airo_models` URDFs and into
`ur-analytic-ik`'s closed-form solution; enough to cause ~1-2mm TCP error.

For some use cases, this error is not a problem. Yet, it inhibits precise motion.
You can avoid it by using the calibrated DH parameters from the robot's control box.

Analytic IK can't consume calibrated DH directly, but it's still the right
tool for picking *which* joint-configuration branch to use; the accuracy then comes
from a second, numerical IK refinement against a calibrated URDF. `airo_drake`
provides `read_calibrated_dh`/`calibrated_dh_to_urdf` to build that URDF straight
from a UR controller. See [`notebooks/06_calibrated_urdf.ipynb`](./notebooks/06_calibrated_urdf.ipynb).

The two-stage IK itself is provided: `two_stage_calibrated_ik` (analytic branch-pick
via `ur-analytic-ik`, then a numerical refine on the calibrated model) and its
refine-only half `refine_calibrated_ik`. To try it on a real arm, run
`scripts/manual_calibrated_ik_hardware_test.py` (make sure to pass the right model to `--model`,
e.g., `ur3e`).

**This calibrated URDF is for IK only.** It has no visual or collision
geometry so it can't be mistaken for a collision or
visualization model. Always use the regular `airo_models` mesh model
(`add_manipulator`) for collision checking and visualization. As the ~1-2mm
calibration delta is far below normal collision padding, it stays valid for the
real robot.

## Getting started 🚀
Complete the [Installation 🔧](#installation-🔧) and then dive right into the [notebooks 📔](./notebooks/)!

## Installation 🔧
`airo-drake` is available on PyPi and installable with pip:
```
pip install airo-drake
```
However it depends on `airo-typing` from [`airo-mono`](https://github.com/airo-ugent/airo-mono) which is not on PyPi, so you have to install that yourself.

## Developer guide 🛠️
See the [`airo-mono`](https://github.com/airo-ugent/airo-mono) developer guide.
A very similar process and tools are used for this package.

### Releasing 🏷️
See [`airo-models`](https://github.com/airo-ugent/airo-models/tree/main), releasing `airo-drake` works the same way.
