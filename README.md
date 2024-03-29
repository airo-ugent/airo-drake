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
