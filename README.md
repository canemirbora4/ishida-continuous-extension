# Reaction-Diffusion Swarm Robots in Continuous Space

A port of Ishida (2025)'s grid-based reaction-diffusion swarm-robot control
algorithm to continuous space. The robots are
physical (mass, velocity, friction) on a 2D plane and self-organize—without a
map, identity, or central control—by exchanging "tokens" only with their
neighbors.

## Contents
- **Demo (no setup):** Open `Cell_Robot_Continuous.html` in a browser and press
  **Start**. Light, walls, noise, robot sizes, failures, and friction are
  controlled via sliders.
- **Experiments:** `node run_experiments.js` → `results/*.csv`;
  `python3 plots.py` → `figures/`. `verify.js` validates the core (22 tests).

## Contribution: three fixes
When the algorithm is copied verbatim, it loses its self-cohesion in continuous
space. (1) Spread tokens to **all neighbors** instead of a single one, (2) use
the potential formula in the **correct direction** (`N_total - w·N_inner`), and
(3) scale the shape score **correctly**. With these, the swarm stays cohesive
even without light, reaches the light ~2–3× faster than the grid version,
withstands noise, and exhibits two new behaviors: self-segregation by size and
self-limiting of density.

Work this is based on: T. Ishida, *J. Intelligent & Robotic Systems* 111:70 (2025).
