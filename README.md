# Robotpy State Space Example

This is a minimal working example as a proof of concept for integrating state space control into FRC team 4774's robot code.

At the moment there are issues with poor interactions between the mechanism estimator and trapezoidal profile. If the the estimator is unconstrained and this allows complete divergence and the trapezoidal profile gets locked in a situation it cant easily recover from.

- We could relax the kinematic constraints on the mechanism but that is sketchy if we care about mechanism longetivity. (It will break itself to recover from a really bad state)
- constrain mechanism after predict and before command dispatch (I think this is the right way atm)

## Setup

### Install dependencies

We use `uv` to manage our dependencies in our development environments.
This includes the Python version, and any Python packages such as `wpilib`.

Install `uv` by following the [`uv` docs](https://docs.astral.sh/uv/).

After installing `uv`, use it to create a virtual environment and install our dependencies.

```sh
uv sync
```

## Run

### Simulation

Sim config files are supplied with helpful views. Hopefully this is not going to break with small size displays but ya cant win em all. If it does, delete all the `.json` files in the root of the repo and recreate and display the `Mechanism2d` on the dashboard to view the bits.

```sh
uv run robotpy sim
```
