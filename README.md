# Scalable and Safe Multi-agent Motion Planner (S2M2)

This is a forked code repository of Scalable and Safe Multi-agent Motion Planner (S2M2). 

The original repository is associated with the paper [_Scalable and Safe Multi-Agent Motion Planning with Nonlinear Dynamics and Bounded Disturbances_](https://jkchengh.github.io/files/chen2021scalable.pdf), the link is https://github.com/jkchengh/s2m2.


# Getting Started
## Clone the repo
```bash
  git clone https://github.com/Tianpeng-Zhang/s2m2.git
```
## Install the necessary Dependecies

Computer setup: tested on Ubuntu 20.04, x86 architecture.

* polytope: a Python package that allows flexible construction of convex and non-convex polytopes, called polytope, which is part of the TuLip control toolbox. Install by `pip install polytope`.
* cvxopt: a convex optimization library. Install by `conda install -c conda-forge cvxopt`. 
* pypoman: this library implements common operations over convex polyhedra such as polytope projection, double description (conversion between halfspace and vertex representations), computing the Chebyshev center, etc. Install by `pip install pypoman`.
* shapely: a powerful library for geometry shape manipulations. Useful in visualization. Install by `conda install -c conda-forge shapely`
* Pyyaml: needed to read the yaml files. `conda install -c conda-forge pyyaml`

* **Gurobi**: a semi-commericalized optimization solver. Install its python interface + gurobi license. No need to install the entire Gurobi software.
  * Install Gurobi Python interface: https://www.gurobi.com/documentation/10.0/quickstart_windows/cs_anaconda_and_grb_conda_.html.
  * Create a Gurobi account, request for an academic license, run grbt get key to install the license on your computer. https://www.gurobi.com/login/

## Quick start

Go to notebooks/, run the Experiment.ipynb.

## Key functions
* `test` in `testSim.py`
   * Run MAMP simulation on a given environment(zigzag, parking, mit, etc). 
* `decentralized_algo` in `algs.decentralized`, 
   * Input: the environment(limites, obstacles, starts, goals), the agents(geometric shape, max velocity, the bloating radius).
   * Output: a sequence of multi-agent collision-free time-position waypoints,`(xt,yt,t)`, that minimize the total flowtime(sum of all travel times).
* `ref2traj` in `algs.ref2traj`
   * Convert the `(xt,yt,t)` generated by `decentralized_alg` into reference waypoint and control inputs, `(xref,yref,thetaref,uref)`.
