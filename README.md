# Multi-Robot Centralized MPC Formation Control

MATLAB implementations of Model Predictive Control for multi-robot formation control using CasADi and acados.

## 📋 Requirements
- **MATLAB** (R2020b+)
- **CasADi** (3.5.5+)
- **acados** MPC solver
- **Control System Toolbox**

## 🤖 Implementations

### 1. Quadratic Cost - Static Formation
**File:** `Quadratic_Cost.`
- Static square formation control
- 4 unicycle robots with quadratic cost function
- Robots converge from random positions to target formation
- Cost: `J = Σ[(x - x_des)ᵀQ(x - x_des) + uᵀRu]`

### 2. Line Trajectory - Individual Paths  
**File:** `Line_Trajectory`
- Each robot follows individual straight-line trajectory
- Custom start/end points with coordinated timing
- Time-varying reference generation
- Formation maintenance metrics

### 3. Circle Trajectory - Formation Motion
**File:** `Circle_Trajectory`
- Entire formation moves along circular path
- Square formation rotates to align with trajectory tangent
- Adjustable radius (30m), center, angular velocity
- Real-time formation shape preservation

### 4. Lemniscate Trajectory - Complex Path
**File:** `Lemniscate_Trajectory`
- Formation follows figure-8 (lemniscate) trajectory
- Handles complex non-linear paths with changing curvature
- Adaptive heading control
- Performance metrics for challenging paths

## 🎯 Common Configuration
- **4 robots** with unicycle dynamics: `ẋ = v·cos(θ)`, `ẏ = v·sin(θ)`, `θ̇ = w`
- **MPC**: 60-step horizon, SQP_RTI solver, RK4 integration
- **Constraints**: v ∈ [-2, 2] m/s, w ∈ [-1, 1] rad/s
- **Simulation**: 80 seconds, comprehensive performance analysis

## 🚀 Quick Start
1. Install CasADi and acados
2. Run any file: `run('Circle_Trajectory')`
3. Start with Quadratic Cost for basics, then explore trajectory following

Each implementation includes trajectories, error analysis, control inputs, and real-time animation.
