# Control Systems Library (csl) — Design Document

**Version:** 0.2.0
**Language Standard:** C++17
**Author:** Learning-driven development alongside *C++ Primer, 5th Edition*

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Library Identity](#2-library-identity)
3. [Directory Structure](#3-directory-structure)
4. [Class Hierarchy](#4-class-hierarchy)
5. [Module Reference](#5-module-reference)
6. [Build System](#6-build-system)
7. [Testing Strategy](#7-testing-strategy)
8. [Design Principles: Build vs. Buy](#8-design-principles-build-vs-buy)
9. [40-Day Learning and Building Plan](#9-40-day-learning-and-building-plan)
10. [Library State Snapshots](#10-library-state-snapshots)
11. [Future Extensions](#11-future-extensions)

---

## 1. Introduction

This document is the authoritative design reference for the **Control Systems Library** (`csl`), a professional-grade C++ library built as a structured learning exercise aligned with *C++ Primer, 5th Edition* by Lippman, Lajoie, and Moo.

### 1.1 Revised Philosophy (v0.2.0)

The original design built every numeric primitive — matrices, vectors, decompositions — entirely from scratch. That approach maximized raw C++ practice but worked against two things the project also cares about: **runtime efficiency** and **not reinventing well-solved problems**. This revision changes the foundation while keeping the learning-by-building spirit fully intact:

- **Generic numerical infrastructure is delegated to best-in-class, battle-tested libraries.** Dense linear algebra (matrices, vectors, decompositions, solvers) comes from **Eigen**. Complex numbers come from `std::complex`. Basic math comes from `<cmath>`. Randomness comes from `<random>`. Containers and algorithms come from the STL. None of this is control-systems knowledge — it is generic numerical/software-engineering infrastructure, and Eigen in particular is faster, more numerically robust, and more thoroughly tested than anything a multi-week learning project could produce.
- **Everything that teaches control systems is still written by hand.** Transfer functions, state-space algebra, stability criteria (Routh-Hurwitz, Lyapunov), frequency response analysis, root locus, PID/Lead-Lag/Ziegler-Nichols, pole placement, LQR/LQG (including the Riccati equation solvers — Eigen has no such solver, and deriving it *is* the lesson), H-infinity synthesis, MPC formulation and its QP solver, iLQR, dynamic programming, Kalman/Extended Kalman/Unscented Kalman/particle filters, analog and digital filter *design* (Butterworth/Chebyshev/Elliptic/Bessel/FIR/IIR coefficient derivation), system identification (RLS/ARMAX/N4SID/frequency-domain), ODE integration for simulation, and — per explicit request — the **Fast Fourier Transform** itself, since frequency-domain thinking is central to how a controls engineer reasons about systems.
- **The result is a library that is fast (Eigen's expression templates and SIMD vectorization), memory-efficient (no redundant copies, no hand-rolled containers reinventing `std::vector`), and pedagogically focused** — every line of hand-written code in `csl` is there because it teaches something about control systems, not because it teaches "how to implement a matrix class."

By Day 40, the library contains production-quality implementations covering the full breadth of modern control engineering:

- Classical frequency-domain design (Bode, Nyquist, PID, Lead-Lag)
- Modern state-space methods (pole placement, LQR, LQG)
- Robust control (H-infinity, H2, mu-synthesis)
- Optimal control (MPC, iLQR, dynamic programming)
- State estimation (Kalman filter, EKF, UKF, particle filter)
- Digital filter design (Butterworth, Chebyshev, Elliptic, Bessel, FIR, IIR)
- System identification (least squares, RLS, ARMAX, subspace, frequency domain via FFT)
- Numerical simulation (ODE solvers, Monte Carlo, signal generation, hand-written FFT/spectral analysis)

### 1.2 Build vs. Hand-Write — At a Glance

| I need... | Source |
|---|---|
| Matrices, vectors, dot/norm, transpose | **Eigen** (`Eigen::MatrixXd`, `Eigen::VectorXd`) |
| LU / QR / SVD / Cholesky / eigendecomposition | **Eigen** (`.partialPivLu()`, `.householderQr()`, `.jacobiSvd()`, `.llt()`, `Eigen::EigenSolver`) |
| Matrix exponential / logarithm / square root | **Eigen** (`unsupported/Eigen/MatrixFunctions`) |
| Complex numbers | **`std::complex<double>`** |
| Basic math (`pow`, `exp`, `log`, `sqrt`, `sin`, `atan2`...) | **`<cmath>`** |
| Random distributions | **`<random>`** |
| Sorting, transforms, accumulation | **STL `<algorithm>`, `<numeric>`** |
| Transfer function / state-space / ZPK representations and algebra | **Hand-written** (this *is* control theory) |
| Stability criteria, frequency response, root locus | **Hand-written** |
| Controller design (PID, LQR, MPC, H-infinity...) | **Hand-written**, using Eigen underneath |
| Riccati / Lyapunov equation solvers | **Hand-written** (Eigen has no equivalent — this is the lesson) |
| State estimators (KF/EKF/UKF/PF) | **Hand-written** |
| Filter *design* (coefficient derivation) | **Hand-written** |
| System identification algorithms | **Hand-written** |
| FFT / spectral analysis | **Hand-written** (radix-2 Cooley-Tukey) |
| ODE integrators (Euler/RK4/RK45) | **Hand-written** |
| QP solver for MPC | **Hand-written** (active-set method) |

Section 8 expands this table with the reasoning behind every row.

---

## 2. Library Identity

| Property | Value |
|----------|-------|
| Namespace | `csl` (with sub-namespaces per module) |
| Language standard | C++17 |
| Build system | CMake 3.20+ |
| Linear algebra backend | **Eigen 3.4+** (header-only, fetched via `FetchContent` or system-installed) |
| Testing framework | Google Test (introduced Week 4) |
| Other external dependencies | None — Eigen and Google Test are the only two |
| Platform | Cross-platform: Windows, Linux, macOS |
| Numeric type | `csl::Real` (alias for `double`) |
| Matrix / Vector type | `Eigen::MatrixXd` / `Eigen::VectorXd`, aliased as `csl::Matrix` / `csl::Vector` |
| Ownership model | Value semantics for small numeric objects (Eigen types, cheap to move); `std::shared_ptr` for polymorphic system/controller/estimator hierarchies |
| Performance posture | Release builds use `-O3 -march=native` (or `/O2 /arch:AVX2` on MSVC); Eigen vectorization enabled by default; Eigen can optionally delegate to Intel MKL or OpenBLAS for further speedup on large problems (see Section 11) |

### Sub-namespace Map

```
csl::core          Fundamental types, control-specific math, exceptions, logging
csl::spectral      Hand-written FFT and spectral analysis tools
csl::systems       System model representations
csl::conversions   Inter-representation transformations
csl::analysis      System analysis tools
csl::design        Control design algorithms
  csl::design::classical
  csl::design::modern
  csl::design::robust
csl::optimal       Optimal control
csl::estimation    State estimation and observers
csl::filters       Analog and digital filter design
csl::sysid         System identification
csl::simulation    ODE solvers and simulation engine
csl::signals       Signal generation utilities
```

Why Eigen specifically: it is header-only (no separate compiled library to link, so it doesn't complicate the build or distribution story), uses expression templates so chained operations like `A.transpose() * B + C` compile down to a single fused loop with no temporary allocations, is SIMD-vectorized (SSE/AVX/NEON) automatically, and is the de facto standard for numerical C++ (used inside ROS, PyTorch's C++ backend, Ceres Solver, TensorFlow, and countless robotics/controls codebases). Learning to build a serious C++ library *on top of* Eigen is itself an industry-relevant skill — most production control/robotics C++ code is written this way, not by hand-rolling matrix classes.

---

## 3. Directory Structure

```
controlSystemCPPLibrary/
|-- CMakeLists.txt                  Root build file (fetches Eigen + GTest)
|-- .gitignore
|-- README.md
|-- docs/
|   `-- DESIGN.md                   This document
|
|-- include/
|   `-- csl/
|       |-- csl.hpp                 Master include (all modules)
|       |-- core/
|       |   |-- Types.hpp           Eigen aliases, std::complex alias, constants, enums
|       |   |-- MathUtils.hpp       Control-specific scalar helpers (dB, wrapAngle, deadband...)
|       |   |-- Polynomial.hpp      Polynomial arithmetic (TF numerator/denominator representation)
|       |   |-- LinearAlgebra.hpp   Control-specific matrix equations NOT provided by Eigen:
|       |   |                      Riccati (ARE/DARE), Lyapunov, controllability/observability, Gramians
|       |   |-- NumericalMethods.hpp Root finding (controller tuning) + active-set QP solver (MPC)
|       |   |-- Logger.hpp          Logging infrastructure
|       |   |-- DataExport.hpp      CSV import/export (Eigen matrices <-> disk)
|       |   |-- Exceptions.hpp      Exception hierarchy
|       |   `-- Registry.hpp        Named system registry
|       |
|       |-- spectral/
|       |   `-- FFT.hpp             Hand-written radix-2 Cooley-Tukey FFT/IFFT, PSD, empirical FRF
|       |
|       |-- systems/
|       |   |-- SystemModel.hpp     Abstract base (pure virtual interface)
|       |   |-- LTISystem.hpp       LTI abstract intermediate
|       |   |-- ContinuousLTI.hpp   Continuous-time LTI abstract
|       |   |-- DiscreteLTI.hpp     Discrete-time LTI abstract
|       |   |-- TransferFunction.hpp
|       |   |-- StateSpace.hpp
|       |   |-- ZeroPoleGain.hpp
|       |   |-- DiscreteTransferFunction.hpp
|       |   |-- DiscreteStateSpace.hpp
|       |   |-- DiscreteZPK.hpp
|       |   `-- CompositeSystem.hpp  Series/parallel/feedback algebra
|       |
|       |-- conversions/
|       |   |-- SystemConversions.hpp  tf2ss, ss2tf, zpk2tf, tf2zpk, ss2zpk
|       |   `-- Discretization.hpp    c2d, d2c (ZOH via Eigen matrix exponential, Tustin, Euler, MPZ)
|       |
|       |-- analysis/
|       |   |-- Stability.hpp         Routh-Hurwitz, Lyapunov, margins, H2/HInf norms
|       |   |-- FrequencyResponse.hpp Bode, Nyquist, Nichols, margins
|       |   |-- RootLocus.hpp         Root locus computation
|       |   `-- TimeResponse.hpp      Step, impulse, ramp, lsim
|       |
|       |-- design/
|       |   |-- classical/
|       |   |   |-- PIDController.hpp
|       |   |   |-- LeadLagCompensator.hpp
|       |   |   |-- ZieglerNichols.hpp
|       |   |   `-- LoopShaping.hpp
|       |   |-- modern/
|       |   |   |-- StateFeedback.hpp    Pole placement (Ackermann)
|       |   |   |-- LuenbergerObserver.hpp
|       |   |   |-- LQR.hpp             Hand-written algebraic Riccati equation solver
|       |   |   `-- LQG.hpp             LQR + Kalman filter
|       |   `-- robust/
|       |       |-- HInfinity.hpp       H-inf synthesis (gamma-iteration)
|       |       `-- H2Control.hpp       H2-optimal control
|       |
|       |-- optimal/
|       |   |-- MPC.hpp                 Model predictive control (hand-written QP solver)
|       |   |-- iLQR.hpp                Iterative LQR for nonlinear systems
|       |   `-- DynamicProgramming.hpp  Value/policy iteration
|       |
|       |-- estimation/
|       |   |-- Estimator.hpp           Abstract base
|       |   |-- KalmanFilter.hpp        Discrete-time linear KF
|       |   |-- ExtendedKalmanFilter.hpp EKF with user or numerical Jacobians
|       |   |-- UnscentedKalmanFilter.hpp UKF with sigma points
|       |   `-- ParticleFilter.hpp      Sequential importance resampling
|       |
|       |-- filters/
|       |   |-- FilterBase.hpp          Abstract filter template base
|       |   |-- ButterworthFilter.hpp
|       |   |-- ChebyshevFilter.hpp     Type I and II
|       |   |-- EllipticFilter.hpp
|       |   |-- BesselFilter.hpp
|       |   |-- FIRFilter.hpp           Windowed FIR design
|       |   |-- IIRFilter.hpp           IIR from analog prototype
|       |   `-- NotchFilter.hpp
|       |
|       |-- sysid/
|       |   |-- SystemIdentifier.hpp    Abstract base
|       |   |-- LeastSquaresID.hpp      Batch LS (built on Eigen's QR solve)
|       |   |-- RecursiveLeastSquares.hpp RLS with forgetting factor
|       |   |-- ARMAXModel.hpp          ARMAX identification
|       |   |-- SubspaceID.hpp          N4SID algorithm (built on Eigen SVD)
|       |   `-- FrequencyDomainID.hpp   Levy method, using csl::spectral FFT
|       |
|       |-- simulation/
|       |   |-- ODESolver.hpp           Euler, RK4, RK45 adaptive (hand-written)
|       |   |-- Simulator.hpp           Full simulation engine
|       |   |-- RingBuffer.hpp          Fixed-capacity ring buffer
|       |   `-- MonteCarloSimulator.hpp Monte Carlo with noise
|       |
|       `-- signals/
|           `-- SignalGenerator.hpp     Step, ramp, sinusoid, chirp, noise, PRBS
|
|-- src/                              Corresponding .cpp files, mirroring include/
|-- tests/                            Google Test suites, mirroring include/
`-- examples/
    |-- ex01_eigen_and_polynomial.cpp
    |-- ex02_pid_step_response.cpp
    |-- ex03_system_conversions.cpp
    |-- ex04_classical_control.cpp
    |-- ex05_lqg_control.cpp
    |-- ex06_kalman_filter.cpp
    |-- ex07_system_identification.cpp
    |-- ex08_mpc_control.cpp
    |-- ex09_filter_design.cpp
    |-- ex10_fft_spectral_analysis.cpp
    `-- ex11_full_pipeline.cpp
```

Notably absent compared to a from-scratch design: `core/Vector.hpp` and `core/Matrix.hpp` no longer exist as hand-rolled classes — they are one-line aliases inside `Types.hpp` pointing at Eigen. `core/LinearAlgebra.hpp` shrinks dramatically because decompositions and solves move to Eigen; what remains are the matrix *equations* that are genuinely control theory (Riccati, Lyapunov).

---

## 4. Class Hierarchy

**Note:** Every `Matrix` and `Vector` type referenced below is the Eigen alias defined in `Types.hpp` (`Eigen::MatrixXd` / `Eigen::VectorXd`), not a hand-rolled class. The hierarchies themselves — what classes exist, what they inherit from, what they store — are unchanged by the Eigen migration, because they describe control-theory relationships, not numeric storage.

### 4.1 System Model Hierarchy

```
SystemModel  [abstract]
  pure virtual: poles(), zeros(), isStable(), dcGain(), evaluate(s), order()
  |
  `-- LTISystem  [abstract]
        pure virtual: toStateSpace(), toTransferFunction()
        |
        |-- ContinuousLTI  [abstract]
        |     virtual: c2d(dt, method)
        |     |
        |     |-- TransferFunction
        |     |     Stores: num (Polynomial), den (Polynomial)
        |     |     Extra:  isProper(), isStrictlyProper(), partialFractions()
        |     |
        |     |-- StateSpace
        |     |     Stores: A, B, C, D (Eigen::MatrixXd)
        |     |     Extra:  isControllable(), isObservable(), gramians()
        |     |
        |     `-- ZeroPoleGain
        |           Stores: zeros, poles (std::vector<std::complex<double>>), gain (Real)
        |           Extra:  toBiquad()
        |
        `-- DiscreteLTI  [abstract]
              virtual: d2c(dt, method)
              |
              |-- DiscreteTransferFunction
              |-- DiscreteStateSpace
              `-- DiscreteZPK
```

### 4.2 Controller Hierarchy

```
Controller  [abstract]
  pure virtual: compute(error, dt) -> Real
  virtual:      reset(), setParameters(ParameterMap)
  |
  |-- ClassicalController  [abstract]
  |     |
  |     |-- PIDController
  |     |     Stores: Kp, Ki, Kd, derivative_filter_N
  |     |     Extra:  setAntiWindup(), setOutputLimits(), tune(method)
  |     |     Inherits: operator()(error, dt)
  |     |
  |     |-- LeadLagCompensator
  |     |     Stores: zero, pole, gain (all in s-domain)
  |     |     Extra:  asTransferFunction(), phaseContribution(w)
  |     |
  |     `-- LoopShapingController
  |           Stores: compensator (TransferFunction)
  |
  |-- StateFeedbackController  [abstract]
  |     Stores: K (gain Matrix), x_ref (Vector)
  |     pure virtual: computeGain(sys, ...)
  |     |
  |     |-- PolePlacementController
  |     |     Extra: ackermannFormula(A, B, desired_poles)
  |     |
  |     `-- LQRController
  |           Extra: solveRiccati(A, B, Q, R) [hand-written], K (optimal gain)
  |           |
  |           `-- LQGController
  |                 Stores: kalman_filter (KalmanFilter), K_lqr (Matrix)
  |                 Extra:  estimate(y, u), computeControl(y)
  |
  `-- MPCController
        Stores: horizon N, Np, Nu, Q, R, constraints
        Extra:  buildQP() [Eigen for matrix algebra], solveQP() [hand-written active-set solver]
```

### 4.3 Estimator Hierarchy

```
Estimator  [abstract]
  pure virtual: predict(u), update(y), getState() -> Vector
  virtual:      reset(), getCovariance() -> Matrix
  |
  |-- LuenbergerObserver
  |     Stores: L (observer gain Matrix), sys (StateSpace)
  |     Extra:  designPoles(desired_observer_poles)
  |
  `-- KalmanFilter  [base for all KF variants]
        Stores: x_hat, P, Q_noise, R_noise (Eigen types)
        Extra:  predict(u), update(y), getKalmanGain()
        |
        |-- ExtendedKalmanFilter
        |     Stores: f (dynamics func), h (output func)
        |             Jf (Jacobian of f), Jh (Jacobian of h)
        |     Note:   User may supply analytic Jacobians, or fall back to
        |             NumericalMethods.hpp finite-difference Jacobian
        |
        `-- UnscentedKalmanFilter
              Stores: alpha, beta, kappa (sigma point params)
              Extra:  generateSigmaPoints(), sigmaWeights()
              |
              `-- ParticleFilter
                    Stores: N_particles, particles, weights
                    Extra:  resample(), importanceSample()
```

### 4.4 Filter Hierarchy

```
Filter<T>  [abstract template]
  pure virtual: process(T input) -> T
  virtual:      reset(), getOrder() -> int
  |
  |-- AnalogFilter  [abstract]
  |     Stores: cutoff_freq, order, ripple_db
  |     virtual: toTransferFunction() -> TransferFunction
  |     virtual: digitize(fs, method) -> IIRFilter
  |     |
  |     |-- ButterworthFilter
  |     |     Extra:  analogPrototype(n), frequencyTransform(type, wc)
  |     |
  |     |-- ChebyshevFilter
  |     |     Extra:  type (I or II), ripple_db
  |     |
  |     |-- EllipticFilter
  |     |     Extra:  passband_ripple, stopband_attenuation
  |     |
  |     `-- BesselFilter
  |           Extra:  groupDelay(), phaseDelay()
  |
  |-- DigitalFilter  [abstract]
  |     Stores: sample_rate, order
  |     |
  |     |-- FIRFilter
  |     |     Stores: coefficients (Vector), window type
  |     |     Extra:  designLowpass(), designHighpass(), designBandpass()
  |     |             applyWindow(type), windowedSinc()
  |     |
  |     |-- IIRFilter
  |     |     Stores: b (numerator), a (denominator) as Vectors
  |     |     Extra:  Direct Form II Transposed implementation
  |     |             toSOS() -> second-order sections
  |     |
  |     `-- NotchFilter
  |           Stores: notch_freq, bandwidth, Q_factor
  |
  `-- AdaptiveFilter  [abstract]
        Stores: filter_order, step_size
        |
        `-- LMSFilter
              Extra: updateWeights(input, error)
```

### 4.5 System Identifier Hierarchy

```
SystemIdentifier  [abstract]
  pure virtual: identify(input, output) -> SystemModel
  virtual:      getResiduals() -> Vector, validate(input, output) -> Real
  |
  |-- LeastSquaresID
  |     Stores: model_order_n, model_order_m
  |     Extra:  buildRegressionMatrix(y, u), solve() [Eigen QR/SVD solve]
  |     |
  |     `-- ARMAXModel
  |           Stores: na, nb, nc (AR, X, MA orders)
  |           Extra:  instrumentalVariable()
  |
  |-- RecursiveLeastSquaresID
  |     Stores: forgetting_factor (lambda), P_matrix, theta
  |     Extra:  update(y_new, u_new), getEstimate() -> Vector
  |
  |-- SubspaceID
  |     Stores: block_rows i, state_order n
  |     Extra:  hankelMatrix(y, u), N4SID() [Eigen SVD]
  |
  `-- FrequencyDomainID
        Stores: frequency_data, FRF data
        Extra:  levyMethod() using csl::spectral::fft for FRF estimation
```

---

## 5. Module Reference

### 5.1 `core` — Foundational Types and Control-Specific Math

This module is the most changed by the Eigen migration. It used to contain hand-rolled `Vector`, `Matrix`, and a full `LinearAlgebra.hpp` of decompositions. Now it contains only thin type aliases plus the handful of matrix *equations* that are genuinely control-theoretic and that Eigen does not provide.

#### `Types.hpp`

```cpp
#pragma once
#include <Eigen/Dense>
#include <complex>
#include <cstddef>
#include <limits>

namespace csl {

using Real  = double;
using Index = Eigen::Index;

// Matrix/Vector are Eigen aliases. We do not hand-roll linear algebra types —
// Eigen is faster, better tested, and SIMD-vectorized. See Section 8.
using Matrix        = Eigen::MatrixXd;
using Vector        = Eigen::VectorXd;
using RowVector      = Eigen::RowVectorXd;
using Complex        = std::complex<Real>;
using ComplexVector  = Eigen::VectorXcd;
using ComplexMatrix  = Eigen::MatrixXcd;

// Mathematical constants (constexpr — this is data, not an algorithm, so
// declaring it ourselves is not "reinventing the wheel")
inline constexpr Real PI          = 3.141592653589793238462643383279502884;
inline constexpr Real TWO_PI      = 2.0 * PI;
inline constexpr Real HALF_PI     = PI / 2.0;
inline constexpr Real EULER_GAMMA = 0.577215664901532860606512090082402431;
inline constexpr Real INF         = std::numeric_limits<Real>::infinity();
inline constexpr Real EPS         = std::numeric_limits<Real>::epsilon();

enum class SystemType              { TF, SS, ZPK, DTF, DSS, DZPK, Nonlinear };
enum class LogLevel                { DEBUG, INFO, WARN, ERROR };
enum class DiscretizationMethod    { ZOH, Tustin, EulerForward, EulerBackward, MatchedPoleZero };
enum class FilterType              { Lowpass, Highpass, Bandpass, Bandstop };
enum class WindowType              { Rectangular, Hamming, Hanning, Blackman, Kaiser };

} // namespace csl
```

**What used to be here and where it went:** the old `Complex` struct (hand-written real/imag arithmetic) is now `std::complex<Real>`, which has correct, well-tested arithmetic, `std::abs`, `std::arg`, `std::conj`, and interoperates directly with Eigen's `ComplexMatrix`/`ComplexVector`. The old `Vector`/`Matrix` classes are gone entirely — `Eigen::VectorXd`/`Eigen::MatrixXd` already provide `.size()`, `.norm()`, `.dot()`, `operator[]`/`operator()`, iterators (`.begin()`/`.end()` since Eigen 3.4), arithmetic operators, and stream output, all more efficiently than a hand-rolled equivalent.

#### `MathUtils.hpp`

Only functions that are genuinely control/DSP-specific remain here — things `<cmath>` does not provide because they are not general math, they are control-engineering conventions.

```cpp
namespace csl {

// Decibel conversions (control/DSP convention, not a generic math operation)
inline Real dB(Real magnitude) noexcept { return 20.0 * std::log10(std::abs(magnitude)); }
inline Real dB_power(Real power)  noexcept { return 10.0 * std::log10(power); }
inline Real fromDB(Real db)       noexcept { return std::pow(10.0, db / 20.0); }

// Angle wrapping for phase computations (Bode/Nyquist phase unwrapping)
inline Real wrapAngle(Real angle) noexcept {
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Actuator/sensor nonlinearity models used throughout controller simulation
inline Real saturate(Real v, Real limit) noexcept { return std::clamp(v, -limit, limit); }
inline Real deadband(Real v, Real band)  noexcept {
    if (std::abs(v) < band) return 0.0;
    return v - (v > 0 ? band : -band);
}

// Filter-design-specific special functions (Bessel filter pole derivation
// needs these; they are not provided by <cmath>)
Real besselPolyCoeff(int n, int k);   // reverse Bessel polynomial coefficients

} // namespace csl
```

Note what is deliberately **absent**: no hand-rolled `clamp` (that is `std::clamp`, C++17), no `sign` reinvention beyond a one-liner using `std::signbit`/comparisons where actually needed inline, no `factorial`/`binomialCoeff`/generic special-function library — those either come from `<cmath>` (`std::tgamma` for factorial-like needs) or, where truly control/filter-specific (Bessel polynomials), are kept because they encode filter-design domain knowledge, not generic math.

#### `Polynomial.hpp`

A transfer function numerator or denominator **is** a polynomial — this class is control-theory representation, not generic math, so it stays hand-written. Root-finding, however, delegates to Eigen: we build the companion matrix and hand it to `Eigen::EigenSolver`, rather than hand-rolling an eigenvalue iteration.

```cpp
namespace csl {

class Polynomial {
public:
    Polynomial() = default;
    explicit Polynomial(std::initializer_list<Real> coeffs);   // descending order
    explicit Polynomial(Vector coeffs);
    static Polynomial fromRoots(const std::vector<Complex>& roots);
    static Polynomial monomial(int degree, Real coeff = 1.0);

    int    degree() const;
    Real   leadingCoeff() const;
    Real   operator()(Real x) const;         // Horner's method evaluation
    Complex evaluate(Complex x) const;

    Polynomial operator+(const Polynomial& rhs) const;
    Polynomial operator-(const Polynomial& rhs) const;
    Polynomial operator*(const Polynomial& rhs) const;
    Polynomial operator*(Real scalar) const;
    Polynomial derivative() const;
    Polynomial monic() const;

    // Roots via companion matrix + Eigen::EigenSolver — we build the control-
    // theory-specific companion matrix; Eigen does the (generic) eigenvalue math.
    std::vector<Complex> roots() const;

    static Polynomial gcd(const Polynomial& a, const Polynomial& b);

    Vector toVector() const;
    friend std::ostream& operator<<(std::ostream& os, const Polynomial& p);

private:
    Vector coeffs_;   // coeffs_[0] is highest degree
};

} // namespace csl
```

#### `LinearAlgebra.hpp`

This is the header that most demonstrates the new philosophy. Everything that used to be a from-scratch decomposition (`solve`, `inverse`, `lstsq`, `rank`, `lu`, `qr`, `svd`, `eig`, `cholesky`, `expm`, `logm`, `sqrtm`) is **deleted** — call Eigen directly wherever those were used (see the cheat sheet below). What remains are matrix *equations specific to control theory* that have no Eigen equivalent because they are not generic linear algebra, they are control theory:

```cpp
namespace csl {

// Continuous-time algebraic Riccati equation: A'P + PA - PBR^{-1}B'P + Q = 0
// Solved via the Schur/Hamiltonian-matrix eigen-decomposition method.
// This is THE method that derives an LQR gain — hand-written because
// deriving it is the point of learning LQR, and Eigen provides no such solver.
Matrix solveContinuousARE(const Matrix& A, const Matrix& B,
                          const Matrix& Q, const Matrix& R);

// Discrete-time algebraic Riccati equation (used by discrete LQR / steady-state KF)
Matrix solveDiscreteARE(const Matrix& A, const Matrix& B,
                        const Matrix& Q, const Matrix& R);

// Continuous Lyapunov equation: A*P + P*A' + Q = 0 (stability analysis, H2 norm,
// controllability/observability Gramians). Solved via Eigen::RealSchur underneath
// (Bartels-Stewart algorithm) — the Schur decomposition is generic (Eigen), but
// assembling and solving the Lyapunov equation from it is the control-theory step.
Matrix solveLyapunov(const Matrix& A, const Matrix& Q);
Matrix solveDiscreteLyapunov(const Matrix& A, const Matrix& Q);

// Controllability / observability (Kalman rank condition, PBH test)
Matrix controllabilityMatrix(const Matrix& A, const Matrix& B);
Matrix observabilityMatrix(const Matrix& A, const Matrix& C);
bool   isControllable(const Matrix& A, const Matrix& B, Real tol = 1e-9);
bool   isObservable(const Matrix& A, const Matrix& C, Real tol = 1e-9);

// Gramians (balancing, H2 norm, MPC weighting)
Matrix controllabilityGramian(const Matrix& A, const Matrix& B);
Matrix observabilityGramian(const Matrix& A, const Matrix& C);

// Kronecker product + vectorization — small linear-algebra helpers needed only
// to reformulate the Lyapunov equation as a linear system when a Schur-based
// solve isn't wanted (e.g. very small systems, or teaching the alternative
// derivation). Kept because they directly support the equations above.
Matrix kron(const Matrix& A, const Matrix& B);
Vector vec(const Matrix& A);
Matrix unvec(const Vector& v, Index rows, Index cols);

} // namespace csl
```

**Eigen cheat sheet — what replaces the old hand-rolled API:**

| Old hand-rolled call | Eigen equivalent |
|---|---|
| `solve(A, b)` | `A.partialPivLu().solve(b)` (or `.fullPivLu()` for rank-deficient) |
| `lstsq(A, b)` | `A.bdcSvd(Eigen::ComputeThinU \| Eigen::ComputeThinV).solve(b)` or `A.colPivHouseholderQr().solve(b)` |
| `inverse(A)` | `A.inverse()` |
| `rank(A)` | `A.fullPivLu().rank()` |
| `A.lu()` | `A.partialPivLu()` / `A.fullPivLu()` |
| `A.qr()` | `A.householderQr()` / `A.colPivHouseholderQr()` |
| `A.svd()` | `A.bdcSvd(...)` / `A.jacobiSvd(...)` |
| `A.cholesky()` | `A.llt()` (or `.ldlt()` for semidefinite) |
| `eigenvalues(A)` (symmetric) | `Eigen::SelfAdjointEigenSolver<Matrix>(A).eigenvalues()` |
| `eigenvalues(A)` (general) | `Eigen::EigenSolver<Matrix>(A).eigenvalues()` |
| `expm(A)` | `#include <unsupported/Eigen/MatrixFunctions>`, then `A.exp()` |
| `A.pseudoInverse()` | `A.completeOrthogonalDecomposition().pseudoInverse()` |
| `A.conditionNumber()` | ratio of `jacobiSvd().singularValues()` extremes |
| `norm(v)` / `norm(A, "fro")` | `v.norm()` / `A.norm()` (Eigen's default matrix norm is Frobenius) |

#### `NumericalMethods.hpp`

Two categories of algorithm remain hand-written here, both because they are the mechanism *inside* a control algorithm the library teaches, not generic utilities:

```cpp
namespace csl {

// Scalar root-finding — used by controller auto-tuning (relay feedback,
// gain/phase crossover search) and Ziegler-Nichols-style methods.
Real bisection(std::function<Real(Real)> f, Real a, Real b, Real tol = 1e-10, int maxIter = 100);
Real newtonRaphson(std::function<Real(Real)> f, std::function<Real(Real)> df,
                   Real x0, Real tol = 1e-10, int maxIter = 100);
Real secantMethod(std::function<Real(Real)> f, Real x0, Real x1, Real tol = 1e-10);
Real goldenSection(std::function<Real(Real)> f, Real a, Real b, Real tol = 1e-8);

// Numerical Jacobian — fallback for EKF when the user has no analytic
// Jacobian available; central finite differences.
Matrix numericalJacobian(std::function<Vector(const Vector&)> f, const Vector& x, Real h = 1e-6);

// Active-set Quadratic Program solver — this is the optimization engine
// inside MPC. Hand-written deliberately: understanding how MPC solves its
// QP at every timestep is central to understanding MPC. (A production
// deployment could later swap this for OSQP without changing the MPC
// formulation code — see Section 11.)
struct QPResult { Vector x; bool converged; int iterations; };
QPResult solveQP(const Matrix& H, const Vector& f,
                 const Matrix& A_ineq, const Vector& b_ineq,
                 const Matrix& A_eq,  const Vector& b_eq);

} // namespace csl
```

#### `Exceptions.hpp`, `Logger.hpp`, `DataExport.hpp`, `Registry.hpp`

Unchanged in spirit from the original design — these are small, general-purpose infrastructure classes (exception hierarchy, logging, CSV I/O, named system registry) with no numerically intensive content, so there was never a "build vs. buy" tension here. `DataExport.hpp` now reads/writes `Eigen::MatrixXd` directly instead of the old hand-rolled `Matrix`.

```cpp
namespace csl {
class CslException      : public std::runtime_error { /* ... */ };
class DimensionMismatch  : public CslException { /* ... */ };
class SingularMatrix     : public CslException { /* ... */ };
class UnstableSystem     : public CslException { /* ... */ };
class ConvergenceFailure : public CslException { /* ... */ };
class InvalidParameter   : public CslException { /* ... */ };

class Logger { /* singleton, log levels, file+console output — unchanged */ };

void   writeCSV(const std::string& filename, const std::vector<std::string>& headers, const Matrix& data);
Matrix readCSV(const std::string& filename, bool has_header = true);

using ParameterMap = std::unordered_map<std::string, Real>;
class SystemRegistry { /* std::map<std::string, std::shared_ptr<SystemModel>> — unchanged */ };
} // namespace csl
```

---

### 5.1b `spectral` — Hand-Written FFT and Spectral Analysis

This is new relative to the original design, and exists specifically because the Fourier transform was called out as something worth writing by hand: frequency-domain thinking (Bode, spectral estimation, empirical transfer function identification) is core to how a controls engineer reasons about systems, so implementing the FFT — not just calling one — is treated as first-class control-systems learning content, on the same footing as writing a Kalman filter.

#### `FFT.hpp`

```cpp
namespace csl::spectral {

// In-place radix-2 Cooley-Tukey FFT. Requires x.size() to be a power of two
// (callers pad with zeros otherwise). O(N log N).
void fft(ComplexVector& x);
void ifft(ComplexVector& x);       // inverse FFT (also in-place)

// Convenience wrappers for real-valued time-domain signals
ComplexVector fft(const Vector& x);              // zero-pads to next power of 2
Vector        fftMagnitude(const Vector& x, Real fs, Vector& freq_out);

// Power spectral density via Welch's method (segmented + averaged periodogram) —
// built on top of the fft() above; the averaging/windowing procedure is the
// control/DSP-specific content, the FFT itself is the primitive it's built from.
Vector welchPSD(const Vector& x, Real fs, int segment_length = 256, Real overlap = 0.5);

// Empirical transfer function estimate from measured input/output data:
// H(jw) = FFT(y) / FFT(u), the basis for csl::sysid::FrequencyDomainID
struct FRFResult { Vector frequencies; ComplexVector H; };
FRFResult empiricalTransferFunction(const Vector& u, const Vector& y, Real fs);

} // namespace csl::spectral
```

Implementation notes carried into the Day 6-7 build tasks (Section 9): the classic recursive Cooley-Tukey formulation is reworked iteratively using a **bit-reversal permutation** (a direct, satisfying application of `std::bitset` and bitwise operators from Chapter 4/17) followed by `log2(N)` butterfly stages — the textbook iterative FFT structure, written once, used everywhere frequency-domain information is needed in the rest of the library (Bode plots still use direct analytic evaluation of `H(jw)`, since that's exact and cheap for a rational transfer function; the FFT is used wherever we only have *sampled data*, e.g. system identification and PSD estimation).

---

### 5.2 `systems` — System Representations

All matrix members are Eigen types now. The public API is essentially unchanged from the original design (that was intentional — the original custom `Matrix` API was already modeled on Eigen's) but internal implementations of poles, controllability, etc. now call Eigen directly instead of a hand-rolled decomposition.

#### `SystemModel.hpp` — Abstract Base

```cpp
namespace csl::systems {

class SystemModel {
public:
    virtual ~SystemModel() = default;
    virtual std::vector<Complex> poles()    const = 0;
    virtual std::vector<Complex> zeros()    const = 0;
    virtual bool                 isStable() const = 0;
    virtual Real                 dcGain()   const = 0;
    virtual Complex              evaluate(Complex s) const = 0;
    virtual int                  order()    const = 0;
    virtual SystemType           type()     const = 0;
    virtual void                 print(std::ostream& os) const = 0;

    bool isMarginallyStable() const;
    bool isMinimumPhase()     const;
    int  relativeOrder()      const;

    friend std::ostream& operator<<(std::ostream& os, const SystemModel& sys) {
        sys.print(os); return os;
    }
protected:
    std::string name_;
};

class LTISystem : public SystemModel {
public:
    virtual StateSpace       toStateSpace()       const = 0;
    virtual TransferFunction toTransferFunction() const = 0;
};

class ContinuousLTI : public LTISystem {
public:
    virtual DiscreteStateSpace c2d(Real dt, DiscretizationMethod m = DiscretizationMethod::ZOH) const = 0;
};

class DiscreteLTI : public LTISystem {
public:
    virtual Real sampleTime() const = 0;
    bool isStable() const override;   // eigenvalues of Ad inside unit circle
};

} // namespace csl::systems
```

#### `TransferFunction.hpp`

```cpp
namespace csl::systems {

class TransferFunction : public ContinuousLTI {
public:
    TransferFunction(Polynomial num, Polynomial den);
    TransferFunction(std::initializer_list<Real> num_c, std::initializer_list<Real> den_c);

    std::vector<Complex> poles() const override;   // Polynomial::roots() -> Eigen::EigenSolver
    std::vector<Complex> zeros() const override;
    bool                 isStable() const override;
    Real                 dcGain() const override;
    Complex              evaluate(Complex s) const override;
    int                  order() const override;

    bool       isProper() const;
    bool       isStrictlyProper() const;
    Real       evaluateMagnitude(Real w) const;
    Real       evaluatePhase(Real w) const;
    Complex    operator()(Complex s) const { return evaluate(s); }   // functor call (Day 29)

    TransferFunction operator*(const TransferFunction& rhs) const;   // series
    TransferFunction operator+(const TransferFunction& rhs) const;   // parallel
    TransferFunction feedback(const TransferFunction& C, Real sign = -1.0) const;

    StateSpace toStateSpace() const override;   // controllable canonical form, built with Eigen
    DiscreteTransferFunction c2d(Real dt, DiscretizationMethod m) const override;

    void print(std::ostream& os) const override;
private:
    Polynomial num_, den_;
};

} // namespace csl::systems
```

#### `StateSpace.hpp`

```cpp
namespace csl::systems {

class StateSpace : public ContinuousLTI {
public:
    StateSpace(Matrix A, Matrix B, Matrix C, Matrix D);   // Matrix = Eigen::MatrixXd

    std::vector<Complex> poles() const override;   // Eigen::EigenSolver<Matrix>(A_).eigenvalues()
    std::vector<Complex> zeros() const override;   // transmission zeros via generalized eigenvalue problem
    bool                 isStable() const override;
    Real                 dcGain() const override;
    Complex              evaluate(Complex s) const override;   // C*(sI-A)^-1*B + D via Eigen solve
    int                  order() const override;

    int numStates()  const { return static_cast<int>(A_.rows()); }
    int numInputs()  const { return static_cast<int>(B_.cols()); }
    int numOutputs() const { return static_cast<int>(C_.rows()); }
    std::tuple<int,int,int> size() const { return {numStates(), numInputs(), numOutputs()}; }

    const Matrix& A() const { return A_; }
    const Matrix& B() const { return B_; }
    const Matrix& C() const { return C_; }
    const Matrix& D() const { return D_; }

    bool   isControllable(Real tol = 1e-9) const;   // csl::isControllable(A_, B_, tol)
    bool   isObservable(Real tol = 1e-9)   const;
    Matrix controllabilityGramian() const;           // csl::controllabilityGramian(A_, B_)
    Matrix observabilityGramian()   const;

    StateSpace operator+(const StateSpace& rhs) const;   // parallel
    StateSpace series(const StateSpace& rhs) const;
    StateSpace feedback(const StateSpace& K, Real sign = -1.0) const;

    TransferFunction   toTransferFunction() const override;   // SISO only
    DiscreteStateSpace  c2d(Real dt, DiscretizationMethod m) const override;   // ZOH via A_.exp()

    void print(std::ostream& os) const override;
private:
    Matrix A_, B_, C_, D_;
};

} // namespace csl::systems
```

`evaluate(Complex s)` is implemented as `(C_.cast<Complex>() * (s * Matrix::Identity(n,n).cast<Complex>() - A_.cast<Complex>()).inverse() * B_.cast<Complex>() + D_.cast<Complex>())` — a single Eigen expression, computed with `ComplexMatrix` (`Eigen::MatrixXcd`), no manual complex-arithmetic loop needed.

#### `ZeroPoleGain.hpp` and `CompositeSystem.hpp`

Same public surface as the original design; `zpk2ss`/`ss2zpk`/`tf2zpk` internals now build companion matrices and hand them to `Eigen::EigenSolver`/`Eigen::ComplexEigenSolver` rather than a hand-written eigenvalue iteration. `CompositeSystem.hpp`'s `series`/`parallel`/`feedback` functions build block matrices with `Eigen::MatrixXd`'s block assignment (`result.block(r,c,rows,cols) = ...`) instead of a hand-rolled `horzcat`/`vertcat`.

---

### 5.3 `conversions` — System Transformations

```cpp
namespace csl::conversions {

StateSpace       tf2ss(const TransferFunction& tf);
TransferFunction ss2tf(const StateSpace& ss);
TransferFunction zpk2tf(const ZeroPoleGain& zpk);
ZeroPoleGain     tf2zpk(const TransferFunction& tf);
StateSpace       zpk2ss(const ZeroPoleGain& zpk);
ZeroPoleGain     ss2zpk(const StateSpace& ss);

DiscreteStateSpace       c2d(const StateSpace& sys, Real dt, DiscretizationMethod m = DiscretizationMethod::ZOH);
DiscreteTransferFunction c2d(const TransferFunction& sys, Real dt, DiscretizationMethod m = DiscretizationMethod::Tustin);
StateSpace               d2c(const DiscreteStateSpace& sys, Real dt, DiscretizationMethod m = DiscretizationMethod::ZOH);
TransferFunction         d2c(const DiscreteTransferFunction& sys, Real dt, DiscretizationMethod m = DiscretizationMethod::Tustin);

} // namespace csl::conversions
```

**ZOH discretization**, the trickiest of these, is now three lines instead of a hand-rolled matrix-exponential integrator:

```cpp
#include <unsupported/Eigen/MatrixFunctions>   // provides Matrix::exp()

DiscreteStateSpace c2d_zoh(const StateSpace& sys, Real dt) {
    int n = sys.numStates(), m = sys.numInputs();
    Matrix M(n + m, n + m);
    M.setZero();
    M.topLeftCorner(n, n)  = sys.A();
    M.topRightCorner(n, m) = sys.B();
    Matrix Md = M.exp();                       // <-- the only "hard part", handled by Eigen
    return DiscreteStateSpace(Md.topLeftCorner(n, n), Md.topRightCorner(n, m), sys.C(), sys.D(), dt);
}
```

This augmented-matrix-exponential trick for exact ZOH discretization is itself a control-theory technique worth learning (why it works is exactly why we still write the *conversion function* by hand) — it just no longer requires us to also write a matrix exponential algorithm.

---

### 5.4 `analysis` — System Analysis

#### `Stability.hpp`

```cpp
namespace csl::analysis {

bool isStable(const SystemModel& sys);
bool isMarginallyStable(const SystemModel& sys);

// Routh-Hurwitz: builds the Routh array from a Polynomial's coefficients and
// counts sign changes in the first column. Pure control theory — no Eigen needed.
int    routhHurwitz(const Polynomial& char_poly);
Matrix routhArray(const Polynomial& char_poly);
bool   routhIsStable(const Polynomial& char_poly);

// Lyapunov stability: A is stable iff solveLyapunov(A, Q) is positive definite
// for any Q > 0. Positive-definiteness check uses Eigen (A.llt().info() == Eigen::Success).
bool isLyapunovStable(const Matrix& A);

struct StabilityMargins {
    Real gain_margin_dB, gain_margin_linear, gain_crossover_freq;
    Real phase_margin_deg, phase_crossover_freq, delay_margin;
    bool is_stable;
};
StabilityMargins margins(const SystemModel& open_loop);

// H2/H-infinity norms — computed via Lyapunov equation and Hamiltonian-matrix
// bisection respectively, both hand-written control-theory techniques.
Real H2norm(const StateSpace& sys);      // sqrt(trace(B'*Wo*B)), Wo = observabilityGramian
Real HInfNorm(const StateSpace& sys, Real tol = 1e-4);

} // namespace csl::analysis
```

#### `FrequencyResponse.hpp`

```cpp
namespace csl::analysis {

struct BodeData    { Vector frequencies, magnitude_dB, magnitude, phase_deg, phase_rad; };
struct NyquistData { Vector frequencies, real_part, imag_part; };

// Analytic evaluation of H(jw) at each frequency — exact, since we have the
// rational transfer function / state-space model in closed form. (Contrast
// with csl::spectral::fft, used only when we have sampled data instead of a model.)
BodeData    bode(const SystemModel& sys, Real wmin, Real wmax, int npts = 200);
NyquistData nyquist(const SystemModel& sys, const Vector& omega);

Complex freqResponse(const SystemModel& sys, Real omega);

} // namespace csl::analysis
```

#### `TimeResponse.hpp` and `RootLocus.hpp`

Unchanged in API from the original design; `lsim`/`stepResponse`/etc. call the hand-written `csl::simulation` ODE integrators (Section 5.12) rather than any hand-rolled matrix machinery, and `rootLocus` walks a gain sweep computing `TransferFunction::poles()` at each gain (which itself now resolves via `Polynomial::roots()` -> Eigen).

---

### 5.5 `design/classical` — Classical Control

Entirely hand-written, as before — a PID controller, Ziegler-Nichols tuning rule, and lead-lag compensator are control theory through and through, with essentially no generic linear algebra inside them (a PID controller's state is three scalars).

```cpp
namespace csl::design::classical {

class PIDController : public Controller {
public:
    struct Config { Real Kp=1, Ki=0, Kd=0, N=100, output_min=-1e9, output_max=1e9; bool anti_windup=true; };
    explicit PIDController(const Config& cfg);
    PIDController(Real Kp, Real Ki, Real Kd, Real N = 100.0);

    Real compute(Real error, Real dt) override;
    Real operator()(Real error, Real dt) { return compute(error, dt); }
    void reset() override;

    void setGains(Real Kp, Real Ki, Real Kd);
    void setOutputLimits(Real lo, Real hi);
    TransferFunction toTransferFunction() const;

    static PIDController zieglerNicholsClosedLoop(Real Ku, Real Tu);
    static PIDController zieglerNicholsOpenLoop(Real K, Real theta, Real tau);
    static PIDController IMC(const TransferFunction& plant, Real lambda);
private:
    Config cfg_;
    Real integral_ = 0, prev_error_ = 0, deriv_filt_ = 0, last_out_ = 0;
};

class LeadCompensator : public ClassicalController {
public:
    LeadCompensator(Real zero_freq, Real pole_freq, Real gain = 1.0);
    static LeadCompensator designForPhaseMargin(const SystemModel& plant, Real pm_deg, Real wc);
    TransferFunction asTransferFunction() const;
    Real compute(Real error, Real dt) override;
    void reset() override;
private:
    Real z_, p_, K_;
};

class ZieglerNichols {
public:
    struct PIDParams { Real Kp, Ki, Kd; };
    static PIDParams openLoop(Real K, Real L, Real T, const std::string& type = "PID");
    static PIDParams closedLoop(Real Ku, Real Tu, const std::string& type = "PID");
    static std::pair<Real,Real> relayFeedback(std::function<Real(Real,Real)> plant_step,
                                              Real relay_amplitude, Real dt, Real T_max);
    PIDController operator()(const TransferFunction& plant) const;   // functor tuning interface
};

} // namespace csl::design::classical
```

---

### 5.6 `design/modern` — Modern State-Space Control

This is where the "hand-write the control theory" principle matters most. Eigen gives us matrix arithmetic and eigendecomposition; it gives us **nothing** for solving a Riccati equation, because that is not generic linear algebra — it is the mathematical core of optimal control, and deriving/implementing it is the entire point of learning LQR.

```cpp
namespace csl::design::modern {

class PolePlacementController : public StateFeedbackController {
public:
    PolePlacementController(const StateSpace& sys, const std::vector<Complex>& desired_poles);

    // Ackermann's formula: K = e_n^T * Cc^{-1} * p_d(A)
    // Cc = controllability matrix (Eigen for the matrix power/inverse; the
    // formula itself, and why it places poles exactly, is the control theory)
    Matrix ackermannFormula(const Matrix& A, const Matrix& B, const Polynomial& desired_char_poly);
    Matrix gain() const;
};

class LQRController : public StateFeedbackController {
public:
    LQRController(const StateSpace& sys, const Matrix& Q, const Matrix& R);

    struct LQRResult { Matrix K, P; Vector closed_loop_poles; Real cost; };
    LQRResult solve() const;   // calls csl::solveContinuousARE(A,B,Q,R) then K = R^-1 B' P

    static LQRResult solveDLQR(const DiscreteStateSpace& sys, const Matrix& Q, const Matrix& R);
    Matrix gain() const;
private:
    Matrix Q_, R_, P_, K_;
};

class LQGController : public Controller {
public:
    LQGController(const StateSpace& sys, const Matrix& Q_lqr, const Matrix& R_lqr,
                  const Matrix& Q_kf, const Matrix& R_kf);
    Real   compute(Real error, Real dt) override;
    Vector computeVector(const Vector& y, const Vector& y_ref) const;
private:
    std::unique_ptr<LQRController> lqr_;
    std::unique_ptr<estimation::KalmanFilter> kf_;
};

} // namespace csl::design::modern
```

`solveContinuousARE` (declared in `core/LinearAlgebra.hpp`, Section 5.1) is implemented via the **Hamiltonian matrix eigendecomposition method**: form `H = [[A, -B*R^-1*B'], [-Q, -A']]`, compute its eigendecomposition with `Eigen::EigenSolver`, take the stable (negative real part) eigenvectors, and extract `P` from the ratio of their top/bottom blocks. Eigen supplies the eigendecomposition (generic linear algebra); everything else in that recipe is Riccati/control theory and is written by hand, with comments explaining each step — this is one of the highest-value hand-written algorithms in the whole library.

---

### 5.7 `design/robust` — Robust Control

```cpp
namespace csl::design::robust {

struct HInfResult { StateSpace controller; Real gamma_optimal; bool converged; int iterations; };

// Gamma-iteration: bisection search (csl::bisection, from NumericalMethods.hpp)
// over the performance level gamma; at each gamma, two Riccati equations
// (via solveContinuousARE) determine whether a stabilizing controller exists.
HInfResult synthesizeHInf(const StateSpace& generalized_plant,
                          int n_measurements, int n_controls,
                          Real gamma_init = 1.0, Real tol = 1e-6);

HInfResult mixedSensitivity(const StateSpace& plant, const TransferFunction& Ws,
                            const TransferFunction& Wt, const TransferFunction& Wks);

} // namespace csl::design::robust
```

---

### 5.8 `optimal` — Optimal Control

```cpp
namespace csl::optimal {

struct MPCConfig {
    int Np = 10, Nu = 3;
    Matrix Q, R, QN;
    Vector u_min, u_max, du_min, du_max, y_min, y_max;
    bool warm_start = true;
};

class MPCController : public Controller {
public:
    MPCController(const DiscreteStateSpace& model, const MPCConfig& cfg);
    Real   compute(Real error, Real dt) override;
    Vector computeVector(const Vector& y_ref, const Vector& x_current) const;

    // Prediction/control matrices are built with Eigen block operations
    // (Phi, Gamma condensed formulation); the QP itself is handed to
    // csl::solveQP (NumericalMethods.hpp) — the hand-written active-set solver.
    Matrix buildPredictionMatrix() const;   // Phi
    Matrix buildControlMatrix() const;      // Gamma
private:
    DiscreteStateSpace model_;
    MPCConfig cfg_;
};

using DynamicsFunc = std::function<Vector(const Vector& x, const Vector& u)>;
using CostFunc     = std::function<Real(const Vector& x, const Vector& u)>;

struct iLQRResult { std::vector<Vector> x_traj, u_traj; Real total_cost; int iterations; bool converged; };
iLQRResult iLQR(DynamicsFunc f, CostFunc l, std::function<Real(const Vector&)> lN,
                const Vector& x0, const std::vector<Vector>& u_init,
                int max_iter = 100, Real tol = 1e-6);

struct MDP { int S, A; Matrix T, R; Real gamma; };
Vector valueIteration(const MDP& mdp, Real tol = 1e-6, int maxIter = 1000);
Vector policyIteration(const MDP& mdp);

} // namespace csl::optimal
```

---

### 5.9 `estimation` — State Estimation

The Kalman filter family is the clearest example in the whole library of "generic math from Eigen, control/estimation theory by hand": the predict/update recursion, the innovation covariance, the Kalman gain formula, the sigma-point transform (UKF), and the resampling step (particle filter) are all estimation theory. Eigen supplies the matrix inverse/multiply inside them.

```cpp
namespace csl::estimation {

class KalmanFilter : public Estimator {
public:
    KalmanFilter(const DiscreteStateSpace& sys, const Matrix& Q, const Matrix& R);

    void predict(const Vector& u) override;   // x = A*x + B*u ;  P = A*P*A' + Q
    void update(const Vector& y)  override;   // K = P*H'*(H*P*H'+R)^-1 ; x += K*(y-H*x) ; P = (I-K*H)*P

    Vector getState()      const override { return x_; }
    Matrix getCovariance() const override { return P_; }
    Matrix getKalmanGain()  const { return K_; }

    static Matrix steadyStateGain(const DiscreteStateSpace& sys, const Matrix& Q, const Matrix& R);
    // steady-state gain solves csl::solveDiscreteARE — same Riccati machinery as LQR,
    // which is precisely the duality between optimal control and optimal estimation.
protected:
    DiscreteStateSpace sys_;
    Vector x_;
    Matrix P_, Q_, R_, K_;
};

class ExtendedKalmanFilter : public KalmanFilter {
public:
    using DynFunc = std::function<Vector(const Vector&, const Vector&)>;
    using OutFunc = std::function<Vector(const Vector&)>;
    using JacFunc = std::function<Matrix(const Vector&, const Vector&)>;   // optional: falls back
                                                                            // to csl::numericalJacobian
    struct Config { DynFunc f; OutFunc h; JacFunc Jf, Jh; Matrix Q, R; };
    explicit ExtendedKalmanFilter(const Config& cfg, int n, int p);
    void predict(const Vector& u) override;
    void update(const Vector& y)  override;
};

class UnscentedKalmanFilter : public KalmanFilter {
public:
    struct Config {
        std::function<Vector(const Vector&, const Vector&)> f;
        std::function<Vector(const Vector&)> h;
        Matrix Q, R;
        Real alpha = 1e-3, beta = 2.0, kappa = 0.0;
    };
    explicit UnscentedKalmanFilter(const Config& cfg, int n, int p);
    void predict(const Vector& u) override;   // unscented transform through f
    void update(const Vector& y)  override;   // unscented transform through h
private:
    void generateSigmaPoints(const Vector& x, const Matrix& P,
                             Matrix& sigma, Vector& wm, Vector& wc) const;   // uses P.llt() (Eigen Cholesky)
};

class ParticleFilter : public Estimator {
public:
    ParticleFilter(int n_particles, DynFunc f, OutFunc h, Real process_std, Real meas_std);
    void predict(const Vector& u) override;   // propagate + add process noise (std::normal_distribution)
    void update(const Vector& y)  override;   // importance weighting + systematic resampling
private:
    int N_;
    std::vector<Vector> particles_;
    std::vector<Real>   weights_;
    std::mt19937         rng_;
};

} // namespace csl::estimation
```

---

### 5.10 `filters` — Filter Design

Filter **design** — deriving the pole locations of a Butterworth prototype, the ripple equations of a Chebyshev filter, the elliptic rational functions, the Kaiser window beta parameter — is DSP/control domain knowledge and stays hand-written. The linear-algebra grunt work inside (finding roots of the resulting polynomial, matrix operations for state-space realizations) is Eigen.

```cpp
namespace csl::filters {

class ButterworthFilter : public AnalogFilter {
public:
    ButterworthFilter(int order, Real cutoff_rad, FilterType type = FilterType::Lowpass);
    static ZeroPoleGain analogPrototype(int n);   // poles at exp(j*pi*(2k+n-1)/(2n)), hand-derived
    static ZeroPoleGain frequencyTransform(const ZeroPoleGain& lp, FilterType type, Real wc, Real bw = 0);
    IIRFilter digitize(Real fs, DiscretizationMethod m = DiscretizationMethod::Tustin) const;
    TransferFunction toTransferFunction() const override;
    Real process(Real input) override;
};

class ChebyshevFilter : public AnalogFilter {
public:
    enum class ChebyType { TypeI, TypeII };
    ChebyshevFilter(int order, Real cutoff_rad, Real ripple_dB, ChebyType type = ChebyType::TypeI);
    TransferFunction toTransferFunction() const override;
    Real process(Real input) override;
};

class EllipticFilter  : public AnalogFilter { /* passband ripple + stopband attenuation spec */ };
class BesselFilter    : public AnalogFilter { /* maximally-flat group delay; uses besselPolyCoeff() */ };

class FIRFilter : public DigitalFilter {
public:
    FIRFilter(Vector coefficients, Real sample_rate);
    static FIRFilter lowpass(int order, Real cutoff_norm, WindowType w = WindowType::Hamming);
    static FIRFilter kaiser(Real passband_ripple_dB, Real stopband_atten_dB, Real transition_width_norm);
    Real process(Real input) override;   // convolution via Eigen dot product against a sliding window
    Vector impulseResponse() const;
private:
    Vector coeffs_;
    Eigen::VectorXd delay_line_;   // ring-buffered via csl::simulation::RingBuffer<Real>
};

class IIRFilter : public DigitalFilter {
public:
    IIRFilter(Vector b, Vector a);   // numerator/denominator coefficients
    Real process(Real input) override;   // Direct Form II Transposed
    std::vector<std::pair<Vector,Vector>> toSOS() const;   // second-order sections for numerical stability
};

class NotchFilter : public DigitalFilter {
public:
    NotchFilter(Real notch_freq_hz, Real bandwidth_hz, Real sample_rate);   // biquad coefficients, hand-derived
    Real process(Real x) override;
};

} // namespace csl::filters
```

---

### 5.11 `sysid` — System Identification

```cpp
namespace csl::sysid {

class LeastSquaresID : public SystemIdentifier {
public:
    LeastSquaresID(int na, int nb);
    TransferFunction identify(const Vector& y, const Vector& u) override;
    // Builds the regression matrix Phi (control-specific: rows are lagged
    // y/u samples per the ARX model structure), then solves
    // theta = Phi.colPivHouseholderQr().solve(y) — Eigen does the solve.
};

class RecursiveLeastSquaresID : public SystemIdentifier {
public:
    struct Config { int na, nb; Real lambda = 0.99; Real P0 = 1e6; };
    explicit RecursiveLeastSquaresID(const Config& cfg);
    void   update(Real y_new, Real u_new);   // online recursive update, forgetting factor lambda
    Vector getTheta() const;
    TransferFunction identifiedModel() const;
private:
    Vector theta_, phi_;
    Matrix P_;
};

class ARMAXModel : public LeastSquaresID {
public:
    ARMAXModel(int na, int nb, int nc);   // A(q)y = B(q)u + C(q)e
    TransferFunction identify(const Vector& y, const Vector& u) override;   // extended LS / IV
};

class SubspaceID : public SystemIdentifier {
public:
    SubspaceID(int block_rows_i, int state_order_n);
    StateSpace identify(const Vector& y, const Vector& u) override;
    // N4SID: builds block Hankel matrices (control-specific structure), then
    // an oblique projection via Eigen's SVD (A.bdcSvd(...)) recovers the
    // observability matrix, from which A and C are extracted algebraically.
};

class FrequencyDomainID : public SystemIdentifier {
public:
    TransferFunction identify(const Vector& y, const Vector& u) override;
    // Uses csl::spectral::empiricalTransferFunction(u, y, fs) to get H(jw)
    // from FFTs of the measured signals, then fits rational TF coefficients
    // via Levy's method (a weighted linear least squares in the frequency
    // domain, solved with Eigen).
};

} // namespace csl::sysid
```

---

### 5.12 `simulation` — Simulation Engine

ODE integration remains fully hand-written per the project's explicit choice: understanding how Euler/RK4/RK45 actually integrate the dynamics is core to understanding simulation, in the same way understanding the Kalman recursion is core to understanding estimation.

```cpp
namespace csl::simulation {

using DynamicsFunc = std::function<Vector(Real t, const Vector& x)>;

Vector eulerStep(DynamicsFunc f, Real t, const Vector& x, Real dt);
Vector rk4Step(DynamicsFunc f, Real t, const Vector& x, Real dt);

struct RK45Result { Vector x_next; Real dt_next, error_estimate; };
RK45Result rk45Step(DynamicsFunc f, Real t, const Vector& x, Real dt,
                    Real tol_abs = 1e-6, Real tol_rel = 1e-6);   // Dormand-Prince adaptive step

struct IntegrateResult { Vector t; std::vector<Vector> x; };
IntegrateResult integrate(DynamicsFunc f, const Vector& x0, Real t0, Real tf, Real dt,
                         const std::string& method = "RK4");

template<typename T>
class RingBuffer {   // unchanged: thin wrapper over std::deque<T>
public:
    explicit RingBuffer(std::size_t capacity) : cap_(capacity) {}
    void push(const T& v) { if (buf_.size()==cap_) buf_.pop_front(); buf_.push_back(v); }
private:
    std::size_t cap_;
    std::deque<T> buf_;
};

struct SimulationResult { Vector time; Matrix outputs, states, inputs; void writeCSV(const std::string&) const; };

class Simulator {
public:
    Simulator(std::weak_ptr<systems::StateSpace> plant, std::weak_ptr<Controller> controller,
              Real dt, Real T);
    SimulationResult run(const Vector& x0, const Vector& y_ref);
    void addMeasurementNoise(Real std_dev);   // std::normal_distribution
};

struct MonteCarloConfig { int n_runs = 100; Real process_noise_std = 0.01, measure_noise_std = 0.01; };
struct MonteCarloResult { std::vector<SimulationResult> runs; Vector mean_output, std_output; };
MonteCarloResult monteCarlo(std::shared_ptr<systems::StateSpace> plant,
                            std::shared_ptr<Controller> controller,
                            const Vector& x0, const Vector& y_ref, const MonteCarloConfig& cfg);

} // namespace csl::simulation
```

---

### 5.13 `signals` — Signal Generation

Almost entirely `<random>` and simple closed-form formulas; the one control/DSP-specific piece is the PRBS generator (linear-feedback shift register), kept hand-written because understanding how a maximal-length sequence is generated matters for system-identification experiment design.

```cpp
namespace csl::signals {

Real stepSignal(Real t, Real amplitude = 1.0, Real t_start = 0.0);
Real sinusoid(Real t, Real amplitude, Real freq_hz, Real phase_rad = 0.0);
Real chirpSignal(Real t, Real f0, Real f1, Real T, Real amplitude = 1.0);

class WhiteNoise {
public:
    explicit WhiteNoise(Real std_dev = 1.0, unsigned seed = std::random_device{}());
    Real   sample() { return dist_(rng_); }
    Vector sampleVector(Index n);
private:
    std::mt19937 rng_;
    std::normal_distribution<Real> dist_;
};

// Pseudo-Random Binary Sequence via linear-feedback shift register — hand-written
// because the LFSR construction (and why it gives a maximal-length sequence with
// a flat spectrum, ideal for system ID excitation) is signal-design domain knowledge.
class PRBS {
public:
    explicit PRBS(int register_length = 10, Real amplitude = 1.0);
    Real next();
    Vector generate(Index n);
private:
    int len_; Real amp_; uint32_t state_;
};

} // namespace csl::signals
```

---

## 6. Build System

### `CMakeLists.txt` (root)

```cmake
cmake_minimum_required(VERSION 3.20)
project(controlSystemCPPLibrary VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Performance flags for Release builds
if(MSVC)
    add_compile_options(/W4 /O2 /arch:AVX2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic
        $<$<CONFIG:Release>:-O3 -march=native>)
endif()

# --- Eigen: header-only, fetched once, no compiled artifact to link ---
include(FetchContent)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen3)

add_library(csl STATIC)
add_library(csl::csl ALIAS csl)

target_include_directories(csl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(csl PUBLIC Eigen3::Eigen)

add_subdirectory(src)

option(CSL_BUILD_TESTS "Build unit tests" ON)
if(CSL_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/v1.14.0.zip)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    add_subdirectory(tests)
endif()

option(CSL_BUILD_EXAMPLES "Build examples" ON)
if(CSL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
```

**Optional: BLAS/LAPACK backend for Eigen.** For very large matrices (state dimension in the hundreds+, e.g. large-scale MPC or subspace identification with long data records), Eigen can transparently delegate its dense linear algebra to a highly-tuned BLAS/LAPACK implementation:

```cmake
option(CSL_USE_MKL "Use Intel MKL as Eigen's BLAS/LAPACK backend" OFF)
if(CSL_USE_MKL)
    find_package(MKL REQUIRED)
    target_compile_definitions(csl PUBLIC EIGEN_USE_MKL_ALL)
    target_link_libraries(csl PUBLIC MKL::MKL)
endif()
```

This is off by default (Eigen's own vectorized kernels are already excellent for the matrix sizes typical in control systems — state dimensions of 2 to ~50), but the switch exists because "extremely efficient" was an explicit goal, and this is how you get BLAS-level performance without writing a single extra line of application code.

### Build Commands

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
cd build && ctest --output-on-failure
```

---

## 7. Testing Strategy

Test organization mirrors the original design (Google Test, one file per class/algorithm, ~200+ tests by Day 40). Two changes:

1. **No tests for Eigen's own correctness.** We don't test that `A.inverse()` is right — Eigen is tested far more rigorously than this project could manage. Tests focus entirely on the hand-written control-theory layer: does `solveContinuousARE` produce a `K` that actually stabilizes the closed loop? Does `routhHurwitz` correctly classify known stable/unstable polynomials? Does the FFT match a known DFT for small test vectors? Does the Kalman filter converge to the right steady-state covariance?
2. **Numerical tolerance policy is unchanged**: `1e-9` for closed-form results, `1e-6` for iterative algorithms (Riccati solves, QP), `1e-4` for algorithms with many compounding iterations (MPC receding horizon, particle filter).

Reference test cases (updated): FFT of a length-8 signal checked against a hand-computed DFT; `solveContinuousARE` on a double integrator checked against the textbook closed-form `K = [1, sqrt(3)]`; Butterworth pole locations checked against the analytic formula; `routhHurwitz` on `s^3+2s^2+3s+4` (unstable) and `s^3+2s^2+3s+10` (stable, textbook example).

---

## 8. Design Principles: Build vs. Buy

This section is the expanded reasoning behind the table in Section 1.2. For every category, the question asked was: **"does writing this by hand teach me something about control systems (or DSP, as directly requested), or is it purely generic numerical/software infrastructure that a mature library already solves better?"**

| Category | Decision | Reasoning |
|---|---|---|
| Dense matrices/vectors, arithmetic, `.transpose()`, `.norm()` | **Eigen** | Pure data structure + generic linear algebra. Eigen's expression templates avoid temporaries; a hand-rolled class would be slower and less safe. |
| LU / QR / SVD / Cholesky / eigendecomposition | **Eigen** | Numerically subtle generic algorithms (pivoting strategies, convergence criteria) with decades of hardening. Not control-theory-specific. |
| Matrix exponential/log/sqrt | **Eigen** (`unsupported/MatrixFunctions`) | Same reasoning; used as a building block inside control-specific code (ZOH discretization). |
| Complex numbers | **`std::complex`** | Standardized, correct, interoperates with Eigen's complex matrix types out of the box. |
| Basic math (`pow`, `exp`, `sqrt`, trig) | **`<cmath>`** | Exactly the "don't reinvent exponentiation" case named explicitly. |
| Random distributions | **`<random>`** | Statistically correct engines/distributions; reinventing a Mersenne Twister teaches nothing about control. |
| Containers/algorithms (`sort`, `transform`, `accumulate`) | **STL** | Generic software engineering, not control theory. |
| Transfer function / state-space / ZPK representations & algebra | **Hand-written** | This literally *is* control systems — how these representations relate to each other is the subject matter. |
| Stability criteria (Routh-Hurwitz, Lyapunov, margins) | **Hand-written** | Domain-specific tabular/analytic algorithms with no generic-library equivalent. |
| Riccati equation solvers (continuous/discrete ARE) | **Hand-written** | No mainstream generic library ships a Riccati solver — because it isn't generic math, it's optimal control theory. This is arguably the single most valuable hand-written algorithm in the library. |
| Lyapunov equation solvers | **Hand-written** (using Eigen's Schur decomposition internally) | Same reasoning; the Bartels-Stewart *algorithm* is control/numerical-linear-algebra crossover, but assembling and applying it to A, Q is the control-theory step. |
| Controller design (PID, lead-lag, pole placement, LQR, LQG, H-infinity) | **Hand-written** | The entire subject matter of classical and modern control. |
| State estimators (KF, EKF, UKF, particle filter) | **Hand-written** | The entire subject matter of estimation theory. |
| Filter *design* (Butterworth/Chebyshev/Elliptic/Bessel coefficient derivation, FIR windowing) | **Hand-written** | DSP domain knowledge — deriving pole locations and window functions is the learning objective. |
| System identification (RLS, ARMAX, N4SID, frequency-domain) | **Hand-written** | The entire subject matter of system identification. |
| Fourier transform (FFT/IFFT) | **Hand-written**, per explicit request | Frequency-domain reasoning is central to how controls engineers think; implementing Cooley-Tukey is treated as core curriculum, not infrastructure. |
| ODE integration (Euler/RK4/RK45) | **Hand-written** | Simulating a dynamical system is core control-systems skill; understanding what an integrator does (and its error/stability trade-offs) matters directly for later work (e.g., choosing MPC sample times). |
| QP solver for MPC | **Hand-written** active-set method | Understanding the optimization problem MPC solves at every timestep is the point of learning MPC; the codebase is structured so this can later be swapped for OSQP/qpOASES with zero change to the MPC formulation code (Section 11). |
| Logging, CSV I/O, exceptions, named registry | **Hand-written** | Small, general infrastructure with no numerically-intensive content — there was never a "buy" option that made sense here, and it's useful C++ practice regardless. |

---

## 9. 40-Day Learning and Building Plan

The 40-day, chapter-by-chapter cadence is unchanged. What changes is *what gets built* in Weeks 1-3: instead of spending three weeks hand-rolling `Matrix`/`Vector`/decompositions, those weeks now spend Day 1 wiring in Eigen and then spend the freed-up time on two things that stayed hand-written by design — the `Polynomial`/`TransferFunction` control representations, and the **FFT**, whose iterative bit-reversal-plus-butterflies structure happens to map beautifully onto exactly the C++ statements/functions/classes material Chapters 5-7 teach. Weeks 4-8 are structurally the same as before; their code tasks now call Eigen methods (`.transpose()`, `.inverse()`, `Eigen::EigenSolver`, `.exp()`) instead of hand-rolled equivalents, and two days gain explicit FFT-consumer tasks (spectral analysis in Week 8, frequency-domain system ID).

---

### Week 1 — C++ Basics, Eigen Integration, and FFT Foundations (Ch. 1-3)

**Goal by end of week:** Buildable project with Eigen wired in via CMake, `csl::Types.hpp` and `csl::MathUtils.hpp` complete, and the skeleton of `csl::spectral::FFT` (data layout + bit-reversal permutation) in place, ready for the actual butterfly algorithm in Week 2.

---

#### Day 1 — Chapter 1: Getting Started

**Read:** All of Chapter 1 (~26 pages) — compiling/linking, `#include`, `main()`, `std::cout`/`std::cin`, namespaces.

**Exercises:** 1.1, 1.2, 1.3, 1.9, 1.10, 1.11, 1.20, 1.21, 1.22, 1.23, 1.25

**Library Coding Task:**
Create the project skeleton **and** wire in Eigen from day one via CMake `FetchContent` (Section 6). Write `src/main.cpp`:

```cpp
#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout << "Control Systems Library (csl) v0.2.0\n";
    std::cout << "Built with C++17, backed by Eigen " << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << "\n";

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    std::cout << "3x3 identity via Eigen:\n" << I << "\n";
    return 0;
}
```

**Deliverable:** `cmake -B build && cmake --build build` succeeds, fetching Eigen automatically, and the executable prints the identity matrix. This proves the whole "build vs. buy" foundation works before any control-systems code is written.

---

#### Day 2 — Chapter 2 Part 1: Primitive Types, Variables, const

**Read:** §2.1-2.3 (~45 pages) — fundamental types, variables, `const`, references, pointers.

**Exercises:** 2.1, 2.3, 2.5, 2.6, 2.9-2.19, 2.21, 2.22

**Library Coding Task:**
Write `include/csl/core/Types.hpp` in full (Section 5.1): `csl::Real`, `csl::Matrix`/`csl::Vector` as Eigen aliases, `csl::Complex` as `std::complex<Real>`, all constants and enums. This day is a natural home for `const`: every constant in the file is `inline constexpr`, and the exercise of choosing `Eigen::MatrixXd` vs. a templated `Eigen::Matrix<Real, Dynamic, Dynamic>` is a direct, concrete application of "what is a type alias and why would I want one."

**Deliverable:** `Types.hpp` compiles standalone; a tiny test driver constructs a `csl::Vector`, a `csl::Complex`, and prints `csl::PI`.

---

#### Day 3 — Chapter 2 Part 2: auto, decltype, References Continued

**Read:** §2.4-2.6 (~25 pages) — const references, `auto`, `decltype`, type aliases, structs, header guards.

**Exercises:** 2.26-2.42

**Library Coding Task:**
Write `include/csl/core/MathUtils.hpp` in full (Section 5.1): `dB`/`fromDB`, `wrapAngle`, `saturate`/`deadband`, `besselPolyCoeff`. Use `auto` and `decltype` where they genuinely clarify (e.g. `auto result = std::clamp(v, -limit, limit);`), and practice passing Eigen types by `const&` — a `const Eigen::VectorXd&` parameter is the idiomatic Eigen convention and a perfect real-world example of exactly what this section teaches about reference binding.

**Deliverable:** `MathUtils.hpp` compiles; every function exercised against a known input/output pair.

---

#### Day 4 — Chapter 3 Part 1: string, vector, Iterators

**Read:** §3.1-3.4 (~55 pages) — `std::string`, `std::vector`, iterators, range-for.

**Exercises:** 3.2-3.25

**Library Coding Task:**
Two things, both genuinely useful (no busywork Vector/Matrix class to build anymore):
1. `include/csl/core/Logger.hpp` skeleton — a legitimate, immediately useful `std::string`-heavy class (log level prefix, timestamp formatting, message concatenation) that exercises exactly what the chapter teaches.
2. Confirm `Eigen::VectorXd` iterator support (`.begin()`/`.end()`, available since Eigen 3.4) by writing a range-for loop over an `Eigen::VectorXd` and a `std::accumulate` call against it — this is the moment to *see* that Eigen vectors already behave like STL containers, reinforcing why a hand-rolled `Vector` class would have been redundant.

**Deliverable:** `Logger` skeleton compiles and logs a few `std::string` messages; a short demo shows range-for and `std::accumulate` both working directly on an `Eigen::VectorXd`.

---

#### Day 5 — Chapter 3 Part 2: Arrays, Multidimensional Arrays

**Read:** §3.5-3.6 (~25 pages) — C arrays, pointer decay, multidimensional arrays.

**Exercises:** 3.27-3.45

**Library Coding Task:**
This chapter's material — raw arrays, contiguous memory, index arithmetic — is the perfect lead-in to **starting the FFT**, since the FFT's classic bit-reversal permutation is usually taught with raw index arrays. Begin `include/csl/spectral/FFT.hpp`:

```cpp
#pragma once
#include "csl/core/Types.hpp"

namespace csl::spectral {

// Declared today; butterfly stages implemented Day 7 once loops are covered.
void fft(ComplexVector& x);
void ifft(ComplexVector& x);

// Bit-reversal permutation: for index i in [0, N), compute the integer whose
// bits are i's bits reversed (within log2(N) bits). Written today using a
// raw fixed-size scratch array to directly apply this chapter's material.
std::size_t bitReverse(std::size_t i, int numBits);

} // namespace csl::spectral
```

Implement `bitReverse` using explicit bit-shift/mask operations over a small local array of bits — deliberately low-level, matching the chapter. Also add a `static_assert`-guarded helper `isPowerOfTwo(std::size_t n)`.

**Week 1 Review:** Eigen is wired into the build; `Types.hpp`/`MathUtils.hpp` are complete; the FFT header exists with its permutation helper ready. Nothing about Week 1 required writing a matrix class — the time went straight into control/DSP-relevant code.

**Deliverable:** `bitReverse(0b011, 3) == 0b110` (and similar) verified by hand in a small test driver.

---

### Week 2 — Expressions, Statements, Functions + the FFT and Control-Specific Linear Algebra (Ch. 4-6)

**Goal by end of week:** A complete, working radix-2 FFT/IFFT; `Polynomial` class with Eigen-backed root finding; `csl::LinearAlgebra.hpp`'s control-specific equations (Riccati/Lyapunov) stubbed with correct signatures; ODE solver skeleton.

---

#### Day 6 — Chapter 4: Expressions

**Read:** All of Chapter 4 (~60 pages) — operator precedence, bitwise operators, conditional operator, `sizeof`, casts.

**Exercises:** 4.1-4.37

**Library Coding Task:**
This chapter's bitwise-operator material is exactly what the FFT's twiddle-factor index computation and the bit-reversal permutation need. Implement `bitReverse` fully (using `>>`, `<<`, `&`, `|`) and write the **twiddle factor table generator**:

```cpp
namespace csl::spectral {
// W_N^k = exp(-2*pi*i*k/N) — the FFT's "twiddle factors." Precomputed once
// per transform size using std::complex + csl::TWO_PI (no reinvented trig).
std::vector<Complex> twiddleFactors(std::size_t N);
}
```

Use the conditional operator and bitwise ops liberally where they clarify (e.g., checking `N & (N-1)) == 0` to verify a power-of-two size — the canonical bit trick this chapter is made for).

**Deliverable:** `twiddleFactors(8)` matches hand-computed values for `k = 0..7`.

---

#### Day 7 — Chapter 5: Statements

**Read:** All of Chapter 5 (~35 pages) — `if`/`switch`/loops/range-for/`break`/`continue`.

**Exercises:** 5.1-5.25

**Library Coding Task:**
This is **the day the FFT itself gets written** — its iterative structure is a textbook example of nested loops:

```cpp
void fft(ComplexVector& x) {
    const std::size_t N = x.size();
    // ... bit-reversal reorder (Day 5) ...
    for (std::size_t len = 2; len <= N; len <<= 1) {          // stage loop
        Complex w = std::polar(1.0, -TWO_PI / static_cast<double>(len));
        for (std::size_t i = 0; i < N; i += len) {             // block loop
            Complex wn = 1.0;
            for (std::size_t j = 0; j < len / 2; ++j) {        // butterfly loop
                Complex u = x[i + j];
                Complex v = x[i + j + len/2] * wn;
                x[i + j]         = u + v;
                x[i + j + len/2] = u - v;
                wn *= w;
            }
        }
    }
}
```

`ifft` is implemented by conjugating, calling `fft`, conjugating again, and dividing by `N` — a direct, satisfying application of "reuse a function you already wrote" plus the arithmetic/statement material from this chapter and the last.

**Deliverable:** `fft` on a length-8 impulse (`[1,0,0,0,0,0,0,0]`) returns all-ones (the defining property of an impulse's spectrum); `ifft(fft(x)) == x` to within `1e-10` for a random test vector.

---

#### Day 8 — Chapter 6 Part 1: Function Basics, Argument Passing, Return

**Read:** §6.1-6.3 (~40 pages) — parameter passing (value/ref/const-ref/pointer), return by value/reference.

**Exercises:** 6.1-6.32

**Library Coding Task:**
Wrap the FFT in the convenience API from Section 5.1b, applying today's material directly: `fft(const Vector& x)` takes its argument by `const&` (no unnecessary copy of potentially large signal data), zero-pads to the next power of two, and returns a new `ComplexVector` by value (cheap — Eigen's move semantics make this free). Implement `fftMagnitude` and stub `welchPSD`/`empiricalTransferFunction` signatures (bodies completed Week 8, once the sysid module needs them).

**Deliverable:** `fft(Vector)` correctly zero-pads a length-100 vector to 128 and returns a length-128 spectrum.

---

#### Day 9 — Chapter 6 Part 2: Overloading, Default Arguments, constexpr

**Read:** §6.4-6.6 (~35 pages) — overloading, default arguments, `inline`, `constexpr` functions.

**Exercises:** 6.33-6.56

**Library Coding Task:**
Write `include/csl/core/Polynomial.hpp` in full (Section 5.1): overloaded arithmetic operators, `constexpr`-eligible `operator()` via Horner's method, and `roots()` built on the companion matrix + `Eigen::EigenSolver` — this is the day the "Eigen does generic eigenvalue math, we do the control-representation part" pattern first shows up concretely:

```cpp
std::vector<Complex> Polynomial::roots() const {
    int n = degree();
    if (n == 0) return {};
    Matrix C = Matrix::Zero(n, n);                 // companion matrix — our construction
    for (int i = 1; i < n; ++i) C(i, i-1) = 1.0;
    for (int i = 0; i < n; ++i) C(0, i) = -coeffs_[i+1] / coeffs_[0];
    Eigen::EigenSolver<Matrix> solver(C);           // Eigen does the eigenvalue math
    std::vector<Complex> result(n);
    for (int i = 0; i < n; ++i) result[i] = solver.eigenvalues()[i];
    return result;
}
```

**Deliverable:** `Polynomial({1, 0, -1}).roots()` returns `{+1, -1}` (roots of `x^2 - 1`).

---

#### Day 10 — Chapter 6 Part 3: Function Pointers + Week Review

**Read:** §6.7 (~10 pages) + review Ch. 4-6.

**Exercises:** 6.58, 6.59 + revisit skipped exercises

**Library Coding Task:**
Begin `include/csl/simulation/ODESolver.hpp` using `std::function` (the modern, safe evolution of a raw function pointer — worth calling out explicitly as today's lesson applied with a C++17 idiom rather than a bare function pointer):

```cpp
namespace csl::simulation {
using DynamicsFunc = std::function<Vector(Real t, const Vector& x)>;
Vector eulerStep(DynamicsFunc f, Real t, const Vector& x, Real dt);
Vector rk4Step(DynamicsFunc f, Real t, const Vector& x, Real dt);
}
```

Also stub `csl::LinearAlgebra.hpp`'s signatures (Section 5.1) — `solveContinuousARE`, `solveLyapunov`, `controllabilityMatrix`, etc. — with `throw NotImplemented(...)` bodies for now; they get real implementations in Weeks 3 and 7 once eigendecomposition-based derivations are covered.

**Week 2 Review:** Run the FFT round-trip test, `Polynomial::roots()` test, and confirm `eulerStep`/`rk4Step` integrate `x_dot = -x` to within 0.1% of `e^{-t}` at `t=5`.

**Deliverable:** Fully working, tested FFT/IFFT; `Polynomial` with Eigen-backed roots; ODE solver skeleton.

---

### Week 3 — Classes + TransferFunction/StateSpace + Riccati/Lyapunov (Ch. 7)

**Goal by end of week:** `TransferFunction` and `StateSpace` fully implemented on top of Eigen; `solveContinuousARE` and `solveLyapunov` — the two most important hand-written control-theory algorithms in the core module — working and tested.

---

#### Day 11 — Chapter 7 Part 1: Class Basics, Constructors, this

**Read:** §7.1-7.2 (~40 pages) — member functions, constructors, member initializer lists, delegating constructors, `const` member functions.

**Exercises:** 7.1-7.20

**Library Coding Task:**
Build `include/csl/systems/TransferFunction.hpp` (Section 5.2) with proper constructors (delegating to a canonical `TransferFunction(Polynomial, Polynomial)`), const-correct accessors, and `isStable()`/`dcGain()`/`evaluate()` built directly on `Polynomial::roots()` from last week.

**Deliverable:** `TransferFunction({1},{1,1}).isStable() == true`, `.dcGain() == 1.0`, `.poles() == {-1}`.

---

#### Day 12 — Chapter 7 Part 2: Access Control, Friends, Mutable

**Read:** §7.3-7.4 (~25 pages) — access specifiers, `friend`, `mutable`, class scope.

**Exercises:** 7.23-7.41

**Library Coding Task:**
Add `friend` stream operators to `TransferFunction` and `Polynomial` (readable `num(s)/den(s)` formatting). Begin `include/csl/systems/StateSpace.hpp` (Section 5.2) with `A_, B_, C_, D_` as `Eigen::MatrixXd` members — note how little boilerplate is needed now: no custom copy/move control to write (Eigen's types already have correct, efficient copy/move semantics), so this day's real content is the `friend` operators and the constructor validating dimension consistency (throwing `DimensionMismatch` from `core/Exceptions.hpp`).

**Deliverable:** `StateSpace` constructs, validates dimensions, and `std::cout << ss` prints `A`, `B`, `C`, `D` legibly.

---

#### Day 13 — Chapter 7 Part 3: Static Members, Nested Classes

**Read:** §7.5-7.6 (~25 pages) — static data/member functions, aggregate/literal classes.

**Exercises:** 7.42-7.58

**Library Coding Task:**
Implement `poles()`, `isControllable()`, `isObservable()` on `StateSpace`, all delegating to `core/LinearAlgebra.hpp` free functions:

```cpp
std::vector<Complex> StateSpace::poles() const {
    Eigen::EigenSolver<Matrix> solver(A_);       // Eigen: generic eigendecomposition
    std::vector<Complex> result(A_.rows());
    for (int i = 0; i < A_.rows(); ++i) result[i] = solver.eigenvalues()[i];
    return result;                                // control-theory meaning: these ARE the poles
}
bool StateSpace::isControllable(Real tol) const { return csl::isControllable(A_, B_, tol); }
```

Implement `csl::controllabilityMatrix`/`isControllable` fully now (they only need `Matrix` products and `Eigen::FullPivLU::rank()` — no eigendecomposition needed, so this is a reasonable day to complete them, ahead of the harder Riccati/Lyapunov work tomorrow). Add a `static` instance counter to `SystemRegistry` (Section 5.1) as this chapter's static-member exercise, applied somewhere genuinely useful.

**Deliverable:** `isControllable`/`isObservable` verified against a textbook double-integrator example (controllable) and a known uncontrollable pair.

---

#### Day 14 — Library Integration Day: Riccati and Lyapunov

**Read:** None — implementation-focused day, but this is worth a short side-read on the Hamiltonian-matrix method for solving the algebraic Riccati equation (not from the book — this is genuinely new control-theory material, exactly the kind the plan is designed to make room for).

**Library Coding Task:**
Implement the two most important functions in `core/LinearAlgebra.hpp`:

```cpp
Matrix solveContinuousARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R) {
    int n = A.rows();
    Matrix H(2*n, 2*n);
    H << A,                    -B * R.inverse() * B.transpose(),
        -Q,                    -A.transpose();
    Eigen::EigenSolver<Matrix> solver(H);          // Eigen: generic eigendecomposition of H
    // Extract the n eigenvectors whose eigenvalues have negative real part
    // (the "stable subspace") and split each into its top/bottom n-blocks;
    // P = Vbottom * Vtop^{-1}. This assembly IS the control theory.
    // ... (full derivation and numerically-robust variant documented inline)
}

Matrix solveLyapunov(const Matrix& A, const Matrix& Q) {
    Eigen::RealSchur<Matrix> schur(A);             // Eigen: generic Schur decomposition
    // Bartels-Stewart: transform to Schur form, solve the resulting
    // quasi-triangular system by back-substitution, transform back.
}
```

Write `examples/ex01_eigen_and_polynomial.cpp` demonstrating Eigen basics (`solve`, `.eigenvalues()`, `.inverse()`) side-by-side with `csl::Polynomial::roots()`, `csl::spectral::fft`, and `csl::solveContinuousARE` on a tiny 2-state example — showing all three "hand-written vs. delegated" categories working together in one program.

**Deliverable:** `solveContinuousARE` on a double integrator (`A=[[0,1],[0,0]]`, `B=[[0],[1]]`, `Q=I`, `R=1`) produces `K = R^{-1}B'P ≈ [1, 1.732]`, matching the textbook closed-form answer.

---

#### Day 15 — Review + Begin Discrete-Time Systems

**Read:** Review Ch. 7.

**Library Coding Task:**
Round out `StateSpace`/`TransferFunction` with the `operator*`/`operator+`/`feedback` interconnection algebra (Section 5.2), and sketch `DiscreteStateSpace` (same shape, `Ad,Bd,Cd,Dd` + sample time `dt`, stability test `|eig(Ad)| < 1`).

**Week 3 Deliverables:** `TransferFunction` and `StateSpace` complete and tested; `solveContinuousARE`/`solveLyapunov` working; `controllabilityMatrix`/`isControllable`/`isObservable` complete; FFT and `Polynomial` from Weeks 1-2 fully integrated. No custom `Matrix` class was ever written — three weeks that used to go into decomposition code went into Riccati/Lyapunov (the two most control-theory-dense algorithms in the core library) and the FFT instead.

---

### Week 4 — C++ Library Features + System Representations Complete (Ch. 8-10)

**Goal by end of week:** Logger/CSV I/O complete; `ZeroPoleGain` and all TF/SS/ZPK conversions; time-domain and frequency-domain response tools, the latter now able to fall back on the FFT for sampled data.

**Day 16 (Ch. 8, IO Library):** Complete `Logger.hpp` and `DataExport.hpp` (`writeCSV`/`readCSV` now read/write `Eigen::MatrixXd` directly — Eigen has no native CSV support, so this glue code is legitimately ours to write). Add `friend operator<<` pretty-printing to `TransferFunction`.

**Day 17 (Ch. 9 pt. 1, Sequential Containers):** Implement `csl::simulation::RingBuffer<T>` (thin `std::deque` wrapper). Begin `StateSpace::controllabilityGramian()`/`observabilityGramian()`, both one-line calls into `core/LinearAlgebra.hpp`'s Lyapunov solver from Day 14.

**Day 18 (Ch. 9 pt. 2, Container Adaptors):** Implement `ZeroPoleGain` (Section 5.2) and `SystemConversions.hpp`'s `tf2ss`/`ss2tf`/`zpk2tf`/`tf2zpk`/`ss2zpk`. `tf2ss` builds the controllable canonical form directly with Eigen block assignment (`A.bottomRows(1) = ...`); no `std::stack`-based coefficient shuffling is needed anymore since Eigen's block operations replace what used to require manual container juggling.

**Day 19 (Ch. 10 pt. 1, Algorithms/Lambdas):** Implement `TimeResponse.hpp` (`stepResponse`/`impulseResponse`/`lsim`), passing a lambda closing over `sys.A()`/`sys.B()` as the `DynamicsFunc` into the Week 2 ODE solver. Use `std::transform`/`std::inner_product` where they clarify — but note most of what used to require STL-algorithm-over-raw-loop refactoring is now simply an Eigen expression (`(A*x + B*u)` is already vectorized, no loop to refactor).

**Day 20 (Ch. 10 pt. 2, Iterators/bind):** Implement `FrequencyResponse.hpp`'s `bode`/`nyquist` (analytic evaluation of `H(jw)`, Section 5.4) **and** wire up `csl::spectral::welchPSD`/`empiricalTransferFunction` bodies (stubbed Day 8) — this is the first day the FFT actually gets *used* by another module, closing the loop between "we wrote an FFT" and "the FFT does something for control analysis."

**Week 4 Deliverable:** Logger/CSV I/O; complete conversions; step/impulse/ramp/lsim; Bode/Nyquist (analytic) and Welch PSD/empirical FRF (FFT-based) both working.

---

### Week 5 — Dynamic Memory + System Models Complete (Ch. 11-12)

**Goal by end of week:** `SystemRegistry` with smart pointers; system interconnection algebra; `PIDController`; full `Simulator`; discrete-time systems; stability analysis (Routh-Hurwitz).

**Day 21 (Ch. 11, Associative Containers):** `SystemRegistry` (`std::map<std::string, std::shared_ptr<SystemModel>>`), `ParameterMap` (`std::unordered_map`). Implement `RootLocus.hpp` (`rootLocus` sweeps gain, calling `TransferFunction::poles()` at each step — pure control theory, no new Eigen usage beyond what `poles()` already does).

**Day 22 (Ch. 12 pt. 1, unique_ptr/shared_ptr):** `CompositeSystem.hpp`'s `series`/`parallel`/`feedback` — implemented with `Eigen::MatrixXd::Zero(...)` plus `.block(...)` assignment for the interconnection block matrices (replacing the old hand-rolled `horzcat`/`vertcat`). `PIDController` (Section 5.5) with a `std::shared_ptr<SystemModel>` plant reference.

**Day 23 (Ch. 12 pt. 2, weak_ptr):** `Simulator` (Section 5.12) using `std::weak_ptr` for non-owning plant/controller references; implement the RK45 adaptive-step integrator (Dormand-Prince coefficients) in `ODESolver.hpp` — fully hand-written per the ODE decision in Section 8. `examples/ex02_pid_step_response.cpp`.

**Day 24 (Integration Day):** Implement `Stability.hpp`'s `routhHurwitz`/`routhArray` (a pure tabular algorithm over `Polynomial` coefficients — no Eigen needed at all, a nice contrast day showing not everything even needs the linear-algebra backend). Add Google Test infrastructure to `CMakeLists.txt`.

**Day 25 (Review + Discrete Systems):** `DiscreteStateSpace`/`DiscreteTransferFunction`; `Discretization.hpp`'s `c2d`/`d2c` — ZOH via `Matrix::exp()` (Section 5.3), Tustin/Euler as direct algebraic formulas. `LeadLagCompensator`.

**Week 5 Deliverable:** Smart-pointer-based registry and interconnection algebra; `PIDController`; `Simulator` with RK45; Routh-Hurwitz; discretization methods.

---

### Week 6 — Operator Overloading + Analysis Complete (Ch. 13-14)

**Goal by end of week:** Full operator algebra on `TransferFunction`/`StateSpace`; controllers callable as functors; classical control module complete.

**Day 26 (Ch. 13 pt. 1, Copy/Destroy):** Copy-control audit — much shorter than originally planned, since `TransferFunction`/`StateSpace`/`Polynomial` all hold only `Eigen::MatrixXd`/`Polynomial` members, whose copy/move semantics are already correct and efficient by default. The genuine exercise here is *understanding why* — reading Eigen's own copy-on-write-free, move-friendly design — rather than writing five special member functions by hand. Complete `core/Exceptions.hpp`.

**Day 27 (Ch. 13 pt. 2, Move Semantics):** Verify (via `static_assert(std::is_nothrow_move_constructible_v<T>)`) that `TransferFunction`, `StateSpace`, `Polynomial` are all nothrow-move-constructible — they are, automatically, because Eigen's types are. Benchmark copy vs. move of a 200-state `StateSpace` to see Eigen's move semantics in action.

**Day 28 (Ch. 14 pt. 1, Arithmetic/Relational Operators):** `TransferFunction::operator*` (series), `operator+` (parallel), `feedback()`; `StateSpace` equivalents built with Eigen block algebra.

**Day 29 (Ch. 14 pt. 2, Call Operator):** `operator()` on `PIDController` and all controllers (functor interface); `TransferFunction::operator()(Complex s)` for direct frequency evaluation; `ZieglerNichols` as a tuning functor.

**Day 30 (Review + Classical Control Complete):** `LoopShaping.hpp`; `examples/ex04_classical_control.cpp` (PID + lead-lag on a 3rd-order plant); comprehensive tests for the classical control module.

**Week 6 Deliverable:** Full operator algebra; functor-style controllers; classical control design complete with examples.

---

### Week 7 — OOP and Templates + Modern/Optimal Control (Ch. 15-16)

**Goal by end of week:** Full polymorphic `SystemModel`/`Controller`/`Estimator` hierarchies; LQR/LQG/Kalman filter/EKF/UKF; generic filter design (Butterworth/Chebyshev/Elliptic/Bessel).

**Day 31 (Ch. 15 pt. 1, Virtual Functions):** `SystemModel`/`LTISystem`/`ContinuousLTI`/`DiscreteLTI` abstract hierarchy (Section 5.2); `Controller` base class. Make `TransferFunction`/`StateSpace`/`ZeroPoleGain` inherit properly with `override`.

**Day 32 (Ch. 15 pt. 2, Abstract Classes):** `PolePlacementController` (Ackermann's formula, Section 5.6); `LQRController` calling `solveContinuousARE` (already implemented Day 14!) — this day is now mostly *using* Week 3's work rather than deriving it from scratch, which is exactly the payoff of front-loading the Riccati solver. `KalmanFilter` (Section 5.9).

**Day 33 (Ch. 15 pt. 3, Virtual Destructors, final/override):** `LQGController` (LQR + KalmanFilter composition); `ExtendedKalmanFilter` with optional analytic Jacobians falling back to `csl::numericalJacobian`; `UnscentedKalmanFilter` with sigma points generated via `P.llt()` (Eigen Cholesky) — note how the *sigma-point construction and weighting* is estimation theory (hand-written), while the matrix square root it needs is Eigen.

**Day 34 (Ch. 16 pt. 1, Function/Class Templates):** `Filter<T>` template base (Section 5.10); `ButterworthFilter` — analog prototype pole formula (hand-derived), digitization via bilinear transform (an algebraic substitution, hand-written) producing an `IIRFilter`.

**Day 35 (Ch. 16 pt. 2, Variadic Templates):** `ChebyshevFilter`/`EllipticFilter`/`BesselFilter`; a variadic `cascade_apply(x, filter1, filter2, ...)` helper for chaining filters; `MPCController` skeleton (prediction/control matrices via Eigen block ops, QP solve deferred to `core::solveQP`).

**Week 7 Deliverable:** Full polymorphic hierarchy; LQR/LQG/KF/EKF/UKF; complete analog filter design suite.

---

### Week 8 — Advanced Topics + Complete Library Integration (Ch. 17-19)

**Goal by end of week:** Signal generation, particle filter, least-squares/RLS/N4SID/frequency-domain system ID (the last of these finally exercising `csl::spectral::empiricalTransferFunction` end-to-end), MPC's QP solver, iLQR, H-infinity, complete test suite and examples.

**Day 36 (Ch. 17, Specialized Library Facilities):** `SignalGenerator.hpp` (`WhiteNoise`, `PRBS` via `<random>`); `ParticleFilter` (importance sampling + resampling); `LeastSquaresID` (`Phi.colPivHouseholderQr().solve(y)` — Eigen does the solve, we build the ARX regression matrix `Phi`, which is the system-identification-specific step). Use `std::tuple` for `StateSpace::size()` and a structured-binding demo (`auto [n,m,p] = sys.size();`); use `std::bitset` to revisit and document the FFT's bit-reversal permutation from Week 1 with the "proper" library type now that the chapter covers it formally.

**Day 37 (Ch. 18, Tools for Large Programs):** Finish the `core::Exceptions.hpp` hierarchy; apply consistent namespacing library-wide; write the master `csl.hpp` umbrella header; `RecursiveLeastSquaresID` (online RLS with forgetting factor).

**Day 38 (Ch. 19, Specialized Tools):** `dynamic_cast`-based safe downcasting in `SystemRegistry`; `typeid`-based system type reporting; `SubspaceID` (N4SID) built on `Eigen::BDCSVD`; `DynamicProgramming.hpp` (value/policy iteration).

**Day 39 (Integration Sprint):** `iLQR` (forward/backward passes, line search); `HInfinity.hpp` (gamma-iteration over `solveContinuousARE`); `FrequencyDomainID` — the payoff day for the FFT module: `csl::spectral::empiricalTransferFunction(u, y, fs)` produces `H(jw)` from measured data, and Levy's method fits rational TF coefficients to it via an Eigen weighted-least-squares solve; `FIRFilter`/`IIRFilter`/`NotchFilter`; complete `MPCController` wiring its prediction matrices into `core::solveQP`. `examples/ex05_lqg_control.cpp`, `examples/ex06_kalman_filter.cpp`, `examples/ex10_fft_spectral_analysis.cpp`.

**Day 40 (Final Integration):** `H2norm`/`HInfNorm` (Lyapunov-based and Hamiltonian-bisection-based respectively); `MonteCarloSimulator`; remaining examples (`ex07`-`ex09`, `ex11_full_pipeline.cpp`); full test suite across all 14 modules (~200+ tests); `README.md`.

**Week 8 Deliverable:** Complete library — all 13 functional modules plus `spectral`, ~70 headers, 11 examples, 200+ tests, zero hand-rolled generic linear algebra anywhere in the codebase.

---

## 10. Library State Snapshots

### End of Week 1
```
include/csl/core/Types.hpp        [Eigen aliases, std::complex, constants, enums]
include/csl/core/MathUtils.hpp    [dB, wrapAngle, saturate, deadband, besselPolyCoeff]
include/csl/core/Logger.hpp       [skeleton]
include/csl/spectral/FFT.hpp      [declarations + bitReverse() implemented]
CMakeLists.txt                    [Eigen fetched and linked]
src/main.cpp
```
Eigen is proven working end-to-end (identity matrix printed) before any control-systems code exists.

### End of Week 2
Everything above, plus:
```
include/csl/spectral/FFT.hpp      [fft/ifft fully implemented and tested]
include/csl/core/Polynomial.hpp   [complete; roots() via Eigen::EigenSolver on companion matrix]
include/csl/core/LinearAlgebra.hpp [signatures stubbed, NotImplemented bodies]
include/csl/simulation/ODESolver.hpp [eulerStep, rk4Step]
```

### End of Week 3
Everything above, plus:
```
include/csl/systems/TransferFunction.hpp   [complete]
include/csl/systems/StateSpace.hpp         [complete: poles(), isControllable/Observable]
include/csl/core/LinearAlgebra.hpp         [solveContinuousARE, solveLyapunov IMPLEMENTED —
                                             the two most important hand-written algorithms
                                             in the core module, done by Day 14]
examples/ex01_eigen_and_polynomial.cpp
```

### End of Week 4
Everything above, plus: `Logger`/`DataExport` complete; `ZeroPoleGain`; all TF/SS/ZPK conversions; `TimeResponse.hpp`; `FrequencyResponse.hpp` (analytic Bode/Nyquist); `csl::spectral::welchPSD`/`empiricalTransferFunction` implemented and first used.

### End of Week 5
Everything above, plus: `SystemRegistry`; `CompositeSystem` (series/parallel/feedback); `PIDController`; `Simulator` with RK45; `Stability.hpp` (Routh-Hurwitz); discrete-time systems and discretization (ZOH via `Matrix::exp()`); Google Test operational.

### End of Week 6
Everything above, plus: full operator algebra (`*`, `+`, `feedback()`) on `TransferFunction`/`StateSpace`; all controllers callable as functors; `ZieglerNichols`; `LoopShaping`; classical control module complete with examples. Copy-control audit confirmed all types are automatically move-efficient courtesy of Eigen.

### End of Week 7
Everything above, plus: full `SystemModel`/`Controller`/`Estimator` polymorphic hierarchies; `PolePlacementController`, `LQRController` (built on Week 3's Riccati solver), `LQGController`; `KalmanFilter`/`ExtendedKalmanFilter`/`UnscentedKalmanFilter`; complete analog filter suite (`ButterworthFilter`/`ChebyshevFilter`/`EllipticFilter`/`BesselFilter`); `MPCController` skeleton.

### End of Week 8 (Final)
All 13 functional modules plus `spectral`: ~70 headers, ~70 implementation files, 11 examples, 200+ unit tests. `FrequencyDomainID` closes the loop on the FFT module by using it for real system identification from data. Zero hand-rolled dense linear algebra anywhere in the codebase — every matrix operation traces back to Eigen; every control-theory algorithm (Riccati, Lyapunov, Kalman, MPC's QP, the FFT itself) is hand-written and tested.

---

## 11. Future Extensions

Unchanged in spirit from the original design, with one adjustment: since Eigen is now the linear-algebra foundation, "performance optimization" is largely about *configuring* Eigen rather than hand-optimizing loops.

- **Eigen + MKL/OpenBLAS backend** (Section 6) for large-scale problems (long subspace-ID data records, large MPC horizons).
- **Swap the hand-written active-set QP solver for OSQP** in `core::solveQP` — the MPC formulation code in `optimal/MPC.hpp` would not need to change at all, since it only depends on the `QPResult solveQP(H, f, A_ineq, b_ineq, A_eq, b_eq)` signature. This is a deliberate seam left in the design specifically so "hand-written for learning" and "swap in production-grade" are not in tension.
- **Nonlinear systems**: `NonlinearSystem` class with user-supplied dynamics + Eigen-based numerical linearization (`core::numericalJacobian`, already built for EKF, reused here).
- **Differential Dynamic Programming (DDP)**, stochastic MPC, square-root/ensemble Kalman filters, moving horizon estimation.
- **SIMD/parallelism**: Eigen already vectorizes; C++17 `<execution>` policies could parallelize Monte Carlo runs and gain-sweep analyses (root locus, gamma-iteration) across cores.
- **Python bindings** via `pybind11` — Eigen has first-class, zero-copy NumPy interop, making this substantially easier than it would have been with a hand-rolled matrix type.
- **Hardware interfaces**: `Actuator`/`Sensor` abstractions, ROS 2 node wrapper, serial/CAN interface for embedded deployment.

---

*End of DESIGN.md*

*This document is a living specification, revised as of v0.2.0 to move all generic numerical infrastructure onto Eigen/STL while keeping every control-theory- and DSP-specific algorithm — including the FFT — hand-written. Update it whenever a module's design changes significantly.*
