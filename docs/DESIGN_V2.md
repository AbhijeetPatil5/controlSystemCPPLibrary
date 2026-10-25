# Control Systems Library (csl) — Design Document V2

**Version:** 2.0.0
**Language standard:** C++17
**Linear-algebra backend:** Eigen 3.4 (header-only)
**Test framework:** Google Test
**Primary platform:** Windows 11, Visual Studio 2022 (MSVC 14.44), CMake 4.x, VS Code (also builds on Linux/macOS with GCC/Clang)
**Author:** Abhijeet Patil — a learning-driven build of a control-systems library, starting from zero C++

---

## Table of Contents

- [0. How to read this document](#0-how-to-read-this-document)
- [1. Goals and ground rules](#1-goals-and-ground-rules)
- [2. What changed from V1, and why](#2-what-changed-from-v1-and-why)
- [3. The two curricula and how they interlock](#3-the-two-curricula-and-how-they-interlock)
- [4. Library architecture](#4-library-architecture)
  - [4.1 Identity](#41-identity)
  - [4.2 Final directory layout](#42-final-directory-layout)
  - [4.3 The type contract — why everything "just works" together](#43-the-type-contract--why-everything-just-works-together)
  - [4.4 Module dependency graph and data-flow map](#44-module-dependency-graph-and-data-flow-map)
  - [4.5 Build vs. buy — the decision table](#45-build-vs-buy--the-decision-table)
- [5. Toolchain, build commands and conventions](#5-toolchain-build-commands-and-conventions)
- [6. The running examples (physical systems used throughout)](#6-the-running-examples-physical-systems-used-throughout)
- [7. The day-by-day plan](#7-the-day-by-day-plan)
  - Phase 1 — Foundations: toolchain, C++ basics, first simulations (Days 1–10)
  - Phase 2 — System representations and the time domain (Days 11–16)
  - Phase 3 — The frequency domain (Days 17–22)
  - Phase 4 — Classical control design (Days 23–28)
  - Phase 5 — Discrete time and digital signal processing (Days 29–36)
  - Phase 6 — State-space design (Days 37–43)
  - Phase 7 — Optimal control (Days 44–49)
  - Phase 8 — State estimation (Days 50–55)
  - Phase 9 — System identification (Days 56–59)
  - Phase 10 — Robustness, MPC, nonlinear optimal control (Days 60–66)
  - Phase 11 — Packaging, documentation, release (Days 67–70)
- [8. Testing strategy](#8-testing-strategy)
- [9. Milestone snapshots](#9-milestone-snapshots)
- [10. Stretch topics (after Day 70)](#10-stretch-topics-after-day-70)
- [11. Glossary](#11-glossary)
- [12. Reading list](#12-reading-list)

---

## 0. How to read this document

This document is a **plan**, a **specification** and a **textbook outline** at the same time.

- Sections 1–6 are the fixed reference: what the library is, what its types are, how modules fit together, which tools you use. Read them once at the start, then come back to Section 4.3 (the type contract) whenever you write a new function — it is the rulebook that keeps every function compatible with every other.
- Section 7 is the plan itself: 70 working days, grouped into 11 phases. **Do one day per sitting.** A "day" is roughly 3–5 hours: about one third reading/understanding, two thirds writing code and tests. If a day takes two sittings, that is fine — the numbering is a sequence, not a calendar.
- Every day has the same shape:

| Block | What it tells you |
|---|---|
| **C++ you learn** | The language/tooling concept of the day, explained from zero, with the *why* |
| **Control / DSP you learn** | The control-theory or signal-processing concept of the day, explained from zero |
| **Files you touch** | Exactly which `.hpp`, `.cpp`, `CMakeLists.txt` and test files you create or edit |
| **Build** | What changes in the build system that day, if anything |
| **Code** | The core code of the day. Early days give complete files; later days give the algorithm and skeleton and leave the routine parts to you |
| **How it connects** | What the new code consumes from earlier days and what later days will consume from it |
| **Verify** | A concrete check with a known numeric answer, and the tests you write |
| **Done when** | A checklist. Do not move to the next day until every box is ticked |

- Code in this document uses the final names and signatures. If you type a function today, the code on Day 50 will call it with exactly that name. That is deliberate: it is what makes the library cohesive.
- Nothing here assumes you know C++. It does assume you know what a loop, a variable and a function are, and basic calculus/linear algebra (derivatives, matrices, complex numbers) — which you refresh as you go.

---

## 1. Goals and ground rules

### 1.1 The two goals

**Goal A — C++ from zero to competent library author.** By Day 70 you can explain and use: compilation vs. linking, translation units, headers vs. source files, `#include` and include guards, CMake targets and how they link, static libraries, value vs. reference vs. pointer semantics, `const`, stack vs. heap memory, dynamic allocation and why modern C++ almost never uses `new`/`delete` directly, RAII, smart pointers, classes, constructors, invariants, operator overloading, inheritance and virtual dispatch, abstract classes, lambdas and `std::function`, templates, the STL containers and algorithms, iterators, exceptions, namespaces, move semantics, `constexpr`, `std::optional`, structured bindings, unit testing with Google Test, debugging with a real debugger, compiler warnings and sanitizers, and packaging a library for others.

**Goal B — Control systems from "basic ideas" to "I can derive, implement and test it".** By Day 70 you can explain, implement and test: dynamic systems and ODEs; time-domain response and specs; state-space, transfer-function, zero-pole-gain, Laplace and z-domain representations and conversions between them; poles, zeros, stability, minimum phase, time delay; frequency response, Bode, Nyquist, gain/phase margins, sensitivity; root locus; Routh–Hurwitz; PID and its practical details, tuning rules, lead/lag, loop shaping; sampling, discretization, aliasing; DFT/FFT, windows, PSD, digital filter design (FIR, IIR, Butterworth, notch); controllability/observability, pole placement, observers, Lyapunov equations, Gramians; LQR (continuous and discrete), Riccati equations; nonlinear systems, linearization; Kalman filter, steady-state KF, LQG, EKF, UKF; system identification (ARX, RLS, ETFE, subspace/ERA); robustness (uncertainty, small-gain, H2/H∞ norms); QP and MPC; iLQR; and an H∞ synthesis capstone. In other words: the core of the MATLAB Control System Toolbox, a good slice of the Signal Processing and System Identification Toolboxes, and the MPC Toolbox's heart — all written by you, understood by you, and tested.

### 1.2 Ground rules (non-negotiable)

1. **Do not reinvent generic numerical infrastructure.** Matrices, vectors, decompositions (LU, QR, SVD, Cholesky, Schur, eigenvalues), matrix exponentials, complex numbers, elementary math, random numbers, containers and algorithms come from **Eigen**, `std::complex`, `<cmath>`, `<random>` and the STL. Writing a matrix class teaches you nothing about control and produces something slower and buggier than Eigen.
2. **Hand-write everything that *is* control theory or DSP.** If understanding the algorithm *is* the learning objective (Riccati solver, Kalman filter, FFT, Butterworth design, Routh array, MPC's QP…), you write it. If writing it would merely be "making something custom", you do not.
3. **The test for any piece of code is: "does writing this teach me C++ or control?"** A `Polynomial` class teaches operator overloading *and* the s-domain: write it. A `Logger` singleton teaches some C++ but no control, and the project does not need one: skip it. (V1 had it; V2 drops it.)
4. **Sequential, no gaps.** Each day uses only what earlier days built, and builds something a later day needs. Nothing is "stubbed for later" without a named day that fills it in.
5. **Cohesive by construction.** One set of types (Section 4.3), one set of conventions (time along rows, rad/s everywhere, descending polynomial coefficients, `Ts == 0` means continuous), so any function's output is a valid input to any function that expects that kind of object. A `StateSpace` goes into `c2d`, whose output goes into `KalmanFilter`, whose output goes into `plot`, with no adapters.
6. **Understand → implement → test, every day.** A concept is not "done" until it is implemented *and* a test with a known numeric answer passes.
7. **Plotting is part of the library.** Results you cannot see, you cannot judge. The library writes self-contained HTML/SVG plots (no external plotting dependency) and CSV files. Writing that plotter is a legitimate C++ exercise (file I/O, string formatting, class design).

---

## 2. What changed from V1, and why

V1 (`docs/DESIGN.md`) was a good first sketch. It got the central decision right — **Eigen for linear algebra, hand-write the control theory** — and its module list is close to what V2 keeps. The problems it had:

| V1 issue | V2 fix |
|---|---|
| 40 days for a from-zero learner to reach H∞ synthesis and N4SID is not realistic; the plan would have collapsed around Week 4 | 70 days in 11 phases, with the C++ basics given the room they actually need (Phase 1 is ten days, not five) |
| Daily tasks were lists of file names, not instructions; a beginner could not start Day 1 from it | Every day has the concept explained, the exact files, the code, the connection to other days, a numeric check and a done-when checklist |
| C++ topics were ordered by the book's chapters, so the library order was sometimes forced (e.g. `Logger` on Day 4 because Chapter 3 is about strings) | Topics are ordered by what the *library* needs next; the book is optional reading per day, not the driver |
| Several classes existed only to practise C++ (`Logger`, `Registry`, a `RingBuffer` wrapper over `std::deque`) | Dropped. Every file in V2 exists because the library needs it |
| No plotting at all, so results were CSVs you had to open elsewhere | `csl::plot` module writes HTML/SVG; Bode, Nyquist, root locus, step response, spectra and filter responses all plot from inside the library |
| Separate class trees for continuous and discrete systems (`TransferFunction` vs. `DiscreteTransferFunction`, etc.) doubled the surface area | One `StateSpace` and one `TransferFunction`, each carrying a sample time `Ts` (`Ts == 0` is continuous — the MATLAB convention). Half the classes, same capability, and the frequency-response code handles both with one branch |
| Discrete-time, DSP and FFT came early (Week 2) before the learner knew what a transfer function was | DSP is Phase 5, after continuous-time analysis, so sampling and the z-domain are learned as "what sampling does to what you already understand" |
| Test tolerance policy existed but there was no "known answer" per day | Every day's Verify block names the number you must get |
| The QP solver was "active-set", which is hard to get right as a first optimizer | Hildreth's dual method (simple, robust for MPC-sized problems) is the primary; active-set is a stretch topic |
| Lyapunov solver via Bartels–Stewart on Day 14 | Kronecker-product formulation first (10 lines, exact, fine up to ~30 states), Bartels–Stewart as a stretch topic |

What V2 keeps from V1: the namespace `csl`, Eigen via CMake `FetchContent`, Google Test, the directory shape `include/csl/<module>`, the hand-written FFT, the decision that the Riccati solver is the single most valuable hand-written algorithm, and the build-vs-buy reasoning.

---

## 3. The two curricula and how they interlock

The table below is the whole plan at a glance. Each row is one phase; the two middle columns show how the C++ topic and the control topic of that phase *need* each other.

| Phase (days) | C++ track | Control / DSP track | Why they pair |
|---|---|---|---|
| 1 (1–10) | Toolchain, compile/link, functions, headers, CMake, `std::vector`, Eigen, lambdas, classes, exceptions, `std::complex`, Google Test, debugger | Dynamic systems as ODEs, first- and second-order systems, Euler and RK4 integration, state-space form, poles and stability | A simulation loop is the natural first program; a state vector is the natural reason to need Eigen; "poles" are the natural reason to need complex numbers and an eigen-solver |
| 2 (11–16) | Operator overloading, friends, static factories, inheritance and virtual functions, Eigen block operations, file I/O, string streams, STL algorithms and iterators | Laplace domain, polynomials, transfer functions, poles/zeros, dc gain, realizations (tf↔ss), step/impulse response, time-domain specs | A polynomial *is* an operator-overloading exercise; "a TF and an SS are both LTI systems" *is* an inheritance exercise; a plotter *is* a file-I/O exercise |
| 3 (17–22) | `std::optional`, overload sets, free vs. member functions, sorting with lambdas, header organization, `inline` | Frequency response, Bode, Nyquist, margins, interconnections, sensitivity, minimum phase, delay, root locus, Routh–Hurwitz | Margins may not exist → `std::optional`; interconnection algebra → operators on systems; root locus → sorting/matching with lambdas |
| 4 (23–28) | Abstract base classes, `protected`, struct configs, smart pointers, RAII, stack vs. heap, `new`/`delete` once and never again, `std::pair`/`tuple`/structured bindings, root finding | PID (practical form), closed-loop sampled-data simulation, anti-windup, tuning rules, relay auto-tune, lead/lag, loop shaping | A `Controller` hierarchy is the first real polymorphic design; a simulator owning plant+controller is the first real ownership problem |
| 5 (29–36) | `enum class`, `switch`, Eigen MatrixFunctions, bitwise ops, `size_t`, in-place algorithms, function and class templates | Sampling, ZOH, z-domain, Tustin, discrete stability, DFT/FFT, windows, PSD, FIR/IIR filters, Butterworth, notch, digital PID | The FFT's bit reversal *is* bitwise arithmetic; a `Filter<T>` *is* a template; discretization methods *are* an enum+switch |
| 6 (37–43) | Numerical rank, Kronecker products, Eigen Schur/LLT, Rule of Zero, copy vs. move, benchmarking | Controllability/observability, pole placement, observers, separation principle, Lyapunov equations, Gramians, integral action | Lyapunov → kron/vec; the copy/move audit is best done once classes hold big Eigen matrices |
| 7 (44–49) | Block matrices with `<<`, complex eigenvectors, sorting by predicate, `std::function` dynamics, numerical differentiation | LQR, ARE (Hamiltonian method), discrete LQR and Riccati recursion, Bryson's rule, LQR margins, nonlinear systems and linearization, dynamic programming | The ARE solver is the library's hardest linear-algebra assembly; nonlinear dynamics are `std::function` |
| 8 (50–55) | `<random>`, seeding, RNG inside classes, Cholesky, optional callbacks | Noise, covariance, Kalman filter, steady-state KF, LQG, EKF, UKF | Estimators are stateful objects with virtual `predict`/`update` — the `Estimator` hierarchy |
| 9 (56–59) | Least-squares via QR, SVD, Hankel matrices, LFSR bit tricks | ARX, RLS, ETFE/coherence, ERA (subspace) identification, validation | Identification consumes the FFT (Phase 5) and produces systems (Phase 2) — the loop closes |
| 10 (60–66) | Bisection on matrix spectra, QP duality, convergence diagnostics, larger algorithms built from library pieces | Uncertainty, small-gain, H2/H∞ norms, QP, MPC, iLQR, H∞ synthesis | Everything here is *composition* of what exists — the proof the library is cohesive |
| 11 (67–70) | CMake install/export, CI, docs, API review | End-to-end pipeline example | Shipping |

---
## 4. Library architecture

### 4.1 Identity

| Property | Value |
|---|---|
| Namespace | `csl`, with one sub-namespace per module (`csl::systems`, `csl::analysis`, …) |
| Language standard | C++17 (no extensions) |
| Build | CMake ≥ 3.20, one static library target `csl::csl`, plus `tests/` and `examples/` |
| Dependencies | Eigen 3.4 (fetched by CMake), Google Test 1.14 (fetched by CMake). Nothing else |
| Scalar | `csl::Real` = `double` |
| Vector / matrix | `csl::Vector` = `Eigen::VectorXd`, `csl::Matrix` = `Eigen::MatrixXd` |
| Complex | `csl::Complex` = `std::complex<double>`; `csl::ComplexVector` = `Eigen::VectorXcd` |
| Ownership | Value semantics everywhere for numeric objects; `std::unique_ptr`/`std::shared_ptr` only for polymorphic objects owned by a simulator |
| Errors | Exceptions derived from `csl::Error` (Section 4.3.6) |
| Platform | Windows (MSVC), Linux (GCC), macOS (Clang) |

### 4.2 Final directory layout

This is the layout at Day 70. Each file is created on the day marked.

```
controlSystemCPPLibrary/
├── CMakeLists.txt                       Day 4 (grows through Day 67)
├── README.md                            Day 68
├── .gitignore                           Day 1
├── .github/workflows/ci.yml             Day 69
├── docs/
│   ├── DESIGN.md                        (V1, kept for history)
│   └── DESIGN_V2.md                     this document
├── cmake/
│   └── cslConfig.cmake.in               Day 67
├── include/csl/
│   ├── csl.hpp                          Day 22  umbrella header
│   ├── core/
│   │   ├── Types.hpp                    Day 6   aliases, constants
│   │   ├── Exceptions.hpp               Day 8   error hierarchy
│   │   ├── MathUtils.hpp                Day 17  dB, wrapAngle, logspace, linspace
│   │   ├── Polynomial.hpp               Day 11
│   │   ├── LinearAlgebra.hpp            Day 37  ctrb/obsv, Lyapunov, Gramians, kron/vec, ARE (Day 44)
│   │   └── NumericalMethods.hpp         Day 27  bisection, numericalJacobian (Day 47), QP (Day 62)
│   ├── systems/
│   │   ├── LTISystem.hpp                Day 13  abstract base
│   │   ├── StateSpace.hpp               Day 8
│   │   ├── TransferFunction.hpp         Day 12
│   │   ├── Conversions.hpp              Day 14  tf2ss, ss2tf, zpk helpers; c2d/d2c (Days 29–30)
│   │   ├── Interconnect.hpp             Day 19  series, parallel, feedback
│   │   └── NonlinearSystem.hpp          Day 47
│   ├── simulation/
│   │   ├── SimResult.hpp                Day 7   the one time-domain result struct
│   │   ├── ODESolver.hpp                Day 4   scalar Euler; vector Euler/RK4/integrate on Day 7
│   │   ├── TimeResponse.hpp             Day 15  step, impulse, lsim, specs (Day 16)
│   │   ├── Simulator.hpp                Day 24  closed-loop sampled-data simulator (estimator hook Day 39, noise Day 50)
│   │   └── MonteCarlo.hpp               Day 50
│   ├── analysis/
│   │   ├── FrequencyResponse.hpp        Day 17  frequencyResponse, bode, nyquist, margins (Day 18)
│   │   ├── RootLocus.hpp                Day 21
│   │   ├── Stability.hpp                Day 22  Routh–Hurwitz; Lyapunov test (Day 40)
│   │   ├── Norms.hpp                    Day 40  H2 norm; H∞ norm (Day 61)
│   │   └── Robustness.hpp               Day 60  uncertainty weights, small-gain tests
│   ├── control/
│   │   ├── Controller.hpp               Day 23  abstract base
│   │   ├── PID.hpp                      Day 23
│   │   ├── Tuning.hpp                   Day 26  Ziegler–Nichols, IMC, relay
│   │   ├── Relay.hpp                    Day 26  relay controller for auto-tuning
│   │   ├── LeadLag.hpp                  Day 27
│   │   ├── TransferFunctionController.hpp Day 30  run any discrete TF as a controller
│   │   ├── StateFeedback.hpp            Day 38  place (Ackermann), StateFeedbackController
│   │   ├── LQR.hpp                      Day 44  lqr, dlqr/finite horizon (Day 45), Bryson/integral (Day 46)
│   │   ├── DynamicProgramming.hpp       Day 48  value iteration on a grid
│   │   ├── LQG.hpp                      Day 52
│   │   ├── MPC.hpp                      Day 63
│   │   ├── iLQR.hpp                     Day 65
│   │   └── HInfinity.hpp                Day 66
│   ├── estimation/
│   │   ├── Estimator.hpp                Day 39  abstract base
│   │   ├── LuenbergerObserver.hpp       Day 39
│   │   ├── KalmanFilter.hpp             Day 51
│   │   ├── ExtendedKalmanFilter.hpp     Day 53
│   │   └── UnscentedKalmanFilter.hpp    Day 54
│   ├── dsp/
│   │   ├── FFT.hpp                      Day 31
│   │   ├── Spectral.hpp                 Day 32  windows, periodogram, welch, etfe (Day 58)
│   │   ├── DigitalFilter.hpp            Day 33  FIRFilter, IIRFilter
│   │   └── FilterDesign.hpp             Day 34  butterworth, firWindowed, notch
│   ├── sysid/
│   │   ├── ARX.hpp                      Day 56
│   │   ├── RLS.hpp                      Day 57
│   │   ├── FrequencyDomainID.hpp        Day 58
│   │   ├── Subspace.hpp                 Day 59  ERA
│   │   └── Validation.hpp               Day 56  fit %, residual tests (Day 59)
│   ├── signals/
│   │   └── Signals.hpp                  Day 16  step/ramp/sine/chirp; WhiteNoise, PRBS (Day 50)
│   └── plot/
│       ├── Figure.hpp                   Day 15  SVG/HTML line plots
│       ├── Plots.hpp                    Day 15  plotStep (Day 15), plotBode/Nyquist/RootLocus/PZ (Days 17–21), plotSpectrum (Day 32)
│       └── CSV.hpp                      Day 5   writeCSV / readCSV
├── src/                                 mirrors include/csl, one .cpp per .hpp that needs one
├── tests/                               mirrors include/csl, one test_*.cpp per header
│   └── CMakeLists.txt                   Day 10
└── examples/
    ├── CMakeLists.txt                   Day 4
    ├── ex01_first_order_sim.cpp         Day 4 (finished Day 10)
    ├── ex02_representations.cpp         Day 22
    ├── ex03_classical_design.cpp        Day 28
    ├── ex04_dsp_and_filters.cpp         Day 36
    ├── ex05_state_space_design.cpp      Day 43
    ├── ex06_lqr_pendulum.cpp            Day 49
    ├── ex07_estimation.cpp              Day 55
    ├── ex08_sysid.cpp                   Day 59
    ├── ex09_mpc.cpp                     Day 64
    └── ex10_full_pipeline.cpp           Day 70
```

Eleven modules, ~52 headers. Compared with V1 this is smaller (no separate discrete classes, no Logger/Registry/RingBuffer, no Chebyshev/Elliptic/Bessel/particle filter — those are stretch topics in Section 10) and everything in it is reached by the plan.

### 4.3 The type contract — why everything "just works" together

This section is the rulebook. Every function in the library obeys it, so every function's output is a legal input to every function that accepts that kind of object.

#### 4.3.1 Scalars, vectors, matrices, complex numbers

```cpp
// include/csl/core/Types.hpp  (Day 6)
#pragma once
#include <Eigen/Dense>
#include <complex>
#include <limits>

namespace csl {

using Real          = double;
using Index         = Eigen::Index;             // signed integer for sizes/indices
using Vector        = Eigen::VectorXd;          // column vector, dynamic size
using RowVector     = Eigen::RowVectorXd;
using Matrix        = Eigen::MatrixXd;          // dynamic size, column-major
using Complex       = std::complex<Real>;
using ComplexVector = Eigen::VectorXcd;
using ComplexMatrix = Eigen::MatrixXcd;

inline constexpr Real PI     = 3.141592653589793238462643383279502884;
inline constexpr Real TWO_PI = 2.0 * PI;
inline constexpr Real EPS    = std::numeric_limits<Real>::epsilon();
inline constexpr Real INF    = std::numeric_limits<Real>::infinity();

} // namespace csl
```

Rules:
- All numeric code uses these aliases, never `double`/`Eigen::MatrixXd` spelled out, so the scalar type could be changed in one place.
- Vectors are **column** vectors. A time series of a scalar signal is a `Vector`. A time series of an `m`-channel signal is a `Matrix` with **one row per time sample and one column per channel** (`N × m`). This "time along rows" convention is used by every function that takes or returns sampled data.
- Frequencies passed to analysis functions are **angular, in rad/s** (`w`). Functions that genuinely work in Hz (signal generators, filter design against a sample rate) say so in the parameter name (`f_hz`, `fs_hz`).

#### 4.3.2 Polynomials

`csl::Polynomial` stores coefficients in **descending** powers (`{1, 3, 2}` is `s² + 3s + 2`), matching MATLAB's `tf([1],[1 3 2])` and every control textbook. `roots()` returns `std::vector<Complex>`. Transfer functions are built from two `Polynomial`s.

#### 4.3.3 Systems

```cpp
namespace csl::systems {

class LTISystem {                       // Day 13 — abstract base
public:
    virtual ~LTISystem() = default;
    virtual Index order()   const = 0;  // number of states / denominator degree
    virtual Index inputs()  const = 0;
    virtual Index outputs() const = 0;
    virtual Real  Ts()      const = 0;  // 0.0 => continuous time; > 0 => discrete with that sample time
    virtual std::vector<Complex> poles() const = 0;
    virtual std::vector<Complex> zeros() const = 0;          // SISO transmission zeros
    virtual ComplexMatrix evaluate(Complex s) const = 0;     // H(s) or H(z): outputs x inputs
    virtual Matrix dcGain() const = 0;
    bool isContinuous() const { return Ts() == 0.0; }
    bool isDiscrete()   const { return Ts() >  0.0; }
    bool isStable()     const;          // continuous: all Re(p) < 0 ; discrete: all |p| < 1
    bool isSISO()       const { return inputs() == 1 && outputs() == 1; }
};

class StateSpace       : public LTISystem { /* A, B, C, D, Ts */ };   // Day 8
class TransferFunction : public LTISystem { /* num, den, Ts  */ };    // Day 12 (SISO)

} // namespace csl::systems
```

Rules:
- **`Ts == 0.0` means continuous time**, `Ts > 0` means discrete time with sample period `Ts` seconds. There are no separate discrete classes. `c2d` returns a `StateSpace`/`TransferFunction` with `Ts > 0`; `d2c` returns one with `Ts == 0`.
- `evaluate(s)` is the single frequency-domain primitive. For continuous systems `frequencyResponse` calls it at `s = jω`; for discrete at `s = e^{jωTs}`. Every Bode/Nyquist/margin/norm function works on either kind of system through this one virtual function.
- `StateSpace` is MIMO. `TransferFunction` is SISO (a MIMO TF matrix is a stretch topic). Analysis functions that are SISO-only check `isSISO()` and throw otherwise.
- Any function that accepts `const LTISystem&` accepts both representations. Functions that need matrices (`c2d`, `lqr`, `KalmanFilter`) take `const StateSpace&`; you convert with `tf2ss` first — explicitly, so realizations are never hidden from you.

#### 4.3.4 Results of simulation and analysis

```cpp
namespace csl::simulation {
struct SimResult {       // Day 15 — produced by step, impulse, lsim, integrate, Simulator::run
    Vector t;            // N
    Matrix x;            // N x n   states   (time along rows)
    Matrix y;            // N x p   outputs
    Matrix u;            // N x m   inputs
};
}
namespace csl::analysis {
struct FrequencyResponse {   // Day 17 — produced by frequencyResponse(sys, w) AND by sysid::etfe(u, y, fs)
    Vector        w;         // rad/s, ascending
    ComplexVector H;         // H(jw) (SISO)
    Real          Ts = 0.0;  // sample time of the system it describes
};
struct Margins { /* std::optional fields */ };                       // Day 18
struct RootLocusData { Vector gains; std::vector<std::vector<Complex>> branches; };  // Day 21
}
```

Rules:
- Time-domain results are always a `SimResult`; frequency-domain results are always a `FrequencyResponse`. The plotting functions take exactly these structs. This is what makes "take a Kalman filter's output and look at its spectrum" a two-line program: `SimResult r = sim.run(...); auto psd = dsp::welch(r.y.col(0), fs);`.
- An **empirical** frequency response from measured data (Day 58) and an **analytic** one from a model (Day 17) are the *same struct*, so you can overlay them on one Bode plot and feed either to the frequency-domain identification fitter.

#### 4.3.5 Controllers, estimators, filters

```cpp
namespace csl::control {
class Controller {                         // Day 23
public:
    virtual ~Controller() = default;
    virtual Vector compute(const Vector& measurement, const Vector& reference, Real t) = 0;
    virtual void   reset() = 0;
    virtual Real   Ts() const = 0;         // controller sample time (> 0: it is digital)
    virtual Index  inputs()  const = 0;    // size of `measurement`
    virtual Index  outputs() const = 0;    // size of the returned control vector
};
}
namespace csl::estimation {
class Estimator {                          // Day 39
public:
    virtual ~Estimator() = default;
    virtual void   predict(const Vector& u) = 0;
    virtual void   update (const Vector& y) = 0;
    virtual Vector state()      const = 0;
    virtual Matrix covariance() const = 0;   // observers return a zero matrix
    virtual void   reset(const Vector& x0) = 0;
};
}
namespace csl::dsp {
class DigitalFilter {                      // Day 33
public:
    virtual ~DigitalFilter() = default;
    virtual Real   process(Real x) = 0;    // one sample in, one out
    Vector         process(const Vector& x);   // whole signal (calls the scalar version)
    virtual void   reset() = 0;
    virtual systems::TransferFunction toTransferFunction() const = 0;   // discrete TF (Ts > 0)
};
}
```

Rules:
- A `Controller` always works on vectors, even when SISO (a scalar PID wraps a length-1 `Vector`). This is what lets the single `Simulator` run a PID, a state-feedback law, an LQG controller or an MPC without special cases.
- A `Controller` that needs the full state (state feedback, LQR, MPC) declares that its `measurement` *is* the state vector. The `Simulator` gives it either the true state or an `Estimator`'s estimate — that choice is made in one place (Day 39).
- A `DigitalFilter` can always be turned into a discrete `TransferFunction`, so its frequency response is plotted with the same `bode()` as any system. Filter design produces filters; analysis consumes systems; the bridge is `toTransferFunction()`.

#### 4.3.6 Errors

```cpp
namespace csl {                                      // Day 8
class Error              : public std::runtime_error { using std::runtime_error::runtime_error; };
class DimensionMismatch  : public Error { using Error::Error; };
class InvalidArgument    : public Error { using Error::Error; };
class NumericalError     : public Error { using Error::Error; };   // singular, no convergence, …
class NotSupported       : public Error { using Error::Error; };   // e.g. MIMO where SISO required
}
```

Every precondition failure throws one of these with a message naming the function and the offending sizes. No function returns a magic number to signal failure; functions that can legitimately have "no answer" (a margin that does not exist) return `std::optional`.

### 4.4 Module dependency graph and data-flow map

Arrows point from a module to the modules it includes. There are no cycles; CMake compiles them in this order.

```
                 core (Types, Exceptions, MathUtils, Polynomial, LinearAlgebra, NumericalMethods)
                   ▲
          ┌────────┴──────────┬──────────────┬───────────────┐
       systems            signals            dsp             plot
   (LTISystem, SS, TF,  (step, sine,      (FFT, Spectral,  (Figure, CSV)
    Conversions,         PRBS, noise)      filters, design)   ▲
    Interconnect,          ▲                 ▲                │
    NonlinearSystem)       │                 │                │
          ▲                │                 │                │
          ├────────────────┴─────────────────┤                │
      simulation (ODESolver, TimeResponse, Simulator)         │
          ▲                                                   │
       analysis (FrequencyResponse, RootLocus, Stability, Norms) ──► plot/Plots.hpp
          ▲
   ┌──────┴────────┐
 control        estimation          sysid
 (PID, LeadLag, (Observer, KF,    (ARX, RLS, ETFE, ERA)
  StateFeedback, EKF, UKF)          uses dsp + systems
  LQR, LQG, MPC,
  iLQR, HInf)
```

Data-flow map — "what produces what, and who eats it":

| Produced by | Type | Consumed by |
|---|---|---|
| `Polynomial` arithmetic | `Polynomial` | `TransferFunction`, Routh–Hurwitz, Ackermann, Tustin substitution |
| `TransferFunction`, `StateSpace`, `tf2ss`, `ss2tf`, `c2d`, `d2c`, `series/parallel/feedback`, filter `toTransferFunction()`, `arx`/`era` identification, `lqg`/`hinfsyn` controllers | `LTISystem` (SS or TF) | everything in `analysis`, `simulation`, `control`, `estimation`; `plot` |
| `step`, `impulse`, `lsim`, `integrate`, `Simulator::run` | `SimResult` | `stepInfo`, `plot`, `dsp::welch` (spectrum of a simulated signal), `sysid::arx` (identify from simulated data) |
| `frequencyResponse(sys, w)`, `dsp::etfe(u, y, fs)` | `FrequencyResponse` | `bode`, `nyquist`, `margins`, `plotBode`, `sysid::levy` (fit a TF to it), `hinfnorm` check |
| `signals::*` | `Vector` (time series) | `lsim`, `Simulator`, `arx`, `etfe`, `dsp` filters |
| `dsp::fft` | `ComplexVector` | `periodogram`, `welch`, `etfe`, convolution |
| `lqr`, `place` | gain `Matrix K` | `StateFeedbackController`, `LQG`, `iLQR` (as warm start / terminal) |
| `KalmanFilter` etc. | `Estimator` | `Simulator` (feeds estimate to controller), `LQG` |
| `solveARE` | `Matrix P` | `lqr`, `dlqr`, steady-state Kalman gain, `hinfnorm`, `hinfsyn` |
| `solveQP` | `Vector` | `MPC` |

### 4.5 Build vs. buy — the decision table

| Need | Source | Reason |
|---|---|---|
| Matrices, vectors, arithmetic, transpose, norms, blocks | **Eigen** | Pure infrastructure; Eigen is vectorized and expression-templated |
| LU/QR/SVD/Cholesky/Schur/eigen decompositions, inverse, rank, least squares | **Eigen** | Numerically subtle generic algorithms; decades of hardening |
| Matrix exponential | **Eigen** `unsupported/Eigen/MatrixFunctions` | Generic; used inside the hand-written ZOH discretization |
| Complex numbers, `abs`/`arg`/`polar`/`conj` | **`std::complex`** | Standard, correct, interoperates with Eigen |
| `sin`, `exp`, `log10`, `tan`, `atan2`, `hypot` … | **`<cmath>`** | Generic math |
| Random engines and distributions | **`<random>`** | Statistically correct, standard |
| Containers, `sort`, `max_element`, `accumulate`, `transform` | **STL** | Generic software engineering |
| Unit testing | **Google Test** | Not the learning target |
| Polynomial class and root finding (companion matrix) | **Hand-written** (Eigen does the eigenvalues) | The s-domain representation is control theory; building the companion matrix teaches the link between polynomials and matrices |
| TF/SS representations, conversions, realizations, interconnections | **Hand-written** | This *is* control theory |
| ODE integrators (Euler, RK4; adaptive RK45 as a stretch topic) | **Hand-written** | Understanding what a simulator does is core to judging every later result |
| Frequency response, Bode, Nyquist, margins, root locus, Routh–Hurwitz | **Hand-written** | Core control analysis |
| Discretization (ZOH, Tustin, Euler, MPZ) | **Hand-written** (Eigen does `exp`) | The mapping s→z is the lesson |
| DFT, FFT, windows, PSD, ETFE | **Hand-written** | Explicitly requested; frequency-domain thinking is central |
| FIR/IIR filter structures and Butterworth/notch/FIR design | **Hand-written** | DSP domain knowledge |
| Controllability/observability, Lyapunov, Gramians | **Hand-written** (Eigen rank/solve) | Control theory; Eigen has no Lyapunov solver |
| Riccati solvers (CARE via Hamiltonian, DARE via recursion) | **Hand-written** | The core of optimal control; no mainstream library ships one |
| PID, tuning rules, lead/lag, pole placement, LQR, LQG, MPC, iLQR, H∞ | **Hand-written** | The subject matter |
| Kalman/EKF/UKF, Luenberger observer | **Hand-written** | The subject matter |
| ARX/RLS/ETFE/ERA identification | **Hand-written** (Eigen QR/SVD) | The subject matter |
| QP solver (Hildreth dual method) | **Hand-written** | Understanding the optimization inside MPC is the point; the seam `solveQP(H, f, A, b)` lets you swap in OSQP later |
| Plotting (SVG/HTML writer), CSV | **Hand-written** | Small, dependency-free, a real C++ I/O exercise; needed to *see* results |
| Logging, registries, ring buffers, string utilities | **Not written** | Would be "custom for its own sake" |

---

## 5. Toolchain, build commands and conventions

### 5.1 Tools (what is on this machine, verified)

| Tool | Version | Role |
|---|---|---|
| Visual Studio 2022 Community, MSVC toolset 14.44 | installed | The C++ compiler (`cl.exe`) and linker (`link.exe`) |
| CMake | 4.3.3 | Build-system generator |
| Ninja | bundled with VS (`…\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe`), not on PATH | Fast build driver; used via the "Developer PowerShell" or the VS Code CMake Tools kit |
| VS Code | 1.143 | Editor; install extensions **C/C++** (Microsoft) and **CMake Tools** (Microsoft) on Day 1 |
| Python | 3.13 | Optional only: cross-checking numbers with `numpy`/`scipy`/`python-control` when you want a second opinion. Never a dependency of the library |
| Git | 2.54 | Version control; commit at the end of every day |

GCC/Clang users: every command has a one-line equivalent noted where it differs.

### 5.2 The build commands you will use every day (from Day 4 on)

Open the **"Developer PowerShell for VS 2022"** (Start menu) or run `& "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"` in a plain shell so `cl.exe` and `ninja.exe` are on the PATH. Then, from the repository root:

```powershell
# Configure once (creates build/ ; re-run only when CMakeLists.txt changes — and even then CMake usually re-runs itself)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Build everything (library, tests, examples)
cmake --build build

# Run the tests
ctest --test-dir build --output-on-failure

# Run an example
.\build\examples\ex01_first_order_sim.exe

# A Release build lives in its own directory
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release
```

If you prefer the Visual Studio generator (`cmake -S . -B build` with no `-G`), it is *multi-config*: add `--config Debug` to `cmake --build` and `-C Debug` to `ctest`. The plan assumes Ninja because the commands are shorter.

In VS Code: open the folder, pick the kit "Visual Studio Community 2022 Release - amd64" when CMake Tools asks, then use the status-bar Build/Test buttons or `F7`. Debugging: `F5` with the CMake Tools "Debug" target set to a test or example executable.

### 5.3 Coding conventions

| Item | Convention | Example |
|---|---|---|
| Header extension / source extension | `.hpp` / `.cpp` | `StateSpace.hpp`, `StateSpace.cpp` |
| Include guard | `#pragma once` (first line of every header) | |
| Include path | Always from the include root | `#include "csl/systems/StateSpace.hpp"` |
| Types | `PascalCase` | `TransferFunction` |
| Functions and variables | `camelCase` | `frequencyResponse`, `dcGain` |
| Private members | trailing underscore | `A_`, `num_` |
| Constants | `UPPER_CASE` for the few global ones, otherwise `constexpr` locals | `PI` |
| Namespaces | `csl::<module>` lower-case | `csl::analysis` |
| Pass Eigen objects | by `const&` in, by value out | `Vector f(const Matrix& A)` |
| Never | `using namespace` in headers; raw `new`/`delete`; C-style casts; global mutable state | |
| Test files | `tests/<module>/test_<Header>.cpp`, one `TEST(HeaderName, Property)` per property | `TEST(StateSpace, PolesOfMassSpringDamper)` |
| Commit | end of each day, message `Day NN: <what was built>` | `Day 12: TransferFunction with poles/zeros` |

### 5.4 Numerical tolerance policy (used in every test)

| Kind of result | Tolerance | Examples |
|---|---|---|
| Closed-form / exact algebra | `1e-9` | polynomial products, `tf2ss` matrices, dc gain |
| Decomposition-based | `1e-8` | poles via eigenvalues, Lyapunov solutions |
| Iterative solvers | `1e-6` | ARE, DARE recursion, QP, RLS steady state |
| Simulation vs. analytic | depends on step; stated per test | RK4 at `dt=0.01` vs `e^{-t}`: `1e-6`; Euler: `1e-2` |
| Statistical (noise) | stated as a bound with a fixed seed | "KF RMSE < 0.05 with seed 42" |

---

## 6. The running examples (physical systems used throughout)

The same five systems appear from Day 2 to Day 70, so you always know what "the right answer" should feel like.

| Name | Equations | States / inputs / outputs | Parameters used | First appears | Used for |
|---|---|---|---|---|---|
| **E1 Thermal (first order)** | `τ ẏ = −y + K u` | 1 / 1 / 1 | `τ = 2 s`, `K = 3` | Day 2 | Euler/RK4 accuracy, time constant, first TF `K/(τs+1)`, first Bode, first PID |
| **E2 Mass–spring–damper** | `m ẍ + c ẋ + k x = F` | 2 / 1 / 1 (position) | `m = 1`, `c = 0.5`, `k = 4` → `ωn = 2`, `ζ = 0.125` | Day 4 | Second-order response, poles/damping, resonance on Bode, specs, Routh, filters (as a resonant plant) |
| **E3 DC motor (voltage → shaft speed)** | `L di/dt = −R i − Kb ω + V`; `J dω/dt = Kt i − b ω` | 2 / 1 / 1 (speed) | `R = 1 Ω`, `L = 0.5 H`, `J = 0.01`, `b = 0.1`, `Kt = Kb = 0.01` (the classic CTMS example) | Day 6 | PID speed control, discretization, identification (ARX/RLS/ETFE), Kalman filtering of noisy speed |
| **E4 Double integrator** | `ẍ = u` | 2 / 1 / 1 | — | Day 9 | Textbook checks: `lqr(Q=I, R=1)` gives `K = [1, √3]`; pole placement; MPC with constraints |
| **E5 Cart–pole (inverted pendulum on a cart)** | nonlinear; see Day 47 | 4 / 1 / 2 (cart position, pole angle) | `M = 1`, `m = 0.1`, `l = 0.5`, `g = 9.81` | Day 47 | Linearization, LQR, EKF/UKF, iLQR, MPC — the "modern control" capstone plant |

Known answers you will hit repeatedly: E1 step response is `y(t) = K(1 − e^{−t/τ})`, 63.2 % at `t = τ`; E2 poles are `−0.25 ± 1.984j`, overshoot `≈ 67.3 %`, resonance peak near `2 rad/s`; E3 open-loop poles are `−2.0025` and `−9.9975` (approximately `−2` and `−10`), dc gain from V to ω `= 0.0999`; E4 LQR gain `[1, 1.7321]`.

---
## 7. The day-by-day plan

A note on the `sandbox/` folder: Days 1–3 happen before the library exists, so their programs live in `sandbox/dayNN/`. From Day 4 on, everything is built by CMake inside the real repository layout. Keep `sandbox/` in git — it is your learning log.

Optional reading is given per day as "C++ Primer 5e §x.y" (the book you already own) and as the matching learncpp.com chapter (free, written for beginners, often clearer for a first pass). Control reading refers to Åström & Murray, *Feedback Systems* (free PDF — "AM" below) and Franklin, Powell & Emami-Naeini, *Feedback Control of Dynamic Systems* ("FPE"). None of the reading is required to do the day; the day explains what it needs.

---

### Phase 1 — Foundations: toolchain, C++ basics, first simulations (Days 1–10)

**Phase goal.** You can compile, link and debug a multi-file C++ program; you have a CMake project with a static library, Eigen and Google Test wired in; you have written an ODE integrator, a `StateSpace` class, and you can compute and interpret the poles of a system. In control terms: you understand what a dynamic system is, how to simulate one, and what its poles mean.

---

#### Day 1 — The toolchain, your first program, and what "compile" and "link" mean

**C++ you learn.** What a C++ program is and how text becomes an executable.

A C++ source file (`.cpp`) is plain text. Three tools turn it into a program:

1. The **preprocessor** handles lines starting with `#`. `#include <iostream>` literally pastes the contents of the file `iostream` (part of the compiler's standard library) into your file at that line. After preprocessing, the result is one big chunk of text called a **translation unit**.
2. The **compiler** (`cl.exe` on MSVC, `g++` on GCC) reads one translation unit and produces an **object file** (`.obj` on Windows, `.o` elsewhere): machine code for the functions defined in that file, plus a list of names it *defines* and names it *uses but does not define* ("unresolved externals").
3. The **linker** (`link.exe` / `ld`) takes all object files plus any libraries, matches every "used" name to exactly one "defined" name, and writes the executable.

Keep this picture; Day 3 makes it concrete with two files, and Day 4 shows that CMake is nothing more than a tool that generates these commands for you.

The minimal program:

```cpp
// sandbox/day01/hello.cpp
#include <iostream>          // brings in std::cout and std::endl

int main() {                 // every program has exactly one main(); the OS calls it
    std::cout << "Control Systems Library - Day 1\n";   // << sends text to the console stream
    return 0;                // 0 tells the OS "success"
}
```

Things to notice: every statement ends with `;`. `{ }` group statements into a block. `std::` is a *namespace* prefix — `cout` lives inside the standard library's namespace `std` (you will create your own namespace `csl` on Day 6). `"...\n"` is a string literal; `\n` is a newline.

**Setup (do once).**
1. Confirm Visual Studio 2022 has the "Desktop development with C++" workload (it does — MSVC 14.44 is installed). Open **Developer PowerShell for VS 2022** from the Start menu. Run `cl` — it must print the compiler banner. Run `cmake --version` (4.3.3).
2. In VS Code install the extensions **C/C++** and **CMake Tools** (both by Microsoft).
3. Create `.gitignore` at the repo root with: `build/`, `build-*/`, `*.obj`, `*.exe`, `*.pdb`, `*.ilk`, `.vs/`, `.vscode/`, `*.html` (plots are generated, not versioned), `*.csv`.

**Compile by hand (this is the point of today).** In Developer PowerShell:

```powershell
cd sandbox\day01
cl /EHsc /W4 /std:c++17 hello.cpp      # one step: compile + link
.\hello.exe
```

`/EHsc` enables standard C++ exception handling (always use it on MSVC), `/W4` turns on a high warning level (always), `/std:c++17` selects the language standard. Now do it in two explicit steps to *see* the pipeline:

```powershell
cl /EHsc /W4 /std:c++17 /c hello.cpp   # /c = compile only -> hello.obj
link hello.obj /OUT:hello2.exe         # link -> hello2.exe
.\hello2.exe
```

(GCC equivalent: `g++ -std=c++17 -Wall -Wextra -c hello.cpp` then `g++ hello.o -o hello`.)

Open `hello.obj` size in Explorer; it is machine code for `main` plus references to `std::cout`. The linker resolved those references against the C++ runtime library automatically.

**Control you learn.** What a *system* is, and the vocabulary used for the next 70 days.

- A **dynamic system** is anything whose output *now* depends on its inputs *in the past*, not just the input now. A resistor is not dynamic (`v = iR`, instantaneous). A capacitor is (`i = C dv/dt`), a car is, a room's temperature is.
- The **input** `u(t)` is what you can manipulate (heater power, motor voltage, force). The **output** `y(t)` is what you measure (temperature, speed, position). The **state** `x(t)` is the minimal set of numbers that, together with future inputs, determines the whole future: for a room it is the temperature; for a mass on a spring it is position *and* velocity.
- **Open loop**: choose `u(t)` in advance. **Closed loop / feedback**: measure `y`, compare with a reference `r`, compute `u` from the error `e = r − y`. Almost everything in the library is about doing that well: fast, accurate, stable, and robust to things you did not model.
- The plan's five running systems (Section 6). Read their equations now; you will simulate E1 tomorrow.

Reading: AM Chapter 1 (what feedback is — read it today, it is short and motivating). C++ Primer §1.1–1.2; learncpp 0.1–0.8 and 1.1–1.4.

**Files you touch.** `sandbox/day01/hello.cpp`, `.gitignore`.

**Verify.** `hello.exe` prints the line. You can explain, in your own words, what `hello.obj` is and why `link` needed it. Deliberately rename `main` to `mian`, compile with `/c` (succeeds — the compiler does not care), then link (fails: "unresolved external symbol main"). Understand why the error comes from the linker, not the compiler.

**Done when.**
- [ ] `cl` and `cmake` run from Developer PowerShell
- [ ] `hello.exe` built both in one step and in two steps
- [ ] You produced and understood one linker error on purpose
- [ ] `git commit -m "Day 1: toolchain and hello"`

---

#### Day 2 — Types, variables, arithmetic, loops: simulating the thermal system with Euler's method

**C++ you learn.** The basic value types and the statements you will use in every file from now on.

- **Fundamental types**: `int` (whole numbers), `double` (64-bit floating point — the library's `Real`), `bool` (`true`/`false`), `char`, `std::size_t` (unsigned size type). Prefer `double` for all physics; never `float`.
- **Declaration and initialization**: `double tau = 2.0;` or the modern brace form `double tau{2.0};` which refuses silent narrowing (`int n{2.5};` is a compile error, `int n = 2.5;` silently truncates — use braces when in doubt).
- **`const`**: `const double K = 3.0;` cannot be changed after initialization. Make everything `const` that does not change; the compiler then catches mistakes for you. `constexpr` (compile-time constant) is the stronger form for true constants.
- **Arithmetic traps**: `1/2` is integer division and equals `0`; `1.0/2` is `0.5`. `%` is remainder on integers only.
- **Loops and branches**: `for (int k = 0; k < N; ++k) { }`, `while (cond) { }`, `if (cond) { } else { }`. `++k` increments.
- **`<cmath>`**: `std::exp`, `std::sqrt`, `std::sin`, `std::pow`, `std::abs`. These are Ground Rule 1 in action: you never write your own exponential.
- **Printing numbers**: `#include <iomanip>` then `std::cout << std::fixed << std::setprecision(6) << value;`.

**Control you learn.** The first-order system (E1) and Euler's method — the simplest possible simulator.

A room heated by a heater: heat in is `K·u` (u is heater power), heat lost is proportional to temperature `y` above ambient. Energy balance gives

```
τ · dy/dt = −y + K·u          (τ: time constant [s], K: steady-state gain)
```

With a unit step input `u = 1` from rest, the exact solution is `y(t) = K (1 − e^{−t/τ})`. At `t = τ` the output has reached 63.2 % of its final value `K`; at `t = 4τ`, 98.2 %. Memorize that: "time constant" means "63 % in τ seconds, settled in about 4τ".

Euler's method turns the derivative into a finite difference: `dy/dt ≈ (y_{k+1} − y_k)/dt`, so

```
y_{k+1} = y_k + dt · f(y_k, u_k),   with  f(y, u) = (−y + K·u) / τ
```

It is first-order accurate: halve `dt` and the error halves. That is bad enough that Day 7 replaces it with RK4, but it is the right first simulator because you can see every step.

**Code.** The complete program:

```cpp
// sandbox/day02/thermal_euler.cpp
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    // System E1: tau * dy/dt = -y + K*u
    const double tau = 2.0;   // s
    const double K   = 3.0;
    const double u   = 1.0;   // unit step input, applied from t = 0

    // Simulation settings
    const double dt    = 0.1;   // step size [s]  -- try 0.5, 1.0, 3.0, 5.0 later
    const double t_end = 10.0;
    const int    N     = static_cast<int>(t_end / dt);   // explicit conversion: double -> int

    double y = 0.0;             // state: starts at rest
    double t = 0.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t,y_euler,y_exact,error\n";
    for (int k = 0; k <= N; ++k) {
        const double y_exact = K * (1.0 - std::exp(-t / tau));
        std::cout << t << "," << y << "," << y_exact << "," << (y - y_exact) << "\n";

        // Euler step: y_{k+1} = y_k + dt * f(y_k, u)
        const double dydt = (-y + K * u) / tau;
        y += dt * dydt;
        t += dt;
    }
    return 0;
}
```

Build and run exactly as yesterday; redirect the output to a file so you can look at it in Excel/Python: `.\thermal_euler.exe > out.csv`.

**Experiments (do all four; they are the control lesson).**
1. `dt = 0.1`: error at `t = 2` is about `+0.028` (Euler runs *ahead* of the true curve: `(1 − dt/τ)^k` decays faster than `e^{−k·dt/τ}`).
2. `dt = 0.01`: error drops to about `+0.0027` — ten times smaller for ten times smaller step. First-order accuracy, observed.
3. `dt = 3.0`: the solution oscillates. `dt = 5.0`: it blows up. Euler on `ẏ = −y/τ` is stable only for `dt < 2τ = 4 s`. **A simulator can be unstable even when the system is stable.** You will meet this again on Day 7 (RK4 stability region) and Day 29 (sampling).
4. Change `K` and `τ`; confirm 63.2 % at `t = τ` every time.

**How it connects.** Tomorrow this loop becomes a reusable function in its own file. Day 7 generalizes it to vectors and RK4; Day 15 wraps it as `step()`. The 63 %/4τ rule reappears on Day 16 (specs) and Day 17 (the corner frequency `1/τ` on a Bode plot is the same number seen from the frequency side).

**Verify.** With `dt = 0.1`: `y(2.0) ≈ 1.9245` (Euler) vs `1.8964` (exact); with `dt = 0.01`: `1.8991` vs `1.8964`. (Closed form for Euler here: `y_k = K(1 − (1 − dt/τ)^k)` — derive it, it is two lines.)

Reading: C++ Primer §2.1–2.3, §5.1–5.4; learncpp chapters 4, 5, 8 (skim). Control: AM §2.1–2.2, FPE §2.1.

**Done when.**
- [ ] Program compiles with `/W4` and no warnings
- [ ] All four experiments done and you can state Euler's stability limit for E1
- [ ] Commit

---

#### Day 3 — Functions, header files, and linking two translation units

**C++ you learn.** How code is split across files and how the pieces find each other. This is the single most confusing topic for newcomers and the one you asked to understand deeply, so today is only about it.

- A **function declaration** (prototype) tells the compiler a function's name, parameter types and return type: `double eulerStep(double y, double u, double dt);`. A **definition** is the declaration plus the body. You may declare many times, define exactly once (the **One Definition Rule**).
- A **header file** (`.hpp`) holds declarations. A **source file** (`.cpp`) holds definitions. Any `.cpp` that wants to call the function `#include`s the header — remember, `#include` is a textual paste — and so sees the declaration. The compiler is satisfied by the declaration alone; it emits a call to a name it trusts the *linker* to find.
- `#include "euler.hpp"` (quotes) searches the current directory first; `#include <iostream>` (angle brackets) searches the system/compiler include paths. Your library headers always use quotes with a path from the include root (`"csl/simulation/ODESolver.hpp"`) — set up on Day 4.
- **Include guards.** If a header is included twice in one translation unit (very common once headers include headers), its declarations would appear twice. Classes declared twice are an error. `#pragma once` as the first line of every header prevents the second paste. (The portable alternative is `#ifndef X / #define X / #endif`; `#pragma once` is supported by every compiler you will meet.)
- **Parameters by value** copy the argument: `double eulerStep(double y, …)` gets its own `y`; modifying it does not affect the caller. Day 5 adds references; for `double`s, copies are correct and cheap.
- Each `.cpp` is compiled separately into its own `.obj`. The linker joins them. Change one `.cpp`, recompile only that one — this is why large projects use many source files.

**Control you learn.** Euler's method as an *algorithm* with properties, separate from the system it integrates.

Local truncation error of Euler is `O(dt²)` per step; over `N = T/dt` steps the global error is `O(dt)`. Its **stability region**: for the test equation `ẏ = λy` (λ real negative), Euler gives `y_{k+1} = (1 + λ dt) y_k`, which decays only if `|1 + λ dt| < 1`, i.e. `dt < 2/|λ|`. For E1, `λ = −1/τ`, so `dt < 2τ` — exactly what you observed yesterday. This is the first time the library's thinking appears: *a number (λ, soon called a pole) determines behaviour, and we reason about it rather than just running the simulation.*

**Code.**

```cpp
// sandbox/day03/euler.hpp
#pragma once

// One explicit Euler step for the scalar first-order system  tau * dy/dt = -y + K*u.
// Returns y at the next time step.
double eulerStepThermal(double y, double u, double dt, double tau, double K);

// Largest stable Euler step for this system (2*tau).
double eulerStabilityLimit(double tau);
```

```cpp
// sandbox/day03/euler.cpp
#include "euler.hpp"

double eulerStepThermal(double y, double u, double dt, double tau, double K) {
    const double dydt = (-y + K * u) / tau;
    return y + dt * dydt;
}

double eulerStabilityLimit(double tau) {
    return 2.0 * tau;
}
```

```cpp
// sandbox/day03/main.cpp
#include <cmath>
#include <iomanip>
#include <iostream>
#include "euler.hpp"          // declarations only; the definitions live in euler.cpp

int main() {
    const double tau = 2.0, K = 3.0, u = 1.0;
    const double dt = 0.1, t_end = 10.0;
    std::cout << "Euler is stable for dt < " << eulerStabilityLimit(tau) << " s\n";

    double y = 0.0;
    std::cout << std::fixed << std::setprecision(6);
    for (double t = 0.0; t <= t_end + 1e-12; t += dt) {
        std::cout << t << "," << y << "," << K * (1.0 - std::exp(-t / tau)) << "\n";
        y = eulerStepThermal(y, u, dt, tau, K);
    }
    return 0;
}
```

Build by hand, in three commands, and watch the pipeline:

```powershell
cl /EHsc /W4 /std:c++17 /c euler.cpp     # -> euler.obj   (defines eulerStepThermal, eulerStabilityLimit)
cl /EHsc /W4 /std:c++17 /c main.cpp      # -> main.obj    (defines main; USES the two functions)
link euler.obj main.obj /OUT:day03.exe   # resolves main.obj's uses against euler.obj's definitions
.\day03.exe
```

Then break it three ways and read each error until it makes sense:
1. Link only `main.obj` → *unresolved external symbol eulerStepThermal*. The compiler was happy (it saw the declaration); the linker could not find the definition.
2. Remove `#include "euler.hpp"` from `main.cpp` → the *compiler* fails: *identifier not found*. Different stage, different error.
3. Copy the body of `eulerStepThermal` into `main.cpp` as well, keep both `.obj`s → *already defined in euler.obj* (ODR violation at link time).

**How it connects.** Tomorrow CMake issues these exact `cl` and `link` commands for you. The header/source split you did by hand is the pattern of every one of the ~45 headers in the library.

**Verify.** Output identical to Day 2 for `dt = 0.1`. All three deliberate errors produced and explained.

Reading: C++ Primer §6.1–6.3, §2.6.3 (headers), §6.1.3; learncpp chapter 2 (all of it — this is its best chapter).

**Done when.**
- [ ] Three-command manual build works
- [ ] You can say which *stage* produces each of the three errors and why
- [ ] Commit

---

#### Day 4 — CMake from scratch, the library skeleton, and the second-order system

**C++ you learn.** CMake. Yesterday's three commands do not scale to 45 headers; CMake is a *generator*: it reads `CMakeLists.txt`, works out the dependency graph, and writes build files (for Ninja, Visual Studio, Make…) that issue the compiler/linker commands. Key ideas:

- Everything in modern CMake is a **target**: an executable (`add_executable`) or a library (`add_library`). You attach properties to targets: include directories, compile features, other targets to link.
- `target_link_libraries(app PRIVATE csl)` means "app uses csl": the linker gets `csl.lib`, *and* app inherits csl's `PUBLIC` include directories and compile features. `PUBLIC` = "I use this and so does anyone who uses me"; `PRIVATE` = "only I use this"; `INTERFACE` = "only my users".
- A **static library** (`add_library(csl STATIC ...)`) is just an archive of `.obj` files (`csl.lib`). Linking against it is the same as linking the `.obj`s directly, which is why Day 3 mattered.
- Builds are **out-of-source**: `cmake -S . -B build` puts everything generated into `build/`, which is gitignored. Deleting `build/` is always safe.
- `CMAKE_BUILD_TYPE` (`Debug`: no optimization, debug symbols, assertions on; `Release`: `/O2`, fast). Use Debug while developing, Release for timing.

**Control you learn.** The mass–spring–damper (E2) and the idea of a *state vector*.

`m ẍ + c ẋ + k x = F`. This is second order; to simulate it with a first-order method, introduce the state `x₁ = x` (position), `x₂ = ẋ` (velocity):

```
ẋ₁ = x₂
ẋ₂ = (F − c x₂ − k x₁) / m
```

Two coupled first-order equations. Euler now updates both: `x₁ += dt·x₂; x₂ += dt·(F − c·x₂ − k·x₁)/m` — note you must compute both derivatives from the *old* state before updating either (a classic bug: updating `x₁` first and then using the new `x₁` in `ẋ₂` is a different, "semi-implicit" method). Two numbers that belong together is the motivation for arrays (tomorrow) and for Eigen vectors (Day 6).

Define `ωn = √(k/m)` (natural frequency, rad/s) and `ζ = c / (2√(km))` (damping ratio). With `m = 1, c = 0.5, k = 4`: `ωn = 2`, `ζ = 0.125` — lightly damped, so the step response rings. Tomorrow you look at it.

**Files you touch.** The real repository layout is born today:

```
CMakeLists.txt
include/csl/simulation/ODESolver.hpp     (scalar Euler, promoted from sandbox)
src/simulation/ODESolver.cpp
examples/CMakeLists.txt
examples/ex01_first_order_sim.cpp
```

**Code.**

```cmake
# CMakeLists.txt  (repository root)
cmake_minimum_required(VERSION 3.20)
project(csl VERSION 2.0.0 LANGUAGES CXX)

# --- Global compiler settings --------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)            # -std=c++17, not gnu++17

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
    add_compile_options(/W4 /permissive- /EHsc /Zc:__cplusplus)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- The library ----------------------------------------------------------
add_library(csl STATIC
    src/simulation/ODESolver.cpp
)
add_library(csl::csl ALIAS csl)          # the name users link against

target_include_directories(csl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(csl PUBLIC cxx_std_17)

# --- Examples ---------------------------------------------------------------
option(CSL_BUILD_EXAMPLES "Build example programs" ON)
if(CSL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
```

```cmake
# examples/CMakeLists.txt
add_executable(ex01_first_order_sim ex01_first_order_sim.cpp)
target_link_libraries(ex01_first_order_sim PRIVATE csl::csl)
```

```cpp
// include/csl/simulation/ODESolver.hpp   (Day 4 version; Day 7 replaces the contents)
#pragma once

namespace csl::simulation {

// Explicit Euler step for the scalar ODE  dy/dt = f  where f is already evaluated.
// Kept deliberately trivial today; it exists so the build has one real library source.
double eulerStepScalar(double y, double dydt, double dt);

} // namespace csl::simulation
```

```cpp
// src/simulation/ODESolver.cpp
#include "csl/simulation/ODESolver.hpp"

namespace csl::simulation {

double eulerStepScalar(double y, double dydt, double dt) {
    return y + dt * dydt;
}

} // namespace csl::simulation
```

`namespace csl::simulation { ... }` is explained properly on Day 6; for today, read it as "these names live in a folder called `csl::simulation`", so the call is `csl::simulation::eulerStepScalar(...)`.

```cpp
// examples/ex01_first_order_sim.cpp   (Day 4 version)
#include <iomanip>
#include <iostream>
#include "csl/simulation/ODESolver.hpp"

int main() {
    // E2: mass-spring-damper, state (x1, x2) = (position, velocity)
    const double m = 1.0, c = 0.5, k = 4.0, F = 1.0;
    const double dt = 0.001, t_end = 20.0;

    double x1 = 0.0, x2 = 0.0;
    std::cout << std::fixed << std::setprecision(6) << "t,x,v\n";
    for (double t = 0.0; t <= t_end; t += dt) {
        std::cout << t << "," << x1 << "," << x2 << "\n";
        const double dx1 = x2;                              // both derivatives from the OLD state
        const double dx2 = (F - c * x2 - k * x1) / m;
        x1 = csl::simulation::eulerStepScalar(x1, dx1, dt);
        x2 = csl::simulation::eulerStepScalar(x2, dx2, dt);
    }
    return 0;
}
```

**Build.**

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
.\build\examples\ex01_first_order_sim.exe > msd.csv
```

Open `build/build.ninja` and find the `cl.exe` line for `ODESolver.cpp` and the `lib.exe`/`link.exe` lines — they are Day 3's commands, generated. Run `cmake --build build -v` once to see them scroll by.

**How it connects.** Every later day adds files to the `add_library(csl STATIC ...)` list (or a `target_sources` call) and, from Day 10, to `tests/`. Nothing else about the build changes until Eigen (Day 6) and Google Test (Day 10) are added.

**Verify.** `msd.csv` shows position oscillating around the final value `F/k = 0.25` with period `2π/ωd ≈ 3.17 s` (`ωd = ωn√(1−ζ²) = 1.984`), decaying slowly. The first peak is at about `t ≈ 1.58 s` with value `≈ 0.418` (overshoot 67 %).

Reading: the CMake tutorial steps 1–2 (cmake.org/cmake/help/latest/guide/tutorial); learncpp 2.11 (header files and CMake-less projects, for contrast). Control: FPE §2.1 (mechanical systems), AM §2.2.

**Done when.**
- [ ] `cmake --build build` succeeds and you can point to the generated compile/link lines
- [ ] You can explain `PUBLIC` vs `PRIVATE` in `target_link_libraries` in one sentence
- [ ] `msd.csv` has the right final value and period
- [ ] Commit

---

#### Day 5 — `std::vector`, references, strings and files: writing CSV, reading the damped response

**C++ you learn.**

- **`std::vector<T>`** (`#include <vector>`) is a resizable array that owns its memory. `std::vector<double> v;  v.push_back(1.0);  v.size();  v[0];  v.back();`. Construct with a size: `std::vector<double> v(N, 0.0);`. It cleans up after itself when it goes out of scope (this is RAII, named properly on Day 24).
- **Range-for**: `for (double x : v) { }` copies each element; `for (const double& x : v)` reads each element without copying; `for (double& x : v) x *= 2;` modifies in place.
- **References** (`T&`): an alias for an existing object, not a copy. `const std::vector<double>& v` as a parameter means "let me *read* the caller's vector without copying it". This is how every Eigen object is passed in the library from Day 6 on. A non-const reference parameter (`std::vector<double>& out`) lets a function fill the caller's container.
- **`std::string`** (`#include <string>`): `std::string s = "t";  s += ",y";  s.size();`. String concatenation with `+`. `std::to_string(3.5)`.
- **File output**: `#include <fstream>`, `std::ofstream file("out.csv"); file << "a,b\n";` — a file stream works exactly like `std::cout`. The file closes when `file` goes out of scope. Check `if (!file) { ... }` after opening.
- **`std::array<T, N>`** is a fixed-size array (size known at compile time) — useful for small fixed states but Eigen replaces it tomorrow.

**Control you learn.** The three regimes of a second-order system and the numbers that describe them.

For `ẍ + 2ζωn ẋ + ωn² x = ωn² u`:
- `ζ < 1` **underdamped**: oscillates at `ωd = ωn√(1−ζ²)`, envelope `e^{−ζωn t}`, overshoot `Mp = exp(−πζ/√(1−ζ²))`.
- `ζ = 1` **critically damped**: fastest with no overshoot.
- `ζ > 1` **overdamped**: two real time constants, sluggish.

Rules of thumb you will test on Day 16: 2 % settling time `≈ 4/(ζωn)`, peak time `π/ωd`, rise time `≈ 1.8/ωn` (for ζ ≈ 0.5). With E2's numbers: `Mp = 67.3 %`, `tp = 1.583 s`, `ts ≈ 16 s`.

**Files you touch.** `include/csl/plot/CSV.hpp`, `src/plot/CSV.cpp`, `examples/ex01_first_order_sim.cpp` (rewritten with vectors and CSV output), root `CMakeLists.txt` (add `src/plot/CSV.cpp`).

**Code.**

```cpp
// include/csl/plot/CSV.hpp
#pragma once
#include <string>
#include <vector>

namespace csl::plot {

// Write columns of equal length to a CSV file with a header row.
// Day 5 version: plain std::vector columns. Day 6 adds an overload taking an Eigen Matrix.
void writeCSV(const std::string& path,
              const std::vector<std::string>& headers,
              const std::vector<std::vector<double>>& columns);

} // namespace csl::plot
```

```cpp
// src/plot/CSV.cpp
#include "csl/plot/CSV.hpp"
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace csl::plot {

void writeCSV(const std::string& path,
              const std::vector<std::string>& headers,
              const std::vector<std::vector<double>>& columns) {
    if (headers.size() != columns.size())
        throw std::runtime_error("writeCSV: headers/columns count mismatch");
    std::ofstream file(path);
    if (!file) throw std::runtime_error("writeCSV: cannot open " + path);

    file << std::setprecision(10);
    for (std::size_t j = 0; j < headers.size(); ++j)
        file << headers[j] << (j + 1 < headers.size() ? "," : "\n");

    const std::size_t rows = columns.empty() ? 0 : columns[0].size();
    for (std::size_t i = 0; i < rows; ++i)
        for (std::size_t j = 0; j < columns.size(); ++j)
            file << columns[j][i] << (j + 1 < columns.size() ? "," : "\n");
}

} // namespace csl::plot
```

(`throw std::runtime_error(...)` is an exception — the program stops with that message unless someone catches it. Day 8 explains exceptions properly and replaces these with the library's own `csl::Error` types.)

```cpp
// examples/ex01_first_order_sim.cpp   (Day 5 version)
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "csl/plot/CSV.hpp"
#include "csl/simulation/ODESolver.hpp"

// Simulate E2 with Euler for a given damping ratio; fill t, x by reference.
void simulateMSD(double zeta, double wn, double dt, double t_end,
                 std::vector<double>& t, std::vector<double>& x) {
    t.clear(); x.clear();
    double x1 = 0.0, x2 = 0.0;
    for (double tk = 0.0; tk <= t_end; tk += dt) {
        t.push_back(tk); x.push_back(x1);
        const double dx1 = x2;
        const double dx2 = wn * wn * 1.0 - 2.0 * zeta * wn * x2 - wn * wn * x1;   // unit step in u
        x1 = csl::simulation::eulerStepScalar(x1, dx1, dt);
        x2 = csl::simulation::eulerStepScalar(x2, dx2, dt);
    }
}

int main() {
    const double wn = 2.0, dt = 0.001, t_end = 12.0;
    const std::vector<double> zetas = {0.125, 0.5, 1.0, 2.0};

    std::vector<std::string> headers = {"t"};
    std::vector<std::vector<double>> cols(1);
    for (double zeta : zetas) {
        std::vector<double> t, x;
        simulateMSD(zeta, wn, dt, t_end, t, x);
        if (cols[0].empty()) cols[0] = t;          // time column once
        headers.push_back("zeta=" + std::to_string(zeta));
        cols.push_back(x);
        const double peak = *std::max_element(x.begin(), x.end());   // needs <algorithm>
        std::cout << "zeta=" << zeta << "  peak=" << peak << "  overshoot=" << (peak - 1.0) * 100 << "%\n";
    }
    csl::plot::writeCSV("msd_zeta_sweep.csv", headers, cols);
    return 0;
}
```

(`std::max_element` returns an *iterator* to the largest element; `*` reads the value. Iterators get their own treatment on Day 16; for today, "a pointer-like handle to an element".) Add `#include <algorithm>`.

**How it connects.** `writeCSV` is used for the rest of the project (every `SimResult` can be dumped). Tomorrow it gets an Eigen overload. The plotter (Day 15) replaces "open the CSV in Excel" with "open the HTML".

**Verify.** Printed overshoots: `ζ = 0.125 → ≈ 67.2 %`, `0.5 → ≈ 16.3 %`, `1.0 → ≈ 0 %`, `2.0 → 0 %`. Open the CSV (Excel, or `python -c "import pandas as pd; pd.read_csv('msd_zeta_sweep.csv').plot(x='t'); import matplotlib.pyplot as plt; plt.show()"`) and see the four shapes.

Reading: C++ Primer §3.1–3.4, §2.3.1 (references), §8.1–8.2 (streams); learncpp 12.1–12.5 (references), 16.1–16.3 (vector), 28.1–28.6 (file I/O). Control: FPE §3.3–3.4 (second-order response), AM §6.3.

**Done when.**
- [ ] `writeCSV` compiled into `csl` and used from the example
- [ ] You can explain why `simulateMSD` takes `std::vector<double>&` and `writeCSV` takes `const std::vector<...>&`
- [ ] Four overshoots match
- [ ] Commit

---
#### Day 6 — Eigen, namespaces, type aliases, overloading: the state-space form

**C++ you learn.**

- **Namespaces.** `namespace csl { ... }` puts every name declared inside it behind the prefix `csl::`. Nested: `namespace csl::simulation { }`. The purpose is collision avoidance (your `Vector` vs. someone else's `Vector`) and organisation. Inside a `.cpp` you may write `using namespace csl;` for brevity; **never** in a header, because the header is pasted into everyone's files and would force your shortcut on them.
- **Type aliases.** `using Real = double;` makes `Real` another name for `double`. The library's `Types.hpp` (Section 4.3.1) is nothing but aliases and constants. The point: write `Matrix` everywhere, and the choice of Eigen type is made in one line.
- **`inline constexpr`** for header constants: `constexpr` = computed at compile time; `inline` = may appear in many translation units without violating the ODR (C++17).
- **Eigen basics.** `#include <Eigen/Dense>`. `Matrix A(2, 2); A << 0, 1, -4, -0.5;` fills row-wise. `Vector x = Vector::Zero(2);` `A * x`, `A.transpose()`, `A.inverse()`, `A(i, j)`, `x(i)` or `x[i]`, `A.rows()`, `A.cols()`, `x.size()`, `x.norm()`, `Matrix::Identity(n, n)`. Printing: `std::cout << A`. Blocks: `A.block(row, col, nrows, ncols)`, `A.topLeftCorner(n, n)`, `A.col(j)`, `A.row(i)`. Always pass Eigen objects as `const Matrix&`; return them by value (Eigen moves them, no copy cost).
- **Function overloading.** Two functions may share a name if their parameter lists differ; the compiler picks by argument types. Today: a second `writeCSV` taking a `Matrix`.
- **CMake FetchContent.** CMake downloads a dependency at configure time and makes its targets available. Eigen is *header-only*: there is nothing to compile, only an include directory to add.

**Control you learn.** The **state-space representation** — the form every "modern control" algorithm in the library consumes.

```
ẋ = A x + B u        x ∈ ℝⁿ states,  u ∈ ℝᵐ inputs
y = C x + D u        y ∈ ℝᵖ outputs
```

`A` (n×n) is the system's internal dynamics — its eigenvalues decide stability (Day 9). `B` (n×m) says how inputs push on states. `C` (p×n) says which combination of states you measure. `D` (p×m) is direct feedthrough (usually zero for physical systems; nonzero for a PID controller's P term, or a filter with instantaneous response).

Write the three continuous examples in this form; you will type these matrices into tests for the next 60 days.

E1 thermal: `A = [−1/τ]`, `B = [K/τ]`, `C = [1]`, `D = [0]`.
E2 mass–spring–damper (state = position, velocity; output = position):
```
A = [ 0      1   ]    B = [ 0  ]    C = [1 0]    D = [0]
    [−k/m  −c/m ]        [1/m ]
```
E3 DC motor (state = current, angular speed; input = voltage; output = speed):
```
A = [ −R/L   −Kb/L ]    B = [ 1/L ]    C = [0 1]    D = [0]
    [  Kt/J   −b/J ]        [  0  ]
```

Why this form and not just "the ODE": it separates *structure* (A, B, C, D) from *data* (x, u), so one algorithm (`integrate`, `lqr`, `KalmanFilter`…) works for every system. A first-order system and a 100-state flexible structure differ only in the sizes of four matrices.

**Files you touch.** Root `CMakeLists.txt` (Eigen), `include/csl/core/Types.hpp` (new, exactly Section 4.3.1), `include/csl/plot/CSV.hpp` + `src/plot/CSV.cpp` (Matrix overload), `examples/plants.hpp` (new), `examples/ex01_first_order_sim.cpp` (use Eigen).

**Build.** Add to the root `CMakeLists.txt`, before `add_library(csl ...)`:

```cmake
# --- Eigen (header-only) ----------------------------------------------------
include(FetchContent)
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR cmake_not_used      # no CMakeLists.txt here => Eigen's own build is NOT added
)
FetchContent_MakeAvailable(eigen)
add_library(eigen INTERFACE)          # an INTERFACE library has no sources; it only carries usage requirements
add_library(Eigen3::Eigen ALIAS eigen)
target_include_directories(eigen INTERFACE ${eigen_SOURCE_DIR})
```

and after `add_library(csl ...)`: `target_link_libraries(csl PUBLIC Eigen3::Eigen)`. `PUBLIC` because `Types.hpp` includes Eigen, so every user of `csl` needs Eigen's include path too. Re-run `cmake -S . -B build` once; the download happens then (`build/_deps/eigen-src`). (Alternative: `GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0` without `SOURCE_SUBDIR` adds Eigen's own CMake project; it works but configures slower and may need `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` with CMake 4.)

**Code.**

```cpp
// include/csl/plot/CSV.hpp  — add this overload (keep the std::vector one)
#include "csl/core/Types.hpp"
namespace csl::plot {
// Each column of `data` is one signal; data.rows() == t.size(). Writes t as the first column.
void writeCSV(const std::string& path, const std::vector<std::string>& headers,
              const Vector& t, const Matrix& data);
}
```

```cpp
// src/plot/CSV.cpp — the overload
void writeCSV(const std::string& path, const std::vector<std::string>& headers,
              const Vector& t, const Matrix& data) {
    if (data.rows() != t.size())
        throw std::runtime_error("writeCSV: data.rows() != t.size()");
    if (static_cast<Index>(headers.size()) != data.cols() + 1)
        throw std::runtime_error("writeCSV: need one header per column plus one for t");
    std::ofstream file(path);
    if (!file) throw std::runtime_error("writeCSV: cannot open " + path);
    file << std::setprecision(10);
    for (std::size_t j = 0; j < headers.size(); ++j)
        file << headers[j] << (j + 1 < headers.size() ? "," : "\n");
    for (Index i = 0; i < t.size(); ++i) {
        file << t(i);
        for (Index j = 0; j < data.cols(); ++j) file << "," << data(i, j);
        file << "\n";
    }
}
```

```cpp
// examples/plants.hpp — the running examples as (A, B, C, D) factories.
// Shared by examples and tests. Day 8 turns the return type into csl::systems::StateSpace.
#pragma once
#include "csl/core/Types.hpp"

namespace plants {

struct ABCD { csl::Matrix A, B, C, D; };

inline ABCD thermal(double tau = 2.0, double K = 3.0) {
    ABCD s;
    s.A = csl::Matrix(1, 1); s.A << -1.0 / tau;
    s.B = csl::Matrix(1, 1); s.B << K / tau;
    s.C = csl::Matrix(1, 1); s.C << 1.0;
    s.D = csl::Matrix::Zero(1, 1);
    return s;
}

inline ABCD massSpringDamper(double m = 1.0, double c = 0.5, double k = 4.0) {
    ABCD s;
    s.A = csl::Matrix(2, 2); s.A << 0.0, 1.0, -k / m, -c / m;
    s.B = csl::Matrix(2, 1); s.B << 0.0, 1.0 / m;
    s.C = csl::Matrix(1, 2); s.C << 1.0, 0.0;
    s.D = csl::Matrix::Zero(1, 1);
    return s;
}

inline ABCD dcMotor(double R = 1.0, double L = 0.5, double J = 0.01, double b = 0.1,
                    double Kt = 0.01, double Kb = 0.01) {
    ABCD s;
    s.A = csl::Matrix(2, 2); s.A << -R / L, -Kb / L, Kt / J, -b / J;
    s.B = csl::Matrix(2, 1); s.B << 1.0 / L, 0.0;
    s.C = csl::Matrix(1, 2); s.C << 0.0, 1.0;
    s.D = csl::Matrix::Zero(1, 1);
    return s;
}

inline ABCD doubleIntegrator() {
    ABCD s;
    s.A = csl::Matrix(2, 2); s.A << 0.0, 1.0, 0.0, 0.0;
    s.B = csl::Matrix(2, 1); s.B << 0.0, 1.0;
    s.C = csl::Matrix(1, 2); s.C << 1.0, 0.0;
    s.D = csl::Matrix::Zero(1, 1);
    return s;
}

} // namespace plants
```

(`inline` on a function defined in a header is what lets several `.cpp` files include it without an ODR clash — the same rule as `inline constexpr` for constants.)

```cpp
// examples/ex01_first_order_sim.cpp   (Day 6 version: Euler on ẋ = Ax + Bu with Eigen)
#include <iostream>
#include "csl/core/Types.hpp"
#include "csl/plot/CSV.hpp"
#include "plants.hpp"

int main() {
    using namespace csl;                         // fine in a .cpp
    const auto p = plants::massSpringDamper();   // auto: deduce the type (plants::ABCD)
    const Real dt = 0.001, t_end = 12.0;
    const Index N = static_cast<Index>(t_end / dt) + 1;

    Vector t(N);
    Matrix X(N, 2);                              // time along rows, one column per state
    Vector x = Vector::Zero(2);
    Vector u(1); u << 1.0;                       // unit step force

    for (Index k = 0; k < N; ++k) {
        t(k) = k * dt;
        X.row(k) = x.transpose();
        x = x + dt * (p.A * x + p.B * u);        // Euler, vector form -- one line for any order n
    }
    plot::writeCSV("msd_eigen.csv", {"t", "x", "v"}, t, X);
    std::cout << "final position = " << X(N - 1, 0) << " (expected F/k = 0.25)\n";
    return 0;
}
```

**How it connects.** `Types.hpp` is included by every header from now on. `plants.hpp` feeds every test. The expression `p.A * x + p.B * u` is the right-hand side that tomorrow becomes a lambda handed to a generic integrator.

**Verify.** `msd_eigen.csv` matches Day 4's `msd.csv` to printing precision. `(p.C * (-p.A.inverse() * p.B))(0,0)` printed for the DC motor equals `0.0999` — this is the dc gain, explained on Day 8.

Reading: Eigen "Getting started" and "The Matrix class" pages (eigen.tuxfamily.org/dox); C++ Primer §2.5 (aliases), §18.2 (namespaces), §6.4 (overloading); learncpp 7.2–7.3, 11.1–11.3. Control: AM §2.2 and §6.1 (state-space models), FPE §7.2.

**Done when.**
- [ ] Eigen downloaded by CMake; `Types.hpp` compiles
- [ ] You can state why `using namespace` is banned in headers
- [ ] `plants.hpp` has E1–E4; dc motor dc gain printed as `0.0999`
- [ ] Commit

---

#### Day 7 — Lambdas, `std::function`, and the generic integrator (Euler, RK4)

**C++ you learn.** Passing *behaviour* into a function.

- A **lambda** is an anonymous function written in-place: `auto f = [](Real t, const Vector& x) { return -x; };`. The `[]` is the **capture list**: `[&]` lets the body use surrounding variables by reference, `[=]` by copy, `[&A, &B]` names them. Captured-by-reference variables must outlive the lambda (fine when the lambda is used immediately, which is our case).
- **`std::function<R(Args...)>`** (`#include <functional>`) is a type that can hold *any* callable with that signature: a lambda, a plain function, a functor. Our dynamics type is `std::function<Vector(Real, const Vector&)>`. There is a small cost per call (an indirect call); for simulation it is irrelevant, and the flexibility is exactly what a library needs.
- **`auto`**: lets the compiler deduce a type. Use it for lambdas (their type is unnameable), iterators and long template types; write the type explicitly when it helps the reader (`Vector x = ...`).
- **`struct`** (first of several uses): a class whose members are public by default. `SimResult` is a struct because it is plain data with no invariant to protect.

**Control you learn.** The fourth-order Runge–Kutta method and the concept of *order of accuracy*.

RK4 samples the slope four times per step and averages them with weights 1,2,2,1:

```
k₁ = f(t, x)
k₂ = f(t + dt/2, x + dt/2·k₁)
k₃ = f(t + dt/2, x + dt/2·k₂)
k₄ = f(t + dt,   x + dt·k₃)
x_{next} = x + dt/6·(k₁ + 2k₂ + 2k₃ + k₄)
```

Global error is `O(dt⁴)`: halve `dt`, error drops 16×. For smooth linear systems with `dt` well below the fastest time constant, RK4 is effectively exact — which is why the library uses it for every continuous-time simulation. Its stability region on `ẏ = λy` extends to `|λ dt| ≲ 2.78` (vs 2 for Euler), so it also tolerates larger steps. Rule for choosing `dt`: at least 20 steps per fastest time constant, and at least 20 steps per period of the fastest oscillation. (Day 29 reconciles this with the *controller's* sample time, which is a different quantity.)

**Files you touch.** `include/csl/simulation/SimResult.hpp` (new), `include/csl/simulation/ODESolver.hpp` + `src/simulation/ODESolver.cpp` (replace the Day 4 scalar version), `examples/ex01_first_order_sim.cpp`.

**Code.**

```cpp
// include/csl/simulation/SimResult.hpp
#pragma once
#include "csl/core/Types.hpp"

namespace csl::simulation {

// The one time-domain result type used by integrate, step, impulse, lsim and Simulator::run.
// Rows are time samples; columns are channels. Unused fields have zero columns.
struct SimResult {
    Vector t;     // N
    Matrix x;     // N x n   states
    Matrix y;     // N x p   outputs  (filled by step/impulse/lsim/Simulator; empty from integrate)
    Matrix u;     // N x m   inputs   (same)
    Index samples() const { return t.size(); }
};

} // namespace csl::simulation
```

```cpp
// include/csl/simulation/ODESolver.hpp
#pragma once
#include <functional>
#include "csl/core/Types.hpp"
#include "csl/simulation/SimResult.hpp"

namespace csl::simulation {

// Right-hand side of  dx/dt = f(t, x).
using Dynamics = std::function<Vector(Real t, const Vector& x)>;

enum class Integrator { Euler, RK4 };

Vector eulerStep(const Dynamics& f, Real t, const Vector& x, Real dt);
Vector rk4Step  (const Dynamics& f, Real t, const Vector& x, Real dt);

// Integrate from t0 to tf (inclusive) with fixed step dt. Result has N = floor((tf-t0)/dt)+1 rows.
SimResult integrate(const Dynamics& f, const Vector& x0, Real t0, Real tf, Real dt,
                    Integrator method = Integrator::RK4);

} // namespace csl::simulation
```

(`enum class Integrator { Euler, RK4 }` is a *scoped enumeration*: a type with named constants, used as `Integrator::RK4`. It gets a full treatment on Day 29; here it just picks the method.)

```cpp
// src/simulation/ODESolver.cpp
#include "csl/simulation/ODESolver.hpp"
#include <cmath>
#include <stdexcept>

namespace csl::simulation {

Vector eulerStep(const Dynamics& f, Real t, const Vector& x, Real dt) {
    return x + dt * f(t, x);
}

Vector rk4Step(const Dynamics& f, Real t, const Vector& x, Real dt) {
    const Vector k1 = f(t, x);
    const Vector k2 = f(t + 0.5 * dt, x + 0.5 * dt * k1);
    const Vector k3 = f(t + 0.5 * dt, x + 0.5 * dt * k2);
    const Vector k4 = f(t + dt,       x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

SimResult integrate(const Dynamics& f, const Vector& x0, Real t0, Real tf, Real dt,
                    Integrator method) {
    if (dt <= 0.0 || tf < t0) throw std::invalid_argument("integrate: need dt > 0 and tf >= t0");
    const Index N = static_cast<Index>(std::floor((tf - t0) / dt + 1e-9)) + 1;
    SimResult r;
    r.t.resize(N);
    r.x.resize(N, x0.size());
    Vector x = x0;
    for (Index k = 0; k < N; ++k) {
        const Real t = t0 + k * dt;
        r.t(k) = t;
        r.x.row(k) = x.transpose();
        if (k + 1 < N)
            x = (method == Integrator::RK4) ? rk4Step(f, t, x, dt) : eulerStep(f, t, x, dt);
    }
    return r;
}

} // namespace csl::simulation
```

Using it (the example):

```cpp
// examples/ex01_first_order_sim.cpp  (Day 7 version)
#include <cmath>
#include <iostream>
#include "csl/simulation/ODESolver.hpp"
#include "csl/plot/CSV.hpp"
#include "plants.hpp"

int main() {
    using namespace csl;
    using namespace csl::simulation;

    const auto p = plants::thermal();            // tau = 2, K = 3
    Vector u(1); u << 1.0;
    // The lambda captures p and u by reference and is the right-hand side for the integrator.
    Dynamics f = [&](Real /*t*/, const Vector& x) { return p.A * x + p.B * u; };

    for (Real dt : {0.5, 0.1, 0.01}) {
        const SimResult euler = integrate(f, Vector::Zero(1), 0.0, 10.0, dt, Integrator::Euler);
        const SimResult rk4   = integrate(f, Vector::Zero(1), 0.0, 10.0, dt, Integrator::RK4);
        const Real exact = 3.0 * (1.0 - std::exp(-10.0 / 2.0));
        std::cout << "dt=" << dt
                  << "  Euler err=" << euler.x(euler.samples() - 1, 0) - exact
                  << "  RK4 err="   << rk4.x(rk4.samples() - 1, 0) - exact << "\n";
    }
    return 0;
}
```

**How it connects.** `integrate` + a lambda built from `(A, B)` is how `step`, `impulse`, `lsim` (Day 15) and the closed-loop `Simulator` (Day 24) all work. `Dynamics` is also the type that `NonlinearSystem` (Day 47) exposes, so the cart–pole is simulated by the same function as E1.

**Verify.** For E1 at `t = 10`: Euler errors ≈ `+1.1e-2`, `+2.4e-3`, `+2.5e-4` for `dt = 0.5, 0.1, 0.01` (linear in `dt`); RK4 errors ≈ `−4e-6`, `−5e-9`, `≈ −5e-13` (a 5× smaller step gives a `5⁴ = 625×` smaller error). Derive the Euler numbers from `y_k = K(1 − (1 − dt/τ)^k)` to be sure your program is right.

Reading: C++ Primer §10.3.2–10.3.3 (lambdas), §14.8.3 (`std::function`); learncpp 20.6–20.7. Control: any numerical-methods text on Runge–Kutta (e.g. Chapra & Canale ch. 25) — 20 minutes is enough.

**Done when.**
- [ ] Old scalar Euler removed; `integrate` compiles and the example prints the error table
- [ ] You can explain what `[&]` captures and why the lambda must not outlive `p` and `u`
- [ ] Commit

---

#### Day 8 — Classes, constructors, invariants and exceptions: the `StateSpace` class

**C++ you learn.** Your first real class. A `struct` was "data"; a **class** is "data plus the guarantees about it".

- **Members** (`Matrix A_;`) are the data. **Member functions** (`Index order() const`) operate on it. `private:` members are only reachable from member functions; `public:` ones from anywhere. The rule: data is private; the public interface is functions.
- A **constructor** runs when the object is created and establishes the **invariant** — the facts that are always true about a valid object. For `StateSpace`: `A` is square, `B` has `A.rows()` rows, `C` has `A.cols()` columns, `D` is `C.rows() × B.cols()`, `Ts ≥ 0`. If the constructor checks these, no other function ever has to.
- **Member initializer list**: `StateSpace(Matrix A, ...) : A_(std::move(A)), ... { checks }` initialises members directly (`std::move` hands over the argument's buffer instead of copying it — Day 42 explains move semantics fully; today, "cheap transfer").
- **`const` member functions** promise not to modify the object; only they can be called on a `const StateSpace&`. Mark every accessor `const`.
- **`explicit`** on single-argument constructors prevents silent implicit conversions.
- **Exceptions**: `throw DimensionMismatch("...")` abandons the current function and unwinds the stack until a `try { } catch (const csl::Error& e) { }` block catches it; if nothing catches it, the program terminates with the message. Use exceptions for *programming errors and impossible inputs*, never for expected outcomes. The library's hierarchy is in `Exceptions.hpp` (Section 4.3.6). The `using Error::Error;` line *inherits the constructors* of the base — your first contact with inheritance, which Day 13 explains in full.
- **Header/source split for classes**: the class definition (members, declarations) goes in the `.hpp`; function bodies go in the `.cpp` as `ReturnType ClassName::function(...) { }`. One-line accessors may be defined inside the class body (they are implicitly `inline`).

**Control you learn.** Dimensions, feedthrough, and the dc gain.

- With `n` states, `m` inputs and `p` outputs the shapes are fixed (Day 6). SISO = `m = p = 1`; MIMO otherwise. The library's `StateSpace` is MIMO from the start — nothing is gained by restricting it.
- **dc gain** (steady-state gain): with constant `u`, steady state has `ẋ = 0 ⇒ x_ss = −A⁻¹ B u`, so `y_ss = (D − C A⁻¹ B) u`. The matrix `G(0) = D − C A⁻¹ B` is the dc gain; it is what the step response settles to. If `A` is singular (an integrator), there is no steady state and the gain is infinite — the library throws `NumericalError`.
- For discrete systems (`Ts > 0`), steady state is `x = A x + B u ⇒ G(1) = D + C (I − A)⁻¹ B`. Write both branches now; Day 29 starts producing discrete systems.

**Files you touch.** `include/csl/core/Exceptions.hpp` (new, exactly Section 4.3.6), `include/csl/systems/StateSpace.hpp`, `src/systems/StateSpace.cpp`, `examples/plants.hpp` (return `StateSpace`), root `CMakeLists.txt`.

**Code.**

```cpp
// include/csl/systems/StateSpace.hpp   (Day 8 version — Day 13 adds the LTISystem base class)
#pragma once
#include <vector>
#include "csl/core/Types.hpp"

namespace csl::systems {

class StateSpace {
public:
    // Ts == 0 : continuous time.  Ts > 0 : discrete time with sample period Ts.
    StateSpace(Matrix A, Matrix B, Matrix C, Matrix D, Real Ts = 0.0);

    Index order()   const { return A_.rows(); }
    Index inputs()  const { return B_.cols(); }
    Index outputs() const { return C_.rows(); }
    Real  Ts()      const { return Ts_; }
    bool  isContinuous() const { return Ts_ == 0.0; }
    bool  isDiscrete()   const { return Ts_ > 0.0; }

    const Matrix& A() const { return A_; }
    const Matrix& B() const { return B_; }
    const Matrix& C() const { return C_; }
    const Matrix& D() const { return D_; }

    // Right-hand side:  continuous -> dx/dt ;  discrete -> x[k+1]
    Vector derivative(const Vector& x, const Vector& u) const;
    Vector output    (const Vector& x, const Vector& u) const;   // y = Cx + Du

    Matrix dcGain() const;   // D - C A^{-1} B  (continuous)  or  D + C (I-A)^{-1} B  (discrete)

private:
    Matrix A_, B_, C_, D_;
    Real   Ts_;
};

} // namespace csl::systems
```

```cpp
// src/systems/StateSpace.cpp
#include "csl/systems/StateSpace.hpp"
#include <string>
#include <utility>
#include "csl/core/Exceptions.hpp"

namespace csl::systems {

namespace {   // anonymous namespace: helpers visible only inside this .cpp
std::string shape(const Matrix& M) {
    return std::to_string(M.rows()) + "x" + std::to_string(M.cols());
}
}

StateSpace::StateSpace(Matrix A, Matrix B, Matrix C, Matrix D, Real Ts)
    : A_(std::move(A)), B_(std::move(B)), C_(std::move(C)), D_(std::move(D)), Ts_(Ts) {
    if (A_.rows() != A_.cols())
        throw DimensionMismatch("StateSpace: A must be square, got " + shape(A_));
    if (B_.rows() != A_.rows())
        throw DimensionMismatch("StateSpace: B must have " + std::to_string(A_.rows()) +
                                " rows, got " + shape(B_));
    if (C_.cols() != A_.cols())
        throw DimensionMismatch("StateSpace: C must have " + std::to_string(A_.cols()) +
                                " cols, got " + shape(C_));
    if (D_.rows() != C_.rows() || D_.cols() != B_.cols())
        throw DimensionMismatch("StateSpace: D must be " + std::to_string(C_.rows()) + "x" +
                                std::to_string(B_.cols()) + ", got " + shape(D_));
    if (Ts_ < 0.0) throw InvalidArgument("StateSpace: Ts must be >= 0");
}

Vector StateSpace::derivative(const Vector& x, const Vector& u) const {
    if (x.size() != order() || u.size() != inputs())
        throw DimensionMismatch("StateSpace::derivative: bad x or u size");
    return A_ * x + B_ * u;
}

Vector StateSpace::output(const Vector& x, const Vector& u) const {
    return C_ * x + D_ * u;
}

Matrix StateSpace::dcGain() const {
    const Index n = order();
    const Matrix M = isContinuous() ? A_ : (Matrix::Identity(n, n) - A_);
    Eigen::FullPivLU<Matrix> lu(M);
    if (!lu.isInvertible())
        throw NumericalError("StateSpace::dcGain: system has a pole at s=0 (or z=1); dc gain is infinite");
    const Matrix X = lu.solve(B_);             // X = M^{-1} B, solved rather than inverted
    return isContinuous() ? Matrix(D_ - C_ * X) : Matrix(D_ + C_ * X);
}

} // namespace csl::systems
```

Note `lu.solve(B_)` instead of `M.inverse() * B_`: solving a linear system is cheaper and more accurate than forming an inverse. This is a habit worth forming on day one of using Eigen.

Update `examples/plants.hpp` so each factory returns `csl::systems::StateSpace` (e.g. `return csl::systems::StateSpace(A, B, C, D);`) and delete the `ABCD` struct. Update `ex01` to use `sys.derivative(x, u)` inside its lambda.

**How it connects.** `StateSpace` is the central type of the library. Day 9 adds `poles()`, Day 13 the `LTISystem` base, Day 14 conversions to/from `TransferFunction`, Day 29 `c2d`. Every design function from Day 37 on takes a `const StateSpace&`.

**Verify.** Tests you will write on Day 10 (write them as a `main` today): `plants::dcMotor().dcGain()(0,0) == 0.0999` (tolerance `1e-9`, exact value `2/20.02`); `plants::massSpringDamper().dcGain()(0,0) == 0.25`; constructing with a `3×2` `A` throws `csl::DimensionMismatch` — check with `try/catch` and print `e.what()`.

Reading: C++ Primer Chapter 7 (all), §5.6 and §18.1 (exceptions); learncpp chapters 14 and 15 (classes), 27.1–27.5 (exceptions). Control: AM §6.1, FPE §7.2–7.3.

**Done when.**
- [ ] `StateSpace` with validated constructor, accessors, `derivative`, `output`, `dcGain`
- [ ] You can state the class invariant and where it is enforced
- [ ] A deliberate dimension error is caught and printed
- [ ] Commit

---

#### Day 9 — `std::complex`, eigenvalues, and poles: what stability means

**C++ you learn.**

- **`std::complex<double>`** (`#include <complex>`), aliased as `csl::Complex`. Construction `Complex z(−0.25, 1.98);` or `using namespace std::complex_literals; auto z = -0.25 + 1.98i;`. `z.real()`, `z.imag()`, `std::abs(z)` (modulus), `std::arg(z)` (angle, radians, in `(−π, π]`), `std::conj(z)`, `std::polar(r, θ)`, `std::exp(z)`. Arithmetic works as expected. Eigen's complex types (`ComplexVector`) hold `std::complex<double>` elements, so the two interoperate directly.
- **`Eigen::EigenSolver<Matrix>`** computes eigenvalues and eigenvectors of a real, non-symmetric matrix. `es.eigenvalues()` returns a `ComplexVector`. (For a symmetric matrix use `Eigen::SelfAdjointEigenSolver`, which is faster and returns real values — you will use it for covariance matrices on Day 50.)
- **Converting Eigen ↔ `std::vector`**: `std::vector<Complex>(ev.data(), ev.data() + ev.size())` copies via pointers to the first and one-past-last elements — your first look at the *iterator range* idiom the whole STL uses (Day 16).
- `for (const Complex& p : poles)` to iterate.

**Control you learn.** The poles of a system are the eigenvalues of `A`, and they tell you almost everything about its behaviour.

The homogeneous solution of `ẋ = Ax` is `x(t) = e^{At} x(0)`. If `A` has eigenvalues `λᵢ` with eigenvectors `vᵢ`, then `x(t) = Σ cᵢ e^{λᵢ t} vᵢ`. So each eigenvalue contributes a **mode** `e^{λt}`:

- `λ` real negative: decaying exponential with time constant `τ = −1/λ`.
- `λ = σ ± jω`: `e^{σt}(cos ωt, sin ωt)` — an oscillation at `ω` with envelope `e^{σt}`. In second-order notation `ωn = |λ|`, `ζ = −σ/|λ|`, `ωd = ω`.
- `Re λ > 0`: the mode grows without bound → **unstable**. `Re λ = 0` (a pole *on* the imaginary axis): neither grows nor decays → marginally stable (integrator, undamped oscillator).

**Stability** (continuous time): all poles strictly in the left half-plane. **Discrete time** (Day 29): all poles strictly inside the unit circle, because the mode is `λᵏ` rather than `e^{λt}`. The **dominant poles** are those closest to the imaginary axis; they decay slowest and shape the response. The 2 % settling time is roughly `4/|Re λ_dominant|`.

Compute for the running examples: E1 `λ = −0.5` (τ = 2, as designed); E2 `−0.25 ± 1.984j` (ωn = 2, ζ = 0.125); E3 `−2.0025`, `−9.9975` (electrical time constant `L/R = 0.5 s` and mechanical `J/b = 0.1 s`, slightly coupled); E4 `0, 0` (double integrator — marginally stable, which is why it needs feedback).

**Files you touch.** `include/csl/systems/StateSpace.hpp`, `src/systems/StateSpace.cpp`, `examples/ex01_first_order_sim.cpp`.

**Code.** Add to the class:

```cpp
    std::vector<Complex> poles() const;          // eigenvalues of A
    bool isStable() const;                       // Day 13 moves this into LTISystem
```

```cpp
std::vector<Complex> StateSpace::poles() const {
    Eigen::EigenSolver<Matrix> es(A_, /*computeEigenvectors=*/false);
    const ComplexVector ev = es.eigenvalues();
    return std::vector<Complex>(ev.data(), ev.data() + ev.size());
}

bool StateSpace::isStable() const {
    for (const Complex& p : poles()) {
        if (isContinuous() ? (p.real() >= 0.0) : (std::abs(p) >= 1.0)) return false;
    }
    return true;
}
```

Add to the example a loop that prints, for each pole, `wn = |p|`, `zeta = -Re p / |p|`, `tau = -1/Re p` — MATLAB's `damp` command. (Day 13 makes this a library function `damp()` once there is a common base class to attach it to.)

**How it connects.** `poles()` is the engine behind `isStable` (every design function checks it), `TransferFunction::poles()` (Day 12, via the companion matrix — the same eigen-solver), the root locus (Day 21), the discrete-stability test (Day 29), and the Hamiltonian-matrix Riccati solver (Day 44 uses the same `EigenSolver` on a `2n×2n` matrix).

**Verify.** The four pole sets above, to `1e-8`. `plants::doubleIntegrator().isStable() == false`; the other three are stable. Build `A = [[0, 1], [−4, 0]]` (undamped oscillator, ζ = 0): poles `±2j`, `isStable() == false` (marginal is not stable), and `integrate` shows a sustained sine of period `π` — simulation agrees with the poles.

Reading: cppreference `std::complex` and Eigen's `EigenSolver` page (C++ Primer has no dedicated section on complex numbers). Control: AM §6.1–6.2 (modes, eigenvalues), FPE §3.1–3.3, §7.3.

**Done when.**
- [ ] `poles()` and `isStable()` implemented and checked on E1–E4
- [ ] You can sketch, for a given pole location, the shape of the mode it produces
- [ ] Commit

---

#### Day 10 — Google Test, the test suite, the debugger, and the Phase 1 review

**C++ you learn.**

- **Unit tests with Google Test.** A test is a function that exercises one property and asserts the result. `TEST(SuiteName, TestName) { EXPECT_NEAR(actual, expected, tol); }`. Assertions: `EXPECT_EQ`, `EXPECT_TRUE`, `EXPECT_NEAR(a, b, tol)` (always this one for `double`s — never `EXPECT_EQ` on floating point), `EXPECT_THROW(expr, ExceptionType)`, `ASSERT_*` variants stop the test on failure. `gtest_main` supplies `main()` for you.
- **CTest** is CMake's test runner: `gtest_discover_tests(target)` registers each `TEST` as its own CTest test; `ctest --test-dir build --output-on-failure` runs them.
- **Debugging.** In VS Code with CMake Tools: set a breakpoint (click left of a line number), choose the test executable as the debug target, press F5. Step over (F10), step into (F11), inspect `x` in the Variables pane; Eigen matrices show their coefficients via the bundled natvis file (`build/_deps/eigen-src/debug/msvc/eigen.natvis` — add it to `launch.json`'s `visualizerFile` if the values are not expanded). Spend 30 minutes stepping through `rk4Step`; it will pay for itself the first time something returns NaN.
- **`assert`** (`#include <cassert>`): `assert(x.size() == n);` aborts in Debug, vanishes in Release. Use it for internal "this cannot happen" checks; use exceptions for public-API precondition failures.
- **Warnings as errors** for your own code: add `/WX` (MSVC) / `-Werror` (GCC) to the library target only (`target_compile_options(csl PRIVATE ...)`), not globally (third-party headers may warn).

**Control you learn.** No new theory. Today you turn the known answers of Days 2–9 into permanent tests, and you review.

**Files you touch.** Root `CMakeLists.txt`, `tests/CMakeLists.txt` (new), `tests/simulation/test_ODESolver.cpp`, `tests/systems/test_StateSpace.cpp`, `tests/plot/test_CSV.cpp`, `examples/ex01_first_order_sim.cpp` (final form: simulate E1 with RK4, print poles and dc gain, write CSV).

**Build.**

```cmake
# root CMakeLists.txt — append
option(CSL_BUILD_TESTS "Build unit tests" ON)
if(CSL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
```

```cmake
# tests/CMakeLists.txt
include(FetchContent)
FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)   # MSVC: use the same C runtime as csl
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(csl_tests
    simulation/test_ODESolver.cpp
    systems/test_StateSpace.cpp
    plot/test_CSV.cpp
)
target_link_libraries(csl_tests PRIVATE csl::csl GTest::gtest_main)
target_include_directories(csl_tests PRIVATE ${CMAKE_SOURCE_DIR}/examples)   # for plants.hpp
gtest_discover_tests(csl_tests)
```

**Code.**

```cpp
// tests/systems/test_StateSpace.cpp
#include <gtest/gtest.h>
#include "csl/core/Exceptions.hpp"
#include "csl/systems/StateSpace.hpp"
#include "plants.hpp"

using namespace csl;
using csl::systems::StateSpace;

TEST(StateSpace, RejectsNonSquareA) {
    Matrix A = Matrix::Zero(3, 2), B = Matrix::Zero(3, 1), C = Matrix::Zero(1, 2), D = Matrix::Zero(1, 1);
    EXPECT_THROW(StateSpace(A, B, C, D), DimensionMismatch);
}

TEST(StateSpace, DcGainOfDcMotor) {
    EXPECT_NEAR(plants::dcMotor().dcGain()(0, 0), 2.0 / 20.02, 1e-12);
}

TEST(StateSpace, PolesOfMassSpringDamper) {
    const auto p = plants::massSpringDamper().poles();
    ASSERT_EQ(p.size(), 2u);
    for (const Complex& z : p) {
        EXPECT_NEAR(z.real(), -0.25, 1e-9);
        EXPECT_NEAR(std::abs(z.imag()), std::sqrt(4.0 - 0.0625), 1e-9);
    }
}

TEST(StateSpace, DoubleIntegratorIsNotStable) {
    EXPECT_FALSE(plants::doubleIntegrator().isStable());
    EXPECT_TRUE(plants::dcMotor().isStable());
}
```

```cpp
// tests/simulation/test_ODESolver.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "csl/simulation/ODESolver.hpp"

using namespace csl;
using namespace csl::simulation;

TEST(ODESolver, RK4IsFourthOrderOnExponentialDecay) {
    Dynamics f = [](Real, const Vector& x) { return Vector(-x); };
    Vector x0(1); x0 << 1.0;
    const Real e1 = std::abs(integrate(f, x0, 0, 5, 0.1 , Integrator::RK4).x.bottomRows(1)(0, 0) - std::exp(-5.0));
    const Real e2 = std::abs(integrate(f, x0, 0, 5, 0.05, Integrator::RK4).x.bottomRows(1)(0, 0) - std::exp(-5.0));
    EXPECT_LT(e1, 1e-6);
    EXPECT_GT(e1 / e2, 12.0);   // ~16 for a 4th-order method; allow slack
}

TEST(ODESolver, EulerIsFirstOrder) {
    Dynamics f = [](Real, const Vector& x) { return Vector(-x); };
    Vector x0(1); x0 << 1.0;
    const Real e1 = std::abs(integrate(f, x0, 0, 5, 0.1 , Integrator::Euler).x.bottomRows(1)(0, 0) - std::exp(-5.0));
    const Real e2 = std::abs(integrate(f, x0, 0, 5, 0.05, Integrator::Euler).x.bottomRows(1)(0, 0) - std::exp(-5.0));
    EXPECT_NEAR(e1 / e2, 2.0, 0.2);
}

TEST(ODESolver, SampleCountAndTimeGrid) {
    Dynamics f = [](Real, const Vector& x) { return Vector(0.0 * x); };
    const auto r = integrate(f, Vector::Zero(1), 0.0, 1.0, 0.25);
    ASSERT_EQ(r.samples(), 5);
    EXPECT_NEAR(r.t(4), 1.0, 1e-12);
}
```

A `test_CSV.cpp` that writes a 3-row file to a temporary path, reads it back line by line with `std::ifstream` and `std::getline`, and checks the header and one value.

**Phase 1 review — answer in writing (a paragraph each, in `sandbox/phase1_review.md`).**
1. Trace `hello.cpp` → `hello.exe`: what does each of the three tools consume and produce?
2. Why does changing `StateSpace.cpp` not recompile `ODESolver.cpp`? What *does* trigger recompiling everything that includes `StateSpace.hpp`?
3. What does `target_link_libraries(csl_tests PRIVATE csl::csl)` give `csl_tests` besides the `.lib`?
4. Explain `const Matrix&` vs. `Matrix` as a parameter, and why `integrate` returns `SimResult` by value without worrying about copies.
5. Control: given poles `−1 ± 3j`, state `ωn`, `ζ`, settling time, and sketch the step response. Given `dt = 1 s` and Euler, is simulating this system stable?
6. Control: what is the dc gain of E3 in words, and what would the number be if `b` doubled?

**Verify.** `ctest --test-dir build --output-on-failure` reports all tests passing. You have stepped through `rk4Step` in the debugger and watched `k1…k4`.

Reading: Google Test primer (google.github.io/googletest/primer.html); C++ Primer §6.5.3 (`assert`); learncpp 3.x (debugging — read it all, it is short and will save you hours).

**Done when.**
- [ ] 8+ tests passing under CTest
- [ ] Debugger used once with a breakpoint inside the library
- [ ] Phase 1 review written
- [ ] Commit `Day 10: tests, Phase 1 complete`

---
### Phase 2 — System representations and the time domain (Days 11–16)

**Phase goal.** The library has `Polynomial`, `TransferFunction`, the abstract `LTISystem` base, conversions `tf2ss`/`ss2tf`, a plotter, and `step`/`impulse`/`lsim` with time-domain specs. In control terms: you can move freely between the ODE, state-space, transfer-function and zero-pole-gain views of the same system and read a step response like an engineer.

---

#### Day 11 — Operator overloading: the `Polynomial` class and the Laplace domain

**C++ you learn.** Making your own types behave like numbers.

- **Operator overloading**: `Polynomial operator+(const Polynomial& a, const Polynomial& b)` lets you write `p + q`. Binary arithmetic operators are best written as **free functions** (non-members) so both operands are treated symmetrically; `operator[]`, `operator()` and `operator=` must be members. Overload only operators whose meaning is obvious (`+ − *` for polynomials: yes; `/`: no, because polynomial division has a remainder).
- **`std::initializer_list<Real>`** lets a constructor accept `Polynomial p{1, 3, 2};`. 
- **`operator()`** makes an object callable: `p(2.0)` evaluates the polynomial. Objects with `operator()` are *functors*; they can be stored in a `std::function` just like lambdas.
- **`friend`**: a function declared `friend` inside the class may access private members although it is not a member. Used for `operator<<` (which must be a free function because its left operand is the stream).
- **`static` member functions** belong to the class, not to an instance: `Polynomial::fromRoots(...)` is a *named constructor* ("factory") — clearer than a constructor overload that could be confused with the coefficient constructor.
- Do **not** overload `==` for floating-point data; provide `isApprox(other, tol)` as Eigen does.

**Control you learn.** The Laplace transform turns differential equations into polynomial algebra.

The Laplace transform `F(s) = ∫₀^∞ f(t) e^{−st} dt` has one property that matters most: **differentiation becomes multiplication by `s`** (for zero initial conditions, `L{ḟ} = sF(s)`). Apply it to E1: `τ s Y(s) + Y(s) = K U(s)`, so

```
Y(s) / U(s) = K / (τ s + 1)  =:  H(s)      — the transfer function
```

Every linear ODE with constant coefficients becomes a *ratio of polynomials in s*. The denominator polynomial is the **characteristic polynomial** — for a state-space model it is `det(sI − A)`, and its roots are exactly the eigenvalues of `A` from Day 9. That is why a polynomial class is the first thing to build in this phase: a transfer function is two polynomials.

The variable `s` is complex: `s = σ + jω`. Setting `s = jω` gives the frequency response (Day 17); the real part σ relates to exponential growth/decay (Day 9's modes again).

**Files you touch.** `include/csl/core/Polynomial.hpp`, `src/core/Polynomial.cpp`, `tests/core/test_Polynomial.cpp`, CMake lists.

**Code.**

```cpp
// include/csl/core/Polynomial.hpp
#pragma once
#include <initializer_list>
#include <iosfwd>
#include <vector>
#include "csl/core/Types.hpp"

namespace csl {

// Real polynomial with coefficients in DESCENDING powers: {1, 3, 2} is s^2 + 3s + 2.
class Polynomial {
public:
    Polynomial();                                        // the zero polynomial
    Polynomial(std::initializer_list<Real> coeffs);      // Polynomial{1, 3, 2}
    explicit Polynomial(const Vector& coeffs);

    static Polynomial fromRoots(const std::vector<Complex>& roots, Real leading = 1.0);

    Index         degree() const { return c_.size() - 1; }
    const Vector& coeffs() const { return c_; }
    Real          operator[](Index i) const { return c_(i); }   // i = 0 is the leading coefficient
    Real          leading() const { return c_(0); }

    Real    operator()(Real x)    const;   // Horner's method
    Complex operator()(Complex x) const;

    Polynomial derivative() const;
    Polynomial monic() const;              // divided by the leading coefficient
    std::vector<Complex> roots() const;    // Day 12: companion matrix + Eigen::EigenSolver

    bool isApprox(const Polynomial& other, Real tol = 1e-9) const;

    friend Polynomial operator+(const Polynomial& a, const Polynomial& b);
    friend Polynomial operator-(const Polynomial& a, const Polynomial& b);
    friend Polynomial operator*(const Polynomial& a, const Polynomial& b);
    friend Polynomial operator*(Real k, const Polynomial& p);
    friend Polynomial operator*(const Polynomial& p, Real k) { return k * p; }
    friend Polynomial operator-(const Polynomial& p) { return -1.0 * p; }
    friend std::ostream& operator<<(std::ostream& os, const Polynomial& p);

private:
    Vector c_;
    void trim();   // remove leading (near-)zero coefficients so degree() is honest
};

} // namespace csl
```

```cpp
// src/core/Polynomial.cpp  (the parts that are not obvious)
#include "csl/core/Polynomial.hpp"
#include <cmath>
#include <ostream>
#include "csl/core/Exceptions.hpp"

namespace csl {

Polynomial::Polynomial() : c_(Vector::Zero(1)) {}

Polynomial::Polynomial(std::initializer_list<Real> coeffs) : c_(coeffs.size()) {
    if (coeffs.size() == 0) throw InvalidArgument("Polynomial: empty coefficient list");
    Index i = 0;
    for (Real v : coeffs) c_(i++) = v;
    trim();
}

void Polynomial::trim() {
    Index first = 0;
    while (first < c_.size() - 1 && std::abs(c_(first)) < 1e-14) ++first;
    if (first > 0) c_ = Vector(c_.tail(c_.size() - first));
}

Real Polynomial::operator()(Real x) const {        // Horner: ((c0 x + c1) x + c2) x + ...
    Real acc = 0.0;
    for (Index i = 0; i < c_.size(); ++i) acc = acc * x + c_(i);
    return acc;
}

Complex Polynomial::operator()(Complex x) const {
    Complex acc = 0.0;
    for (Index i = 0; i < c_.size(); ++i) acc = acc * x + c_(i);
    return acc;
}

Polynomial operator*(const Polynomial& a, const Polynomial& b) {   // convolution of coefficients
    Vector c = Vector::Zero(a.degree() + b.degree() + 1);
    for (Index i = 0; i <= a.degree(); ++i)
        for (Index j = 0; j <= b.degree(); ++j)
            c(i + j) += a[i] * b[j];
    return Polynomial(c);
}

Polynomial operator+(const Polynomial& a, const Polynomial& b) {   // align by LOWEST power
    const Index n = std::max(a.degree(), b.degree()) + 1;
    Vector c = Vector::Zero(n);
    c.tail(a.degree() + 1) += a.coeffs();
    c.tail(b.degree() + 1) += b.coeffs();
    return Polynomial(c);
}

Polynomial Polynomial::fromRoots(const std::vector<Complex>& roots, Real leading) {
    // Multiply (x - r_i) in complex arithmetic, then drop the (tiny) imaginary parts.
    // Roots must come in conjugate pairs for the result to be real.
    ComplexVector c(1); c(0) = leading;
    for (const Complex& r : roots) {
        ComplexVector next = ComplexVector::Zero(c.size() + 1);
        next.head(c.size()) += c;          // c * x
        next.tail(c.size()) -= r * c;      // - r * c
        c = next;
    }
    if (c.imag().cwiseAbs().maxCoeff() > 1e-8 * (1.0 + c.cwiseAbs().maxCoeff()))
        throw InvalidArgument("Polynomial::fromRoots: roots are not closed under conjugation");
    return Polynomial(Vector(c.real()));
}

} // namespace csl
```

Write `derivative()`, `monic()`, `operator-`, `k * p`, `isApprox` (compare coefficient vectors after padding to the same length) and `operator<<` (print `s^2 + 3 s + 2`, skipping zero terms) yourself.

**How it connects.** `TransferFunction` (tomorrow) is two `Polynomial`s. Routh–Hurwitz (Day 22) walks `coeffs()`. Ackermann (Day 38) evaluates a `Polynomial` at a *matrix*. Tustin discretization (Day 30) substitutes one polynomial into another using these operators.

**Verify.** Tests: `Polynomial{1,1} * Polynomial{1,2}` is approx `{1,3,2}`; `Polynomial{1,3,2}(−1.0) == 0`; `Polynomial{1,3,2}.derivative()` is `{2,3}`; `fromRoots({−1, −2})` is `{1,3,2}`; `fromRoots({Complex(−1,2), Complex(−1,−2)})` is `{1,2,5}`; `Polynomial{0, 0, 1, 3}` has degree 1 after trim.

Reading: C++ Primer Chapter 14 (§14.1–14.3, 14.8); learncpp chapter 21 (operator overloading). Control: FPE §3.1 (Laplace), AM §9.1–9.2.

**Done when.**
- [ ] `Polynomial` complete with 6+ tests
- [ ] You can explain why `operator+` is a friend free function but `operator()` is a member
- [ ] Commit

---

#### Day 12 — `TransferFunction`, root finding via the companion matrix, pretty printing

**C++ you learn.**

- **Composition**: a class whose members are other classes (`TransferFunction` holds two `Polynomial`s). Prefer composition to inheritance when the relation is "has a", not "is a".
- **Reusing a tool for a new problem**: polynomial roots are the eigenvalues of the *companion matrix* — the same `Eigen::EigenSolver` from Day 9. This is Ground Rule 1 in its most instructive form: *we* build the control-specific matrix, Eigen does the generic numerics.
- **`std::ostringstream`** (`#include <sstream>`) builds a string with stream syntax; used by `operator<<` to lay out `num / den` on three lines.
- **`std::sort`** on a `std::vector<Complex>` with a lambda comparator (by real part, then imaginary) so printed pole lists are deterministic.
- **Normalisation in the constructor**: divide both polynomials by the denominator's leading coefficient so the denominator is monic. One canonical form, so comparisons and conversions are simpler everywhere.

**Control you learn.** The transfer function as an object: `H(s) = N(s)/D(s)`.

- **Poles** = roots of `D(s)`; **zeros** = roots of `N(s)`. Poles are the system's modes (Day 9); zeros shape *how much* each mode is excited and can block a frequency entirely (a zero at `jω₀` means a sinusoid at ω₀ produces no steady-state output — the notch filter of Day 34 is built on this).
- **Proper**: `deg N ≤ deg D` (physically realisable); **strictly proper**: `deg N < deg D` (no instantaneous feedthrough, `D = 0` in state space). **Relative degree** `deg D − deg N` equals the number of integrations between input and output.
- **dc gain** `H(0) = N(0)/D(0)` — the same number as `D − CA⁻¹B` from Day 8 (you will prove it on Day 14).
- **Zero-pole-gain form** `H(s) = k ∏(s − zᵢ) / ∏(s − pᵢ)` — the same object, factored. `fromZPK` builds it with `Polynomial::fromRoots`.
- **Pole-zero cancellation**: `(s+1)/((s+1)(s+2))` is "really" `1/(s+2)`; `minreal` cancels pairs closer than a tolerance. Important later: an uncontrollable or unobservable state-space model shows up as a cancellation in `ss2tf`.

The companion matrix of the monic polynomial `sⁿ + a₁sⁿ⁻¹ + … + aₙ`:

```
C = [ −a₁  −a₂  …  −aₙ ]
    [  1    0   …   0  ]
    [  0    1   …   0  ]
    [  …              ]
```

`det(sI − C)` equals the polynomial, so its eigenvalues are the roots. (Notice it is the transpose-ish cousin of tomorrow's controllable canonical form; it is no coincidence.)

**Files you touch.** `src/core/Polynomial.cpp` (`roots()`), `include/csl/systems/TransferFunction.hpp`, `src/systems/TransferFunction.cpp`, `tests/systems/test_TransferFunction.cpp`, `examples/plants.hpp` (add `thermalTF()`, `massSpringDamperTF()`).

**Code.**

```cpp
std::vector<Complex> Polynomial::roots() const {
    const Index n = degree();
    if (n < 1) return {};
    Matrix C = Matrix::Zero(n, n);
    for (Index j = 0; j < n; ++j) C(0, j) = -c_(j + 1) / c_(0);
    for (Index i = 1; i < n; ++i) C(i, i - 1) = 1.0;
    Eigen::EigenSolver<Matrix> es(C, false);
    const ComplexVector ev = es.eigenvalues();
    std::vector<Complex> r(ev.data(), ev.data() + ev.size());
    std::sort(r.begin(), r.end(), [](const Complex& a, const Complex& b) {
        return a.real() != b.real() ? a.real() < b.real() : a.imag() < b.imag();
    });
    return r;
}
```

```cpp
// include/csl/systems/TransferFunction.hpp   (Day 12 version; Day 13 adds ": public LTISystem")
#pragma once
#include <initializer_list>
#include <iosfwd>
#include <vector>
#include "csl/core/Polynomial.hpp"
#include "csl/core/Types.hpp"

namespace csl::systems {

class TransferFunction {
public:
    TransferFunction(Polynomial num, Polynomial den, Real Ts = 0.0);
    TransferFunction(std::initializer_list<Real> num, std::initializer_list<Real> den, Real Ts = 0.0);
    static TransferFunction fromZPK(const std::vector<Complex>& zeros,
                                    const std::vector<Complex>& poles, Real gain, Real Ts = 0.0);

    const Polynomial& num() const { return num_; }
    const Polynomial& den() const { return den_; }   // always monic
    Real  Ts()    const { return Ts_; }
    Index order() const { return den_.degree(); }
    bool  isProper()         const { return num_.degree() <= den_.degree(); }
    bool  isStrictlyProper() const { return num_.degree() <  den_.degree(); }
    Index relativeDegree()   const { return den_.degree() - num_.degree(); }

    std::vector<Complex> poles() const { return den_.roots(); }
    std::vector<Complex> zeros() const { return num_.roots(); }
    Complex evaluate(Complex s)  const { return num_(s) / den_(s); }
    Real    dcGain() const;                                  // H(0), or H(1) if discrete
    TransferFunction minreal(Real tol = 1e-8) const;        // cancel near-identical pole/zero pairs

    friend std::ostream& operator<<(std::ostream& os, const TransferFunction& H);

private:
    Polynomial num_, den_;
    Real Ts_;
};

} // namespace csl::systems
```

Constructor: throw `InvalidArgument` if `den` is the zero polynomial or `Ts < 0`; divide both by `den.leading()`. `dcGain`: `evaluate(isContinuous ? 0 : 1).real()`, throwing `NumericalError` if the denominator evaluates to ~0. `minreal`: for each zero, find a pole within `tol`; remove both; rebuild with `fromRoots` keeping the gain `num.leading()`.

**How it connects.** Day 13 gives `TransferFunction` and `StateSpace` a common base. Day 14 converts between them. Day 19 overloads `*` and `+` for series/parallel. Day 30 produces discrete TFs (`Ts > 0`) from `c2d`. Filters (Day 33) report themselves as discrete TFs.

**Verify.** `TransferFunction{{1}, {1, 3, 2}}.poles()` = `{−2, −1}`; `plants::thermalTF()` (= `{3}/{2, 1}`, stored as `{1.5}/{1, 0.5}`) has pole `−0.5` and `dcGain() == 3`; `fromZPK({}, {−1, −2}, 1)` isApprox `{{1},{1,3,2}}`; `TransferFunction{{1, 1}, {1, 3, 2}}.minreal()` has order 1 and pole `−2`; `TransferFunction{{1}, {0}}` throws. Print one: output should look like

```
        1
  -------------
  s^2 + 0.5 s + 4
```

Reading: C++ Primer §8.3 (string streams), §10.3.1 (`sort` with a predicate). Control: FPE §3.1–3.2, AM §9.2–9.3 (transfer functions), §9.4 (zeros).

**Done when.**
- [ ] `roots()`, `TransferFunction` complete, 6+ tests
- [ ] You can write down the companion matrix of `s³ + 6s² + 11s + 6` and say what its eigenvalues are without computing them
- [ ] Commit

---

#### Day 13 — Inheritance and virtual functions: the `LTISystem` base class

**C++ you learn.** The mechanism that lets one function (`bode`, `step`, `margins`) accept *any* system.

- **Inheritance**: `class StateSpace : public LTISystem` makes every `StateSpace` also an `LTISystem`. Code that takes `const LTISystem&` can be handed a `StateSpace` or a `TransferFunction`.
- **Virtual functions**: a function marked `virtual` in the base is dispatched *at run time* on the actual object type. `virtual std::vector<Complex> poles() const = 0;` — the `= 0` makes it **pure virtual**: the base provides no implementation, and a class with any pure virtual function is **abstract** (cannot be instantiated; exists only to be derived from).
- **`override`**: write it on every overriding function; the compiler then checks that you really are overriding (a typo in the signature otherwise silently creates a *new* function).
- **Virtual destructor**: `virtual ~LTISystem() = default;` — required so that deleting through a base pointer runs the derived destructor. Rule: a class with any virtual function gets a virtual destructor.
- **Non-virtual functions implemented in terms of virtuals** (`isStable()` calls `poles()`): the "template method" pattern. Write the logic once in the base.
- **Slicing**: `LTISystem s = someStateSpace;` would copy only the base part — but it cannot compile here because the base is abstract. That is one reason abstract bases are good. Always pass polymorphic objects by reference or pointer, never by value.
- `final` (on a class or function) forbids further overriding; use it on `StateSpace`/`TransferFunction` if you like — they are leaves.

**Control you learn.** What is common to *every* LTI system, and the one thing that connects all representations: `H(s)`.

- Any LTI system, however represented, has an order, poles, zeros, a dc gain and a frequency-domain transfer `H(s)` (a `p × m` matrix for MIMO). For a state-space model, `H(s) = C (sI − A)⁻¹ B + D`. Prove it: Laplace-transform `ẋ = Ax + Bu` → `sX = AX + BU` → `X = (sI − A)⁻¹ B U` → `Y = [C(sI − A)⁻¹B + D] U`. That one line unifies Days 8 and 12.
- **Transmission zeros** of a state-space model are the `s` at which the *Rosenbrock system matrix* `[[A − sI, B], [C, D]]` loses rank — for a SISO system, the generalised eigenvalues of the pair `(M, N)` with `M = [[A, B],[C, D]]` and `N = diag(I, 0)`. Eigen's `GeneralizedEigenSolver` returns `α/β` pairs; `β ≈ 0` entries are "zeros at infinity" (there are `relative degree` of them) and are discarded. For SISO systems you will cross-check against `ss2tf(...).zeros()` tomorrow.
- MATLAB's `damp`: for each pole, `ωn = |p|`, `ζ = −Re p / |p|`, `τ = −1/Re p`. (For discrete poles, first map `p → ln(p)/Ts` — Day 29.)

**Files you touch.** `include/csl/systems/LTISystem.hpp` (new, exactly Section 4.3.3 plus `damp`), `src/systems/LTISystem.cpp`, `StateSpace.hpp/.cpp` and `TransferFunction.hpp/.cpp` (derive, `override`, add `evaluate` for SS and `zeros` for SS), `tests/systems/test_LTISystem.cpp`.

**Code.**

```cpp
// include/csl/systems/LTISystem.hpp
#pragma once
#include <vector>
#include "csl/core/Types.hpp"

namespace csl::systems {

class LTISystem {
public:
    virtual ~LTISystem() = default;

    virtual Index order()   const = 0;
    virtual Index inputs()  const = 0;
    virtual Index outputs() const = 0;
    virtual Real  Ts()      const = 0;
    virtual std::vector<Complex> poles() const = 0;
    virtual std::vector<Complex> zeros() const = 0;
    virtual ComplexMatrix evaluate(Complex s) const = 0;   // outputs x inputs
    virtual Matrix dcGain() const = 0;

    bool isContinuous() const { return Ts() == 0.0; }
    bool isDiscrete()   const { return Ts() >  0.0; }
    bool isSISO()       const { return inputs() == 1 && outputs() == 1; }
    bool isStable()     const;                                // implemented once, via poles()
};

struct PoleInfo { Complex pole; Real wn; Real zeta; Real tau; };
std::vector<PoleInfo> damp(const LTISystem& sys);             // MATLAB's damp()

} // namespace csl::systems
```

`TransferFunction::evaluate` now returns a `1×1 ComplexMatrix` (keep a convenience `Complex evaluateScalar(Complex s) const`). `StateSpace::evaluate`:

```cpp
ComplexMatrix StateSpace::evaluate(Complex s) const {
    const Index n = order();
    const ComplexMatrix M = s * ComplexMatrix::Identity(n, n) - A_.cast<Complex>();
    const ComplexMatrix X = M.partialPivLu().solve(B_.cast<Complex>());   // (sI - A)^{-1} B
    return C_.cast<Complex>() * X + D_.cast<Complex>();
}

std::vector<Complex> StateSpace::zeros() const {
    if (!isSISO()) throw NotSupported("StateSpace::zeros: SISO only (MIMO zeros are a stretch topic)");
    const Index n = order();
    Matrix M(n + 1, n + 1), N = Matrix::Zero(n + 1, n + 1);
    M << A_, B_, C_, D_;
    N.topLeftCorner(n, n).setIdentity();
    Eigen::GeneralizedEigenSolver<Matrix> ges(M, N);
    std::vector<Complex> z;
    for (Index i = 0; i < n + 1; ++i) {
        const Real beta = ges.betas()(i);
        if (std::abs(beta) > 1e-10 * (1.0 + std::abs(ges.alphas()(i))))   // finite zero
            z.push_back(ges.alphas()(i) / beta);
    }
    std::sort(z.begin(), z.end(), /* same comparator as Polynomial::roots */);
    return z;
}
```

Move `isStable()` from `StateSpace` into `LTISystem.cpp` (continuous: `Re p < 0` for all; discrete: `|p| < 1` for all). A polymorphism demonstration for the example:

```cpp
void report(const csl::systems::LTISystem& sys) {       // works for SS and TF alike
    std::cout << "order " << sys.order() << ", stable=" << sys.isStable() << ", |H(j1)|="
              << std::abs(sys.evaluate(csl::Complex(0, 1))(0, 0)) << "\n";
}
```

**How it connects.** From now on every analysis function takes `const LTISystem&`. `evaluate` is what `frequencyResponse` (Day 17) calls. `Controller`, `Estimator`, `DigitalFilter` (Days 23, 39, 33) repeat exactly this pattern of an abstract base with pure virtuals.

**Verify.** For E2: `ss.evaluate(s)(0,0)` equals `tf.evaluate(s)(0,0)` at `s = 1 + 2j` and at `s = 0.3j` to `1e-9` — the first cross-representation test. `StateSpace` for `H = (s+3)/(s²+3s+2)` (type its controllable canonical form by hand from Day 14's formula, or wait for `tf2ss`) has `zeros() == {−3}`. `damp(plants::massSpringDamper())` gives `ωn = 2`, `ζ = 0.125` for both poles. Attempting `LTISystem x;` fails to compile (try it, read the error, delete it).

Reading: C++ Primer Chapter 15 (§15.1–15.5 today, the rest on Day 23); learncpp chapters 24–25. Control: AM §9.2 (`H(s) = C(sI−A)⁻¹B + D`), FPE §7.3.

**Done when.**
- [ ] `LTISystem` abstract base; both concrete classes derive with `override` everywhere
- [ ] `report()` called with both representations prints the same numbers for E2
- [ ] You can explain what would go wrong without the virtual destructor
- [ ] Commit

---

#### Day 14 — Realizations: `tf2ss`, `ss2tf`, similarity transforms (Eigen block operations)

**C++ you learn.**

- **Eigen block operations**: `A.topRightCorner(r, c)`, `A.bottomRows(1)`, `A.block(i, j, r, c)`, `A.diagonal(1)` (super-diagonal), `.setIdentity()`, `.setZero()`, `M.trace()`. Writing a matrix by *describing its structure* rather than looping over entries is both clearer and less error-prone.
- **Free functions in a namespace** as the API for conversions (`csl::systems::tf2ss`), rather than member functions — because a conversion between two types belongs to neither.
- A numerical identity used as a **self-check** (`assert` that Faddeev–LeVerrier's last matrix is zero — the Cayley–Hamilton theorem). Cheap, catches bugs early.

**Control you learn.** Realization theory in one day.

- A **realization** of `H(s)` is any `(A, B, C, D)` with `C(sI−A)⁻¹B + D = H(s)`. There are infinitely many: for any invertible `T`, `(TAT⁻¹, TB, CT⁻¹, D)` is another (a **similarity transform** — a change of state coordinates). Poles, zeros, dc gain and `H(s)` are invariant; the state vector's *meaning* is not.
- The **controllable canonical form** for `H(s) = (b₀sⁿ + … + bₙ)/(sⁿ + a₁sⁿ⁻¹ + … + aₙ)`:

```
A = [ 0    1    0  …  0  ]   B = [0]   C = [ bₙ−b₀aₙ  …  b₁−b₀a₁ ]   D = b₀
    [ 0    0    1  …  0  ]       [0]
    [ …                  ]       […]
    [ −aₙ −aₙ₋₁  …  −a₁ ]       [1]
```

(With `b₀ = 0` for strictly proper systems, `C` is just the reversed numerator.) It is always controllable (Day 37); it may be unobservable if there is a pole-zero cancellation — a **minimal** realization has order equal to the order of `minreal(H)`.
- `ss2tf` via the **Faddeev–LeVerrier** recursion computes `det(sI − A)` and `adj(sI − A)` at once:

```
M₁ = I,   a₁ = −tr(A M₁)
M_k = A M_{k−1} + a_{k−1} I,   a_k = −tr(A M_k) / k      (k = 2…n)
det(sI − A) = sⁿ + a₁sⁿ⁻¹ + … + aₙ
adj(sI − A) = Σ_{k=1}^{n} s^{n−k} M_k
```

so `H(s) = [C adj(sI−A) B + D det(sI−A)] / det(sI−A)` — numerator coefficient of `s^{n−k}` is `C M_k B + D a_k`. It is `O(n⁴)` and only sensible for `n ≲ 20`, which covers everything in this plan; it is chosen because it *shows* where the characteristic polynomial comes from.

**Files you touch.** `include/csl/systems/Conversions.hpp`, `src/systems/Conversions.cpp`, `tests/systems/test_Conversions.cpp`.

**Code.**

```cpp
// include/csl/systems/Conversions.hpp  (Day 14 part; Days 29–30 add c2d/d2c here)
#pragma once
#include "csl/core/Polynomial.hpp"
#include "csl/systems/StateSpace.hpp"
#include "csl/systems/TransferFunction.hpp"

namespace csl::systems {

StateSpace       tf2ss(const TransferFunction& H);               // controllable canonical form
TransferFunction ss2tf(const StateSpace& sys);                   // SISO; Faddeev-LeVerrier
Polynomial       charPoly(const Matrix& A);                      // det(sI - A), descending
StateSpace       similarityTransform(const StateSpace& sys, const Matrix& T);   // x_new = T x

// Convenience: get a StateSpace from any LTISystem (identity for SS, tf2ss for TF).
StateSpace       toStateSpace(const LTISystem& sys);

} // namespace csl::systems
```

```cpp
StateSpace tf2ss(const TransferFunction& H) {
    if (!H.isProper()) throw InvalidArgument("tf2ss: transfer function must be proper");
    const Index n = H.order();
    const Vector a = H.den().coeffs();                 // monic: a(0) = 1, a(1..n) = a_1..a_n
    Vector b = Vector::Zero(n + 1);                    // numerator padded to degree n
    const Index dn = H.num().degree();
    for (Index i = 0; i <= dn; ++i) b(n - dn + i) = H.num()[i];
    const Real d = b(0);

    Matrix A = Matrix::Zero(n, n);
    if (n > 1) A.topRightCorner(n - 1, n - 1).setIdentity();      // ones on the super-diagonal
    for (Index j = 0; j < n; ++j) A(n - 1, j) = -a(n - j);         // last row: -a_n ... -a_1
    Matrix B = Matrix::Zero(n, 1); B(n - 1, 0) = 1.0;
    Matrix C(1, n);
    for (Index j = 0; j < n; ++j) C(0, j) = b(n - j) - d * a(n - j);
    Matrix D(1, 1); D << d;
    return StateSpace(A, B, C, D, H.Ts());
}

TransferFunction ss2tf(const StateSpace& sys) {
    if (!sys.isSISO()) throw NotSupported("ss2tf: SISO only");
    const Index n = sys.order();
    const Matrix& A = sys.A();
    const Real d = sys.D()(0, 0);
    Vector a(n + 1), b(n + 1);
    a(0) = 1.0; b(0) = d;
    Matrix M = Matrix::Identity(n, n);                  // M_1
    for (Index k = 1; k <= n; ++k) {
        a(k) = -(A * M).trace() / static_cast<Real>(k);
        b(k) = (sys.C() * M * sys.B())(0, 0) + d * a(k);
        M = A * M + a(k) * Matrix::Identity(n, n);      // M_{k+1}
    }
    assert(M.norm() < 1e-8 * (1.0 + A.norm()));         // Cayley-Hamilton: M_{n+1} == 0
    return TransferFunction(Polynomial(b), Polynomial(a), sys.Ts());
}
```

`charPoly` is the `a` vector of the same loop (factor it out). `similarityTransform` applies the formula with `T.partialPivLu().solve(...)` for the inverses.

**How it connects.** `toStateSpace` is what `step`, `lsim`, `c2d`, `lqr`, Kalman — everything matrix-based — call on an `LTISystem`. `charPoly` is used by Ackermann's formula (Day 38) and Routh (Day 22). `ss2tf` is how you check a state-space design in the transfer-function world (and how a `DigitalFilter` built from matrices reports its TF).

**Verify.** `ss2tf(plants::massSpringDamper())` isApprox `{1}/{1, 0.5, 4}`; `ss2tf(plants::dcMotor())` isApprox `{2}/{1, 12, 20.02}`; `ss2tf(tf2ss(H))` isApprox `H` for `H = {1, 3}/{1, 3, 2}` and for a proper `H = {2, 1, 3}/{1, 3, 2}` (`D = 2`); `tf2ss` of E2's TF has `A = [[0, 1], [−4, −0.5]]`, `B = [0; 1]`, `C = [1, 0]` — identical to `plants::massSpringDamper()` (because `m = 1`); `similarityTransform(sys, T)` with `T = [[2, 0], [1, 1]]` has the same poles and `evaluate(1+j)`; `StateSpace::zeros()` of `tf2ss({1,3}/{1,3,2})` is `{−3}` (cross-check of Day 13).

Reading: Eigen "Block operations" page. Control: FPE §7.3–7.4 (canonical forms, realization), AM §9.2; Chen, *Linear System Theory and Design* §4.4 if you want the full theory.

**Done when.**
- [ ] `tf2ss`, `ss2tf`, `charPoly`, `similarityTransform`, `toStateSpace` with tests
- [ ] You can explain why two different `A` matrices can have the same transfer function
- [ ] Commit

---

#### Day 15 — The plotter (`csl::plot::Figure`) and `step` / `impulse` / `lsim`

**C++ you learn.** A small, complete class with real I/O: this is the day "I can write a program" becomes "I can write a tool".

- **Designing a class API**: `Figure` collects series and settings, then renders. Setters return `Figure&` (`return *this;`) so calls chain: `fig.line(t, y, "y").xlabel("t [s]").save("step.html")`.
- **String building** with `std::ostringstream` and `std::setprecision`; generating SVG (an XML text format: `<svg>`, `<polyline points="x,y x,y …"/>`, `<text>`, `<line>`) and wrapping it in a minimal HTML page so any browser renders it. Zero dependencies.
- **Coordinate mapping** (data → pixels), **"nice" tick computation** (`niceStep(range, ~6)` rounding to 1/2/5 × 10ᵏ), `std::isfinite` to skip NaNs, `std::min_element`/`std::max_element` over Eigen data via `.minCoeff()`/`.maxCoeff()`.
- **Default arguments** and **`const` member functions** that produce output without mutating.
- Keep `Figure.hpp` ignorant of control concepts; `Plots.hpp` holds the control-specific wrappers (`plotStep` etc.). Separation of layers in practice.

**Control you learn.** The three standard time-domain experiments and how they are simulated correctly.

- **Step response**: `u(t) = 1` for `t ≥ 0`, `x(0) = 0`. Settles to the dc gain. The library's bread and butter.
- **Impulse response**: `u = δ(t)`. For a strictly proper system this equals the zero-input response from `x(0) = B` (the impulse instantaneously loads the state) — so `impulse` is `integrate` from `x0 = B` with `u = 0`. If `D ≠ 0` the response also contains `D·δ(t)`, which cannot be plotted; the function records it in the result as a note and ignores it. The impulse response is the derivative of the step response (check it numerically).
- **`lsim`**: arbitrary input samples `u(tₖ)`. Between samples the input is *held constant* (zero-order hold), which is exactly what a digital controller does. With RK4 and a sufficiently small `dt` the error is negligible; Day 29 shows the *exact* way (`c2d` with ZOH, then a discrete recursion) and you will compare.
- For a **discrete** system (`Ts > 0`) all three are exact recursions `x[k+1] = A x[k] + B u[k]`; write that branch today — Day 29 will produce such systems.
- Default simulation horizon: `tf ≈ 8 / min|Re pᵢ|` over stable poles (so the slowest mode decays), `dt = tf / 2000`, capped so there are ≥ 20 samples per fastest mode. For an unstable or marginal system use `tf = 10` and warn.

**Files you touch.** `include/csl/plot/Figure.hpp`, `src/plot/Figure.cpp`, `include/csl/plot/Plots.hpp`, `src/plot/Plots.cpp`, `include/csl/simulation/TimeResponse.hpp`, `src/simulation/TimeResponse.cpp`, tests, `examples/ex01` (final tidy: plot the step).

**Code.**

```cpp
// include/csl/plot/Figure.hpp
#pragma once
#include <string>
#include <vector>
#include "csl/core/Types.hpp"

namespace csl::plot {

class Figure {
public:
    explicit Figure(std::string title = "");

    Figure& line   (const Vector& x, const Vector& y, const std::string& label = "");
    Figure& scatter(const Vector& x, const Vector& y, const std::string& label = "",
                    const std::string& marker = "o");       // "o" or "x"
    Figure& hline(Real y, const std::string& label = "");
    Figure& vline(Real x, const std::string& label = "");
    Figure& xlabel(const std::string& s) { xlabel_ = s; return *this; }
    Figure& ylabel(const std::string& s) { ylabel_ = s; return *this; }
    Figure& logX(bool on = true) { logX_ = on; return *this; }
    Figure& logY(bool on = true) { logY_ = on; return *this; }
    Figure& xlim(Real lo, Real hi);
    Figure& ylim(Real lo, Real hi);
    Figure& size(int w, int h) { width_ = w; height_ = h; return *this; }

    std::string svg() const;                                 // the <svg>…</svg> element
    void save(const std::string& htmlPath) const;            // one figure per page
    static void savePage(const std::string& htmlPath, const std::vector<Figure>& figures);  // stacked

private:
    struct Series { Vector x, y; std::string label; bool markers; std::string marker; };
    struct Guide  { bool horizontal; Real value; std::string label; };
    std::string title_, xlabel_, ylabel_;
    std::vector<Series> series_;
    std::vector<Guide>  guides_;
    bool logX_ = false, logY_ = false;
    bool hasXlim_ = false, hasYlim_ = false;
    Real xlo_ = 0, xhi_ = 1, ylo_ = 0, yhi_ = 1;
    int  width_ = 820, height_ = 420;

    void dataRange(Real& xlo, Real& xhi, Real& ylo, Real& yhi) const;
    static std::vector<Real> ticks(Real lo, Real hi, bool log);   // "nice" tick positions
};

} // namespace csl::plot
```

The core of `svg()` (the rest is layout):

```cpp
std::string Figure::svg() const {
    const int L = 70, R = 20, T = 40, Bm = 50;             // margins
    const Real pw = width_ - L - R, ph = height_ - T - Bm;
    Real xlo, xhi, ylo, yhi; dataRange(xlo, xhi, ylo, yhi);
    auto fx = [&](Real x) { Real v = logX_ ? std::log10(x) : x, a = logX_ ? std::log10(xlo) : xlo,
                            b = logX_ ? std::log10(xhi) : xhi;  return L + (v - a) / (b - a) * pw; };
    auto fy = [&](Real y) { Real v = logY_ ? std::log10(y) : y, a = logY_ ? std::log10(ylo) : ylo,
                            b = logY_ ? std::log10(yhi) : yhi;  return T + ph - (v - a) / (b - a) * ph; };
    static const char* palette[] = {"#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"};

    std::ostringstream s;
    s << std::setprecision(8);
    s << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width_ << "' height='" << height_
      << "' font-family='sans-serif' font-size='12'>\n";
    // grid + ticks
    for (Real xt : ticks(xlo, xhi, logX_)) {
        s << "<line x1='" << fx(xt) << "' y1='" << T << "' x2='" << fx(xt) << "' y2='" << T + ph
          << "' stroke='#ddd'/>\n<text x='" << fx(xt) << "' y='" << T + ph + 16
          << "' text-anchor='middle'>" << xt << "</text>\n";
    }
    for (Real yt : ticks(ylo, yhi, logY_)) {
        s << "<line x1='" << L << "' y1='" << fy(yt) << "' x2='" << L + pw << "' y2='" << fy(yt)
          << "' stroke='#ddd'/>\n<text x='" << L - 6 << "' y='" << fy(yt) + 4
          << "' text-anchor='end'>" << yt << "</text>\n";
    }
    // series
    int c = 0;
    for (const Series& sr : series_) {
        const char* color = palette[c++ % 6];
        if (!sr.markers) {
            s << "<polyline fill='none' stroke='" << color << "' stroke-width='1.5' points='";
            for (Index i = 0; i < sr.x.size(); ++i)
                if (std::isfinite(sr.x(i)) && std::isfinite(sr.y(i)) && (!logX_ || sr.x(i) > 0) && (!logY_ || sr.y(i) > 0))
                    s << fx(sr.x(i)) << "," << fy(sr.y(i)) << " ";
            s << "'/>\n";
        } else {
            for (Index i = 0; i < sr.x.size(); ++i) {
                if (sr.marker == "x")
                    s << "<path d='M" << fx(sr.x(i)) - 4 << "," << fy(sr.y(i)) - 4 << " l8,8 m0,-8 l-8,8' stroke='" << color << "' stroke-width='1.5'/>\n";
                else
                    s << "<circle cx='" << fx(sr.x(i)) << "' cy='" << fy(sr.y(i)) << "' r='3.5' fill='none' stroke='" << color << "' stroke-width='1.5'/>\n";
            }
        }
    }
    // axes box, title, labels, legend  (straightforward <rect>, <text> elements)
    ...
    s << "</svg>\n";
    return s.str();
}
```

`ticks()`: for linear axes, `step = niceStep((hi−lo)/6)` where `niceStep` rounds to `{1, 2, 5} × 10ᵏ`; produce multiples of `step` within `[lo, hi]`. For log axes, one tick per decade (`10ᵏ`). `dataRange()`: min/max over all finite series points unless `xlim/ylim` were set, with 5 % padding on y. `save()` writes `<!doctype html><html><body>` + `svg()` + `</body></html>`.

```cpp
// include/csl/simulation/TimeResponse.hpp
#pragma once
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/LTISystem.hpp"

namespace csl::simulation {

// tf <= 0 or dt <= 0 => choose automatically from the poles.
SimResult step   (const systems::LTISystem& sys, Real tf = -1, Real dt = -1, Index inputIndex = 0);
SimResult impulse(const systems::LTISystem& sys, Real tf = -1, Real dt = -1, Index inputIndex = 0);
// u: N x m samples at times t (uniform spacing). Input is held constant between samples (ZOH).
SimResult lsim   (const systems::LTISystem& sys, const Matrix& u, const Vector& t,
                  const Vector& x0 = Vector());

void autoHorizon(const systems::LTISystem& sys, Real& tf, Real& dt);

} // namespace csl::simulation
```

`lsim` is the worker: convert with `toStateSpace`, then for continuous systems loop over samples applying `rk4Step` with `u` fixed per interval (`r.x`, `r.y = C x + D u`, `r.u`); for discrete systems apply the recursion. `step` builds a constant `u` matrix and calls `lsim`; `impulse` calls `lsim` with `u = 0` and `x0 = B.col(inputIndex)`.

```cpp
// include/csl/plot/Plots.hpp  (Day 15 part)
#pragma once
#include <string>
#include "csl/plot/Figure.hpp"
#include "csl/simulation/SimResult.hpp"

namespace csl::plot {
Figure plotOutputs(const simulation::SimResult& r, const std::string& title = "");   // y columns vs t
Figure plotStates (const simulation::SimResult& r, const std::string& title = "");
}
```

**How it connects.** `step`/`impulse`/`lsim` are used by specs (Day 16), by every controller example, by identification experiments (Day 56: generate data with `lsim`, identify, compare), and by the `Simulator` (Day 24 generalises `lsim` to closed loop). `Figure` is used by every `plot*` function from here to Day 70.

**Verify.** `step(plants::thermalTF())` at `t = 2` gives `1.8964 ± 1e-6`; `step(E2)` settles to `0.25` and peaks at `0.418 ± 0.002` near `t = 1.58`; `impulse(E1)` at `t = 0` equals `K/τ = 1.5` and `impulse(E2).y` numerically differentiated… simpler: `impulse(E2).y` equals the finite-difference derivative of `step(E2).y` to `1e-3`; `lsim` of E1 with `u = sin(t)` matches the analytic steady state after 10 s. Open `step_msd.html` in a browser: axes, grid, legend, title present. Tests on `Figure` are structural: the SVG string contains one `<polyline` per series and the title text.

Reading: C++ Primer §8.1–8.3, §17.5 (formatted I/O); learncpp 28.x. SVG: the MDN "SVG: polyline / text / line" pages (15 minutes). Control: FPE §3.3 (time response), AM §6.3.

**Done when.**
- [ ] `Figure` renders a multi-series HTML plot with log-axis support
- [ ] `step`, `impulse`, `lsim` for continuous and discrete systems, tests passing
- [ ] `ex01` saves `step_thermal.html` and you looked at it
- [ ] Commit

---

#### Day 16 — STL algorithms and iterators: time-domain specifications and signal generators

**C++ you learn.** The standard algorithms, and the iterator concept that makes them work on any container.

- An **iterator** is a position in a container; `v.begin()` is the first element, `v.end()` is *one past* the last. Algorithms take `[first, last)` ranges and never know what container they came from — Eigen vectors provide `begin()/end()` since 3.4, so every algorithm below works on `Vector` directly.
- `std::max_element(first, last)` → iterator to the max; `std::distance(first, it)` → index. `std::find_if(first, last, predicate)` → first element satisfying a lambda. Reverse search: `std::find_if(v.rbegin(), v.rend(), pred)` then convert with `std::distance(it, v.rend()) − 1`. `std::accumulate` (in `<numeric>`), `std::transform`, `std::count_if`, `std::all_of/any_of`.
- Prefer an algorithm to a hand-written loop when it *names the intent* (`max_element` says "I want the maximum"); keep the loop when it is clearer.
- **`struct` with many fields + a `print()`** for `StepInfo`, mirroring MATLAB's `stepinfo`.
- Signal generators as small free functions returning `Vector` given a time grid; `std::sin`, `std::fmod`.

**Control you learn.** Reading a step response quantitatively, and the second-order approximations that let you go from specs to pole locations (and back).

| Spec | Definition | Second-order formula (`ζ < 1`) |
|---|---|---|
| Steady-state value `y_ss` | final value (dc gain × step size) | `H(0)` |
| Rise time `t_r` | 10 % → 90 % of `y_ss` | `≈ 1.8/ωn` (ζ ≈ 0.5); exact value needs the curve |
| Peak time `t_p` | time of the maximum | `π/ωd`, `ωd = ωn√(1−ζ²)` |
| Overshoot `M_p` | `(y_max − y_ss)/y_ss` | `exp(−πζ/√(1−ζ²))` |
| Settling time `t_s` | last time `|y − y_ss| > 2 % y_ss` | `≈ 4/(ζωn)` |

Design reading: "`M_p < 10 %` ⇒ `ζ > 0.59`"; "`t_s < 1 s` ⇒ `Re p < −4`"; "`t_r < 0.5 s` ⇒ `ωn > 3.6`". These inequalities define a region in the s-plane; pole placement (Day 38) and root locus (Day 21) are about putting the closed-loop poles inside it.

Standard test signals: step, ramp (tests type-1 tracking — a system with one integrator follows a step with zero error but a ramp with constant error), sinusoid (frequency response, Day 17), chirp (frequency sweep, for identification on Day 58), square wave.

**Files you touch.** `include/csl/simulation/TimeResponse.hpp` (+ `StepInfo`, `stepInfo`), `src/simulation/TimeResponse.cpp`, `include/csl/signals/Signals.hpp`, `src/signals/Signals.cpp`, tests.

**Code.**

```cpp
// TimeResponse.hpp — additions
namespace csl::simulation {
struct StepInfo {
    Real steadyState, riseTime, peakTime, peak, overshootPercent, settlingTime;
    Real settlingThreshold;   // e.g. 0.02
};
StepInfo stepInfo(const SimResult& r, Index outputIndex = 0, Real settlingThreshold = 0.02);
StepInfo stepInfo(const systems::LTISystem& sys, Index outputIndex = 0, Real settlingThreshold = 0.02);
}
```

```cpp
StepInfo stepInfo(const SimResult& r, Index j, Real thr) {
    const Vector y = r.y.col(j);
    const Vector& t = r.t;
    StepInfo s{};
    s.settlingThreshold = thr;
    s.steadyState = y.tail(std::max<Index>(1, y.size() / 50)).mean();     // mean of the last 2 %
    const Real yss = s.steadyState;

    auto peakIt = std::max_element(y.begin(), y.end());
    s.peak = *peakIt;
    s.peakTime = t(std::distance(y.begin(), peakIt));
    s.overshootPercent = yss != 0.0 ? std::max(0.0, (s.peak - yss) / std::abs(yss) * 100.0) : 0.0;

    auto t10 = std::find_if(y.begin(), y.end(), [&](Real v) { return v >= 0.1 * yss; });
    auto t90 = std::find_if(y.begin(), y.end(), [&](Real v) { return v >= 0.9 * yss; });
    s.riseTime = t(std::distance(y.begin(), t90)) - t(std::distance(y.begin(), t10));

    auto lastOut = std::find_if(y.rbegin(), y.rend(), [&](Real v) { return std::abs(v - yss) > thr * std::abs(yss); });
    s.settlingTime = (lastOut == y.rend()) ? 0.0 : t(std::distance(lastOut, y.rend()) - 1);
    return s;
}
```

(Interpolate between samples for `riseTime`/`settlingTime` if you want sub-sample accuracy — a good exercise, not required.)

```cpp
// include/csl/signals/Signals.hpp  (Day 16 part; Day 50 adds WhiteNoise and PRBS)
#pragma once
#include "csl/core/Types.hpp"
namespace csl::signals {
Vector timeGrid(Real t0, Real tf, Real dt);                          // same grid rule as integrate
Vector step  (const Vector& t, Real amplitude = 1.0, Real tStart = 0.0);
Vector ramp  (const Vector& t, Real slope = 1.0, Real tStart = 0.0);
Vector sine  (const Vector& t, Real amplitude, Real w_rad_s, Real phase = 0.0);
Vector square(const Vector& t, Real amplitude, Real period);
Vector chirp (const Vector& t, Real f0_hz, Real f1_hz, Real amplitude = 1.0);   // linear sweep
}
```

**How it connects.** `stepInfo` is how every controller design in Phases 4–7 is judged. `signals::*` feed `lsim` now, the `Simulator` later, identification on Days 56–58.

**Verify.** `stepInfo(plants::massSpringDamper())` (simulated with `dt = 0.001`): `steadyState = 0.25`, `overshootPercent = 67.3 ± 0.3`, `peakTime = 1.583 ± 0.002`, `riseTime = 0.56 ± 0.01`, `settlingTime ≈ 14.5 ± 0.2` (the rule of thumb `4/(ζωn)` gives 16 — the rule is conservative). For E1: `riseTime = τ ln 9 = 4.394 ± 0.01`, `settlingTime = τ ln 50 = 7.82 ± 0.02`, overshoot 0. `lsim(E1, ramp)` output lags the ramp by `τ` in steady state (`y(t) ≈ K(t − τ)`).

Reading: C++ Primer §10.1–10.3, §9.2 (iterators); learncpp 18.x (iterators, algorithms). Control: FPE §3.4 (specs), AM §6.3.

**Done when.**
- [ ] `stepInfo` + five signal generators with tests
- [ ] You can state the four second-order formulas and use them backwards (spec → pole region)
- [ ] Commit `Day 16: Phase 2 complete`

---
### Phase 3 — The frequency domain (Days 17–22)

**Phase goal.** Frequency response, Bode, Nyquist, margins, interconnection algebra (`series`, `parallel`, `feedback`, sensitivity), minimum phase and delay, root locus, Routh–Hurwitz — all implemented, plotted and cross-checked against each other. In C++ terms: `std::optional`, overload sets, free-function operators, sorting with predicates, header organisation.

---

#### Day 17 — Frequency response and the Bode plot

**C++ you learn.**

- Small **inline utility functions in a header** (`MathUtils.hpp`): `dB`, `fromDB`, `rad2deg`, `linspace`, `logspace`, `wrapToPi`, `unwrapPhase`. `inline` on a function defined in a header is required (ODR); for one-liners it is also a hint to the optimiser.
- **Returning a struct** (`FrequencyResponse`) rather than two vectors by reference — the convention from Section 4.3.4.
- **`std::arg`, `std::abs`, `std::exp(Complex)`** for the discrete-time case (`z = e^{jωTs}`).
- Extending `Figure` with log axes (already designed in; today they are exercised), and `Figure::savePage` for stacked magnitude/phase plots.

**Control you learn.** What `H(jω)` means and how to read a Bode plot.

- **Sinusoidal steady state.** Feed a stable LTI system `u = sin ωt`. After transients die, `y = |H(jω)| sin(ωt + ∠H(jω))`. Proof sketch: `e^{jωt}` is an eigenfunction of convolution; `H(jω)` is the eigenvalue. So the complex number `H(jω)` *is* "gain and phase shift at frequency ω". Frequency response = `H(s)` evaluated on the imaginary axis.
- **Bode plot**: magnitude in decibels `20 log₁₀|H|` vs. `log ω`, and phase in degrees vs. `log ω`. Logs turn products into sums, so a transfer function's plot is the sum of its factors' plots. The asymptotes:

| Factor | Magnitude slope beyond corner | Phase contribution |
|---|---|---|
| gain `K` | 0 dB/dec, level `20 log K` | 0° (180° if K < 0) |
| integrator `1/s` | −20 dB/dec everywhere | −90° |
| real pole `1/(s/a + 1)` | 0 → −20 dB/dec at `ω = a` | 0° → −90°, −45° at `a` |
| real zero `(s/a + 1)` | 0 → +20 dB/dec | 0° → +90° |
| complex pair, `ζ` | 0 → −40 dB/dec at `ωn`, peak `1/(2ζ√(1−ζ²))` for `ζ < 0.707` | 0° → −180°, sharper for small ζ |

- E1's corner frequency is `1/τ = 0.5 rad/s`: magnitude is 3 dB down and phase −45° there — the same τ you measured in time on Day 2, seen from the frequency side. E2 shows a resonance peak near `ωn = 2`.
- **Phase unwrapping**: `std::arg` returns values in `(−π, π]`, so phase that passes −180° jumps to +180°. Unwrap by adding ±2π whenever consecutive samples differ by more than π. Everything downstream (margins, Nyquist) needs the unwrapped phase.
- **Discrete systems**: `z = e^{jωTs}`; the response is periodic in ω with period `2π/Ts` and only meaningful up to the Nyquist frequency `π/Ts` (Day 29).

**Files you touch.** `include/csl/core/MathUtils.hpp`, `include/csl/analysis/FrequencyResponse.hpp`, `src/analysis/FrequencyResponse.cpp`, `include/csl/plot/Plots.hpp` + `.cpp` (`plotBode`), tests.

**Code.**

```cpp
// include/csl/core/MathUtils.hpp
#pragma once
#include <cmath>
#include "csl/core/Types.hpp"
namespace csl {
inline Real dB(Real magnitude)     { return 20.0 * std::log10(magnitude); }
inline Real fromDB(Real db)        { return std::pow(10.0, db / 20.0); }
inline Real rad2deg(Real r)        { return r * 180.0 / PI; }
inline Real deg2rad(Real d)        { return d * PI / 180.0; }
inline Real wrapToPi(Real a)       { return std::atan2(std::sin(a), std::cos(a)); }
Vector linspace(Real a, Real b, Index n);
Vector logspace(Real exp10_a, Real exp10_b, Index n);     // 10^a ... 10^b
void   unwrapPhase(Vector& phase_rad);                     // in place
}
```

```cpp
// include/csl/analysis/FrequencyResponse.hpp
#pragma once
#include "csl/core/Types.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::analysis {

struct FrequencyResponse {          // Section 4.3.4
    Vector        w;                // rad/s
    ComplexVector H;
    Real          Ts = 0.0;
};
struct BodeData { Vector w, magnitude_dB, phase_deg; };   // phase unwrapped

Vector            defaultFrequencies(const systems::LTISystem& sys, Index n = 400);
FrequencyResponse frequencyResponse(const systems::LTISystem& sys, const Vector& w);
FrequencyResponse frequencyResponse(const systems::LTISystem& sys);    // default grid
BodeData          bode(const FrequencyResponse& fr);

} // namespace csl::analysis
```

```cpp
FrequencyResponse frequencyResponse(const systems::LTISystem& sys, const Vector& w) {
    if (!sys.isSISO()) throw NotSupported("frequencyResponse: SISO only");
    FrequencyResponse fr;
    fr.w = w; fr.Ts = sys.Ts(); fr.H.resize(w.size());
    for (Index k = 0; k < w.size(); ++k) {
        const Complex s = sys.isContinuous() ? Complex(0.0, w(k))
                                             : std::exp(Complex(0.0, w(k) * sys.Ts()));
        fr.H(k) = sys.evaluate(s)(0, 0);
    }
    return fr;
}

BodeData bode(const FrequencyResponse& fr) {
    BodeData b; b.w = fr.w;
    b.magnitude_dB = fr.H.cwiseAbs().unaryExpr([](Real m) { return dB(m); });
    Vector ph(fr.H.size());
    for (Index k = 0; k < ph.size(); ++k) ph(k) = std::arg(fr.H(k));
    unwrapPhase(ph);
    b.phase_deg = ph.unaryExpr([](Real r) { return rad2deg(r); });
    return b;
}
```

`defaultFrequencies`: two decades below the smallest nonzero `|pole|`/`|zero|` to two decades above the largest; for a discrete system cap the top at `0.999·π/Ts`. `plotBode(const std::vector<std::pair<FrequencyResponse, std::string>>&, path)` draws two stacked figures (magnitude, phase) with `logX`, `hline(0)` on magnitude and `hline(−180)` on phase.

**How it connects.** `FrequencyResponse` is consumed by `margins`/`nyquist` (Day 18), the sensitivity functions (Day 28), filter-response plots (Day 33), `etfe` produces the same struct (Day 58), and `hinfnorm` checks against it (Day 61).

**Verify.** E1 at `ω = 0.5`: `|H| = 3/√2` → `6.532 dB`, phase `−45.00°`. E2 at `ω = 2`: `0 dB`, `−90°`; at `ω = 0.01`: `−12.04 dB`; peak magnitude `0.07 dB` near `ω = 1.968`. `1/s` at `ω = 1`: `0 dB`, `−90°`. For a discrete `TransferFunction{{1},{1,−0.5}, 0.1}` at `ω = 0`: `H = 2`. Unwrap test: feed `[3.0, −3.0, 3.0]` → `[3.0, 3.283, 3.0]`.

Reading: FPE §6.1 (frequency response — the central section of the book), AM §9.4–9.5, §10.1. C++: nothing new.

**Done when.**
- [ ] `frequencyResponse`, `bode`, `plotBode`, `MathUtils` with tests
- [ ] You sketched E2's Bode plot by hand from the asymptote table *before* plotting it, and they agree
- [ ] Commit

---

#### Day 18 — Nyquist, the stability criterion, and gain/phase margins (`std::optional`)

**C++ you learn.**

- **`std::optional<T>`** (`#include <optional>`): a value that may be absent. `std::optional<Real> gm; if (gm) use(*gm); gm.value_or(INF); gm.has_value()`. It is the right return type when "no answer" is a *normal* outcome (a loop with no phase crossover has infinite gain margin — not an error). Never use NaN or −1 for this.
- **Interpolation helper** with a clear contract (find sign changes in a sampled function, refine by linear interpolation in `log ω`).
- Returning a struct of optionals plus booleans (`Margins`).

**Control you learn.** Stability of the closed loop from the open loop.

- **Nyquist plot**: the curve `L(jω)` in the complex plane for ω from 0 to ∞ (mirror for negative ω). `L` is the **loop transfer function** — controller times plant (`C·P`) — not the closed loop.
- **Nyquist criterion**: `Z = N + P`, where `P` = number of open-loop poles in the RHP, `N` = net clockwise encirclements of `−1` by the full Nyquist contour, `Z` = closed-loop RHP poles. For the common case `P = 0`: *closed loop is stable iff the Nyquist plot does not encircle −1.* Intuition: `1 + L(jω) = 0` would mean a sinusoid sustains itself around the loop; the plot shows how close `L` gets to that point.
- **Gain margin** GM: at the **phase crossover** `ω_pc` where `∠L = −180°`, `GM = 1/|L(jω_pc)|` (in dB: `−20 log|L|`). How much the gain can increase before instability.
- **Phase margin** PM: at the **gain crossover** `ω_gc` where `|L| = 1`, `PM = 180° + ∠L(jω_gc)`. How much extra phase lag (e.g. from delay) can be tolerated. **Delay margin** `= PM (rad) / ω_gc` seconds.
- Typical design targets: `GM ≥ 6 dB`, `PM 30°–60°`. PM ≈ `100·ζ` for a second-order loop — the frequency-domain handle on damping (Day 16's ζ, again).
- Margins can fail to exist (no crossover) and can be misleading for conditionally stable or multi-crossover loops; peak sensitivity `Ms` (Day 28) is the more honest single number.

**Files you touch.** `FrequencyResponse.hpp/.cpp` (+ `Margins`, `margins`, `NyquistData`, `nyquist`), `Plots.hpp/.cpp` (`plotNyquist`), tests.

**Code.**

```cpp
struct Margins {
    std::optional<Real> gainMargin_dB, phaseCrossover_w;    // absent if phase never reaches -180
    std::optional<Real> phaseMargin_deg, gainCrossover_w;   // absent if |L| never crosses 1
    std::optional<Real> delayMargin_s;
    bool closedLoopStable = false;                          // filled from Day 19 (feedback + isStable)
};
Margins margins(const systems::LTISystem& L);
Margins margins(const FrequencyResponse& fr);               // from sampled data (ETFE on Day 58!)

struct NyquistData { Vector w; Vector re, im; };
NyquistData nyquist(const FrequencyResponse& fr);
```

Algorithm for `margins(fr)`: compute `BodeData b = bode(fr)`. Gain crossover: scan `b.magnitude_dB` for a sign change; interpolate `log ω` linearly to the zero; evaluate phase there by interpolation; `PM = 180 + phase`. If several crossovers, take the one giving the *smallest* PM. Phase crossover: scan `b.phase_deg + 180` for sign changes; `GM_dB = −magnitude_dB` at the crossing with the smallest GM. (Make `phase_deg` relative: unwrapped phase may start at −360° for some loops — compare against the nearest odd multiple of 180° by taking `wrapToPi(deg2rad(phase)) + π`.)

**How it connects.** `margins` is the acceptance test for every classical design (Days 26–28) and for digital implementations (Day 35, where sampling eats phase margin). On Day 58 `margins(etfe(...))` computes margins from *measured* data with no model at all — the same function.

**Verify.** `L = 1/(s(s+1)(s+2))` (`TransferFunction{{1},{1,3,2,0}}`): `GM = 15.56 dB` (factor 6) at `ω_pc = 1.414`, `PM = 53.4°` at `ω_gc = 0.446`, delay margin `2.09 s` — all `±0.5 %`. `L = 1/(s+1)`: the phase never reaches −180°, so `gainMargin_dB` is absent, and `|L| < 1` for every ω > 0, so `phaseMargin_deg` is absent too. `L = 10/(s(s+1))`: `PM ≈ 18.0°` at `ω_gc ≈ 3.08`. Nyquist of `1/(s(s+1)(s+2))` passes through `(−1/6, 0)` at `ω = √2`.

Reading: FPE §6.3–6.4 (Nyquist, margins) — read slowly; AM §10.1–10.3. C++: cppreference `std::optional`.

**Done when.**
- [ ] `margins`, `nyquist`, `plotNyquist` with the four tests above
- [ ] You can explain, with the Nyquist picture, why GM is measured at −180° and PM at |L| = 1
- [ ] Commit

---

#### Day 19 — Interconnections: `series`, `parallel`, `feedback`, sensitivity (operators on systems)

**C++ you learn.**

- **Overload sets across types**: `series(const TransferFunction&, const TransferFunction&)` and `series(const StateSpace&, const StateSpace&)` coexist; the compiler picks by argument types. A third overload taking `const LTISystem&` pairs converts both to state space — the general fallback.
- **Operators as free functions** for TFs: `operator*` = series, `operator+` = parallel. Not for `StateSpace` (would `*` mean series or element-wise? ambiguous ⇒ no operator).
- **Block-matrix assembly** with Eigen's comma initialiser `M << A, B, C, D;` (block sizes must be consistent) and `Matrix::Zero` fill.
- Mixed continuous/discrete or mismatched `Ts` is an error — check and throw `InvalidArgument`.

**Control you learn.** Block-diagram algebra and the four closed-loop transfer functions.

- Series `H₂H₁` (signal passes through `H₁` then `H₂`): TF multiply numerators and denominators; state space:

```
A = [ A₁    0  ]   B = [ B₁  ]   C = [ D₂C₁  C₂ ]   D = D₂D₁
    [ B₂C₁  A₂ ]       [ B₂D₁]
```

- Parallel `H₁ + H₂`: `A = diag(A₁, A₂)`, `B = [B₁; B₂]`, `C = [C₁ C₂]`, `D = D₁ + D₂`.
- Negative feedback of `G` with `H` in the return path (closed loop from `r` to `y`): `G/(1 + GH)`. In state space, with `u = r − y_h`, `y = Cx + Du`, `y_h = C_h x_h + D_h y` and `E = I + D D_h` (must be invertible — an algebraic loop otherwise):

```
y   = E⁻¹ (C x − D C_h x_h + D r)
u   = r − C_h x_h − D_h y
ẋ   = A x + B u
ẋ_h = A_h x_h + B_h y
```

Substitute and collect the coefficients of `x`, `x_h`, `r` into `A_cl`, `B_cl`, `C_cl`, `D_cl`. Write the algebra out on paper first; the code is the paper transcribed.
- Unity feedback with loop `L = CP`: closed-loop `T = L/(1+L)` (reference → output, the **complementary sensitivity**), `S = 1/(1+L)` (output disturbance → output, and reference → error, the **sensitivity**), with `S + T = 1` identically. Control sensitivity `CS = C/(1+L)` (reference → control effort) and load sensitivity `PS = P/(1+L)` (input disturbance → output). These four are the "gang of four"; you look at all four before calling a design good.
- The **closed-loop poles** are the roots of `1 + L(s) = 0`, i.e. of `den_L + num_L`. The Nyquist criterion (yesterday) and the root locus (Day 21) are two ways of studying that equation.

**Files you touch.** `include/csl/systems/Interconnect.hpp`, `src/systems/Interconnect.cpp`, `TransferFunction.hpp` (operators), `FrequencyResponse.cpp` (`margins(...).closedLoopStable` via `feedback(L, unity).isStable()`), tests.

**Code.**

```cpp
// include/csl/systems/Interconnect.hpp
#pragma once
#include "csl/systems/StateSpace.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::systems {

TransferFunction series  (const TransferFunction& first, const TransferFunction& second); // second * first
TransferFunction parallel(const TransferFunction& a, const TransferFunction& b);
TransferFunction feedback(const TransferFunction& G, const TransferFunction& H, int sign = -1);
TransferFunction feedback(const TransferFunction& G, int sign = -1);        // unity feedback

StateSpace series  (const StateSpace& first, const StateSpace& second);
StateSpace parallel(const StateSpace& a, const StateSpace& b);
StateSpace feedback(const StateSpace& G, const StateSpace& H, int sign = -1);
StateSpace feedback(const StateSpace& G, int sign = -1);

inline TransferFunction operator*(const TransferFunction& a, const TransferFunction& b) { return series(b, a); } // a*b = "a after b", like matrices
inline TransferFunction operator+(const TransferFunction& a, const TransferFunction& b) { return parallel(a, b); }

// Gang of four for loop L = C*P (unity negative feedback)
struct GangOfFour { TransferFunction S, T, CS, PS; };
GangOfFour gangOfFour(const TransferFunction& C, const TransferFunction& P);

} // namespace csl::systems
```

```cpp
TransferFunction feedback(const TransferFunction& G, const TransferFunction& H, int sign) {
    checkSameTs(G, H);
    // G / (1 - sign*G*H) = (nG dH) / (dG dH - sign nG nH)
    const Polynomial num = G.num() * H.den();
    const Polynomial den = G.den() * H.den() - static_cast<Real>(sign) * (G.num() * H.num());
    return TransferFunction(num, den, G.Ts());
}
```

**How it connects.** `feedback` turns an open-loop design into the closed loop that `step`/`stepInfo` evaluate (Days 23–28). `gangOfFour` feeds the sensitivity analysis of Day 28 and the robustness tests of Day 60. The state-space `feedback` is used by observer-based control (Day 39) and LQG (Day 52).

**Verify.** E2 under proportional control `K = 10` (`feedback(K·P)`): closed-loop den `{1, 0.5, 14}`, dc gain `10/14`. `feedback(1/(s(s+1)(s+2)))` stable; with gain 7 unstable; with 6 marginal (poles at `±j√2`) — Day 18's gain margin confirmed. `series(tf2ss(a), tf2ss(b))` and `series(a, b)` agree in `evaluate(1 + j)` to `1e-9`. `S + T` isApprox `1` as a TF (after `minreal`). `feedback(G, H)` with `D_G = D_H = 1` throws (algebraic loop `E` singular when `sign = −1`? — `E = I + 1·1 = 2`, fine; make `D_H = −1` to get `E = 0`, that throws).

Reading: FPE §3.2 (block diagrams), AM §9.3 (interconnections), §11.1 (gang of four — AM calls them the "Gang of Six" with feedforward).

**Done when.**
- [ ] All interconnections for TF and SS with tests; `margins(...).closedLoopStable` works
- [ ] You can derive the feedback state-space formula on paper
- [ ] Commit

---

#### Day 20 — Minimum phase, right-half-plane zeros, time delay and Padé approximation

**C++ you learn.**

- **Static factory functions** (`TransferFunction::pade(T, n)`), `std::tgamma` for factorials (or a small loop), **building coefficient vectors in ascending order then reversing** (`std::reverse` or Eigen `.reverse()`).
- `std::any_of` with a lambda for "is any zero in the RHP".
- A design note on **naming for the reader**: `isMinimumPhase`, not `checkZeros`.

**Control you learn.** The things a Bode magnitude plot does *not* tell you.

- A zero at `s = −z` and a zero at `s = +z` give *identical* magnitude plots and *opposite* phase (lead vs. lag). A system is **minimum phase** if all zeros (and poles) are in the LHP: its phase is the smallest possible for its magnitude, and the magnitude plot alone determines it (Bode's gain–phase relation). **Non-minimum-phase** (NMP) systems have RHP zeros: extra phase lag that no controller can remove.
- Consequences of an RHP zero at `+z`: the step response starts in the *wrong direction* (undershoot); achievable closed-loop bandwidth is limited to roughly `z/2`; aggressive control makes things worse. Classic physical examples: the water level in a boiler drum (shrink/swell), a bicycle's steering, an aircraft's altitude response to elevator.
- **All-pass factor** `(z − s)/(z + s)`: unit magnitude at all frequencies, pure phase lag. Any NMP system = minimum-phase part × all-pass part.
- **Time delay** `e^{−sT}`: magnitude 1, phase `−ωT` (radians) — unbounded lag, hence Day 18's delay margin. It is not rational, so it needs an approximation to enter the TF machinery: the **Padé approximation** of order `n`,

```
e^{−sT} ≈ P_n(−sT) / P_n(sT),   P_n(x) = Σ_{k=0}^{n} [(2n−k)! n!] / [(2n)! k! (n−k)!] · x^k
```

First order: `(1 − sT/2)/(1 + sT/2)` — an RHP zero at `2/T`. A delay *is* non-minimum-phase behaviour. Higher orders match the phase to higher frequency (`n = 3` is accurate to about `ωT ≈ 3`).

**Files you touch.** `TransferFunction.hpp/.cpp` (`pade`, `isMinimumPhase`), `Stability.hpp` begins (just `isMinimumPhase(const LTISystem&)` — Day 22 fills it), tests.

**Code.**

```cpp
TransferFunction TransferFunction::pade(Real T, int order) {
    if (T <= 0 || order < 1) throw InvalidArgument("pade: T > 0 and order >= 1 required");
    const int n = order;
    auto fact = [](int k) { Real f = 1; for (int i = 2; i <= k; ++i) f *= i; return f; };
    Vector numAsc(n + 1), denAsc(n + 1);                 // ascending powers of s
    for (int k = 0; k <= n; ++k) {
        const Real c = fact(2 * n - k) * fact(n) / (fact(2 * n) * fact(k) * fact(n - k));
        denAsc(k) = c * std::pow(T, k);
        numAsc(k) = c * std::pow(-T, k);
    }
    return TransferFunction(Polynomial(Vector(numAsc.reverse())), Polynomial(Vector(denAsc.reverse())));
}

bool isMinimumPhase(const systems::LTISystem& sys) {
    const auto z = sys.zeros();
    const auto p = sys.poles();
    auto inRHP = [&](const Complex& c) { return sys.isContinuous() ? c.real() > 0 : std::abs(c) > 1; };
    return !std::any_of(z.begin(), z.end(), inRHP) && !std::any_of(p.begin(), p.end(), inRHP);
}
```

**How it connects.** `pade` lets delays enter `series(...)`, `margins`, root locus and the Simulator (as part of the plant). Day 58 identifies FOPDT models (gain, time constant, delay) from data and needs `pade` to compare them. Day 60's robustness analysis treats delay as a multiplicative uncertainty.

**Verify.** `pade(1, 1)` isApprox `{−0.5, 1}/{0.5, 1}` (as monic: `{−1, 2}/{1, 2}`). Phase of `pade(1, 3)` at `ω = 1` is `−57.30° ± 0.01°` (exact `−ωT = −1 rad`), at `ω = 3`: `−171.9°` vs exact `−171.9°`, at `ω = 6` noticeably off. Magnitude is `0 dB` at every ω (all-pass). `step` of `(1 − s)/((s+1)(s+2))`: `y` is negative for small `t` (initial slope `−1`), final value `0.5`. `isMinimumPhase` is false for it and true for E1–E3. Adding `pade(0.5, 2)` in series with `L = 1/(s(s+1)(s+2))` reduces PM from `53.4°` to about `40.7°` (`53.4 − 57.3·0.446·0.5`).

Reading: FPE §6.1.2 (non-minimum phase), §6.6 (delay); AM §9.4 (zeros), §11.4 (limits of performance — skim now, return on Day 60).

**Done when.**
- [ ] `pade`, `isMinimumPhase`, tests
- [ ] You can say in one sentence what an RHP zero does to a step response and to achievable bandwidth
- [ ] Commit

---

#### Day 21 — Root locus

**C++ you learn.**

- **Nested containers**: `std::vector<std::vector<Complex>>` — `branches[b][k]` = position of branch `b` at gain index `k`.
- **Greedy matching** of root sets between consecutive gains (nearest-neighbour with a `std::vector<bool> used` mask) so the branches are continuous curves rather than a scrambled point cloud.
- **`std::sort` with a lambda comparator** (again) and `std::min_element` with a custom key.
- Plotting with markers: `Figure::scatter(..., "x")` for poles, `"o"` for zeros, lines for branches.

**Control you learn.** How the closed-loop poles move as a single gain changes.

- Closed-loop characteristic equation with loop `K·L(s)`, `L = N/D`: `1 + K N(s)/D(s) = 0 ⇔ D(s) + K N(s) = 0`. The **root locus** is the set of roots of this polynomial as `K` goes from 0 to ∞. At `K = 0` the roots are the open-loop poles; as `K → ∞`, `n − m` of them go to the open-loop zeros and the remaining `n − m` go to infinity along asymptotes.
- Sketching rules (Evans): (1) branches start at poles, end at zeros/infinity; (2) real-axis segments to the left of an odd number of real poles+zeros are on the locus; (3) asymptote angles `(2k+1)·180°/(n−m)`, centroid `(Σ pᵢ − Σ zᵢ)/(n−m)`; (4) breakaway points where `dK/ds = 0`; (5) imaginary-axis crossings found from Routh (tomorrow) or from `K` at which `D + KN` has a root on `jω` — which is the gain margin (Day 18) once more.
- Design use: draw the lines of constant `ζ` (`∠p = 180° ± acos ζ`) and constant `ωn` (circles); pick the `K` where the dominant branch crosses the desired region (Day 16's spec region). Lead/lag (Day 27) bends the locus by adding poles/zeros.
- The library computes the locus numerically (repeated `Polynomial::roots`), which is the modern way; the rules are for *reading* and *sanity-checking* it.

**Files you touch.** `include/csl/analysis/RootLocus.hpp`, `src/analysis/RootLocus.cpp`, `Plots.hpp/.cpp` (`plotRootLocus`, `plotPoleZero`), tests.

**Code.**

```cpp
// include/csl/analysis/RootLocus.hpp
#pragma once
#include <vector>
#include "csl/systems/TransferFunction.hpp"
namespace csl::analysis {
struct RootLocusData {
    Vector gains;                                     // K values
    std::vector<std::vector<Complex>> branches;       // branches[b][k], b = 0..n-1
    std::vector<Complex> openLoopPoles, openLoopZeros;
};
RootLocusData rootLocus(const systems::TransferFunction& L, const Vector& gains);
RootLocusData rootLocus(const systems::TransferFunction& L);           // gains = {0} ∪ logspace(-3, 3, 600)
Vector        asymptoteAngles_deg(const systems::TransferFunction& L);
Real          asymptoteCentroid(const systems::TransferFunction& L);
// Gain and poles at which a branch crosses the imaginary axis (continuous) / unit circle (discrete), if any.
std::vector<std::pair<Real, Complex>> stabilityBoundaryCrossings(const RootLocusData& rl);
}
```

```cpp
RootLocusData rootLocus(const systems::TransferFunction& L, const Vector& gains) {
    RootLocusData rl;
    rl.gains = gains;
    rl.openLoopPoles = L.poles(); rl.openLoopZeros = L.zeros();
    const Index n = L.order();
    rl.branches.assign(n, std::vector<Complex>(gains.size()));
    std::vector<Complex> prev = rl.openLoopPoles;          // K = 0 roots
    for (Index k = 0; k < gains.size(); ++k) {
        std::vector<Complex> cur = (L.den() + gains(k) * L.num()).roots();
        if (static_cast<Index>(cur.size()) != n) throw NumericalError("rootLocus: degree dropped");
        // greedy nearest-neighbour assignment of cur to prev so each branch is continuous
        std::vector<bool> used(n, false);
        for (Index b = 0; b < n; ++b) {
            Index best = -1; Real bestDist = INF;
            for (Index j = 0; j < n; ++j)
                if (!used[j] && std::abs(cur[j] - prev[b]) < bestDist) { bestDist = std::abs(cur[j] - prev[b]); best = j; }
            used[best] = true;
            rl.branches[b][k] = cur[best];
        }
        for (Index b = 0; b < n; ++b) prev[b] = rl.branches[b][k];
    }
    return rl;
}
```

(Degree drops when `deg N = deg D` and `K` cancels the leading coefficient — only for improper-ish loops; the check documents the assumption.)

**How it connects.** `rootLocus` uses `Polynomial` arithmetic (Day 11) and `roots` (Day 12); it is a design tool for Days 26–27 and a teaching bridge to pole placement (Day 38: "what if I could move *all* poles anywhere, not just along the locus?").

**Verify.** `L = 1/(s(s+1)(s+2))`: `asymptoteAngles_deg = {60, 180, 300}` (any order), centroid `−1`; `stabilityBoundaryCrossings` finds `K ≈ 6.0 ± 0.05` at `±1.414j` (refine with bisection on `K` for a sharper test). `L = 1/(s(s+2))`: for `K > 1` the branches are `−1 ± j√(K−1)` — check `K = 5` gives `−1 ± 2j`. Plot both and compare with the sketching rules.

Reading: FPE Chapter 5 (§5.1–5.4); AM does not cover root locus in depth — Nise, *Control Systems Engineering*, Chapter 8 is the clearest treatment.

**Done when.**
- [ ] `rootLocus`, asymptotes, crossings, `plotRootLocus`, tests
- [ ] You sketched the locus of `1/(s(s+1)(s+2))` by the rules before plotting it
- [ ] Commit

---

#### Day 22 — Routh–Hurwitz, the umbrella header, `ex02`, and the Phase 2–3 review

**C++ you learn.**

- **Header hygiene**: every header includes exactly what it uses and nothing more (`<iosfwd>` instead of `<iostream>` when only declaring `operator<<`); no header relies on an include order. Test: each header compiles alone in an empty `.cpp`.
- **Umbrella header** `csl/csl.hpp` that includes everything, for examples and users; the library's own sources include only what they need (faster compiles, clearer dependencies).
- `inline` vs. non-inline free functions: a function *defined* in a header must be `inline`; one *declared* in a header and defined in a `.cpp` must not.
- Handling special cases in a tabular algorithm (zero pivot, all-zero row) with small `constexpr` epsilons and clear comments.

**Control you learn.** Routh–Hurwitz: stability from coefficients, without computing roots.

For `a₀sⁿ + a₁sⁿ⁻¹ + … + aₙ` build the Routh array:

```
sⁿ   : a₀  a₂  a₄ …
sⁿ⁻¹ : a₁  a₃  a₅ …
sⁿ⁻² : b₁  b₂  …      bᵢ = (a₁ a_{2i} − a₀ a_{2i+1}) / a₁
sⁿ⁻³ : c₁  c₂  …      cᵢ = (b₁ a_{2i+1} − a₁ b_{i+1}) / b₁
 …
s⁰   : …
```

The number of sign changes in the first column equals the number of RHP roots. Necessary condition: all coefficients positive (for a stable polynomial). Special cases: a zero in the first column with nonzero row → replace by ε and take limits; an entire zero row → replace by the derivative of the auxiliary polynomial formed from the row above (indicates symmetric roots, e.g. on the `jω` axis).

Why still learn it when `roots()` exists: it answers *parametric* questions in closed form. Closed loop of `1/(s(s+1)(s+2))` with gain `K`: `s³ + 3s² + 2s + K`; third-order rule `a₁a₂ > a₀a₃ ⇒ 3·2 > K ⇒ K < 6`. The same 6 as the gain margin (Day 18) and the locus crossing (Day 21), now derived by hand in one line.

**Files you touch.** `include/csl/analysis/Stability.hpp`, `src/analysis/Stability.cpp`, `include/csl/csl.hpp`, `examples/ex02_representations.cpp`, tests, `sandbox/phase2_3_review.md`.

**Code.**

```cpp
// include/csl/analysis/Stability.hpp  (Day 22 part; Day 40 adds the Lyapunov test)
#pragma once
#include "csl/core/Polynomial.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::analysis {
Matrix routhArray(const Polynomial& p);            // (n+1) x ceil((n+1)/2)
int    routhRHPRootCount(const Polynomial& p);     // sign changes in the first column
bool   routhIsStable(const Polynomial& p);
bool   isMinimumPhase(const systems::LTISystem& sys);   // moved here from Day 20
}
```

```cpp
// include/csl/csl.hpp — umbrella
#pragma once
#include "csl/core/Types.hpp"
#include "csl/core/Exceptions.hpp"
#include "csl/core/MathUtils.hpp"
#include "csl/core/Polynomial.hpp"
#include "csl/systems/LTISystem.hpp"
#include "csl/systems/StateSpace.hpp"
#include "csl/systems/TransferFunction.hpp"
#include "csl/systems/Conversions.hpp"
#include "csl/systems/Interconnect.hpp"
#include "csl/simulation/ODESolver.hpp"
#include "csl/simulation/TimeResponse.hpp"
#include "csl/analysis/FrequencyResponse.hpp"
#include "csl/analysis/RootLocus.hpp"
#include "csl/analysis/Stability.hpp"
#include "csl/signals/Signals.hpp"
#include "csl/plot/Figure.hpp"
#include "csl/plot/Plots.hpp"
#include "csl/plot/CSV.hpp"
// later phases append their headers here
```

`examples/ex02_representations.cpp`: build E2 as `StateSpace`; `ss2tf`; print poles, zeros, dc gain, `damp`; `plotStep`, `plotBode`, `plotNyquist` for `10·P`, `plotRootLocus(P)`; print the Routh-derived stability range of `K` for `feedback(K·P)` and the margins of `10·P`; show `pade` delay reducing PM. One program that touches every module built so far — if it compiles and the numbers agree with each other, the library is cohesive.

**Verify.** `routhIsStable({1, 2, 3, 4})` is **true** (`2·3 = 6 > 4`) and `routhRHPRootCount({1, 2, 3, 10})` is **2** (`6 < 10`) — check against `roots()` (V1's design document had these two the wrong way round; the roots do not lie). `routhRHPRootCount({1, 0, −1})` = 1 (roots ±1). The ε case: `{1, 1, 2, 2, 3}` (zero pivot) gives 2 RHP roots. Each header compiles standalone.

**Phase 2–3 review (write in `sandbox/phase2_3_review.md`).**
1. Explain virtual dispatch: what happens at run time when `report(const LTISystem&)` calls `sys.poles()`?
2. Why does `TransferFunction` normalise the denominator to monic in the constructor? What would break if it did not?
3. Give three ways the library computes "closed loop with gain 6 of `1/(s(s+1)(s+2))` is marginal" and name the day each came from.
4. From Bode asymptotes alone, sketch `10(s+1)/(s(s+10)(s²+0.4s+4))`.
5. What is the difference between the sensitivity `S` and the phase margin as robustness measures?

Reading: FPE §3.6 (Routh), Nise §6.2–6.4. C++: C++ Primer §6.5.2 (`inline`), §2.6.3 again (headers).

**Done when.**
- [ ] Routh with special cases; umbrella header; `ex02` runs and its numbers cross-check
- [ ] Review written
- [ ] Commit `Day 22: Phase 3 complete`

---
### Phase 4 — Classical control design (Days 23–28)

**Phase goal.** A `Controller` hierarchy with a production-quality PID, a closed-loop sampled-data `Simulator` that owns its parts through smart pointers, tuning rules (Ziegler–Nichols, SIMC, relay auto-tune), lead/lag design on the Bode plot, and loop-shaping analysis with the gang of four. In C++ terms: abstract bases in anger, ownership and RAII, `std::unique_ptr`/`std::shared_ptr`, stack vs. heap, tuples and structured bindings, numerical root finding.

---

#### Day 23 — The `Controller` base class and a practical PID

**C++ you learn.**

- A second abstract base, `Controller` (Section 4.3.5), designed *before* its first concrete class — think about what the Simulator needs (`compute`, `reset`, `Ts`, sizes) and nothing else.
- **`protected`**: members accessible to derived classes but not to users. Use sparingly; today `PID` keeps everything private.
- **Struct configs with default member initialisers**: `struct Config { Real Kp = 1.0; Real Ki = 0.0; … };` then `PID::Config cfg; cfg.Kp = 2.0;` — readable construction of objects with many parameters, and new parameters can be added without breaking callers.
- **Stateful objects**: a PID holds an integral and the previous error/measurement; `reset()` clears them. Tests must drive sequences, not single calls.
- `std::clamp(v, lo, hi)` (`<algorithm>`).

**Control you learn.** PID — the controller in 90 % of industrial loops — and why the textbook formula is not what you implement.

Textbook: `u = Kp e + Ki ∫e dt + Kd de/dt`, `e = r − y`.
- **P** reacts to the present error; alone it leaves a steady-state offset on type-0 plants (E1 with `Kp = 2`: `y_ss = KpK/(1+KpK) = 6/7`).
- **I** accumulates past error and drives the steady-state error to zero (adds a pole at 0 to the loop → type 1), at the cost of phase lag (−90° at low frequency) and overshoot.
- **D** anticipates (phase lead, more damping) but amplifies measurement noise — a pure differentiator has gain `Kd ω` → infinite at high frequency.

The implementable form:
1. **Filtered derivative**: `D(s) = Kd s / (T_f s + 1)` with `T_f = Kd/(Kp N)`, `N ≈ 8–20`. Limits the high-frequency gain to `Kp N`.
2. **Derivative on measurement** (`−dy/dt` instead of `de/dt`): avoids the "derivative kick" when the reference steps.
3. **Discrete implementation** at sample time `Ts` (backward Euler on I and D — unconditionally stable):

```
e_k   = r_k − y_k
I_k   = I_{k−1} + Ki·Ts·e_k
D_k   = (T_f·D_{k−1} − Kd·(y_k − y_{k−1})) / (T_f + Ts)       (derivative on measurement)
u_k   = clamp(Kp·e_k + I_k + D_k, u_min, u_max)
```

4. The continuous transfer function of the ideal filtered PID (for Bode/margin analysis with Day 19's `series`):

```
C(s) = [ (Kp T_f + Kd) s² + (Kp + Ki T_f) s + Ki ] / [ T_f s² + s ]
```

Anti-windup is Day 25's topic; today's output clamp is the bare minimum.

**Files you touch.** `include/csl/control/Controller.hpp` (exactly Section 4.3.5), `include/csl/control/PID.hpp`, `src/control/PID.cpp`, tests.

**Code.**

```cpp
// include/csl/control/PID.hpp
#pragma once
#include "csl/control/Controller.hpp"
#include "csl/systems/TransferFunction.hpp"

namespace csl::control {

class PID final : public Controller {
public:
    struct Config {
        Real Kp = 1.0, Ki = 0.0, Kd = 0.0;
        Real N  = 10.0;                 // derivative filter ratio: Tf = Kd / (Kp N)  (Tf = 0 if Kd = 0)
        Real Ts = 0.01;                 // sample time [s]
        Real uMin = -INF, uMax = INF;   // output limits
        bool derivativeOnMeasurement = true;
        Real setpointWeight = 1.0;      // b in  P-term = Kp (b r - y)    (Day 25)
        enum class AntiWindup { None, Clamping, BackCalculation } antiWindup = AntiWindup::Clamping;  // Day 25
        Real trackingTime = -1.0;       // Tt for back-calculation; < 0 => sqrt(Ti Td) or Ti
    };
    explicit PID(const Config& cfg);

    Vector compute(const Vector& measurement, const Vector& reference, Real t) override;  // sizes 1, 1 -> 1
    Real   compute(Real y, Real r);                                                       // scalar convenience
    void   reset() override;
    Real   Ts() const override { return cfg_.Ts; }
    Index  inputs()  const override { return 1; }
    Index  outputs() const override { return 1; }

    const Config& config() const { return cfg_; }
    systems::TransferFunction toTransferFunction() const;   // continuous ideal filtered PID
    Real derivativeTimeConstant() const;                     // Tf

private:
    Config cfg_;
    Real integral_ = 0.0, derivative_ = 0.0;
    Real prevMeasurement_ = 0.0, prevError_ = 0.0;
    bool first_ = true;
};

} // namespace csl::control
```

```cpp
Real PID::compute(Real y, Real r) {
    const Real e  = r - y;
    const Real Tf = derivativeTimeConstant();
    const Real Ts = cfg_.Ts;
    if (first_) { prevMeasurement_ = y; prevError_ = e; first_ = false; }

    const Real p = cfg_.Kp * (cfg_.setpointWeight * r - y);
    const Real dInput = cfg_.derivativeOnMeasurement ? -(y - prevMeasurement_) : (e - prevError_);
    if (cfg_.Kd != 0.0) derivative_ = (Tf * derivative_ + cfg_.Kd * dInput) / (Tf + Ts);

    const Real iCandidate = integral_ + cfg_.Ki * Ts * e;
    const Real uUnsat = p + iCandidate + derivative_;
    const Real u = std::clamp(uUnsat, cfg_.uMin, cfg_.uMax);

    switch (cfg_.antiWindup) {                                   // Day 25 fills these in
        case Config::AntiWindup::None:            integral_ = iCandidate; break;
        case Config::AntiWindup::Clamping:        /* Day 25 */ integral_ = iCandidate; break;
        case Config::AntiWindup::BackCalculation: /* Day 25 */ integral_ = iCandidate; break;
    }
    prevMeasurement_ = y; prevError_ = e;
    return u;
}
```

`toTransferFunction()` builds the polynomials above (with `Tf = 0` and `Kd = 0` reduce to `(Kp s + Ki)/s`).

**How it connects.** `Controller` is what the `Simulator` (tomorrow) drives. `PID::toTransferFunction()` goes into `series(C, P)` → `margins` (Day 18) and `feedback` → `stepInfo` (Day 16): design in the frequency domain, verify in the time domain, both with code you wrote. Tuning rules (Day 26) return `PID::Config`.

**Verify.** `PID` with `Kp = 2` only: `compute(0.5, 1.0) == 1.0`. With `Ki = 1, Ts = 0.1`, `e = 1` for ten calls: output after the tenth is `Kp·1 + 1.0`. `toTransferFunction()` for `Kp = 2, Ki = 1, Kd = 0.5, N = 10` (`Tf = 0.025`) isApprox `{0.55, 2.025, 1}/{0.025, 1, 0}`. Closed loop in the TF world: E1 with PI `Kp = 1, Ki = 1`: `feedback(C·P).dcGain() == 1` (integral action ⇒ zero steady-state error) and `stepInfo` overshoot `≈ 4.5 %`… compute it and record the number; it becomes tomorrow's cross-check against the simulator.

Reading: AM Chapter 11 (PID — all of it, it is excellent and practical); FPE §4.3. C++: C++ Primer §15.2–15.3 again, §7.3.3 (default member initialisers via in-class initialisers).

**Done when.**
- [ ] `Controller` base, `PID` with filtered derivative and output limits, `toTransferFunction`, tests
- [ ] You can explain why derivative-on-measurement and the derivative filter exist
- [ ] Commit

---

#### Day 24 — Ownership: stack vs. heap, `new`/`delete`, RAII, smart pointers — and the closed-loop `Simulator`

**C++ you learn.** The memory model, once and properly. This is the topic you asked about by name ("what is dynamic allocation?"); here is the whole story.

- **Stack**: local variables live in the function's stack frame and die when the function returns. Fast, automatic, size fixed at compile time (`double a[3]`, `Vector v` — the *object* `v` is on the stack, but its *buffer*…).
- **Heap** (free store): memory requested at run time with a size decided at run time. `std::vector`, `std::string` and Eigen's dynamic matrices keep their *data* on the heap and manage it for you. Their handle object is on the stack; when it dies, its destructor frees the heap buffer. That is **RAII** — Resource Acquisition Is Initialization: acquire in the constructor, release in the destructor, so cleanup is automatic and exception-safe.
- Raw dynamic allocation, done once so you recognise it and never write it again:

```cpp
double* buf = new double[1000];   // heap allocation; buf is a pointer (an address)
buf[0] = 1.0;
delete[] buf;                     // you must free it. Forget => leak. Twice => crash. Throw before => leak.
buf = nullptr;
```

Every problem in that snippet is solved by `std::vector<double> buf(1000);`. Rule for this library: **no `new`, no `delete`**, ever.
- **Pointers** are still needed for *polymorphism*: a `Controller` object cannot be stored by value (abstract), so the Simulator must hold a pointer to it. Modern C++ uses **smart pointers**, which are RAII handles to heap objects:
  - `std::unique_ptr<Controller> c = std::make_unique<PID>(cfg);` — sole owner; freed automatically; cannot be copied, only *moved* (`std::move(c)`) — ownership transfer is explicit and visible.
  - `std::shared_ptr<T>` — shared ownership with reference counting; use only when two owners genuinely exist. Default to `unique_ptr`.
  - Pass by `Controller&` or `const Controller&` when you only *use* an object; pass `std::unique_ptr<Controller>` (by value) when you *take ownership*. Return `std::unique_ptr` from factories.
- `nullptr`, `->` to call through a pointer, `if (ptr)` to test for null.

**Control you learn.** What a real control loop looks like in time: a **sampled-data system**.

- The plant is continuous; the controller runs every `Ts` seconds; between samples the actuator holds `u` constant (ZOH). The Simulator reproduces exactly that: at each controller tick, measure `y = Cx` (plus noise from Day 50, through an estimator from Day 39), compute `u`, then integrate the plant over `[t, t + Ts]` with RK4 sub-steps holding `u` fixed. The library's `SimResult` convention (time along rows) stores the sub-step trajectory so plots are smooth.
- **Algebraic loop**: if the plant has `D ≠ 0` and the controller reacts instantly, `y` depends on `u` depends on `y`. Physical plants have `D = 0`; the Simulator requires it and throws otherwise.
- **Sample time** matters: the hold adds an effective delay of `Ts/2`, i.e. phase lag `ω Ts/2` at the crossover frequency (Day 35 quantifies it on margins). First rule of thumb: `Ts ≤ (rise time)/10`, or `ω_s = 2π/Ts ≥ 20 ω_bandwidth`.
- Disturbances enter at the plant input (load disturbance) or the output; references change in steps. The Simulator takes all three as `std::function<Vector(Real)>` so experiments are one-liners.

**Files you touch.** `include/csl/simulation/Simulator.hpp`, `src/simulation/Simulator.cpp`, tests, `examples/ex03_classical_design.cpp` (started).

**Code.**

```cpp
// include/csl/simulation/Simulator.hpp
#pragma once
#include <functional>
#include <memory>
#include "csl/control/Controller.hpp"
#include "csl/estimation/Estimator.hpp"      // Day 39 (forward-declare until then: namespace csl::estimation { class Estimator; })
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/StateSpace.hpp"

namespace csl::simulation {

class Simulator {
public:
    using Signal = std::function<Vector(Real t)>;

    Simulator(systems::StateSpace plant, std::unique_ptr<control::Controller> controller);

    void setReference(Signal r)            { reference_ = std::move(r); }
    void setInputDisturbance(Signal d)     { disturbance_ = std::move(d); }
    void setOutputDisturbance(Signal d)    { outputDisturbance_ = std::move(d); }
    void setPlantSubsteps(int n)           { substeps_ = n; }          // RK4 steps per controller period (default 10)
    void setEstimator(std::unique_ptr<estimation::Estimator> est);     // Day 39
    void setMeasurementNoise(Real stdDev, unsigned seed = 1);           // Day 50

    SimResult run(const Vector& x0, Real tf);

private:
    systems::StateSpace plant_;
    std::unique_ptr<control::Controller> controller_;
    std::unique_ptr<estimation::Estimator> estimator_;
    Signal reference_, disturbance_, outputDisturbance_;
    int substeps_ = 10;
    Real noiseStd_ = 0.0; unsigned seed_ = 1;
};

} // namespace csl::simulation
```

```cpp
SimResult Simulator::run(const Vector& x0, Real tf) {
    if (plant_.D().norm() != 0.0) throw InvalidArgument("Simulator: plant must have D = 0 (algebraic loop)");
    const Real Ts = controller_->Ts();
    if (Ts <= 0) throw InvalidArgument("Simulator: controller Ts must be > 0");
    const Real dt = Ts / substeps_;
    const Index steps = static_cast<Index>(std::round(tf / Ts));
    const Index N = steps * substeps_ + 1;
    const Index n = plant_.order(), m = plant_.inputs(), p = plant_.outputs();

    SimResult r; r.t.resize(N); r.x.resize(N, n); r.y.resize(N, p); r.u.resize(N, m);
    Vector x = x0, u = Vector::Zero(m);
    controller_->reset();
    Index row = 0;
    for (Index k = 0; k < steps; ++k) {
        const Real t = k * Ts;
        Vector y = plant_.C() * x;
        if (outputDisturbance_) y += outputDisturbance_(t);
        // (Day 50: y += noise)   (Day 39: measurement = estimator ? estimate : y)
        const Vector ref = reference_ ? reference_(t) : Vector::Zero(controller_->inputs());
        u = controller_->compute(y, ref, t);
        if (disturbance_) u += disturbance_(t);
        auto f = [&](Real, const Vector& xx) { return plant_.derivative(xx, u); };
        for (int s = 0; s < substeps_; ++s) {
            const Real ts = t + s * dt;
            r.t(row) = ts; r.x.row(row) = x.transpose(); r.y.row(row) = (plant_.C() * x).transpose(); r.u.row(row) = u.transpose();
            ++row;
            x = rk4Step(f, ts, x, dt);
        }
    }
    r.t(row) = steps * Ts; r.x.row(row) = x.transpose(); r.y.row(row) = (plant_.C() * x).transpose(); r.u.row(row) = u.transpose();
    return r;
}
```

Usage:

```cpp
control::PID::Config cfg; cfg.Kp = 2.0; cfg.Ts = 0.001;
simulation::Simulator sim(plants::thermal(), std::make_unique<control::PID>(cfg));
sim.setReference([](Real) { return Vector::Constant(1, 1.0); });
const auto r = sim.run(Vector::Zero(1), 10.0);
```

**How it connects.** Every controller from here on (PID, TF controllers, state feedback, LQR, LQG, MPC) is exercised through this one class. Day 39 adds the estimator hook, Day 50 the noise hook. `SimResult` goes to `stepInfo`, `plot*`, `welch` and `arx` unchanged.

**Verify.** E1 with P-control `Kp = 2`, `Ts = 0.001`: final `y = 0.8571 ± 1e-3`, closed-loop time constant `τ/(1+KpK) = 0.2857` (63 % at that time). With the PI from yesterday (`Kp = 1, Ki = 1`), `stepInfo(sim.run(...))` overshoot matches yesterday's TF-world number within `0.3 %` at `Ts = 0.001` — the discrete simulation and the continuous algebra agree. Raise `Ts` to `0.5`: the overshoot grows visibly (ZOH lag). A `unique_ptr` is moved into the Simulator and the original variable is null afterwards (`EXPECT_EQ(ptr, nullptr)`).

Reading: C++ Primer §12.1 (dynamic memory and smart pointers — read it all), §13.6.1 (rvalue references, lightly); learncpp chapter 19 (dynamic allocation) and 22 (smart pointers). Control: AM §11.5 (implementation), FPE §8.1–8.2 (sampled-data overview).

**Done when.**
- [ ] `Simulator` runs a PID loop; results match the TF-world prediction
- [ ] You wrote and then deleted one `new`/`delete` pair, and can list three things `unique_ptr` fixes
- [ ] Commit

---

#### Day 25 — Practical PID: saturation, integrator windup, back-calculation, setpoint weighting

**C++ you learn.**

- Implementing **strategy via `enum class` + `switch`**; behaviour tests that run *sequences* and compare whole trajectories (`EXPECT_LT(overshoot_with, overshoot_without)`).
- Refactoring a growing member function into named private helpers without changing behaviour (run the tests before and after — this is what tests are for).
- A note on **numerical hygiene**: integrators accumulate; use `Real`, never `float`, and reset state on mode changes.

**Control you learn.** What goes wrong when the actuator saturates, and the standard fixes.

- **Windup**: the actuator is at its limit, the error persists, the integral keeps growing far beyond what the actuator can deliver. When the error finally changes sign the integral must "unwind" first → long overshoot and sluggish recovery.
- **Clamping** (conditional integration): stop integrating when the output is saturated *and* the error has the same sign as the saturation.
- **Back-calculation**: feed `(u_sat − u_unsat)/T_t` back into the integrator so it tracks the saturated output; `T_t` (tracking time) typically `√(Ti·Td)` or `Ti`. Smoother than clamping, one tuning knob.
- **Setpoint weighting** `b`: P-term uses `b·r − y` (`b ∈ [0, 1]`) to reduce overshoot on reference steps without affecting disturbance response (the two responses are decoupled — two-degree-of-freedom control).
- **Bumpless transfer** (manual↔auto): initialise the integrator so the output is continuous; mention only.

**Files you touch.** `PID.cpp` (fill the two anti-windup branches; apply `setpointWeight`), tests, `ex03`.

**Code (the anti-windup branches).**

```cpp
case Config::AntiWindup::Clamping: {
    const bool saturated = (u != uUnsat);
    const bool pushingFurther = (uUnsat > cfg_.uMax && e > 0) || (uUnsat < cfg_.uMin && e < 0);
    integral_ = (saturated && pushingFurther) ? integral_ : iCandidate;
    break;
}
case Config::AntiWindup::BackCalculation: {
    const Real Ti = cfg_.Ki > 0 ? cfg_.Kp / cfg_.Ki : INF;
    const Real Td = cfg_.Kp > 0 ? cfg_.Kd / cfg_.Kp : 0.0;
    const Real Tt = cfg_.trackingTime > 0 ? cfg_.trackingTime : (Td > 0 ? std::sqrt(Ti * Td) : Ti);
    integral_ = iCandidate + (Ts / Tt) * (u - uUnsat);
    break;
}
```

**How it connects.** The Simulator's `setInputDisturbance` and the PID's limits are how you test disturbance rejection under saturation — the scenario that distinguishes a demo controller from a usable one. MPC (Day 63) handles the same constraints *optimally*; this is the baseline it is compared to.

**Verify.** E3 (DC motor speed) with PI `Kp = 100, Ki = 200, Ts = 0.001`, reference step `1 rad/s`, voltage limit `±5 V` (the motor needs `≈ 10 V` for 1 rad/s at steady state → heavy saturation; use a reference of `0.3 rad/s` so steady state is reachable, with saturation during the transient): overshoot with `AntiWindup::None` is at least 2× the overshoot with `Clamping`; `BackCalculation` is between or better; without limits all three coincide. Setpoint weight `b = 0.5` reduces reference-step overshoot but leaves the response to an input disturbance at `t = 2 s` unchanged (compare `y` after `t = 2` to `1e-9`). Record the numbers in the test as bounds.

Reading: AM §11.4 (integrator windup) — the canonical reference; FPE §4.3.3.

**Done when.**
- [ ] Two anti-windup strategies and setpoint weighting with comparative tests
- [ ] You can explain in one sentence each what windup is and how back-calculation prevents it
- [ ] Commit

---

#### Day 26 — Tuning rules: Ziegler–Nichols, SIMC, relay auto-tuning (tuples and structured bindings)

**C++ you learn.**

- **`std::pair`/`std::tuple`** and **structured bindings**: `auto [K, L, T] = fitFOPDT(response);` unpacks a struct or tuple into named variables. Prefer a named struct for return types that have meaning (`FOPDT`); use `pair`/`tuple` only for ad-hoc groupings.
- **Namespaced free functions returning configs** (`tuning::zieglerNicholsUltimate(Ku, Tu) -> PID::Config`).
- A **second concrete `Controller`** (`Relay`) that is trivially simple — the point is that the Simulator runs it with no changes.
- Detecting oscillation peaks in a `SimResult` (reuse Day 16's algorithms).

**Control you learn.** Three ways to get PID gains without a model, and one with.

1. **Ziegler–Nichols step (process reaction) method**: fit a *first-order-plus-dead-time* model `K e^{−Ls}/(Ts + 1)` to the open-loop step response (steepest tangent gives `L` and `T`; `K` from the final value). Then `Kp = 1.2 T/(K L)`, `Ti = 2L`, `Td = 0.5L` for PID. Aggressive (quarter-amplitude decay, ~25 % overshoot).
2. **Ziegler–Nichols ultimate method**: increase P-gain until sustained oscillation: `Ku` and period `Tu`. `Kp = 0.6 Ku`, `Ti = Tu/2`, `Td = Tu/8`. With a model, `Ku` and `Tu` are the gain margin (linear) and `2π/ω_pc` from Day 18 — no experiment needed.
3. **Relay auto-tuning** (Åström–Hägglund): replace the controller by a relay of amplitude `d`; the loop oscillates at `ω_pc` with amplitude `a`; `Ku ≈ 4d/(πa)`, `Tu` = measured period. Safe (amplitude bounded by the relay), automatic, the basis of every "auto-tune" button.
4. **SIMC / IMC (model-based, Skogestad)**: for `K e^{−Ls}/(Ts+1)` and desired closed-loop time constant `τ_c` (default `τ_c = L`): `Kp = T/(K(τ_c + L))`, `Ti = min(T, 4(τ_c + L))`, `Td = 0`. Smooth, robust, one knob. The modern default.

**Files you touch.** `include/csl/control/Tuning.hpp`, `src/control/Tuning.cpp`, `include/csl/control/Relay.hpp` (tiny), tests.

**Code.**

```cpp
// include/csl/control/Tuning.hpp
#pragma once
#include "csl/control/PID.hpp"
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::control::tuning {

struct FOPDT { Real K, L, T; systems::TransferFunction model(int padeOrder = 2) const; };
FOPDT fitFOPDT(const simulation::SimResult& openLoopStep, Index outputIndex = 0, Real stepAmplitude = 1.0);

struct UltimatePoint { Real Ku, Tu; };
UltimatePoint ultimateFromModel(const systems::LTISystem& plant);               // via margins()
UltimatePoint relayExperiment(const systems::StateSpace& plant, Real relayAmplitude,
                              Real Ts, Real tf, Real hysteresis = 0.0);         // via Simulator + Relay

enum class Rule { ZN_Step_PID, ZN_Step_PI, ZN_Ultimate_PID, ZN_Ultimate_PI, SIMC_PI, SIMC_PID };
PID::Config tune(Rule rule, const FOPDT& m, Real Ts, Real tauC = -1.0);        // step-based rules
PID::Config tune(Rule rule, const UltimatePoint& u, Real Ts);                  // ultimate-based rules

} // namespace csl::control::tuning
```

`Relay` implements `Controller` with `compute` returning `+d` if `e > h`, `−d` if `e < −h`, else the previous output. `relayExperiment` runs it, finds the last few peaks of `y` (`std::max_element` on windows, or zero-crossings of `e`), takes `a` = mean peak amplitude, `Tu` = mean peak spacing, `Ku = 4d/(πa)`.

**How it connects.** `fitFOPDT` consumes `step(...)` (Day 15); `ultimateFromModel` is `margins` (Day 18); `relayExperiment` is the `Simulator` with a new `Controller` (Day 24); all return `PID::Config` (Day 23) which the Simulator then runs. Nothing new underneath — the day is pure composition, which is the point.

**Verify.** `ultimateFromModel(1/(s(s+1)(s+2)))` gives `Ku = 6.00 ± 0.02`, `Tu = 4.443 ± 0.01`. `relayExperiment` on the same plant (`d = 1`) gives `Ku` within 15 % of 6 (the describing-function approximation) and `Tu` within 3 %. `fitFOPDT(step(E1))` returns `K = 3`, `L ≈ 0`, `T ≈ 2`; on `3 e^{−0.5 s}/(2s+1)` (E1 with `pade(0.5, 3)`) returns `L = 0.5 ± 0.05`, `T = 2 ± 0.1`. SIMC on that model with `τ_c = L`: `Kp = 2/(3·1) = 0.667`, `Ti = min(2, 4) = 2`; the closed loop (Simulator, `Ts = 0.01`) has overshoot `< 5 %` and settles in `< 6 s`. ZN-ultimate PID on `1/(s(s+1)(s+2))` has overshoot in the 25–60 % range (it is designed to be aggressive).

Reading: AM §11.3 (tuning rules, relay), Skogestad's 2003 SIMC paper (sections 1–4). C++: C++ Primer §17.1 (`tuple`), cppreference "structured binding".

**Done when.**
- [ ] FOPDT fit, ZN (both), SIMC, relay experiment, tests
- [ ] You can explain why relay tuning is safe and what it measures
- [ ] Commit

---

#### Day 27 — Lead and lag compensation on the Bode plot (and a bisection root finder)

**C++ you learn.**

- **Numerical root finding as a library primitive**: `bisection(f, a, b, tol)` taking `std::function<Real(Real)>`. Robust (guaranteed if `f(a)·f(b) < 0`), slow (one bit per iteration), fine for everything in this library. Add `goldenSection` for 1-D minimisation (used by `peakSensitivity` tomorrow). Both in `NumericalMethods.hpp`.
- A *design* function that **searches** (`designLead` iterates on the crossover frequency) and returns a result struct documenting what it did (`phiMax`, `wm`, achieved PM). Good design functions explain their result.

**Control you learn.** Shaping the loop with one zero and one pole.

- **Steady-state error constants**: for unity feedback with loop `L`, position `K_p = L(0)`, velocity `K_v = lim s L(s)`, acceleration `K_a = lim s² L(s)`. System *type* = number of integrators in `L`. Type 1: zero error to a step, error `1/K_v` to a ramp. Specs often say "`K_v ≥ 10`" — that fixes the loop gain at low frequency.
- **Lead compensator** `C(s) = K (s + z)/(s + p)`, `z < p`, `α = p/z > 1`. It adds phase lead with maximum `φ_max = asin((α−1)/(α+1))` at `ω_m = √(zp)`, where its magnitude is `+10 log₁₀ α` dB. Design procedure (Bode):
  1. Set `K` from the error-constant spec.
  2. Measure the PM of `K·P`; the lead must supply `φ = PM_target − PM_current + (5°…12°)` (margin for the crossover shift).
  3. `α = (1 + sin φ)/(1 − sin φ)`.
  4. Find `ω_m` where `|K P(jω)| = −10 log α` dB (bisection on the magnitude!) — that is the new crossover.
  5. `z = ω_m/√α`, `p = ω_m √α`. Verify with `margins(series(C, P))`.
- **Lag compensator** (`z > p`, both well below crossover): raises low-frequency gain by `z/p` (bigger `K_v`) with a small PM penalty (−5°) if placed a decade below `ω_gc`. Lead = transient (PM/bandwidth), lag = steady state (error constants); lead-lag = both.
- Lead is a crude PD; lag is a crude PI. Same phase arguments as Day 23, now on the Bode plot.

**Files you touch.** `include/csl/core/NumericalMethods.hpp`, `src/core/NumericalMethods.cpp`, `include/csl/control/LeadLag.hpp`, `src/control/LeadLag.cpp`, tests.

**Code.**

```cpp
// NumericalMethods.hpp (Day 27 part)
namespace csl {
Real bisection(const std::function<Real(Real)>& f, Real a, Real b, Real tol = 1e-10, int maxIter = 200);
Real goldenSectionMin(const std::function<Real(Real)>& f, Real a, Real b, Real tol = 1e-8);
}
```

```cpp
// LeadLag.hpp
namespace csl::control {
systems::TransferFunction lead(Real z, Real p, Real K = 1.0);      // K (s+z)/(s+p), z < p
systems::TransferFunction lag (Real z, Real p, Real K = 1.0);      // z > p
struct ErrorConstants { Real Kp, Kv, Ka; int systemType; };
ErrorConstants errorConstants(const systems::TransferFunction& L);
struct LeadDesign { systems::TransferFunction C; Real z, p, K, phiMax_deg, wm, achievedPM_deg, achievedGM_dB; };
LeadDesign designLead(const systems::LTISystem& P, Real K, Real targetPM_deg, Real safety_deg = 8.0);
LeadDesign designLag (const systems::LTISystem& P, Real K, Real targetPM_deg, Real decadeBelow = 10.0);
}
```

```cpp
LeadDesign designLead(const systems::LTISystem& P, Real K, Real targetPM, Real safety) {
    auto magdB = [&](Real w) { return dB(std::abs(K * P.evaluate(Complex(0, w))(0, 0))); };
    const Margins m0 = margins(/* K*P as TF */);
    const Real phiNeeded = deg2rad(targetPM - m0.phaseMargin_deg.value_or(0.0) + safety);
    if (phiNeeded <= 0) { /* no lead needed: return C = K */ }
    if (phiNeeded >= deg2rad(70)) throw InvalidArgument("designLead: more than 70 deg needed; use two leads");
    const Real alpha = (1 + std::sin(phiNeeded)) / (1 - std::sin(phiNeeded));
    const Real target = -10.0 * std::log10(alpha);
    const Real wm = bisection([&](Real w) { return magdB(w) - target; }, 1e-3, 1e3);   // assumes monotone |KP|
    LeadDesign d; d.z = wm / std::sqrt(alpha); d.p = wm * std::sqrt(alpha); d.K = K; d.wm = wm;
    d.phiMax_deg = rad2deg(std::asin((alpha - 1) / (alpha + 1)));
    d.C = lead(d.z, d.p, K * d.p / d.z);     // keep dc gain = K  (K (s+z)/(s+p) * p/z has dc gain K)
    const Margins m1 = margins(series(d.C, /*P as TF*/));
    d.achievedPM_deg = m1.phaseMargin_deg.value_or(NAN); d.achievedGM_dB = m1.gainMargin_dB.value_or(INF);
    return d;
}
```

(Note the dc-gain convention: the compensator is scaled so the low-frequency gain stays `K` and the error constant is preserved; the magnitude at `ω_m` is then `+10 log α` *relative to K* — consistent with the search above.)

**How it connects.** `bisection` is reused by `peakSensitivity` (Day 28), `butterworthOrder` (Day 34), the H∞ norm bisection (Day 61) and γ-iteration (Day 66). `lead`/`lag` are `TransferFunction`s, so they run in the Simulator via Day 30's `TransferFunctionController` and are analysed with Days 17–19's tools.

**Verify.** `lead(1, 10)` has `φ_max = 54.9°` at `ω_m = 3.162` (`α = 10`). `errorConstants(10/(s(s+1)))`: type 1, `K_v = 10`. `designLead(1/(s(s+1)), K = 10, targetPM = 45°)`: achieved PM in `[45°, 52°]` (uncompensated PM was `18°`), `ω_gc` increased from `3.08` to about `4.5–5`. `designLag(1/(s(s+1)(s+2))… )` — FPE example: with `K = 5` (`K_v = 2.5`) uncompensated PM ≈ `13°`, lag to reach 45°: achieved `≥ 44°`.

Reading: FPE §6.7 (compensation — the lead/lag procedure in full), Nise §11.1–11.5. C++: nothing new.

**Done when.**
- [ ] `bisection`, `goldenSectionMin`, lead/lag design with tests
- [ ] You can draw a lead's Bode plot from `z`, `p` and state where its phase peaks
- [ ] Commit

---

#### Day 28 — Loop shaping, the gang of four, bandwidth, peak sensitivity; `ex03`; Phase 4 review

**C++ you learn.**

- **Composition over new mechanism**: today adds no language feature; it adds functions that combine Days 17–27. Notice how little code each needs.
- A `const`-correctness and **API review** pass over `control/` and `analysis/`: every accessor `const`, every input `const&`, no copies of `StateSpace` where a reference would do.
- Extending `Plots.hpp` with `plotGangOfFour` (four Bode magnitude plots on one page).

**Control you learn.** Judging a loop by its sensitivity functions.

- `S = 1/(1+L)`: output-disturbance → output, and reference → error. Want `|S| ≪ 1` at low frequency (good tracking/rejection). `T = L/(1+L) = 1 − S`: reference → output, *and measurement noise → output*. Want `|T| ≪ 1` at high frequency. The two are in tension because `S + T = 1`: the crossover region is where neither is small — that is why the loop's shape *at crossover* (slope ≈ −20 dB/dec, adequate PM) decides everything.
- **Bandwidth** `ω_b`: where `|T|` drops to −3 dB (or where `|S|` first reaches −3 dB; both are in use). Roughly `ω_gc`.
- **Peak sensitivity** `M_s = max_ω |S(jω)|` = 1 / (shortest distance from the Nyquist curve to −1). A single robustness number: `M_s ≤ 2` guarantees `GM ≥ 2` (6 dB) and `PM ≥ 29°`; `M_s ≤ 1.4` is conservative. Prefer it to GM/PM when reporting.
- **Bode's sensitivity integral** (the "waterbed"): for a stable `L` with relative degree ≥ 2, `∫₀^∞ ln|S(jω)| dω = π Σ Re(p_k)` over unstable open-loop poles (= 0 if `L` is stable). Push `|S|` down somewhere and it pops up elsewhere. Compute it numerically and watch it hold.
- **Loop-shaping rules**: `|L| ≫ 1` below `ω_b` with slope −20 dB/dec through crossover; `|L| ≪ 1` above, roll off fast; avoid RHP zeros near `ω_b` (Day 20); delay limits `ω_b ≲ 1/T_delay`.

**Files you touch.** `include/csl/analysis/FrequencyResponse.hpp/.cpp` (`sensitivityPeak`, `bandwidth`, `sensitivityIntegral`), `Plots.hpp` (`plotGangOfFour`), `examples/ex03_classical_design.cpp`, `sandbox/phase4_review.md`.

**Code.**

```cpp
namespace csl::analysis {
struct SensitivityPeak { Real Ms, w; Real impliedGM_dB, impliedPM_deg; };
SensitivityPeak sensitivityPeak(const systems::TransferFunction& L);     // max |S| via dense grid + golden refine
Real bandwidth(const systems::TransferFunction& T, Real dropDB = -3.0);  // first w where |T| falls below T(0)+dropDB
Real sensitivityIntegral(const systems::TransferFunction& L, Real wMax = 1e4, Index n = 20000);  // ∫ ln|S| dw (trapezoid, log grid)
}
```

`impliedGM = Ms/(Ms−1)`, `impliedPM = 2 asin(1/(2Ms))`.

`ex03_classical_design.cpp`: E3 DC motor speed loop. (1) FOPDT fit from `step`; (2) SIMC PI vs. ZN PID; (3) Simulator runs with voltage saturation ±12 V and anti-windup; (4) `margins`, `sensitivityPeak`, `plotGangOfFour` for both designs; (5) a lead design on `1/(s(s+1))` to a PM spec, root locus before/after; (6) print a comparison table. The program is the deliverable of the phase.

**Verify.** `sensitivityPeak(1/(s(s+1)(s+2)))`: `Ms ≈ 1.57` at `ω ≈ 0.73` (hand check: at `ω = 0.75`, `L = −0.421 − 0.269j`, so `|1 + L| = 0.638`); implied `GM = 8.8 dB`, `PM = 37.2°` — both *below* the actual `15.6 dB`/`53.4°`, as guarantees must be. `S + T` isApprox `1`. `sensitivityIntegral` for the same `L` is `0 ± 0.02` (relative degree 3, stable); for `L = 1/((s−1)(s+3))` with a stabilising `K = 5` it is `π·1 ± 0.05`. `bandwidth(feedback(10/(s(s+1))))` = `4.83 rad/s ± 0.05` (second-order formula `ωn√(1 − 2ζ² + √(4ζ⁴ − 4ζ² + 2))` with `ωn = 3.162`, `ζ = 0.158`; above `ω_gc = 3.08`, as usual for low-PM loops).

**Phase 4 review (write in `sandbox/phase4_review.md`).**
1. Draw the memory layout of a `Simulator` object: what is on the stack, what on the heap, who frees what and when.
2. Why does `Simulator` take `std::unique_ptr<Controller>` by value and `StateSpace` by value, but `margins` takes `const LTISystem&`?
3. Explain windup to a colleague with a picture of `u_unsat`, `u_sat` and `I` over time.
4. Given `M_s = 1.6`, what GM and PM are guaranteed? Why can the actual margins be larger?
5. A loop needs `K_v ≥ 20` and `PM ≥ 50°`. Which compensator(s) and in which order do you design?

Reading: AM §11.1–11.2, Chapter 12 (robust performance, skim), §12.4 (waterbed); FPE §6.7.5–6.8. C++: none.

**Done when.**
- [ ] Gang of four, `Ms`, bandwidth, waterbed integral with tests; `ex03` runs end to end
- [ ] Review written
- [ ] Commit `Day 28: Phase 4 complete`

---
### Phase 5 — Discrete time and digital signal processing (Days 29–36)

**Phase goal.** Sampling and discretization (`c2d`/`d2c`, ZOH, Tustin, Euler, matched pole–zero), the z-domain, a hand-written FFT, spectral estimation (periodogram, Welch), FIR/IIR filter structures and Butterworth/notch/windowed-FIR design, and digital control (emulation vs. direct design, dead-beat). In C++ terms: `enum class`/`switch`, Eigen's matrix functions, bitwise arithmetic, in-place algorithms, function templates with Eigen expressions.

---

#### Day 29 — Sampling and zero-order-hold discretization (`enum class`, matrix exponential)

**C++ you learn.**

- **`enum class`** in full: a scoped, strongly typed enumeration — no implicit conversion to `int`, no name pollution, and a `switch` over it with no `default:` makes the compiler warn (`/W4` → C4062) when you forget a case. That is the idiom for "a fixed set of methods".
- `#include <unsupported/Eigen/MatrixFunctions>` gives `M.exp()` and `M.log()` (Padé-scaling-squaring matrix exponential — Ground Rule 1: we use it, we do not write it).
- A `std::vector<PoleInfo>` function (`damp`) gaining a discrete branch — extending existing code without breaking its callers.

**Control you learn.** What the computer sees and does: sampling, hold, and the exact discrete model.

- The controller sees `y(kT_s)` and emits `u(kT_s)`, held constant until the next tick (**zero-order hold**, ZOH). Over one period the continuous plant integrates exactly:

```
x[k+1] = A_d x[k] + B_d u[k],   A_d = e^{A T_s},   B_d = ∫₀^{T_s} e^{Aτ} dτ · B
```

Both come from **one** matrix exponential of the augmented matrix `M = [[A, B],[0, 0]]·T_s`: `e^M = [[A_d, B_d],[0, I]]`. `C_d = C`, `D_d = D`. This is *exact* for piecewise-constant inputs — no approximation at all. Day 15's `lsim` with RK4 was approximating this; now you can compare.
- **Pole mapping**: continuous pole `s` ↦ discrete pole `z = e^{sT_s}`. Left half-plane ↦ inside the unit circle; `s = 0 ↦ z = 1`; `s = ±jπ/T_s ↦ z = −1`. Discrete stability: `|z| < 1`. `damp` for discrete systems maps back with `s = ln(z)/T_s`.
- **Aliasing**: `e^{j(ω + 2π/T_s)T_s} = e^{jωT_s}` — frequencies that differ by the sampling rate are indistinguishable after sampling. The **Nyquist frequency** `π/T_s` (= `f_s/2`) is the highest representable; anything above folds down. Anti-aliasing filters (Day 34) must remove it *before* sampling.
- The ZOH itself has frequency response `(1 − e^{−jωT_s})/(jωT_s)` ≈ a delay of `T_s/2` — phase loss `ωT_s/2` at crossover (Day 35 measures it). Sample-time rule: `ω_s ≥ 20–40 × ω_b`.
- `d2c` (ZOH): `A = ln(A_d)/T_s` (matrix logarithm; valid when `A_d` has no negative real eigenvalues), `B = (A_d − I)⁻¹ A B_d`… more robustly, take the log of the augmented `[[A_d, B_d],[0, I]]` and read off `A` and `B` divided by `T_s`.

**Files you touch.** `include/csl/systems/Conversions.hpp` + `.cpp` (`DiscretizationMethod`, `c2d`, `d2c`), `LTISystem.cpp` (`damp` discrete), `TimeResponse.cpp` (already handles `Ts > 0`; now tested), tests.

**Code.**

```cpp
// Conversions.hpp additions
namespace csl::systems {
enum class DiscretizationMethod { ZOH, Tustin, ForwardEuler, BackwardEuler, MatchedPoleZero };

StateSpace       c2d(const StateSpace& sys, Real Ts, DiscretizationMethod m = DiscretizationMethod::ZOH, Real prewarp_w = 0.0);
TransferFunction c2d(const TransferFunction& H, Real Ts, DiscretizationMethod m = DiscretizationMethod::Tustin, Real prewarp_w = 0.0);
StateSpace       d2c(const StateSpace& sys, DiscretizationMethod m = DiscretizationMethod::ZOH);
TransferFunction d2c(const TransferFunction& H, DiscretizationMethod m = DiscretizationMethod::Tustin);
}
```

```cpp
#include <unsupported/Eigen/MatrixFunctions>

StateSpace c2d(const StateSpace& sys, Real Ts, DiscretizationMethod m, Real prewarp_w) {
    if (!sys.isContinuous()) throw InvalidArgument("c2d: system is already discrete");
    if (Ts <= 0) throw InvalidArgument("c2d: Ts must be > 0");
    const Index n = sys.order(), nu = sys.inputs();
    switch (m) {
        case DiscretizationMethod::ZOH: {
            Matrix M = Matrix::Zero(n + nu, n + nu);
            M.topLeftCorner(n, n)   = sys.A() * Ts;
            M.topRightCorner(n, nu) = sys.B() * Ts;
            const Matrix E = M.exp();
            return StateSpace(E.topLeftCorner(n, n), E.topRightCorner(n, nu), sys.C(), sys.D(), Ts);
        }
        case DiscretizationMethod::ForwardEuler:   // s = (z-1)/Ts
            return StateSpace(Matrix::Identity(n, n) + Ts * sys.A(), Ts * sys.B(), sys.C(), sys.D(), Ts);
        case DiscretizationMethod::BackwardEuler: { // s = (z-1)/(Ts z)
            const Matrix Ainv = (Matrix::Identity(n, n) - Ts * sys.A()).partialPivLu().inverse();
            return StateSpace(Ainv, Ainv * sys.B() * Ts, sys.C() * Ainv, sys.D() + sys.C() * Ainv * sys.B() * Ts, Ts);
        }
        case DiscretizationMethod::Tustin: {        // Day 30
            const Real a = (prewarp_w > 0) ? prewarp_w / std::tan(prewarp_w * Ts / 2) : 2.0 / Ts;   // s = a (z-1)/(z+1)
            const Matrix I = Matrix::Identity(n, n);
            const Matrix Minv = (a * I - sys.A()).partialPivLu().inverse();
            const Matrix Ad = Minv * (a * I + sys.A());
            const Matrix Bd = std::sqrt(2 * a) * Minv * sys.B();
            const Matrix Cd = std::sqrt(2 * a) * sys.C() * Minv;
            const Matrix Dd = sys.D() + sys.C() * Minv * sys.B();
            return StateSpace(Ad, Bd, Cd, Dd, Ts);
        }
        case DiscretizationMethod::MatchedPoleZero:  // Day 30, via the TF route
            return tf2ss(c2d(ss2tf(sys), Ts, m, prewarp_w));
    }
    throw InvalidArgument("c2d: unknown method");
}
```

(The Tustin state-space form is MATLAB's balanced scaling; its transfer function equals the polynomial substitution of Day 30 exactly — the test tomorrow proves it.)

**How it connects.** `c2d` produces the discrete `StateSpace` that the Kalman filter (Day 51), discrete LQR (Day 45), the Luenberger observer (Day 39), MPC (Day 63) and `TransferFunctionController` (Day 30) all consume. `lsim`/`step` already accept `Ts > 0` (Day 15), so discrete simulations work immediately.

**Verify.** E1, `T_s = 0.1`: `A_d = e^{−0.05} = 0.951229`, `B_d = 3(1 − e^{−0.05}) = 0.146313`, both to `1e-9`. `step(c2d(E1, 0.1)).y` equals `step(E1)` *at the sample instants* to `1e-10` (ZOH is exact for a step). `c2d(E2, 0.1).poles()` = `e^{(−0.25 ± 1.984j)·0.1}`: modulus `0.97531`, angle `±0.19843 rad`. `d2c(c2d(E3, 0.05))` recovers `A`, `B` to `1e-8`. `damp(c2d(E2, 0.1))` reports `ωn = 2`, `ζ = 0.125`. A discrete system with a pole at `z = 1.05` is `!isStable()`. Aliasing demo in the example: sample `sin(2π·450 t)` at 500 Hz and plot — it looks like 50 Hz.

Reading: FPE §8.1–8.3 (sampled-data systems); Åström & Wittenmark, *Computer-Controlled Systems* §2.1–2.4 if you have it (the definitive source). C++: Eigen "MatrixFunctions module" page; C++ Primer §19.3 (enumerations).

**Done when.**
- [ ] `c2d` (ZOH, both Eulers, Tustin skeleton), `d2c` (ZOH), discrete `damp`, tests
- [ ] You can explain the augmented-matrix exponential trick and why ZOH is exact
- [ ] Commit

---

#### Day 30 — The z-transform, Tustin and matched pole–zero on transfer functions, and running any transfer function as a digital controller

**C++ you learn.**

- **Polynomial composition**: substituting `s = a(z−1)/(z+1)` into `N(s)/D(s)` and clearing denominators. Implemented with `Polynomial` products and a `pow(p, k)` helper — the Day 11 operators earning their keep.
- A **generic `Controller` adapter** (`TransferFunctionController`) that turns *any* discrete SISO transfer function into something the `Simulator` can run: the difference equation evaluated with two history buffers.
- `std::deque`? No — two small `Vector`s shifted with `.head()/.tail()` assignments are clearer here. (Day 33's `IIRFilter` introduces the better DF2T structure.)

**Control you learn.** The z-transform and the family of `s → z` maps.

- `Z{x[k−1]} = z⁻¹ X(z)`: delay by one sample is multiplication by `z⁻¹`, as a derivative was multiplication by `s`. A discrete transfer function `H(z) = (b₀zⁿ + … + bₙ)/(zⁿ + a₁zⁿ⁻¹ + … + aₙ)` *is* the difference equation

```
y[k] = b₀u[k] + b₁u[k−1] + … + bₙu[k−n] − a₁y[k−1] − … − aₙy[k−n]
```

That line is what a digital controller executes every `T_s`.
- **Maps** from continuous designs to discrete:

| Method | Substitution | Stability preserved? | Notes |
|---|---|---|---|
| Forward Euler | `s = (z−1)/T_s` | No (can destabilise) | simplest; avoid |
| Backward Euler | `s = (z−1)/(zT_s)` | Yes (adds damping) | PID integrators |
| Tustin (bilinear) | `s = (2/T_s)(z−1)/(z+1)` | Yes, exactly (`jω` axis ↦ unit circle) | frequency **warping** `ω_d = (2/T_s) atan(ω_c T_s/2)`; **prewarp** at `ω₀` by using `a = ω₀/tan(ω₀T_s/2)` so the response is exact there |
| Matched pole–zero | `z = e^{sT_s}` on each pole and zero; add zeros at `z = −1` to make relative degree 0 or 1; match dc gain | Yes | good for controllers |
| ZOH | exact for held inputs | Yes | plants, not controllers |

- Tustin is the default for controllers (it preserves the frequency response shape best); ZOH is the default for plants (it *is* what happens physically).

**Files you touch.** `Conversions.cpp` (TF `c2d` for Tustin/Euler/MPZ, TF `d2c` Tustin), `Polynomial.hpp` (`pow`), `include/csl/control/TransferFunctionController.hpp` + `.cpp`, tests.

**Code.**

```cpp
// Tustin on polynomials: p(s) with s = a (z-1)/(z+1), multiplied through by (z+1)^n
static Polynomial bilinearSubstitute(const Polynomial& p, Real a, Index n) {   // n = max degree in the TF
    const Polynomial zm1{1.0, -1.0}, zp1{1.0, 1.0};
    Polynomial out;                                                            // zero
    const Index d = p.degree();
    for (Index i = 0; i <= d; ++i) {                                           // term p[i] s^(d-i)
        const Index k = d - i;
        out = out + p[i] * std::pow(a, k) * pow(zm1, k) * pow(zp1, n - k);
    }
    return out;
}

TransferFunction c2d(const TransferFunction& H, Real Ts, DiscretizationMethod m, Real prewarp_w) {
    switch (m) {
        case DiscretizationMethod::Tustin: {
            const Real a = (prewarp_w > 0) ? prewarp_w / std::tan(prewarp_w * Ts / 2) : 2.0 / Ts;
            const Index n = std::max(H.num().degree(), H.den().degree());
            return TransferFunction(bilinearSubstitute(H.num(), a, n), bilinearSubstitute(H.den(), a, n), Ts);
        }
        case DiscretizationMethod::MatchedPoleZero: {
            auto mapRoots = [&](const std::vector<Complex>& r) { std::vector<Complex> out; for (auto& c : r) out.push_back(std::exp(c * Ts)); return out; };
            std::vector<Complex> z = mapRoots(H.zeros()), p = mapRoots(H.poles());
            while (z.size() + 1 < p.size()) z.push_back(Complex(-1.0, 0.0));    // zeros at infinity -> z = -1 (keep rel. degree 1)
            TransferFunction G = TransferFunction::fromZPK(z, p, 1.0, Ts);
            const Real k = H.dcGain() / G.dcGain();                             // match dc gain (if finite; else match at a chosen w)
            return TransferFunction::fromZPK(z, p, k, Ts);
        }
        case DiscretizationMethod::ZOH: case DiscretizationMethod::ForwardEuler: case DiscretizationMethod::BackwardEuler:
            return ss2tf(c2d(tf2ss(H), Ts, m, prewarp_w));
    }
    throw InvalidArgument("c2d(TF): unknown method");
}
```

```cpp
// include/csl/control/TransferFunctionController.hpp
class TransferFunctionController final : public Controller {
public:
    explicit TransferFunctionController(systems::TransferFunction discreteH);   // Ts > 0, proper, SISO; input is e = r - y
    Vector compute(const Vector& y, const Vector& r, Real t) override;         // y[k] from e[k] via the difference equation
    void reset() override;
    Real Ts() const override { return H_.Ts(); }
    Index inputs() const override { return 1; }  Index outputs() const override { return 1; }
private:
    systems::TransferFunction H_;
    Vector b_, a_;          // a_(0) == 1 after normalisation; both length n+1 (num zero-padded in front)
    Vector uHist_, yHist_;  // last n inputs/outputs
};
```

**How it connects.** `TransferFunctionController` lets Days 27–28's lead/lag designs run in the Simulator, and lets Day 23's PID be checked against its own `c2d(toTransferFunction())`. `bilinearSubstitute` is reused by Butterworth design (Day 34). The emulation-vs-direct comparison of Day 35 is now possible.

**Verify.** `c2d({1},{1,1}, 0.1, Tustin)` isApprox `{0.047619, 0.047619}/{1, −0.904762}` and `dcGain() == 1`. `ss2tf(c2d(tf2ss(H), Ts, Tustin))` isApprox `c2d(H, Ts, Tustin)` for `H = E2` (state-space and polynomial Tustin agree). With prewarp at `ω₀ = 10` and `T_s = 0.1`: `|c2d(E2, 0.1, Tustin, 10)` evaluated at `z = e^{j·10·0.1}` equals `|E2(j10)|` to `1e-9`; without prewarp it differs by a few percent. MPZ of `1/(s+1)` at `T_s = 0.1`: pole `0.904837`, one zero at `−1`, dc gain 1. `TransferFunctionController` running `H(z) = T_s/(z − 1)` (an integrator) on `e = 1` outputs `(k−1)T_s` at step `k`. Forward Euler of E2 at `T_s = 0.5` is unstable while backward Euler and Tustin are stable — print the pole moduli.

Reading: FPE §8.3–8.4 (z-transform, discretization), Åström & Wittenmark §2.5–2.8 and §8.2 (emulation).

**Done when.**
- [ ] All five methods for SS and TF; `TransferFunctionController`; tests
- [ ] You can write the difference equation of `(z + 0.5)/(z² − 1.5z + 0.7)` from memory of the rule
- [ ] Commit

---

#### Day 31 — The DFT and the FFT (bitwise arithmetic, in-place algorithms)

**C++ you learn.**

- **Bitwise operators**: `&`, `|`, `^`, `~`, `<<`, `>>` on unsigned integers. `n & (n − 1) == 0` tests for a power of two (clears the lowest set bit). Bit reversal of an index by shifting bits out of one word into another. Use `std::size_t` or `Index` consistently; never mix signed and unsigned in a comparison without a cast (`/W4` warns, and it should).
- **In-place algorithms**: `void fft(ComplexVector& x)` modifies its argument (non-const reference) — the only place in the library where a function takes a non-const reference to data for performance; document it loudly.
- `std::swap`, `std::polar`, and a naive `dft` kept purely as the test oracle.
- **Measuring time** with `std::chrono::steady_clock` to *see* `O(N²)` vs `O(N log N)`.

**DSP you learn.** The discrete Fourier transform and why the FFT changed engineering.

- **DFT**: `X[k] = Σ_{n=0}^{N−1} x[n] e^{−j2πkn/N}`, `k = 0…N−1`. Bin `k` corresponds to frequency `f_k = k·f_s/N` (Hz) or `ω_k = 2πk/(N T_s)`. For real `x`, `X[N−k] = conj(X[k])` — bins above `N/2` are the negative frequencies; the one-sided spectrum is `k = 0…N/2`. Frequency **resolution** is `Δf = f_s/N`: to resolve two tones 1 Hz apart you need at least 1 s of data, however fast you sample. Zero-padding interpolates the spectrum; it does not add resolution.
- Inverse: `x[n] = (1/N) Σ X[k] e^{+j2πkn/N}` — same computation with conjugated twiddles and a `1/N`. Parseval: `Σ|x|² = (1/N) Σ|X|²`.
- **Cooley–Tukey radix-2**: split `x` into even and odd samples; `X[k] = E[k] + W_N^k O[k]`, `X[k+N/2] = E[k] − W_N^k O[k]` with `W_N = e^{−j2π/N}`. Recursing gives `O(N log N)`. The **iterative** form: permute the input into bit-reversed order, then do `log₂N` stages of "butterflies" of growing span `2, 4, 8, …, N`.
- A real-input signal of length `N` needs only `N/2 + 1` complex outputs; `rfft` returns those.

**Files you touch.** `include/csl/dsp/FFT.hpp`, `src/dsp/FFT.cpp`, tests, a timing snippet in `ex04` (started).

**Code.**

```cpp
// include/csl/dsp/FFT.hpp
#pragma once
#include "csl/core/Types.hpp"
namespace csl::dsp {
bool  isPowerOfTwo(Index n);
Index nextPowerOfTwo(Index n);
Index bitReverse(Index i, int bits);

void fft (ComplexVector& x);        // in place; x.size() must be a power of two
void ifft(ComplexVector& x);        // in place, includes the 1/N
ComplexVector fft (const Vector& x, Index nfft = -1);   // real input, zero-padded to nfft (default: next power of two)
ComplexVector rfft(const Vector& x, Index nfft = -1);   // the N/2+1 non-negative-frequency bins
Vector fftFrequencies(Index nfft, Real fs_hz);          // Hz, bins 0..nfft-1 with negatives folded (like numpy.fft.fftfreq)
Vector rfftFrequencies(Index nfft, Real fs_hz);         // Hz, bins 0..nfft/2
ComplexVector dft(const Vector& x);                     // O(N^2) reference, tests only
}
```

```cpp
void fft(ComplexVector& x) {
    const Index N = x.size();
    if (!isPowerOfTwo(N)) throw InvalidArgument("fft: length must be a power of two (use fft(const Vector&) to pad)");
    int bits = 0; while ((Index(1) << bits) < N) ++bits;
    for (Index i = 0; i < N; ++i) {                       // bit-reversal permutation
        const Index j = bitReverse(i, bits);
        if (j > i) std::swap(x(i), x(j));
    }
    for (Index len = 2; len <= N; len <<= 1) {            // butterfly stages: span 2, 4, 8, ..., N
        const Complex wLen = std::polar(1.0, -TWO_PI / static_cast<Real>(len));
        for (Index start = 0; start < N; start += len) {  // each block of this span
            Complex w = 1.0;
            for (Index k = 0; k < len / 2; ++k) {         // butterflies inside the block
                const Complex u = x(start + k);
                const Complex v = x(start + k + len / 2) * w;
                x(start + k)           = u + v;
                x(start + k + len / 2) = u - v;
                w *= wLen;
            }
        }
    }
}

Index bitReverse(Index i, int bits) {
    Index r = 0;
    for (int b = 0; b < bits; ++b) { r = (r << 1) | (i & 1); i >>= 1; }
    return r;
}

void ifft(ComplexVector& x) {                            // conj, fft, conj, scale
    x = x.conjugate(); fft(x); x = x.conjugate() / static_cast<Real>(x.size());
}
```

**How it connects.** `fft` is the engine of `periodogram`/`welch` (Day 32), `csd`/`etfe` (Day 58), and fast convolution for long FIR filters (optional, Day 33). It is the first place frequency-domain information comes from *data* rather than from a model — the mirror image of `frequencyResponse` (Day 17).

**Verify.** `fft` of the length-8 impulse `[1,0,…,0]` is all ones; of `[1,1,…,1]` is `[8,0,…,0]`. `bitReverse(0b0011, 4) == 0b1100`. `fft(x)` vs `dft(x)` on a random length-1024 vector: max abs difference `< 1e-9`. `ifft(fft(x)) == x` to `1e-12`. `x = cos(2π·5·t)`, `f_s = 64`, `N = 64`: `|X[5]| = |X[59]| = 32`, all other bins `< 1e-9`. Parseval to `1e-9`. Timing with `N = 2¹⁶`: `fft` in a few ms, `dft` in seconds (Release build) — write the two numbers in the test's output.

Reading: Oppenheim & Schafer, *Discrete-Time Signal Processing* §8.1–8.6 (DFT), §9.1–9.3 (FFT); or Smith, *The Scientist and Engineer's Guide to DSP* chapters 8 and 12 (free online, very readable). C++: C++ Primer §4.8 (bitwise operators), §17.2 (`bitset`, optional).

**Done when.**
- [ ] `fft`, `ifft`, `rfft`, frequencies, `dft` oracle, tests, timing printed
- [ ] You can draw the 8-point butterfly diagram and explain the bit-reversal step
- [ ] Commit

---

#### Day 32 — Windows, leakage, periodogram and Welch's PSD

**C++ you learn.**

- `enum class Window` + a `window(Window, N)` factory returning a `Vector`; `Vector::segment(start, len)`, `.cwiseProduct`, `.cwiseAbs2()`, `.squaredNorm()`.
- Loops over overlapping segments with careful index arithmetic (`step = nperseg·(1 − overlap)`), and a `struct Spectrum { Vector f_hz; Vector P; }` result.
- Numerical **unit discipline** in a function's contract: say in the doc comment that `P` is in `units²/Hz`, one-sided, so a user can integrate it back to a variance.

**DSP you learn.** Estimating a spectrum honestly.

- **Leakage**: a sinusoid that does not complete an integer number of cycles in the window "leaks" into neighbouring bins (the rectangular window's `sinc` sidelobes, −13 dB). **Windows** taper the ends: Hann (sidelobes −31 dB, main lobe 2× wider), Hamming (−43 dB), Blackman (−58 dB, 3× wider). Trade-off: sidelobe suppression vs. frequency resolution.
- **Periodogram** `P[k] = |X[k]|² / (f_s · Σ w²)`, one-sided (×2 for `0 < k < N/2`). It is an *inconsistent* estimator: its variance does not shrink as `N` grows.
- **Welch's method**: split the signal into overlapping segments (50 % overlap, Hann), window each, average their periodograms. Variance drops by the number of segments; resolution is set by the segment length. This is the default PSD estimator everywhere (`scipy.signal.welch`, MATLAB `pwelch`).
- Sanity relations: `∫ P df = variance` (Parseval); white noise with variance `σ²` has one-sided `P = 2σ²/f_s`; a sinusoid of amplitude `A` concentrates power `A²/2` in one bin.
- **Cross-spectral density** `P_xy` (same machinery with `conj(X)·Y`) — needed by the empirical transfer function on Day 58; write it today while the code is fresh.

**Files you touch.** `include/csl/dsp/Spectral.hpp`, `src/dsp/Spectral.cpp`, `Plots.hpp` (`plotSpectrum`, log y), tests.

**Code.**

```cpp
// include/csl/dsp/Spectral.hpp (Day 32 part)
#pragma once
#include "csl/core/Types.hpp"
namespace csl::dsp {
enum class Window { Rectangular, Hann, Hamming, Blackman };
Vector window(Window w, Index N);

struct Spectrum { Vector f_hz; Vector P; };                       // one-sided PSD, units^2/Hz
struct CrossSpectrum { Vector f_hz; ComplexVector Pxy; Vector Pxx, Pyy; };

Spectrum periodogram(const Vector& x, Real fs_hz, Window w = Window::Rectangular, Index nfft = -1);
Spectrum welch(const Vector& x, Real fs_hz, Index nperseg = 256, Real overlap = 0.5, Window w = Window::Hann);
CrossSpectrum csd(const Vector& x, const Vector& y, Real fs_hz, Index nperseg = 256, Real overlap = 0.5, Window w = Window::Hann);
Real integratePSD(const Spectrum& s);                             // trapezoid over f -> variance
}
```

```cpp
Spectrum welch(const Vector& x, Real fs, Index nperseg, Real overlap, Window wt) {
    if (nperseg > x.size()) nperseg = x.size();
    const Index nfft = nextPowerOfTwo(nperseg);
    const Vector w = window(wt, nperseg);
    const Real scale = 1.0 / (fs * w.squaredNorm());
    const Index step = std::max<Index>(1, static_cast<Index>(nperseg * (1.0 - overlap)));
    Vector acc = Vector::Zero(nfft / 2 + 1);
    Index count = 0;
    for (Index start = 0; start + nperseg <= x.size(); start += step, ++count) {
        Vector seg = x.segment(start, nperseg).cwiseProduct(w);
        seg.array() -= seg.mean() * 0;                          // (optionally detrend: subtract mean)
        const ComplexVector X = rfft(seg, nfft);
        acc += X.cwiseAbs2() * scale;
    }
    acc /= static_cast<Real>(std::max<Index>(count, 1));
    acc.segment(1, nfft / 2 - 1) *= 2.0;                        // one-sided: double the interior bins
    return Spectrum{rfftFrequencies(nfft, fs), acc};
}
```

**How it connects.** `welch` is applied to `SimResult` columns from here on (noise spectra on Day 50, residual whiteness checks on Day 59). `csd` is the heart of `etfe` (Day 58). The window table is reused by FIR design (Day 34).

**Verify.** `window(Hann, 8)` = `[0, 0.1883, 0.6113, 0.9505, 0.9505, 0.6113, 0.1883, 0]`. `periodogram` of `A·sin(2π·10t)`, `f_s = 1000`, `N = 1000` (integer cycles): peak bin power × `Δf` (= 1 Hz) ≈ `A²/2`; shifting to 10.5 Hz spreads it over neighbouring bins (leakage) and a Hann window halves the sidelobes in dB terms. Pseudo-white noise (fixed-seed `std::mt19937` + `std::normal_distribution`, `σ = 1`, `f_s = 1000`, 65536 samples): `welch` mean level within 10 % of `0.002`; `integratePSD ≈ 1.0 ± 5 %`. `plotSpectrum(welch(step(E2) + noise…))` shows the resonance at `0.318 Hz` (= `2 rad/s`).

Reading: Oppenheim & Schafer §10.5–10.6 (spectral estimation), Smith chapters 9 and 16; the SciPy `welch` documentation (for conventions).

**Done when.**
- [ ] `window`, `periodogram`, `welch`, `csd`, `integratePSD`, `plotSpectrum`, tests
- [ ] You can explain leakage and the resolution/variance trade-off of Welch's method
- [ ] Commit

---

#### Day 33 — Digital filter structures: `DigitalFilter`, IIR (Direct Form II Transposed), FIR

**C++ you learn.**

- The third abstract base (`DigitalFilter`, Section 4.3.5) with a **non-virtual convenience overload** (`Vector process(const Vector&)`) implemented once in terms of the virtual scalar one.
- **Normalisation in the constructor** (`a[0] = 1`), **state vectors** sized by the order, `reset()`.
- A `final` class, `std::rotate` or index arithmetic for the FIR delay line (choose one; measure if curious).
- Free function `filtfilt` (forward–backward filtering) built *only* from the public interface — a sign the interface is sufficient.

**DSP you learn.** How filters are executed, and that a filter *is* a discrete system.

- **IIR** (recursive) filter `H(z) = B(z)/A(z)`; the **Direct Form II Transposed** structure executes it with `n` state variables and good numerical behaviour:

```
y      = b₀ x + s₁
s_i    = b_i x − a_i y + s_{i+1}      (i = 1 … n−1)
s_n    = b_n x − a_n y
```

- **FIR** filter `y[k] = Σ_{i=0}^{M} h_i x[k−i]`: no feedback, always stable, exactly **linear phase** (constant group delay `M/2` samples) if `h` is symmetric — which is why FIRs are used where waveform shape matters. Costs more taps for a sharp cutoff than an IIR.
- Two filters you already understand: the **moving average** (FIR, `h_i = 1/N`; nulls at multiples of `f_s/N`, poor stopband) and **exponential smoothing** `y = αx + (1−α)y_prev` (first-order IIR, pole at `1 − α`, cutoff `f_c ≈ α f_s/(2π)` for small α).
- `toTransferFunction()` returns a discrete `TransferFunction` — so Bode (Day 17), poles, `step` all apply, and `step(filter.toTransferFunction())` must equal `filter.process(stepSignal)`. A filter and a system are the same thing seen from two modules.
- **Zero-phase filtering** (`filtfilt`): filter forward, reverse, filter again, reverse — squared magnitude, zero phase; offline only (non-causal). Useful for cleaning identification data (Day 56).

**Files you touch.** `include/csl/dsp/DigitalFilter.hpp`, `src/dsp/DigitalFilter.cpp`, tests.

**Code.**

```cpp
// include/csl/dsp/DigitalFilter.hpp
#pragma once
#include <memory>
#include "csl/core/Types.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::dsp {

class DigitalFilter {
public:
    virtual ~DigitalFilter() = default;
    virtual Real process(Real x) = 0;
    Vector process(const Vector& x);                 // calls the scalar version N times
    virtual void reset() = 0;
    virtual Real Ts() const = 0;
    virtual systems::TransferFunction toTransferFunction() const = 0;
    virtual std::unique_ptr<DigitalFilter> clone() const = 0;    // for filtfilt and cascades
};

class IIRFilter final : public DigitalFilter {
public:
    IIRFilter(Vector b, Vector a, Real Ts);           // H(z) = B(z)/A(z) in descending powers of z (same length; pad with zeros)
    explicit IIRFilter(const systems::TransferFunction& H);   // discrete, proper
    Real process(Real x) override;                     // Direct Form II Transposed
    void reset() override { s_.setZero(); }
    Real Ts() const override { return Ts_; }
    systems::TransferFunction toTransferFunction() const override;
    std::unique_ptr<DigitalFilter> clone() const override { return std::make_unique<IIRFilter>(*this); }
private:
    Vector b_, a_, s_;  Real Ts_;
};

class FIRFilter final : public DigitalFilter {
public:
    FIRFilter(Vector h, Real Ts);
    Real process(Real x) override;
    void reset() override;
    Real Ts() const override { return Ts_; }
    systems::TransferFunction toTransferFunction() const override;   // H(z) = sum h_i z^{-i}  => num = h, den = z^M
    std::unique_ptr<DigitalFilter> clone() const override { return std::make_unique<FIRFilter>(*this); }
    Real groupDelaySamples() const { return (h_.size() - 1) / 2.0; }
private:
    Vector h_, line_;  Index head_ = 0;  Real Ts_;    // circular delay line
};

Vector filtfilt(const DigitalFilter& f, const Vector& x);       // zero-phase, offline
FIRFilter movingAverageFIR(Index N, Real Ts);
IIRFilter exponentialSmoothing(Real alpha, Real Ts);
}
```

```cpp
Real IIRFilter::process(Real x) {
    const Index n = s_.size();                 // order
    const Real y = b_(0) * x + (n > 0 ? s_(0) : 0.0);
    for (Index i = 0; i + 1 < n; ++i) s_(i) = b_(i + 1) * x - a_(i + 1) * y + s_(i + 1);
    if (n > 0) s_(n - 1) = b_(n) * x - a_(n) * y;
    return y;
}
```

**How it connects.** Filter *design* (tomorrow) returns these classes. `IIRFilter(const TransferFunction&)` means any discretized controller is also a filter (and vice versa); `TransferFunctionController` could be reimplemented on top of it — do so if you like, the tests will tell you it is equivalent. Day 35's anti-aliasing filter and Day 50's noise-shaping use them; Day 56 cleans identification data with `filtfilt`.

**Verify.** `movingAverageFIR(4)` at `f = f_s/4` has `|H| < 1e-12` (a null) and `|H(1)| = 1`. `exponentialSmoothing(0.1)` has pole `0.9` and dc gain 1. For `H(z) = (0.5 z + 0.5)/(z − 0.3)`: `IIRFilter(H).process(step)` equals `step(H).y` sample for sample to `1e-12`. `filtfilt` of a sine with the Butterworth from tomorrow (or today with `exponentialSmoothing`) has zero phase shift (cross-correlate, or check the peak position). `FIRFilter` with symmetric `h` delays an impulse's centroid by `(M)/2` samples. `toTransferFunction` round-trips.

Reading: Oppenheim & Schafer §6.3–6.5 (structures), Smith chapters 14–15 and 19. C++: nothing new.

**Done when.**
- [ ] `DigitalFilter`, `IIRFilter` (DF2T), `FIRFilter`, `filtfilt`, two example filters, tests
- [ ] You can draw the DF2T block diagram for a second-order section
- [ ] Commit

---

#### Day 34 — Filter design: Butterworth, windowed-sinc FIR, notch (function templates)

**C++ you learn.**

- **Function templates**: `template <typename Derived> Real rms(const Eigen::MatrixBase<Derived>& v)` accepts *any* Eigen expression (`r.y.col(0)`, `a − b`, a `Vector`) without creating a temporary — the standard Eigen idiom. Also `template <typename T> T clampTo(T v, T lo, T hi)`. A template is a recipe the compiler instantiates per type; errors appear at instantiation and are famously verbose — learn to read the *first* line.
- Give `Figure::line` the same treatment so plots accept Eigen expressions directly.
- Design functions returning concrete filter types (`IIRFilter butterworth(...)`), with the analog prototype exposed as a `TransferFunction` for study.

**DSP you learn.** Three classic designs.

- **Butterworth** lowpass: maximally flat, `|H(jω)|² = 1/(1 + (ω/ω_c)^{2n})`. Analog prototype poles lie on a circle of radius `ω_c` at angles `π/2 + (2k−1)π/(2n)`, `k = 1…n` (left half only). Order from specs: `n ≥ log₁₀[(10^{A_s/10} − 1)/(10^{A_p/10} − 1)] / (2 log₁₀(ω_s/ω_p))`. **Digital** Butterworth = analog prototype → bilinear transform (Day 30's `c2d(..., Tustin, prewarp = ω_c)`) so the −3 dB point lands exactly on `f_c`. Highpass: substitute `s → ω_c/s` in the prototype (poles invert: `p → ω_c²/p`… for Butterworth the pole set is symmetric, zeros move to `s = 0`). Band-pass/stop: stretch topic.
- **Windowed-sinc FIR**: the ideal lowpass impulse response is `h[n] = sin(ω_c(n − M/2)) / (π(n − M/2))` (`= ω_c/π` at the centre), truncated to `M+1` taps and multiplied by a window (Day 32). Transition width ≈ `3.3/M` cycles/sample for Hamming — choose `M` from the allowed transition band. Linear phase for free.
- **Notch** (biquad): `H(z) = (1 − 2cos ω₀ z⁻¹ + z⁻²)/(1 − 2r cos ω₀ z⁻¹ + r² z⁻²)`, `ω₀ = 2π f₀/f_s`, `r = 1 − π·BW/f_s` (≈ for narrow notches; `r = e^{−π BW/f_s}` is cleaner). Zeros on the unit circle at `±ω₀` kill the tone (Day 12: a zero at `jω₀` blocks that frequency); poles just inside make the notch narrow. Removes mains hum (50/60 Hz) from measurements.

**Files you touch.** `include/csl/dsp/FilterDesign.hpp`, `src/dsp/FilterDesign.cpp`, `core/MathUtils.hpp` (`rms`, `clampTo` templates), `Figure.hpp` (template `line`), tests.

**Code.**

```cpp
// include/csl/dsp/FilterDesign.hpp
#pragma once
#include "csl/dsp/DigitalFilter.hpp"
#include "csl/dsp/Spectral.hpp"        // Window
namespace csl::dsp {
enum class FilterType { Lowpass, Highpass };

std::vector<Complex>      butterworthPoles(int order, Real wc_rad_s);                 // analog, LHP
systems::TransferFunction butterworthAnalog(int order, Real wc_rad_s, FilterType t = FilterType::Lowpass);
int                       butterworthOrder(Real wp_rad_s, Real ws_rad_s, Real Ap_dB, Real As_dB);
IIRFilter                 butterworth(int order, Real fc_hz, Real fs_hz, FilterType t = FilterType::Lowpass);

FIRFilter firWindowedSinc(int numTaps, Real fc_hz, Real fs_hz, Window w = Window::Hamming, FilterType t = FilterType::Lowpass);
IIRFilter notch(Real f0_hz, Real bandwidth_hz, Real fs_hz);
}
```

```cpp
std::vector<Complex> butterworthPoles(int n, Real wc) {
    std::vector<Complex> p;
    for (int k = 1; k <= n; ++k) p.push_back(wc * std::polar(1.0, PI / 2 + (2.0 * k - 1) * PI / (2.0 * n)));
    return p;
}
IIRFilter butterworth(int order, Real fc, Real fs, FilterType t) {
    const Real wc = TWO_PI * fc;
    const systems::TransferFunction Ha = butterworthAnalog(order, wc, t);
    const systems::TransferFunction Hd = systems::c2d(Ha, 1.0 / fs, systems::DiscretizationMethod::Tustin, wc);  // prewarp at fc
    return IIRFilter(Hd);
}
IIRFilter notch(Real f0, Real bw, Real fs) {
    const Real w0 = TWO_PI * f0 / fs, r = std::exp(-PI * bw / fs);
    Vector b(3), a(3);
    b << 1.0, -2.0 * std::cos(w0), 1.0;
    a << 1.0, -2.0 * r * std::cos(w0), r * r;
    const Real g = /* normalise dc gain to 1: */ (a.sum()) / (b.sum());
    return IIRFilter(g * b, a, 1.0 / fs);
}
```

**How it connects.** `butterworth` is the anti-aliasing and measurement filter of Day 35, the noise-shaping tool of Day 50, and the pre-processing filter of Day 56. `notch` removes a disturbance tone in `ex04`. Every filter is analysed with `plotBode(frequencyResponse(f.toTransferFunction()))` — no new plotting code.

**Verify.** `butterworthPoles(2, 1)` = `−0.7071 ± 0.7071j`; `butterworthAnalog(3, 1)` isApprox `{1}/{1, 2, 2, 1}`; `|H_a(j1)| = −3.0103 dB`. `butterworth(4, 100, 1000)`: `|H|` at 100 Hz = `−3.01 dB ± 0.01` (prewarped), at 200 Hz = `−27.97 dB ± 0.05` (`10 log(1 + (tan(0.2π)/tan(0.1π))⁸) = 10 log 626`), poles inside the unit circle. `butterworthOrder(2π·100, 2π·200, 1, 40)` = 8 (the formula gives 7.62, rounded up). `firWindowedSinc(51, 100, 1000)`: group delay 25 samples, `|H(0)| = 1 ± 0.01`, `|H|` at 300 Hz `< −50 dB`. `notch(50, 2, 1000)`: `|H|` at 50 Hz `< −40 dB`, at 40 Hz and 60 Hz `> −1 dB`, dc gain 1.

Reading: Oppenheim & Schafer §7.1–7.3 (Butterworth, bilinear), §7.5 (windowed FIR); Smith chapters 16 and 20. C++: C++ Primer §16.1 (function templates — read §16.1.1–16.1.3 today), Eigen "Writing functions taking Eigen types as parameters".

**Done when.**
- [ ] Butterworth (analog + digital, lowpass/highpass, order formula), windowed FIR, notch, tests
- [ ] `rms` works on an Eigen expression without a temporary; you can read a template error
- [ ] Commit

---

#### Day 35 — Digital control: emulation vs. direct design, sampling's cost, dead-beat control

**C++ you learn.** No new feature. A day of **composition and measurement**: build experiments from existing pieces, record numbers, and promote the ones worth keeping into tests. Add `Simulator::setMeasurementFilter(std::unique_ptr<DigitalFilter>)` (applied to `y` before the controller — modelling an anti-aliasing/measurement filter running at the controller rate).

**Control you learn.** What sampling does to a loop you designed in continuous time, and what you can do *only* in discrete time.

- **Emulation**: design `C(s)` continuous (Phases 3–4), discretize with Tustin at `T_s`, run. Cost: the ZOH's `T_s/2` delay plus the controller's own one-sample computational delay eat phase margin: `ΔPM ≈ ω_gc·(T_s/2 + T_s)` radians (hold + computation). Rule: keep `ω_gc T_s ≤ 0.15–0.5` (PM loss ≤ 4–15°).
- **Direct digital design**: discretize the plant with ZOH (`P(z)`), then design in the z-domain. All of Days 17–21 work unchanged on discrete systems: Bode via `z = e^{jωT_s}` (up to the Nyquist frequency), Nyquist, margins, root locus in the z-plane (with the unit circle as the stability boundary and constant-ζ *spirals* instead of lines), Routh via the bilinear map `z = (1+w)/(1−w)`.
- **Dead-beat control**: place all closed-loop poles at `z = 0`: the response reaches the reference in `n` samples and stays — impossible in continuous time. Controller `C(z) = P(z)⁻¹ · z⁻ⁿ/(1 − z⁻ⁿ)`-like constructions, or pole placement at the origin (Day 38 formula on the discrete model). Demands large control effort and exact models — a lesson in what "fast" costs.
- **Aliasing in the loop**: measurement noise above `f_s/2` folds into the control band; a continuous anti-aliasing filter (Butterworth, corner at `~0.4 f_s`) before the sampler is mandatory in practice — but it adds phase lag, so it enters the margin budget too.

**Files you touch.** `Simulator.hpp/.cpp` (measurement filter hook), `examples/ex04_dsp_and_filters.cpp` (part), tests.

**Experiments (each becomes one test or one example section).**
1. E1 with PI (`Kp = 1, Ki = 1`): continuous PM from `margins(C·P)`; discretize `C` with Tustin at `T_s ∈ {0.01, 0.1, 0.5}` and the plant with ZOH; `margins(series(C_d, P_d))` shows PM dropping by ≈ `ω_gc T_s/2`·57.3° (`+ ω_gc T_s`·57.3° if you also insert a one-sample delay `z⁻¹`). Table it.
2. Simulator step responses for the three `T_s` values overlaid on the continuous `feedback(C·P)` step: overshoot grows with `T_s` consistently with (1).
3. Dead-beat on E1 at `T_s = 0.1`: `P(z) = 0.1463/(z − 0.9512)`; a controller placing the closed-loop pole at `z = 0` makes `y` reach `r` in exactly one sample; print `u[0]` (large).
4. Root locus of `P_d(z)` for E2 at `T_s = 0.1` with proportional gain: branches leave the unit circle at the gain where `margins` says `GM`.
5. Noise at 450 Hz on a loop sampled at 500 Hz appears as 50 Hz in `y`; with `setMeasurementFilter(butterworth(2, 200, 500))` it is attenuated — and the PM drops by the filter's phase at `ω_gc`.

**Verify.** Experiment (1)'s measured PM losses agree with the `ω_gc T_s/2` prediction within 2° for `ω_gc T_s < 0.5`. Experiment (3): `|y[1] − r| < 1e-9`. Experiment (4): the z-plane crossing gain equals the discrete gain margin within 1 %.

Reading: FPE §8.4–8.6 (digital design, emulation), Åström & Wittenmark Chapter 4 (pole placement in discrete time; dead-beat).

**Done when.**
- [ ] Five experiments done, the quantitative ones promoted to tests
- [ ] You can state the sample-time rule and *why* (hold delay + computation delay)
- [ ] Commit

---

#### Day 36 — Phase 5 review and `ex04_dsp_and_filters`

**C++ you learn.** Reading your own code critically: run the whole test suite under `/W4 /WX`, fix every warning (signed/unsigned comparisons from the FFT day are the usual suspects), and run the tests in Release to catch anything that only worked by accident in Debug (uninitialised variables). Optional: try the Address Sanitizer (`/fsanitize=address` on MSVC 2022) on the test binary once — it catches memory errors you would otherwise never see.

**Control / DSP you learn.** Nothing new; consolidation.

`ex04_dsp_and_filters.cpp`: (1) generate a chirp 1–100 Hz at `f_s = 1000` with Day 16's `chirp`; (2) FFT it and plot the spectrum (flat over the sweep band); (3) design `butterworth(4, 30, 1000)`, filter the chirp, plot before/after spectra, overlay the filter's Bode from `toTransferFunction()`; (4) simulate E2's step with Day 32's test noise plus a 50 Hz tone added, remove the tone with `notch(50, 2, 1000)`, show `welch` before/after and `filtfilt` vs. `process` phase behaviour; (5) the Day 35 sample-time table.

**Phase 5 review (`sandbox/phase5_review.md`).**
1. In the FFT, why is the permutation *bit-reversal* and not something else? What is the data flow of one butterfly?
2. Why does Tustin preserve stability and forward Euler not? Draw where each maps the `jω` axis.
3. You have 2 s of data sampled at 1 kHz. What is the finest frequency resolution you can get, and what does zero-padding to 8192 points do?
4. Explain to a mechanical engineer why a digital PID at 10 Hz on a loop with crossover at 3 rad/s "feels sluggish" compared with the continuous design.
5. Where in the type system does a `DigitalFilter` meet an `LTISystem`, and why does that matter?

**Done when.**
- [ ] `ex04` runs and produces five HTML plots; zero warnings at `/W4`; tests pass in Release
- [ ] Review written
- [ ] Commit `Day 36: Phase 5 complete`

---
### Phase 6 — State-space design (Days 37–43)

**Phase goal.** Controllability and observability, pole placement (Ackermann), the Luenberger observer and the `Estimator` base, Lyapunov equations and Gramians, reference tracking with feedforward and integral action, and a copy/move audit of the whole library. In C++ terms: numerical rank, Kronecker products and `Eigen::Map`, Cholesky as a definiteness test, the Rule of Zero/Five, move semantics measured with a stopwatch.

---

#### Day 37 — Controllability and observability (`LinearAlgebra.hpp`)

**C++ you learn.**

- **Numerical rank**: rank is not a continuous function of the matrix entries, so "rank" of a floating-point matrix means "number of singular values above a tolerance". Use `Eigen::JacobiSVD` (or `BDCSVD`) singular values with `tol = max(rows, cols) · σ_max · EPS` (the LAPACK/NumPy convention), or `FullPivLU` with `setThreshold`. Expose the tolerance as a parameter; never hard-code `1e-9` deep inside.
- Building block matrices in a loop (`W.block(0, k*m, n, m) = Ak * B; Ak = A * Ak;`).
- The start of `core/LinearAlgebra.hpp`: control-specific matrix functions that Eigen does not have. It grows on Days 40 and 44.

**Control you learn.** Which states you can steer, and which you can see.

- **Controllability**: `(A, B)` is controllable if any state can be reached from any other in finite time. Test: `W_c = [B, AB, A²B, …, Aⁿ⁻¹B]` has rank `n` (Kalman). Intuition: `Aᵏ B` are the directions the input can push in after `k` steps; by Cayley–Hamilton no new directions appear after `n − 1`.
- **Observability**: `(A, C)` is observable if the initial state can be reconstructed from a finite output record. Test: `W_o = [C; CA; …; CAⁿ⁻¹]` has rank `n`. **Duality**: `(A, C)` observable ⇔ `(Aᵀ, Cᵀ)` controllable — one function, two uses.
- **PBH (Popov–Belevitch–Hautus) test**: `(A, B)` controllable ⇔ `rank [λI − A, B] = n` for every eigenvalue `λ`. Unlike the Kalman test it tells you *which mode* is uncontrollable. **Stabilizable** = every unstable mode is controllable; **detectable** = every unstable mode is observable. These weaker conditions are what LQR/LQG actually need (Day 44).
- Physical examples: E2 measuring velocity only (`C = [0, 1]`) is still observable (velocity's derivative involves position); two identical, decoupled first-order tanks fed by the same valve (`A = −I`, `B = [1; 1]`) are *not* controllable (you cannot make their levels differ); a mode that is uncontrollable *and* unobservable cancels in `ss2tf` (Day 14's `minreal`).
- Numerical warning: `W_c` has condition number that explodes with `n` (powers of `A`); for `n > 10` the Gramian test (Day 40) is more trustworthy.

**Files you touch.** `include/csl/core/LinearAlgebra.hpp`, `src/core/LinearAlgebra.cpp`, `StateSpace.hpp` (`isControllable()`, `isObservable()` thin wrappers), tests.

**Code.**

```cpp
// include/csl/core/LinearAlgebra.hpp  (Day 37 part)
#pragma once
#include <vector>
#include "csl/core/Types.hpp"
namespace csl {
Index  numericalRank(const Matrix& M, Real tol = -1.0);        // tol < 0 => max(rows,cols)*smax*EPS
Matrix controllabilityMatrix(const Matrix& A, const Matrix& B);  // n x (n*m)
Matrix observabilityMatrix  (const Matrix& A, const Matrix& C);  // (n*p) x n
bool   isControllable(const Matrix& A, const Matrix& B, Real tol = -1.0);
bool   isObservable  (const Matrix& A, const Matrix& C, Real tol = -1.0);
std::vector<Complex> uncontrollableModes(const Matrix& A, const Matrix& B, Real tol = -1.0);   // PBH
std::vector<Complex> unobservableModes  (const Matrix& A, const Matrix& C, Real tol = -1.0);
bool   isStabilizable(const Matrix& A, const Matrix& B, Real Ts = 0.0, Real tol = -1.0);
bool   isDetectable  (const Matrix& A, const Matrix& C, Real Ts = 0.0, Real tol = -1.0);
}
```

```cpp
Matrix controllabilityMatrix(const Matrix& A, const Matrix& B) {
    const Index n = A.rows(), m = B.cols();
    Matrix W(n, n * m), Ak = Matrix::Identity(n, n);
    for (Index k = 0; k < n; ++k) { W.block(0, k * m, n, m) = Ak * B; Ak = A * Ak; }
    return W;
}
std::vector<Complex> uncontrollableModes(const Matrix& A, const Matrix& B, Real tol) {
    const Index n = A.rows();
    std::vector<Complex> bad;
    Eigen::EigenSolver<Matrix> es(A, false);
    for (Index i = 0; i < n; ++i) {
        const Complex lam = es.eigenvalues()(i);
        ComplexMatrix M(n, n + B.cols());
        M << lam * ComplexMatrix::Identity(n, n) - A.cast<Complex>(), B.cast<Complex>();
        Eigen::JacobiSVD<ComplexMatrix> svd(M);
        const Real smax = svd.singularValues()(0);
        const Real t = tol < 0 ? std::max(M.rows(), M.cols()) * smax * EPS : tol;
        if ((svd.singularValues().array() > t).count() < n) bad.push_back(lam);
    }
    return bad;
}
```

**How it connects.** `isControllable` is the precondition of `place` (tomorrow) and `lqr` (Day 44); `isObservable` of observers and Kalman filters; `isStabilizable/isDetectable` are what the Riccati solvers check. `uncontrollableModes` explains *why* a design function refused.

**Verify.** `plants::doubleIntegrator()`: controllable, observable. `A = −I₂`, `B = [1; 1]`: rank 1, `uncontrollableModes` = `{−1}`, but `isStabilizable` true (the mode is stable). E2 with `C = [0, 1]`: observable. `A = diag(1, −1)`, `B = [0; 1]`: not stabilizable (unstable mode 1 uncontrollable). `tf2ss({1,1}/{1,3,2})` (pole–zero cancellation at −1): controllable (canonical form always is), `unobservableModes` = `{−1}`. `numericalRank` of `[[1, 2],[2, 4 + 1e-15]]` is 1.

Reading: FPE §7.4 (controllability/observability), AM §6.2 and §7.? (reachability, observability — AM uses "reachability"), Chen §6.

**Done when.**
- [ ] Rank, `W_c`/`W_o`, PBH modes, stabilizable/detectable with tests
- [ ] You can explain the two-tank example without equations
- [ ] Commit

---

#### Day 38 — Pole placement: Ackermann's formula and the `StateFeedbackController`

**C++ you learn.**

- **Evaluating a polynomial at a matrix** (Horner with matrices): `Polynomial::evaluate(const Matrix&)` — `P = c₀ I; for i: P = P·A + cᵢ I`. Same algorithm as Day 11, different ring.
- **Solve, don't invert**: `K = eₙᵀ W_c⁻¹ Δ(A)` is computed as `rᵀ Δ(A)` where `r` solves `W_cᵀ r = eₙ`.
- A `Controller` whose "measurement" is the full state (`StateFeedbackController`), plus the Simulator convention that lets you test it before an observer exists: pass the plant with `C = I` so `y = x`.

**Control you learn.** With full state feedback you can put the closed-loop poles *anywhere*.

- `u = −Kx` gives `ẋ = (A − BK)x`. If `(A, B)` is controllable, for every desired monic polynomial `Δ_d(s)` there is a `K` with `det(sI − A + BK) = Δ_d(s)` (unique for SISO). In controllable canonical form it is obvious: the last row of `A − BK` is `−[aₙ + Kₙ … a₁ + K₁]`, so `K` just adjusts the coefficients. **Ackermann's formula** does the coordinate change for you:

```
K = [0 … 0 1] · W_c⁻¹ · Δ_d(A)
```

- Choosing poles: dominant pair from the specs (Day 16), remaining poles 4–10× faster (fast enough not to matter, slow enough not to need huge gains); or standard patterns (ITAE, Bessel). Faster poles ⇒ larger `K` ⇒ larger `u`, more sensitivity to noise and model error. LQR (Day 44) makes this trade-off explicit.
- Discrete time: identical formula with z-plane poles; dead-beat = all poles at `z = 0` (Day 35, now with the general tool).
- Reference tracking with `u = −Kx + N̄ r` where `N̄ = −1/(C(A − BK)⁻¹B)` fixes the dc gain to 1 (SISO). Fragile under model error — Day 41 fixes that with integral action.

**Files you touch.** `include/csl/control/StateFeedback.hpp`, `src/control/StateFeedback.cpp`, `Polynomial.hpp` (`evaluate(Matrix)`), tests.

**Code.**

```cpp
// include/csl/control/StateFeedback.hpp
#pragma once
#include <vector>
#include "csl/control/Controller.hpp"
#include "csl/systems/StateSpace.hpp"
namespace csl::control {

Matrix place(const Matrix& A, const Matrix& B, const std::vector<Complex>& poles);   // SISO Ackermann; throws if uncontrollable
Matrix place(const systems::StateSpace& sys, const std::vector<Complex>& poles);
Real   feedforwardGain(const systems::StateSpace& sys, const Matrix& K);              // Nbar (SISO)

class StateFeedbackController final : public Controller {
public:
    // measurement = full state x (n), reference = r (p). u = -K x + Nbar r   (Day 41 adds integral action)
    StateFeedbackController(Matrix K, Matrix Nbar, Real Ts);
    Vector compute(const Vector& x, const Vector& r, Real t) override;
    void   reset() override {}
    Real   Ts() const override { return Ts_; }
    Index  inputs()  const override { return K_.cols(); }
    Index  outputs() const override { return K_.rows(); }
    const Matrix& gain() const { return K_; }
private:
    Matrix K_, Nbar_; Real Ts_;
};
}
```

```cpp
Matrix place(const Matrix& A, const Matrix& B, const std::vector<Complex>& poles) {
    const Index n = A.rows();
    if (B.cols() != 1) throw NotSupported("place: SISO Ackermann only (MIMO placement is a stretch topic)");
    if (static_cast<Index>(poles.size()) != n) throw InvalidArgument("place: need exactly n poles");
    const Matrix Wc = controllabilityMatrix(A, B);
    if (numericalRank(Wc) < n) throw InvalidArgument("place: (A,B) is not controllable");
    const Polynomial delta = Polynomial::fromRoots(poles);        // desired characteristic polynomial
    Vector en = Vector::Zero(n); en(n - 1) = 1.0;
    const Vector r = Wc.transpose().partialPivLu().solve(en);      // r^T = e_n^T Wc^{-1}
    return r.transpose() * delta.evaluate(A);                      // 1 x n
}
```

**How it connects.** `place` is reused *by duality* for observer gains tomorrow (`place(Aᵀ, Cᵀ, …)ᵀ`), for Kalman-gain sanity checks (Day 52), and as the "compare against" for LQR (Day 44). `StateFeedbackController` is the chassis that LQR and integral control plug a gain into.

**Verify.** Double integrator, poles `−1 ± j`: `K = [2, 2]` (`det(sI − A + BK) = s² + K₂s + K₁`). E3 motor, poles `{−20, −25}`: `K` from `place` reproduces `charPoly(A − BK)` = `{1, 45, 500}` to `1e-9`. Discrete E1 at `T_s = 0.1`, pole `{0}`: closed loop is dead-beat (`A_d − B_d K = 0`). Simulator with plant `C = I` and `StateFeedbackController(K, N̄)`: E3 speed step reaches `1.0 ± 1e-3` with no overshoot for real poles. `place` on the uncontrollable pair throws `InvalidArgument`.

Reading: FPE §7.5 (full-state feedback, Ackermann), AM §6.2–6.3 (state feedback), Chen §8.2.

**Done when.**
- [ ] `place`, `feedforwardGain`, `StateFeedbackController`, tests
- [ ] You can derive Ackermann's formula for `n = 2` on paper
- [ ] Commit

---

#### Day 39 — The Luenberger observer, the `Estimator` base, and the separation principle

**C++ you learn.**

- The fourth abstract base (`Estimator`, Section 4.3.5) — its `predict`/`update` split is dictated by *when* information is available in a sampled loop, and the Kalman filter (Day 51) has the same shape by design.
- The `Simulator` estimator hook: `measurement_for_controller = estimator ? estimator->state() : y`, and the order of operations per tick: `update(y)` → `compute` → `predict(u)`. Get this order wrong and every estimator "lags by one sample" forever; write it once, test it once.
- `std::unique_ptr` member, moved in through a setter; `if (estimator_)` checks.

**Control you learn.** Reconstructing the state from the output, and why the controller and the observer can be designed separately.

- You cannot measure all states (E3: current and speed? usually only speed). An **observer** runs a copy of the model and corrects it with the measured output: `x̂̇ = A x̂ + B u + L (y − C x̂)`. The estimation error `e = x − x̂` obeys `ė = (A − LC) e` — it decays at a rate *you choose* by placing the eigenvalues of `A − LC`. By duality, `L = place(Aᵀ, Cᵀ, poles)ᵀ`. Observable ⇔ any poles achievable.
- **Discrete, current-estimator form** (what the sampled Simulator runs, and the shape of the Kalman filter):

```
predict:  x̄[k+1] = A_d x̂[k] + B_d u[k]
update:   x̂[k+1] = x̄[k+1] + L (y[k+1] − C x̄[k+1])
```

Error dynamics `e[k+1] = (I − LC) A_d e[k]`, so design `L` by `place(A_dᵀ, (C A_d)ᵀ, poles)ᵀ`. Observer poles typically 2–6× faster than the controller's (in continuous terms; map with `z = e^{sT_s}`).
- **Separation principle**: with `u = −K x̂`, the closed-loop poles are exactly `eig(A − BK) ∪ eig(A − LC)`. Proof: write the dynamics in `(x, e)` coordinates — the matrix is block triangular. So you design `K` as if you had the state, and `L` as if there were no controller. This is the single most useful fact in state-space design.
- The observer-based compensator is itself a dynamic system `C(s) = K (sI − A + BK + LC)⁻¹ L` from `y` to `−u` — compute it as a `StateSpace`, convert with `ss2tf`, and look at the loop's margins (Days 17–18). Fast observers trade estimation speed for noise sensitivity, which is why the Kalman filter (Day 51) chooses `L` from noise statistics instead.

**Files you touch.** `include/csl/estimation/Estimator.hpp` (Section 4.3.5), `include/csl/estimation/LuenbergerObserver.hpp` + `.cpp`, `Simulator.hpp/.cpp` (hook), `control/StateFeedback.hpp` (`observerBasedCompensator`), tests.

**Code.**

```cpp
// include/csl/estimation/LuenbergerObserver.hpp
class LuenbergerObserver final : public Estimator {
public:
    LuenbergerObserver(systems::StateSpace discretePlant, Matrix L);                 // Ts > 0
    static Matrix designGain(const systems::StateSpace& discretePlant, const std::vector<Complex>& zPoles);
    void   predict(const Vector& u) override { xbar_ = sys_.A() * xhat_ + sys_.B() * u; predicted_ = true; }
    void   update (const Vector& y) override { const Vector& base = predicted_ ? xbar_ : xhat_; xhat_ = base + L_ * (y - sys_.C() * base); predicted_ = false; }
    Vector state()      const override { return xhat_; }
    Matrix covariance() const override { return Matrix::Zero(xhat_.size(), xhat_.size()); }
    void   reset(const Vector& x0) override { xhat_ = x0; predicted_ = false; }
private:
    systems::StateSpace sys_; Matrix L_; Vector xhat_, xbar_; bool predicted_ = false;
};
```

```cpp
// Simulator::run — the per-tick order with an estimator
Vector y = plant_.C() * x; /* + disturbances/noise */
if (estimator_) estimator_->update(y);
const Vector meas = estimator_ ? estimator_->state() : y;
u = controller_->compute(meas, ref, t);
if (estimator_) estimator_->predict(u);
/* integrate plant over [t, t+Ts] holding u */
```

```cpp
// control/StateFeedback.hpp addition — the compensator from y to u (continuous), for analysis
systems::StateSpace observerBasedCompensator(const systems::StateSpace& plant, const Matrix& K, const Matrix& L);
```

**How it connects.** `Estimator` is the slot the Kalman filter, EKF and UKF fill (Days 51–54) — the Simulator never changes again for estimation. `designGain` is `place` by duality. `observerBasedCompensator` lets you analyse state-space designs with classical tools (and will show LQG's poor margins on Day 52).

**Verify.** E3 discretized at `T_s = 0.01`, observer poles at `z = e^{−40·0.01}` and `e^{−50·0.01}`: starting from a wrong initial estimate, `‖e[k]‖` decays and the ratio `‖e[k+1]‖/‖e[k]‖` approaches `e^{−0.4}` (the slower pole) — test after 50 steps. Closed loop (controller poles `{−20, −25}` placed in continuous time, converted) in the Simulator: output tracks; the closed-loop poles of the continuous `feedback(plant, observerBasedCompensator)` are `{−20, −25, −40, −50}` to `1e-8` — the separation principle, numerically. Removing the observer (`y` = full state) gives the same output to `1e-3` once the observer has converged. Order-of-operations test: with a perfect initial estimate and no noise, `estimator->state()` equals the true state at every tick to `1e-12`.

Reading: FPE §7.6–7.7 (estimators, compensator design), AM §7.2–7.3, Chen §8.3–8.4.

**Done when.**
- [ ] `Estimator`, `LuenbergerObserver`, Simulator hook, `observerBasedCompensator`, tests
- [ ] You can prove the separation principle in `(x, e)` coordinates on paper
- [ ] Commit

---

#### Day 40 — Lyapunov equations, Lyapunov stability, Gramians, the H2 norm

**C++ you learn.**

- **Kronecker product** `kron(A, B)` and **vectorisation** `vec(X)` via `Eigen::Map<const Vector>(X.data(), X.size())` (column-major, so `vec` is free), `unvec` via `Eigen::Map<const Matrix>`. A `Map` is a view over existing memory — no copy; understand that it is only valid while the underlying matrix lives.
- Solving an `n²×n²` linear system with `partialPivLu` (fine for `n ≤ 30`; mention Bartels–Stewart as the scalable alternative, Section 10).
- **Cholesky as a definiteness test**: `Eigen::LLT<Matrix> llt(P); llt.info() == Eigen::Success` ⇔ `P ≻ 0` (numerically). `SelfAdjointEigenSolver` for the eigenvalues of a symmetric matrix when you want to *see* how definite it is.
- Symmetrise after solving (`P = (P + Pᵀ)/2`) — floating point does not know the equation was symmetric.

**Control you learn.** Energy functions, and quadratic equations in matrices.

- **Lyapunov's direct method**: if `V(x) = xᵀPx` with `P ≻ 0` has `V̇ = xᵀ(AᵀP + PA)x < 0` for all `x ≠ 0`, then `ẋ = Ax` is asymptotically stable. Theorem: `A` is stable ⇔ for any `Q ≻ 0` the **Lyapunov equation** `AᵀP + PA + Q = 0` has a unique `P ≻ 0`. (MATLAB's `lyap(A, Q)` solves `AX + XAᵀ + Q = 0`; use that convention for the solver and feed `Aᵀ` when you want `P`.) Discrete: `AᵀPA − P + Q = 0`.
- Solving via Kronecker: `vec(AX) = (I ⊗ A) vec X`, `vec(XAᵀ) = (A ⊗ I) vec X`, so `(I ⊗ A + A ⊗ I) vec X = −vec Q`. Discrete: `(A ⊗ A − I) vec X = −vec Q`. Ten lines, exact, and it teaches the structure. Unique iff no two eigenvalues sum to zero (continuous) / no product equals one (discrete).
- **Gramians**: `W_c = ∫₀^∞ e^{At} B Bᵀ e^{Aᵀt} dt` solves `A W_c + W_c Aᵀ + BBᵀ = 0`; `W_o` solves `AᵀW_o + W_oA + CᵀC = 0`. Controllable ⇔ `W_c ≻ 0`, and `xᵀ W_c⁻¹ x` is the minimum input energy to reach `x` — the Gramian measures *how* controllable each direction is (eigenvalues/vectors). Numerically far better than `W_c` of Day 37 for large `n`.
- **H2 norm** `‖G‖₂² = tr(B ᵀ W_o B) = tr(C W_c Cᵀ)` = the energy of the impulse response = the output variance under unit white-noise input. First of the system norms (H∞ on Day 61); the LQG cost (Day 52) is an H2 norm.
- Balanced realisation (equal, diagonal Gramians) and model reduction: stretch topic.

**Files you touch.** `LinearAlgebra.hpp/.cpp` (`kron`, `vec`, `unvec`, `lyap`, `dlyap`, `controllabilityGramian`, `observabilityGramian`), `analysis/Stability.hpp` (`isLyapunovStable`), `analysis/Norms.hpp` (`h2norm` — created today; `hinfnorm` on Day 61), tests.

**Code.**

```cpp
Matrix kron(const Matrix& A, const Matrix& B) {
    Matrix K(A.rows() * B.rows(), A.cols() * B.cols());
    for (Index i = 0; i < A.rows(); ++i)
        for (Index j = 0; j < A.cols(); ++j)
            K.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
    return K;
}
Vector vec(const Matrix& X)                        { return Eigen::Map<const Vector>(X.data(), X.size()); }
Matrix unvec(const Vector& v, Index r, Index c)    { return Eigen::Map<const Matrix>(v.data(), r, c); }

Matrix lyap(const Matrix& A, const Matrix& Q) {     // A X + X A^T + Q = 0
    const Index n = A.rows();
    const Matrix I = Matrix::Identity(n, n);
    const Matrix M = kron(I, A) + kron(A, I);
    Eigen::PartialPivLU<Matrix> lu(M);
    const Vector x = lu.solve(-vec(Q));
    if ((M * x + vec(Q)).norm() > 1e-8 * (1.0 + vec(Q).norm())) throw NumericalError("lyap: singular (eigenvalues of A sum to zero)");
    Matrix X = unvec(x, n, n);
    return 0.5 * (X + X.transpose());
}
Matrix dlyap(const Matrix& A, const Matrix& Q) {    // A X A^T - X + Q = 0
    const Index n = A.rows();
    const Matrix M = kron(A, A) - Matrix::Identity(n * n, n * n);
    Matrix X = unvec(Vector(M.partialPivLu().solve(-vec(Q))), n, n);
    return 0.5 * (X + X.transpose());
}
Matrix controllabilityGramian(const systems::StateSpace& s) {
    return s.isContinuous() ? lyap(s.A(), s.B() * s.B().transpose()) : dlyap(s.A(), s.B() * s.B().transpose());
}
Real h2norm(const systems::StateSpace& s) {
    if (!s.isStable()) throw InvalidArgument("h2norm: system must be stable");
    if (s.isContinuous() && s.D().norm() > 0) throw InvalidArgument("h2norm: continuous system must be strictly proper");
    const Matrix Wo = observabilityGramian(s);
    return std::sqrt((s.B().transpose() * Wo * s.B()).trace() + (s.isDiscrete() ? (s.D().transpose() * s.D()).trace() : 0.0));
}
```

**How it connects.** `lyap`/`dlyap` are reused by the H2-norm, the LQG cost evaluation (Day 52), the Kalman steady-state covariance check (Day 51: `P` of the KF satisfies a Lyapunov equation given `K`), and H∞ loop-shaping bounds (Day 61). `isLyapunovStable` is the third stability test in the library (after poles and Routh) — and the one that generalises to nonlinear systems in theory.

**Verify.** `lyap(−1, 2)` = `1`. `lyap(A, Q)` for E2 and `Q = I`: residual `‖AX + XAᵀ + Q‖ < 1e-12`, `X ≻ 0`. `controllabilityGramian(E2)` equals the numerical integral `∫₀^{60} e^{At}BBᵀe^{Aᵀt} dt` (trapezoid, `dt = 0.001`, using `(A·t).exp()`) to `1e-4`. `isLyapunovStable(A)` true for E1–E3, false for `A = [[0,1],[−4,0]]` (marginal) and for the double integrator. `h2norm({1},{1,1})` = `0.70711`; `h2norm(E2)` = `0.5` (closed form `1/(4ζωn³)` under the root = `1/4`); discrete `h2norm` of `0.5/(z − 0.5)` = `√(0.25/(1 − 0.25)) = 0.57735`. Uncontrollable pair `A = −I`, `B = [1;1]`: `W_c` singular (min eigenvalue `< 1e-12`).

Reading: Chen §5.4–5.5 (Lyapunov equations and stability); Zhou, Doyle & Glover, *Essentials of Robust Control* chapters 3–4 (Gramians, H2 norm — concise); Eigen "Map" page.

**Done when.**
- [ ] `kron`/`vec`, `lyap`/`dlyap`, Gramians, `isLyapunovStable`, `h2norm`, tests
- [ ] You can explain why `xᵀW_c⁻¹x` is a minimum energy
- [ ] Commit

---

#### Day 41 — Reference tracking: feedforward `(N_x, N_u)` and integral action

**C++ you learn.**

- **Augmented systems built with blocks** (`A_aug = [[A, 0],[−C, 0]]`, `B_aug = [B; 0]`), and a `Controller` that now *has state* (the integral of the tracking error) — `reset()` matters again.
- A configuration struct for the controller (`IntegralStateFeedback::Config { Matrix K, Ki; Real Ts; … }`) so the same class serves pole placement today and LQR on Day 46.
- Solving a small linear system for `(N_x, N_u)` with `partialPivLu`.

**Control you learn.** Making the output follow a reference *robustly*.

- **Feedforward**: steady state with `y = r` requires `0 = A x_ss + B u_ss`, `r = C x_ss + D u_ss`. Solve `[[A, B],[C, D]] [N_x; N_u] = [0; I]`, then `x_ss = N_x r`, `u_ss = N_u r`, and `u = −K(x − N_x r) + N_u r = −Kx + N̄ r` with `N̄ = N_u + K N_x` (reduces to yesterday's SISO `N̄`). Exact tracking *if the model is exact*. Any gain error or constant disturbance ⇒ steady-state error.
- **Integral action**: add a state `x_I` with `ẋ_I = r − y`, so the augmented plant is `[ẋ; ẋ_I] = [[A, 0],[−C, 0]] [x; x_I] + [B; 0] u + [0; I] r`. Design `[K, −K_I]` on `(A_aug, B_aug)` by pole placement (or LQR); then `u = −Kx − K_I x_I`… with sign conventions chosen so that `K_I > 0` integrates the error. Steady state forces `y = r` regardless of constant disturbances or gain errors — same reason the PID's I-term works (Day 23), now in state space and MIMO-ready. Controllability of the augmented pair requires the plant to have no zero at `s = 0`.
- Both can be combined (feedforward for speed, integral for accuracy). Anti-windup applies to `x_I` exactly as on Day 25.

**Files you touch.** `StateFeedback.hpp/.cpp` (`feedforwardGains`, `augmentWithIntegrator`, `IntegralStateFeedbackController`), tests.

**Code.**

```cpp
struct FeedforwardGains { Matrix Nx, Nu, Nbar; };
FeedforwardGains feedforwardGains(const systems::StateSpace& sys, const Matrix& K);
systems::StateSpace augmentWithIntegrator(const systems::StateSpace& sys);   // state [x; xI], extra input r not included (B unchanged)

class IntegralStateFeedbackController final : public Controller {
public:
    struct Config { Matrix K; Matrix Ki; Real Ts; Real uMin = -INF, uMax = INF; bool antiWindup = true; };
    explicit IntegralStateFeedbackController(Config cfg);
    Vector compute(const Vector& x, const Vector& r, Real t) override;   // u = -K x - Ki xI,  xI += Ts (r - C x)  — C passed in Config or r - y supplied as measurement tail
    ...
};
```

(Design decision to make consciously: the controller needs `y` to integrate `r − y` but its measurement is `x`. Either give the `Config` the output matrix `C` so it computes `y = Cx`, or have the Simulator append `y` to the measurement. The former keeps the Simulator generic — choose it and document it.)

**How it connects.** `augmentWithIntegrator` + `place` today; `augmentWithIntegrator` + `lqr` on Day 46; the same controller class in both. The disturbance-rejection test is reused for LQG (Day 52) and MPC (Day 64).

**Verify.** E3 motor: `feedforwardGains` with `K` from Day 38 gives `N̄` equal to Day 38's SISO formula to `1e-12`; step reference reaches `1.0` to `1e-6` with an exact model, but with the plant's `b` increased by 20 % the steady-state error is ≈ `(something you compute)` ≠ 0. With integral action (augmented poles `{−20, −25, −10}`), the output returns to `1.0 ± 1e-3` within 2 s after a step input disturbance of `2 V` at `t = 2 s`, model mismatch or not. `augmentWithIntegrator(E1)` is controllable; of a plant with a zero at 0 (`s/(s+1)` realised) it is not.

Reading: FPE §7.8–7.9 (reference input, integral control), AM §7.4 (integral action).

**Done when.**
- [ ] Feedforward gains, augmentation, integral state feedback, disturbance test
- [ ] You can say why integral action makes the loop robust to gain error and feedforward does not
- [ ] Commit

---

#### Day 42 — Copy, move, and the Rule of Zero: a library-wide audit with a stopwatch

**C++ you learn.** The *special member functions* — the last big piece of the C++ object model.

- Every class has six special members the compiler can generate: default constructor, destructor, copy constructor, copy assignment, **move constructor**, **move assignment**. Copy duplicates resources; move *steals* them from an expiring object (an rvalue: a temporary, or something you `std::move`d) leaving it empty-but-valid. For an Eigen matrix or `std::vector`, copy is `O(n)` and move is `O(1)` (three pointers swap).
- **Rule of Zero**: if your class holds only members that manage themselves (`Matrix`, `std::vector`, `std::string`, `std::unique_ptr`), declare *none* of the six — the compiler-generated ones are correct and efficient. Every class in `csl` is Rule-of-Zero. **Rule of Five**: if you declare *any* of destructor/copy/move (because you manage a raw resource), you must think about all five. You never need it here; know it exists.
- `= default` (ask for the compiler's version explicitly, e.g. a virtual destructor) and `= delete` (forbid: `Simulator(const Simulator&) = delete;` because copying a `unique_ptr` member is impossible — in fact the compiler already deletes it implicitly; writing it makes the intent visible).
- `std::move` is a cast, not an operation: it marks an lvalue as movable. After moving from `v`, do not read `v` (except to assign or destroy it). Return local objects by value without `std::move` — **NRVO/RVO** elides the copy altogether, and `std::move` would *prevent* the elision.
- `noexcept` on move operations lets `std::vector` move instead of copy when it reallocates. Eigen's are `noexcept`; so are yours by inheritance of the rule. Verify with `static_assert(std::is_nothrow_move_constructible_v<csl::systems::StateSpace>)`.
- **Measure**: `std::chrono::steady_clock::now()` around a copy and a move of a `500×500` `StateSpace` (`A` has 250 000 doubles). Expect copy ≈ hundreds of µs, move ≈ tens of ns.

**Control you learn.** Nothing new — but use the free half-day to add `balancedRealization(sys)` (transform so `W_c = W_o = diag(σ)`) and `balancedTruncation(sys, k)` if you want a taste of model reduction (uses Day 40's Gramians + `LLT` + SVD). Optional; it is in Section 10 otherwise.

**Files you touch.** `tests/core/test_SpecialMembers.cpp` (static asserts + timing), small `= delete`/`= default` annotations across `simulation/Simulator.hpp` and the abstract bases, `docs/notes/object_model.md` (write your own one-page summary — the act of writing it is the lesson).

**Verify.** All `static_assert`s compile: `is_nothrow_move_constructible` for `Polynomial`, `StateSpace`, `TransferFunction`, `SimResult`, `FrequencyResponse`, `PID`; `!is_copy_constructible<Simulator>` and `is_move_constructible<Simulator>`. The timing test prints copy/move times and asserts move is at least 100× faster. A test that returns a large `SimResult` from a function and checks (by address of `r.t.data()` before/after) that no copy happened.

Reading: C++ Primer Chapter 13 (all of it — this is the chapter), learncpp chapter 22 (move semantics). Herb Sutter, "GotW #77 / Rule of Zero" blog posts.

**Done when.**
- [ ] Static asserts, timing test, annotations, your one-page object-model note
- [ ] You can explain RVO and why `return std::move(x);` is wrong
- [ ] Commit

---

#### Day 43 — Phase 6 review and `ex05_state_space_design`

**C++ you learn.** Example-driven API polish: while writing `ex05` note every place the API was awkward (an argument order you forgot, a missing overload, a conversion you had to do by hand) and fix it *in the library* rather than working around it in the example.

**`ex05_state_space_design.cpp`.** E3 DC motor, speed measured only: (1) controllability/observability report and Gramian eigenvalues; (2) pole placement for a 2 % settling time of 0.2 s (poles from Day 16's formulas), `N̄` feedforward, Simulator with `C = I`; (3) current-estimator observer at `T_s = 1 ms`, 5× faster; Simulator with the observer, plot true vs. estimated current; (4) integral action and a load-torque disturbance at `t = 1 s`; (5) the observer-based compensator's Bode and margins (classical view of a modern design); (6) `h2norm` of the closed loop from disturbance to speed for designs (2) and (4).

**Phase 6 review (`sandbox/phase6_review.md`).**
1. Write the closed-loop `(x, e)` dynamics for observer-based state feedback and read off the poles.
2. Why is `W_c` (Day 37) numerically worse than the Gramian (Day 40) for large `n`?
3. Which of the six special member functions does `Simulator` have, and why?
4. State the Lyapunov theorem precisely and explain the role of `Q`.
5. When does feedforward beat integral action, and vice versa?

**Done when.**
- [ ] `ex05` runs; any API friction fixed in the library with tests
- [ ] Review written
- [ ] Commit `Day 43: Phase 6 complete`

---
### Phase 7 — Optimal control (Days 44–49)

**Phase goal.** LQR with a hand-written continuous Riccati solver (Hamiltonian method), discrete LQR via the Riccati recursion and a DARE solver, LQR tuning and its guaranteed margins, nonlinear systems with numerical linearization (the cart–pole), and dynamic programming on a grid. In C++ terms: `2n×2n` block matrices, complex eigenvectors and subspace extraction, convergence loops with diagnostics, `std::function`-based nonlinear models, numerical differentiation, multi-dimensional grid indexing.

---

#### Day 44 — LQR and the continuous algebraic Riccati equation (Hamiltonian method)

**C++ you learn.**

- Assembling a `2n×2n` matrix with the comma initialiser from four `n×n` blocks; `Eigen::EigenSolver` *with* eigenvectors (`es.eigenvectors()` is a `ComplexMatrix`); selecting the `n` columns whose eigenvalues have negative real part into a `ComplexMatrix`; a complex **solve** for `P = X₂ X₁⁻¹` written as a transposed solve; `.real()` and symmetrisation; a residual check as a built-in self-test (`‖AᵀP + PA − PSP + Q‖` must be tiny — if not, throw with the number in the message).
- A **general** solver (`solveARE(A, S, Q)`) and thin wrappers (`lqr`) — the general form is what H∞ (Day 66) and the norm computation (Day 61) need.

**Control you learn.** The one algorithm the whole library was pointing at.

- **Problem**: minimise `J = ∫₀^∞ (xᵀQx + uᵀRu) dt`, `Q ⪰ 0`, `R ≻ 0`, subject to `ẋ = Ax + Bu`. `Q` prices state deviation, `R` prices control effort.
- **Solution**: `u = −Kx`, `K = R⁻¹BᵀP`, where `P = Pᵀ ⪰ 0` solves the **continuous algebraic Riccati equation**

```
AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0
```

Sketch of why (completing the square with `V = xᵀPx`): `d/dt(xᵀPx) + xᵀQx + uᵀRu = (u + R⁻¹BᵀPx)ᵀ R (u + R⁻¹BᵀPx)` when `P` satisfies the ARE; the right side is ≥ 0 and zero exactly for `u = −Kx`, so integrating gives `J ≥ x₀ᵀPx₀` with equality for the LQR law. The optimal cost is `x₀ᵀ P x₀`.
- **Conditions**: `(A, B)` stabilizable and `(A, Q^{1/2})` detectable ⇒ a unique stabilizing `P ⪰ 0` exists and `A − BK` is stable. (Day 37's tests are the preconditions.)
- **Hamiltonian method**: with `S = BR⁻¹Bᵀ`, the `2n×2n` matrix `H = [[A, −S],[−Q, −Aᵀ]]` has eigenvalues symmetric about the imaginary axis. Take the `n` eigenvectors with `Re λ < 0`, stack them as `[X₁; X₂]`; then `P = X₂ X₁⁻¹`. The stable eigenvalues of `H` are the closed-loop poles. (Why: the ARE says the subspace spanned by `[I; P]` is `H`-invariant with `A − SP` as the restriction.) Numerically, an ordered Schur decomposition is more robust than eigenvectors; the eigenvector method is adequate for `n ≲ 30` and is far more transparent — it is what you implement.

**Files you touch.** `LinearAlgebra.hpp/.cpp` (`solveARE`, `solveCARE`), `include/csl/control/LQR.hpp`, `src/control/LQR.cpp`, tests.

**Code.**

```cpp
// LinearAlgebra.hpp additions
namespace csl {
// Solves  A^T X + X A - X S X + Q = 0  for the unique stabilizing symmetric X (S = B R^{-1} B^T).
Matrix solveARE(const Matrix& A, const Matrix& S, const Matrix& Q);
Matrix solveCARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);   // wrapper forming S
}
```

```cpp
Matrix solveARE(const Matrix& A, const Matrix& S, const Matrix& Q) {
    const Index n = A.rows();
    Matrix H(2 * n, 2 * n);
    H << A, -S,
        -Q, -A.transpose();
    Eigen::EigenSolver<Matrix> es(H, true);
    const ComplexVector ev = es.eigenvalues();
    const ComplexMatrix V  = es.eigenvectors();

    ComplexMatrix X1(n, n), X2(n, n);
    Index col = 0;
    for (Index i = 0; i < 2 * n; ++i) {
        if (ev(i).real() < 0.0) {
            if (col == n) throw NumericalError("solveARE: more than n stable eigenvalues (numerical trouble)");
            X1.col(col) = V.block(0, i, n, 1);
            X2.col(col) = V.block(n, i, n, 1);
            ++col;
        }
    }
    if (col != n) throw NumericalError("solveARE: expected n stable Hamiltonian eigenvalues, found " + std::to_string(col) +
                                       " (eigenvalues on the imaginary axis: check stabilizability/detectability)");
    // P = X2 * X1^{-1}  computed as  (X1^T \ X2^T)^T
    const ComplexMatrix Pc = X1.transpose().partialPivLu().solve(X2.transpose()).transpose();
    Matrix P = Pc.real();
    P = 0.5 * (P + P.transpose());
    const Real res = (A.transpose() * P + P * A - P * S * P + Q).norm();
    if (res > 1e-6 * (1.0 + Q.norm() + P.norm())) throw NumericalError("solveARE: residual " + std::to_string(res));
    return P;
}
```

```cpp
// include/csl/control/LQR.hpp
namespace csl::control {
struct LQRResult { Matrix K, P; std::vector<Complex> closedLoopPoles; };
LQRResult lqr(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);
LQRResult lqr(const systems::StateSpace& sys, const Matrix& Q, const Matrix& R);     // continuous; throws for discrete (use dlqr)
Real      lqrCost(const Matrix& P, const Vector& x0);                                 // x0' P x0
}
```

**How it connects.** `solveARE` is called by `lqr` (today), `dlqr`'s cross-check (Day 45), the steady-state Kalman gain (Day 52, through the discrete version), the H∞ norm (Day 61) and H∞ synthesis (Day 66). `LQRResult::K` drops straight into Day 38's `StateFeedbackController`. The cost `x₀ᵀPx₀` is checked against a simulated integral — Days 7, 24 and 40 all in one test.

**Verify.** Double integrator, `Q = I`, `R = 1`: `P = [[√3, 1],[1, √3]]`, `K = [1, 1.7321]`, closed-loop poles `−0.8660 ± 0.5j` (`= e^{±j150°}` — the Butterworth pattern!), all to `1e-9`; ARE residual `< 1e-12`. E3 motor with `Q = diag(1, 100)`, `R = 1`: `A − BK` stable; the simulated cost `∫₀^{T}(xᵀQx + uᵀRu)dt` (Simulator with `C = I`, `T_s = 1e-4`, trapezoid) from `x₀ = [0, 1]` equals `x₀ᵀPx₀` within `0.5 %`. The stable eigenvalues of `H` equal `eig(A − BK)` to `1e-8`. Non-detectable case (`Q = 0`, unstable `A`) throws `NumericalError` with a helpful message.

Reading: AM §6.5 (LQR), FPE §7.10 (symmetric root locus and LQR), Anderson & Moore *Optimal Control: Linear Quadratic Methods* §2–3 (free PDF) for the derivation; Laub (1979) "A Schur method for solving algebraic Riccati equations" for the numerical story.

**Done when.**
- [ ] `solveARE`, `lqr`, cost check, tests
- [ ] You can reproduce the completing-the-square argument on paper
- [ ] Commit

---

#### Day 45 — Discrete LQR: the finite-horizon Riccati recursion and the DARE

**C++ you learn.**

- **Backward loops** over a `std::vector<Matrix>` of gains (index `N−1 … 0`), and **convergence loops** with a relative-change criterion, an iteration cap, and a `NumericalError` that reports the last change when it fails. These two loop shapes recur in DP (Day 48), the Kalman filter (Day 51), RLS (Day 57), value iteration and iLQR (Day 65).
- Reusing the `LQRResult` struct for the discrete case; a `TimeVaryingGains` struct for the finite-horizon result.

**Control you learn.** Optimal control as dynamic programming — the Riccati *recursion*.

- **Finite horizon**: `J = Σ_{k=0}^{N−1} (xₖᵀQxₖ + uₖᵀRuₖ) + x_Nᵀ Q_N x_N`. Bellman: the cost-to-go from `k` is quadratic, `V_k(x) = xᵀP_kx`, and going backwards from `P_N = Q_N`:

```
K_k = (R + Bᵀ P_{k+1} B)⁻¹ Bᵀ P_{k+1} A
P_k = Q + Aᵀ P_{k+1} (A − B K_k)                       (one of several equivalent forms)
```

`u_k = −K_k x_k` is optimal, and `K_k` is *time-varying* near the end of the horizon. This is the principle of optimality made executable; MPC (Day 63) runs exactly this over a sliding window.
- **Infinite horizon**: the recursion converges (stabilizable + detectable) to the solution of the **discrete ARE**

```
P = Q + AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA,     K = (R + BᵀPB)⁻¹ BᵀPA
```

Implement `solveDARE` by iterating the recursion to convergence (simple, robust, linear convergence — typically < 500 iterations for the systems here); the doubling algorithm and the symplectic-pencil method are Section 10 stretch topics.
- Discrete LQR of a ZOH-discretized plant with weights `Q_d ≈ Q·T_s`, `R_d ≈ R·T_s` approaches continuous LQR as `T_s → 0` (sampled-data LQR with exact weights: stretch).

**Files you touch.** `LinearAlgebra.hpp/.cpp` (`solveDARE`), `LQR.hpp/.cpp` (`dlqr`, `finiteHorizonLQR`), tests.

**Code.**

```cpp
Matrix solveDARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R,
                 Real tol = 1e-10, int maxIter = 10000) {
    Matrix P = Q;
    for (int it = 0; it < maxIter; ++it) {
        const Matrix BtP = B.transpose() * P;
        const Matrix K = (R + BtP * B).ldlt().solve(BtP * A);
        const Matrix Pn = Q + A.transpose() * P * (A - B * K);
        const Real change = (Pn - P).norm() / (1.0 + Pn.norm());
        P = 0.5 * (Pn + Pn.transpose());
        if (change < tol) return P;
    }
    throw NumericalError("solveDARE: no convergence (is (A,B) stabilizable and (A,Q^1/2) detectable?)");
}

struct TimeVaryingGains { std::vector<Matrix> K; std::vector<Matrix> P; };   // K[k], k = 0..N-1 ; P[k], k = 0..N
TimeVaryingGains finiteHorizonLQR(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R, const Matrix& QN, Index N);
LQRResult dlqr(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R);
LQRResult dlqr(const systems::StateSpace& sysd, const Matrix& Q, const Matrix& R);     // requires Ts > 0
```

**How it connects.** `solveDARE` is the steady-state Kalman filter's engine (Day 52 — the same equation with `A ↦ Aᵀ`, `B ↦ Cᵀ`); `finiteHorizonLQR` is the unconstrained MPC (Day 63) and the backward pass of iLQR (Day 65). `dlqr` + `c2d` is how LQR actually runs in the sampled Simulator from now on.

**Verify.** Discrete double integrator (`T_s = 0.1`: `A = [[1, 0.1],[0, 1]]`, `B = [0.005; 0.1]`), `Q = I`, `R = 1`: DARE residual `< 1e-9`; `finiteHorizonLQR(…, N = 300).K[0]` equals `dlqr(...).K` to `1e-6`, while `K[N−1]` differs (end effect); closed-loop poles inside the unit circle. `dlqr(c2d(E3, T_s), Q·T_s, R·T_s).K → lqr(E3, Q, R).K` within `2 %` at `T_s = 0.001`. Divergent case (`A = 2`, `B = 0`) throws. Simulator closed loop with the `dlqr` gain in a `StateFeedbackController` at `T_s = 0.1` on the double integrator from `x₀ = [1, 0]` converges with the predicted cost `x₀ᵀPx₀ ± 1 %` (sum, not integral, of the discrete stage costs).

Reading: Bertsekas, *Dynamic Programming and Optimal Control* Vol. 1 §4.1 (the LQ problem as DP); Åström & Wittenmark §11.1–11.2; Anderson & Moore §2.4.

**Done when.**
- [ ] `solveDARE`, `finiteHorizonLQR`, `dlqr`, tests
- [ ] You can write the Riccati recursion from the Bellman equation on paper
- [ ] Commit

---

#### Day 46 — LQR in practice: Bryson's rule, the trade-off curve, guaranteed margins, integral LQR

**C++ you learn.**

- **Parameter sweeps producing tables** (`std::vector<std::tuple<Real, Real, Real>>` or a small struct), printed with `std::setw` alignment; sorting results by a predicate.
- **Composition of library functions** into a design helper (`lqrIntegral`) — no new mechanism.
- Evaluating a frequency-domain property on a grid and asserting a *bound* (`min |1 + L(jω)| ≥ 1`), with care for the tolerance.

**Control you learn.** How to *use* LQR, and what it guarantees.

- **Bryson's rule**: `Q_ii = 1/(max acceptable x_i)²`, `R_jj = 1/(max acceptable u_j)²`. Makes every term of the cost dimensionless and order one; then adjust one scalar `ρ` in `R = ρ R₀` to trade speed for effort.
- **Trade-off curve**: sweep `ρ` over decades; for each, record settling time, overshoot, peak `|u|`, closed-loop poles. Larger `ρ` ⇒ slower, gentler. Poles move along the *symmetric root locus* (roots of `1 + ρ⁻¹ G(−s)G(s) = 0`) toward a Butterworth pattern — Day 44's `±150°` poles were the `n = 2` case.
- **Guaranteed margins** (Kalman 1964, Safonov–Athans 1977): for SISO LQR with `Q ⪰ 0`, `R > 0`, the loop broken at the plant input, `L(s) = K(sI − A)⁻¹B`, satisfies the *return-difference inequality* `|1 + L(jω)| ≥ 1` for all ω. Consequences: gain margin `[½, ∞)` (−6 dB to +∞), phase margin `≥ 60°`. The Nyquist plot of `L` never enters the unit disc around −1. (These margins are at the *input* of the plant and assume full-state feedback; Day 52 shows they can vanish with an observer.)
- **Output weighting** `Q = Cᵀ Q_y C`: penalise outputs rather than states when states have no physical meaning (e.g. after `tf2ss`).
- **Integral LQR**: Day 41's augmentation with `lqr` on `(A_aug, B_aug)` and a weight on `x_I`.

**Files you touch.** `LQR.hpp/.cpp` (`brysonWeights`, `lqrIntegral`, `lqrLoopTransfer`), `analysis/FrequencyResponse.hpp` (`returnDifferenceMin`), tests, `ex06` (started).

**Code.**

```cpp
Matrix brysonWeights(const Vector& maxValues);                                   // diag(1/max^2)
systems::TransferFunction lqrLoopTransfer(const systems::StateSpace& sys, const Matrix& K);   // K (sI-A)^{-1} B, SISO
Real returnDifferenceMin(const systems::TransferFunction& L, const Vector& w);   // min_w |1 + L(jw)|
struct IntegralLQRResult { Matrix K, Ki, P; };
IntegralLQRResult lqrIntegral(const systems::StateSpace& sys, const Matrix& Q, const Matrix& Qi, const Matrix& R);
```

**How it connects.** `lqrLoopTransfer` + `margins` is the bridge from optimal to classical control. `lqrIntegral` drops into `IntegralStateFeedbackController` (Day 41). The trade-off table is reused in `ex06` and the MPC comparison (Day 64).

**Verify.** For E3 with Bryson weights (`x_max = [2 A, 1 rad/s]`, `u_max = 12 V`) and `ρ ∈ {0.01, 0.1, 1, 10, 100}`: settling time increases monotonically with `ρ`, peak `|u|` decreases monotonically. For every `ρ`: `margins(lqrLoopTransfer).phaseMargin_deg ≥ 60 − 1e-6`, gain margin absent or `≥ 6 dB`, and `returnDifferenceMin ≥ 1 − 1e-9` on a 1000-point log grid. Double integrator `ρ` sweep: closed-loop poles stay on the `±135°`–`±150°` rays (Butterworth pattern for `n = 2`: exactly `±135°` when `Q = diag(1,0)`; verify `Q = diag(1, 0), R = ρ` gives poles at angle `135°` for all ρ). `lqrIntegral` on E3 rejects a step input disturbance to `1e-3`.

Reading: AM §6.5–6.6, FPE §7.10, Anderson & Moore §5 (return difference), Safonov & Athans (1977) abstract.

**Done when.**
- [ ] Bryson, trade-off sweep, margin bound test, integral LQR, tests
- [ ] You can state the LQR margin guarantee precisely, including where the loop is broken
- [ ] Commit

---

#### Day 47 — Nonlinear systems, numerical Jacobians, linearization: the cart–pole

**C++ you learn.**

- A `NonlinearSystem` class holding `std::function` dynamics `f(t, x, u)` and output `h(x, u)` with sizes; **numerical differentiation** (`numericalJacobian`, central differences with step `h = cbrt(EPS)·max(1, |xᵢ|)`), and `linearize(sys, x_eq, u_eq) → StateSpace`.
- Making the `Simulator` accept either plant kind: store the plant internally as `{Dynamics f, Output h, n, m, p}` and give `StateSpace` a conversion into that form. One `run()` for both. (This is a refactor; the Day 24 tests must still pass unchanged.)
- `findEquilibrium(sys, x_guess, u)` with Newton's method using the numerical Jacobian — a few lines that reuse everything.

**Control you learn.** Where linear models come from.

- Real systems are nonlinear: `ẋ = f(x, u)`, `y = h(x, u)`. An **equilibrium** `(x_e, u_e)` has `f(x_e, u_e) = 0`. **Linearization**: for small deviations `x̃ = x − x_e`, `ũ = u − u_e`, Taylor gives `x̃̇ ≈ A x̃ + B ũ` with `A = ∂f/∂x`, `B = ∂f/∂u` at the equilibrium (`C = ∂h/∂x`, `D = ∂h/∂u`). Everything in Phases 2–7 applies to `(A, B, C, D)` — *near* the equilibrium. Lyapunov's indirect method: if the linearization is strictly stable (unstable), so is the equilibrium of the nonlinear system (locally).
- The **cart–pole** (E5): cart mass `M`, pole mass `m`, half-length `l`, force `F` on the cart; state `[p, ṗ, θ, θ̇]`, `θ = 0` upright:

```
p̈ = [ F + m l θ̇² sin θ − m g sin θ cos θ ] / [ M + m sin²θ ]
θ̈ = [ −F cos θ − m l θ̇² sin θ cos θ + (M + m) g sin θ ] / [ l (M + m sin²θ) ]
```

Linearized at upright (`M = 1, m = 0.1, l = 0.5, g = 9.81`):

```
A = [0 1   0     0]     B = [ 0   ]
    [0 0 −0.981  0]         [ 1   ]
    [0 0   0     1]         [ 0   ]
    [0 0 21.582  0]         [−2   ]
```

Open-loop poles `{0, 0, ±4.646}` — unstable, as a balanced broomstick should be. LQR on the linearization stabilises the *nonlinear* system from moderate initial angles; the **region of attraction** is finite — find it by experiment.

**Files you touch.** `include/csl/systems/NonlinearSystem.hpp` + `.cpp`, `NumericalMethods.hpp/.cpp` (`numericalJacobian`, `newtonSolve`), `Simulator` refactor, `examples/plants.hpp` (`cartPole()`), tests.

**Code.**

```cpp
// include/csl/systems/NonlinearSystem.hpp
#pragma once
#include <functional>
#include "csl/systems/StateSpace.hpp"
namespace csl::systems {
class NonlinearSystem {
public:
    using Dynamics = std::function<Vector(Real t, const Vector& x, const Vector& u)>;
    using Output   = std::function<Vector(const Vector& x, const Vector& u)>;
    NonlinearSystem(Dynamics f, Output h, Index n, Index m, Index p);
    Vector derivative(Real t, const Vector& x, const Vector& u) const { return f_(t, x, u); }
    Vector output(const Vector& x, const Vector& u) const { return h_(x, u); }
    Index order() const { return n_; }  Index inputs() const { return m_; }  Index outputs() const { return p_; }
    StateSpace linearize(const Vector& xe, const Vector& ue, Real t = 0.0) const;   // numerical Jacobians
    Vector findEquilibrium(const Vector& xGuess, const Vector& ue, Real tol = 1e-10) const;   // Newton
    static NonlinearSystem fromLinear(const StateSpace& sys);
private:
    Dynamics f_; Output h_; Index n_, m_, p_;
};
}
```

```cpp
// NumericalMethods.hpp addition
Matrix numericalJacobian(const std::function<Vector(const Vector&)>& f, const Vector& x);   // central differences
```

```cpp
// examples/plants.hpp addition
inline csl::systems::NonlinearSystem cartPole(double M = 1.0, double m = 0.1, double l = 0.5, double g = 9.81) {
    auto f = [=](csl::Real, const csl::Vector& x, const csl::Vector& u) {
        const double th = x(2), dth = x(3), F = u(0), s = std::sin(th), c = std::cos(th);
        const double den = M + m * s * s;
        csl::Vector dx(4);
        dx << x(1),
              (F + m * l * dth * dth * s - m * g * s * c) / den,
              dth,
              (-F * c - m * l * dth * dth * s * c + (M + m) * g * s) / (l * den);
        return dx;
    };
    auto h = [](const csl::Vector& x, const csl::Vector&) { csl::Vector y(2); y << x(0), x(2); return y; };
    return csl::systems::NonlinearSystem(f, h, 4, 1, 2);
}
```

**How it connects.** `linearize` feeds `lqr` (today's example), the EKF's Jacobians (Day 53), and iLQR's local models (Day 65). The Simulator now runs nonlinear plants, so Phases 8 and 10 can use the cart–pole throughout. `numericalJacobian` is the EKF's fallback.

**Verify.** `numericalJacobian` of `f(x) = [x₁², x₁x₂]` at `[2, 3]` is `[[4, 0],[3, 2]]` to `1e-7`. `cartPole().linearize(0, 0)` matches the analytic `A`, `B` above to `1e-6`; poles `{0, 0, ±4.646}`. `findEquilibrium` from `θ = 0.3` with `u = 0` returns `θ = 0` (or `π` — document which basin Newton finds; test from `0.3` expecting `0`). LQR with `Q = diag(1, 1, 10, 1)`, `R = 0.1` on the linearization, applied to the *nonlinear* plant in the Simulator (`T_s = 0.01`, measurement = true state): from `θ₀ = 0.2 rad` the state norm is `< 1e-3` at `t = 10 s`; from `θ₀ = 1.0 rad` it fails (`|θ|` exceeds `π/2` at some point) — record the approximate boundary. `Simulator` with `NonlinearSystem::fromLinear(E1)` reproduces Day 24's results to `1e-12`.

Reading: Khalil, *Nonlinear Systems* §2.1–2.3 and §4.3 (linearization, Lyapunov's indirect method); AM §5.1–5.3; the CTMS "Inverted Pendulum: System Modeling" page for the cart–pole derivation.

**Done when.**
- [ ] `NonlinearSystem`, Jacobian, `linearize`, Newton equilibrium, Simulator refactor, cart–pole stabilised, tests
- [ ] You can derive the cart–pole linearization entries `−mg/M` and `(M+m)g/(Ml)` by hand
- [ ] Commit

---

#### Day 48 — Dynamic programming on a grid: value iteration and the Bellman equation

**C++ you learn.**

- **Multi-dimensional grids flattened into one `std::vector<Real>`** (index `i·N₂ + j`), nearest-neighbour and bilinear interpolation on the grid, `argmin` loops over a discrete action set; a `GridMDP` struct.
- The cost of brute force: time one sweep, see `O(N_x^d · N_u)`, and feel the **curse of dimensionality** — the reason LQR/MPC exist.

**Control you learn.** The principle behind both LQR and MPC.

- **Bellman's principle of optimality**: an optimal policy has the property that, whatever the first decision, the remaining decisions are optimal from the resulting state. For a discrete-time cost `Σ l(x, u)`:

```
V(x) = min_u [ l(x, u) + V(f(x, u)) ]                      (infinite horizon, discounted or not)
V_k(x) = min_u [ l(x, u) + V_{k+1}(f(x, u)) ]               (finite horizon, backwards)
```

LQR is the case where `V` is quadratic and the minimisation is in closed form — Day 45's recursion *is* this equation. With constraints (`|u| ≤ u_max`) `V` is no longer quadratic and there is no closed form; DP on a grid computes it numerically, MPC (Day 63) computes it only along the current trajectory.
- **Value iteration**: start with `V = 0`, apply the Bellman operator until the change is small; the optimal policy is the arg-min. **Policy iteration**: alternate policy evaluation (solve for `V` under a fixed policy) and improvement.
- Example with a known answer: the **double integrator with `|u| ≤ 1`**, cost = time to reach the origin (or quadratic). The optimal policy is bang-bang with the switching curve `x₁ = −½ x₂|x₂|` — visible as a sharp boundary in the computed policy. Compare with saturated LQR (`clamp(−Kx, ±1)`): DP wins (fewer steps) because it *plans* the saturation.

**Files you touch.** `include/csl/control/DynamicProgramming.hpp` + `.cpp` (add to the layout under `control/`), tests, `ex06` section.

**Code.**

```cpp
namespace csl::control {
struct GridDP {
    std::vector<Vector> axes;                         // per state dimension: grid coordinates
    Vector actions;                                   // scalar input set
    std::function<Vector(const Vector&, Real)> step;  // discrete dynamics x_{k+1} = f(x, u)
    std::function<Real(const Vector&, Real)> stageCost;
    Real discount = 1.0;
};
struct DPResult { std::vector<Real> V; std::vector<Real> policy; int iterations; Real lastChange; };
DPResult valueIteration(const GridDP& p, Real tol = 1e-6, int maxIter = 2000);     // nearest/bilinear interpolation of V(f(x,u))
Real     interpolate(const GridDP& p, const std::vector<Real>& V, const Vector& x);
}
```

**How it connects.** The Bellman operator written today is the conceptual parent of `finiteHorizonLQR` (Day 45), `MPC` (Day 63) and `iLQR` (Day 65). The bang-bang result is the benchmark that MPC with input constraints should approach on Day 64.

**Verify.** 1-D system `x⁺ = 0.9x + u`, cost `x² + u²`, grid `[−3, 3]` with 601 points, actions 201 in `[−3, 3]`: the computed `V(x)` matches `x²·P` from `solveDARE(0.9, 1, 1, 1)` (`P = 1.4839`, the positive root of `P² − 0.81P − 1 = 0`) within 3 % over `|x| ≤ 2`. Double integrator (`T_s = 0.1`, `|u| ≤ 1`, time-to-origin cost with a terminal box): the policy sign at `(x₁, x₂) = (1, 0)` is `−1`, at `(−1, 0)` is `+1`, at `(0, 1)` is `−1`, at `(−0.6, 1)` (just left of the switching curve `x₁ = −0.5`) is `+1`; VI converges (`lastChange < tol`). Print the sweep time.

Reading: Bertsekas Vol. 1 §1.1–1.3 and §2; Kirk, *Optimal Control Theory* §3.1–3.4 (DP and the Hamilton–Jacobi–Bellman equation, with the bang-bang example).

**Done when.**
- [ ] `valueIteration` on a grid, both tests, timing noted
- [ ] You can explain in one paragraph why LQR, DP and MPC are the same idea at three price points
- [ ] Commit

---

#### Day 49 — Phase 7 review and `ex06_lqr_pendulum`

**`ex06_lqr_pendulum.cpp`.** Cart–pole: (1) linearize, print poles, controllability; (2) Bryson weights from `p_max = 1 m`, `θ_max = 0.2 rad`, `F_max = 10 N`; (3) `lqr` continuous and `dlqr` at `T_s = 0.01`, compare gains; (4) Simulator on the nonlinear plant from `θ₀ = 0.2`, plots of `p`, `θ`, `F`; (5) `ρ` sweep table (settling, peak force); (6) LQR loop transfer margins at the plant input; (7) region-of-attraction scan over `θ₀`; (8) finite-horizon gains plotted over `k` for `N = 100` to show the end effect.

**Phase 7 review (`sandbox/phase7_review.md`).**
1. Derive `K = R⁻¹BᵀP` from the ARE by completing the square.
2. Why do the stable eigenvalues of the Hamiltonian equal the closed-loop poles?
3. Explain the difference between `P_k` in the finite-horizon recursion and `P` from the DARE, and when you would use each.
4. What does the LQR 60° phase margin guarantee *not* cover?
5. Why can DP solve the constrained double integrator exactly while LQR cannot, and what would DP cost for the 4-state cart–pole on a 50-point grid?

**Done when.**
- [ ] `ex06` runs with eight sections and plots; review written
- [ ] Commit `Day 49: Phase 7 complete`

---
### Phase 8 — State estimation (Days 50–55)

**Phase goal.** Noise and randomness done properly, the Kalman filter and its steady state, LQG, the extended and unscented Kalman filters, and Monte Carlo evaluation — all plugged into the Simulator through the `Estimator` base from Day 39. In C++ terms: `<random>` engines and distributions, RNGs as class members, fixed seeds in tests, Cholesky factors, optional callbacks.

---

#### Day 50 — Randomness and noise: `<random>`, `WhiteNoise`, `PRBS`, covariance, Monte Carlo

**C++ you learn.**

- **`<random>`** has two layers: an **engine** (`std::mt19937`, a Mersenne Twister — deterministic given a seed) and **distributions** (`std::normal_distribution<Real>`, `std::uniform_real_distribution`, `std::bernoulli_distribution`) that shape the engine's bits. Distributions have state; keep the engine and the distribution together as members of a class. Seed with a constant in tests (reproducibility) and with `std::random_device{}()` in experiments when you want fresh noise. Never use `rand()`.
- **LFSR** (linear feedback shift register) for the PRBS: more bitwise work (`^`, `>>`, masks); a 10-bit maximal-length register uses taps `x¹⁰ + x⁷ + 1`.
- **Sample statistics** on Eigen data (`.colwise().mean()`, centred matrix, `(Xcᵀ Xc)/(N−1)`), and a `std::vector<SimResult>` aggregated into mean/std trajectories.
- Adding the noise hook to `Simulator` (seeded `WhiteNoise` member).

**Control you learn.** The vocabulary of stochastic signals used by the Kalman filter.

- A **white** sequence has uncorrelated samples and a flat PSD (Day 32 can check both). **Gaussian** white noise with covariance `Σ` is `v ~ N(0, Σ)`; generate it as `v = L z` with `L Lᵀ = Σ` (Cholesky) and `z` standard normal. **Covariance matrix** `E[(x−μ)(x−μ)ᵀ]`: diagonal = variances, off-diagonal = how components move together.
- Model noise: `x_{k+1} = A x_k + B u_k + w_k` (process noise, `Q`), `y_k = C x_k + v_k` (measurement noise, `R`). A continuous white noise with intensity `Q_c` becomes, after ZOH sampling, roughly `Q_d ≈ Q_c T_s` (exact: `∫₀^{T_s} e^{Aτ} Q_c e^{Aᵀτ} dτ`, computable with the Day 29 augmented-exponential trick — implement it as `discretizeNoise`).
- **PRBS** (pseudo-random binary sequence): ±1 sequence with period `2ⁿ − 1`, autocorrelation ≈ an impulse, flat spectrum up to its clock rate — the ideal *persistently exciting* input for identification (Day 56), friendlier to actuators than Gaussian noise.
- **Monte Carlo**: run the closed loop `N` times with independent noise; report mean, standard deviation and worst case of the quantities you care about. The only honest way to evaluate a controller under noise.

**Files you touch.** `signals/Signals.hpp/.cpp` (`WhiteNoise`, `PRBS`, `multivariateNormal`), `LinearAlgebra.hpp` (`sampleCovariance`, `discretizeNoise`), `Simulator` (`setMeasurementNoise`, `setProcessNoise`), `simulation/MonteCarlo.hpp` + `.cpp` (add to layout), tests.

**Code.**

```cpp
// Signals.hpp additions
namespace csl::signals {
class WhiteNoise {
public:
    explicit WhiteNoise(Real stdDev = 1.0, unsigned seed = 1);
    Real   sample()         { return dist_(rng_); }
    Vector sample(Index n);
    void   reseed(unsigned seed) { rng_.seed(seed); dist_.reset(); }
private:
    std::mt19937 rng_;
    std::normal_distribution<Real> dist_;
};
class MultivariateNormal {                   // v = L z, L L^T = Sigma
public:
    MultivariateNormal(const Matrix& Sigma, unsigned seed = 1);
    Vector sample();
private:
    Matrix L_; std::mt19937 rng_; std::normal_distribution<Real> dist_;
};
class PRBS {                                 // maximal-length LFSR, values +-amplitude
public:
    explicit PRBS(int bits = 10, Real amplitude = 1.0, unsigned seed = 1);
    Real   next();
    Vector sample(Index n);
    Index  period() const { return (Index(1) << bits_) - 1; }
private:
    int bits_; Real amp_; std::uint32_t state_;
};
}
```

```cpp
Real PRBS::next() {
    // taps for a maximal-length 10-bit LFSR: bits 10 and 7 (x^10 + x^7 + 1); other lengths: see a taps table
    const std::uint32_t bit = ((state_ >> 0) ^ (state_ >> 3)) & 1u;
    state_ = (state_ >> 1) | (bit << (bits_ - 1));
    return (state_ & 1u) ? amp_ : -amp_;
}
```

```cpp
// MonteCarlo.hpp
namespace csl::simulation {
struct MonteCarloResult { std::vector<SimResult> runs; Vector t; Matrix yMean, yStd, uMean, uStd; };
MonteCarloResult monteCarlo(const std::function<Simulator(unsigned seed)>& makeSimulator,
                            const Vector& x0, Real tf, int runs, unsigned baseSeed = 1000);
}
```

(`makeSimulator` is a factory lambda the caller writes — the Simulator is move-only, so each run builds a fresh one; this is the first time a *factory function* is the natural API.)

**How it connects.** `WhiteNoise` and `PRBS` feed identification (Days 56–58) and every estimator test; `MultivariateNormal` generates process noise; `discretizeNoise` produces the `Q_d` the Kalman filter needs; `monteCarlo` evaluates LQG (Day 52) and MPC (Day 64).

**Verify.** `WhiteNoise(2.0, 42)`: over `10⁵` samples the mean is within `0.02` of 0 and the variance within `2 %` of 4; `welch` of it is flat within `±15 %` across bins. `PRBS(10)`: period 1023, exactly 512 `+1`s per period, normalised autocorrelation at lags `1…50` has magnitude `< 0.01`… `≤ 1/1023 + small` (ideal value `−1/1023`). `MultivariateNormal` with `Σ = [[4, 1],[1, 2]]`: `sampleCovariance` of `10⁵` samples within `3 %` elementwise. `discretizeNoise(A = −1, Q_c = 2, T_s = 0.1)` = `∫₀^{0.1} 2e^{−2τ}dτ = 1 − e^{−0.2} = 0.18127`. Monte Carlo of E1 with PI and measurement noise `σ = 0.05`, 50 runs: `yStd` at `t = 5` within `±30 %` of the value predicted by `h2norm` of the noise-to-output closed loop times `σ` (bonus check tying Day 40 in).

Reading: C++ Primer §17.4 (random numbers); learncpp 8.x on `<random>`. Control: Simon, *Optimal State Estimation* §2–3 (probability refresher), Ljung §13.3 (PRBS).

**Done when.**
- [ ] `WhiteNoise`, `MultivariateNormal`, `PRBS`, `sampleCovariance`, `discretizeNoise`, Simulator noise hooks, `monteCarlo`, tests
- [ ] You can explain why tests seed the engine and experiments may not
- [ ] Commit

---

#### Day 51 — The Kalman filter

**C++ you learn.**

- A concrete `Estimator` with rich *read-only diagnostics* (`innovation()`, `innovationCovariance()`, `gain()`) — expose what a user needs to judge the filter, not just the estimate.
- **Numerical symmetry**: the **Joseph form** of the covariance update keeps `P` symmetric positive semi-definite in floating point; add a `symmetrize()` after every update anyway.
- Tests with a *true* simulated trajectory, a fixed seed, and statistical assertions (RMSE bound, mean NIS within a χ² band).

**Control you learn.** Optimal estimation for linear Gaussian systems.

- Model: `x_{k+1} = A x_k + B u_k + w_k`, `y_k = C x_k + v_k`, `w ~ N(0, Q)`, `v ~ N(0, R)`, independent. The filter keeps the estimate `x̂` and its error covariance `P` (how much to trust `x̂`).

```
Predict:   x̂⁻ = A x̂ + B u               P⁻ = A P Aᵀ + Q
Update:    S  = C P⁻ Cᵀ + R              (innovation covariance)
           K  = P⁻ Cᵀ S⁻¹                (Kalman gain)
           x̂  = x̂⁻ + K (y − C x̂⁻)        (innovation = y − C x̂⁻)
           P  = (I − KC) P⁻ (I − KC)ᵀ + K R Kᵀ     (Joseph form; equals (I − KC)P⁻ in exact arithmetic)
```

- Where it comes from: minimise `tr P` over `K` (or: the conditional mean of a Gaussian given a linear measurement). The gain balances model confidence (`P⁻`) against measurement confidence (`R`): `R → 0` ⇒ trust the measurement; `Q → 0` ⇒ trust the model. It is exactly Day 39's current-estimator observer with `L = K` chosen from noise statistics instead of pole placement — the same `predict`/`update` interface by construction.
- **Consistency checks**: the innovation sequence should be white with covariance `S`; the **NIS** `νᵀS⁻¹ν` should have mean `p` (χ² with `p` degrees of freedom). If the NIS is too large, `Q`/`R` are too optimistic (tune them up).
- **Convergence**: with `(A, C)` detectable and `(A, Q^{1/2})` stabilizable, `P⁻` converges to the solution of a DARE — tomorrow's duality.

**Files you touch.** `include/csl/estimation/KalmanFilter.hpp`, `src/estimation/KalmanFilter.cpp`, tests.

**Code.**

```cpp
class KalmanFilter : public Estimator {                       // not final: EKF/UKF derive conceptually; keep virtuals
public:
    KalmanFilter(systems::StateSpace discretePlant, Matrix Q, Matrix R, const Vector& x0, const Matrix& P0);
    void   predict(const Vector& u) override;
    void   update (const Vector& y) override;
    Vector state()      const override { return x_; }
    Matrix covariance() const override { return P_; }
    void   reset(const Vector& x0) override;
    void   reset(const Vector& x0, const Matrix& P0);
    const Vector& innovation() const { return nu_; }
    const Matrix& innovationCovariance() const { return S_; }
    const Matrix& gain() const { return K_; }
    Real   nis() const { return nu_.dot(S_.ldlt().solve(nu_)); }
protected:
    systems::StateSpace sys_; Matrix Q_, R_;
    Vector x_, nu_; Matrix P_, S_, K_;
    void symmetrize() { P_ = 0.5 * (P_ + P_.transpose()); }
};
```

```cpp
void KalmanFilter::update(const Vector& y) {
    const Matrix& C = sys_.C();
    const Index n = x_.size();
    S_  = C * P_ * C.transpose() + R_;
    K_  = S_.ldlt().solve(C * P_).transpose();                  // K = P C^T S^{-1}  via  S K^T = C P
    nu_ = y - C * x_;
    x_ += K_ * nu_;
    const Matrix IKC = Matrix::Identity(n, n) - K_ * C;
    P_  = IKC * P_ * IKC.transpose() + K_ * R_ * K_.transpose();
    symmetrize();
}
```

**How it connects.** `KalmanFilter` plugs into `Simulator::setEstimator` unchanged from Day 39. Day 52 builds LQG on it and checks `P` against `solveDARE`. The EKF (Day 53) keeps the same update with linearised matrices; the UKF (Day 54) keeps the same *interface*. RLS (Day 57) turns out to be a Kalman filter.

**Verify.** Constant-velocity target: `A = [[1, T],[0, 1]]`, `T = 0.1`, `C = [1, 0]`, `Q = σ_a² [[T⁴/4, T³/2],[T³/2, T²]]` with `σ_a = 0.5`, `R = 1`, seed 7, 500 steps: position RMSE of the estimate `< 0.45` (the raw measurement's is `1.0`), velocity RMSE `< 0.5`; mean NIS in `[0.8, 1.25]`; `P⁻` converges (`‖ΔP‖ < 1e-9`) and equals `solveDARE(Aᵀ, Cᵀ, Q, R)` to `1e-8` (do the duality check today; explain it tomorrow). With `R` 100× too small (overconfident), the mean NIS exceeds 3. E3 motor with noisy speed (`σ_v = 0.05`), estimating current from speed: current RMSE smaller than the naive "compute current from the model only" open-loop estimate. An unobservable configuration (`C = 0`) makes `tr P` grow without bound.

Reading: Simon §5 (the derivation, clear); AM §7.4 (Kalman filter in the observer context); Welch & Bishop, "An Introduction to the Kalman Filter" (short, free); Bar-Shalom for NIS.

**Done when.**
- [ ] `KalmanFilter` with Joseph form, diagnostics, NIS; tests including the DARE duality check
- [ ] You can explain the gain formula in words ("trust ratio") and the Joseph form's purpose
- [ ] Commit

---

#### Day 52 — The steady-state Kalman filter, duality, and LQG

**C++ you learn.**

- A `Controller` built by **composition** of an `Estimator` and a gain (`LQGController` owns a `KalmanFilter` by value and a `Matrix K`) — the controller handles its own estimation so the Simulator sees a plain output-feedback controller. Compare with the Day 39 arrangement (Simulator holds the estimator): both are valid; LQG as one object is how you would ship it.
- A function returning a `StateSpace` representation of the whole dynamic compensator for analysis (`lqgCompensator`), reusing Day 39's `observerBasedCompensator` shape.
- Running the Day 50 `monteCarlo` for a quantitative comparison.

**Control you learn.** The two Riccati equations are one equation, and the optimal output-feedback controller falls out of two designs you already have.

- **Steady-state Kalman gain**: as `k → ∞`, `P⁻ → P_∞` solving `P = A P Aᵀ − A P Cᵀ (C P Cᵀ + R)⁻¹ C P Aᵀ + Q` — the DARE of Day 45 with `(A, B, Q, R) ↦ (Aᵀ, Cᵀ, Q, R)`. **Duality table**: control `(A, B, Q, R, K)` ↔ estimation `(Aᵀ, Cᵀ, Q_w, R_v, Lᵀ)`. Use the constant gain `K_∞` when the transient is unimportant — it is cheaper (no `P` recursion online) and it makes the filter a fixed LTI system you can analyse with Days 17–18.
- **LQG**: minimise `E[xᵀQx + uᵀRu]` under Gaussian noise. **Separation theorem**: the optimal controller is the LQR gain applied to the Kalman estimate, `u = −K_lqr x̂`. Closed-loop poles = LQR poles ∪ Kalman filter poles (Day 39's principle, now with an *optimal* pair). The LQG controller is a dynamic compensator: `x̂_{k+1} = (A − BK − LC) x̂ + L y` (predictor form) or the current-estimator equivalent.
- **LQG has no guaranteed margins** (Doyle, 1978): unlike LQR (Day 46), the loop broken at the plant input can have arbitrarily small gain/phase margins when the filter is "fast" relative to the plant. Check `margins(lqgCompensator ∘ plant)` *every time*. **Loop transfer recovery** (LTR: push `Q_w → ∞` along `B` to recover LQR's margins) is a stretch topic; mention it.
- `h2norm` of the closed loop from noise to `[Q^{1/2}x; R^{1/2}u]` *is* the LQG cost (Day 40) — one more cross-check.

**Files you touch.** `KalmanFilter.hpp/.cpp` (`steadyStateGain`, `steadyStateCovariance`), `include/csl/control/LQG.hpp` + `.cpp`, tests, `ex07` (started).

**Code.**

```cpp
// KalmanFilter.hpp additions
struct SteadyStateKF { Matrix Pminus, Pplus, K; std::vector<Complex> poles; };
SteadyStateKF steadyStateKalman(const systems::StateSpace& sysd, const Matrix& Q, const Matrix& R);

// LQG.hpp
class LQGController final : public Controller {
public:
    struct Config { Matrix Qx, Ru;  Matrix Qw, Rv;  Vector x0;  Matrix P0;  bool steadyState = true;  Matrix Nbar; };
    LQGController(systems::StateSpace discretePlant, Config cfg);
    Vector compute(const Vector& y, const Vector& r, Real t) override;    // update(y) -> u = -K xhat + Nbar r -> predict(u)
    ...
    const KalmanFilter& filter() const;  const Matrix& gain() const;
};
systems::StateSpace lqgCompensator(const systems::StateSpace& sysd, const Matrix& K, const Matrix& L);   // y -> u, discrete
```

**How it connects.** `steadyStateKalman` is `solveDARE` (Day 45) on the dual; `LQGController` is `dlqr` (Day 45) + `KalmanFilter` (Day 51); the margins check is Day 18 on Day 39's compensator form; the Monte Carlo is Day 50. Nothing new is written except the glue — the signature of a mature library.

**Verify.** `steadyStateKalman` on the Day 51 target: `K` equals the converged filter gain to `1e-8`, `Pminus` equals `solveDARE(Aᵀ, Cᵀ, Q, R)`. E3 motor LQG (`T_s = 1 ms`, speed measured with `σ_v = 0.05`, process noise `σ_w` on current): closed-loop poles of `feedback(plant_d, lqgCompensator)` equal `eig(A − BK) ∪ eig((I − LC)A)` to `1e-8`; Monte Carlo (30 runs): speed RMSE after `t = 0.5 s` is lower than with the Day 39 observer placed at the same poles as the KF's (`±5 %` tolerance on "lower"); `margins` of the LQG loop are printed and — with a deliberately fast filter (`Rv` tiny) — the phase margin drops below 30° (test asserts the drop relative to the LQR loop's ≥ 60°).

Reading: AM §7.5; Anderson & Moore chapter 8 (LQG); Doyle (1978) "Guaranteed margins for LQG regulators" (one page — read it); Simon §7.

**Done when.**
- [ ] Steady-state KF, `LQGController`, compensator form, margin and Monte Carlo tests
- [ ] You can fill in the duality table from memory and explain Doyle's point
- [ ] Commit

---

#### Day 53 — The extended Kalman filter

**C++ you learn.**

- **Optional callbacks**: `std::optional<JacobianFunc>` for analytic Jacobians with a fallback to `numericalJacobian` (Day 47) — `std::optional` holding a `std::function`.
- Discretising a continuous nonlinear model for the filter with one RK4 step (`f_d(x, u) = rk4Step(f, 0, x, T_s)`), and differentiating *that map* numerically — the Jacobian of the integrator, not of `f`. A subtle point worth a comment block.
- Angle wrapping of innovations (`wrapToPi`) when a state is an angle — a tiny `std::function<Vector(Vector)> innovationFix` hook.

**Control you learn.** Kalman filtering for nonlinear systems by relinearising every step.

- Model `x_{k+1} = f(x_k, u_k) + w_k`, `y_k = h(x_k) + v_k`. EKF: predict with the nonlinear `f` and propagate `P` with `F = ∂f/∂x|_{x̂}`; update with the nonlinear `h` and `H = ∂h/∂x|_{x̂⁻}`. Everything else is the Day 51 filter. It is a heuristic (no optimality), works well when the nonlinearity is mild over one step and the estimate is close to the truth; it can diverge from a bad initial estimate or with strong nonlinearities — the UKF (tomorrow) handles those better.
- Tuning: inflate `Q` to cover linearisation error; check the NIS exactly as for the KF.
- Test plant: the **simple pendulum** `θ̈ = −(g/l) sin θ − c θ̇ + u`, measuring `y = sin θ` (nonlinear measurement) — small enough to reason about; then the cart–pole measuring `[p, θ]` with noise, closing the loop with the Day 47 LQR gain on the EKF estimate.

**Files you touch.** `include/csl/estimation/ExtendedKalmanFilter.hpp` + `.cpp`, `examples/plants.hpp` (`pendulum()`), tests.

**Code.**

```cpp
class ExtendedKalmanFilter final : public Estimator {
public:
    using StepFunc = std::function<Vector(const Vector& x, const Vector& u)>;    // discrete-time f_d
    using OutFunc  = std::function<Vector(const Vector& x)>;
    using JacX     = std::function<Matrix(const Vector& x, const Vector& u)>;
    using JacH     = std::function<Matrix(const Vector& x)>;
    struct Config {
        StepFunc f; OutFunc h; Matrix Q, R; Vector x0; Matrix P0;
        std::optional<JacX> F;  std::optional<JacH> H;            // analytic Jacobians; numerical if absent
        std::function<Vector(const Vector&)> innovationFix;       // e.g. wrap angle components
    };
    explicit ExtendedKalmanFilter(Config cfg);
    static StepFunc discretize(const systems::NonlinearSystem& sys, Real Ts);   // one RK4 step
    void predict(const Vector& u) override;   // x = f(x,u); P = F P F' + Q
    void update (const Vector& y) override;   // H at x-; then the KF update with h(x-)
    ...
};
```

**How it connects.** Uses `rk4Step` (Day 7), `numericalJacobian` (Day 47), `NonlinearSystem` (Day 47), the KF update (Day 51); plugs into the Simulator (Day 39) with the nonlinear plant (Day 47) and the LQR gain (Day 47). The UKF (tomorrow) shares `Config`'s `f`, `h`, `Q`, `R`.

**Verify.** On a *linear* plant the EKF (numerical Jacobians) matches the `KalmanFilter` to `1e-9` at every step. Pendulum (`l = 1`, `c = 0.1`, `T_s = 0.01`, `σ_v = 0.05` on `sin θ`, seed 3, start `θ₀ = 0.5`, filter initial `0`): RMSE of `θ̂` over `t ∈ [2, 20]` `< 0.02`; analytic vs numerical Jacobians give estimates within `1e-6`; mean NIS in `[0.7, 1.4]`. Cart–pole closed loop (LQR on EKF estimate, noise on `p`, `θ` with `σ = 0.01`, process noise on `F`): balances from `θ₀ = 0.15`, `θ` RMSE in steady state `< 0.01`. Large initial error (`θ̂₀ = 1.5` for a true `0.5`) with the pendulum: EKF converges here (mild case) — record the convergence time for comparison with the UKF.

Reading: Simon §13.1–13.2; Thrun, Burgard & Fox, *Probabilistic Robotics* §3.3 (EKF, very clear).

**Done when.**
- [ ] EKF with optional analytic Jacobians, angle fix, tests on linear/pendulum/cart–pole
- [ ] You can explain why `F` is the Jacobian of the *discrete* map
- [ ] Commit

---

#### Day 54 — The unscented Kalman filter

**C++ you learn.**

- `Eigen::LLT<Matrix>` and `.matrixL()` to get a matrix square root `S` with `S Sᵀ = (n + λ)P`; generating `2n + 1` sigma points as columns of a `Matrix`; weighted means and covariances with `.rowwise()`/loops; cross-covariance.
- Guarding the Cholesky factorisation (`llt.info()`): if `P` loses definiteness, add `ε I` and warn — numerical robustness code that *explains itself* in a comment.
- Sharing `Config` fields with the EKF (a common `NonlinearFilterConfig` base struct, or just the same field names).

**Control you learn.** Propagating a distribution through a nonlinearity without derivatives.

- **Unscented transform**: pick `2n + 1` **sigma points** `χ₀ = x̂`, `χᵢ = x̂ ± (√((n+λ)P))ᵢ`, push *each* through the nonlinear function, and recombine with weights `W_m`, `W_c` (`λ = α²(n + κ) − n`; typical `α = 1e-3…1`, `β = 2`, `κ = 0`). The result matches the true mean and covariance to second order (third for Gaussians) — better than the EKF's first order, at the cost of `2n + 1` function evaluations instead of one Jacobian.
- UKF predict: sigma points of `(x̂, P)` through `f`, recombine, add `Q`. Update: sigma points of `(x̂⁻, P⁻)` through `h`, predicted measurement mean `ŷ`, covariances `P_yy (+R)` and `P_xy`; `K = P_xy P_yy⁻¹`; the usual correction. No Jacobians anywhere.
- When UKF beats EKF: strong nonlinearity over a step, large uncertainty (big `P` relative to the curvature), angle-heavy models (range–bearing). When it does not: mild nonlinearity — both are fine; EKF is cheaper.

**Files you touch.** `include/csl/estimation/UnscentedKalmanFilter.hpp` + `.cpp`, `LinearAlgebra.hpp` (`unscentedTransform` as a free function for testing), tests, `ex07`.

**Code.**

```cpp
struct UnscentedTransformResult { Vector mean; Matrix cov; Matrix crossCov; Matrix sigmaOut; };
UnscentedTransformResult unscentedTransform(const std::function<Vector(const Vector&)>& g,
                                            const Vector& mean, const Matrix& cov,
                                            Real alpha = 1e-3, Real beta = 2.0, Real kappa = 0.0);

class UnscentedKalmanFilter final : public Estimator {
public:
    struct Config { ExtendedKalmanFilter::StepFunc f; ExtendedKalmanFilter::OutFunc h; Matrix Q, R; Vector x0; Matrix P0;
                    Real alpha = 1e-3, beta = 2.0, kappa = 0.0; std::function<Vector(const Vector&)> innovationFix; };
    explicit UnscentedKalmanFilter(Config cfg);
    void predict(const Vector& u) override;
    void update (const Vector& y) override;
    ...
};
```

**How it connects.** Same interface as the EKF, so the Simulator and the cart–pole example swap filters with one line. `unscentedTransform` is reused by the particle filter if you do the stretch topic. The Cholesky factor is the same tool as the `MultivariateNormal` sampler (Day 50).

**Verify.** `unscentedTransform` of a linear map `g(x) = Mx + c` reproduces `Mμ + c` and `MΣMᵀ` to `1e-12`. `g(x) = x²` with `x ~ N(0, 1)`, `β = 2`: mean `1.0` (exact) and variance `2.0` (exact for a Gaussian) to `1e-9` — the EKF would say mean `0`, variance `0`. Pendulum test of Day 53 with the UKF: RMSE within `±10 %` of the EKF's (both fine). Large-initial-error pendulum case: UKF converges at least as fast as the EKF. A range–bearing tracking test (2-D position, measurements `[√(x²+y²), atan2(y, x)]` with `σ_bearing = 0.1 rad`, initial error 5 m): UKF position RMSE at least 20 % lower than the EKF's over the first 50 steps (fixed seed; record the numbers).

Reading: Wan & van der Merwe (2000), "The unscented Kalman filter for nonlinear estimation"; Simon §14; Thrun et al. §3.4.

**Done when.**
- [ ] `unscentedTransform`, `UnscentedKalmanFilter`, tests including the `x²` sanity check and range–bearing comparison
- [ ] You can explain the sigma-point idea with a sketch and say when it is worth its cost
- [ ] Commit

---

#### Day 55 — Phase 8 review and `ex07_estimation`

**`ex07_estimation.cpp`.** (1) Constant-velocity target: raw measurements vs. KF estimate, NIS plot with the χ² band; (2) E3 motor: steady-state KF vs. pole-placed observer under noise, Monte Carlo RMSE table; (3) LQG on E3 with margins printed for two filter speeds; (4) cart–pole: LQR on EKF vs. on UKF estimates with noisy `p, θ`, Monte Carlo of 20 runs, plots of true vs. estimated `θ`; (5) the range–bearing EKF/UKF comparison plot.

**Phase 8 review (`sandbox/phase8_review.md`).**
1. Write the KF predict/update from memory and explain each term's role.
2. State the duality between LQR and the Kalman filter as a table and in one sentence.
3. Why can LQG lose the margins LQR guarantees? Which signal path loses them?
4. EKF vs. UKF: what each approximates, and a case where each is the right choice.
5. What does a mean NIS of 4 (for `p = 1`) tell you to do?

**Done when.**
- [ ] `ex07` with five sections; review written
- [ ] Commit `Day 55: Phase 8 complete`

---
### Phase 9 — System identification (Days 56–59)

**Phase goal.** Build models *from data*: ARX least squares, recursive least squares with forgetting, the empirical transfer function estimate with coherence and a rational fit, and the eigensystem realisation algorithm (a subspace method). Every identified model is a `TransferFunction` or `StateSpace`, so everything from Phases 2–8 applies to it immediately. In C++ terms: regression matrices, QR least squares, SVD, Hankel matrices, numerical conditioning.

---

#### Day 56 — ARX identification by least squares

**C++ you learn.**

- Building a **regression matrix** `Φ` (rows = time samples, columns = lagged signals) with careful index arithmetic; solving the least-squares problem with `Φ.colPivHouseholderQr().solve(y)` (never form `ΦᵀΦ` — squaring the condition number is the classic beginner's error); reading the **condition number** from `JacobiSVD` to judge the data.
- A result struct (`ARXModel`) that knows how to become a `TransferFunction`, how to predict one step ahead, and how to simulate — three different ways to judge a model.
- Train/validation split as a function argument, not a comment.

**Control / ID you learn.** The simplest parametric model and the least-squares machinery behind most of identification.

- **ARX** model: `y[k] + a₁y[k−1] + … + a_{na}y[k−na] = b₁u[k−nk] + … + b_{nb}u[k−nk−nb+1] + e[k]`, equation error `e` assumed white. Write `y[k] = φ[k]ᵀθ + e[k]` with `φ[k] = [−y[k−1] … −y[k−na], u[k−nk] … u[k−nk−nb+1]]ᵀ`, `θ = [a₁…a_{na}, b₁…b_{nb}]ᵀ`; stacking rows gives `y = Φθ + e`, and least squares `θ̂ = argmin ‖y − Φθ‖²`.
- The transfer function `B(z)/A(z)` (with the `nk`-sample delay) is an `LTISystem` with `T_s` — poles, Bode, `c2d`/`d2c`, everything.
- **Persistency of excitation**: `Φ` must have full column rank — a step input cannot identify more than a gain and a time constant; use PRBS (Day 50) or a chirp. **Fit** `= 100·(1 − ‖y − ŷ‖/‖y − ȳ‖)` %. **One-step prediction** (uses measured past `y`) flatters any model; **simulation** (uses only `u`) is the honest test. Validate on data the fit did not see.
- **Order selection**: increase `na`/`nb` until the validation fit stops improving (or use AIC `N ln(σ̂²) + 2·dim θ`). **Bias**: ARX is unbiased only if the true equation error is white; coloured noise biases the poles (ARMAX, output-error and instrumental variables fix this — Section 10).

**Files you touch.** `include/csl/sysid/ARX.hpp`, `src/sysid/ARX.cpp`, `include/csl/sysid/Validation.hpp` + `.cpp` (`fitPercent`, `splitData`), tests, `ex08` (started).

**Code.**

```cpp
// include/csl/sysid/ARX.hpp
#pragma once
#include "csl/systems/TransferFunction.hpp"
namespace csl::sysid {
struct ARXModel {
    Vector a, b;  Index na, nb, nk;  Real Ts;
    Real   noiseVariance;  Real conditionNumber;
    systems::TransferFunction toTransferFunction() const;     // B(z) / A(z) with delay nk (zeros padded)
    Vector predictOneStep(const Vector& y, const Vector& u) const;
    Vector simulate(const Vector& u) const;                     // zero initial conditions
};
ARXModel arx(const Vector& y, const Vector& u, Index na, Index nb, Index nk, Real Ts);
Matrix   arxRegressor(const Vector& y, const Vector& u, Index na, Index nb, Index nk);   // exposed for tests/RLS
Real     fitPercent(const Vector& y, const Vector& yhat);
}
```

```cpp
ARXModel arx(const Vector& y, const Vector& u, Index na, Index nb, Index nk, Real Ts) {
    const Index start = std::max(na, nb + nk - 1);
    const Matrix Phi = arxRegressor(y, u, na, nb, nk);               // rows start..N-1
    const Vector rhs = y.tail(y.size() - start);
    Eigen::ColPivHouseholderQR<Matrix> qr(Phi);
    const Vector theta = qr.solve(rhs);
    Eigen::JacobiSVD<Matrix> svd(Phi);
    ARXModel m;
    m.a = theta.head(na); m.b = theta.tail(nb); m.na = na; m.nb = nb; m.nk = nk; m.Ts = Ts;
    m.noiseVariance = (rhs - Phi * theta).squaredNorm() / static_cast<Real>(rhs.size() - theta.size());
    m.conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    return m;
}
```

**How it connects.** Data come from `lsim` (Day 15) or the `Simulator` (Day 24) with `PRBS` (Day 50) excitation; the model becomes a `TransferFunction` (Day 12) to compare with the truth via `bode`, `poles`, `step`; RLS (tomorrow) uses `arxRegressor` row by row.

**Verify.** Noise-free data from `lsim(c2d(E3, 0.01, ZOH), PRBS(10) of 3000 samples)` with `na = nb = 2`, `nk = 1`: recovered `A(z)`, `B(z)` match the discretised E3 to `1e-8`; fit 100 %. With measurement noise `σ = 0.01` added to `y`: validation *simulation* fit `> 95 %`, poles within `2 %` of the truth; order sweep `na = 1…4` shows the validation fit saturating at 2. Step-input data (not persistently exciting) gives `conditionNumber > 1e6` — the test asserts it. `toTransferFunction().dcGain()` within `3 %` of E3's `0.0999` on noisy data.

Reading: Ljung, *System Identification: Theory for the User* §4.2 (ARX), §7.3 (least squares), §13 (experiment design); or Ljung & Glad, *Modeling of Dynamic Systems* chapters 9–10 for a gentler start.

**Done when.**
- [ ] `arx`, regressor, validation helpers, tests
- [ ] You can explain why simulation fit, not prediction fit, is the honest metric
- [ ] Commit

---

#### Day 57 — Recursive least squares with forgetting (and why it is a Kalman filter)

**C++ you learn.**

- A **stateful online estimator** class (`RLS`) with `update(y, φ)` per sample, exposing `theta()`, `covariance()`, `traceP()`; symmetrising `P` each step; tests that feed a stream and assert on the trajectory of `θ̂`.
- Comparing two implementations of the same mathematics (RLS vs. a `KalmanFilter` on a static-parameter model) to `1e-10` — equivalence tests are the strongest tests you can write.

**Control / ID you learn.** Turning batch least squares into a recursion, and the forgetting factor.

- From batch LS, adding one row `(y, φ)` and applying the matrix-inversion lemma gives

```
K = P φ / (λ + φᵀ P φ)
θ̂ ← θ̂ + K (y − φᵀ θ̂)
P ← (P − K φᵀ P) / λ
```

with **forgetting factor** `λ ∈ (0, 1]`: `λ = 1` is exact batch LS (all data weighted equally); `λ = 0.98` weights data with an effective memory of `1/(1−λ) = 50` samples, so the estimate can track slowly varying parameters. Price: with little excitation, `P` grows (`1/λ` per step) — **covariance blow-up** — and the estimate becomes noise-sensitive; remedies: `λ = 1` when the input is quiet, or a `P` ceiling.
- **RLS is a Kalman filter** for the model `θ_{k+1} = θ_k` (static parameter, `A = I`), `y_k = φ_kᵀ θ_k + v_k` (time-varying `C = φᵀ`), `R = 1`, and with `λ < 1` equivalent to a process noise that inflates `P`. Day 51's filter with these matrices gives the same numbers.

**Files you touch.** `include/csl/sysid/RLS.hpp`, `src/sysid/RLS.cpp`, `KalmanFilter.hpp` (allow a time-varying `C` via a setter, or build a tiny filter in the test), tests.

**Code.**

```cpp
class RLS {
public:
    RLS(Index nParams, Real lambda = 1.0, Real P0 = 1e4);
    void update(Real y, const Vector& phi);
    const Vector& theta() const { return theta_; }
    const Matrix& covariance() const { return P_; }
    Real   lastResidual() const { return eps_; }
    void   reset(Real P0);
private:
    Vector theta_; Matrix P_; Real lambda_, eps_ = 0.0;
};
// Convenience: run RLS over an ARX data record and return the parameter history (N x nParams)
Matrix rlsARX(const Vector& y, const Vector& u, Index na, Index nb, Index nk, Real lambda, Real P0 = 1e4);
```

**How it connects.** Uses `arxRegressor` rows (Day 56); the equivalence test uses `KalmanFilter` (Day 51). Adaptive control (indirect: RLS + online pole placement) is a stretch topic sketched in Section 10 using exactly these pieces.

**Verify.** Noise-free E3 data: `θ̂` is within `1e-6` of the batch ARX estimate after 50 samples with `λ = 1`. Time-varying case: double E3's gain `K_t` at `k = 2000` (regenerate the second half with the new plant); with `λ = 0.98` the `b` estimates reach the new values within 200 samples, with `λ = 1` they are still `> 20 %` off after 1000 samples. Quiet-input case (`u = 0` after `k = 1000`, `λ = 0.98`): `trace(P)` grows by at least 100× over 500 steps (blow-up demonstrated). Equivalence: RLS with `λ = 1` and a `KalmanFilter` with `A = I`, `Q = 0`, `R = 1`, `C_k = φ_kᵀ`, `P₀ = P0·I` agree in `θ̂` to `1e-10` at every step.

Reading: Ljung §11.2 (RLS), §11.7 (forgetting); Åström & Wittenmark, *Adaptive Control* §2.2–2.3.

**Done when.**
- [ ] `RLS`, `rlsARX`, tracking/blow-up/equivalence tests
- [ ] You can derive the RLS update from the matrix-inversion lemma on paper
- [ ] Commit

---

#### Day 58 — Frequency-domain identification: ETFE, coherence, rational fitting, margins from data

**C++ you learn.**

- Elementwise complex operations on `ComplexVector` (`.cwiseQuotient`, `.cwiseAbs2`), returning the library's `FrequencyResponse` struct from a *data* function — the design decision of Section 4.3.4 paying off.
- **Weighted linear least squares with complex data**: stack real and imaginary parts into a real system; iterate weights (Sanathanan–Koerner) in a loop with a convergence check.
- Overlaying a measured and a model response in one `plotBode` call (same struct, same function).

**Control / ID you learn.** Getting a Bode plot from measurements, and a model from the Bode plot.

- **ETFE** (empirical transfer function estimate): `Ĥ(ω) = Y(ω)/U(ω)` from FFTs is unbiased but has variance that does *not* decrease with record length. Smoothing with Welch cross-spectra (Day 32): `Ĥ = P_uy / P_uu`. The **coherence** `γ²(ω) = |P_uy|² / (P_uu P_yy) ∈ [0, 1]` tells where the estimate is trustworthy (linear, noise-free relation ⇒ 1; noise or nonlinearity ⇒ lower). Report `Ĥ` only where `γ² > 0.8–0.9`.
- Excitation for FRF measurement: chirp (Day 16) or PRBS (Day 50) covering the band of interest; long records for low frequencies.
- **Rational fit (Levy / Sanathanan–Koerner)**: find `N(s)/D(s)` with `D` monic minimising `Σ_k |N(jω_k) − Ĥ_k D(jω_k)|²` — linear in the coefficients (Levy). Levy over-weights high frequencies (through `|D|`); SK iteration reweights by `1/|D_prev(jω_k)|²` until the coefficients stop changing. Works for continuous (`s = jω`) and discrete (`z = e^{jωT_s}`) fits.
- **Margins straight from data**: `margins(FrequencyResponse)` (Day 18) on the measured loop FRF — no model at all. This is how a loop is checked on real hardware.
- Also extract a FOPDT model from the FRF (gain from `|Ĥ(0)|`, corner from −3 dB, delay from the phase slope at high frequency) as a second route to Day 26's tuning rules.

**Files you touch.** `include/csl/dsp/Spectral.hpp` (`etfe`, `coherence`), `include/csl/sysid/FrequencyDomainID.hpp` + `.cpp` (`fitRational`, `fitFOPDTfromFRF`), tests, `ex08`.

**Code.**

```cpp
// Spectral.hpp additions
namespace csl::dsp {
struct ETFEResult { analysis::FrequencyResponse H; Vector coherence; Vector f_hz; };
ETFEResult etfe(const Vector& u, const Vector& y, Real fs_hz, Index nperseg = 1024, Real overlap = 0.5, Window w = Window::Hann);
}
// FrequencyDomainID.hpp
namespace csl::sysid {
struct RationalFitOptions { Index numOrder, denOrder; int skIterations = 10; Real minCoherence = 0.0; };
systems::TransferFunction fitRational(const analysis::FrequencyResponse& H, const RationalFitOptions& opt,
                                      const Vector& weights = Vector());   // weights default: 1 (or coherence)
control::tuning::FOPDT fitFOPDTfromFRF(const analysis::FrequencyResponse& H);
}
```

```cpp
ETFEResult etfe(const Vector& u, const Vector& y, Real fs, Index nperseg, Real overlap, Window w) {
    const CrossSpectrum c = csd(u, y, fs, nperseg, overlap, w);        // Pxy = conj(U) Y averaged, plus Puu, Pyy
    ETFEResult r;
    r.f_hz = c.f_hz;
    r.H.w  = TWO_PI * c.f_hz;
    r.H.Ts = 1.0 / fs;
    r.H.H  = c.Pxy.cwiseQuotient(c.Pxx.cast<Complex>());
    r.coherence = c.Pxy.cwiseAbs2().cwiseQuotient(c.Pxx.cwiseProduct(c.Pyy));
    return r;
}
```

**How it connects.** `etfe` = `csd` (Day 32) = `fft` (Day 31). Its output is a `FrequencyResponse`, so `plotBode` (Day 17), `margins` (Day 18) and `fitRational` consume it directly; `fitRational` produces a `TransferFunction` for Phases 3–7. `fitFOPDTfromFRF` feeds Day 26's SIMC rule.

**Verify.** E2 excited by a chirp `0.05–10 Hz` over 120 s at `f_s = 100 Hz`, measurement noise `σ = 0.01`: where `γ² > 0.95`, `|Ĥ|` is within `0.5 dB` and `∠Ĥ` within `3°` of `frequencyResponse(E2)`. `fitRational(Ĥ, {0, 2})` (continuous) recovers `{1}/{1, 0.5, 4}` with coefficients within `2 %`; with the coherence weights the fit is at least as good as unweighted. Discrete fit of `c2d(E2, 0.01)` from the same data: poles within `1 %`. Closed loop E2 with a known PI: `margins(etfe(r_excitation, loop_signal…))` — measure `L` by injecting at the plant input and recording `u` and `y` — phase margin within `2°` of `margins(series(C, P))`. `fitFOPDTfromFRF` on E1 with `pade(0.5, 3)` gives `K = 3 ± 0.05`, `T = 2 ± 0.1`, `L = 0.5 ± 0.05`.

Reading: Ljung §6.3–6.4 (ETFE, spectral analysis), Pintelon & Schoukens, *System Identification: A Frequency Domain Approach* §1–2 (overview); Levy (1959) and Sanathanan & Koerner (1963) originals are short.

**Done when.**
- [ ] `etfe` with coherence, `fitRational` (Levy + SK), FOPDT from FRF, margins-from-data test
- [ ] You can explain what coherence below 1 means and two causes of it
- [ ] Commit

---

#### Day 59 — Subspace identification (ERA / Ho–Kalman), model validation, `ex08`

**C++ you learn.**

- **Hankel matrices** from blocks of Markov parameters; `Eigen::BDCSVD` with thin `U`/`V`; `singularValues()` for order selection; `.asDiagonal()`, elementwise `sqrt` via `.array().sqrt()`; truncation by `.leftCols(n)`.
- A **model validation** toolkit: residual autocorrelation (`autocorrelation(Vector, maxLag)` → whiteness test with the `±1.96/√N` band), cross-correlation between residuals and input, pole comparison, and a `validate(model, u, y)` report struct.

**Control / ID you learn.** State-space models straight from data, and how to tell a good model from a bad one.

- **Markov parameters** `h₀ = D`, `h_k = C A^{k−1} B` (`k ≥ 1`) — the discrete impulse response. From data: (a) `arx` → `simulate` an impulse (parametric route), or (b) correlation deconvolution when the input is white (`R_uy(k) = σ_u² h_k`).
- **ERA (eigensystem realisation algorithm)**: build block Hankel matrices `H₀ = [h_{i+j−1}]`, `H₁ = [h_{i+j}]` (`i, j = 1…r`). SVD `H₀ = UΣVᵀ`; the number of significant singular values is the **model order** `n` (look for the gap). Truncate to `n` and read off

```
A = Σ⁻¹ᐟ² Uᵀ H₁ V Σ⁻¹ᐟ²,     B = first m columns of Σ¹ᐟ² Vᵀ,     C = first p rows of U Σ¹ᐟ²
```

a *balanced* realisation (equal Gramians), minimal by construction. This is the 1960s ancestor of N4SID/MOESP (Section 10), and it already shows the key idea: **the rank of a Hankel matrix of outputs is the system order**.
- **Validation**: (1) simulation fit on fresh data; (2) **residual whiteness** — autocorrelation of `y − ŷ` inside the `±1.96/√N` band for lags ≥ 1; (3) **residual–input independence** — cross-correlation inside the band (otherwise there is unmodelled dynamics); (4) compare poles/Bode across identified models (ARX vs ERA vs rational fit) — agreement across methods is the best confidence you can get without the truth.

**Files you touch.** `include/csl/sysid/Subspace.hpp` + `.cpp` (`markovParameters`, `era`), `Validation.hpp/.cpp` (`autocorrelation`, `crossCorrelation`, `whitenessTest`, `ValidationReport validate(...)`), `examples/ex08_sysid.cpp`, tests.

**Code.**

```cpp
namespace csl::sysid {
std::vector<Matrix> markovParametersFromImpulse(const Matrix& h, Index p, Index m);   // h: K x (p*m) rows of impulse data
std::vector<Matrix> markovParametersByCorrelation(const Vector& u, const Vector& y, Index K);  // white u, SISO
struct ERAResult { systems::StateSpace sys; Vector singularValues; Index order; };
ERAResult era(const std::vector<Matrix>& h, Index r, Index order, Real Ts);          // order <= 0 => auto (gap > 100x)
struct ValidationReport { Real fitPercent; Vector residualAutocorr, residualInputXcorr; Real whitenessBand; bool residualsWhite, residualsIndependent; };
ValidationReport validate(const systems::LTISystem& model, const Vector& u, const Vector& y, Index maxLag = 25);
}
```

```cpp
ERAResult era(const std::vector<Matrix>& h, Index r, Index order, Real Ts) {
    const Index p = h[1].rows(), m = h[1].cols();
    if (static_cast<Index>(h.size()) < 2 * r + 1) throw InvalidArgument("era: need at least 2r+1 Markov parameters");
    Matrix H0(r * p, r * m), H1(r * p, r * m);
    for (Index i = 0; i < r; ++i)
        for (Index j = 0; j < r; ++j) { H0.block(i * p, j * m, p, m) = h[i + j + 1]; H1.block(i * p, j * m, p, m) = h[i + j + 2]; }
    Eigen::BDCSVD<Matrix> svd(H0, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Vector s = svd.singularValues();
    Index n = order;
    if (n <= 0) { n = 1; while (n < s.size() && s(n) > s(0) * 1e-2) ++n; }    // crude gap rule; expose the values so the user can choose
    const Vector sq = s.head(n).array().sqrt();
    const Matrix U = svd.matrixU().leftCols(n), V = svd.matrixV().leftCols(n);
    const Matrix Si = sq.cwiseInverse().asDiagonal(), Sr = sq.asDiagonal();
    const Matrix A = Si * U.transpose() * H1 * V * Si;
    const Matrix B = (Sr * V.transpose()).leftCols(m);
    const Matrix C = (U * Sr).topRows(p);
    return ERAResult{systems::StateSpace(A, B, C, h[0], Ts), s, n};
}
```

**How it connects.** Markov parameters come from `impulse` (Day 15) or correlation with `PRBS` (Day 50); ERA produces a `StateSpace` ready for `dlqr`/`KalmanFilter`; `validate` uses `lsim` and `autocorrelation`. `ex08` runs all three identification routes on the same data and compares them with `plotBode` — the Phase 9 deliverable.

**Verify.** ERA on the *exact* impulse response of `c2d(E3, 0.01)` with `r = 10`: singular values `s(0), s(1)` significant and `s(2)/s(1) < 1e-10`; identified poles equal the true discrete poles to `1e-8`; `ss2tf` matches to `1e-8`. With 1 % noise on the impulse data: order still clearly 2 (`s(2)/s(1) < 0.05`), poles within 2 %. `markovParametersByCorrelation` from PRBS data of 20 000 samples gives `h_k` within 3 % of the truth for `k ≤ 20`. `validate(arxModel, u_val, y_val)` on a correct-order model: `residualsWhite == true`; on an under-fitted model (`na = 1`): `false`. `ex08` prints a table: ARX / ERA / rational-fit poles and dc gains, all within 2 % of each other and of the truth.

Reading: Juang & Pappa (1985), "An eigensystem realization algorithm…"; Ljung §10.6 (subspace methods overview), §16.6 (model validation); Van Overschee & De Moor for N4SID (stretch).

**Done when.**
- [ ] Markov parameters (two routes), `era`, validation toolkit, `ex08`, tests
- [ ] You can explain why the Hankel matrix's rank is the system order
- [ ] Commit `Day 59: Phase 9 complete`

---
### Phase 10 — Robustness, MPC, nonlinear optimal control (Days 60–66)

**Phase goal.** Uncertainty models and the small-gain theorem, exact H∞ and H2 norms, a QP solver, model predictive control with constraints, iterative LQR for the cart–pole swing-up, and an H∞ synthesis capstone. Everything here is *composition* of what the library already has; the days are long on theory and short on new C++.

---

#### Day 60 — Uncertainty models, the small-gain theorem, robust stability and performance

**C++ you learn.**

- Functions over `FrequencyResponse` values (pointwise products, maxima), `std::optional<Real>` for "no bound found", building a **weight** `TransferFunction` from a numeric envelope by Day 58's `fitRational` — the identification tool reused for a design task.
- A `RobustnessReport` struct printed as a table.

**Control you learn.** Designing for the model you *don't* have.

- Models are wrong. Describe the error as an **uncertainty set**. The workhorse: **multiplicative (relative) uncertainty** `P̃(s) = P(s)(1 + W_m(s)Δ(s))`, `‖Δ‖∞ ≤ 1`, where `|W_m(jω)|` bounds the relative error at each frequency (small at low frequency, growing above the model's bandwidth — nobody knows the high-frequency dynamics).
- Building `W_m`: from a family of plausible plants `{P_i}` (parameter variations, measured FRFs from Day 58), compute `max_i |(P_i(jω) − P(jω))/P(jω)|` on a grid and fit a low-order stable `W_m` that covers it (`fitRational` with the "must be above" constraint checked afterwards). An unknown delay `τ ≤ τ_max` is covered by `|e^{−jωτ} − 1| ≤ |W_m|` with `W_m(s) = τ_max s / (1 + τ_max s/2)`·(1.05) — check it covers numerically.
- **Small-gain theorem**: the interconnection of stable `M` and `Δ` is stable for all `‖Δ‖∞ ≤ 1` iff `‖M‖∞ < 1`. For multiplicative uncertainty the relevant `M` is `W_m T`: **robust stability ⇔ `‖W_m T‖∞ < 1`** — the closed loop must roll off `T` where the model is unreliable. (Nyquist view: the disc of radius `|W_m L|` around `L(jω)` must not touch −1.)
- **Nominal performance** `‖W_s S‖∞ < 1` (`W_s` large at low frequency = good tracking). **Robust performance** (SISO): `max_ω (|W_s S| + |W_m T|) < 1`. These three inequalities are the design *specification language* of robust control, and tomorrow's norm is their yardstick.
- Limits revisited (Day 20, Day 28): RHP zeros, delays and the waterbed all appear here as constraints on how small `S` and `T` can be simultaneously.

**Files you touch.** `include/csl/analysis/Robustness.hpp` + `.cpp` (add to the `analysis/` layout), tests.

**Code.**

```cpp
namespace csl::analysis {
Vector relativeErrorEnvelope(const systems::LTISystem& nominal, const std::vector<const systems::LTISystem*>& family, const Vector& w);
systems::TransferFunction delayUncertaintyWeight(Real tauMax, Real margin = 1.05);
struct RobustnessReport { Real robustStabilityPeak; Real nominalPerformancePeak; Real robustPerformancePeak;
                          Real worstFrequency; bool robustlyStable, nominallyPerforming, robustlyPerforming; };
RobustnessReport robustness(const systems::TransferFunction& L, const systems::TransferFunction& Wm,
                            const systems::TransferFunction& Ws, const Vector& w);   // grid version; Day 61 makes RS exact
}
```

**How it connects.** `Wm`, `Ws` are `TransferFunction`s; `S`, `T` are from `gangOfFour` (Day 19); peaks from the grid today and from `hinfnorm` tomorrow; the delay-uncertainty case is checked against Day 18's delay margin. The specification `‖W_s S‖∞`, `‖W_m T‖∞` is exactly what H∞ synthesis (Day 66) minimises.

**Verify.** E3 with the Day 26 SIMC PI: `delayUncertaintyWeight(τ_max)` covers `|e^{−jωτ} − 1|` for all `τ ≤ τ_max` on a grid (test with `τ_max = 0.02`); `robustness(...).robustlyStable` is true for `τ_max` below the delay margin from Day 18 and false above it (`±10 %` — the small-gain test is sufficient, not necessary, so it turns false *slightly before* the exact delay margin; assert that ordering). A ±30 % gain family: `relativeErrorEnvelope` is the constant `0.3`; `‖0.3·T‖∞ < 1` ⇔ `‖T‖∞ < 3.33` — compare with the actual peak of `T`. Robust performance fails when `W_s` demands a bandwidth above where `|W_m| ≈ 1` — construct that case and assert `robustlyPerforming == false`.

Reading: AM Chapter 12 (robust performance — the whole chapter), Skogestad & Postlethwaite, *Multivariable Feedback Control* §7.1–7.6 (uncertainty, small gain), Doyle, Francis & Tannenbaum, *Feedback Control Theory* chapters 4–6 (free PDF; the cleanest SISO treatment).

**Done when.**
- [ ] Envelope, delay weight, `robustness` report, tests
- [ ] You can state the small-gain theorem and draw the Nyquist disc picture
- [ ] Commit

---

#### Day 61 — System norms: the H∞ norm by Hamiltonian bisection, H2 recap

**C++ you learn.**

- **Bisection over a matrix property**: for each trial `γ` build a Hamiltonian and test whether any eigenvalue lies on the imaginary axis (`|Re λ| < ε(1 + |λ|)`); a robust tolerance choice and a *bracket* (`γ_lo` from a grid peak, `γ_hi = 2·γ_lo + ‖D‖`); reuse of `Eigen::EigenSolver`.
- A **MIMO** function (`σ_max` via `JacobiSVD<ComplexMatrix>`) and the SISO fast path; `hinfnorm` overloads for `LTISystem` (exact) and `FrequencyResponse` (grid, for data).

**Control you learn.** The number that robust control is built on.

- `‖G‖∞ = sup_ω σ_max(G(jω))`: the worst-case gain over all frequencies (and over all input directions for MIMO); equals the peak of the Bode magnitude for SISO; equals the induced `L₂`-gain (energy in → energy out). `‖S‖∞ = M_s` of Day 28; `‖W_m T‖∞` of Day 60.
- **Bounded real lemma**: for stable `G = (A, B, C, D)` and `γ > σ_max(D)`, `‖G‖∞ < γ` ⇔ the Hamiltonian

```
H_γ = [ A + B R⁻¹ Dᵀ C            B R⁻¹ Bᵀ         ]      R = γ² I − Dᵀ D
      [ −Cᵀ(I + D R⁻¹ Dᵀ) C      −(A + B R⁻¹ Dᵀ C)ᵀ ]
```

has no eigenvalues on the imaginary axis. (When it does, their imaginary parts are the frequencies where `σ_max(G(jω)) = γ`.) Bisect on `γ` to the desired precision — exact, no grid, no missed narrow peaks (which a grid *will* miss for a lightly damped resonance). This is the same Hamiltonian structure as the ARE of Day 44.
- `‖G‖₂` (Day 40) vs `‖G‖∞`: average (white-noise/impulse-energy) gain vs worst-case sinusoidal gain. LQG minimises an H2 norm; H∞ synthesis (Day 66) minimises an H∞ norm.

**Files you touch.** `include/csl/analysis/Norms.hpp` + `.cpp` (`hinfnorm`, `sigmaMax`, `peakFrequency`), `Robustness.cpp` (exact RS test), tests.

**Code.**

```cpp
Real hinfnorm(const systems::StateSpace& G, Real tol = 1e-6) {
    if (!G.isStable()) throw InvalidArgument("hinfnorm: system must be stable");
    if (G.isDiscrete()) { /* bilinear map to continuous (Day 30, Tustin preserves the norm) then recurse */ }
    const Matrix &A = G.A(), &B = G.B(), &C = G.C(), &D = G.D();
    const Index n = A.rows();
    auto hasImagEigen = [&](Real gamma) {
        const Matrix R = gamma * gamma * Matrix::Identity(D.cols(), D.cols()) - D.transpose() * D;
        const Matrix Rinv = R.ldlt().solve(Matrix::Identity(R.rows(), R.cols()));
        const Matrix Abar = A + B * Rinv * D.transpose() * C;
        Matrix H(2 * n, 2 * n);
        H << Abar, B * Rinv * B.transpose(),
             -C.transpose() * (Matrix::Identity(D.rows(), D.rows()) + D * Rinv * D.transpose()) * C, -Abar.transpose();
        Eigen::EigenSolver<Matrix> es(H, false);
        const Real scale = H.norm();
        for (Index i = 0; i < 2 * n; ++i)
            if (std::abs(es.eigenvalues()(i).real()) < 1e-8 * (1.0 + scale)) return true;
        return false;
    };
    Real lo = std::max(sigmaMax(D), gridPeak(G, 200)) * (1 - 1e-6);   // a lower bound from a coarse grid
    Real hi = 2.0 * lo + 1.0;
    while (hasImagEigen(hi)) hi *= 2.0;                                 // ensure the bracket
    while ((hi - lo) > tol * (1.0 + hi)) { const Real mid = 0.5 * (lo + hi); (hasImagEigen(mid) ? lo : hi) = mid; }
    return hi;
}
```

**How it connects.** `hinfnorm` makes Day 60's tests exact and is the γ-test inside Day 66; `peakFrequency` tells you where a design is most fragile; the discrete branch uses Day 30's bilinear map in reverse. `sigmaMax` prepares the MIMO view for the stretch topics.

**Verify.** `hinfnorm({1},{1,1})` = `1.0 ± 1e-6`. `hinfnorm(E2)` = `1.0079` (`= (1/k)/(2ζ√(1−ζ²))`) at `ω = 1.968` — and a 200-point grid *underestimates* it by at least 0.1 % (assert both, to show why bisection exists). `hinfnorm(S)` for `L = 1/(s(s+1)(s+2))` equals Day 28's `M_s = 1.57` to `1e-3`. A `2×2` MIMO example (two coupled E1s): `hinfnorm` equals the dense-grid `σ_max` peak within `1e-4`. Discrete: `hinfnorm(c2d(E2, 0.01))` within `0.1 %` of the continuous value. The Day 60 delay-margin ordering test now uses the exact norm and the `±10 %` becomes `±2 %`.

Reading: Zhou, Doyle & Glover §4.3–4.4 (norms, bounded real lemma); Boyd & Balakrishnan (1990) and Bruinsma & Steinbuch (1990) for the algorithm; Skogestad & Postlethwaite §4.10.

**Done when.**
- [ ] `hinfnorm` (bisection, SISO/MIMO, discrete via bilinear), `sigmaMax`, tests including the grid-miss case
- [ ] You can explain what "an imaginary-axis eigenvalue of `H_γ`" means for the Bode plot
- [ ] Commit

---

#### Day 62 — Quadratic programming: KKT conditions, equality-constrained QP, Hildreth's dual method

**C++ you learn.**

- A **solver with diagnostics**: `QPResult { Vector x; Vector lambda; int iterations; bool converged; Real primalResidual; }`; convergence loops with relative tolerances; a hard `NumericalError` for infeasibility detected by dual blow-up.
- `Eigen::LDLT` for the symmetric indefinite KKT system; small dense problems where clarity beats cleverness.
- Tests against **hand-solvable** problems (geometry you can draw) and against the KKT conditions themselves (a solver-independent oracle: verify stationarity, feasibility and complementary slackness of the returned point).

**Control / optimisation you learn.** The optimisation problem inside MPC.

- **QP**: `min_x ½xᵀHx + fᵀx` s.t. `Ax ≤ b`, `A_eq x = b_eq`. Convex when `H ⪰ 0`; then any local minimum is global and the **KKT conditions** are necessary and sufficient: `Hx + f + Aᵀλ + A_eqᵀμ = 0` (stationarity), `Ax ≤ b`, `A_eq x = b_eq` (primal feasibility), `λ ≥ 0` (dual feasibility), `λᵢ(Ax − b)ᵢ = 0` (complementary slackness — a constraint is either active with `λᵢ ≥ 0` or inactive with `λᵢ = 0`).
- **Equality-constrained QP** = one linear system: `[[H, A_eqᵀ],[A_eq, 0]] [x; μ] = [−f; b_eq]`.
- **Inequalities — Hildreth's method** (dual coordinate ascent): the dual problem is `max_{λ ≥ 0} −½λᵀ(AH⁻¹Aᵀ)λ − λᵀ(AH⁻¹f + b)`; update one `λᵢ` at a time by its unconstrained maximiser projected onto `λᵢ ≥ 0`, sweep until the change is small; recover `x = −H⁻¹(f + Aᵀλ)`. Simple, needs `H ≻ 0`, converges linearly, handles infeasibility (λ diverges) detectably, and is a classic in the MPC literature (Wang, 2009). Active-set and interior-point methods are faster for large problems — Section 10 — and the seam `solveQP(H, f, A, b, A_eq, b_eq)` lets you swap one in.
- Why MPC produces QPs: a quadratic cost in the inputs plus linear constraints (bounds, rate limits, state limits through the linear prediction) — tomorrow.

**Files you touch.** `NumericalMethods.hpp/.cpp` (`solveQP`, `solveEqualityQP`, `kktResidual`), tests.

**Code.**

```cpp
struct QPResult { Vector x, lambda, mu; int iterations = 0; bool converged = false; Real maxViolation = 0.0; };
QPResult solveEqualityQP(const Matrix& H, const Vector& f, const Matrix& Aeq, const Vector& beq);
QPResult solveQP(const Matrix& H, const Vector& f, const Matrix& A, const Vector& b,
                 const Matrix& Aeq = Matrix(), const Vector& beq = Vector(),
                 Real tol = 1e-9, int maxIter = 5000, const Vector& lambda0 = Vector());   // warm start
Real     kktResidual(const Matrix& H, const Vector& f, const Matrix& A, const Vector& b, const QPResult& r);
```

```cpp
// Hildreth (inequalities only; equalities are eliminated beforehand or appended as two inequalities)
QPResult solveQP(const Matrix& H, const Vector& f, const Matrix& A, const Vector& b, const Matrix& Aeq, const Vector& beq, Real tol, int maxIter, const Vector& lambda0) {
    Eigen::LLT<Matrix> llt(H);
    if (llt.info() != Eigen::Success) throw InvalidArgument("solveQP: H must be positive definite");
    const Vector xu = -llt.solve(f);                      // unconstrained minimiser
    QPResult r; r.x = xu;
    if (A.rows() == 0) { r.converged = true; return r; }
    const Matrix HinvAt = llt.solve(A.transpose());
    const Matrix P = A * HinvAt;                          // dual Hessian
    const Vector d = A * xu - b;                          // dual gradient part (= A xu - b)
    Vector lam = (lambda0.size() == A.rows()) ? lambda0 : Vector::Zero(A.rows());
    for (int it = 0; it < maxIter; ++it) {
        Real change = 0.0;
        for (Index i = 0; i < A.rows(); ++i) {
            const Real w = d(i) + P.row(i).dot(lam) - P(i, i) * lam(i);   // gradient excluding own term
            const Real li = std::max(0.0, -w / P(i, i));
            change = std::max(change, std::abs(li - lam(i)));
            lam(i) = li;
        }
        r.iterations = it + 1;
        if (change < tol * (1.0 + lam.norm())) { r.converged = true; break; }
    }
    r.lambda = lam;
    r.x = xu - HinvAt * lam;
    r.maxViolation = (A * r.x - b).cwiseMax(0.0).maxCoeff();
    if (!r.converged && lam.norm() > 1e8) throw NumericalError("solveQP: dual diverged — problem is likely infeasible");
    return r;
}
```

**How it connects.** `solveQP` is called at every MPC step (Days 63–64) with a warm-started `λ`; `kktResidual` is the oracle in MPC's tests; the equality solver is reused for the KKT-based active-set stretch topic.

**Verify.** `min (x₁−1)² + (x₂−2)²` s.t. `x₁ + x₂ ≤ 2` → `(0.5, 1.5)`, `λ = 1`; with the constraint `x₁ + x₂ ≤ 5` (inactive) → `(1, 2)`, `λ = 0`. Box constraints `−1 ≤ x ≤ 1` on `min ½‖x − 3·1‖²` → all ones. `kktResidual < 1e-7` for a random `20×20` convex QP with 30 constraints (`H = MᵀM + I`, feasible by construction); the active set reported (`λ > 0`) matches the constraints that are tight. Equality QP: `min ½xᵀx` s.t. `x₁ + x₂ = 1` → `(0.5, 0.5)`. Infeasible (`x ≤ 0` and `x ≥ 1`) throws. Warm start from the previous solution of a slightly perturbed problem needs `< 20 %` of the cold-start iterations.

Reading: Boyd & Vandenberghe, *Convex Optimization* §4.4 (QP), §5.5 (KKT); Wang, *Model Predictive Control System Design and Implementation Using MATLAB* §2.4 (Hildreth); Nocedal & Wright §16 for the active-set method (stretch).

**Done when.**
- [ ] `solveQP` (Hildreth, warm start), equality QP, KKT residual, tests
- [ ] You can state the KKT conditions and explain complementary slackness with a picture
- [ ] Commit

---

#### Day 63 — Model predictive control: prediction matrices and the condensed QP

**C++ you learn.**

- **Block-Toeplitz construction** in loops (`Φ` lower block-triangular of `C Aᵏ B` terms, or in state form), a `Config` with many optional fields (`Vector` empty = no constraint), and a `Controller` that owns a *solver state* (the warm-start `λ`).
- Separating **formulation** (`buildQP` returns `H, f, A, b` — pure function, easy to test) from **execution** (`compute` calls `solveQP`).

**Control you learn.** Optimal control with constraints, solved online over a receding horizon.

- At each step: measure (or estimate) `x₀`; solve a finite-horizon problem over `N` steps for the input sequence `U = [u₀; …; u_{N−1}]`; apply only `u₀`; repeat next step with the new `x₀`. The **receding horizon** turns an open-loop optimisation into feedback.
- **Prediction**: `X = F x₀ + Φ U` with `F = [A; A²; …; A^N]`, `Φ_{ij} = A^{i−j} B` for `i ≥ j` (zero above the diagonal) — exact for the linear model (Day 29's ZOH model).
- **Cost** `J = Σ_{k=1}^{N} (x_k − x_ref)ᵀQ(x_k − x_ref) + Σ_{k=0}^{N−1} u_kᵀRu_k` with the last `Q` replaced by the **terminal weight** `Q_N`. Substituting the prediction: `J = ½UᵀHU + fᵀU + const` with `H = 2(ΦᵀQ̄Φ + R̄)`, `f = 2ΦᵀQ̄(Fx₀ − X_ref)`, `Q̄ = diag(Q, …, Q, Q_N)`, `R̄ = diag(R, …, R)`. **Constraints**: `u_min ≤ u_k ≤ u_max` → `[I; −I] U ≤ [u_max; −u_min]` stacked; rate limits `Δu` via a differencing matrix; state/output limits `x_min ≤ F x₀ + Φ U ≤ x_max` → rows of `±Φ`. All *linear* in `U`: it is a QP (Day 62).
- **Stability by design**: choose `Q_N = P` from the DARE (Day 45) — then the unconstrained MPC equals infinite-horizon LQR exactly, and with constraints the terminal cost acts as a Lyapunov function (under feasibility and a terminal constraint set — the full theory is in the reading). **Control horizon** `N_u < N` (hold `u` constant after `N_u` steps via a blocking matrix) reduces the QP size.
- **Feasibility**: constraints can make the QP infeasible (a disturbance pushed the state where no input keeps it inside the limits); soften output constraints with slack variables weighted heavily (Day 64).

**Files you touch.** `include/csl/control/MPC.hpp`, `src/control/MPC.cpp`, tests.

**Code.**

```cpp
class MPC final : public Controller {
public:
    struct Config {
        Index N = 20, Nu = -1;                  // prediction / control horizon (Nu < 0 => Nu = N)
        Matrix Q, R, QN;                        // QN empty => DARE solution (recommended)
        Vector uMin, uMax, duMin, duMax;        // empty => unconstrained
        Vector xMin, xMax;                      // state box (hard); Day 64 adds soft output constraints
        bool warmStart = true;
    };
    MPC(systems::StateSpace discretePlant, Config cfg);
    struct QP { Matrix H; Vector f; Matrix A; Vector b; };
    QP     buildQP(const Vector& x0, const Vector& xRef) const;        // pure; tests target this
    Vector compute(const Vector& x, const Vector& r, Real t) override;  // r = state reference (size n) or output reference mapped via Nx (Day 41)
    Matrix predictionF() const;  Matrix predictionPhi() const;
    const QPResult& lastSolve() const;
    ...
private:
    systems::StateSpace sys_; Config cfg_; Matrix F_, Phi_, Qbar_, Rbar_, Hqp_; Vector uPrev_; QPResult last_;
};
```

**How it connects.** The plant is `c2d` (Day 29); `QN` from `solveDARE` (Day 45); the QP from Day 62; the controller runs in the Simulator (Day 24) with the Kalman filter (Day 51) supplying `x` when needed. The unconstrained equivalence to `dlqr` (Day 45) is the first test; the constrained comparison with DP (Day 48) is tomorrow's.

**Verify.** `predictionF/Phi` reproduce `lsim(sys_d, U)` for a random `U` to `1e-12`. With `QN = P_DARE` and no constraints, `MPC::compute` returns `u₀ = −K_dlqr x` to `1e-8` for random `x` (`N = 5` suffices — the terminal cost makes the horizon exact). With `QN = Q` (no terminal weight) the gain differs and converges to `K_dlqr` as `N` grows (`N = 50`: within `1e-4`). `buildQP` with input bounds yields `A, b` such that `kktResidual < 1e-8` at the returned solution. Blocking with `Nu = 2` produces a `U` whose entries after the second are equal.

Reading: Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design* §1–2 (free PDF); Wang §1–3 (condensed form, very hands-on); Maciejowski, *Predictive Control with Constraints* §2–3.

**Done when.**
- [ ] `MPC` with `buildQP`, prediction matrices, terminal weight, blocking, tests
- [ ] You can write `F` and `Φ` for `N = 3` by hand and explain why `Q_N = P` matters
- [ ] Commit

---

#### Day 64 — MPC with constraints in closed loop: soft constraints, warm start, comparisons; `ex09`

**C++ you learn.**

- **Slack variables**: extend the decision vector `[U; ε]` and the matrices accordingly — a mechanical but error-prone extension; test it by checking that the solution with a huge slack weight equals the hard-constrained solution when feasible.
- Timing each solve (`std::chrono`), reporting mean/max solve time and iteration counts with and without warm start; a `MonteCarlo` run of MPC under noise.

**Control you learn.** MPC's practical side, and where it beats everything before it.

- **Soft output constraints**: `y_min − ε ≤ y ≤ y_max + ε`, `ε ≥ 0`, with a large linear+quadratic penalty on `ε`; keeps the QP feasible under disturbances while enforcing limits whenever possible. Input constraints stay hard (actuators cannot be pushed).
- **Warm start**: the previous solution shifted by one step is a good initial `λ`/`U`; cuts iterations several-fold.
- **Comparisons** (the point of the day): (a) double integrator with `|u| ≤ 1` — saturated LQR vs MPC vs the DP bang-bang policy (Day 48): MPC with a long horizon approaches the DP optimum; saturated LQR is slower because it does not *plan* the saturation. (b) Cart–pole with `|F| ≤ 5 N`: from an initial angle where saturated LQR fails, MPC on the linearised model recovers (and the EKF of Day 53 supplies the state). (c) Reference tracking with state constraints: the MPC keeps `p` inside `[−0.5, 0.5]` while balancing — no classical controller does this cleanly.
- Tuning knobs: `N` (longer is better until the solve time bites), `Q/R` (as LQR), constraint softness, `T_s` (solve time must fit in `T_s`).

**Files you touch.** `MPC.hpp/.cpp` (soft constraints, timing), `examples/ex09_mpc.cpp`, tests.

**`ex09_mpc.cpp`.** The three comparisons above with plots of `x`, `u`, constraint lines, and a table: settling time, cost, max `|u|`, mean/max solve time, iterations (cold vs warm). Monte Carlo (20 runs) of the cart–pole MPC under measurement noise with the EKF.

**Verify.** Double integrator, `|u| ≤ 1`, `x₀ = [2, 0]`, `T_s = 0.1`: MPC (`N = 40`, `Q_N = P`) reaches `‖x‖ < 0.05` in at most 10 % more steps than DP's bang-bang policy and at least 20 % fewer than saturated LQR (fixed numbers once measured — record them). `|u| ≤ 1 + 1e-9` at every step (hard). Soft state constraint `|x₁| ≤ 1.5` from `x₀ = [2, 0]`: violation only at `k = 0`–few steps and `ε → 0` afterwards; the same constraint hard makes the QP throw at `k = 0` (infeasible start) — assert the throw. Cart–pole from `θ₀ = 0.25 rad` with `|F| ≤ 5`: MPC balances (`|θ| < 0.01` at `t = 5 s`), saturated LQR with the same weights does not. Warm start reduces mean iterations by at least 50 %.

Reading: Rawlings, Mayne & Diehl §2.4–2.5 (stability, terminal sets), §1.5 (soft constraints); Maciejowski §3.4.

**Done when.**
- [ ] Soft constraints, warm start, timing; `ex09` with three comparisons and the table; tests
- [ ] You can explain why MPC outperforms saturated LQR under constraints
- [ ] Commit

---

#### Day 65 — Iterative LQR for nonlinear trajectory optimisation: the cart–pole swing-up

**C++ you learn.**

- A larger algorithm from library pieces: `std::vector<Matrix>` for per-step Jacobians and gains, a backward pass with regularisation (`μ I` added when a Hessian is not positive definite — check with `LLT`), a forward pass with **backtracking line search** over `α ∈ {1, ½, ¼, …}`, and a convergence loop on the cost. Each piece is small; the discipline is in the bookkeeping (indices `k = 0…N`, sizes).
- Numerical Jacobians at every step (slow but correct); note the cost and the obvious caching.

**Control you learn.** Optimal control for nonlinear systems by repeated local LQR.

- Problem: `min Σ_{k=0}^{N−1} l(x_k, u_k) + l_N(x_N)` s.t. `x_{k+1} = f(x_k, u_k)`. **iLQR**: (1) roll out a nominal trajectory from a guess `U`; (2) linearise the dynamics along it (`A_k, B_k` — Day 47's Jacobian of the discrete map) and quadratise the cost; (3) **backward pass**: a Riccati-like recursion (Day 45) producing feedforward `k_k` and feedback `K_k`; (4) **forward pass**: `u_k = ū_k + α k_k + K_k (x_k − x̄_k)` rolled through the *nonlinear* dynamics with the largest `α` that decreases the cost; (5) repeat until the cost stops improving. DDP additionally keeps second-order dynamics terms; iLQR drops them (cheaper, nearly as good).
- Result: an open-loop optimal trajectory **plus** a time-varying feedback law `K_k` around it — the natural combination with MPC (iLQR as the solver of a nonlinear MPC, stretch) and with the EKF (tracking the trajectory under noise).
- Task: **cart–pole swing-up** from hanging (`θ = π`) to upright (`θ = 0`) in 3 s with `|F| ≤ 10` (penalised quadratically, or clamped in the rollout), cost `Σ (x − x_goal)ᵀQ(x − x_goal) + uᵀRu + terminal`. The solution swings back and forth to pump energy — qualitatively what a human does, found by the algorithm with no hint.

**Files you touch.** `include/csl/control/iLQR.hpp`, `src/control/iLQR.cpp`, tests, `ex06` (add a swing-up section) or `ex10`.

**Code.**

```cpp
struct iLQRProblem {
    std::function<Vector(const Vector&, const Vector&)> f;           // discrete dynamics
    std::function<Real(const Vector&, const Vector&)> l;             // stage cost
    std::function<Real(const Vector&)> lN;                           // terminal cost
    Index n, m, N;  Real Ts;
};
struct iLQRResult { std::vector<Vector> x, u; std::vector<Matrix> K; std::vector<Real> costHistory; bool converged; int iterations; };
iLQRResult ilqr(const iLQRProblem& p, const Vector& x0, const std::vector<Vector>& uInit,
                int maxIter = 100, Real tol = 1e-6, Real muInit = 1e-6);
```

**How it connects.** Dynamics from `NonlinearSystem` + `rk4Step` (Days 7, 47); Jacobians from `numericalJacobian` (Day 47); the backward pass is `finiteHorizonLQR` with a feedforward term (Day 45); the trajectory can be tracked in the Simulator with `K_k` and the EKF (Day 53). Quadratic cost + linear dynamics must reduce to Day 45 exactly — the first test.

**Verify.** Linear plant (discrete double integrator) with quadratic cost: `ilqr` converges in ≤ 2 iterations and `u` equals `finiteHorizonLQR`'s rollout to `1e-8`. Cart–pole swing-up (`T_s = 0.02`, `N = 150`, `Q = diag(1, 0.1, 10, 0.1)`, `R = 0.01`, terminal `100·Q`): final `|θ_N| < 0.05 rad`, `|p_N| < 0.2 m`, cost strictly decreasing over iterations, converged in `< 60` iterations; the trajectory contains at least one swing reversal (sign change of `θ̇` before reaching upright). Tracking the result in the Simulator with `K_k` from a perturbed initial state (`θ₀ = π + 0.05`) still reaches `|θ_N| < 0.1`.

Reading: Tassa, Erez & Todorov (2012), "Synthesis and stabilization of complex behaviors through online trajectory optimization" (iLQR/DDP with the regularisation and line search used here); Li & Todorov (2004); Jacobson & Mayne for DDP.

**Done when.**
- [ ] `ilqr` with regularisation and line search; linear equivalence test; swing-up test
- [ ] You can explain the backward pass as "LQR around a trajectory" and why the line search is needed
- [ ] Commit

---

#### Day 66 — Capstone: H∞ synthesis (two-Riccati central controller) and mixed sensitivity

**C++ you learn.** The largest single function in the library, assembled from `solveARE`, `series/parallel/feedback`, block matrices, `hinfnorm` and `bisection`. The lesson is engineering: break it into testable pieces (`generalizedPlant`, `hinfsynFixedGamma`, `gammaIterate`), check every assumption with an exception that *names* it, and verify the result with an independent computation (`hinfnorm` of the closed loop).

**Control you learn.** Where robust control's theory meets code.

- **Generalised plant** `P`: inputs `w` (disturbances/references/noise) and `u` (control); outputs `z` (weighted performance signals) and `y` (measurements). The closed loop `T_zw = F_l(P, K)` (lower LFT). **H∞ problem**: find stabilising `K` with `‖T_zw‖∞ < γ`, then push `γ` down (γ-iteration by bisection). Day 60's inequalities `‖W_s S‖∞ < 1`, `‖W_m T‖∞ < 1` become `z = [W_s S w; W_m T w]` — the **mixed-sensitivity** problem `‖[W_s S; W_u KS; W_m T]‖∞ < γ`.
- **DGKF solution** (Doyle, Glover, Khargonekar, Francis 1989) under the simplifying assumptions `D₁₁ = 0`, `D₂₂ = 0`, `D₁₂ᵀ[C₁ D₁₂] = [0 I]`, `D₂₁[B₁ᵀ D₂₁ᵀ] = [0 I]` (arrange the weights so these hold — scaling and the general case are in the references):

```
X∞ = Ric( [ A,               γ⁻²B₁B₁ᵀ − B₂B₂ᵀ ] )          Y∞ = Ric( [ Aᵀ,              γ⁻²C₁ᵀC₁ − C₂ᵀC₂ ] )
          [ −C₁ᵀC₁,           −Aᵀ              ]                     [ −B₁B₁ᵀ,           −A               ]
```

(i.e. `solveARE(A, S = B₂B₂ᵀ − γ⁻²B₁B₁ᵀ, Q = C₁ᵀC₁)` and its dual). A solution exists iff `X∞ ⪰ 0`, `Y∞ ⪰ 0`, and `ρ(X∞Y∞) < γ²`. The **central controller**:

```
F = −B₂ᵀX∞,   L = −Y∞C₂ᵀ,   Z = (I − γ⁻²Y∞X∞)⁻¹
Â = A + γ⁻²B₁B₁ᵀX∞ + B₂F + Z L C₂
K:  x̂̇ = Â x̂ − Z L y,    u = F x̂
```

As `γ → ∞` the two Riccati equations become the LQR and Kalman AREs and `K` becomes the LQG controller — H∞ is LQG with a worst-case adversary; verify this limit numerically.
- Honest scope: this handles the "regular" problem; singular problems, `D₁₁ ≠ 0`, and MIMO weight selection need the loop-shifting/scaling of the references and are stretch topics. What you build is enough to design a robust SISO loop from `W_s`, `W_u`, `W_m` and to *see* H∞ make the trade-offs of Days 28 and 60 automatically.

**Files you touch.** `include/csl/control/HInfinity.hpp`, `src/control/HInfinity.cpp`, tests, `ex10` (section).

**Code.**

```cpp
namespace csl::control {
struct GeneralizedPlant { systems::StateSpace P; Index nw, nu, nz, ny; };
GeneralizedPlant mixedSensitivityPlant(const systems::StateSpace& G, const systems::TransferFunction& Ws,
                                       const systems::TransferFunction& Wu, const systems::TransferFunction& Wt);   // z = [Ws S; Wu KS; Wt T] w
struct HInfResult { systems::StateSpace K; Real gamma; Matrix Xinf, Yinf; bool ok; std::string message; };
HInfResult hinfsynFixedGamma(const GeneralizedPlant& P, Real gamma);         // DGKF central controller or ok=false with the failed condition
HInfResult hinfsyn(const GeneralizedPlant& P, Real gammaLo = 0.0, Real gammaHi = 1e3, Real tol = 1e-3);   // bisection
systems::StateSpace closedLoop(const GeneralizedPlant& P, const systems::StateSpace& K);                  // T_zw for hinfnorm
}
```

**How it connects.** `solveARE` (Day 44) twice; `hinfnorm` (Day 61) as the independent check; `series/parallel/feedback` (Day 19) to build `P` and `T_zw`; `lqr` + `steadyStateKalman` (Days 44, 52) for the `γ → ∞` limit; `margins`/`gangOfFour` (Days 18, 28) to inspect the result classically.

**Verify.** Plant `G = 1/(s − 1)` (unstable), `W_s = (s/2 + 1)/(s + 0.01)`·0.5 (tracking below ~1 rad/s), `W_u = 0.1`, `W_t = (s + 1)/(s/10 + 1)`·0.1 (roll-off above 10 rad/s) (adjust so the DGKF assumptions hold — the test documents the scaling): `hinfsyn` returns `γ* ∈ [0.9, 1.2]`… lock the number once computed; `hinfnorm(closedLoop(P, K)) ≤ γ* (1 + 1e-3)`; `K` stabilises `G` (`feedback(series(K, G)).isStable()`); the gang of four shows `|S| ≤ γ*/|W_s|` and `|T| ≤ γ*/|W_t|` on a grid. For `γ = 1e4`, `K`'s gains are within `1 %` of the LQG controller built from `lqr(A, B₂, C₁ᵀC₁, I)` and `steadyStateKalman(…, B₁B₁ᵀ, I)` (continuous version — add `solveCARE` dual). `hinfsynFixedGamma` below `γ*` returns `ok = false` with a message naming the failed condition (`ρ(XY) ≥ γ²` or an indefinite `X`).

Reading: Doyle, Glover, Khargonekar & Francis (1989) — the paper; Zhou, Doyle & Glover §14 (and §16 for the general case); Skogestad & Postlethwaite §9.3 (mixed sensitivity, weight selection — practical).

**Done when.**
- [ ] `mixedSensitivityPlant`, `hinfsynFixedGamma`, `hinfsyn`, `closedLoop`, tests including the LQG limit
- [ ] You can explain in words what each of `X∞`, `Y∞`, `ρ(X∞Y∞) < γ²` means
- [ ] Commit `Day 66: Phase 10 complete`

---
### Phase 11 — Packaging, documentation, release (Days 67–70)

**Phase goal.** The library is something another person can build, install, link against and understand. In C++ terms: CMake install/export and package config, a consumer project, continuous integration, formatting, documentation, and a final end-to-end example.

---

#### Day 67 — Installing and exporting the library (`find_package(csl)`)

**C++/CMake you learn.** How a library is *consumed*, which is the other half of how it is built.

- `install(TARGETS csl EXPORT cslTargets ...)` copies the compiled library; `install(DIRECTORY include/ ...)` copies headers; `install(EXPORT cslTargets NAMESPACE csl:: ...)` writes a CMake file describing the target *with its usage requirements* (include dirs, C++17, the Eigen dependency). A `cslConfig.cmake.in` + `write_basic_package_version_file` make `find_package(csl 2.0 REQUIRED)` work in another project.
- The `$<BUILD_INTERFACE:...>`/`$<INSTALL_INTERFACE:...>` generator expressions you wrote on Day 4 finally mean something: build-tree include path vs installed include path.
- Eigen as a dependency of an installed library: either `find_dependency(Eigen3)` in the config (requires the consumer to have Eigen) or install the fetched Eigen headers alongside (`install(DIRECTORY ${eigen_SOURCE_DIR}/Eigen ...)`) — choose the second for a self-contained package and document it.
- A **consumer test**: `tests/consumer/CMakeLists.txt` with a 10-line program that `find_package(csl)`s the installed tree and runs `step(E1)`; a script `scripts/test_install.ps1` that configures, builds, installs to a temp prefix and builds the consumer. If this passes, the package works.

**Files you touch.** Root `CMakeLists.txt` (install rules, `CMakePackageConfigHelpers`), `cmake/cslConfig.cmake.in`, `tests/consumer/`, `scripts/test_install.ps1` (and `.sh`).

**Verify.** `cmake --install build --prefix C:\tmp\csl-install` produces `lib/csl.lib`, `include/csl/...`, `include/Eigen/...`, `lib/cmake/csl/cslConfig.cmake`; the consumer project builds and runs against it with no reference to the source tree. `find_package(csl 3.0)` fails with a clear version message.

**Done when.**
- [ ] Install/export, package config, consumer test script passing
- [ ] You can explain what `install(EXPORT)` writes and why `PUBLIC` on Day 4 matters now
- [ ] Commit

---

#### Day 68 — Documentation: README, API reference, tutorials

**What you write.**

1. `README.md`: what the library is (one paragraph), the module table (Section 4.2), a 15-line quick start (build, run `ex01`, run tests), a "gallery" of 6 plots (commit the HTMLs' PNG screenshots under `docs/img/` — the only images in the repo), the build-vs-buy principle in three sentences, and a link to this document.
2. **API reference**: add Doxygen-style comments (`/// \brief ...`) to every public header — one line per function saying what it computes, the units and conventions (rad/s, `Ts == 0`, time-along-rows), and what it throws. Optionally run Doxygen (`docs/Doxyfile`) to generate HTML; the comments are the deliverable, the HTML is a bonus.
3. `docs/tutorials/`: five short pages, each a worked example you have already written, now explained for a reader: *From ODE to Bode in 20 lines*, *Tuning a PID three ways*, *Digital control: what sampling costs*, *LQR → LQG → MPC on the cart–pole*, *Identifying a model from data*.
4. A **naming and consistency audit**: every function obeys Section 5.3; every `Ts` is a `Real` in seconds; every frequency parameter says `w` (rad/s) or `_hz`; every result struct is one of the Section 4.3.4 types. Fix inconsistencies now — this is the last cheap moment.

**Verify.** A colleague (or you, after a week away) can build and run the quick start from the README alone. Every public function has a doc comment (grep for `^[A-Za-z].*\(` in headers without a preceding `///` — write the script).

**Done when.**
- [ ] README with gallery, doc comments on every public header, five tutorials, audit done
- [ ] Commit

---

#### Day 69 — Continuous integration, formatting, static analysis

**What you set up.**

- `.github/workflows/ci.yml`: two jobs — `windows-latest` (MSVC, Ninja, Debug + Release, `ctest`) and `ubuntu-latest` (GCC, `-Wall -Wextra -Werror`, Debug with `-fsanitize=address,undefined`, Release, `ctest`). CMake's `FetchContent` works unchanged in CI (cache `build/_deps` with `actions/cache` to save minutes). Add the install/consumer script as a third step.
- `.clang-format` (e.g. based on `Google` or `LLVM` style, 4-space indent, 110 columns) and a one-time `clang-format -i` over the tree; a CI check that formatting is clean.
- Compiler warnings as errors on the library and tests (not on the examples' `plants.hpp` copies if any third-party noise appears).
- Optional: `clang-tidy` with a small checks list (`bugprone-*`, `performance-*`, `modernize-use-override`) — read the output once, fix what is real.
- `CHANGELOG.md` with `2.0.0` and a `git tag v2.0.0` on Day 70.

**Verify.** Both CI jobs green on a pushed branch; the ASan/UBSan job finds nothing (if it finds something, that is a real bug — fix it and write the test that would have caught it).

**Done when.**
- [ ] CI green on Windows and Linux, formatting enforced, sanitizers clean
- [ ] Commit

---

#### Day 70 — `ex10_full_pipeline` and the retrospective

**`ex10_full_pipeline.cpp`** — one program that uses every module, on E3 (DC motor) with the cart–pole as the nonlinear finale:

1. **Model** E3 as `StateSpace`; convert to TF; print poles, dc gain, `damp`; Bode and step plots. *(systems, analysis, simulation, plot)*
2. **Experiment**: discretize with ZOH at 1 ms; excite with PRBS; add measurement noise; `lsim` the data. *(conversions, signals, simulation)*
3. **Identify**: ARX, ERA and ETFE+rational fit; validate; overlay the three Bode plots on the truth; pick the best by validation fit. *(sysid, dsp, analysis, plot)*
4. **Classical design** on the identified model: SIMC PI; margins, `M_s`, gang of four; Simulator with saturation and anti-windup; discretize the controller with Tustin and measure the PM loss. *(control, analysis, simulation)*
5. **Modern design**: LQR with Bryson weights; steady-state KF; LQG; margins of the LQG loop; Monte Carlo under noise. *(control, estimation, simulation)*
6. **Constrained design**: MPC with voltage limits and a current limit (state constraint); comparison table with LQG. *(control/MPC, NumericalMethods)*
7. **Robustness**: multiplicative uncertainty from the spread of the three identified models; small-gain check and `hinfnorm`; an H∞ mixed-sensitivity controller and its margins. *(analysis/Robustness, Norms, control/HInfinity)*
8. **Nonlinear finale**: cart–pole swing-up with iLQR, then balancing with LQR on an EKF estimate, with MPC taking over when the force limit binds. *(NonlinearSystem, iLQR, EKF, MPC)*
9. **Spectral check**: Welch PSD of the closed-loop residuals, whiteness test. *(dsp)*
10. Everything written to `docs/img/` as HTML and to CSV.

If this program compiles, runs and its numbers agree with each other, the library is cohesive — which was the design goal from Day 1.

**Retrospective (`docs/RETROSPECTIVE.md`, one page each).**
- C++: list the ten concepts you found hardest and, for each, the day it clicked and the piece of library code that made it click.
- Control: the same for ten control concepts.
- Library: what you would change in the type contract (Section 4.3) knowing what you know now; what you would *not* hand-write again; what you would hand-write that this plan delegated.
- Next: pick three items from Section 10 and sketch their day plans in the format of this document.

**Done when.**
- [ ] `ex10` runs end to end; retrospective written; `git tag v2.0.0`
- [ ] Commit `Day 70: v2.0.0`

---

## 8. Testing strategy

**Principles.**
1. Every day adds tests; the suite never shrinks. By Day 70 expect roughly 350–450 `TEST`s across ~45 test files mirroring `include/csl`.
2. Tests target the **hand-written layer only**. There are no tests that Eigen inverts a matrix correctly; there are many that `solveARE` returns a stabilising `P`, that `fft` matches `dft`, that `c2d` is exact for a step, that `arx` recovers a known model.
3. Every test has a **known answer**: a closed form (poles of E2), a textbook value (LQR gain `[1, √3]`), a cross-representation identity (`ss.evaluate == tf.evaluate`), an invariant (`S + T = 1`, Parseval, KKT residual, ARE residual), a convergence law (RK4 error ratio 16), or an independent computation (bisection vs grid, ERA vs ARX). "It ran without crashing" is not a test.
4. **Cross-checks across days** are the most valuable tests in the library (the "6" of `1/(s(s+1)(s+2))` from margins, root locus and Routh; the LQR cost vs simulated integral; the KF covariance vs DARE; MPC vs dlqr). Write them deliberately.
5. **Stochastic tests** use fixed seeds and assert *bounds* with margin (RMSE < x, mean NIS in [a, b]). If a bound is flaky across platforms, loosen the bound and note why — never remove the seed.
6. Tolerances follow Section 5.4. A test that needs a looser tolerance than the policy says is a sign of a numerical problem to investigate first.

**Oracle types used, by module.**

| Module | Oracles |
|---|---|
| core | closed-form polynomial algebra; companion roots vs known roots; `kron`/`vec` identities; ARE/DARE/Lyapunov residuals; KKT residuals; hand-solved QPs |
| systems | cross-representation `evaluate`; `ss2tf(tf2ss(H)) == H`; similarity invariance; interconnection identities; `c2d` exact for steps; `d2c(c2d) == id` |
| simulation | analytic responses (E1 exponential, E2 damped sine); order-of-accuracy ratios; specs vs second-order formulas; Simulator vs TF-world closed loop |
| analysis | textbook margins; Nyquist/root-locus/Routh agreement; `S + T = 1`; sensitivity integral; `hinfnorm` vs grid peak and vs closed form; small-gain vs delay margin ordering |
| control | PID algebra; tuning-rule tables; LQR known gain and cost; LQR margin bounds; separation-principle pole sets; MPC = dlqr unconstrained; iLQR = LQR on linear problems; H∞ → LQG limit; `hinfnorm(T_zw) ≤ γ` |
| estimation | KF vs DARE; EKF = KF on linear plants; UT exactness on linear and `x²` maps; RMSE/NIS bounds with seeds |
| dsp | `fft` vs `dft`; impulse/constant spectra; Parseval; window tables; PSD level of white noise; filter responses at known frequencies; `filter.process(step) == step(filter.toTF())` |
| sysid | exact recovery on noise-free data; bounded error with noise; RLS = batch; RLS = KF; ERA singular-value gap; validation whiteness on correct vs under-fitted models |
| signals / plot | statistics with seeds; PRBS period/balance/autocorrelation; SVG structure; CSV round-trip |

**CI** (Day 69) runs the whole suite on Windows/MSVC and Linux/GCC, Debug with sanitizers and Release.

---

## 9. Milestone snapshots

| After | You can… | Library has… |
|---|---|---|
| Day 10 | compile/link/debug multi-file C++, drive CMake, write tests; simulate any linear ODE with RK4; compute and interpret poles | `Types`, `Exceptions`, `ODESolver`, `SimResult`, `StateSpace` (poles, dc gain), `CSV`, Eigen + GTest wired, `ex01` |
| Day 16 | use classes, operators, inheritance, file I/O, STL algorithms; move between ODE/SS/TF/ZPK; read a step response quantitatively | `Polynomial`, `TransferFunction`, `LTISystem`, `Conversions`, `Figure`, `Plots`, `TimeResponse` + specs, `Signals` |
| Day 22 | reason in the frequency domain; use `optional`, overload sets, sorting predicates; organise headers | `MathUtils`, `FrequencyResponse` (Bode, Nyquist, margins), `Interconnect` + gang of four, `pade`, `RootLocus`, `Stability` (Routh), `csl.hpp`, `ex02` |
| Day 28 | design classical controllers and verify them in simulation; manage ownership with smart pointers | `Controller`, `PID` (practical), `Simulator`, `Tuning` (ZN, SIMC, relay), `LeadLag`, `NumericalMethods` (bisection), sensitivity tools, `ex03` |
| Day 36 | work in discrete time; write an FFT; design and run digital filters; judge sampling's cost | `c2d`/`d2c` (5 methods), `TransferFunctionController`, `FFT`, `Spectral` (Welch, csd), `DigitalFilter` (IIR/FIR), `FilterDesign` (Butterworth, FIR, notch), `ex04` |
| Day 43 | design state-space controllers and observers; solve Lyapunov equations; explain copy/move | `LinearAlgebra` (ctrb/obsv, Lyapunov, Gramians), `StateFeedback` (place, integral), `Estimator`, `LuenbergerObserver`, `Norms` (H2), `ex05` |
| Day 49 | derive and implement LQR (both domains); linearise nonlinear systems; relate DP, LQR and MPC | `solveARE`/`solveDARE`, `LQR` (+ finite horizon, integral, margins), `NonlinearSystem`, `DynamicProgramming`, cart–pole, `ex06` |
| Day 55 | model noise; implement KF/EKF/UKF; build LQG and judge its margins; run Monte Carlo | `WhiteNoise`, `PRBS`, `MonteCarlo`, `KalmanFilter` (+ steady state), `LQG`, `EKF`, `UKF`, `ex07` |
| Day 59 | identify models from time- and frequency-domain data and validate them | `ARX`, `RLS`, `etfe` + `fitRational`, `Subspace` (ERA), `Validation`, `ex08` |
| Day 66 | specify and check robustness; solve QPs; implement MPC and iLQR; synthesise an H∞ controller | `Robustness`, `hinfnorm`, `solveQP`, `MPC`, `iLQR`, `HInfinity`, `ex09` |
| Day 70 | ship a C++ library | install/export, consumer test, CI, docs, `ex10`, `v2.0.0` |

---

## 10. Stretch topics (after Day 70)

Each is a 1–3 day addition in the format of Section 7. They are ordered by how much they reuse what exists.

| Topic | Builds on | What you would write |
|---|---|---|
| Adaptive RK45 (Dormand–Prince) with error control | Day 7 | `rk45Step`, `integrateAdaptive` with step rejection; test vs RK4 at tight tolerance |
| Bartels–Stewart Lyapunov solver | Day 40 | `Eigen::RealSchur` + quasi-triangular back-substitution; scales to `n` in the hundreds |
| Ordered-Schur ARE solver (Laub) | Day 44 | ordering of the real Schur form (Eigen lacks it — write Givens-based swapping); more robust than eigenvectors |
| DARE by doubling / symplectic pencil | Day 45 | quadratic convergence; MIMO-friendly |
| MIMO transfer-function matrices and transmission zeros | Days 12–14 | `TransferFunctionMatrix`; zeros via the generalised eigenproblem for non-square systems |
| MIMO pole placement (Kautsky–Nichols) | Day 38 | robust eigenstructure assignment |
| Balanced realisation and truncation | Days 40, 42 | model reduction with Hankel-singular-value error bounds |
| Active-set and interior-point QP; OSQP seam | Day 62 | replace Hildreth behind `solveQP`; benchmark on MPC problems |
| Nonlinear MPC with iLQR as the inner solver | Days 63–65 | real-time iteration scheme |
| Particle filter | Days 50, 54 | sequential importance resampling; compare with UKF on a multimodal problem |
| Loop transfer recovery (LTR) | Day 52 | recover LQR margins from LQG by `Q_w` scaling; plot the convergence |
| μ-analysis (structured singular value bounds) | Days 60–61 | D-scaling upper bound for diagonal uncertainty |
| H∞ general case (loop shifting, scaling), H∞ loop shaping (McFarlane–Glover) | Day 66 | the practical MIMO design method |
| ARMAX, output-error, instrumental variables, prediction-error method | Day 56 | unbiased identification under coloured noise; Gauss–Newton |
| N4SID / MOESP | Day 59 | subspace identification from input–output data directly (oblique projection, SVD) |
| Chebyshev, elliptic, Bessel filters; band-pass/stop transforms; Parks–McClellan FIR | Day 34 | the rest of the filter-design toolbox |
| Describing functions and limit cycles | Days 26, 47 | relay feedback explained properly; Nyquist with a nonlinearity |
| Indirect adaptive control | Days 57, 38 | RLS + online pole placement (self-tuning regulator) |
| Python bindings (pybind11) | Day 67 | zero-copy Eigen ↔ NumPy; the library from a notebook |
| Real-time: fixed-size Eigen types, no allocation in `compute`, a `Controller` on an MCU | Day 24 | profile and remove heap use from the hot path |

---

## 11. Glossary

**C++ and tooling**

| Term | Meaning |
|---|---|
| Translation unit | One `.cpp` after preprocessing; the unit the compiler compiles |
| Object file | Compiled machine code for one translation unit plus its symbol table |
| Linker | Joins object files and libraries into an executable by resolving symbol references |
| Static library | An archive of object files (`.lib`/`.a`) linked into the executable |
| Header / include guard | Declarations shared between translation units / `#pragma once` to prevent double inclusion |
| ODR | One Definition Rule: define each function/class once across the program (`inline` relaxes it for headers) |
| Target (CMake) | A buildable thing (executable or library) carrying its own usage requirements |
| `PUBLIC` / `PRIVATE` / `INTERFACE` | Who inherits a target's property: me and my users / only me / only my users |
| Value / reference / pointer | A copy / an alias to an existing object / an address that may be null |
| `const` / `constexpr` | Cannot change after initialisation / computed at compile time |
| Stack / heap | Automatic storage freed on scope exit / dynamic storage you (or a smart pointer) free |
| RAII | Resource Acquisition Is Initialization: constructors acquire, destructors release |
| `unique_ptr` / `shared_ptr` | Sole-owner / reference-counted owning pointers to heap objects |
| Class / struct / invariant | A type with data and functions / the same with public default / what is always true of a valid object |
| Constructor / destructor | Runs at creation (establishes the invariant) / at destruction (releases resources) |
| Special member functions | Default ctor, dtor, copy ctor/assign, move ctor/assign; Rule of Zero: declare none if members manage themselves |
| Move semantics / rvalue | Transferring a resource from an expiring object / an expression that denotes such an object |
| Operator overloading | Giving `+`, `*`, `()`, `<<`… meaning for your types |
| Inheritance / virtual / override / abstract | "is-a" relation / run-time dispatch / checked overriding / a class with a pure virtual (`= 0`) function |
| Polymorphism | Calling through a base reference and getting the derived behaviour |
| Template | A recipe the compiler instantiates per type |
| Lambda / `std::function` / functor | Anonymous function with captures / type-erased callable holder / object with `operator()` |
| Iterator / range / algorithm | Position in a container / `[begin, end)` / a function over a range (`sort`, `max_element`…) |
| `std::optional` | A value that may be absent, without sentinels |
| Exception | An error object thrown up the stack until caught |
| `enum class` | A scoped set of named constants |
| Namespace | A named scope for avoiding collisions (`csl::analysis`) |
| Expression template (Eigen) | `A*x + b` builds a lazy expression evaluated in one loop, no temporaries |
| Unit test / fixture / assertion | A function checking one property / shared setup / `EXPECT_NEAR` etc. |
| Sanitizer | A compiler instrumentation that detects memory/UB errors at run time |

**Control and signal processing**

| Term | Meaning |
|---|---|
| State / input / output | Minimal memory of the system / what you manipulate / what you measure |
| LTI | Linear, time-invariant: superposition holds, parameters constant |
| State space `(A, B, C, D)` | `ẋ = Ax + Bu`, `y = Cx + Du` |
| Transfer function | Ratio of Laplace (or z) transforms of output and input, zero initial conditions |
| Poles / zeros | Roots of the denominator (modes) / numerator (what blocks or shapes modes) |
| Stability (continuous / discrete) | All poles in the open left half-plane / inside the unit circle |
| Time constant / `ωn` / `ζ` | 63 % time of a first-order mode / natural frequency / damping ratio of a second-order mode |
| dc gain | Steady-state output per unit constant input, `H(0)` or `H(1)` |
| Step / impulse response | Output for a unit step / unit impulse input from rest |
| Rise, peak, settling time, overshoot | Time-domain specs (Day 16) |
| Frequency response | `H(jω)`: gain and phase at each sinusoidal frequency |
| Bode / Nyquist / Nichols | Magnitude-and-phase vs log ω / `H(jω)` in the complex plane / magnitude vs phase |
| Gain margin / phase margin / delay margin | Gain, phase or delay increase that just destabilises the loop |
| Loop `L` / sensitivity `S` / complementary sensitivity `T` | `CP` / `1/(1+L)` / `L/(1+L)`; `S + T = 1` |
| Bandwidth | Frequency where `|T|` falls to −3 dB (or `|S|` rises to −3 dB) |
| Minimum phase / NMP / all-pass | No RHP zeros / RHP zeros present / unit magnitude, pure phase |
| Root locus | Closed-loop poles as a function of a gain |
| Routh–Hurwitz | Stability from polynomial coefficients, no root finding |
| PID / anti-windup / setpoint weighting | Proportional-integral-derivative control / preventing integrator run-away under saturation / 2-DOF reference shaping |
| Lead / lag | Phase-adding / low-frequency-gain-adding first-order compensators |
| Sampling / ZOH / aliasing / Nyquist frequency | Reading `y` every `T_s` / holding `u` between samples / frequency folding / `f_s/2` |
| z-transform / Tustin / MPZ | Discrete-time transform / bilinear `s→z` map / matched pole–zero map |
| DFT / FFT / PSD / window / leakage | Discrete Fourier transform / its fast algorithm / power per Hz / taper / spectral smearing |
| FIR / IIR / linear phase / group delay | Non-recursive / recursive filters / constant delay for all frequencies / `−dφ/dω` |
| Butterworth / notch | Maximally flat lowpass / narrow band-stop |
| Controllable / observable / stabilizable / detectable | Can steer every state / can infer every state / unstable modes controllable / unstable modes observable |
| Pole placement / Ackermann | Choosing `K` for given closed-loop poles / the SISO formula |
| Observer / separation principle | State estimator from `y` / controller and observer poles add, designed independently |
| Lyapunov equation / function / Gramian | `AᵀP + PA + Q = 0` / `xᵀPx` energy / `W_c`, `W_o` measures of controllability/observability |
| LQR / Riccati equation (ARE, DARE) | Optimal state feedback for a quadratic cost / the matrix equation giving `P` |
| Bryson's rule | Weight choice `1/max²` |
| Linearization / equilibrium | Jacobians at a steady state / `f(x_e, u_e) = 0` |
| Dynamic programming / Bellman equation | Optimal control by backward recursion / `V(x) = min_u[l + V(f)]` |
| White noise / covariance / process and measurement noise | Uncorrelated flat-spectrum signal / `E[xxᵀ]` / `w` in dynamics, `v` in output |
| Kalman filter / innovation / NIS | Optimal linear estimator / `y − Cx̂⁻` / normalised innovation squared (consistency) |
| LQG / duality | LQR on KF estimate / LQR ↔ KF correspondence |
| EKF / UKF / sigma points | Linearised KF / derivative-free KF / the UKF's sample points |
| ARX / RLS / forgetting factor / persistency of excitation | Equation-error model / online LS / memory of RLS / input richness |
| ETFE / coherence / Markov parameters / ERA / Hankel matrix | FRF from data / trustworthiness / impulse response / subspace realisation / its data matrix |
| Multiplicative uncertainty / small-gain / robust stability / performance | `P(1 + W_mΔ)` / `‖M‖∞ < 1` / `‖W_mT‖∞ < 1` / `‖W_sS‖∞ < 1` |
| H2 norm / H∞ norm | Impulse-energy (white-noise) gain / worst-case sinusoidal gain |
| QP / KKT / Hildreth | Quadratic program / optimality conditions / dual coordinate ascent |
| MPC / receding horizon / terminal cost | Online constrained optimal control / apply first input, repeat / `Q_N = P` for stability |
| iLQR / DDP | Trajectory optimisation by repeated local LQR / with second-order dynamics terms |
| Generalised plant / LFT / mixed sensitivity / γ-iteration | `P` with `(w, u) → (z, y)` / closed-loop map / `[W_sS; W_uKS; W_tT]` / bisection on the H∞ level |

---

## 12. Reading list

**C++ (choose one primary; dip into the other).**
- Lippman, Lajoie & Moo, *C++ Primer*, 5th ed. — the book you own; the day plan cites sections. Note it predates C++17 (no `std::optional`, structured bindings, `if constexpr`); cppreference.com fills the gaps.
- learncpp.com — free, beginner-oriented, current (C++17/20); chapters cited per day.
- cppreference.com — the reference; read the examples at the bottom of each page.
- Eigen documentation: "Getting started", "The Matrix class", "Block operations", "Writing functions taking Eigen types", "MatrixFunctions module".
- CMake: the official tutorial (steps 1–4), Craig Scott, *Professional CMake* (if you want depth), and "It's Time To Do CMake Right" (blog, for the target-based mindset).
- Google Test primer; the VS Code C++ debugging docs.
- Later: Meyers, *Effective Modern C++* (after Day 42); Sutter & Alexandrescu, *C++ Coding Standards*.

**Control (in order of use).**
- Åström & Murray, *Feedback Systems* (free PDF) — the backbone: modelling, state space, frequency domain, PID, robust performance.
- Franklin, Powell & Emami-Naeini, *Feedback Control of Dynamic Systems* — classical design, root locus, digital control, state space.
- Nise, *Control Systems Engineering* — root locus and compensator design walkthroughs.
- Åström & Wittenmark, *Computer-Controlled Systems* — sampling and digital design.
- Chen, *Linear System Theory and Design* — realisation theory, controllability/observability, proofs.
- Anderson & Moore, *Optimal Control: Linear Quadratic Methods* (free) — LQR/LQG derivations.
- Bertsekas, *Dynamic Programming and Optimal Control*, Vol. 1 — DP; Kirk, *Optimal Control Theory* — HJB, bang-bang.
- Khalil, *Nonlinear Systems* — linearisation, Lyapunov.
- Simon, *Optimal State Estimation* — KF/EKF/UKF/particle; Thrun, Burgard & Fox, *Probabilistic Robotics* — the same, applied.
- Ljung, *System Identification: Theory for the User* — ARX, RLS, ETFE, subspace, validation; Pintelon & Schoukens for the frequency domain.
- Skogestad & Postlethwaite, *Multivariable Feedback Control*; Doyle, Francis & Tannenbaum, *Feedback Control Theory* (free); Zhou, Doyle & Glover, *Essentials of Robust Control* — robustness, norms, H∞.
- Rawlings, Mayne & Diehl, *Model Predictive Control* (free); Wang, *MPC System Design and Implementation Using MATLAB* (Hildreth, condensed form); Boyd & Vandenberghe, *Convex Optimization* (free) — QP/KKT.
- Papers cited by day: Laub (1979), Doyle (1978), DGKF (1989), Wan & van der Merwe (2000), Juang & Pappa (1985), Tassa et al. (2012), Skogestad (2003), Bruinsma & Steinbuch (1990).

**DSP.**
- Oppenheim & Schafer, *Discrete-Time Signal Processing* — DFT/FFT, spectral estimation, filter structures and design.
- Smith, *The Scientist and Engineer's Guide to Digital Signal Processing* (free online) — the readable version of the same.
- SciPy `signal` documentation — for conventions (`welch`, `butter`, `filtfilt`) to cross-check against.

---

*End of Design Document V2. This document is the plan; the code and the tests are the proof. Revise it at the end of every phase (Days 10, 16, 22, 28, 36, 43, 49, 55, 59, 66, 70) — the retrospective questions exist so the revisions are informed.*
