# Control Systems Library (csl) — Design Document V3

| Field | Value |
|---|---|
| Document | Design Document and Project Plan, Version 3.0 |
| Project | Control Systems Library (`csl`) |
| Language standard | C++17 |
| Dependencies | Eigen 3.4 (linear algebra), Google Test 1.14 (unit testing) |
| Target platforms | Windows 11 / MSVC 14.44 (primary); Linux / GCC; macOS / Clang |
| Status | Approved plan; implementation not started |
| Supersedes | Design Document V2 (`docs/DESIGN_V2.md`), V1 (`docs/DESIGN.md`) |
| Date | 2026-10-24 |

---

## Contents

1. Introduction
2. Objectives and Design Principles
3. Development Environment
4. Architecture
5. Notation and Reference Models
6. Curriculum Overview and Progress Tracker
7. Phase 1 — Foundations (Days 1–10)
8. Phase 2 — System Representations and Time-Domain Analysis (Days 11–16)
9. Phase 3 — Frequency-Domain Analysis (Days 17–22)
10. Phase 4 — Classical Control Design (Days 23–28)
11. Phase 5 — Discrete Time and Digital Signal Processing (Days 29–36)
12. Phase 6 — State-Space Design (Days 37–43)
13. Phase 7 — Optimal Control (Days 44–49)
14. Phase 8 — State Estimation (Days 50–55)
15. Phase 9 — System Identification (Days 56–59)
16. Phase 10 — Robust, Predictive and Nonlinear Optimal Control (Days 60–66)
17. Phase 11 — Packaging and Release (Days 67–70)
18. Testing Strategy
19. Extensions
Appendix A — Glossary
Appendix B — References

**List of figures:** Figure 3-1 Repository layout · Figure 4-1 Module dependency graph · Figure 7-1 Compilation pipeline · Figure 10-1 Sampled-data control loop · Figure 11-1 Eight-point FFT signal flow · Figure 12-1 Observer-based state feedback · Figure 16-1 Generalized plant interconnection

**List of tables:** Table 3-1 Toolchain · Table 3-2 Coding standard · Table 3-3 Numerical tolerance policy · Table 4-1 Modules · Table 4-2 Data flow between modules · Table 4-3 Build-versus-buy decisions · Table 5-1 Notation · Table 5-2 Reference models · Table 5-3 Reference values · Table 6-1 Phase overview · Table 6-2 Progress tracker · Table 18-1 Test oracles by module · Table 19-1 Extensions

---

## 1. Introduction

### 1.1 Purpose

This document specifies the design of the Control Systems Library (`csl`) and the 70-day implementation plan by which it is built. The library is a C++17 implementation of the core functionality of a control-systems and signal-processing toolbox: system representations and conversions, time- and frequency-domain analysis, classical and state-space controller design, discretization and digital filtering, optimal and predictive control, state estimation, and system identification.

The plan serves two purposes simultaneously: it produces a working, tested library, and it is structured so that a developer with no prior C++ experience and introductory control-systems knowledge acquires both subjects in a fixed, dependency-ordered sequence. Each working day introduces one or more C++ concepts and one or more control concepts, and the library component built on that day requires both.

### 1.2 Scope

In scope: everything listed in Table 4-1 (modules) and Table 6-1 (phases). Out of scope: hardware interfaces, real-time scheduling, MIMO transfer-function matrices, and the extensions of Table 19-1, which are specified only at the level of a title and prerequisites.

### 1.3 Audience and Prerequisites

The document is written for the implementer of the library. Prerequisites are: familiarity with programming fundamentals (variables, loops, functions) in any language; calculus (derivatives, integrals, Taylor series); linear algebra (matrices, eigenvalues); complex numbers. No C++ knowledge is assumed. Control-systems knowledge beyond the definition of a feedback loop is not assumed; each concept is introduced in the day that uses it.

### 1.4 Document Conventions

**Numbering.** Figures, tables, listings and equations are numbered by chapter: Figure 4-1 is the first figure of Chapter 4; Eq. (5-3) is the third equation of Chapter 5. Verification items are numbered `V<day>.<item>` (for example V12.3).

**References.** Sources are listed once in Appendix B and cited as `[R9, §6.3]`. No source is described in the body text.

**Equations** are displayed in fenced blocks with the equation number at the right margin. Matrices are written in bracketed row form, for example `[[0, 1], [−4, −0.5]]`. Transposition is `ᵀ`, inverse is `⁻¹`, the Laplace variable is `s`, the z-transform variable is `z`, angular frequency is `ω` in rad/s, sample time is `T_s` in seconds.

**Code** is C++17 and follows the coding standard of Table 3-2. Listings show declarations in full and implementations where the algorithm is the subject of the day; routine bodies are elided with `/* ... */`. All identifiers in listings are final: later days call them by those names.

**Day entries** follow the template of Section 6.2.

### 1.5 Structure of This Document

Chapters 2–6 are the specification: objectives, environment, architecture, notation, and the curriculum overview with the progress tracker. Chapters 7–17 are the plan, one chapter per phase and one section per day. Chapter 18 specifies testing. Chapter 19 lists extensions. The appendices hold the glossary and references.

---

## 2. Objectives and Design Principles

### 2.1 C++ Learning Objectives

On completion the implementer shall be able to: compile and link multi-file programs by hand and with CMake; explain translation units, headers, include guards and the One Definition Rule; use value, reference and pointer semantics correctly; distinguish stack and heap storage, apply RAII and smart pointers, and justify the absence of `new`/`delete` in the code base; design classes with constructors, invariants, `const` correctness and operator overloading; use inheritance, virtual dispatch and abstract base classes; use lambdas, `std::function`, templates, STL containers, iterators and algorithms, exceptions, `std::optional`, structured bindings, `enum class`, move semantics and the Rule of Zero; write Google Test suites; use a debugger, compiler warnings and sanitizers; package and install a library with CMake.

### 2.2 Control and Signal-Processing Learning Objectives

On completion the implementer shall be able to derive, implement and test: dynamic systems as ODEs and numerical integration; state-space, transfer-function and zero-pole-gain representations and their conversions; poles, zeros, stability, minimum phase and time delay; time-domain responses and specifications; frequency response, Bode and Nyquist analysis, stability margins and sensitivity functions; root locus and Routh–Hurwitz; PID, tuning rules, lead/lag and loop shaping; sampling, discretization and the z-domain; DFT/FFT, spectral estimation and FIR/IIR filter design; controllability, observability, pole placement, observers, Lyapunov equations and Gramians; LQR and Riccati equations; nonlinear systems and linearization; dynamic programming; Kalman, extended and unscented Kalman filters and LQG; ARX, recursive least squares, frequency-domain and subspace identification; uncertainty models, small-gain, H2/H∞ norms; quadratic programming and MPC; iterative LQR; H∞ synthesis.

### 2.3 Design Principles

- P1 **Delegate generic numerics.** Matrices, vectors, decompositions, matrix functions, complex arithmetic, elementary functions, random numbers, containers and algorithms are taken from Eigen, `std::complex`, `<cmath>`, `<random>` and the STL. No such facility is reimplemented (Table 4-3).
- P2 **Hand-write domain algorithms.** Every algorithm that constitutes control theory or signal processing is implemented in the library, including the FFT, the Riccati solvers, the Kalman filters, filter design and the QP solver.
- P3 **Admit a component only if it teaches.** A component exists only if implementing it teaches C++ or control. Infrastructure that teaches neither (logging, registries, container wrappers) is excluded.
- P4 **Sequential dependency.** Each day uses only components built on earlier days and produces a component used on a later day. No component is stubbed without a named day that completes it.
- P5 **Cohesion by contract.** One set of types and conventions (Section 4.3) is used by every module, so that the output of any function is a valid input to any function accepting that type.
- P6 **Verification per concept.** A concept is complete only when its implementation passes a test with a known numeric answer (Chapter 18).
- P7 **Visualisation inside the library.** The library writes self-contained HTML/SVG plots and CSV files with no external dependency.

### 2.4 Changes from Version 2

Version 3 retains the curriculum, module set, type contract and verification values of Version 2 without change. It restructures the document into specification chapters followed by plan chapters, numbers all figures, tables, listings and equations, consolidates all sources into Appendix B, adds the progress tracker (Table 6-2), standardises the day template (Section 6.2), and removes informal and repeated text.

---

## 3. Development Environment

### 3.1 Toolchain

Table 3-1 lists the tools used. All versions were verified on the development machine.

**Table 3-1 — Toolchain**

| Tool | Version | Role |
|---|---|---|
| Visual Studio 2022 Community, MSVC toolset | 14.44 | Compiler (`cl.exe`) and linker (`link.exe`) |
| CMake | 4.3.3 | Build-system generator |
| Ninja | bundled with Visual Studio (`Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja`) | Build driver |
| Visual Studio Code with extensions *C/C++* and *CMake Tools* | 1.143 | Editor and debugger front end |
| Git | 2.54 | Version control |
| Python with NumPy/SciPy | 3.13 | Optional, for independent cross-checks only; not a dependency |
| GCC ≥ 11 or Clang ≥ 14 | — | Alternative compilers (Linux/macOS, CI) |

All commands in this document assume the *Developer PowerShell for VS 2022*, which places `cl.exe` and `ninja.exe` on the path. Equivalent GCC/Clang commands are given where they differ.

### 3.2 Repository Layout

Figure 3-1 shows the repository at completion (Day 70). The annotation on each entry is the day on which it is created. The layout is referenced by every day entry; day entries do not repeat it.

**Figure 3-1 — Repository layout (final state, with creation day)**

```
controlSystemCPPLibrary/
├── CMakeLists.txt                        Day 4   root build file
├── README.md                             Day 68
├── CHANGELOG.md                          Day 69
├── .gitignore                            Day 1
├── .clang-format                         Day 69
├── .github/workflows/ci.yml              Day 69
├── cmake/cslConfig.cmake.in              Day 67
├── docs/                                 design documents, tutorials (Day 68), retrospective (Day 70)
├── sandbox/                              Days 1–3 programs; phase review notes
├── scripts/test_install.ps1 | .sh        Day 67
├── include/csl/
│   ├── csl.hpp                           Day 22  umbrella header
│   ├── core/
│   │   ├── Types.hpp                     Day 6   scalar, vector, matrix, complex aliases; constants
│   │   ├── Exceptions.hpp                Day 8
│   │   ├── MathUtils.hpp                 Day 17  dB, angle, linspace/logspace, unwrap; templates (Day 34)
│   │   ├── Polynomial.hpp                Day 11
│   │   ├── LinearAlgebra.hpp             Day 37  rank, ctrb/obsv; kron, Lyapunov, Gramians (Day 40); ARE (Day 44); DARE (Day 45)
│   │   └── NumericalMethods.hpp          Day 27  bisection; Jacobian (Day 47); QP (Day 62)
│   ├── systems/
│   │   ├── LTISystem.hpp                 Day 13  abstract base
│   │   ├── StateSpace.hpp                Day 8
│   │   ├── TransferFunction.hpp          Day 12
│   │   ├── Conversions.hpp               Day 14  tf2ss, ss2tf; c2d/d2c (Days 29–30)
│   │   ├── Interconnect.hpp              Day 19  series, parallel, feedback, gang of four
│   │   └── NonlinearSystem.hpp           Day 47
│   ├── simulation/
│   │   ├── SimResult.hpp                 Day 7
│   │   ├── ODESolver.hpp                 Day 4   scalar Euler; vector Euler/RK4/integrate (Day 7)
│   │   ├── TimeResponse.hpp              Day 15  step, impulse, lsim; specifications (Day 16)
│   │   ├── Simulator.hpp                 Day 24  estimator hook (Day 39), noise (Day 50), nonlinear plant (Day 47)
│   │   └── MonteCarlo.hpp                Day 50
│   ├── analysis/
│   │   ├── FrequencyResponse.hpp         Day 17  frequencyResponse, bode; nyquist, margins (Day 18); sensitivity (Day 28)
│   │   ├── RootLocus.hpp                 Day 21
│   │   ├── Stability.hpp                 Day 22  Routh–Hurwitz, minimum phase; Lyapunov test (Day 40)
│   │   ├── Norms.hpp                     Day 40  H2; H∞ (Day 61)
│   │   └── Robustness.hpp                Day 60
│   ├── control/
│   │   ├── Controller.hpp                Day 23  abstract base
│   │   ├── PID.hpp                       Day 23  anti-windup (Day 25)
│   │   ├── Tuning.hpp                    Day 26
│   │   ├── Relay.hpp                     Day 26
│   │   ├── LeadLag.hpp                   Day 27
│   │   ├── TransferFunctionController.hpp Day 30
│   │   ├── StateFeedback.hpp             Day 38  integral action (Day 41)
│   │   ├── LQR.hpp                       Day 44  dlqr, finite horizon (Day 45); Bryson, integral (Day 46)
│   │   ├── DynamicProgramming.hpp        Day 48
│   │   ├── LQG.hpp                       Day 52
│   │   ├── MPC.hpp                       Day 63  soft constraints (Day 64)
│   │   ├── iLQR.hpp                      Day 65
│   │   └── HInfinity.hpp                 Day 66
│   ├── estimation/
│   │   ├── Estimator.hpp                 Day 39  abstract base
│   │   ├── LuenbergerObserver.hpp        Day 39
│   │   ├── KalmanFilter.hpp              Day 51  steady state (Day 52)
│   │   ├── ExtendedKalmanFilter.hpp      Day 53
│   │   └── UnscentedKalmanFilter.hpp     Day 54
│   ├── dsp/
│   │   ├── FFT.hpp                       Day 31
│   │   ├── Spectral.hpp                  Day 32  windows, periodogram, welch, csd; etfe (Day 58)
│   │   ├── DigitalFilter.hpp             Day 33
│   │   └── FilterDesign.hpp              Day 34
│   ├── sysid/
│   │   ├── ARX.hpp                       Day 56
│   │   ├── Validation.hpp                Day 56  residual tests (Day 59)
│   │   ├── RLS.hpp                       Day 57
│   │   ├── FrequencyDomainID.hpp         Day 58
│   │   └── Subspace.hpp                  Day 59
│   ├── signals/Signals.hpp               Day 16  deterministic signals; noise, PRBS (Day 50)
│   └── plot/
│       ├── CSV.hpp                       Day 5
│       ├── Figure.hpp                    Day 15
│       └── Plots.hpp                     Day 15  plotStep; plotBode/Nyquist/RootLocus (Days 17–21); plotSpectrum (Day 32)
├── src/                                  one .cpp per header requiring one, mirroring include/csl
├── tests/
│   ├── CMakeLists.txt                    Day 10
│   ├── <module>/test_<Header>.cpp        one per header
│   └── consumer/                         Day 67
└── examples/
    ├── CMakeLists.txt                    Day 4
    ├── plants.hpp                        Day 6   reference models E1–E5 (Table 5-2)
    ├── ex01_first_order_sim.cpp          Day 4
    ├── ex02_representations.cpp          Day 22
    ├── ex03_classical_design.cpp         Day 28
    ├── ex04_dsp_and_filters.cpp          Day 36
    ├── ex05_state_space_design.cpp       Day 43
    ├── ex06_lqr_pendulum.cpp             Day 49
    ├── ex07_estimation.cpp               Day 55
    ├── ex08_sysid.cpp                    Day 59
    ├── ex09_mpc.cpp                      Day 64
    └── ex10_full_pipeline.cpp            Day 70
```

### 3.3 Build Procedure

Listing 3-1 gives the commands used from Day 4 onward. Configuration is repeated only when `CMakeLists.txt` files change; CMake re-runs itself automatically in that case.

**Listing 3-1 — Build, test and run commands (Developer PowerShell)**

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug      # configure (once)
cmake --build build                                         # build library, tests, examples
ctest --test-dir build --output-on-failure                  # run unit tests
.\build\examples\ex01_first_order_sim.exe                   # run an example
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release                                 # optimised build (timing, benchmarks)
```

With the Visual Studio generator (no `-G`), the build is multi-configuration: append `--config Debug` to `cmake --build` and `-C Debug` to `ctest`.

Debugging: in VS Code, set the CMake Tools debug target to a test or example executable, place a breakpoint, and start debugging (F5). Eigen matrices are displayed through the `eigen.natvis` file in `build/_deps/eigen-src/debug/msvc/`.

### 3.4 Coding Standard

**Table 3-2 — Coding standard**

| Item | Rule |
|---|---|
| File extensions | Headers `.hpp`, sources `.cpp`; one header per class or function group |
| Include guard | `#pragma once` as the first line of every header |
| Include paths | From the include root: `#include "csl/systems/StateSpace.hpp"` |
| Namespaces | `csl::<module>` (lower case); no `using namespace` in headers |
| Type names | `PascalCase` (`TransferFunction`) |
| Functions and variables | `camelCase` (`frequencyResponse`, `dcGain`) |
| Private data members | trailing underscore (`A_`, `num_`) |
| Constants | `UPPER_CASE` for namespace-scope constants; `constexpr` locals otherwise |
| Parameters | Eigen and class objects passed by `const&`; results returned by value |
| Prohibited | raw `new`/`delete`; C-style casts; global mutable state; `float` |
| Error handling | preconditions throw a `csl::Error` subtype (Listing 4-6); legitimately absent results use `std::optional` |
| Tests | `tests/<module>/test_<Header>.cpp`; `TEST(HeaderName, Property)`; floating-point comparison with `EXPECT_NEAR` |
| Commits | one per day: `Day NN: <summary>` |
| Formatting | `.clang-format` (Day 69); 4-space indentation; 110-column limit |

### 3.5 Numerical Tolerance Policy

**Table 3-3 — Numerical tolerance policy for tests**

| Result class | Tolerance | Examples |
|---|---|---|
| Closed-form algebra | `1e-9` | polynomial products, `tf2ss` matrices, dc gain |
| Decomposition-based | `1e-8` | poles from eigenvalues, Lyapunov solutions |
| Iterative solvers | `1e-6` | ARE, DARE, QP, RLS steady state |
| Simulation versus analytic solution | stated per test | RK4 at `dt = 0.01` versus `e^{−t}`: `1e-6` |
| Statistical | bound with fixed seed, stated per test | "RMSE < 0.45 with seed 7" |

### 3.6 Version Control

One commit per completed day, after all exit criteria of the day are met. Generated artefacts (`build/`, `*.html`, `*.csv`, `*.obj`, `*.exe`, `*.pdb`, `.vs/`, `.vscode/`) are excluded by `.gitignore`. Phase completions are tagged `phase-<n>`; the release is tagged `v2.0.0` on Day 70 (the library version is independent of the document version).

---
## 4. Architecture

### 4.1 Modules

The library comprises eleven modules (Table 4-1), each a sub-namespace of `csl` and a directory under `include/csl/` (Figure 3-1).

**Table 4-1 — Modules**

| Module | Namespace | Responsibility | Introduced |
|---|---|---|---|
| core | `csl` | Type aliases, constants, exceptions, polynomials, control-specific linear algebra (Lyapunov, Riccati, Gramians), numerical methods (root finding, Jacobians, QP) | Phase 1 |
| systems | `csl::systems` | `LTISystem` base, `StateSpace`, `TransferFunction`, conversions, discretization, interconnections, `NonlinearSystem` | Phase 1 |
| simulation | `csl::simulation` | ODE integrators, `SimResult`, time responses and specifications, closed-loop `Simulator`, Monte Carlo | Phase 1 |
| analysis | `csl::analysis` | Frequency response, Bode, Nyquist, margins, sensitivity, root locus, Routh–Hurwitz, Lyapunov stability, norms, robustness | Phase 3 |
| control | `csl::control` | `Controller` base, PID, tuning, lead/lag, transfer-function controller, state feedback, LQR, dynamic programming, LQG, MPC, iLQR, H∞ | Phase 4 |
| estimation | `csl::estimation` | `Estimator` base, Luenberger observer, Kalman, extended and unscented Kalman filters | Phase 6 |
| dsp | `csl::dsp` | FFT, windows, spectral estimation, digital filter structures and design | Phase 5 |
| sysid | `csl::sysid` | ARX, RLS, frequency-domain identification, ERA, validation | Phase 9 |
| signals | `csl::signals` | Deterministic test signals, white noise, PRBS | Phase 2 |
| plot | `csl::plot` | CSV I/O, SVG/HTML figures, control-specific plot helpers | Phase 1 |
| examples | — | Reference models (`plants.hpp`) and ten example programs | Phase 1 |

### 4.2 Dependencies

Figure 4-1 shows the permitted include dependencies. An arrow from X to Y means X may include headers of Y. The graph is acyclic.

**Figure 4-1 — Module dependency graph**

```
                        ┌─────────┐
                        │  core   │
                        └────▲────┘
        ┌────────────┬───────┼────────┬────────────┐
   ┌────┴────┐  ┌────┴────┐ │  ┌─────┴────┐  ┌────┴────┐
   │ systems │  │ signals │ │  │   dsp    │  │  plot   │
   └────▲────┘  └────▲────┘ │  └─────▲────┘  └────▲────┘
        │            │      │        │            │
   ┌────┴────────────┴──────┴────┐   │            │
   │          simulation         │   │            │
   └────▲────────────────────────┘   │            │
        │                            │            │
   ┌────┴────────────────────────────┴────┐       │
   │              analysis                ├───────┘  (plot/Plots.hpp depends on analysis result types)
   └────▲──────────────▲───────────────▲──┘
        │              │               │
   ┌────┴────┐   ┌─────┴──────┐   ┌────┴────┐
   │ control │   │ estimation │   │  sysid  │
   └─────────┘   └────────────┘   └─────────┘
```

### 4.3 Type Contract

The type contract is the set of types and conventions shared by all modules. It is the mechanism by which principle P5 (Section 2.3) is achieved. Every public function accepts and returns only the types defined here, Eigen types, standard-library types, or classes declared in the module headers of Figure 3-1.

#### 4.3.1 Scalar and Linear-Algebra Types

**Listing 4-1 — `include/csl/core/Types.hpp` (Day 6)**

```cpp
#pragma once
#include <Eigen/Dense>
#include <complex>
#include <limits>

namespace csl {

using Real          = double;
using Index         = Eigen::Index;
using Vector        = Eigen::VectorXd;          // column vector
using RowVector     = Eigen::RowVectorXd;
using Matrix        = Eigen::MatrixXd;          // column-major
using Complex       = std::complex<Real>;
using ComplexVector = Eigen::VectorXcd;
using ComplexMatrix = Eigen::MatrixXcd;

inline constexpr Real PI     = 3.141592653589793238462643383279502884;
inline constexpr Real TWO_PI = 2.0 * PI;
inline constexpr Real EPS    = std::numeric_limits<Real>::epsilon();
inline constexpr Real INF    = std::numeric_limits<Real>::infinity();

} // namespace csl
```

Conventions:
- C1 All numeric code uses the aliases of Listing 4-1; `double` and `Eigen::MatrixXd` are not spelled out elsewhere.
- C2 A sampled multi-channel signal is a `Matrix` with one row per time sample and one column per channel ("time along rows"). A sampled scalar signal is a `Vector`.
- C3 Frequencies are angular (rad/s) and named `w`, except in functions whose domain convention is hertz, where the parameter name carries the suffix `_hz`.
- C4 `Polynomial` coefficients are stored in descending powers: `{1, 3, 2}` denotes `s² + 3s + 2`. The denominator of a `TransferFunction` is monic.

#### 4.3.2 Systems

**Listing 4-2 — `LTISystem` interface (Day 13)**

```cpp
namespace csl::systems {

class LTISystem {
public:
    virtual ~LTISystem() = default;
    virtual Index order()   const = 0;
    virtual Index inputs()  const = 0;
    virtual Index outputs() const = 0;
    virtual Real  Ts()      const = 0;                       // 0: continuous; > 0: discrete, sample time in s
    virtual std::vector<Complex> poles() const = 0;
    virtual std::vector<Complex> zeros() const = 0;          // SISO transmission zeros
    virtual ComplexMatrix evaluate(Complex s) const = 0;     // H(s) or H(z), outputs x inputs
    virtual Matrix dcGain() const = 0;
    bool isContinuous() const { return Ts() == 0.0; }
    bool isDiscrete()   const { return Ts() >  0.0; }
    bool isSISO()       const { return inputs() == 1 && outputs() == 1; }
    bool isStable()     const;                               // continuous: Re p < 0; discrete: |p| < 1
};

class StateSpace       : public LTISystem { /* A, B, C, D, Ts — Day 8 */ };
class TransferFunction : public LTISystem { /* num, den, Ts — Day 12; SISO */ };

} // namespace csl::systems
```

Conventions:
- C5 `Ts() == 0` denotes continuous time; `Ts() > 0` denotes discrete time with that sample period. There are no separate discrete-time classes. `c2d` returns systems with `Ts > 0`; `d2c` returns systems with `Ts == 0`.
- C6 `evaluate` is the single frequency-domain primitive. Analysis functions call it at `s = jω` for continuous systems and at `s = e^{jωT_s}` for discrete systems.
- C7 `StateSpace` is MIMO; `TransferFunction` is SISO. Functions restricted to SISO systems test `isSISO()` and throw `NotSupported` otherwise.
- C8 Functions requiring matrices accept `const StateSpace&`; conversion from a transfer function is performed explicitly by the caller with `tf2ss` or `toStateSpace`.

#### 4.3.3 Result Types

**Listing 4-3 — Result types**

```cpp
namespace csl::simulation {
struct SimResult {           // Day 7; produced by integrate, step, impulse, lsim, Simulator::run
    Vector t;                // N
    Matrix x;                // N x n states
    Matrix y;                // N x p outputs
    Matrix u;                // N x m inputs
    Index samples() const { return t.size(); }
};
}
namespace csl::analysis {
struct FrequencyResponse {   // Day 17; produced by frequencyResponse(sys, w) and by dsp::etfe(u, y, fs)
    Vector        w;         // rad/s, ascending
    ComplexVector H;         // H(jw) or H(e^{jwTs}), SISO
    Real          Ts = 0.0;
};
}
```

Convention C9: all time-domain results are `SimResult`; all SISO frequency-domain results are `FrequencyResponse`, whether computed from a model or estimated from data. Plot functions, specification functions, spectral functions and identification functions accept these types directly.

#### 4.3.4 Controllers, Estimators and Filters

**Listing 4-4 — `Controller` interface (Day 23)**

```cpp
namespace csl::control {
class Controller {
public:
    virtual ~Controller() = default;
    virtual Vector compute(const Vector& measurement, const Vector& reference, Real t) = 0;
    virtual void   reset() = 0;
    virtual Real   Ts() const = 0;             // controller sample time, > 0
    virtual Index  inputs()  const = 0;        // size of measurement
    virtual Index  outputs() const = 0;        // size of the returned control vector
};
}
```

**Listing 4-5 — `Estimator` and `DigitalFilter` interfaces (Days 39, 33)**

```cpp
namespace csl::estimation {
class Estimator {
public:
    virtual ~Estimator() = default;
    virtual void   predict(const Vector& u) = 0;
    virtual void   update (const Vector& y) = 0;
    virtual Vector state()      const = 0;
    virtual Matrix covariance() const = 0;     // zero matrix for deterministic observers
    virtual void   reset(const Vector& x0) = 0;
};
}
namespace csl::dsp {
class DigitalFilter {
public:
    virtual ~DigitalFilter() = default;
    virtual Real process(Real x) = 0;
    Vector       process(const Vector& x);     // applies the scalar version sample by sample
    virtual void reset() = 0;
    virtual Real Ts() const = 0;
    virtual systems::TransferFunction toTransferFunction() const = 0;   // discrete, Ts > 0
    virtual std::unique_ptr<DigitalFilter> clone() const = 0;
};
}
```

Conventions:
- C10 A `Controller` operates on vectors even when SISO; the `Simulator` is therefore independent of controller type.
- C11 A controller requiring the full state declares that its `measurement` argument is the state vector. The `Simulator` supplies either the true state or an `Estimator`'s estimate (Day 39).
- C12 A `DigitalFilter` is convertible to a discrete `TransferFunction`; filters are therefore analysed with the same functions as systems.

#### 4.3.5 Errors

**Listing 4-6 — `include/csl/core/Exceptions.hpp` (Day 8)**

```cpp
#pragma once
#include <stdexcept>
namespace csl {
class Error             : public std::runtime_error { using std::runtime_error::runtime_error; };
class DimensionMismatch : public Error { using Error::Error; };
class InvalidArgument   : public Error { using Error::Error; };
class NumericalError    : public Error { using Error::Error; };   // singularity, non-convergence
class NotSupported      : public Error { using Error::Error; };   // e.g. MIMO where SISO is required
}
```

Convention C13: precondition violations throw one of these types with a message naming the function and the offending values. Absent-but-valid results (for example a non-existent gain margin) are returned as `std::optional`, never as sentinel values.

### 4.4 Data Flow

Table 4-2 lists, for each result type, the functions that produce it and the functions that consume it. The table is the operational statement of principle P5.

**Table 4-2 — Data flow between modules**

| Type | Produced by | Consumed by |
|---|---|---|
| `Polynomial` | constructors, arithmetic, `fromRoots`, `charPoly` | `TransferFunction`, `routhArray`, `place`, Tustin substitution |
| `LTISystem` (`StateSpace`, `TransferFunction`) | constructors, `tf2ss`, `ss2tf`, `c2d`, `d2c`, `series`/`parallel`/`feedback`, `DigitalFilter::toTransferFunction`, `arx`, `era`, `fitRational`, `lqgCompensator`, `hinfsyn` | all of `analysis`, `simulation`, `control`, `estimation`; `plot` |
| `SimResult` | `integrate`, `step`, `impulse`, `lsim`, `Simulator::run` | `stepInfo`, `fitFOPDT`, `plotOutputs`, `welch`, `arx`, `markovParameters` |
| `FrequencyResponse` | `frequencyResponse`, `etfe` | `bode`, `nyquist`, `margins`, `plotBode`, `fitRational`, `relativeErrorEnvelope` |
| `Vector` (signal) | `signals::*`, `DigitalFilter::process`, `PRBS`, `WhiteNoise` | `lsim`, `Simulator`, `arx`, `etfe`, `welch`, filters |
| `ComplexVector` (spectrum) | `fft`, `rfft` | `periodogram`, `welch`, `csd` |
| gain `Matrix K` | `place`, `lqr`, `dlqr`, `lqrIntegral` | `StateFeedbackController`, `IntegralStateFeedbackController`, `LQGController`, `iLQR` |
| `Matrix P` (Riccati) | `solveARE`, `solveDARE` | `lqr`, `dlqr`, `steadyStateKalman`, `MPC` (terminal weight), `hinfnorm`, `hinfsyn` |
| `Estimator` | `LuenbergerObserver`, `KalmanFilter`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter` | `Simulator`, `LQGController` |
| `QPResult` | `solveQP` | `MPC` |

### 4.5 Build-Versus-Buy Decisions

**Table 4-3 — Build-versus-buy decisions**

| Capability | Source | Rationale |
|---|---|---|
| Dense matrices, vectors, arithmetic, blocks, norms | Eigen | Generic infrastructure; vectorised and expression-templated |
| LU, QR, SVD, Cholesky, Schur, eigenvalue decompositions; inverse; rank; least squares | Eigen | Numerically subtle generic algorithms |
| Matrix exponential and logarithm | Eigen `unsupported/MatrixFunctions` | Generic; used inside hand-written discretization |
| Complex arithmetic | `std::complex` | Standard; interoperates with Eigen |
| Elementary functions | `<cmath>` | Generic |
| Random engines and distributions | `<random>` | Standard, statistically correct |
| Containers and algorithms | STL | Generic |
| Unit-test framework | Google Test | Not a learning objective |
| Polynomial class; root finding via companion matrix | Library | Transfer-function representation is control theory; Eigen computes the eigenvalues |
| System representations, conversions, interconnections, discretization | Library | Control theory |
| ODE integrators (Euler, RK4) | Library | Simulation is a core competence |
| Frequency response, Bode, Nyquist, margins, root locus, Routh–Hurwitz, sensitivity | Library | Control analysis |
| DFT/FFT, windows, PSD, ETFE; FIR/IIR structures; Butterworth, windowed FIR, notch design | Library | Signal processing is a stated objective |
| Controllability, observability, Lyapunov, Gramians, Riccati (CARE, DARE) | Library | No library equivalent; the core of state-space control |
| PID, tuning rules, lead/lag, pole placement, LQR, LQG, MPC, iLQR, H∞ | Library | Subject matter |
| Luenberger, Kalman, EKF, UKF | Library | Subject matter |
| ARX, RLS, ETFE fit, ERA | Library | Subject matter |
| QP solver (Hildreth's method) | Library | The optimisation inside MPC; `solveQP` is the seam for a future external solver |
| SVG/HTML plotting; CSV | Library | Small, dependency-free, a file-I/O exercise (P7) |
| Logging, registries, container wrappers | Not implemented | Principle P3 |

---

## 5. Notation and Reference Models

### 5.1 Notation

**Table 5-1 — Notation**

| Symbol | Meaning | Code |
|---|---|---|
| `x ∈ ℝⁿ`, `u ∈ ℝᵐ`, `y ∈ ℝᵖ` | state, input, output | `Vector x, u, y` |
| `A, B, C, D` | state-space matrices (`n×n`, `n×m`, `p×n`, `p×m`) | `sys.A()` … |
| `T_s` | sample time [s]; `0` denotes continuous time | `Ts` |
| `s`, `z` | Laplace and z-transform variables | `Complex s` |
| `ω` | angular frequency [rad/s] | `w` |
| `H(s)`, `N(s)/D(s)` | transfer function, numerator/denominator polynomials | `num()`, `den()` |
| `p_i`, `z_i` | poles, zeros | `poles()`, `zeros()` |
| `ω_n`, `ζ`, `τ` | natural frequency, damping ratio, time constant | `wn`, `zeta`, `tau` |
| `L`, `S`, `T` | loop, sensitivity, complementary sensitivity transfer functions | `L`, `S`, `T` |
| `K`, `L_o` | state-feedback gain, observer gain | `K`, `L` |
| `Q`, `R` | state and input weights (LQR); process and measurement noise covariances (KF) — context disambiguates | `Q`, `R` |
| `P` | Riccati solution; error covariance (KF) | `P` |
| `‖G‖₂`, `‖G‖∞` | H2 and H∞ norms | `h2norm`, `hinfnorm` |
| `N`, `N_u` | prediction and control horizons (MPC) | `N`, `Nu` |
| `λ` | eigenvalue; forgetting factor (RLS); dual variable (QP) — context disambiguates | `lambda` |

### 5.2 Reference Models

Five physical systems are used throughout the plan (Table 5-2). Their factories are defined in `examples/plants.hpp` (Day 6) and shared by examples and tests.

**Table 5-2 — Reference models**

| ID | System | States / inputs / outputs | Parameters | Factory |
|---|---|---|---|---|
| E1 | First-order thermal system | 1 / 1 / 1 | `τ = 2 s`, `K = 3` | `plants::thermal()` |
| E2 | Mass–spring–damper (position output) | 2 / 1 / 1 | `m = 1`, `c = 0.5`, `k = 4` | `plants::massSpringDamper()` |
| E3 | DC motor, voltage to shaft speed | 2 / 1 / 1 | `R = 1`, `L = 0.5`, `J = 0.01`, `b = 0.1`, `K_t = K_b = 0.01` | `plants::dcMotor()` |
| E4 | Double integrator | 2 / 1 / 1 | — | `plants::doubleIntegrator()` |
| E5 | Cart–pole (inverted pendulum on a cart) | 4 / 1 / 2 | `M = 1`, `m = 0.1`, `l = 0.5`, `g = 9.81` | `plants::cartPole()` (Day 47) |

Equations of motion:

```
E1:  τ ẏ = −y + K u                                                              (5-1)
E2:  m ẍ + c ẋ + k x = F,   ω_n = √(k/m),   ζ = c / (2√(k m))                      (5-2)
E3:  L di/dt = −R i − K_b ω + V,   J dω/dt = K_t i − b ω                           (5-3)
E4:  ẍ = u                                                                       (5-4)
E5:  p̈ = [F + m l θ̇² sin θ − m g sin θ cos θ] / [M + m sin²θ]
     θ̈ = [−F cos θ − m l θ̇² sin θ cos θ + (M + m) g sin θ] / [l (M + m sin²θ)]    (5-5)
```

State-space forms of E1–E4 (state vectors: E1 `[y]`; E2 `[x, ẋ]`; E3 `[i, ω]`; E4 `[x, ẋ]`):

```
E1:  A = [−1/τ],  B = [K/τ],  C = [1],  D = [0]
E2:  A = [[0, 1], [−k/m, −c/m]],  B = [[0], [1/m]],  C = [1, 0],  D = [0]
E3:  A = [[−R/L, −K_b/L], [K_t/J, −b/J]],  B = [[1/L], [0]],  C = [0, 1],  D = [0]       (5-6)
E4:  A = [[0, 1], [0, 0]],  B = [[0], [1]],  C = [1, 0],  D = [0]
E5 (linearised at the upright equilibrium, Day 47):
     A = [[0,1,0,0], [0,0,−mg/M,0], [0,0,0,1], [0,0,(M+m)g/(Ml),0]],  B = [0, 1/M, 0, −1/(Ml)]ᵀ
```

### 5.3 Reference Values

Table 5-3 lists closed-form values used repeatedly in verification. All are derived from Table 5-2.

**Table 5-3 — Reference values**

| ID | Quantity | Value |
|---|---|---|
| E1 | pole; dc gain; step response at `t = τ` | `−0.5`; `3`; `63.2 %` of final value (`1.8964`) |
| E1 | 10–90 % rise time; 2 % settling time | `τ ln 9 = 4.394 s`; `τ ln 50 = 7.824 s` |
| E2 | poles; `ω_n`; `ζ`; `ω_d` | `−0.25 ± 1.9843j`; `2`; `0.125`; `1.9843` |
| E2 | dc gain; overshoot; peak time; peak value | `0.25`; `67.3 %`; `1.583 s`; `0.418` |
| E2 | resonance peak `|H|_max` and its frequency | `1.0079` (`0.07 dB`) at `1.968 rad/s` |
| E2 | H2 norm | `0.5` |
| E3 | poles; dc gain; transfer function | `−2.0025`, `−9.9975`; `0.0999` (`= 2/20.02`); `2/(s² + 12s + 20.02)` |
| E4 | LQR with `Q = I`, `R = 1`: `P`, `K`, closed-loop poles | `[[√3, 1], [1, √3]]`; `[1, 1.7321]`; `−0.866 ± 0.5j` |
| E5 | open-loop poles of the upright linearisation | `0, 0, ±4.646` |
| `L = 1/(s(s+1)(s+2))` | gain margin; phase crossover; phase margin; gain crossover; delay margin | `6 (15.56 dB)`; `1.414 rad/s`; `53.4°`; `0.446 rad/s`; `2.09 s` |
| `L = 1/(s(s+1)(s+2))` | peak sensitivity `M_s` and its frequency; stability limit of `K` | `1.57` at `0.73 rad/s`; `K < 6` |
| `L = 10/(s(s+1))` | phase margin; gain crossover; closed-loop bandwidth | `18.0°`; `3.08 rad/s`; `4.83 rad/s` |

---

## 6. Curriculum Overview and Progress Tracker

### 6.1 Phases

The plan comprises 70 working days in eleven phases (Table 6-1). A working day is 3–5 hours. The sequence is fixed; the calendar is not.

**Table 6-1 — Phase overview**

| Phase | Days | C++ topics | Control / DSP topics | Deliverable |
|---|---|---|---|---|
| 1 Foundations | 1–10 | Toolchain, compile/link, functions, headers, CMake, `std::vector`, references, Eigen, lambdas, classes, exceptions, `std::complex`, Google Test, debugger | Dynamic systems, first- and second-order responses, Euler and RK4, state-space form, poles and stability | `StateSpace`, integrators, tests, `ex01` |
| 2 Representations | 11–16 | Operator overloading, friends, factories, inheritance, Eigen blocks, file I/O, string streams, iterators, algorithms | Laplace domain, polynomials, transfer functions, realizations, step/impulse response, specifications | `Polynomial`, `TransferFunction`, `LTISystem`, conversions, plotter, `TimeResponse` |
| 3 Frequency domain | 17–22 | `std::optional`, overload sets, operators, sorting predicates, header organisation | Frequency response, Bode, Nyquist, margins, interconnections, sensitivity, minimum phase, delay, root locus, Routh–Hurwitz | `analysis` module, `csl.hpp`, `ex02` |
| 4 Classical design | 23–28 | Abstract bases, configs, smart pointers, RAII, memory model, tuples, root finding | PID, sampled-data simulation, anti-windup, tuning, lead/lag, loop shaping | `Controller`, `PID`, `Simulator`, `Tuning`, `LeadLag`, `ex03` |
| 5 Discrete time and DSP | 29–36 | `enum class`, matrix functions, bitwise arithmetic, in-place algorithms, templates | Sampling, discretization, z-domain, FFT, spectral estimation, filters, digital control | `c2d`/`d2c`, `FFT`, `Spectral`, `DigitalFilter`, `FilterDesign`, `ex04` |
| 6 State-space design | 37–43 | Numerical rank, Kronecker products, `Eigen::Map`, Cholesky, Rule of Zero, move semantics | Controllability, observability, pole placement, observers, Lyapunov equations, Gramians, integral action | `LinearAlgebra`, `StateFeedback`, `Estimator`, observer, `ex05` |
| 7 Optimal control | 44–49 | Block matrices, complex eigenvectors, convergence loops, `std::function` dynamics, grids | LQR, CARE, DARE, LQR properties, nonlinear systems, dynamic programming | `solveARE`, `LQR`, `NonlinearSystem`, DP, cart–pole, `ex06` |
| 8 Estimation | 50–55 | `<random>`, seeded RNGs, Cholesky factors, optional callbacks | Noise, Kalman filter, steady state, LQG, EKF, UKF, Monte Carlo | `signals` noise, `KalmanFilter`, `LQG`, `EKF`, `UKF`, `ex07` |
| 9 Identification | 56–59 | Regression matrices, QR, SVD, Hankel matrices | ARX, RLS, ETFE, rational fit, ERA, validation | `sysid` module, `ex08` |
| 10 Robust, predictive, nonlinear | 60–66 | Composition of library components; QP duality | Uncertainty, small gain, norms, QP, MPC, iLQR, H∞ synthesis | `Robustness`, `hinfnorm`, `solveQP`, `MPC`, `iLQR`, `HInfinity`, `ex09` |
| 11 Release | 67–70 | Install/export, CI, formatting, documentation | End-to-end pipeline | package, CI, docs, `ex10`, tag |

### 6.2 Day Entry Template

Each day in Chapters 7–17 has the following sections.

| Section | Content |
|---|---|
| **Objectives** | The C++ concepts and the control/DSP concepts the day introduces |
| **Background** | The theory required, with numbered equations |
| **Implementation** | Files created or modified, and listings of the code that is the subject of the day |
| **Verification** | Numbered checks `V<day>.<n>` with expected values; each becomes a unit test unless marked "example only" |
| **Exit criteria** | Conditions that must hold before the day is marked complete in Table 6-2 |
| **References** | Citations into Appendix B, with sections |

### 6.3 Progress Tracker

Table 6-2 is the project tracker. A day is marked complete when all its exit criteria are met and its commit exists. Phase rows are marked when the phase review (last day of the phase) is written and the phase tag is created.

**Table 6-2 — Progress tracker**

| Day | Title | Principal deliverable | Done |
|---|---|---|---|
| 1 | Toolchain and compilation pipeline | `sandbox/day01`, `.gitignore` | ☐ |
| 2 | Types, loops, Euler simulation of E1 | `sandbox/day02` | ☐ |
| 3 | Functions, headers, linking | `sandbox/day03` | ☐ |
| 4 | CMake and library skeleton; E2 | `CMakeLists.txt`, `ODESolver` (scalar), `ex01` | ☐ |
| 5 | `std::vector`, references, CSV output | `plot/CSV` | ☐ |
| 6 | Eigen, namespaces, state-space form | `core/Types`, `plants.hpp` | ☐ |
| 7 | Lambdas, generic integrator (Euler, RK4) | `simulation/ODESolver`, `SimResult` | ☐ |
| 8 | Classes, exceptions, `StateSpace` | `systems/StateSpace`, `core/Exceptions` | ☐ |
| 9 | Complex numbers, poles, stability | `StateSpace::poles` | ☐ |
| 10 | Google Test, debugger, Phase 1 review | `tests/` | ☐ |
| **Phase 1** | Foundations complete | tag `phase-1` | ☐ |
| 11 | Operator overloading, `Polynomial` | `core/Polynomial` | ☐ |
| 12 | `TransferFunction`, companion-matrix roots | `systems/TransferFunction` | ☐ |
| 13 | Inheritance, `LTISystem` | `systems/LTISystem` | ☐ |
| 14 | Realizations `tf2ss`, `ss2tf` | `systems/Conversions` | ☐ |
| 15 | Plotter; step, impulse, lsim | `plot/Figure`, `plot/Plots`, `simulation/TimeResponse` | ☐ |
| 16 | Algorithms, specifications, signals | `stepInfo`, `signals/Signals` | ☐ |
| **Phase 2** | Representations complete | tag `phase-2` | ☐ |
| 17 | Frequency response, Bode | `analysis/FrequencyResponse`, `core/MathUtils` | ☐ |
| 18 | Nyquist, margins | `margins`, `nyquist` | ☐ |
| 19 | Interconnections, sensitivity | `systems/Interconnect` | ☐ |
| 20 | Minimum phase, delay, Padé | `pade`, `isMinimumPhase` | ☐ |
| 21 | Root locus | `analysis/RootLocus` | ☐ |
| 22 | Routh–Hurwitz, umbrella header, review | `analysis/Stability`, `csl.hpp`, `ex02` | ☐ |
| **Phase 3** | Frequency domain complete | tag `phase-3` | ☐ |
| 23 | `Controller` base, PID | `control/Controller`, `control/PID` | ☐ |
| 24 | Memory model, smart pointers, `Simulator` | `simulation/Simulator` | ☐ |
| 25 | Anti-windup, setpoint weighting | `PID` complete | ☐ |
| 26 | Tuning rules, relay auto-tuning | `control/Tuning`, `control/Relay` | ☐ |
| 27 | Lead/lag design, bisection | `control/LeadLag`, `core/NumericalMethods` | ☐ |
| 28 | Loop shaping, gang of four, review | sensitivity functions, `ex03` | ☐ |
| **Phase 4** | Classical design complete | tag `phase-4` | ☐ |
| 29 | Sampling, ZOH discretization | `c2d`, `d2c` | ☐ |
| 30 | z-domain, Tustin, MPZ, TF controller | `c2d` (TF), `control/TransferFunctionController` | ☐ |
| 31 | DFT and FFT | `dsp/FFT` | ☐ |
| 32 | Windows, periodogram, Welch | `dsp/Spectral` | ☐ |
| 33 | Filter structures (IIR, FIR) | `dsp/DigitalFilter` | ☐ |
| 34 | Filter design, templates | `dsp/FilterDesign` | ☐ |
| 35 | Digital control experiments | `Simulator` measurement filter | ☐ |
| 36 | Phase 5 review | `ex04` | ☐ |
| **Phase 5** | Discrete time and DSP complete | tag `phase-5` | ☐ |
| 37 | Controllability, observability | `core/LinearAlgebra` | ☐ |
| 38 | Pole placement | `control/StateFeedback` | ☐ |
| 39 | Observer, `Estimator`, separation | `estimation/Estimator`, `LuenbergerObserver` | ☐ |
| 40 | Lyapunov equations, Gramians, H2 norm | `lyap`, `dlyap`, `analysis/Norms` | ☐ |
| 41 | Feedforward and integral action | `IntegralStateFeedbackController` | ☐ |
| 42 | Copy/move audit | static assertions, benchmark | ☐ |
| 43 | Phase 6 review | `ex05` | ☐ |
| **Phase 6** | State-space design complete | tag `phase-6` | ☐ |
| 44 | LQR and CARE | `solveARE`, `control/LQR` | ☐ |
| 45 | Discrete LQR and DARE | `solveDARE`, `dlqr`, `finiteHorizonLQR` | ☐ |
| 46 | LQR properties, Bryson, integral LQR | `lqrLoopTransfer`, `lqrIntegral` | ☐ |
| 47 | Nonlinear systems, linearization | `systems/NonlinearSystem`, `numericalJacobian`, E5 | ☐ |
| 48 | Dynamic programming | `control/DynamicProgramming` | ☐ |
| 49 | Phase 7 review | `ex06` | ☐ |
| **Phase 7** | Optimal control complete | tag `phase-7` | ☐ |
| 50 | Randomness, noise, PRBS, Monte Carlo | `signals` noise, `simulation/MonteCarlo` | ☐ |
| 51 | Kalman filter | `estimation/KalmanFilter` | ☐ |
| 52 | Steady-state KF, LQG | `steadyStateKalman`, `control/LQG` | ☐ |
| 53 | Extended Kalman filter | `estimation/ExtendedKalmanFilter` | ☐ |
| 54 | Unscented Kalman filter | `estimation/UnscentedKalmanFilter` | ☐ |
| 55 | Phase 8 review | `ex07` | ☐ |
| **Phase 8** | Estimation complete | tag `phase-8` | ☐ |
| 56 | ARX least squares | `sysid/ARX`, `sysid/Validation` | ☐ |
| 57 | Recursive least squares | `sysid/RLS` | ☐ |
| 58 | ETFE, coherence, rational fit | `etfe`, `sysid/FrequencyDomainID` | ☐ |
| 59 | ERA, validation, review | `sysid/Subspace`, `ex08` | ☐ |
| **Phase 9** | Identification complete | tag `phase-9` | ☐ |
| 60 | Uncertainty, small gain | `analysis/Robustness` | ☐ |
| 61 | H∞ norm | `hinfnorm` | ☐ |
| 62 | Quadratic programming | `solveQP` | ☐ |
| 63 | MPC formulation | `control/MPC` | ☐ |
| 64 | Constrained MPC, comparisons | `MPC` complete, `ex09` | ☐ |
| 65 | Iterative LQR | `control/iLQR` | ☐ |
| 66 | H∞ synthesis | `control/HInfinity` | ☐ |
| **Phase 10** | Robust, predictive, nonlinear complete | tag `phase-10` | ☐ |
| 67 | Install and export | `cmake/`, consumer test | ☐ |
| 68 | Documentation | `README.md`, tutorials, API comments | ☐ |
| 69 | Continuous integration | `.github/workflows/ci.yml`, `.clang-format` | ☐ |
| 70 | Full pipeline, retrospective | `ex10`, `docs/RETROSPECTIVE.md`, tag `v2.0.0` | ☐ |
| **Phase 11** | Release complete | tag `v2.0.0` | ☐ |

---
## 7. Phase 1 — Foundations (Days 1–10)

**Phase objective.** Establish the toolchain and build system; implement the ODE integrators and the `StateSpace` class with validated construction, dc gain, poles and stability; establish the test suite. Days 1–3 use `sandbox/`; the repository layout of Figure 3-1 begins on Day 4.

### Day 1 — Toolchain and Compilation Pipeline

**Objectives.** C++: the roles of preprocessor, compiler and linker; the structure of a minimal program. Control: the vocabulary of dynamic systems (state, input, output, open and closed loop) and the reference models of Table 5-2.

**Background.** A C++ program is produced in three stages (Figure 7-1). The preprocessor expands `#include` and other directives, yielding a *translation unit*. The compiler converts one translation unit into an *object file* containing machine code and a symbol table of defined and referenced names. The linker resolves every referenced name to exactly one definition across all object files and libraries and writes the executable.

**Figure 7-1 — Compilation pipeline**

```
 hello.cpp ──preprocess──▶ translation unit ──compile (cl /c)──▶ hello.obj ──link──▶ hello.exe
                                                                    ▲
                                                 C++ runtime library (automatic)
```

A dynamic system is one whose current output depends on past inputs. Its *state* `x` is the minimal information that, with future inputs, determines all future outputs; its *input* `u` is manipulated; its *output* `y` is measured. In open-loop operation `u` is chosen in advance; in closed-loop operation `u` is computed from the error `e = r − y` between a reference `r` and the measured output.

**Implementation.**

| File | Action |
|---|---|
| `sandbox/day01/hello.cpp` | Create (Listing 7-1) |
| `.gitignore` | Create with entries `build/`, `build-*/`, `*.obj`, `*.exe`, `*.pdb`, `*.ilk`, `.vs/`, `.vscode/`, `*.html`, `*.csv` |

Environment setup: confirm `cl` and `cmake --version` run in the Developer PowerShell; install the VS Code extensions of Table 3-1.

**Listing 7-1 — `sandbox/day01/hello.cpp`**

```cpp
#include <iostream>

int main() {
    std::cout << "Control Systems Library - Day 1\n";
    return 0;
}
```

Build in one step and in two explicit steps:

```powershell
cl /EHsc /W4 /std:c++17 hello.cpp              # compile and link
cl /EHsc /W4 /std:c++17 /c hello.cpp           # compile only -> hello.obj
link hello.obj /OUT:hello2.exe                 # link -> hello2.exe
```

`/EHsc` enables standard exception handling, `/W4` the high warning level, `/std:c++17` the language standard. GCC equivalent: `g++ -std=c++17 -Wall -Wextra -c hello.cpp && g++ hello.o -o hello`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V1.1 | Run `hello.exe` and `hello2.exe` | Both print the line |
| V1.2 | Rename `main` to `mian`; compile with `/c`; link | Compilation succeeds; link fails with "unresolved external symbol main" |

**Exit criteria.**
- [ ] `cl` and `cmake` run from the Developer PowerShell; VS Code extensions installed
- [ ] V1.1–V1.2 observed; the stage producing the V1.2 error is identified
- [ ] Commit `Day 1: toolchain and hello`

**References.** [R1, §1.1–1.2]; [R2, ch. 0–1]; [R8, ch. 1].

---

### Day 2 — Types, Loops and Euler Simulation of E1

**Objectives.** C++: fundamental types, initialisation, `const`, arithmetic, `for`/`while`/`if`, `<cmath>`, formatted output. Control: the first-order system E1 (Eq. 5-1), its step response and time constant; Euler's method and its stability limit.

**Background.** With a unit step input from rest, the solution of Eq. (5-1) is

```
y(t) = K (1 − e^{−t/τ})                                                          (7-1)
```

reaching 63.2 % of `K` at `t = τ` and 98.2 % at `t = 4τ`. Euler's method replaces the derivative by a forward difference:

```
y_{k+1} = y_k + Δt · f(y_k, u_k),    f(y, u) = (−y + K u) / τ                      (7-2)
```

Its global error is proportional to `Δt` (first order). For E1 it has the closed form `y_k = K(1 − (1 − Δt/τ)^k)`, which is bounded only for `Δt < 2τ`: a simulation may be unstable although the simulated system is stable.

Language notes: `double` is used for all physical quantities (`float` is prohibited, Table 3-2); brace initialisation `double tau{2.0}` rejects narrowing conversions; integer division truncates (`1/2 == 0`); `std::setprecision` from `<iomanip>` controls output precision.

**Implementation.**

| File | Action |
|---|---|
| `sandbox/day02/thermal_euler.cpp` | Create (Listing 7-2) |

**Listing 7-2 — `sandbox/day02/thermal_euler.cpp`**

```cpp
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    const double tau = 2.0, K = 3.0, u = 1.0;         // E1 parameters, unit step input
    const double dt = 0.1, t_end = 10.0;
    const int N = static_cast<int>(t_end / dt);

    double y = 0.0, t = 0.0;
    std::cout << std::fixed << std::setprecision(6) << "t,y_euler,y_exact,error\n";
    for (int k = 0; k <= N; ++k) {
        const double y_exact = K * (1.0 - std::exp(-t / tau));
        std::cout << t << "," << y << "," << y_exact << "," << (y - y_exact) << "\n";
        const double dydt = (-y + K * u) / tau;         // Eq. (7-2)
        y += dt * dydt;
        t += dt;
    }
    return 0;
}
```

Run with `.\thermal_euler.exe > out.csv` and inspect the file.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V2.1 | `y(2.0)`, `Δt = 0.1` | Euler `1.9245`; exact `1.8964`; error `+0.028` |
| V2.2 | `y(2.0)`, `Δt = 0.01` | Euler `1.8991`; error `+0.0027` (tenfold reduction) |
| V2.3 | `Δt = 3.0`; `Δt = 5.0` | Oscillation; divergence (limit `2τ = 4 s`) |
| V2.4 | Vary `K`, `τ` | 63.2 % of `K` at `t = τ` in every case |

**Exit criteria.**
- [ ] Compiles with `/W4` without warnings
- [ ] V2.1–V2.4 observed; the Euler stability limit is stated
- [ ] Commit

**References.** [R1, §2.1–2.3, §5.1–5.4]; [R2, ch. 4–5, 8]; [R8, §2.1–2.2]; [R9, §2.1].

---

### Day 3 — Functions, Headers and Linking

**Objectives.** C++: declarations versus definitions, headers and source files, `#include` semantics, include guards, translation units, the One Definition Rule, separate compilation and linking. Control: Euler's method as an algorithm with an order of accuracy and a stability region.

**Background.** A *declaration* introduces a name and its type; a *definition* additionally supplies the body. A name may be declared many times but defined once per program (One Definition Rule). Headers (`.hpp`) carry declarations and are textually inserted by `#include`; sources (`.cpp`) carry definitions and are compiled independently. `#pragma once` prevents repeated insertion of a header within one translation unit. The compiler is satisfied by a declaration; the linker requires the definition.

For the test equation `ẏ = λy` with `λ < 0`, Euler gives `y_{k+1} = (1 + λΔt) y_k`, stable only when `|1 + λΔt| < 1`, i.e. `Δt < 2/|λ|`. For E1, `λ = −1/τ`, reproducing the limit observed on Day 2.

**Implementation.**

| File | Action |
|---|---|
| `sandbox/day03/euler.hpp`, `euler.cpp`, `main.cpp` | Create (Listing 7-3) |

**Listing 7-3 — Three-file program**

```cpp
// euler.hpp
#pragma once
double eulerStepThermal(double y, double u, double dt, double tau, double K);   // one Euler step of Eq. (5-1)
double eulerStabilityLimit(double tau);                                        // 2 tau

// euler.cpp
#include "euler.hpp"
double eulerStepThermal(double y, double u, double dt, double tau, double K) {
    return y + dt * ((-y + K * u) / tau);
}
double eulerStabilityLimit(double tau) { return 2.0 * tau; }

// main.cpp
#include <cmath>
#include <iomanip>
#include <iostream>
#include "euler.hpp"
int main() {
    const double tau = 2.0, K = 3.0, u = 1.0, dt = 0.1, t_end = 10.0;
    std::cout << "Euler stable for dt < " << eulerStabilityLimit(tau) << " s\n";
    double y = 0.0;
    std::cout << std::fixed << std::setprecision(6);
    for (double t = 0.0; t <= t_end + 1e-12; t += dt) {
        std::cout << t << "," << y << "," << K * (1.0 - std::exp(-t / tau)) << "\n";
        y = eulerStepThermal(y, u, dt, tau, K);
    }
    return 0;
}
```

Build:

```powershell
cl /EHsc /W4 /std:c++17 /c euler.cpp
cl /EHsc /W4 /std:c++17 /c main.cpp
link euler.obj main.obj /OUT:day03.exe
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V3.1 | Output for `Δt = 0.1` | Identical to Day 2 |
| V3.2 | Link `main.obj` alone | Linker: unresolved external `eulerStepThermal` |
| V3.3 | Remove `#include "euler.hpp"` from `main.cpp` | Compiler: identifier not found |
| V3.4 | Duplicate the definition of `eulerStepThermal` in `main.cpp` | Linker: symbol already defined (ODR violation) |

**Exit criteria.**
- [ ] V3.1–V3.4 reproduced; each error is attributed to its stage
- [ ] Commit

**References.** [R1, §2.6.3, §6.1–6.3]; [R2, ch. 2].

---

### Day 4 — CMake, Library Skeleton and the Second-Order System

**Objectives.** C++: CMake targets, `add_library`/`add_executable`, `target_include_directories`, `target_link_libraries` and the `PUBLIC`/`PRIVATE`/`INTERFACE` scopes, out-of-source builds, build types, generator expressions. Control: the mass–spring–damper E2 (Eq. 5-2) and the state vector.

**Background.** CMake generates build files (for Ninja, Visual Studio, Make) from `CMakeLists.txt`. A *target* is an executable or library carrying usage requirements (include directories, compile features, dependencies). `target_link_libraries(app PRIVATE csl)` links `csl` into `app` and propagates `csl`'s `PUBLIC` requirements. A static library is an archive of object files; linking against it is equivalent to linking the object files directly. `CMAKE_BUILD_TYPE` selects `Debug` (no optimisation, symbols) or `Release` (`/O2`).

E2 is second order; with state `x₁ = x`, `x₂ = ẋ`:

```
ẋ₁ = x₂,    ẋ₂ = (F − c x₂ − k x₁) / m                                           (7-3)
```

Both derivatives are evaluated from the current state before either state is updated. With the parameters of Table 5-2, `ω_n = 2 rad/s`, `ζ = 0.125`.

**Implementation.**

| File | Action |
|---|---|
| `CMakeLists.txt` | Create (Listing 7-4) |
| `examples/CMakeLists.txt` | Create (Listing 7-5) |
| `include/csl/simulation/ODESolver.hpp`, `src/simulation/ODESolver.cpp` | Create scalar Euler step (Listing 7-6); replaced on Day 7 |
| `examples/ex01_first_order_sim.cpp` | Create (Listing 7-7) |

**Listing 7-4 — Root `CMakeLists.txt` (Day 4 state)**

```cmake
cmake_minimum_required(VERSION 3.20)
project(csl VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
if(MSVC)
    add_compile_options(/W4 /permissive- /EHsc /Zc:__cplusplus)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(csl STATIC src/simulation/ODESolver.cpp)
add_library(csl::csl ALIAS csl)
target_include_directories(csl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(csl PUBLIC cxx_std_17)

option(CSL_BUILD_EXAMPLES "Build example programs" ON)
if(CSL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
```

**Listing 7-5 — `examples/CMakeLists.txt`**

```cmake
add_executable(ex01_first_order_sim ex01_first_order_sim.cpp)
target_link_libraries(ex01_first_order_sim PRIVATE csl::csl)
```

**Listing 7-6 — Scalar Euler step (temporary)**

```cpp
// include/csl/simulation/ODESolver.hpp
#pragma once
namespace csl::simulation {
double eulerStepScalar(double y, double dydt, double dt);
}
// src/simulation/ODESolver.cpp
#include "csl/simulation/ODESolver.hpp"
namespace csl::simulation {
double eulerStepScalar(double y, double dydt, double dt) { return y + dt * dydt; }
}
```

**Listing 7-7 — `examples/ex01_first_order_sim.cpp` (Day 4 state)**

```cpp
#include <iomanip>
#include <iostream>
#include "csl/simulation/ODESolver.hpp"

int main() {
    const double m = 1.0, c = 0.5, k = 4.0, F = 1.0;   // E2, unit step force
    const double dt = 0.001, t_end = 20.0;
    double x1 = 0.0, x2 = 0.0;
    std::cout << std::fixed << std::setprecision(6) << "t,x,v\n";
    for (double t = 0.0; t <= t_end; t += dt) {
        std::cout << t << "," << x1 << "," << x2 << "\n";
        const double dx1 = x2;                              // Eq. (7-3), from the current state
        const double dx2 = (F - c * x2 - k * x1) / m;
        x1 = csl::simulation::eulerStepScalar(x1, dx1, dt);
        x2 = csl::simulation::eulerStepScalar(x2, dx2, dt);
    }
    return 0;
}
```

Build with Listing 3-1; redirect the example's output to `msd.csv`. Inspect `build/build.ninja` for the generated compile and link commands (`cmake --build build -v` prints them).

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V4.1 | Build succeeds; compile/link lines located in `build.ninja` | — |
| V4.2 | Final position in `msd.csv` | `F/k = 0.25` |
| V4.3 | Period of oscillation; first peak | `2π/ω_d = 3.17 s`; `0.418` at `t ≈ 1.58 s` |

**Exit criteria.**
- [ ] V4.1–V4.3 observed; `PUBLIC` versus `PRIVATE` scope can be stated
- [ ] Commit

**References.** [R5, tutorial steps 1–2]; [R9, §2.1]; [R8, §2.2].

---

### Day 5 — `std::vector`, References and CSV Output

**Objectives.** C++: `std::vector`, range-based `for`, references and `const&` parameters, `std::string`, `std::ofstream`, `std::max_element`. Control: the underdamped, critically damped and overdamped regimes of a second-order system.

**Background.** `std::vector<T>` is a resizable, self-managing array; `for (const double& v : vec)` iterates without copying. A reference `T&` aliases an existing object; `const T&` parameters allow read access without copying and are the standard way to pass Eigen objects from Day 6 onward. `std::ofstream` writes to a file with the same operators as `std::cout` and closes on scope exit.

For `ẍ + 2ζω_n ẋ + ω_n² x = ω_n² u`: `ζ < 1` oscillates at `ω_d = ω_n√(1 − ζ²)` with overshoot `M_p = exp(−πζ/√(1 − ζ²))`; `ζ = 1` is the fastest non-oscillatory response; `ζ > 1` has two real time constants.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/plot/CSV.hpp`, `src/plot/CSV.cpp` | Create (Listing 7-8); add the source to `add_library` |
| `examples/ex01_first_order_sim.cpp` | Rewrite: simulate E2 for `ζ ∈ {0.125, 0.5, 1, 2}` into `std::vector<double>` columns; write `msd_zeta_sweep.csv`; print peak and overshoot per `ζ` |

**Listing 7-8 — `writeCSV` (vector overload)**

```cpp
// include/csl/plot/CSV.hpp
#pragma once
#include <string>
#include <vector>
namespace csl::plot {
void writeCSV(const std::string& path, const std::vector<std::string>& headers,
              const std::vector<std::vector<double>>& columns);
}

// src/plot/CSV.cpp
#include "csl/plot/CSV.hpp"
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace csl::plot {
void writeCSV(const std::string& path, const std::vector<std::string>& headers,
              const std::vector<std::vector<double>>& columns) {
    if (headers.size() != columns.size()) throw std::runtime_error("writeCSV: headers/columns count mismatch");
    std::ofstream file(path);
    if (!file) throw std::runtime_error("writeCSV: cannot open " + path);
    file << std::setprecision(10);
    for (std::size_t j = 0; j < headers.size(); ++j) file << headers[j] << (j + 1 < headers.size() ? "," : "\n");
    const std::size_t rows = columns.empty() ? 0 : columns[0].size();
    for (std::size_t i = 0; i < rows; ++i)
        for (std::size_t j = 0; j < columns.size(); ++j) file << columns[j][i] << (j + 1 < columns.size() ? "," : "\n");
}
}
```

(`std::runtime_error` is replaced by the library's exception types on Day 8.)

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V5.1 | Overshoot for `ζ = 0.125, 0.5, 1, 2` | `67.2 %`, `16.3 %`, `0`, `0` (Euler, `Δt = 0.001`) |
| V5.2 | `msd_zeta_sweep.csv` opens in a spreadsheet; four curves have the expected shapes | — |

**Exit criteria.**
- [ ] `writeCSV` is part of `csl`; V5.1–V5.2 observed
- [ ] The parameter kinds `std::vector<double>&` (output) and `const std::vector<double>&` (input) are used correctly
- [ ] Commit

**References.** [R1, §2.3.1, §3.1–3.4, §8.1–8.2]; [R2, ch. 12, 16, 28]; [R9, §3.3–3.4]; [R8, §6.3].

---

### Day 6 — Eigen, Namespaces and the State-Space Form

**Objectives.** C++: namespaces, type aliases, `inline constexpr`, Eigen construction and arithmetic, function overloading, CMake `FetchContent`. Control: the state-space representation and the matrices of E1–E4 (Eq. 5-6).

**Background.** The state-space form

```
ẋ = A x + B u,    y = C x + D u                                                  (7-4)
```

separates structure (`A`, `B`, `C`, `D`) from data (`x`, `u`), so that one algorithm applies to any order `n`. `A` (`n×n`) holds the internal dynamics, `B` (`n×m`) the input coupling, `C` (`p×n`) the measured combinations, `D` (`p×m`) the direct feedthrough.

A namespace is a named scope; `using Real = double;` introduces an alias. `inline constexpr` permits a constant to be defined in a header included by many translation units. Eigen is header-only; it is fetched by CMake and exposed as an `INTERFACE` target carrying only an include directory.

**Implementation.**

| File | Action |
|---|---|
| `CMakeLists.txt` | Add Listing 7-9 before `add_library`; add `target_link_libraries(csl PUBLIC Eigen3::Eigen)` after it |
| `include/csl/core/Types.hpp` | Create (Listing 4-1) |
| `include/csl/plot/CSV.hpp`, `src/plot/CSV.cpp` | Add overload `writeCSV(path, headers, const Vector& t, const Matrix& data)` writing `t` as the first column and `data` column-wise (rows must equal `t.size()`) |
| `examples/plants.hpp` | Create (Listing 7-10) |
| `examples/ex01_first_order_sim.cpp` | Rewrite: Euler on `x ← x + Δt(Ax + Bu)` for E2 with Eigen; write `msd_eigen.csv` |

**Listing 7-9 — Eigen via `FetchContent`**

```cmake
include(FetchContent)
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR cmake_not_used)          # no CMakeLists.txt there: Eigen's own project is not added
FetchContent_MakeAvailable(eigen)
add_library(eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS eigen)
target_include_directories(eigen INTERFACE ${eigen_SOURCE_DIR})
```

**Listing 7-10 — `examples/plants.hpp` (Day 6 state; return type becomes `StateSpace` on Day 8)**

```cpp
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
inline ABCD dcMotor(double R = 1.0, double L = 0.5, double J = 0.01, double b = 0.1, double Kt = 0.01, double Kb = 0.01) {
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
}
```

Eigen usage required today: `Matrix A(2, 2); A << …;`, `Vector::Zero(n)`, `A * x`, `A.transpose()`, `A.inverse()`, `A(i, j)`, `x(i)`, `A.rows()`, `X.row(k) = x.transpose()`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V6.1 | Eigen is downloaded to `build/_deps/eigen-src`; `Types.hpp` compiles | — |
| V6.2 | `msd_eigen.csv` versus Day 4's `msd.csv` | Identical to printed precision |
| V6.3 | `(p.C * (-p.A.inverse() * p.B))(0, 0)` for E3 | `0.0999` (dc gain, Table 5-3) |

**Exit criteria.**
- [ ] V6.1–V6.3 observed; the prohibition of `using namespace` in headers is understood
- [ ] Commit

**References.** [R4, "Getting started", "The Matrix class"]; [R1, §2.5, §6.4, §18.2]; [R8, §2.2, §6.1]; [R9, §7.2].

---

### Day 7 — Lambdas, `std::function` and the Generic Integrator

**Objectives.** C++: lambdas and capture lists, `std::function`, `auto`, `struct` for plain data, `enum class` as a method selector. Control: fourth-order Runge–Kutta integration, order of accuracy, step-size selection.

**Background.** RK4 averages four slope evaluations per step:

```
k₁ = f(t, x),  k₂ = f(t + Δt/2, x + Δt k₁/2),  k₃ = f(t + Δt/2, x + Δt k₂/2),  k₄ = f(t + Δt, x + Δt k₃)
x_{k+1} = x_k + (Δt/6)(k₁ + 2k₂ + 2k₃ + k₄)                                        (7-5)
```

Global error is `O(Δt⁴)`; its stability interval on `ẏ = λy` is `|λΔt| ≲ 2.78`. Step-size rule: at least 20 steps per fastest time constant and per fastest oscillation period.

A lambda `[&](Real t, const Vector& x) { … }` is an anonymous function capturing enclosing variables by reference; captured objects must outlive the lambda. `std::function<Vector(Real, const Vector&)>` stores any callable of that signature.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/simulation/SimResult.hpp` | Create (Listing 4-3, `SimResult`) |
| `include/csl/simulation/ODESolver.hpp`, `src/simulation/ODESolver.cpp` | Replace with Listing 7-11 |
| `examples/ex01_first_order_sim.cpp` | Rewrite: integrate E1 with Euler and RK4 for `Δt ∈ {0.5, 0.1, 0.01}`; print errors at `t = 10` |

**Listing 7-11 — Generic integrator**

```cpp
// include/csl/simulation/ODESolver.hpp
#pragma once
#include <functional>
#include "csl/core/Types.hpp"
#include "csl/simulation/SimResult.hpp"
namespace csl::simulation {
using Dynamics = std::function<Vector(Real t, const Vector& x)>;
enum class Integrator { Euler, RK4 };
Vector eulerStep(const Dynamics& f, Real t, const Vector& x, Real dt);
Vector rk4Step  (const Dynamics& f, Real t, const Vector& x, Real dt);
SimResult integrate(const Dynamics& f, const Vector& x0, Real t0, Real tf, Real dt,
                    Integrator method = Integrator::RK4);   // N = floor((tf - t0)/dt) + 1 samples
}

// src/simulation/ODESolver.cpp
#include "csl/simulation/ODESolver.hpp"
#include <cmath>
#include <stdexcept>
namespace csl::simulation {
Vector eulerStep(const Dynamics& f, Real t, const Vector& x, Real dt) { return x + dt * f(t, x); }

Vector rk4Step(const Dynamics& f, Real t, const Vector& x, Real dt) {
    const Vector k1 = f(t, x);
    const Vector k2 = f(t + 0.5 * dt, x + 0.5 * dt * k1);
    const Vector k3 = f(t + 0.5 * dt, x + 0.5 * dt * k2);
    const Vector k4 = f(t + dt,       x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

SimResult integrate(const Dynamics& f, const Vector& x0, Real t0, Real tf, Real dt, Integrator method) {
    if (dt <= 0.0 || tf < t0) throw std::invalid_argument("integrate: dt > 0 and tf >= t0 required");
    const Index N = static_cast<Index>(std::floor((tf - t0) / dt + 1e-9)) + 1;
    SimResult r;
    r.t.resize(N);
    r.x.resize(N, x0.size());
    Vector x = x0;
    for (Index k = 0; k < N; ++k) {
        const Real t = t0 + k * dt;
        r.t(k) = t;
        r.x.row(k) = x.transpose();
        if (k + 1 < N) x = (method == Integrator::RK4) ? rk4Step(f, t, x, dt) : eulerStep(f, t, x, dt);
    }
    return r;
}
}
```

Example usage: `Dynamics f = [&](Real, const Vector& x) { return p.A * x + p.B * u; };`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V7.1 | E1 error at `t = 10`, Euler, `Δt = 0.5, 0.1, 0.01` | `+1.1e-2`, `+2.4e-3`, `+2.5e-4` |
| V7.2 | E1 error at `t = 10`, RK4, same steps | `−4e-6`, `−5e-9`, `≈ −5e-13` |
| V7.3 | `integrate` sample count and final time, `t0 = 0`, `tf = 1`, `Δt = 0.25` | `N = 5`, `t(4) = 1.0` |

**Exit criteria.**
- [ ] Listing 7-6 removed; V7.1–V7.3 observed
- [ ] Commit

**References.** [R1, §10.3.2–10.3.3, §14.8.3]; [R2, ch. 20]; [R42, ch. 25].

---

### Day 8 — Classes, Exceptions and the `StateSpace` Class

**Objectives.** C++: class definition, constructors and member-initialiser lists, invariants, `const` member functions, `explicit`, header/source split for classes, exceptions and the library's exception hierarchy, anonymous namespaces. Control: dimensions of a state-space model, direct feedthrough, dc gain.

**Background.** A class couples data to the guarantees about it. The constructor establishes the *invariant* (for `StateSpace`: `A` square, `B` has `n` rows, `C` has `n` columns, `D` is `p×m`, `T_s ≥ 0`); no other member function need re-check it. Exceptions (`throw`) abandon the current computation and unwind the stack to a matching `catch`; they are used for precondition violations only (C13).

The steady state under constant `u` has `ẋ = 0`, hence

```
G(0) = D − C A⁻¹ B           (continuous);     G(1) = D + C (I − A)⁻¹ B           (discrete)    (7-6)
```

`G(0)` is the *dc gain*, the value to which the step response settles. If `A` (or `I − A`) is singular the gain is infinite and the function throws `NumericalError`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/core/Exceptions.hpp` | Create (Listing 4-6) |
| `include/csl/systems/StateSpace.hpp`, `src/systems/StateSpace.cpp` | Create (Listing 7-12) |
| `examples/plants.hpp` | Return `csl::systems::StateSpace` from each factory; delete `ABCD` |
| `src/plot/CSV.cpp` | Replace `std::runtime_error` with `InvalidArgument` |

**Listing 7-12 — `StateSpace` (Day 8 state; derives from `LTISystem` on Day 13)**

```cpp
// include/csl/systems/StateSpace.hpp
#pragma once
#include <vector>
#include "csl/core/Types.hpp"
namespace csl::systems {
class StateSpace {
public:
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
    Vector derivative(const Vector& x, const Vector& u) const;   // A x + B u  (continuous: dx/dt; discrete: x[k+1])
    Vector output    (const Vector& x, const Vector& u) const;   // C x + D u
    Matrix dcGain() const;                                       // Eq. (7-6)
private:
    Matrix A_, B_, C_, D_;
    Real   Ts_;
};
}

// src/systems/StateSpace.cpp
#include "csl/systems/StateSpace.hpp"
#include <string>
#include <utility>
#include "csl/core/Exceptions.hpp"
namespace csl::systems {
namespace {
std::string shape(const Matrix& M) { return std::to_string(M.rows()) + "x" + std::to_string(M.cols()); }
}
StateSpace::StateSpace(Matrix A, Matrix B, Matrix C, Matrix D, Real Ts)
    : A_(std::move(A)), B_(std::move(B)), C_(std::move(C)), D_(std::move(D)), Ts_(Ts) {
    if (A_.rows() != A_.cols()) throw DimensionMismatch("StateSpace: A must be square, got " + shape(A_));
    if (B_.rows() != A_.rows()) throw DimensionMismatch("StateSpace: B must have " + std::to_string(A_.rows()) + " rows, got " + shape(B_));
    if (C_.cols() != A_.cols()) throw DimensionMismatch("StateSpace: C must have " + std::to_string(A_.cols()) + " cols, got " + shape(C_));
    if (D_.rows() != C_.rows() || D_.cols() != B_.cols()) throw DimensionMismatch("StateSpace: D must be " + std::to_string(C_.rows()) + "x" + std::to_string(B_.cols()) + ", got " + shape(D_));
    if (Ts_ < 0.0) throw InvalidArgument("StateSpace: Ts must be >= 0");
}
Vector StateSpace::derivative(const Vector& x, const Vector& u) const {
    if (x.size() != order() || u.size() != inputs()) throw DimensionMismatch("StateSpace::derivative: bad x or u size");
    return A_ * x + B_ * u;
}
Vector StateSpace::output(const Vector& x, const Vector& u) const { return C_ * x + D_ * u; }
Matrix StateSpace::dcGain() const {
    const Index n = order();
    const Matrix M = isContinuous() ? A_ : Matrix(Matrix::Identity(n, n) - A_);
    Eigen::FullPivLU<Matrix> lu(M);
    if (!lu.isInvertible()) throw NumericalError("StateSpace::dcGain: pole at s = 0 (or z = 1); dc gain is infinite");
    const Matrix X = lu.solve(B_);                             // solve rather than invert
    return isContinuous() ? Matrix(D_ - C_ * X) : Matrix(D_ + C_ * X);
}
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V8.1 | `plants::dcMotor().dcGain()(0,0)` | `2/20.02` to `1e-12` |
| V8.2 | `plants::massSpringDamper().dcGain()(0,0)` | `0.25` |
| V8.3 | Construction with `A` of size `3×2` | throws `DimensionMismatch` |

**Exit criteria.**
- [ ] V8.1–V8.3 observed (as a temporary `main`; converted to tests on Day 10)
- [ ] The class invariant and its enforcement point can be stated
- [ ] Commit

**References.** [R1, ch. 7, §5.6, §18.1]; [R2, ch. 14, 15, 27]; [R8, §6.1]; [R9, §7.2–7.3].

---

### Day 9 — Complex Numbers, Poles and Stability

**Objectives.** C++: `std::complex`, `Eigen::EigenSolver`, conversion between Eigen and `std::vector` by iterator range. Control: eigenvalues of `A` as poles; modes; stability; `ω_n`, `ζ`, `τ` from pole location; dominant poles.

**Background.** The homogeneous solution of `ẋ = Ax` is `x(t) = Σ cᵢ e^{λᵢt} vᵢ` over eigenpairs `(λᵢ, vᵢ)` of `A`. Each eigenvalue contributes a *mode*: real `λ < 0` decays with `τ = −1/λ`; a pair `σ ± jω` oscillates at `ω` with envelope `e^{σt}`, `ω_n = |λ|`, `ζ = −σ/|λ|`. A continuous system is stable if and only if every pole has `Re λ < 0`; a discrete system if and only if every pole has `|λ| < 1`. Poles on the boundary are marginal and classified as not stable. The poles nearest the boundary dominate the response; the 2 % settling time is approximately `4/|Re λ_dom|`.

**Implementation.**

| File | Action |
|---|---|
| `StateSpace.hpp`, `StateSpace.cpp` | Add `poles()` and `isStable()` (Listing 7-13); `isStable` moves to `LTISystem` on Day 13 |
| `examples/ex01_first_order_sim.cpp` | Print, for each reference model, poles with `ω_n`, `ζ`, `τ` |

**Listing 7-13 — Poles and stability**

```cpp
std::vector<Complex> StateSpace::poles() const {
    Eigen::EigenSolver<Matrix> es(A_, /*computeEigenvectors=*/false);
    const ComplexVector ev = es.eigenvalues();
    return std::vector<Complex>(ev.data(), ev.data() + ev.size());
}
bool StateSpace::isStable() const {
    for (const Complex& p : poles())
        if (isContinuous() ? (p.real() >= 0.0) : (std::abs(p) >= 1.0)) return false;
    return true;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V9.1 | Poles of E1, E2, E3, E4 | Table 5-3 values to `1e-8` |
| V9.2 | `isStable()` | E1–E3 true; E4 false |
| V9.3 | `A = [[0, 1], [−4, 0]]` | poles `±2j`; `isStable()` false; `integrate` gives a sustained sinusoid of period `π` |

**Exit criteria.**
- [ ] V9.1–V9.3 observed
- [ ] Commit

**References.** [R3, `std::complex`]; [R4, `EigenSolver`]; [R8, §6.1–6.2]; [R9, §3.1–3.3, §7.3].

---

### Day 10 — Google Test, Debugger and Phase 1 Review

**Objectives.** C++: Google Test (`TEST`, `EXPECT_NEAR`, `EXPECT_THROW`, `ASSERT_*`), CTest integration, breakpoints and stepping in the debugger, `assert`, warnings as errors.

**Background.** A unit test exercises one property and asserts a known result. `gtest_discover_tests` registers each `TEST` with CTest. `assert` (in `<cassert>`) checks internal invariants in Debug builds and is removed in Release; public preconditions use exceptions (C13).

**Implementation.**

| File | Action |
|---|---|
| `CMakeLists.txt` | Append Listing 7-14 (part 1) |
| `tests/CMakeLists.txt` | Create (Listing 7-14, part 2) |
| `tests/systems/test_StateSpace.cpp`, `tests/simulation/test_ODESolver.cpp`, `tests/plot/test_CSV.cpp` | Create (Listing 7-15 and analogous tests for V7.1–V7.3, V5-style CSV round trip) |
| `examples/ex01_first_order_sim.cpp` | Final form: integrate E1 with RK4; print poles and dc gain; write CSV |
| `sandbox/phase1_review.md` | Phase review (below) |

**Listing 7-14 — Test infrastructure**

```cmake
# root CMakeLists.txt (append)
option(CSL_BUILD_TESTS "Build unit tests" ON)
if(CSL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# tests/CMakeLists.txt
include(FetchContent)
FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)       # MSVC: match the C runtime of csl
FetchContent_MakeAvailable(googletest)
include(GoogleTest)
add_executable(csl_tests
    simulation/test_ODESolver.cpp
    systems/test_StateSpace.cpp
    plot/test_CSV.cpp)
target_link_libraries(csl_tests PRIVATE csl::csl GTest::gtest_main)
target_include_directories(csl_tests PRIVATE ${CMAKE_SOURCE_DIR}/examples)   # plants.hpp
gtest_discover_tests(csl_tests)
```

**Listing 7-15 — `tests/systems/test_StateSpace.cpp`**

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "csl/core/Exceptions.hpp"
#include "csl/systems/StateSpace.hpp"
#include "plants.hpp"
using namespace csl;
using csl::systems::StateSpace;

TEST(StateSpace, RejectsNonSquareA) {
    Matrix A = Matrix::Zero(3, 2), B = Matrix::Zero(3, 1), C = Matrix::Zero(1, 2), D = Matrix::Zero(1, 1);
    EXPECT_THROW(StateSpace(A, B, C, D), DimensionMismatch);
}
TEST(StateSpace, DcGainOfDcMotor) { EXPECT_NEAR(plants::dcMotor().dcGain()(0, 0), 2.0 / 20.02, 1e-12); }
TEST(StateSpace, PolesOfMassSpringDamper) {
    const auto p = plants::massSpringDamper().poles();
    ASSERT_EQ(p.size(), 2u);
    for (const Complex& z : p) {
        EXPECT_NEAR(z.real(), -0.25, 1e-9);
        EXPECT_NEAR(std::abs(z.imag()), std::sqrt(4.0 - 0.0625), 1e-9);
    }
}
TEST(StateSpace, StabilityClassification) {
    EXPECT_FALSE(plants::doubleIntegrator().isStable());
    EXPECT_TRUE(plants::dcMotor().isStable());
}
```

The ODE solver tests assert V7.1–V7.3 as error ratios: RK4 error at `Δt = 0.1` below `1e-6` and the ratio of errors at `Δt = 0.1` and `0.05` above 12; Euler ratio `2.0 ± 0.2`.

**Phase 1 review** (`sandbox/phase1_review.md`; one paragraph each): (1) the inputs and outputs of each stage of Figure 7-1; (2) why modifying `StateSpace.cpp` does not recompile `ODESolver.cpp`, and what modifying `StateSpace.hpp` triggers; (3) what `target_link_libraries(csl_tests PRIVATE csl::csl)` propagates besides the library file; (4) `const Matrix&` versus `Matrix` parameters, and why returning `SimResult` by value is efficient; (5) for poles `−1 ± 3j`: `ω_n`, `ζ`, settling time, response sketch, and Euler stability at `Δt = 1 s`; (6) the dc gain of E3 in physical terms, and its value if `b` doubles.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V10.1 | `ctest --test-dir build --output-on-failure` | All tests pass (≥ 8) |
| V10.2 | Breakpoint inside `rk4Step`; `k1`–`k4` inspected | — |

**Exit criteria.**
- [ ] V10.1–V10.2; phase review written
- [ ] Commit `Day 10: tests, Phase 1 complete`; tag `phase-1`

**References.** [R6, primer]; [R1, §6.5.3]; [R2, ch. 3].

---
## 8. Phase 2 — System Representations and Time-Domain Analysis (Days 11–16)

**Phase objective.** Implement `Polynomial`, `TransferFunction`, the `LTISystem` base, the conversions `tf2ss`/`ss2tf`, the SVG/HTML plotter, time responses (`step`, `impulse`, `lsim`), time-domain specifications and deterministic test signals.

### Day 11 — Operator Overloading and the `Polynomial` Class

**Objectives.** C++: operator overloading as free and member functions, `std::initializer_list`, the call operator, `friend`, static factory functions, approximate comparison. Control: the Laplace transform, the transfer function as a ratio of polynomials, the characteristic polynomial.

**Background.** For zero initial conditions, differentiation maps to multiplication by `s` under the Laplace transform, so Eq. (5-1) becomes `τ s Y(s) + Y(s) = K U(s)` and

```
H(s) = Y(s)/U(s) = K / (τ s + 1)                                                   (8-1)
```

Every linear constant-coefficient ODE yields a ratio of polynomials in `s`; the denominator is the *characteristic polynomial*, equal to `det(sI − A)` for a state-space model, and its roots are the eigenvalues of `A` (Day 9). The polynomial class stores coefficients in descending order (C4).

Binary arithmetic operators are declared as free (friend) functions for symmetric treatment of operands; `operator[]`, `operator()` and assignment must be members. Equality on floating-point coefficients is not overloaded; `isApprox` is provided instead.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/core/Polynomial.hpp`, `src/core/Polynomial.cpp` | Create (Listing 8-1); `roots()` is implemented on Day 12 |
| `tests/core/test_Polynomial.cpp` | Create |

**Listing 8-1 — `Polynomial`**

```cpp
// include/csl/core/Polynomial.hpp
#pragma once
#include <initializer_list>
#include <iosfwd>
#include <vector>
#include "csl/core/Types.hpp"
namespace csl {
class Polynomial {
public:
    Polynomial();                                        // zero polynomial
    Polynomial(std::initializer_list<Real> coeffs);      // descending powers
    explicit Polynomial(const Vector& coeffs);
    static Polynomial fromRoots(const std::vector<Complex>& roots, Real leading = 1.0);

    Index         degree()  const { return c_.size() - 1; }
    const Vector& coeffs()  const { return c_; }
    Real          operator[](Index i) const { return c_(i); }   // i = 0: leading coefficient
    Real          leading() const { return c_(0); }
    Real    operator()(Real x)    const;                 // Horner evaluation
    Complex operator()(Complex x) const;
    Polynomial derivative() const;
    Polynomial monic() const;
    Matrix     evaluate(const Matrix& A) const;          // p(A), Horner with matrices (Day 38)
    std::vector<Complex> roots() const;                  // Day 12
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
    void trim();                                         // remove leading near-zero coefficients
};
Polynomial pow(const Polynomial& p, Index k);
}

// src/core/Polynomial.cpp (principal functions)
Polynomial operator*(const Polynomial& a, const Polynomial& b) {      // coefficient convolution
    Vector c = Vector::Zero(a.degree() + b.degree() + 1);
    for (Index i = 0; i <= a.degree(); ++i)
        for (Index j = 0; j <= b.degree(); ++j) c(i + j) += a[i] * b[j];
    return Polynomial(c);
}
Polynomial operator+(const Polynomial& a, const Polynomial& b) {      // align by lowest power
    const Index n = std::max(a.degree(), b.degree()) + 1;
    Vector c = Vector::Zero(n);
    c.tail(a.degree() + 1) += a.coeffs();
    c.tail(b.degree() + 1) += b.coeffs();
    return Polynomial(c);
}
Real Polynomial::operator()(Real x) const {
    Real acc = 0.0;
    for (Index i = 0; i < c_.size(); ++i) acc = acc * x + c_(i);
    return acc;
}
Polynomial Polynomial::fromRoots(const std::vector<Complex>& roots, Real leading) {
    ComplexVector c(1); c(0) = leading;
    for (const Complex& r : roots) {                                   // multiply by (x - r)
        ComplexVector next = ComplexVector::Zero(c.size() + 1);
        next.head(c.size()) += c;
        next.tail(c.size()) -= r * c;
        c = next;
    }
    if (c.imag().cwiseAbs().maxCoeff() > 1e-8 * (1.0 + c.cwiseAbs().maxCoeff()))
        throw InvalidArgument("Polynomial::fromRoots: roots are not closed under conjugation");
    return Polynomial(Vector(c.real()));
}
```

`trim()` removes leading coefficients with magnitude below `1e-14` so that `degree()` is exact; the zero polynomial retains one coefficient.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V11.1 | `Polynomial{1,1} * Polynomial{1,2}` | isApprox `{1,3,2}` |
| V11.2 | `Polynomial{1,3,2}(−1.0)` | `0` |
| V11.3 | `Polynomial{1,3,2}.derivative()` | `{2,3}` |
| V11.4 | `fromRoots({−1, −2})`; `fromRoots({−1+2j, −1−2j})` | `{1,3,2}`; `{1,2,5}` |
| V11.5 | `Polynomial{0,0,1,3}.degree()` | `1` |

**Exit criteria.**
- [ ] V11.1–V11.5 as tests; the member/free-function rule for operators can be stated
- [ ] Commit

**References.** [R1, §14.1–14.3, §14.8]; [R2, ch. 21]; [R9, §3.1]; [R8, §9.1–9.2].

---

### Day 12 — `TransferFunction` and Companion-Matrix Roots

**Objectives.** C++: composition, reuse of `Eigen::EigenSolver` for a new problem, `std::ostringstream`, `std::sort` with a comparator, constructor normalisation. Control: poles and zeros, properness and relative degree, dc gain, zero-pole-gain form, pole–zero cancellation.

**Background.** The roots of a monic polynomial `sⁿ + a₁sⁿ⁻¹ + … + aₙ` are the eigenvalues of its *companion matrix*

```
C = [[−a₁, −a₂, …, −aₙ], [1, 0, …, 0], [0, 1, …, 0], …]                             (8-2)
```

For `H(s) = N(s)/D(s)`: poles are the roots of `D`, zeros the roots of `N`; `H` is *proper* if `deg N ≤ deg D` and *strictly proper* if `deg N < deg D`; the relative degree `deg D − deg N` is the number of net integrations. `H(0) = N(0)/D(0)` equals the dc gain of Eq. (7-6). The zero-pole-gain form `H(s) = k ∏(s − zᵢ)/∏(s − pᵢ)` is constructed with `Polynomial::fromRoots`. A common root of `N` and `D` cancels; `minreal` removes pairs within a tolerance.

**Implementation.**

| File | Action |
|---|---|
| `src/core/Polynomial.cpp` | Implement `roots()` (Listing 8-2) |
| `include/csl/systems/TransferFunction.hpp`, `src/systems/TransferFunction.cpp` | Create (Listing 8-3); derives from `LTISystem` on Day 13 |
| `examples/plants.hpp` | Add `thermalTF()`, `massSpringDamperTF()` |
| `tests/systems/test_TransferFunction.cpp` | Create |

**Listing 8-2 — `Polynomial::roots`**

```cpp
std::vector<Complex> Polynomial::roots() const {
    const Index n = degree();
    if (n < 1) return {};
    Matrix C = Matrix::Zero(n, n);                                    // Eq. (8-2)
    for (Index j = 0; j < n; ++j) C(0, j) = -c_(j + 1) / c_(0);
    for (Index i = 1; i < n; ++i) C(i, i - 1) = 1.0;
    Eigen::EigenSolver<Matrix> es(C, false);
    const ComplexVector ev = es.eigenvalues();
    std::vector<Complex> r(ev.data(), ev.data() + ev.size());
    std::sort(r.begin(), r.end(), [](const Complex& a, const Complex& b) {
        return a.real() != b.real() ? a.real() < b.real() : a.imag() < b.imag(); });
    return r;
}
```

**Listing 8-3 — `TransferFunction` interface**

```cpp
#pragma once
#include <initializer_list>
#include <iosfwd>
#include <vector>
#include "csl/core/Polynomial.hpp"
namespace csl::systems {
class TransferFunction {
public:
    TransferFunction(Polynomial num, Polynomial den, Real Ts = 0.0);                       // normalises den to monic
    TransferFunction(std::initializer_list<Real> num, std::initializer_list<Real> den, Real Ts = 0.0);
    static TransferFunction fromZPK(const std::vector<Complex>& zeros, const std::vector<Complex>& poles, Real gain, Real Ts = 0.0);
    static TransferFunction pade(Real T, int order);                                        // Day 20

    const Polynomial& num() const { return num_; }
    const Polynomial& den() const { return den_; }
    Real  Ts()    const { return Ts_; }
    Index order() const { return den_.degree(); }
    bool  isProper()         const { return num_.degree() <= den_.degree(); }
    bool  isStrictlyProper() const { return num_.degree() <  den_.degree(); }
    Index relativeDegree()   const { return den_.degree() - num_.degree(); }
    std::vector<Complex> poles() const { return den_.roots(); }
    std::vector<Complex> zeros() const { return num_.roots(); }
    Complex evaluateScalar(Complex s) const { return num_(s) / den_(s); }
    Real    dcGain() const;                                                                // H(0) or H(1); throws if infinite
    TransferFunction minreal(Real tol = 1e-8) const;
    friend std::ostream& operator<<(std::ostream& os, const TransferFunction& H);           // three-line layout
private:
    Polynomial num_, den_;
    Real Ts_;
};
}
```

The constructor throws `InvalidArgument` for a zero denominator or `T_s < 0`, then divides both polynomials by `den.leading()`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V12.1 | `TransferFunction{{1},{1,3,2}}.poles()` | `{−2, −1}` |
| V12.2 | `plants::thermalTF()` (`{3}/{2,1}`): stored form; pole; `dcGain()` | `{1.5}/{1,0.5}`; `−0.5`; `3` |
| V12.3 | `fromZPK({}, {−1,−2}, 1)` | isApprox `{{1},{1,3,2}}` |
| V12.4 | `TransferFunction{{1,1},{1,3,2}}.minreal()` | order 1, pole `−2` |
| V12.5 | `TransferFunction{{1},{0}}` | throws `InvalidArgument` |

**Exit criteria.**
- [ ] V12.1–V12.5 as tests; `operator<<` prints numerator over denominator
- [ ] Commit

**References.** [R1, §8.3, §10.3.1]; [R9, §3.1–3.2]; [R8, §9.2–9.4].

---

### Day 13 — Inheritance and the `LTISystem` Base Class

**Objectives.** C++: public inheritance, virtual and pure virtual functions, abstract classes, `override`, virtual destructors, polymorphic references, slicing, non-virtual functions implemented through virtuals. Control: `H(s) = C(sI − A)⁻¹B + D`; transmission zeros of a state-space model; the `damp` summary.

**Background.** Laplace transformation of Eq. (7-4) gives `sX = AX + BU`, hence

```
H(s) = C (sI − A)⁻¹ B + D                                                           (8-3)
```

which unifies the two representations through the single virtual function `evaluate` (C6). Transmission zeros of a SISO state-space model are the finite generalised eigenvalues of the pair `(M, N)`, `M = [[A, B], [C, D]]`, `N = diag(I, 0)`; eigenvalues with `β ≈ 0` are zeros at infinity and are discarded. For each pole, `ω_n = |p|`, `ζ = −Re p/|p|`, `τ = −1/Re p`.

A pure virtual function (`= 0`) makes the class abstract; derived classes mark overrides with `override`; a class with virtual functions has a virtual destructor; polymorphic objects are passed by reference or pointer, never by value.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/systems/LTISystem.hpp`, `src/systems/LTISystem.cpp` | Create: Listing 4-2 plus `struct PoleInfo { Complex pole; Real wn, zeta, tau; }` and `std::vector<PoleInfo> damp(const LTISystem&)`; implement `isStable()` here and remove it from `StateSpace` |
| `StateSpace.hpp/.cpp` | Derive from `LTISystem`; add `evaluate` and `zeros` (Listing 8-4); mark overrides |
| `TransferFunction.hpp/.cpp` | Derive from `LTISystem`; `evaluate` returns a `1×1 ComplexMatrix`; `dcGain` returns a `1×1 Matrix` |
| `tests/systems/test_LTISystem.cpp` | Create |

**Listing 8-4 — `StateSpace::evaluate` and `StateSpace::zeros`**

```cpp
ComplexMatrix StateSpace::evaluate(Complex s) const {                       // Eq. (8-3)
    const Index n = order();
    const ComplexMatrix M = s * ComplexMatrix::Identity(n, n) - A_.cast<Complex>();
    const ComplexMatrix X = M.partialPivLu().solve(B_.cast<Complex>());
    return C_.cast<Complex>() * X + D_.cast<Complex>();
}
std::vector<Complex> StateSpace::zeros() const {
    if (!isSISO()) throw NotSupported("StateSpace::zeros: SISO only");
    const Index n = order();
    Matrix M(n + 1, n + 1), N = Matrix::Zero(n + 1, n + 1);
    M << A_, B_, C_, D_;
    N.topLeftCorner(n, n).setIdentity();
    Eigen::GeneralizedEigenSolver<Matrix> ges(M, N);
    std::vector<Complex> z;
    for (Index i = 0; i < n + 1; ++i) {
        const Real beta = ges.betas()(i);
        if (std::abs(beta) > 1e-10 * (1.0 + std::abs(ges.alphas()(i)))) z.push_back(ges.alphas()(i) / beta);
    }
    std::sort(z.begin(), z.end(), /* comparator of Listing 8-2 */);
    return z;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V13.1 | E2: `ss.evaluate(s)(0,0)` versus `tf.evaluate(s)(0,0)` at `s = 1 + 2j` and `s = 0.3j` | equal to `1e-9` |
| V13.2 | `zeros()` of the controllable canonical form of `(s+3)/(s²+3s+2)` (entered by hand) | `{−3}` |
| V13.3 | `damp(plants::massSpringDamper())` | `ω_n = 2`, `ζ = 0.125` for both poles |
| V13.4 | A function `report(const LTISystem&)` printing order, stability and `|H(j1)|` called with both representations of E2 | identical output |
| V13.5 | `LTISystem x;` | does not compile (abstract class) |

**Exit criteria.**
- [ ] V13.1–V13.5; every override is marked `override`
- [ ] Commit

**References.** [R1, §15.1–15.5]; [R2, ch. 24–25]; [R8, §9.2]; [R9, §7.3].

---

### Day 14 — Realizations: `tf2ss`, `ss2tf` and Similarity Transforms

**Objectives.** C++: Eigen block operations (`topRightCorner`, `block`, `setIdentity`, `trace`), free conversion functions, numerical identities as internal assertions. Control: realizations and their non-uniqueness, the controllable canonical form, the Faddeev–LeVerrier recursion, minimal realizations.

**Background.** A *realization* of `H(s)` is any `(A, B, C, D)` satisfying Eq. (8-3). For any invertible `T`, `(TAT⁻¹, TB, CT⁻¹, D)` is also a realization (a *similarity transform*); poles, zeros and `H(s)` are invariant. For `H(s) = (b₀sⁿ + … + bₙ)/(sⁿ + a₁sⁿ⁻¹ + … + aₙ)` the *controllable canonical form* is

```
A = [[0, 1, 0, …, 0], [0, 0, 1, …, 0], …, [−aₙ, −aₙ₋₁, …, −a₁]],   B = [0, …, 0, 1]ᵀ
C = [bₙ − b₀aₙ, …, b₁ − b₀a₁],   D = b₀                                             (8-4)
```

The *Faddeev–LeVerrier* recursion computes `det(sI − A)` and `adj(sI − A)` jointly:

```
M₁ = I,   a₁ = −tr(A M₁);   M_k = A M_{k−1} + a_{k−1} I,   a_k = −tr(A M_k)/k   (k = 2…n)
det(sI − A) = sⁿ + a₁sⁿ⁻¹ + … + aₙ,   adj(sI − A) = Σ_{k=1}^{n} s^{n−k} M_k            (8-5)
```

so the numerator coefficient of `s^{n−k}` in `H(s) = [C adj(sI−A) B + D det(sI−A)]/det(sI−A)` is `C M_k B + D a_k`. By the Cayley–Hamilton theorem `M_{n+1} = 0`, which the implementation asserts. A realization is *minimal* when its order equals that of `minreal(H)`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/systems/Conversions.hpp`, `src/systems/Conversions.cpp` | Create (Listing 8-5) |
| `tests/systems/test_Conversions.cpp` | Create |

**Listing 8-5 — Conversions**

```cpp
// include/csl/systems/Conversions.hpp  (Day 14 part)
#pragma once
#include "csl/core/Polynomial.hpp"
#include "csl/systems/StateSpace.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::systems {
StateSpace       tf2ss(const TransferFunction& H);                            // Eq. (8-4)
TransferFunction ss2tf(const StateSpace& sys);                                // Eq. (8-5), SISO
Polynomial       charPoly(const Matrix& A);                                   // det(sI - A)
StateSpace       similarityTransform(const StateSpace& sys, const Matrix& T); // x_new = T x
StateSpace       toStateSpace(const LTISystem& sys);                          // identity for StateSpace; tf2ss otherwise
}

// src/systems/Conversions.cpp
StateSpace tf2ss(const TransferFunction& H) {
    if (!H.isProper()) throw InvalidArgument("tf2ss: transfer function must be proper");
    const Index n = H.order();
    const Vector a = H.den().coeffs();                       // monic: a(0) = 1
    Vector b = Vector::Zero(n + 1);
    const Index dn = H.num().degree();
    for (Index i = 0; i <= dn; ++i) b(n - dn + i) = H.num()[i];
    const Real d = b(0);
    Matrix A = Matrix::Zero(n, n);
    if (n > 1) A.topRightCorner(n - 1, n - 1).setIdentity();
    for (Index j = 0; j < n; ++j) A(n - 1, j) = -a(n - j);
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
    Matrix M = Matrix::Identity(n, n);                        // M_1
    for (Index k = 1; k <= n; ++k) {
        a(k) = -(A * M).trace() / static_cast<Real>(k);
        b(k) = (sys.C() * M * sys.B())(0, 0) + d * a(k);
        M = A * M + a(k) * Matrix::Identity(n, n);            // M_{k+1}
    }
    assert(M.norm() < 1e-8 * (1.0 + A.norm()));               // Cayley–Hamilton
    return TransferFunction(Polynomial(b), Polynomial(a), sys.Ts());
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V14.1 | `ss2tf(plants::massSpringDamper())`; `ss2tf(plants::dcMotor())` | `{1}/{1,0.5,4}`; `{2}/{1,12,20.02}` |
| V14.2 | `ss2tf(tf2ss(H))` for `H = {1,3}/{1,3,2}` and `H = {2,1,3}/{1,3,2}` | isApprox `H` |
| V14.3 | `tf2ss` of E2's transfer function | matrices equal to `plants::massSpringDamper()` |
| V14.4 | `similarityTransform(sys, [[2,0],[1,1]])` | same poles and `evaluate(1+j)` as `sys` |
| V14.5 | `tf2ss({1,3}/{1,3,2}).zeros()` | `{−3}` |

**Exit criteria.**
- [ ] V14.1–V14.5; the non-uniqueness of realizations can be explained
- [ ] Commit

**References.** [R4, "Block operations"]; [R9, §7.3–7.4]; [R12, §4.4]; [R8, §9.2].

---

### Day 15 — The Plotter and Time Responses

**Objectives.** C++: class design with method chaining, `std::ostringstream`, SVG generation, coordinate mapping and tick computation, default arguments, layering (`Figure` is domain-independent; `Plots` is control-specific). Control: step, impulse and arbitrary-input responses; zero-order hold between samples; automatic horizon selection.

**Background.** The *step response* is the output for `u = 1`, `t ≥ 0`, from rest; it settles to the dc gain. The *impulse response* of a strictly proper system equals the zero-input response from `x(0) = B`; a direct-feedthrough term `D δ(t)` is not representable in samples and is omitted. `lsim` applies a sampled input held constant between samples (zero-order hold) and integrates with RK4 within each interval; for discrete systems the exact recursion `x[k+1] = A x[k] + B u[k]` is used. The default horizon is `t_f = 8/min|Re pᵢ|` over stable poles with `Δt = t_f/2000`, limited to at least 20 samples per fastest mode; for systems without stable poles `t_f = 10`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/plot/Figure.hpp`, `src/plot/Figure.cpp` | Create (Listing 8-6) |
| `include/csl/plot/Plots.hpp`, `src/plot/Plots.cpp` | Create: `Figure plotOutputs(const SimResult&, title)`, `Figure plotStates(const SimResult&, title)` |
| `include/csl/simulation/TimeResponse.hpp`, `src/simulation/TimeResponse.cpp` | Create (Listing 8-7) |
| `examples/ex01_first_order_sim.cpp` | Add `plotOutputs(step(E1)).save("step_thermal.html")` |
| `tests/plot/test_Figure.cpp`, `tests/simulation/test_TimeResponse.cpp` | Create |

**Listing 8-6 — `Figure` interface**

```cpp
#pragma once
#include <string>
#include <vector>
#include "csl/core/Types.hpp"
namespace csl::plot {
class Figure {
public:
    explicit Figure(std::string title = "");
    Figure& line   (const Vector& x, const Vector& y, const std::string& label = "");
    Figure& scatter(const Vector& x, const Vector& y, const std::string& label = "", const std::string& marker = "o");  // "o" | "x"
    Figure& hline(Real y, const std::string& label = "");
    Figure& vline(Real x, const std::string& label = "");
    Figure& xlabel(const std::string& s) { xlabel_ = s; return *this; }
    Figure& ylabel(const std::string& s) { ylabel_ = s; return *this; }
    Figure& logX(bool on = true) { logX_ = on; return *this; }
    Figure& logY(bool on = true) { logY_ = on; return *this; }
    Figure& xlim(Real lo, Real hi);
    Figure& ylim(Real lo, Real hi);
    Figure& size(int w, int h) { width_ = w; height_ = h; return *this; }
    std::string svg() const;                                           // <svg> element
    void save(const std::string& htmlPath) const;                      // standalone HTML page
    static void savePage(const std::string& htmlPath, const std::vector<Figure>& figures);   // stacked figures
private:
    struct Series { Vector x, y; std::string label; bool markers; std::string marker; };
    struct Guide  { bool horizontal; Real value; std::string label; };
    std::string title_, xlabel_, ylabel_;
    std::vector<Series> series_;
    std::vector<Guide>  guides_;
    bool logX_ = false, logY_ = false, hasXlim_ = false, hasYlim_ = false;
    Real xlo_ = 0, xhi_ = 1, ylo_ = 0, yhi_ = 1;
    int  width_ = 820, height_ = 420;
    void dataRange(Real& xlo, Real& xhi, Real& ylo, Real& yhi) const;
    static std::vector<Real> ticks(Real lo, Real hi, bool log);        // 1/2/5 x 10^k steps; decades when log
};
}
```

`svg()` maps data to pixels with margins (left 70, right 20, top 40, bottom 50), draws grid lines and tick labels, one `<polyline>` per line series (skipping non-finite points and non-positive values on logarithmic axes), `<circle>` or cross `<path>` markers for scatter series, guide lines, axes, title, axis labels and a legend. Colours cycle through a fixed six-entry palette. `save()` wraps the element in a minimal HTML document.

**Listing 8-7 — Time responses**

```cpp
#pragma once
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::simulation {
SimResult step   (const systems::LTISystem& sys, Real tf = -1, Real dt = -1, Index inputIndex = 0);   // tf, dt <= 0: automatic
SimResult impulse(const systems::LTISystem& sys, Real tf = -1, Real dt = -1, Index inputIndex = 0);
SimResult lsim   (const systems::LTISystem& sys, const Matrix& u, const Vector& t, const Vector& x0 = Vector());   // u: N x m, ZOH
void autoHorizon(const systems::LTISystem& sys, Real& tf, Real& dt);
}
```

`lsim` converts with `toStateSpace`, then for continuous systems applies `rk4Step` with `u` held over each interval and records `x`, `y = Cx + Du`, `u`; for discrete systems it applies the recursion. `step` calls `lsim` with a constant input; `impulse` calls `lsim` with zero input and `x0 = B.col(inputIndex)`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V15.1 | `step(plants::thermalTF())` at `t = 2` | `1.8964 ± 1e-6` |
| V15.2 | `step(E2)`: final value; peak; peak time | `0.25`; `0.418 ± 0.002`; `≈ 1.58 s` |
| V15.3 | `impulse(E1).y(0)` | `K/τ = 1.5` |
| V15.4 | `impulse(E2).y` versus the finite-difference derivative of `step(E2).y` | agree to `1e-3` |
| V15.5 | `Figure::svg()` for two series | contains two `<polyline` elements and the title |
| V15.6 | `step_thermal.html` renders in a browser with axes, grid, legend | example only |

**Exit criteria.**
- [ ] V15.1–V15.6; `Figure` supports logarithmic axes (exercised on Day 17)
- [ ] Commit

**References.** [R1, §8.1–8.3, §17.5]; [R47]; [R9, §3.3]; [R8, §6.3].

---

### Day 16 — STL Algorithms, Specifications and Test Signals

**Objectives.** C++: iterators and `[first, last)` ranges, `std::max_element`, `std::find_if`, reverse iterators, `std::distance`, `std::accumulate`, algorithms over Eigen vectors. Control: time-domain specifications and the second-order approximations relating them to pole locations; standard test signals.

**Background.** For a step response with steady-state value `y_ss`:

| Specification | Definition | Second-order approximation (`ζ < 1`) |
|---|---|---|
| Rise time `t_r` | 10 % → 90 % of `y_ss` | `≈ 1.8/ω_n` (near `ζ = 0.5`) |
| Peak time `t_p` | time of maximum | `π/ω_d` |
| Overshoot `M_p` | `(y_max − y_ss)/y_ss` | `exp(−πζ/√(1−ζ²))` |
| Settling time `t_s` | last time `|y − y_ss| > 2 % y_ss` | `≈ 4/(ζω_n)` |

Specifications define a region of the s-plane for the dominant poles; pole placement (Day 38) and root locus (Day 21) target that region. A ramp input tests type-1 tracking (zero steady-state error to a step, constant error to a ramp); sinusoids probe the frequency response (Day 17); a chirp sweeps frequency for identification (Day 58).

**Implementation.**

| File | Action |
|---|---|
| `TimeResponse.hpp/.cpp` | Add Listing 8-8 |
| `include/csl/signals/Signals.hpp`, `src/signals/Signals.cpp` | Create: `timeGrid(t0, tf, dt)`, `step(t, amplitude, tStart)`, `ramp(t, slope, tStart)`, `sine(t, amplitude, w, phase)`, `square(t, amplitude, period)`, `chirp(t, f0_hz, f1_hz, amplitude)` (linear sweep); all return `Vector` |
| tests | Extend `test_TimeResponse.cpp`; create `tests/signals/test_Signals.cpp` |

**Listing 8-8 — Step specifications**

```cpp
struct StepInfo { Real steadyState, riseTime, peakTime, peak, overshootPercent, settlingTime, settlingThreshold; };
StepInfo stepInfo(const SimResult& r, Index outputIndex = 0, Real settlingThreshold = 0.02);
StepInfo stepInfo(const systems::LTISystem& sys, Index outputIndex = 0, Real settlingThreshold = 0.02);

StepInfo stepInfo(const SimResult& r, Index j, Real thr) {
    const Vector y = r.y.col(j);
    const Vector& t = r.t;
    StepInfo s{};
    s.settlingThreshold = thr;
    s.steadyState = y.tail(std::max<Index>(1, y.size() / 50)).mean();            // mean of the last 2 % of samples
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

**Verification.**

| ID | Check | Expected (`Δt = 0.001`) |
|---|---|---|
| V16.1 | `stepInfo(E2)`: steady state; overshoot; peak time; rise time; settling time | `0.25`; `67.3 ± 0.3 %`; `1.583 ± 0.002 s`; `0.56 ± 0.01 s`; `14.5 ± 0.2 s` |
| V16.2 | `stepInfo(E1)`: rise time; settling time; overshoot | `4.394 ± 0.01`; `7.82 ± 0.02`; `0` |
| V16.3 | `lsim(E1, ramp)`: steady-state lag | `y(t) ≈ K(t − τ)` |
| V16.4 | `sine`, `square`, `chirp` at sample points | closed-form values |

**Exit criteria.**
- [ ] V16.1–V16.4; the four second-order formulas can be used to map specifications to a pole region
- [ ] Commit `Day 16: Phase 2 complete`; tag `phase-2`

**References.** [R1, §9.2, §10.1–10.3]; [R2, ch. 18]; [R9, §3.4]; [R8, §6.3].

---
## 9. Phase 3 — Frequency-Domain Analysis (Days 17–22)

**Phase objective.** Implement frequency response, Bode and Nyquist analysis, stability margins, interconnections and sensitivity functions, minimum-phase and delay handling, root locus and Routh–Hurwitz; introduce the umbrella header and the second example.

### Day 17 — Frequency Response and the Bode Plot

**Objectives.** C++: inline header utilities, `std::arg`, `std::exp` of complex arguments, logarithmic axes in `Figure`, stacked figure pages. Control: sinusoidal steady state, Bode magnitude and phase, asymptotic rules, phase unwrapping, the discrete-time frequency response.

**Background.** For a stable LTI system driven by `u = sin ωt`, the steady-state output is `|H(jω)| sin(ωt + ∠H(jω))`. The *frequency response* is `H(s)` on the imaginary axis. The Bode plot shows `20 log₁₀|H(jω)|` (dB) and `∠H(jω)` (degrees) against `log ω`; products of factors become sums of plots. Asymptotic contributions:

| Factor | Magnitude slope beyond the corner | Phase |
|---|---|---|
| gain `K` | 0 dB/decade at `20 log K` | 0° |
| `1/s` | −20 dB/decade | −90° |
| real pole `1/(s/a + 1)` | 0 → −20 dB/decade at `ω = a` | 0° → −90° (−45° at `a`) |
| real zero `(s/a + 1)` | 0 → +20 dB/decade | 0° → +90° |
| complex pair | 0 → −40 dB/decade at `ω_n`; peak `1/(2ζ√(1−ζ²))` for `ζ < 0.707` | 0° → −180° |

`std::arg` returns values in `(−π, π]`; the phase is *unwrapped* by adding `±2π` where consecutive samples jump by more than `π`. For discrete systems the response is evaluated at `z = e^{jωT_s}` (C6) and is meaningful up to the Nyquist frequency `π/T_s`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/core/MathUtils.hpp`, `src/core/MathUtils.cpp` | Create: `dB`, `fromDB`, `rad2deg`, `deg2rad`, `wrapToPi` (inline); `linspace`, `logspace(exp10_a, exp10_b, n)`, `unwrapPhase(Vector&)` |
| `include/csl/analysis/FrequencyResponse.hpp`, `src/analysis/FrequencyResponse.cpp` | Create (Listing 9-1) |
| `plot/Plots.hpp/.cpp` | Add `plotBode(const std::vector<std::pair<FrequencyResponse, std::string>>&, path)`: two stacked figures with logarithmic frequency axes and guide lines at 0 dB and −180° |
| tests | `tests/core/test_MathUtils.cpp`, `tests/analysis/test_FrequencyResponse.cpp` |

**Listing 9-1 — Frequency response**

```cpp
#pragma once
#include "csl/core/Types.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::analysis {
struct FrequencyResponse { Vector w; ComplexVector H; Real Ts = 0.0; };    // Listing 4-3
struct BodeData { Vector w, magnitude_dB, phase_deg; };                     // phase unwrapped

Vector            defaultFrequencies(const systems::LTISystem& sys, Index n = 400);   // two decades around the pole/zero range; capped at 0.999 pi/Ts
FrequencyResponse frequencyResponse(const systems::LTISystem& sys, const Vector& w);
FrequencyResponse frequencyResponse(const systems::LTISystem& sys);
BodeData          bode(const FrequencyResponse& fr);
}

FrequencyResponse frequencyResponse(const systems::LTISystem& sys, const Vector& w) {
    if (!sys.isSISO()) throw NotSupported("frequencyResponse: SISO only");
    FrequencyResponse fr;
    fr.w = w; fr.Ts = sys.Ts(); fr.H.resize(w.size());
    for (Index k = 0; k < w.size(); ++k) {
        const Complex s = sys.isContinuous() ? Complex(0.0, w(k)) : std::exp(Complex(0.0, w(k) * sys.Ts()));
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

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V17.1 | E1 at `ω = 0.5` | `6.532 dB`, `−45.00°` |
| V17.2 | E2 at `ω = 2`; at `ω = 0.01`; peak | `0 dB`, `−90°`; `−12.04 dB`; `0.07 dB` near `1.968 rad/s` |
| V17.3 | `1/s` at `ω = 1` | `0 dB`, `−90°` |
| V17.4 | Discrete `TransferFunction{{1},{1,−0.5}, 0.1}` at `ω = 0` | `H = 2` |
| V17.5 | `unwrapPhase` of `[3.0, −3.0, 3.0]` | `[3.0, 3.283, 3.0]` |

**Exit criteria.**
- [ ] V17.1–V17.5; E2's Bode plot sketched from the asymptote table agrees with the computed plot
- [ ] Commit

**References.** [R9, §6.1]; [R8, §9.4–9.5, §10.1].

---

### Day 18 — Nyquist Plot and Stability Margins

**Objectives.** C++: `std::optional`, interpolation on sampled data, result structs mixing optionals and flags. Control: the Nyquist criterion, gain, phase and delay margins, their limitations.

**Background.** The Nyquist plot is the curve `L(jω)`, `ω ∈ [0, ∞)`, of the *loop transfer function* `L = CP`. For an open-loop stable `L`, the closed loop is stable if and only if the plot does not encircle `−1`; in general `Z = N + P` (closed-loop RHP poles = clockwise encirclements + open-loop RHP poles). At the *phase crossover* `ω_pc` (`∠L = −180°`) the gain margin is `GM = 1/|L(jω_pc)|`; at the *gain crossover* `ω_gc` (`|L| = 1`) the phase margin is `PM = 180° + ∠L(jω_gc)`; the delay margin is `PM [rad]/ω_gc`. Usual targets are `GM ≥ 6 dB`, `PM ∈ [30°, 60°]`; `PM ≈ 100ζ` for a second-order loop. A margin is absent when its crossover does not exist (`std::optional`, C13); with multiple crossovers the smallest margin is reported.

**Implementation.**

| File | Action |
|---|---|
| `FrequencyResponse.hpp/.cpp` | Add Listing 9-2 |
| `plot/Plots.hpp/.cpp` | Add `plotNyquist(const FrequencyResponse&, path)` with the mirror image and the point `−1` |
| tests | Extend `test_FrequencyResponse.cpp` |

**Listing 9-2 — Margins and Nyquist data**

```cpp
struct Margins {
    std::optional<Real> gainMargin_dB, phaseCrossover_w;
    std::optional<Real> phaseMargin_deg, gainCrossover_w;
    std::optional<Real> delayMargin_s;
    bool closedLoopStable = false;                   // set on Day 19 via feedback(L).isStable()
};
Margins margins(const systems::LTISystem& L);
Margins margins(const FrequencyResponse& fr);        // from sampled data (used by Day 58)
struct NyquistData { Vector w, re, im; };
NyquistData nyquist(const FrequencyResponse& fr);
```

Algorithm of `margins(fr)`: compute `bode(fr)`; locate sign changes of `magnitude_dB` and interpolate linearly in `log ω` for `ω_gc`; interpolate the phase there; `PM = 180 + phase`. Locate sign changes of the phase relative to the nearest odd multiple of `180°` (compare `wrapToPi(deg2rad(phase)) + π`) for `ω_pc`; `GM_dB = −magnitude_dB` there. Report the smallest of each.

**Verification.**

| ID | Check | Expected (`±0.5 %`) |
|---|---|---|
| V18.1 | `L = {1}/{1,3,2,0}` | `GM = 15.56 dB` at `1.414`; `PM = 53.4°` at `0.446`; delay margin `2.09 s` |
| V18.2 | `L = 1/(s+1)` | both margins absent |
| V18.3 | `L = 10/(s(s+1))` | `PM = 18.0°` at `3.08 rad/s` |
| V18.4 | Nyquist of `{1}/{1,3,2,0}` at `ω = √2` | passes through `(−1/6, 0)` |

**Exit criteria.**
- [ ] V18.1–V18.4; the Nyquist interpretation of each margin can be stated
- [ ] Commit

**References.** [R3, `std::optional`]; [R9, §6.3–6.4]; [R8, §10.1–10.3].

---

### Day 19 — Interconnections and Sensitivity Functions

**Objectives.** C++: overload sets across types, free operator functions for transfer functions, Eigen comma-initialised block matrices, consistency checks on sample times. Control: series, parallel and feedback connections; closed-loop poles; the gang of four.

**Background.** For series connection (`H₂` after `H₁`), parallel connection, and negative feedback of `G` with `H` in the return path:

```
series:    A = [[A₁, 0], [B₂C₁, A₂]],  B = [B₁; B₂D₁],  C = [D₂C₁, C₂],  D = D₂D₁
parallel:  A = diag(A₁, A₂),  B = [B₁; B₂],  C = [C₁, C₂],  D = D₁ + D₂                    (9-1)
feedback:  G/(1 + GH)   (transfer functions: num = N_G D_H,  den = D_G D_H + N_G N_H)
```

The state-space feedback connection is derived from `u = r − y_h`, `y = Cx + Du`, `y_h = C_h x_h + D_h y`, with `E = I + D D_h` required invertible (otherwise an algebraic loop exists). With unity feedback and loop `L = CP`:

```
S = 1/(1 + L),   T = L/(1 + L),   CS = C/(1 + L),   PS = P/(1 + L),   S + T = 1          (9-2)
```

`S` maps output disturbances and reference-to-error; `T` maps reference and measurement noise to output; `CS` is the control sensitivity; `PS` the load sensitivity. Closed-loop poles are the roots of `1 + L(s) = 0`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/systems/Interconnect.hpp`, `src/systems/Interconnect.cpp` | Create (Listing 9-3) |
| `FrequencyResponse.cpp` | Set `Margins::closedLoopStable = feedback(L).isStable()` |
| `tests/systems/test_Interconnect.cpp` | Create |

**Listing 9-3 — Interconnections**

```cpp
#pragma once
#include "csl/systems/StateSpace.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::systems {
TransferFunction series  (const TransferFunction& first, const TransferFunction& second);   // second * first
TransferFunction parallel(const TransferFunction& a, const TransferFunction& b);
TransferFunction feedback(const TransferFunction& G, const TransferFunction& H, int sign = -1);
TransferFunction feedback(const TransferFunction& G, int sign = -1);                         // unity feedback
StateSpace series  (const StateSpace& first, const StateSpace& second);
StateSpace parallel(const StateSpace& a, const StateSpace& b);
StateSpace feedback(const StateSpace& G, const StateSpace& H, int sign = -1);
StateSpace feedback(const StateSpace& G, int sign = -1);
inline TransferFunction operator*(const TransferFunction& a, const TransferFunction& b) { return series(b, a); }   // a after b
inline TransferFunction operator+(const TransferFunction& a, const TransferFunction& b) { return parallel(a, b); }
struct GangOfFour { TransferFunction S, T, CS, PS; };
GangOfFour gangOfFour(const TransferFunction& C, const TransferFunction& P);
}

TransferFunction feedback(const TransferFunction& G, const TransferFunction& H, int sign) {
    checkSameTs(G, H);
    const Polynomial num = G.num() * H.den();
    const Polynomial den = G.den() * H.den() - static_cast<Real>(sign) * (G.num() * H.num());
    return TransferFunction(num, den, G.Ts());
}
```

Mixed or unequal sample times throw `InvalidArgument`. Operators are not defined for `StateSpace` (the meaning of `*` would be ambiguous).

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V19.1 | E2 under proportional gain 10: `feedback(10·P)` | denominator `{1,0.5,14}`, dc gain `10/14` |
| V19.2 | `feedback(K·L)`, `L = {1}/{1,3,2,0}`, `K = 1, 6, 7` | stable; marginal (poles `±j√2`); unstable |
| V19.3 | `series(tf2ss(a), tf2ss(b))` versus `series(a, b)` at `s = 1 + j` | equal to `1e-9` |
| V19.4 | `(S + T).minreal()` | isApprox `1` |
| V19.5 | `feedback(G, H)` with `D_G = 1`, `D_H = −1` | throws (`E = 0`) |

**Exit criteria.**
- [ ] V19.1–V19.5; the state-space feedback formula derived on paper
- [ ] Commit

**References.** [R9, §3.2]; [R8, §9.3, §11.1].

---

### Day 20 — Minimum Phase, Right-Half-Plane Zeros and Time Delay

**Objectives.** C++: static factories, factorials, coefficient reversal, `std::any_of`. Control: minimum-phase and non-minimum-phase systems, all-pass factors, time delay and its Padé approximation.

**Background.** Zeros at `−z` and `+z` produce identical magnitude plots and opposite phase. A system is *minimum phase* when all poles and zeros lie in the open left half-plane; its phase is then determined by its magnitude. A right-half-plane zero at `+z` causes initial undershoot in the step response and limits achievable bandwidth to roughly `z/2`. The all-pass factor `(z − s)/(z + s)` has unit magnitude. A time delay `e^{−sT}` has unit magnitude and phase `−ωT`; its Padé approximation of order `n` is

```
e^{−sT} ≈ P_n(−sT)/P_n(sT),   P_n(x) = Σ_{k=0}^{n} [(2n−k)! n!] / [(2n)! k! (n−k)!] x^k              (9-3)
```

First order: `(1 − sT/2)/(1 + sT/2)`, which has a right-half-plane zero at `2/T`.

**Implementation.**

| File | Action |
|---|---|
| `TransferFunction.hpp/.cpp` | Implement `pade` (Listing 9-4) |
| `include/csl/analysis/Stability.hpp`, `src/analysis/Stability.cpp` | Create with `bool isMinimumPhase(const LTISystem&)` (continuous: no poles or zeros with positive real part; discrete: none outside the unit circle); Day 22 completes the file |
| tests | Extend `test_TransferFunction.cpp`; create `tests/analysis/test_Stability.cpp` |

**Listing 9-4 — Padé approximation**

```cpp
TransferFunction TransferFunction::pade(Real T, int order) {
    if (T <= 0 || order < 1) throw InvalidArgument("pade: T > 0 and order >= 1 required");
    const int n = order;
    auto fact = [](int k) { Real f = 1; for (int i = 2; i <= k; ++i) f *= i; return f; };
    Vector numAsc(n + 1), denAsc(n + 1);                                     // ascending powers
    for (int k = 0; k <= n; ++k) {
        const Real c = fact(2 * n - k) * fact(n) / (fact(2 * n) * fact(k) * fact(n - k));   // Eq. (9-3)
        denAsc(k) = c * std::pow(T, k);
        numAsc(k) = c * std::pow(-T, k);
    }
    return TransferFunction(Polynomial(Vector(numAsc.reverse())), Polynomial(Vector(denAsc.reverse())));
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V20.1 | `pade(1, 1)` | isApprox `{−1, 2}/{1, 2}` |
| V20.2 | Phase of `pade(1, 3)` at `ω = 1`; at `ω = 3`; magnitude at all `ω` | `−57.30° ± 0.01°`; `−171.9° ± 0.5°`; `0 dB` |
| V20.3 | `step((1 − s)/((s+1)(s+2)))` | initial slope `−1`; final value `0.5` |
| V20.4 | `isMinimumPhase` | false for V20.3's system; true for E1–E3 |
| V20.5 | `margins(series(pade(0.5, 2), L))`, `L = {1}/{1,3,2,0}` | `PM ≈ 40.6°` |

**Exit criteria.**
- [ ] V20.1–V20.5
- [ ] Commit

**References.** [R9, §6.1.2, §6.6]; [R8, §9.4, §11.4].

---

### Day 21 — Root Locus

**Objectives.** C++: nested `std::vector`s, greedy nearest-neighbour matching with a `used` mask, `std::sort` with lambda comparators, scatter markers. Control: the root locus, Evans' sketching rules, imaginary-axis crossings.

**Background.** With loop `K L(s)`, `L = N/D`, the closed-loop characteristic equation is

```
D(s) + K N(s) = 0                                                                 (9-4)
```

The root locus is the set of roots of Eq. (9-4) for `K ∈ [0, ∞)`: branches start at the open-loop poles (`K = 0`); `m` end at the zeros and `n − m` tend to infinity along asymptotes at angles `(2k+1)·180°/(n−m)` from the centroid `(Σpᵢ − Σzᵢ)/(n−m)`; real-axis segments to the left of an odd number of real poles and zeros belong to the locus. Imaginary-axis (or unit-circle) crossings give the gain margin. The library computes the locus by repeated root finding and uses the rules for interpretation.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/analysis/RootLocus.hpp`, `src/analysis/RootLocus.cpp` | Create (Listing 9-5) |
| `plot/Plots.hpp/.cpp` | Add `plotRootLocus(const RootLocusData&, path)` (branches as lines, poles as `x`, zeros as `o`) and `plotPoleZero(const LTISystem&, path)` |
| `tests/analysis/test_RootLocus.cpp` | Create |

**Listing 9-5 — Root locus**

```cpp
#pragma once
#include <vector>
#include "csl/systems/TransferFunction.hpp"
namespace csl::analysis {
struct RootLocusData {
    Vector gains;
    std::vector<std::vector<Complex>> branches;      // branches[b][k]
    std::vector<Complex> openLoopPoles, openLoopZeros;
};
RootLocusData rootLocus(const systems::TransferFunction& L, const Vector& gains);
RootLocusData rootLocus(const systems::TransferFunction& L);                 // gains = {0} ∪ logspace(-3, 3, 600)
Vector        asymptoteAngles_deg(const systems::TransferFunction& L);
Real          asymptoteCentroid(const systems::TransferFunction& L);
std::vector<std::pair<Real, Complex>> stabilityBoundaryCrossings(const RootLocusData& rl);   // (gain, pole) at Re = 0 or |z| = 1
}

RootLocusData rootLocus(const systems::TransferFunction& L, const Vector& gains) {
    RootLocusData rl;
    rl.gains = gains; rl.openLoopPoles = L.poles(); rl.openLoopZeros = L.zeros();
    const Index n = L.order();
    rl.branches.assign(n, std::vector<Complex>(gains.size()));
    std::vector<Complex> prev = rl.openLoopPoles;
    for (Index k = 0; k < gains.size(); ++k) {
        std::vector<Complex> cur = (L.den() + gains(k) * L.num()).roots();     // Eq. (9-4)
        if (static_cast<Index>(cur.size()) != n) throw NumericalError("rootLocus: degree dropped");
        std::vector<bool> used(n, false);
        for (Index b = 0; b < n; ++b) {                                        // nearest-neighbour continuation
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

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V21.1 | `L = {1}/{1,3,2,0}`: asymptote angles; centroid | `{60, 180, 300}°`; `−1` |
| V21.2 | `stabilityBoundaryCrossings` for V21.1 | `K = 6.0 ± 0.05` at `±1.414j` |
| V21.3 | `L = 1/(s(s+2))` at `K = 5` | poles `−1 ± 2j` |
| V21.4 | Plots of V21.1 and V21.3 agree with hand sketches | example only |

**Exit criteria.**
- [ ] V21.1–V21.4
- [ ] Commit

**References.** [R9, §5.1–5.4]; [R10, ch. 8].

---

### Day 22 — Routh–Hurwitz, Umbrella Header and Phase 3 Review

**Objectives.** C++: header hygiene (self-sufficient headers, `<iosfwd>`), the umbrella header, `inline` versus out-of-line free functions, tabular algorithms with special cases. Control: the Routh array and its use for parametric stability questions.

**Background.** For `a₀sⁿ + a₁sⁿ⁻¹ + … + aₙ`, the Routh array has first rows `(a₀, a₂, a₄, …)` and `(a₁, a₃, a₅, …)`; each subsequent row is formed from the two above by

```
b_i = (a₁ a_{2i} − a₀ a_{2i+1}) / a₁,   c_i = (b₁ a_{2i+1} − a₁ b_{i+1}) / b₁,  …                  (9-5)
```

The number of sign changes in the first column equals the number of right-half-plane roots. A zero first-column entry in a non-zero row is replaced by `ε → 0⁺`; an all-zero row is replaced by the derivative of the auxiliary polynomial formed from the row above. The third-order condition `a₁a₂ > a₀a₃` applied to `s³ + 3s² + 2s + K` gives `K < 6`, agreeing with V18.1 and V21.2.

**Implementation.**

| File | Action |
|---|---|
| `Stability.hpp/.cpp` | Add `Matrix routhArray(const Polynomial&)`, `int routhRHPRootCount(const Polynomial&)`, `bool routhIsStable(const Polynomial&)` |
| `include/csl/csl.hpp` | Create: includes every public header of Figure 3-1 that exists; extended at the end of each later phase |
| `examples/ex02_representations.cpp` | Create: E2 as `StateSpace` → `ss2tf`; poles, zeros, dc gain, `damp`; `plotStep`, `plotBode`, `plotNyquist` of `10·P`; `plotRootLocus(P)`; Routh-derived stability range of `K`; margins with and without `pade(0.5, 2)` |
| `sandbox/phase2_3_review.md` | Phase review |

Each header must compile alone in an empty translation unit.

**Phase 2–3 review**: (1) the run-time sequence when `report(const LTISystem&)` calls `poles()`; (2) why `TransferFunction` normalises the denominator, and what would fail otherwise; (3) three independent computations in the library that establish `K = 6` as the stability limit of `1/(s(s+1)(s+2))`, with their days; (4) the asymptotic Bode sketch of `10(s+1)/(s(s+10)(s²+0.4s+4))`; (5) the sensitivity peak versus the phase margin as robustness measures.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V22.1 | `routhIsStable({1,2,3,4})`; `routhRHPRootCount({1,2,3,10})` | `true` (`2·3 > 4`); `2` |
| V22.2 | `routhRHPRootCount({1,0,−1})`; `routhRHPRootCount({1,1,2,2,3})` (zero pivot) | `1`; `2` |
| V22.3 | Each header compiles standalone; `ex02` runs and its numbers agree across methods | — |

**Exit criteria.**
- [ ] V22.1–V22.3; review written
- [ ] Commit `Day 22: Phase 3 complete`; tag `phase-3`

**References.** [R9, §3.6]; [R10, §6.2–6.4]; [R1, §2.6.3, §6.5.2].

---
## 10. Phase 4 — Classical Control Design (Days 23–28)

**Phase objective.** Implement the `Controller` base class, a practical PID, the sampled-data closed-loop `Simulator`, tuning rules, lead/lag design and loop-shaping analysis.

### Day 23 — The `Controller` Base Class and PID

**Objectives.** C++: a second abstract base designed from its consumer's needs, `protected`, configuration structs with default member initialisers, stateful objects, `std::clamp`. Control: the PID law, the effects of each term, the implementable form.

**Background.** The textbook law is `u = K_p e + K_i ∫e dt + K_d de/dt`, `e = r − y`. Proportional action leaves a steady-state offset on type-0 plants (E1 with `K_p = 2`: `y_ss = K_pK/(1 + K_pK) = 6/7`); integral action removes it at the cost of phase lag; derivative action adds damping but amplifies noise. The implementable form uses a filtered derivative `K_d s/(T_f s + 1)`, `T_f = K_d/(K_p N)`, `N ∈ [8, 20]`, derivative on the measurement to avoid the derivative kick, backward-Euler discretization at sample time `T_s`, and output limits:

```
e_k = r_k − y_k,   I_k = I_{k−1} + K_i T_s e_k,   D_k = (T_f D_{k−1} − K_d (y_k − y_{k−1})) / (T_f + T_s)
u_k = clamp(K_p (b r_k − y_k) + I_k + D_k, u_min, u_max)                                     (10-1)
```

with setpoint weight `b` (Day 25). The continuous transfer function of the ideal filtered PID, used for frequency-domain analysis, is

```
C(s) = [(K_p T_f + K_d) s² + (K_p + K_i T_f) s + K_i] / [T_f s² + s]                          (10-2)
```

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/Controller.hpp` | Create (Listing 4-4) |
| `include/csl/control/PID.hpp`, `src/control/PID.cpp` | Create (Listing 10-1); the anti-windup branches are completed on Day 25 |
| `tests/control/test_PID.cpp` | Create |

**Listing 10-1 — `PID`**

```cpp
#pragma once
#include "csl/control/Controller.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::control {
class PID final : public Controller {
public:
    struct Config {
        Real Kp = 1.0, Ki = 0.0, Kd = 0.0;
        Real N  = 10.0;                        // Tf = Kd / (Kp N); Tf = 0 when Kd = 0
        Real Ts = 0.01;
        Real uMin = -INF, uMax = INF;
        bool derivativeOnMeasurement = true;
        Real setpointWeight = 1.0;             // b in Eq. (10-1)
        enum class AntiWindup { None, Clamping, BackCalculation } antiWindup = AntiWindup::Clamping;
        Real trackingTime = -1.0;              // Tt for back-calculation; < 0: sqrt(Ti Td) or Ti
    };
    explicit PID(const Config& cfg);
    Vector compute(const Vector& measurement, const Vector& reference, Real t) override;   // sizes 1,1 -> 1
    Real   compute(Real y, Real r);
    void   reset() override;
    Real   Ts() const override { return cfg_.Ts; }
    Index  inputs()  const override { return 1; }
    Index  outputs() const override { return 1; }
    const Config& config() const { return cfg_; }
    systems::TransferFunction toTransferFunction() const;   // Eq. (10-2)
    Real derivativeTimeConstant() const;
private:
    Config cfg_;
    Real integral_ = 0.0, derivative_ = 0.0, prevMeasurement_ = 0.0, prevError_ = 0.0;
    bool first_ = true;
};
}

Real PID::compute(Real y, Real r) {
    const Real e = r - y, Tf = derivativeTimeConstant(), Ts = cfg_.Ts;
    if (first_) { prevMeasurement_ = y; prevError_ = e; first_ = false; }
    const Real p = cfg_.Kp * (cfg_.setpointWeight * r - y);
    const Real dInput = cfg_.derivativeOnMeasurement ? -(y - prevMeasurement_) : (e - prevError_);
    if (cfg_.Kd != 0.0) derivative_ = (Tf * derivative_ + cfg_.Kd * dInput) / (Tf + Ts);
    const Real iCandidate = integral_ + cfg_.Ki * Ts * e;
    const Real uUnsat = p + iCandidate + derivative_;
    const Real u = std::clamp(uUnsat, cfg_.uMin, cfg_.uMax);
    switch (cfg_.antiWindup) {                                   // Listing 10-3 completes the two strategies
        case Config::AntiWindup::None:            integral_ = iCandidate; break;
        case Config::AntiWindup::Clamping:        integral_ = iCandidate; break;
        case Config::AntiWindup::BackCalculation: integral_ = iCandidate; break;
    }
    prevMeasurement_ = y; prevError_ = e;
    return u;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V23.1 | `K_p = 2` only: `compute(0.5, 1.0)` | `1.0` |
| V23.2 | `K_i = 1`, `T_s = 0.1`, `e = 1` for ten calls | `K_p + 1.0` after the tenth |
| V23.3 | `toTransferFunction()` for `K_p = 2, K_i = 1, K_d = 0.5, N = 10` | isApprox `{0.55, 2.025, 1}/{0.025, 1, 0}` |
| V23.4 | E1 with PI `K_p = 1, K_i = 1`: `feedback(C·P).dcGain()`; overshoot from `stepInfo` | `1`; record the value for V24.2 |

**Exit criteria.**
- [ ] V23.1–V23.4; the purpose of the derivative filter and derivative-on-measurement can be stated
- [ ] Commit

**References.** [R8, ch. 11]; [R9, §4.3]; [R1, §7.3.3, §15.2–15.3].

---

### Day 24 — Memory Model, Smart Pointers and the Closed-Loop Simulator

**Objectives.** C++: stack versus heap storage, dynamic allocation with `new`/`delete` and its hazards, RAII, `std::unique_ptr` and `std::make_unique`, `std::shared_ptr`, ownership transfer by move, `nullptr`. Control: the sampled-data control loop, the algebraic-loop condition, sample-time selection.

**Background.** Local objects live in the function's stack frame and are destroyed on return; heap storage is requested at run time. Containers and Eigen objects hold their data on the heap and release it in their destructors (RAII), which makes manual `new`/`delete` unnecessary; the latter leaks on early return or exception and is prohibited (Table 3-2). Polymorphic objects cannot be held by value (abstract bases), so the Simulator owns its controller through `std::unique_ptr<Controller>`: sole ownership, automatic release, transferable only by `std::move`. `std::shared_ptr` is reserved for genuinely shared ownership. Objects that are only used are passed by reference.

Figure 10-1 shows the loop the Simulator reproduces: the controller executes every `T_s`; its output is held constant (zero-order hold) while the continuous plant is integrated with RK4 sub-steps. A plant with `D ≠ 0` would create an algebraic loop and is rejected. The hold introduces an effective delay `T_s/2`; the sample-time rule is `T_s ≤ t_r/10`, equivalently `ω_s = 2π/T_s ≥ 20 ω_b`.

**Figure 10-1 — Sampled-data control loop**

```
            r(kTs)                  u(kTs)                                          y(t)
  reference ──────▶ ┌────────────┐ ───────▶ ┌─────┐  u(t)   ┌──────────────────┐ ─────┐
                    │ Controller │          │ ZOH │ ───────▶│ Plant  ẋ = Ax+Bu │      │
            ┌─────▶ │ (every Ts) │          └─────┘   + d(t) │        y = Cx    │      │
            │       └────────────┘                           └──────────────────┘      │
            │   measurement y(kTs) [+ noise, Day 50] [→ Estimator x̂, Day 39]           │
            └──────────────────────────────────────────────────────────────────────────┘
```

**Implementation.**

| File | Action |
|---|---|
| `include/csl/simulation/Simulator.hpp`, `src/simulation/Simulator.cpp` | Create (Listing 10-2) |
| `tests/simulation/test_Simulator.cpp` | Create |
| `examples/ex03_classical_design.cpp` | Begin: E1 with P and PI control |

**Listing 10-2 — `Simulator`**

```cpp
#pragma once
#include <functional>
#include <memory>
#include "csl/control/Controller.hpp"
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/StateSpace.hpp"
namespace csl::estimation { class Estimator; }                 // defined on Day 39
namespace csl::simulation {
class Simulator {
public:
    using Signal = std::function<Vector(Real t)>;
    Simulator(systems::StateSpace plant, std::unique_ptr<control::Controller> controller);
    void setReference(Signal r)         { reference_ = std::move(r); }
    void setInputDisturbance(Signal d)  { disturbance_ = std::move(d); }
    void setOutputDisturbance(Signal d) { outputDisturbance_ = std::move(d); }
    void setPlantSubsteps(int n)        { substeps_ = n; }       // RK4 steps per controller period (default 10)
    void setEstimator(std::unique_ptr<estimation::Estimator> est);   // Day 39
    void setMeasurementNoise(Real stdDev, unsigned seed = 1);        // Day 50
    SimResult run(const Vector& x0, Real tf);
private:
    systems::StateSpace plant_;
    std::unique_ptr<control::Controller> controller_;
    std::unique_ptr<estimation::Estimator> estimator_;
    Signal reference_, disturbance_, outputDisturbance_;
    int substeps_ = 10;
};
}

SimResult Simulator::run(const Vector& x0, Real tf) {
    if (plant_.D().norm() != 0.0) throw InvalidArgument("Simulator: plant must have D = 0");
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
        const Vector ref = reference_ ? reference_(t) : Vector::Zero(controller_->inputs());
        u = controller_->compute(y, ref, t);                      // Day 39: measurement = estimate when present
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

Usage: `Simulator sim(plants::thermal(), std::make_unique<PID>(cfg)); sim.setReference([](Real) { return Vector::Constant(1, 1.0); }); auto r = sim.run(Vector::Zero(1), 10.0);`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V24.1 | E1, `K_p = 2`, `T_s = 0.001`: final `y`; closed-loop time constant | `0.8571 ± 1e-3`; `τ/(1 + K_pK) = 0.2857` |
| V24.2 | E1 with the Day 23 PI at `T_s = 0.001`: overshoot | within `0.3 %` of V23.4 |
| V24.3 | Same with `T_s = 0.5` | overshoot visibly larger (hold delay) |
| V24.4 | After `std::make_unique<PID>` is moved into the Simulator, the source pointer | `nullptr` |

**Exit criteria.**
- [ ] V24.1–V24.4; a `new`/`delete` pair was written in `sandbox/`, understood and removed
- [ ] Commit

**References.** [R1, §12.1, §13.6.1]; [R2, ch. 19, 22]; [R8, §11.5]; [R9, §8.1–8.2].

---

### Day 25 — Anti-Windup and Setpoint Weighting

**Objectives.** C++: strategy selection by `enum class` and `switch`; behavioural tests over trajectories; refactoring under test. Control: integrator windup, clamping, back-calculation, setpoint weighting, bumpless transfer.

**Background.** When the actuator saturates while the error persists, the integral term grows beyond what the actuator can deliver (*windup*) and must unwind before the output recovers, producing a long overshoot. *Clamping* stops integration while the output is saturated in the direction of the error. *Back-calculation* feeds `(u_sat − u_unsat)/T_t` into the integrator, with tracking time `T_t = √(T_i T_d)` or `T_i`. *Setpoint weighting* `b ∈ [0, 1]` applies the proportional term to `b r − y`, reducing reference-step overshoot without changing the disturbance response (a two-degree-of-freedom structure). *Bumpless transfer* initialises the integrator so that `u` is continuous on mode change.

**Implementation.**

| File | Action |
|---|---|
| `src/control/PID.cpp` | Complete the `switch` of Listing 10-1 with Listing 10-3 |
| `tests/control/test_PID.cpp` | Add comparative trajectory tests |

**Listing 10-3 — Anti-windup strategies**

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

**Verification.** Plant E3, PI `K_p = 100`, `K_i = 200`, `T_s = 0.001`, reference `0.3 rad/s`, voltage limit `±5 V`.

| ID | Check | Expected |
|---|---|---|
| V25.1 | Overshoot with `None` versus `Clamping` | `None ≥ 2 × Clamping` |
| V25.2 | `BackCalculation` overshoot | `≤ None` |
| V25.3 | No limits: all three strategies | identical trajectories |
| V25.4 | `b = 0.5` versus `b = 1`: response after an input disturbance at `t = 2 s` | identical to `1e-9`; reference-step overshoot reduced |

**Exit criteria.**
- [ ] V25.1–V25.4 as tests with recorded bounds
- [ ] Commit

**References.** [R8, §11.4]; [R9, §4.3.3].

---

### Day 26 — Tuning Rules and Relay Auto-Tuning

**Objectives.** C++: `std::pair`/`std::tuple`, structured bindings, result structs, a minimal second `Controller` (`Relay`), peak detection on `SimResult`. Control: Ziegler–Nichols step and ultimate methods, SIMC, relay feedback.

**Background.** A first-order-plus-dead-time (FOPDT) model `K e^{−Ls}/(Ts + 1)` is fitted to an open-loop step response by the steepest tangent (`L`, `T`) and the final value (`K`). The tuning rules used are:

| Rule | `K_p` | `T_i` | `T_d` | Source |
|---|---|---|---|---|
| ZN step, PID | `1.2 T/(K L)` | `2L` | `0.5L` | [R8, §11.3] |
| ZN step, PI | `0.9 T/(K L)` | `3L` | — | |
| ZN ultimate, PID | `0.6 K_u` | `T_u/2` | `T_u/8` | |
| ZN ultimate, PI | `0.45 K_u` | `T_u/1.2` | — | |
| SIMC, PI | `T/(K(τ_c + L))` | `min(T, 4(τ_c + L))` | — | [R38] (default `τ_c = L`) |

with `K_i = K_p/T_i`, `K_d = K_p T_d`. The ultimate gain `K_u` and period `T_u` are the linear gain margin and `2π/ω_pc` (Day 18) when a model exists, or are measured by *relay feedback*: with a relay of amplitude `d` in the loop, the oscillation amplitude `a` and period give `K_u ≈ 4d/(πa)`, `T_u` = period.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/Relay.hpp` | Create: `Relay(amplitude, hysteresis, Ts)` implementing `Controller` (output `+d` for `e > h`, `−d` for `e < −h`, otherwise previous output) |
| `include/csl/control/Tuning.hpp`, `src/control/Tuning.cpp` | Create (Listing 10-4) |
| `tests/control/test_Tuning.cpp` | Create |

**Listing 10-4 — Tuning interface**

```cpp
#pragma once
#include "csl/control/PID.hpp"
#include "csl/simulation/SimResult.hpp"
#include "csl/systems/LTISystem.hpp"
namespace csl::control::tuning {
struct FOPDT { Real K, L, T; systems::TransferFunction model(int padeOrder = 2) const; };
FOPDT fitFOPDT(const simulation::SimResult& openLoopStep, Index outputIndex = 0, Real stepAmplitude = 1.0);
struct UltimatePoint { Real Ku, Tu; };
UltimatePoint ultimateFromModel(const systems::LTISystem& plant);                               // via margins()
UltimatePoint relayExperiment(const systems::StateSpace& plant, Real relayAmplitude, Real Ts, Real tf, Real hysteresis = 0.0);
enum class Rule { ZN_Step_PID, ZN_Step_PI, ZN_Ultimate_PID, ZN_Ultimate_PI, SIMC_PI };
PID::Config tune(Rule rule, const FOPDT& m, Real Ts, Real tauC = -1.0);
PID::Config tune(Rule rule, const UltimatePoint& u, Real Ts);
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V26.1 | `ultimateFromModel({1}/{1,3,2,0})` | `K_u = 6.00 ± 0.02`, `T_u = 4.443 ± 0.01` |
| V26.2 | `relayExperiment` on the same plant, `d = 1` | `K_u` within 15 % of 6; `T_u` within 3 % |
| V26.3 | `fitFOPDT(step(E1))`; `fitFOPDT(step(E1 · pade(0.5, 3)))` | `K = 3, L ≈ 0, T ≈ 2`; `L = 0.5 ± 0.05`, `T = 2 ± 0.1` |
| V26.4 | SIMC PI on V26.3's second model (`τ_c = L`): gains; closed loop at `T_s = 0.01` | `K_p = 0.667`, `T_i = 2`; overshoot `< 5 %`, settling `< 6 s` |
| V26.5 | ZN ultimate PID on `{1}/{1,3,2,0}` | overshoot in `[25 %, 60 %]` |

**Exit criteria.**
- [ ] V26.1–V26.5
- [ ] Commit

**References.** [R8, §11.3]; [R38, §1–4]; [R1, §17.1]; [R3, structured binding].

---

### Day 27 — Lead/Lag Compensation and Root Finding

**Objectives.** C++: `bisection` and `goldenSectionMin` as library primitives over `std::function`; design functions that search and report. Control: error constants and system type; lead and lag compensator design on the Bode plot.

**Background.** For unity feedback with loop `L`: position constant `K_p = L(0)`, velocity constant `K_v = lim_{s→0} sL(s)`, acceleration constant `K_a = lim s²L(s)`; the system *type* is the number of integrators in `L`. A *lead* compensator `C(s) = K(s + z)/(s + p)`, `z < p`, `α = p/z`, contributes maximum phase

```
φ_max = asin((α − 1)/(α + 1))   at   ω_m = √(zp),   where |C| = +10 log₁₀ α dB relative to K         (10-3)
```

Design procedure: (1) fix `K` from the error-constant specification; (2) `φ = PM_target − PM(KP) + safety` (`5°–12°`); (3) `α = (1 + sin φ)/(1 − sin φ)`; (4) find `ω_m` where `|KP(jω_m)| = −10 log₁₀ α` dB by bisection; (5) `z = ω_m/√α`, `p = ω_m√α`; (6) verify with `margins`. A *lag* compensator (`z > p`, both a decade below `ω_gc`) raises the low-frequency gain by `z/p` with a phase-margin loss of about `5°`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/core/NumericalMethods.hpp`, `src/core/NumericalMethods.cpp` | Create: `Real bisection(const std::function<Real(Real)>&, Real a, Real b, Real tol = 1e-10, int maxIter = 200)`; `Real goldenSectionMin(const std::function<Real(Real)>&, Real a, Real b, Real tol = 1e-8)` |
| `include/csl/control/LeadLag.hpp`, `src/control/LeadLag.cpp` | Create (Listing 10-5) |
| tests | `tests/core/test_NumericalMethods.cpp`, `tests/control/test_LeadLag.cpp` |

**Listing 10-5 — Lead/lag interface and lead design**

```cpp
namespace csl::control {
systems::TransferFunction lead(Real z, Real p, Real K = 1.0);     // K (s+z)/(s+p), z < p
systems::TransferFunction lag (Real z, Real p, Real K = 1.0);     // z > p
struct ErrorConstants { Real Kp, Kv, Ka; int systemType; };
ErrorConstants errorConstants(const systems::TransferFunction& L);
struct LeadDesign { systems::TransferFunction C; Real z, p, K, phiMax_deg, wm, achievedPM_deg, achievedGM_dB; };
LeadDesign designLead(const systems::LTISystem& P, Real K, Real targetPM_deg, Real safety_deg = 8.0);
LeadDesign designLag (const systems::LTISystem& P, Real K, Real targetPM_deg, Real decadeBelow = 10.0);
}

LeadDesign designLead(const systems::LTISystem& P, Real K, Real targetPM, Real safety) {
    auto magdB = [&](Real w) { return dB(std::abs(K * P.evaluate(Complex(0, w))(0, 0))); };
    const Margins m0 = margins(/* K*P as a TransferFunction */);
    const Real phi = deg2rad(targetPM - m0.phaseMargin_deg.value_or(0.0) + safety);
    if (phi >= deg2rad(70)) throw InvalidArgument("designLead: more than 70 deg required; cascade two leads");
    const Real alpha = (1 + std::sin(phi)) / (1 - std::sin(phi));                 // Eq. (10-3) inverted
    const Real wm = bisection([&](Real w) { return magdB(w) + 10.0 * std::log10(alpha); }, 1e-3, 1e3);
    LeadDesign d; d.K = K; d.wm = wm; d.z = wm / std::sqrt(alpha); d.p = wm * std::sqrt(alpha);
    d.phiMax_deg = rad2deg(std::asin((alpha - 1) / (alpha + 1)));
    d.C = lead(d.z, d.p, K * d.p / d.z);                                            // dc gain K preserved
    const Margins m1 = margins(series(d.C, /* P */));
    d.achievedPM_deg = m1.phaseMargin_deg.value_or(NAN); d.achievedGM_dB = m1.gainMargin_dB.value_or(INF);
    return d;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V27.1 | `bisection` on `x² − 2` in `[0, 2]`; `goldenSectionMin` on `(x − 1)²` | `√2`; `1` (to tolerance) |
| V27.2 | `lead(1, 10)`: `φ_max`; `ω_m` | `54.9°`; `3.162` |
| V27.3 | `errorConstants(10/(s(s+1)))` | type 1, `K_v = 10` |
| V27.4 | `designLead(1/(s(s+1)), K = 10, PM = 45°)`: achieved PM; new `ω_gc` | `[45°, 52°]`; `≈ 4.5–5 rad/s` |
| V27.5 | `designLag({1}/{1,3,2,0}, K = 5, PM = 45°)` | achieved PM `≥ 44°` |

**Exit criteria.**
- [ ] V27.1–V27.5
- [ ] Commit

**References.** [R9, §6.7]; [R10, §11.1–11.5].

---

### Day 28 — Loop Shaping, the Gang of Four and Phase 4 Review

**Objectives.** C++: composition of existing functions; `const`-correctness review of `control/` and `analysis/`. Control: sensitivity-based loop assessment, bandwidth, peak sensitivity, Bode's sensitivity integral, loop-shaping rules.

**Background.** `|S| ≪ 1` is required at low frequency (tracking, disturbance rejection) and `|T| ≪ 1` at high frequency (noise rejection); `S + T = 1` makes the crossover region decisive. The *bandwidth* `ω_b` is where `|T|` falls to `−3 dB`. The *peak sensitivity* `M_s = max_ω |S(jω)|` is the reciprocal of the minimum distance from the Nyquist curve to `−1`, and guarantees

```
GM ≥ M_s/(M_s − 1),   PM ≥ 2 asin(1/(2M_s))                                                  (10-4)
```

(`M_s ≤ 2` gives `GM ≥ 6 dB`, `PM ≥ 29°`). Bode's sensitivity integral for stable `L` with relative degree ≥ 2 states `∫₀^∞ ln|S(jω)| dω = π Σ Re(p_k)` over unstable open-loop poles (zero for stable `L`). Loop-shaping rules: `|L| ≫ 1` below `ω_b` with slope `−20 dB/decade` through crossover; `|L| ≪ 1` above; right-half-plane zeros and delays bound `ω_b` (Day 20).

**Implementation.**

| File | Action |
|---|---|
| `FrequencyResponse.hpp/.cpp` | Add `struct SensitivityPeak { Real Ms, w, impliedGM_dB, impliedPM_deg; }`; `sensitivityPeak(const TransferFunction& L)` (dense grid, golden-section refinement); `bandwidth(const TransferFunction& T, Real dropDB = −3)`; `sensitivityIntegral(const TransferFunction& L, Real wMax = 1e4, Index n = 20000)` (trapezoid on a logarithmic grid) |
| `plot/Plots.hpp/.cpp` | Add `plotGangOfFour(const GangOfFour&, path)` |
| `examples/ex03_classical_design.cpp` | Complete: E3 FOPDT fit; SIMC PI versus ZN PID; Simulator with `±12 V` saturation and anti-windup; margins, `M_s`, gang of four; lead design on `1/(s(s+1))`; comparison table |
| `sandbox/phase4_review.md` | Phase review |

**Phase 4 review**: (1) the storage layout of a `Simulator` object (stack, heap, ownership, release); (2) why `Simulator` takes `std::unique_ptr<Controller>` and `StateSpace` by value while `margins` takes `const LTISystem&`; (3) windup explained with the trajectories of `u_unsat`, `u_sat` and `I`; (4) margins guaranteed by `M_s = 1.6`, and why actual margins may exceed them; (5) the design order for the specification `K_v ≥ 20`, `PM ≥ 50°`.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V28.1 | `sensitivityPeak({1}/{1,3,2,0})` | `M_s = 1.57` at `0.73 rad/s`; implied `GM = 8.8 dB`, `PM = 37.2°` (both below the actual margins) |
| V28.2 | `sensitivityIntegral` for the same `L`; for `L = 5/((s−1)(s+3))` | `0 ± 0.02`; `π ± 0.05` |
| V28.3 | `bandwidth(feedback(10/(s(s+1))))` | `4.83 ± 0.05 rad/s` |
| V28.4 | `ex03` runs end to end | example only |

**Exit criteria.**
- [ ] V28.1–V28.4; review written
- [ ] Commit `Day 28: Phase 4 complete`; tag `phase-4`

**References.** [R8, §11.1–11.2, ch. 12, §12.4]; [R9, §6.7.5–6.8].

---
## 11. Phase 5 — Discrete Time and Digital Signal Processing (Days 29–36)

**Phase objective.** Implement discretization (`c2d`, `d2c`), the z-domain transfer-function controller, the FFT, spectral estimation, digital filter structures and design, and the digital-control experiments.

### Day 29 — Sampling and Zero-Order-Hold Discretization

**Objectives.** C++: `enum class` with exhaustive `switch`, Eigen matrix exponential and logarithm. Control: sampling, the zero-order hold, the exact discrete model, pole mapping, aliasing, the Nyquist frequency.

**Background.** A digital controller observes `y(kT_s)` and holds `u` constant over `[kT_s, (k+1)T_s)`. Over one period the continuous plant integrates exactly to

```
x[k+1] = A_d x[k] + B_d u[k],   A_d = e^{A T_s},   B_d = ∫₀^{T_s} e^{Aτ} dτ · B,   C_d = C,  D_d = D     (11-1)
```

Both matrices are obtained from one exponential of the augmented matrix: `exp([[A, B], [0, 0]] T_s) = [[A_d, B_d], [0, I]]`. Poles map by `z = e^{sT_s}`: the left half-plane maps to the open unit disc, `s = 0` to `z = 1`, `s = ±jπ/T_s` to `z = −1`. Frequencies differing by `2π/T_s` are indistinguishable after sampling (*aliasing*); the *Nyquist frequency* `π/T_s` is the highest representable. The hold has frequency response `(1 − e^{−jωT_s})/(jωT_s)`, approximately a delay `T_s/2`. `d2c` for ZOH takes the logarithm of `[[A_d, B_d], [0, I]]` and divides by `T_s`.

**Implementation.**

| File | Action |
|---|---|
| `Conversions.hpp/.cpp` | Add Listing 11-1 (the `Tustin` and `MatchedPoleZero` branches are completed on Day 30) |
| `LTISystem.cpp` | `damp` maps discrete poles by `s = ln(z)/T_s` |
| `tests/systems/test_Conversions.cpp` | Extend |

**Listing 11-1 — Discretization of state-space models**

```cpp
enum class DiscretizationMethod { ZOH, Tustin, ForwardEuler, BackwardEuler, MatchedPoleZero };
StateSpace       c2d(const StateSpace& sys, Real Ts, DiscretizationMethod m = DiscretizationMethod::ZOH, Real prewarp_w = 0.0);
TransferFunction c2d(const TransferFunction& H, Real Ts, DiscretizationMethod m = DiscretizationMethod::Tustin, Real prewarp_w = 0.0);
StateSpace       d2c(const StateSpace& sys, DiscretizationMethod m = DiscretizationMethod::ZOH);
TransferFunction d2c(const TransferFunction& H, DiscretizationMethod m = DiscretizationMethod::Tustin);

#include <unsupported/Eigen/MatrixFunctions>
StateSpace c2d(const StateSpace& sys, Real Ts, DiscretizationMethod m, Real prewarp_w) {
    if (!sys.isContinuous()) throw InvalidArgument("c2d: system is already discrete");
    if (Ts <= 0) throw InvalidArgument("c2d: Ts must be > 0");
    const Index n = sys.order(), nu = sys.inputs();
    const Matrix I = Matrix::Identity(n, n);
    switch (m) {
        case DiscretizationMethod::ZOH: {
            Matrix M = Matrix::Zero(n + nu, n + nu);
            M.topLeftCorner(n, n) = sys.A() * Ts;
            M.topRightCorner(n, nu) = sys.B() * Ts;
            const Matrix E = M.exp();                                                   // Eq. (11-1)
            return StateSpace(E.topLeftCorner(n, n), E.topRightCorner(n, nu), sys.C(), sys.D(), Ts);
        }
        case DiscretizationMethod::ForwardEuler:                                       // s = (z - 1)/Ts
            return StateSpace(I + Ts * sys.A(), Ts * sys.B(), sys.C(), sys.D(), Ts);
        case DiscretizationMethod::BackwardEuler: {                                    // s = (z - 1)/(Ts z)
            const Matrix Ainv = (I - Ts * sys.A()).partialPivLu().inverse();
            return StateSpace(Ainv, Ainv * sys.B() * Ts, sys.C() * Ainv, sys.D() + sys.C() * Ainv * sys.B() * Ts, Ts);
        }
        case DiscretizationMethod::Tustin: {                                           // s = a (z - 1)/(z + 1)
            const Real a = (prewarp_w > 0) ? prewarp_w / std::tan(prewarp_w * Ts / 2) : 2.0 / Ts;
            const Matrix Minv = (a * I - sys.A()).partialPivLu().inverse();
            return StateSpace(Minv * (a * I + sys.A()), std::sqrt(2 * a) * Minv * sys.B(),
                              std::sqrt(2 * a) * sys.C() * Minv, sys.D() + sys.C() * Minv * sys.B(), Ts);
        }
        case DiscretizationMethod::MatchedPoleZero:
            return tf2ss(c2d(ss2tf(sys), Ts, m, prewarp_w));
    }
    throw InvalidArgument("c2d: unknown method");
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V29.1 | E1, `T_s = 0.1`: `A_d`; `B_d` | `e^{−0.05} = 0.951229`; `3(1 − e^{−0.05}) = 0.146313` (to `1e-9`) |
| V29.2 | `step(c2d(E1, 0.1)).y` versus `step(E1)` at sample instants | equal to `1e-10` |
| V29.3 | `c2d(E2, 0.1).poles()` | modulus `0.97531`, angle `±0.19843 rad` |
| V29.4 | `d2c(c2d(E3, 0.05))` | `A`, `B` recovered to `1e-8` |
| V29.5 | `damp(c2d(E2, 0.1))` | `ω_n = 2`, `ζ = 0.125` |
| V29.6 | `sin(2π·450t)` sampled at 500 Hz | indistinguishable from 50 Hz (example only) |

**Exit criteria.**
- [ ] V29.1–V29.6
- [ ] Commit

**References.** [R9, §8.1–8.3]; [R11, §2.1–2.4]; [R4, "MatrixFunctions module"]; [R1, §19.3].

---

### Day 30 — The z-Domain, Tustin and Matched Pole–Zero Mapping, Transfer-Function Controller

**Objectives.** C++: polynomial composition by substitution, a generic `Controller` adapter for discrete transfer functions. Control: the z-transform, difference equations, the `s → z` maps and their properties, frequency warping and prewarping.

**Background.** `Z{x[k−1]} = z⁻¹X(z)`: a one-sample delay is multiplication by `z⁻¹`. The discrete transfer function `H(z) = (b₀zⁿ + … + bₙ)/(zⁿ + a₁zⁿ⁻¹ + … + aₙ)` is the difference equation

```
y[k] = b₀u[k] + b₁u[k−1] + … + bₙu[k−n] − a₁y[k−1] − … − aₙy[k−n]                                  (11-2)
```

| Method | Substitution | Stability preserved | Note |
|---|---|---|---|
| Forward Euler | `s = (z − 1)/T_s` | no | avoid |
| Backward Euler | `s = (z − 1)/(T_s z)` | yes | adds damping |
| Tustin | `s = (2/T_s)(z − 1)/(z + 1)` | yes (`jω` axis ↦ unit circle) | warps frequency: `ω_d = (2/T_s) atan(ω_c T_s/2)`; prewarping at `ω₀` uses `a = ω₀/tan(ω₀T_s/2)` |
| Matched pole–zero | `z = e^{sT_s}` for each pole and zero; zeros at `z = −1` added to relative degree 1; dc gain matched | yes | controllers |
| Zero-order hold | Eq. (11-1) | yes | plants |

For polynomials, Tustin substitution of `p(s) = Σ pᵢ s^{d−i}` with `s = a(z−1)/(z+1)` and common factor `(z+1)^n` gives `Σ pᵢ a^{d−i} (z−1)^{d−i} (z+1)^{n−d+i}`.

**Implementation.**

| File | Action |
|---|---|
| `Conversions.cpp` | Implement `c2d(const TransferFunction&, …)` (Listing 11-2) and `d2c(const TransferFunction&, Tustin)` (inverse substitution) |
| `include/csl/control/TransferFunctionController.hpp`, `src/control/TransferFunctionController.cpp` | Create: `explicit TransferFunctionController(TransferFunction discreteH)` implementing `Controller` through Eq. (11-2) with input `e = r − y` and two history vectors |
| tests | Extend `test_Conversions.cpp`; create `tests/control/test_TransferFunctionController.cpp` |

**Listing 11-2 — Tustin and matched pole–zero on transfer functions**

```cpp
static Polynomial bilinearSubstitute(const Polynomial& p, Real a, Index n) {
    const Polynomial zm1{1.0, -1.0}, zp1{1.0, 1.0};
    Polynomial out;
    const Index d = p.degree();
    for (Index i = 0; i <= d; ++i) {
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
            auto mapRoots = [&](const std::vector<Complex>& r) { std::vector<Complex> o; for (auto& c : r) o.push_back(std::exp(c * Ts)); return o; };
            std::vector<Complex> z = mapRoots(H.zeros()), p = mapRoots(H.poles());
            while (z.size() + 1 < p.size()) z.push_back(Complex(-1.0, 0.0));
            const TransferFunction G = TransferFunction::fromZPK(z, p, 1.0, Ts);
            return TransferFunction::fromZPK(z, p, H.dcGain() / G.dcGain(), Ts);
        }
        default:
            return ss2tf(c2d(tf2ss(H), Ts, m, prewarp_w));
    }
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V30.1 | `c2d({1},{1,1}, 0.1, Tustin)`; its dc gain | isApprox `{0.047619, 0.047619}/{1, −0.904762}`; `1` |
| V30.2 | `ss2tf(c2d(tf2ss(E2), T_s, Tustin))` versus `c2d(E2_tf, T_s, Tustin)` | isApprox |
| V30.3 | Tustin with prewarp `ω₀ = 10`, `T_s = 0.1`: `|H_d(e^{j·1.0})|` versus `|E2(j10)|` | equal to `1e-9`; without prewarp differs by several percent |
| V30.4 | MPZ of `1/(s+1)`, `T_s = 0.1` | pole `0.904837`, one zero at `−1`, dc gain 1 |
| V30.5 | `TransferFunctionController` with `H(z) = T_s/(z − 1)` and `e = 1` | output `(k − 1)T_s` at step `k` |
| V30.6 | E2 at `T_s = 0.5`: forward Euler; backward Euler; Tustin | unstable; stable; stable |

**Exit criteria.**
- [ ] V30.1–V30.6
- [ ] Commit

**References.** [R9, §8.3–8.4]; [R11, §2.5–2.8, §8.2].

---

### Day 31 — The DFT and the FFT

**Objectives.** C++: bitwise operators, power-of-two tests, bit reversal, in-place algorithms on non-const references, `std::swap`, `std::chrono` timing. DSP: the discrete Fourier transform, frequency bins and resolution, conjugate symmetry, Parseval's relation, the radix-2 Cooley–Tukey algorithm.

**Background.**

```
X[k] = Σ_{n=0}^{N−1} x[n] e^{−j2πkn/N},   x[n] = (1/N) Σ_{k=0}^{N−1} X[k] e^{+j2πkn/N},   k, n = 0…N−1     (11-3)
```

Bin `k` corresponds to `f_k = k f_s/N`; for real `x`, `X[N−k] = conj(X[k])`, so bins `0…N/2` suffice. Resolution is `Δf = f_s/N`; zero padding interpolates the spectrum without improving resolution. Parseval: `Σ|x|² = (1/N)Σ|X|²`. Splitting `x` into even and odd samples gives `X[k] = E[k] + W_N^k O[k]`, `X[k + N/2] = E[k] − W_N^k O[k]`, `W_N = e^{−j2π/N}`; applied recursively this is `O(N log N)`. The iterative form permutes the input into bit-reversed order and performs `log₂N` butterfly stages of span `2, 4, …, N` (Figure 11-1).

**Figure 11-1 — Eight-point radix-2 FFT signal flow (after bit reversal)**

```
 x[0] ─┬─▶ ⊕ ──┬─▶ ⊕ ──────┬─▶ ⊕ ─▶ X[0]
 x[4] ─┴─▶ ⊖ ──┼─▶ ⊕·W⁰────┼─▶ ⊕ ─▶ X[1]        stage 1: span 2,  W_2
 x[2] ─┬─▶ ⊕ ──┴─▶ ⊖ ──────┼─▶ ⊕ ─▶ X[2]        stage 2: span 4,  W_4^k
 x[6] ─┴─▶ ⊖ ────▶ ⊖·W¹────┼─▶ ⊕ ─▶ X[3]        stage 3: span 8,  W_8^k
 x[1] ─┬─▶ ⊕ ──┬─▶ ⊕ ──────┼─▶ ⊖ ─▶ X[4]
 x[5] ─┴─▶ ⊖ ──┼─▶ ⊕·W⁰────┼─▶ ⊖ ─▶ X[5]        each butterfly: (u, v·W) → (u + vW, u − vW)
 x[3] ─┬─▶ ⊕ ──┴─▶ ⊖ ──────┼─▶ ⊖ ─▶ X[6]
 x[7] ─┴─▶ ⊖ ────▶ ⊖·W¹────┴─▶ ⊖ ─▶ X[7]
```

**Implementation.**

| File | Action |
|---|---|
| `include/csl/dsp/FFT.hpp`, `src/dsp/FFT.cpp` | Create (Listing 11-3) |
| `tests/dsp/test_FFT.cpp` | Create, including a timing comparison printed to the test log |

**Listing 11-3 — FFT**

```cpp
#pragma once
#include "csl/core/Types.hpp"
namespace csl::dsp {
bool  isPowerOfTwo(Index n);                             // n > 0 && (n & (n - 1)) == 0
Index nextPowerOfTwo(Index n);
Index bitReverse(Index i, int bits);
void  fft (ComplexVector& x);                            // in place; size must be a power of two
void  ifft(ComplexVector& x);                            // in place; includes 1/N
ComplexVector fft (const Vector& x, Index nfft = -1);    // real input, zero-padded to nfft (default: next power of two)
ComplexVector rfft(const Vector& x, Index nfft = -1);    // bins 0..nfft/2
Vector fftFrequencies (Index nfft, Real fs_hz);          // two-sided, negatives folded
Vector rfftFrequencies(Index nfft, Real fs_hz);          // one-sided
ComplexVector dft(const Vector& x);                      // O(N^2) reference; tests only
}

void fft(ComplexVector& x) {
    const Index N = x.size();
    if (!isPowerOfTwo(N)) throw InvalidArgument("fft: length must be a power of two");
    int bits = 0; while ((Index(1) << bits) < N) ++bits;
    for (Index i = 0; i < N; ++i) { const Index j = bitReverse(i, bits); if (j > i) std::swap(x(i), x(j)); }
    for (Index len = 2; len <= N; len <<= 1) {
        const Complex wLen = std::polar(1.0, -TWO_PI / static_cast<Real>(len));
        for (Index start = 0; start < N; start += len) {
            Complex w = 1.0;
            for (Index k = 0; k < len / 2; ++k) {
                const Complex u = x(start + k), v = x(start + k + len / 2) * w;
                x(start + k) = u + v;
                x(start + k + len / 2) = u - v;
                w *= wLen;
            }
        }
    }
}
Index bitReverse(Index i, int bits) { Index r = 0; for (int b = 0; b < bits; ++b) { r = (r << 1) | (i & 1); i >>= 1; } return r; }
void ifft(ComplexVector& x) { x = x.conjugate(); fft(x); x = x.conjugate() / static_cast<Real>(x.size()); }
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V31.1 | `fft` of `[1,0,…,0]` (N = 8); of `[1,…,1]` | all ones; `[8,0,…,0]` |
| V31.2 | `bitReverse(0b0011, 4)` | `0b1100` |
| V31.3 | `fft(x)` versus `dft(x)`, random `N = 1024` | max difference `< 1e-9` |
| V31.4 | `ifft(fft(x))` | `x` to `1e-12` |
| V31.5 | `cos(2π·5t)`, `f_s = 64`, `N = 64` | `|X[5]| = |X[59]| = 32`; other bins `< 1e-9` |
| V31.6 | Parseval; timing at `N = 2¹⁶` (Release) | equality to `1e-9`; `fft` in milliseconds, `dft` in seconds |

**Exit criteria.**
- [ ] V31.1–V31.6
- [ ] Commit

**References.** [R29, §8.1–8.6, §9.1–9.3]; [R30, ch. 8, 12]; [R1, §4.8].

---

### Day 32 — Windows, Leakage, Periodogram and Welch's Method

**Objectives.** C++: `enum class` factories, `Vector::segment`, coefficient-wise products, overlapping-segment index arithmetic, unit-bearing result structs. DSP: spectral leakage, window functions, the periodogram and its variance, Welch averaging, cross-spectral density.

**Background.** A sinusoid that does not complete an integer number of cycles within the record leaks into neighbouring bins through the window's side lobes (rectangular: −13 dB). Tapering windows reduce side lobes at the cost of main-lobe width (Hann −31 dB, Hamming −43 dB, Blackman −58 dB). The one-sided periodogram is

```
P[k] = 2 |X[k]|² / (f_s Σ w²)   for 0 < k < N/2   (factor 1 at k = 0 and N/2)                          (11-4)
```

in units²/Hz; its variance does not decrease with `N`. Welch's method averages periodograms of overlapping windowed segments (default: Hann, 50 % overlap), reducing variance by the segment count at the cost of resolution. Checks: `∫P df` equals the signal variance; white noise of variance `σ²` has `P = 2σ²/f_s`. The cross-spectral density `P_xy` uses `conj(X)·Y` and is required by Day 58.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/dsp/Spectral.hpp`, `src/dsp/Spectral.cpp` | Create (Listing 11-4) |
| `plot/Plots.hpp/.cpp` | Add `plotSpectrum(const Spectrum&, path)` with logarithmic y-axis |
| `tests/dsp/test_Spectral.cpp` | Create |

**Listing 11-4 — Spectral estimation**

```cpp
#pragma once
#include "csl/core/Types.hpp"
namespace csl::dsp {
enum class Window { Rectangular, Hann, Hamming, Blackman };
Vector window(Window w, Index N);                                        // symmetric definitions
struct Spectrum { Vector f_hz; Vector P; };                              // one-sided, units^2/Hz
struct CrossSpectrum { Vector f_hz; ComplexVector Pxy; Vector Pxx, Pyy; };
Spectrum periodogram(const Vector& x, Real fs_hz, Window w = Window::Rectangular, Index nfft = -1);
Spectrum welch(const Vector& x, Real fs_hz, Index nperseg = 256, Real overlap = 0.5, Window w = Window::Hann);
CrossSpectrum csd(const Vector& x, const Vector& y, Real fs_hz, Index nperseg = 256, Real overlap = 0.5, Window w = Window::Hann);
Real integratePSD(const Spectrum& s);                                    // trapezoid over f
}

Spectrum welch(const Vector& x, Real fs, Index nperseg, Real overlap, Window wt) {
    if (nperseg > x.size()) nperseg = x.size();
    const Index nfft = nextPowerOfTwo(nperseg);
    const Vector w = window(wt, nperseg);
    const Real scale = 1.0 / (fs * w.squaredNorm());
    const Index step = std::max<Index>(1, static_cast<Index>(nperseg * (1.0 - overlap)));
    Vector acc = Vector::Zero(nfft / 2 + 1);
    Index count = 0;
    for (Index start = 0; start + nperseg <= x.size(); start += step, ++count) {
        const Vector seg = x.segment(start, nperseg).cwiseProduct(w);
        acc += rfft(seg, nfft).cwiseAbs2() * scale;
    }
    acc /= static_cast<Real>(std::max<Index>(count, 1));
    acc.segment(1, nfft / 2 - 1) *= 2.0;                                   // Eq. (11-4)
    return Spectrum{rfftFrequencies(nfft, fs), acc};
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V32.1 | `window(Hann, 8)` | `[0, 0.1883, 0.6113, 0.9505, 0.9505, 0.6113, 0.1883, 0]` |
| V32.2 | `periodogram` of `A sin(2π·10t)`, `f_s = 1000`, `N = 1000`: peak power × `Δf` | `A²/2`; at 10.5 Hz the power spreads (leakage), reduced by the Hann window |
| V32.3 | `welch` of unit-variance pseudo-white noise (`std::mt19937`, seed fixed, `f_s = 1000`, `N = 65536`): mean level; `integratePSD` | `0.002 ± 10 %`; `1.0 ± 5 %` |
| V32.4 | `plotSpectrum(welch(step(E2) + noise))` | resonance visible at `0.318 Hz` (example only) |

**Exit criteria.**
- [ ] V32.1–V32.4
- [ ] Commit

**References.** [R29, §10.5–10.6]; [R30, ch. 9, 16]; [R31, `welch`].

---

### Day 33 — Digital Filter Structures

**Objectives.** C++: the third abstract base with a non-virtual convenience overload, constructor normalisation, state vectors, `final`, `clone`. DSP: IIR and FIR filters, Direct Form II Transposed, linear phase and group delay, moving-average and exponential-smoothing filters, zero-phase filtering.

**Background.** An IIR filter `H(z) = B(z)/A(z)` is executed in Direct Form II Transposed with `n` state variables:

```
y = b₀x + s₁,   s_i = b_i x − a_i y + s_{i+1}  (i = 1…n−1),   s_n = b_n x − a_n y                         (11-5)
```

An FIR filter `y[k] = Σ_{i=0}^{M} h_i x[k−i]` has no feedback, is always stable, and has exactly linear phase (constant group delay `M/2` samples) when `h` is symmetric. The moving average (`h_i = 1/N`) has nulls at multiples of `f_s/N`; exponential smoothing `y = αx + (1−α)y_prev` is a first-order IIR filter with pole `1 − α`. A filter is a discrete LTI system (C12); forward–backward filtering (`filtfilt`) yields zero phase and squared magnitude and is applicable offline only.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/dsp/DigitalFilter.hpp`, `src/dsp/DigitalFilter.cpp` | Create (Listing 11-5) |
| `tests/dsp/test_DigitalFilter.cpp` | Create |

**Listing 11-5 — Filter structures**

```cpp
#pragma once
#include <memory>
#include "csl/core/Types.hpp"
#include "csl/systems/TransferFunction.hpp"
namespace csl::dsp {
class DigitalFilter { /* Listing 4-5 */ };

class IIRFilter final : public DigitalFilter {
public:
    IIRFilter(Vector b, Vector a, Real Ts);                     // descending powers of z; equal lengths after padding; a(0) normalised to 1
    explicit IIRFilter(const systems::TransferFunction& H);     // discrete, proper
    Real process(Real x) override;                               // Eq. (11-5)
    void reset() override { s_.setZero(); }
    Real Ts() const override { return Ts_; }
    systems::TransferFunction toTransferFunction() const override;
    std::unique_ptr<DigitalFilter> clone() const override { return std::make_unique<IIRFilter>(*this); }
private:
    Vector b_, a_, s_; Real Ts_;
};
class FIRFilter final : public DigitalFilter {
public:
    FIRFilter(Vector h, Real Ts);
    Real process(Real x) override;                               // circular delay line
    void reset() override;
    Real Ts() const override { return Ts_; }
    systems::TransferFunction toTransferFunction() const override;   // num = h, den = z^M
    std::unique_ptr<DigitalFilter> clone() const override { return std::make_unique<FIRFilter>(*this); }
    Real groupDelaySamples() const { return (h_.size() - 1) / 2.0; }
private:
    Vector h_, line_; Index head_ = 0; Real Ts_;
};
Vector    filtfilt(const DigitalFilter& f, const Vector& x);
FIRFilter movingAverage(Index N, Real Ts);
IIRFilter exponentialSmoothing(Real alpha, Real Ts);
}

Real IIRFilter::process(Real x) {
    const Index n = s_.size();
    const Real y = b_(0) * x + (n > 0 ? s_(0) : 0.0);
    for (Index i = 0; i + 1 < n; ++i) s_(i) = b_(i + 1) * x - a_(i + 1) * y + s_(i + 1);
    if (n > 0) s_(n - 1) = b_(n) * x - a_(n) * y;
    return y;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V33.1 | `movingAverage(4)`: `|H|` at `f_s/4`; at `f = 0` | `< 1e-12`; `1` |
| V33.2 | `exponentialSmoothing(0.1)`: pole; dc gain | `0.9`; `1` |
| V33.3 | `H(z) = (0.5z + 0.5)/(z − 0.3)`: `IIRFilter(H).process(step)` versus `step(H).y` | equal sample-wise to `1e-12` |
| V33.4 | `filtfilt` on a sinusoid | zero phase shift |
| V33.5 | Symmetric FIR impulse-response centroid | delayed by `M/2` samples |

**Exit criteria.**
- [ ] V33.1–V33.5
- [ ] Commit

**References.** [R29, §6.3–6.5]; [R30, ch. 14–15, 19].

---

### Day 34 — Filter Design and Function Templates

**Objectives.** C++: function templates, the `Eigen::MatrixBase<Derived>` parameter idiom, template error reading. DSP: Butterworth analog prototype and bilinear digitisation, order selection, windowed-sinc FIR design, the notch biquad.

**Background.** The Butterworth lowpass has `|H(jω)|² = 1/(1 + (ω/ω_c)^{2n})`; its analog poles lie on the circle of radius `ω_c` at

```
p_k = ω_c e^{j[π/2 + (2k−1)π/(2n)]},   k = 1…n                                                           (11-6)
```

The order required for passband attenuation `A_p` at `ω_p` and stopband attenuation `A_s` at `ω_s` is `n ≥ log₁₀[(10^{A_s/10} − 1)/(10^{A_p/10} − 1)] / (2 log₁₀(ω_s/ω_p))`. The digital filter is the bilinear transform of the prototype with prewarping at `ω_c` (Day 30), which places the −3 dB point exactly at `f_c`; highpass prototypes use `s → ω_c/s`. The windowed-sinc FIR lowpass has `h[n] = sin(ω_c(n − M/2))/(π(n − M/2))` (`ω_c/π` at the centre) multiplied by a window; the Hamming transition width is approximately `3.3/M` cycles per sample. The notch biquad

```
H(z) = (1 − 2 cos ω₀ z⁻¹ + z⁻²) / (1 − 2r cos ω₀ z⁻¹ + r² z⁻²),   ω₀ = 2πf₀/f_s,   r = e^{−πBW/f_s}        (11-7)
```

places zeros on the unit circle at `±ω₀` and poles just inside.

A function template `template <typename Derived> Real rms(const Eigen::MatrixBase<Derived>& v)` accepts any Eigen expression without a temporary.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/dsp/FilterDesign.hpp`, `src/dsp/FilterDesign.cpp` | Create (Listing 11-6) |
| `core/MathUtils.hpp` | Add templates `rms`, `clampTo`; `Figure::line` accepts `MatrixBase` expressions |
| `tests/dsp/test_FilterDesign.cpp` | Create |

**Listing 11-6 — Filter design**

```cpp
#pragma once
#include "csl/dsp/DigitalFilter.hpp"
#include "csl/dsp/Spectral.hpp"
namespace csl::dsp {
enum class FilterType { Lowpass, Highpass };
std::vector<Complex>      butterworthPoles(int order, Real wc_rad_s);                                      // Eq. (11-6)
systems::TransferFunction butterworthAnalog(int order, Real wc_rad_s, FilterType t = FilterType::Lowpass);
int                       butterworthOrder(Real wp_rad_s, Real ws_rad_s, Real Ap_dB, Real As_dB);
IIRFilter                 butterworth(int order, Real fc_hz, Real fs_hz, FilterType t = FilterType::Lowpass);
FIRFilter firWindowedSinc(int numTaps, Real fc_hz, Real fs_hz, Window w = Window::Hamming, FilterType t = FilterType::Lowpass);
IIRFilter notch(Real f0_hz, Real bandwidth_hz, Real fs_hz);                                                // Eq. (11-7), dc gain 1
}

IIRFilter butterworth(int order, Real fc, Real fs, FilterType t) {
    const Real wc = TWO_PI * fc;
    const systems::TransferFunction Ha = butterworthAnalog(order, wc, t);
    return IIRFilter(systems::c2d(Ha, 1.0 / fs, systems::DiscretizationMethod::Tustin, wc));   // prewarp at fc
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V34.1 | `butterworthPoles(2, 1)`; `butterworthAnalog(3, 1)`; `|H_a(j1)|` | `−0.7071 ± 0.7071j`; `{1}/{1,2,2,1}`; `−3.0103 dB` |
| V34.2 | `butterworth(4, 100, 1000)`: `|H|` at 100 Hz; at 200 Hz; pole moduli | `−3.01 ± 0.01 dB`; `−27.97 ± 0.05 dB`; `< 1` |
| V34.3 | `butterworthOrder(2π·100, 2π·200, 1, 40)` | `8` |
| V34.4 | `firWindowedSinc(51, 100, 1000)`: group delay; `|H(0)|`; `|H|` at 300 Hz | `25` samples; `1 ± 0.01`; `< −50 dB` |
| V34.5 | `notch(50, 2, 1000)`: `|H|` at 50 Hz; at 40 and 60 Hz; dc gain | `< −40 dB`; `> −1 dB`; `1` |
| V34.6 | `rms(r.y.col(0))` compiles without a temporary | — |

**Exit criteria.**
- [ ] V34.1–V34.6
- [ ] Commit

**References.** [R29, §7.1–7.3, §7.5]; [R30, ch. 16, 20]; [R1, §16.1.1–16.1.3]; [R4, "Writing functions taking Eigen types"].

---

### Day 35 — Digital Control Experiments

**Objectives.** C++: composition and measurement; a measurement-filter hook in the Simulator. Control: emulation versus direct digital design, the phase cost of sampling, dead-beat control, aliasing in the loop.

**Background.** *Emulation* designs `C(s)` in continuous time and discretizes it (Tustin); the hold and the one-sample computation delay cost approximately `ΔPM ≈ ω_gc(T_s/2 + T_s)` radians, so `ω_gc T_s ≤ 0.15–0.5` is required. *Direct design* discretizes the plant (ZOH) and designs in the z-domain, where all Phase 3 tools apply with the unit circle as the stability boundary. *Dead-beat* control places all closed-loop poles at `z = 0`, reaching the reference in `n` samples at the cost of large inputs and exact-model dependence. Measurement noise above `f_s/2` aliases into the control band; an anti-aliasing filter before the sampler is required and contributes phase lag.

**Implementation.**

| File | Action |
|---|---|
| `Simulator.hpp/.cpp` | Add `setMeasurementFilter(std::unique_ptr<dsp::DigitalFilter>)` applied to `y` before the controller |
| `examples/ex04_dsp_and_filters.cpp` | Begin with the experiments below |
| `tests/control/test_DigitalControl.cpp` | Quantitative experiments as tests |

Experiments: (E1) E1 with PI `K_p = K_i = 1`: continuous PM; Tustin-discretized controller and ZOH plant at `T_s ∈ {0.01, 0.1, 0.5}`; tabulate the PM loss against `ω_gc T_s/2` (and with an added `z⁻¹` against `3ω_gc T_s/2`). (E2) Simulator step responses for the three `T_s` versus the continuous closed loop. (E3) Dead-beat on E1 at `T_s = 0.1` (`P(z) = 0.1463/(z − 0.9512)`). (E4) Root locus of `c2d(E2, 0.1)` under proportional gain. (E5) 450 Hz noise on a loop sampled at 500 Hz, with and without `butterworth(2, 200, 500)` as measurement filter.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V35.1 | Measured PM losses (E1) versus prediction, `ω_gc T_s < 0.5` | within `2°` |
| V35.2 | Dead-beat (E3): `|y[1] − r|` | `< 1e-9` |
| V35.3 | z-plane crossing gain (E4) versus discrete gain margin | within `1 %` |
| V35.4 | (E5) 50 Hz alias present without the filter, attenuated with it | example only |

**Exit criteria.**
- [ ] V35.1–V35.4
- [ ] Commit

**References.** [R9, §8.4–8.6]; [R11, ch. 4].

---

### Day 36 — Phase 5 Review

**Objectives.** Zero warnings at `/W4` (library and tests); tests pass in Release; optional Address Sanitizer run (`/fsanitize=address`).

**Implementation.** `examples/ex04_dsp_and_filters.cpp`: (1) chirp 1–100 Hz at `f_s = 1000`; (2) its FFT; (3) `butterworth(4, 30, 1000)` applied, spectra before and after, filter Bode from `toTransferFunction()`; (4) E2 step with noise and a 50 Hz tone, removed by `notch(50, 2, 1000)`, `welch` before and after, `filtfilt` versus `process`; (5) the Day 35 sample-time table. `sandbox/phase5_review.md`: (1) why the FFT permutation is bit reversal and the data flow of one butterfly; (2) why Tustin preserves stability and forward Euler does not; (3) the resolution obtainable from 2 s at 1 kHz, and the effect of zero padding to 8192 points; (4) why a 10 Hz digital PID degrades a loop with crossover at 3 rad/s; (5) where a `DigitalFilter` meets an `LTISystem` in the type system.

**Exit criteria.**
- [ ] `ex04` produces five plots; zero warnings; Release tests pass; review written
- [ ] Commit `Day 36: Phase 5 complete`; tag `phase-5`

---
## 12. Phase 6 — State-Space Design (Days 37–43)

**Phase objective.** Implement controllability and observability tests, pole placement, the `Estimator` base and Luenberger observer, Lyapunov equations, Gramians and the H2 norm, feedforward and integral action, and complete a copy/move audit.

### Day 37 — Controllability and Observability

**Objectives.** C++: numerical rank through singular values with an explicit tolerance, block assembly in loops. Control: Kalman rank tests, the PBH test, duality, stabilizability and detectability.

**Background.** `(A, B)` is *controllable* if every state is reachable from every other in finite time; equivalently

```
rank W_c = n,   W_c = [B, AB, A²B, …, Aⁿ⁻¹B]                                                         (12-1)
```

`(A, C)` is *observable* if the initial state is determined by a finite output record; equivalently `rank W_o = n`, `W_o = [C; CA; …; CAⁿ⁻¹]`. Duality: `(A, C)` observable ⇔ `(Aᵀ, Cᵀ)` controllable. The PBH test, `rank [λI − A, B] = n` for every eigenvalue `λ`, identifies the uncontrollable modes. *Stabilizable*: every unstable mode is controllable; *detectable*: every unstable mode is observable — the conditions required by the Riccati solvers (Days 44–45). A mode that is both uncontrollable and unobservable cancels in `ss2tf`. `W_c` is ill-conditioned for large `n`; the Gramian test (Day 40) is preferred beyond `n ≈ 10`. Numerical rank is the number of singular values exceeding `max(rows, cols)·σ_max·ε`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/core/LinearAlgebra.hpp`, `src/core/LinearAlgebra.cpp` | Create (Listing 12-1) |
| `StateSpace.hpp` | Add `isControllable(tol)`, `isObservable(tol)` delegating to the free functions |
| `tests/core/test_LinearAlgebra.cpp` | Create |

**Listing 12-1 — Controllability and observability**

```cpp
#pragma once
#include <vector>
#include "csl/core/Types.hpp"
namespace csl {
Index  numericalRank(const Matrix& M, Real tol = -1.0);                 // tol < 0: default threshold
Matrix controllabilityMatrix(const Matrix& A, const Matrix& B);         // Eq. (12-1)
Matrix observabilityMatrix  (const Matrix& A, const Matrix& C);
bool   isControllable(const Matrix& A, const Matrix& B, Real tol = -1.0);
bool   isObservable  (const Matrix& A, const Matrix& C, Real tol = -1.0);
std::vector<Complex> uncontrollableModes(const Matrix& A, const Matrix& B, Real tol = -1.0);   // PBH
std::vector<Complex> unobservableModes  (const Matrix& A, const Matrix& C, Real tol = -1.0);
bool   isStabilizable(const Matrix& A, const Matrix& B, Real Ts = 0.0, Real tol = -1.0);
bool   isDetectable  (const Matrix& A, const Matrix& C, Real Ts = 0.0, Real tol = -1.0);
}

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
        const Real t = tol < 0 ? std::max(M.rows(), M.cols()) * svd.singularValues()(0) * EPS : tol;
        if ((svd.singularValues().array() > t).count() < n) bad.push_back(lam);
    }
    return bad;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V37.1 | E4 | controllable and observable |
| V37.2 | `A = −I₂`, `B = [1; 1]` | rank 1; `uncontrollableModes = {−1}`; stabilizable |
| V37.3 | E2 with `C = [0, 1]` | observable |
| V37.4 | `A = diag(1, −1)`, `B = [0; 1]` | not stabilizable |
| V37.5 | `tf2ss({1,1}/{1,3,2})` | controllable; `unobservableModes = {−1}` |
| V37.6 | `numericalRank([[1, 2], [2, 4 + 1e-15]])` | `1` |

**Exit criteria.**
- [ ] V37.1–V37.6
- [ ] Commit

**References.** [R9, §7.4]; [R8, §6.2, §7.1]; [R12, ch. 6].

---

### Day 38 — Pole Placement and the `StateFeedbackController`

**Objectives.** C++: polynomial evaluation at a matrix (Horner), solving instead of inverting, a `Controller` whose measurement is the state. Control: closed-loop pole assignment by state feedback, Ackermann's formula, pole-selection guidelines, feedforward gain.

**Background.** With `u = −Kx`, `ẋ = (A − BK)x`. If `(A, B)` is controllable, for every monic `Δ_d(s)` of degree `n` there is a `K` (unique for SISO) with `det(sI − A + BK) = Δ_d(s)`:

```
K = [0, …, 0, 1] W_c⁻¹ Δ_d(A)                                                                        (12-2)
```

Pole selection: dominant pair from the specifications (Day 16), remaining poles 4–10 times faster; faster poles increase `K`, `u` and noise sensitivity. The same formula with z-plane poles applies to discrete systems; all poles at `z = 0` gives dead-beat control. Reference tracking with `u = −Kx + N̄r`, `N̄ = −1/(C(A − BK)⁻¹B)` (SISO), yields unit dc gain for an exact model.

**Implementation.**

| File | Action |
|---|---|
| `Polynomial.hpp/.cpp` | Implement `evaluate(const Matrix&)` |
| `include/csl/control/StateFeedback.hpp`, `src/control/StateFeedback.cpp` | Create (Listing 12-2) |
| `tests/control/test_StateFeedback.cpp` | Create |

**Listing 12-2 — Pole placement**

```cpp
#pragma once
#include <vector>
#include "csl/control/Controller.hpp"
#include "csl/systems/StateSpace.hpp"
namespace csl::control {
Matrix place(const Matrix& A, const Matrix& B, const std::vector<Complex>& poles);   // SISO Ackermann; throws if uncontrollable
Matrix place(const systems::StateSpace& sys, const std::vector<Complex>& poles);
Real   feedforwardGain(const systems::StateSpace& sys, const Matrix& K);              // Nbar, SISO
class StateFeedbackController final : public Controller {
public:
    StateFeedbackController(Matrix K, Matrix Nbar, Real Ts);     // measurement = x; u = -K x + Nbar r
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

Matrix place(const Matrix& A, const Matrix& B, const std::vector<Complex>& poles) {
    const Index n = A.rows();
    if (B.cols() != 1) throw NotSupported("place: SISO only");
    if (static_cast<Index>(poles.size()) != n) throw InvalidArgument("place: n poles required");
    const Matrix Wc = controllabilityMatrix(A, B);
    if (numericalRank(Wc) < n) throw InvalidArgument("place: (A, B) is not controllable");
    const Polynomial delta = Polynomial::fromRoots(poles);
    Vector en = Vector::Zero(n); en(n - 1) = 1.0;
    const Vector r = Wc.transpose().partialPivLu().solve(en);         // r^T = e_n^T Wc^{-1}
    return r.transpose() * delta.evaluate(A);                         // Eq. (12-2)
}
```

The Simulator supplies the full state by using a plant copy with `C = I` until the observer exists (Day 39).

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V38.1 | E4, poles `−1 ± j` | `K = [2, 2]` |
| V38.2 | E3, poles `{−20, −25}`: `charPoly(A − BK)` | `{1, 45, 500}` to `1e-9` |
| V38.3 | `c2d(E1, 0.1)`, pole `{0}` | `A_d − B_d K = 0` |
| V38.4 | Simulator, E3 with `C = I`, real poles, reference 1 | final `1.0 ± 1e-3`, no overshoot |
| V38.5 | `place` on V37.2's pair | throws `InvalidArgument` |

**Exit criteria.**
- [ ] V38.1–V38.5; Ackermann's formula derived for `n = 2` on paper
- [ ] Commit

**References.** [R9, §7.5]; [R8, §6.2–6.3]; [R12, §8.2].

---

### Day 39 — Luenberger Observer, `Estimator` Base and the Separation Principle

**Objectives.** C++: the `Estimator` base, the Simulator's estimator hook and per-tick operation order, `std::unique_ptr` members. Control: state reconstruction from outputs, observer error dynamics, the current-estimator form, the separation principle, the observer-based compensator.

**Background.** The observer `x̂̇ = Ax̂ + Bu + L(y − Cx̂)` has error dynamics `ė = (A − LC)e`; `L = place(Aᵀ, Cᵀ, poles)ᵀ` by duality. The discrete current-estimator form used by the Simulator and shared with the Kalman filter is

```
predict:  x̄[k+1] = A_d x̂[k] + B_d u[k];    update:  x̂[k+1] = x̄[k+1] + L (y[k+1] − C x̄[k+1])            (12-3)
```

with error dynamics `e[k+1] = (I − LC)A_d e[k]`, so `L = place(A_dᵀ, (CA_d)ᵀ, poles)ᵀ`. Observer poles are chosen 2–6 times faster than the controller's. *Separation principle*: with `u = −Kx̂`, the closed-loop poles are `eig(A − BK) ∪ eig(A − LC)` (Figure 12-1; the `(x, e)` dynamics are block-triangular). The compensator from `y` to `−u`, `K(sI − A + BK + LC)⁻¹L`, is a dynamic system amenable to Phase 3 analysis.

**Figure 12-1 — Observer-based state feedback**

```
   r ──▶(Nbar)──▶ ⊕ ──u──▶ ┌───────────┐──y──▶
                  ▲         │   Plant   │      │
                  │         └───────────┘      │
                  │    ┌──────────────────┐    │
                  └─── │ u = −K x̂         │ ◀──┘
          x̂ ◀───────── │ Observer (A,B,C,L)│ ◀── u
                       └──────────────────┘
   closed-loop poles = eig(A − BK) ∪ eig(A − LC)
```

Per-tick order in the Simulator: `update(y)` → `compute(x̂, r)` → `predict(u)`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/estimation/Estimator.hpp` | Create (Listing 4-5) |
| `include/csl/estimation/LuenbergerObserver.hpp`, `src/estimation/LuenbergerObserver.cpp` | Create (Listing 12-3) |
| `Simulator.hpp/.cpp` | Implement `setEstimator`; measurement passed to the controller is `estimator_->state()` when present |
| `StateFeedback.hpp/.cpp` | Add `StateSpace observerBasedCompensator(const StateSpace& plant, const Matrix& K, const Matrix& L)` |
| tests | `tests/estimation/test_LuenbergerObserver.cpp`; extend `test_Simulator.cpp` |

**Listing 12-3 — Luenberger observer (current-estimator form)**

```cpp
class LuenbergerObserver final : public Estimator {
public:
    LuenbergerObserver(systems::StateSpace discretePlant, Matrix L);
    static Matrix designGain(const systems::StateSpace& discretePlant, const std::vector<Complex>& zPoles);
    void   predict(const Vector& u) override { xbar_ = sys_.A() * xhat_ + sys_.B() * u; predicted_ = true; }
    void   update (const Vector& y) override {
        const Vector& base = predicted_ ? xbar_ : xhat_;
        xhat_ = base + L_ * (y - sys_.C() * base);                 // Eq. (12-3)
        predicted_ = false;
    }
    Vector state()      const override { return xhat_; }
    Matrix covariance() const override { return Matrix::Zero(xhat_.size(), xhat_.size()); }
    void   reset(const Vector& x0) override { xhat_ = x0; predicted_ = false; }
private:
    systems::StateSpace sys_; Matrix L_; Vector xhat_, xbar_; bool predicted_ = false;
};
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V39.1 | E3 at `T_s = 0.01`, observer poles `e^{−0.4}`, `e^{−0.5}`, wrong initial estimate: `‖e[k+1]‖/‖e[k]‖` after 50 steps | `→ e^{−0.4}` |
| V39.2 | Closed-loop poles of `feedback(plant, observerBasedCompensator)` with controller poles `{−20, −25}` and observer poles `{−40, −50}` | `{−20, −25, −40, −50}` to `1e-8` |
| V39.3 | Simulator output with observer versus with true state, after convergence | equal to `1e-3` |
| V39.4 | Exact initial estimate, no noise: `estimator->state()` versus true state at every tick | equal to `1e-12` (operation order) |

**Exit criteria.**
- [ ] V39.1–V39.4; the separation principle proved in `(x, e)` coordinates on paper
- [ ] Commit

**References.** [R9, §7.6–7.7]; [R8, §7.2–7.3]; [R12, §8.3–8.4].

---

### Day 40 — Lyapunov Equations, Gramians and the H2 Norm

**Objectives.** C++: Kronecker products, `vec`/`unvec` with `Eigen::Map`, Cholesky as a definiteness test, symmetrisation. Control: Lyapunov's direct method, Lyapunov equations, controllability and observability Gramians, the H2 norm.

**Background.** If `V = xᵀPx` with `P ≻ 0` satisfies `V̇ = xᵀ(AᵀP + PA)x < 0`, then `ẋ = Ax` is asymptotically stable; `A` is stable if and only if `AᵀP + PA + Q = 0` has a solution `P ≻ 0` for any `Q ≻ 0`. The library's solver uses the convention

```
lyap(A, Q):   A X + X Aᵀ + Q = 0;     dlyap(A, Q):   A X Aᵀ − X + Q = 0                              (12-4)
```

solved through vectorisation: `(I ⊗ A + A ⊗ I) vec X = −vec Q` and `(A ⊗ A − I) vec X = −vec Q`, using `vec(AXB) = (Bᵀ ⊗ A) vec X`. The Gramians `W_c = lyap(A, BBᵀ)` and `W_o = lyap(Aᵀ, CᵀC)` satisfy `W_c = ∫₀^∞ e^{At}BBᵀe^{Aᵀt}dt`; `(A, B)` is controllable iff `W_c ≻ 0`, and `xᵀW_c⁻¹x` is the minimum input energy to reach `x`. The H2 norm of a stable strictly proper system is

```
‖G‖₂² = tr(Bᵀ W_o B) = tr(C W_c Cᵀ)                                                                 (12-5)
```

the energy of the impulse response and the output variance under unit white-noise input.

**Implementation.**

| File | Action |
|---|---|
| `LinearAlgebra.hpp/.cpp` | Add Listing 12-4 and `controllabilityGramian(const StateSpace&)`, `observabilityGramian(const StateSpace&)` (continuous: `lyap`; discrete: `dlyap`) |
| `Stability.hpp/.cpp` | Add `bool isLyapunovStable(const Matrix& A)` (solve `lyap(Aᵀ, I)`; test `LLT` success) |
| `include/csl/analysis/Norms.hpp`, `src/analysis/Norms.cpp` | Create: `Real h2norm(const StateSpace&)` by Eq. (12-5), throwing for unstable or (continuous) non-strictly-proper systems |
| tests | Extend `test_LinearAlgebra.cpp`; create `tests/analysis/test_Norms.cpp` |

**Listing 12-4 — Kronecker product and Lyapunov solvers**

```cpp
Matrix kron(const Matrix& A, const Matrix& B) {
    Matrix K(A.rows() * B.rows(), A.cols() * B.cols());
    for (Index i = 0; i < A.rows(); ++i)
        for (Index j = 0; j < A.cols(); ++j) K.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
    return K;
}
Vector vec(const Matrix& X)                     { return Eigen::Map<const Vector>(X.data(), X.size()); }
Matrix unvec(const Vector& v, Index r, Index c) { return Eigen::Map<const Matrix>(v.data(), r, c); }
Matrix lyap(const Matrix& A, const Matrix& Q) {                                   // Eq. (12-4)
    const Index n = A.rows();
    const Matrix I = Matrix::Identity(n, n), M = kron(I, A) + kron(A, I);
    const Vector x = M.partialPivLu().solve(-vec(Q));
    if ((M * x + vec(Q)).norm() > 1e-8 * (1.0 + vec(Q).norm())) throw NumericalError("lyap: singular");
    const Matrix X = unvec(x, n, n);
    return 0.5 * (X + X.transpose());
}
Matrix dlyap(const Matrix& A, const Matrix& Q) {
    const Index n = A.rows();
    const Matrix M = kron(A, A) - Matrix::Identity(n * n, n * n);
    const Matrix X = unvec(Vector(M.partialPivLu().solve(-vec(Q))), n, n);
    return 0.5 * (X + X.transpose());
}
```

The vectorised solver is `O(n⁶)` and adequate for `n ≤ 30`; Bartels–Stewart is listed in Table 19-1.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V40.1 | `lyap(−1, 2)` | `1` |
| V40.2 | `lyap(A, I)` for E2: residual; definiteness | `< 1e-12`; `X ≻ 0` |
| V40.3 | `controllabilityGramian(E2)` versus numerical `∫₀^{60} e^{At}BBᵀe^{Aᵀt}dt` | equal to `1e-4` |
| V40.4 | `isLyapunovStable` | true for E1–E3; false for E4 and for `[[0, 1], [−4, 0]]` |
| V40.5 | `h2norm({1},{1,1})`; `h2norm(E2)`; discrete `h2norm(0.5/(z − 0.5))` | `0.70711`; `0.5`; `0.57735` |
| V40.6 | `W_c` of V37.2's pair | singular (minimum eigenvalue `< 1e-12`) |

**Exit criteria.**
- [ ] V40.1–V40.6
- [ ] Commit

**References.** [R12, §5.4–5.5]; [R23, ch. 3–4]; [R4, "Map"].

---

### Day 41 — Feedforward and Integral Action

**Objectives.** C++: augmented block matrices, a stateful state-feedback controller with configuration struct. Control: steady-state feedforward `(N_x, N_u)`, integral augmentation, robustness of integral action.

**Background.** Steady state with `y = r` requires

```
[[A, B], [C, D]] [N_x; N_u] = [0; I],   u = −K(x − N_x r) + N_u r = −Kx + N̄r,   N̄ = N_u + K N_x       (12-6)
```

which tracks exactly only for an exact model. Integral augmentation adds `ẋ_I = r − y`:

```
[ẋ; ẋ_I] = [[A, 0], [−C, 0]] [x; x_I] + [B; 0] u + [0; I] r,   u = −K x − K_I x_I                      (12-7)
```

Pole placement or LQR on `(A_aug, B_aug)` yields `[K, K_I]`; the steady state then satisfies `y = r` under constant disturbances and gain errors. The augmented pair is controllable iff the plant has no zero at `s = 0`. Anti-windup applies to `x_I` as on Day 25.

**Implementation.**

| File | Action |
|---|---|
| `StateFeedback.hpp/.cpp` | Add `struct FeedforwardGains { Matrix Nx, Nu, Nbar; }`; `feedforwardGains(sys, K)`; `StateSpace augmentWithIntegrator(const StateSpace&)`; `class IntegralStateFeedbackController` with `Config { Matrix K, Ki, C; Real Ts; Real uMin, uMax; bool antiWindup; }` (`C` supplied so that `y = Cx` is computed from the state measurement) |
| `tests/control/test_StateFeedback.cpp` | Extend |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V41.1 | `feedforwardGains(E3, K).Nbar` versus Day 38's `N̄` | equal to `1e-12` |
| V41.2 | E3 with feedforward, exact model; with `b` increased 20 % | final `1.0 ± 1e-6`; non-zero steady-state error (record) |
| V41.3 | Integral action (augmented poles `{−20, −25, −10}`), input disturbance `2 V` at `t = 2 s`, model mismatch | output returns to `1.0 ± 1e-3` within 2 s |
| V41.4 | `augmentWithIntegrator(E1)`; of a realisation of `s/(s+1)` | controllable; not controllable |

**Exit criteria.**
- [ ] V41.1–V41.4
- [ ] Commit

**References.** [R9, §7.8–7.9]; [R8, §7.4].

---

### Day 42 — Copy and Move Semantics: Library Audit

**Objectives.** C++: the six special member functions, the Rule of Zero and Rule of Five, `= default`/`= delete`, `std::move` as a cast, return-value optimisation, `noexcept` move operations, `std::chrono` benchmarking.

**Background.** Every class may have a default constructor, destructor, copy constructor, copy assignment, move constructor and move assignment. Classes whose members manage themselves (Eigen types, `std::vector`, `std::string`, `std::unique_ptr`) require none to be declared (Rule of Zero); every class in the library is of this kind. A class declaring any of destructor, copy or move must consider all five (Rule of Five). Moving transfers a resource from an expiring object in `O(1)`; `std::move` only marks an lvalue as movable; local objects are returned without `std::move` so that elision applies. `Simulator` is move-only because of its `std::unique_ptr` members.

**Implementation.**

| File | Action |
|---|---|
| `tests/core/test_SpecialMembers.cpp` | `static_assert(std::is_nothrow_move_constructible_v<T>)` for `Polynomial`, `StateSpace`, `TransferFunction`, `SimResult`, `FrequencyResponse`, `PID`; `static_assert(!std::is_copy_constructible_v<Simulator> && std::is_move_constructible_v<Simulator>)`; a timed copy versus move of a `500×500` `StateSpace`; an address check that returning a `SimResult` by value does not copy its buffer |
| `Simulator.hpp`, abstract bases | Add explicit `= delete` (copy of `Simulator`) and `= default` (virtual destructors) annotations |
| `docs/notes/object_model.md` | One-page summary written by the implementer |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V42.1 | All static assertions compile | — |
| V42.2 | Move versus copy time, `500×500` | move at least 100× faster |
| V42.3 | `r.t.data()` before and after return by value | identical address |

**Exit criteria.**
- [ ] V42.1–V42.3; the note written
- [ ] Commit

**References.** [R1, ch. 13]; [R2, ch. 22]; [R7, items 17, 23–25]; [R49].

---

### Day 43 — Phase 6 Review

**Implementation.** `examples/ex05_state_space_design.cpp`: E3 with speed measurement only: (1) controllability/observability report and Gramian eigenvalues; (2) pole placement for a 0.2 s settling time with feedforward, Simulator with `C = I`; (3) current-estimator observer at `T_s = 1 ms`, five times faster; true versus estimated current plotted; (4) integral action with a load disturbance at `t = 1 s`; (5) Bode and margins of the observer-based compensator loop; (6) `h2norm` from disturbance to speed for designs (2) and (4). API friction discovered while writing the example is corrected in the library with tests.

`sandbox/phase6_review.md`: (1) the `(x, e)` closed-loop dynamics and their poles; (2) why `W_c` is numerically worse than the Gramian for large `n`; (3) the special member functions `Simulator` possesses, and why; (4) Lyapunov's theorem stated precisely, with the role of `Q`; (5) when feedforward is preferable to integral action and vice versa.

**Exit criteria.**
- [ ] `ex05` runs; review written
- [ ] Commit `Day 43: Phase 6 complete`; tag `phase-6`

---
## 13. Phase 7 — Optimal Control (Days 44–49)

**Phase objective.** Implement the continuous and discrete Riccati solvers and LQR, characterise LQR's properties, introduce nonlinear systems with numerical linearization (the cart–pole), and implement grid-based dynamic programming.

### Day 44 — LQR and the Continuous Algebraic Riccati Equation

**Objectives.** C++: `2n×2n` block assembly, `Eigen::EigenSolver` with eigenvectors, column selection by predicate into complex matrices, transposed complex solves, residual self-checks. Control: the LQR problem, the completing-the-square argument, the continuous algebraic Riccati equation, the Hamiltonian solution method.

**Background.** The LQR problem minimises `J = ∫₀^∞ (xᵀQx + uᵀRu) dt`, `Q ⪰ 0`, `R ≻ 0`, subject to `ẋ = Ax + Bu`. The solution is `u = −Kx` with

```
K = R⁻¹ Bᵀ P,    AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0   (CARE)                                               (13-1)
```

Derivation: with `V = xᵀPx` and `P` satisfying Eq. (13-1), `d/dt(xᵀPx) + xᵀQx + uᵀRu = (u + Kx)ᵀR(u + Kx) ≥ 0`, so `J ≥ x₀ᵀPx₀` with equality for `u = −Kx`; the optimal cost is `x₀ᵀPx₀`. A unique stabilising `P ⪰ 0` exists when `(A, B)` is stabilizable and `(A, Q^{1/2})` detectable (Day 37). With `S = BR⁻¹Bᵀ`, the Hamiltonian matrix

```
H = [[A, −S], [−Q, −Aᵀ]]                                                                              (13-2)
```

has eigenvalues symmetric about the imaginary axis; stacking the `n` eigenvectors with `Re λ < 0` as `[X₁; X₂]` gives `P = X₂X₁⁻¹`, and those eigenvalues are the closed-loop poles. The eigenvector method is adequate for `n ≤ 30`; the ordered-Schur method is listed in Table 19-1.

**Implementation.**

| File | Action |
|---|---|
| `LinearAlgebra.hpp/.cpp` | Add `Matrix solveARE(const Matrix& A, const Matrix& S, const Matrix& Q)` (Listing 13-1) and `solveCARE(A, B, Q, R)` forming `S` |
| `include/csl/control/LQR.hpp`, `src/control/LQR.cpp` | Create: `struct LQRResult { Matrix K, P; std::vector<Complex> closedLoopPoles; }`; `lqr(A, B, Q, R)`; `lqr(const StateSpace&, Q, R)` (continuous only); `Real lqrCost(const Matrix& P, const Vector& x0)` |
| tests | Extend `test_LinearAlgebra.cpp`; create `tests/control/test_LQR.cpp` |

**Listing 13-1 — ARE solver (Hamiltonian eigenvector method)**

```cpp
Matrix solveARE(const Matrix& A, const Matrix& S, const Matrix& Q) {     // A^T X + X A - X S X + Q = 0
    const Index n = A.rows();
    Matrix H(2 * n, 2 * n);
    H << A, -S, -Q, -A.transpose();                                       // Eq. (13-2)
    Eigen::EigenSolver<Matrix> es(H, true);
    const ComplexVector ev = es.eigenvalues();
    const ComplexMatrix V  = es.eigenvectors();
    ComplexMatrix X1(n, n), X2(n, n);
    Index col = 0;
    for (Index i = 0; i < 2 * n; ++i) {
        if (ev(i).real() < 0.0) {
            if (col == n) throw NumericalError("solveARE: more than n stable eigenvalues");
            X1.col(col) = V.block(0, i, n, 1);
            X2.col(col) = V.block(n, i, n, 1);
            ++col;
        }
    }
    if (col != n) throw NumericalError("solveARE: " + std::to_string(col) + " stable eigenvalues; check stabilizability/detectability");
    const ComplexMatrix Pc = X1.transpose().partialPivLu().solve(X2.transpose()).transpose();   // X2 X1^{-1}
    Matrix P = Pc.real();
    P = 0.5 * (P + P.transpose());
    const Real res = (A.transpose() * P + P * A - P * S * P + Q).norm();
    if (res > 1e-6 * (1.0 + Q.norm() + P.norm())) throw NumericalError("solveARE: residual " + std::to_string(res));
    return P;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V44.1 | E4, `Q = I`, `R = 1`: `P`; `K`; closed-loop poles; residual | Table 5-3 values to `1e-9`; residual `< 1e-12` |
| V44.2 | E3, `Q = diag(1, 100)`, `R = 1`: stability; simulated cost from `x₀ = [0, 1]` (Simulator with `C = I`, `T_s = 1e-4`, trapezoid) versus `x₀ᵀPx₀` | stable; within `0.5 %` |
| V44.3 | Stable eigenvalues of `H` versus `eig(A − BK)` | equal to `1e-8` |
| V44.4 | Non-detectable case (`Q = 0`, unstable `A`) | throws `NumericalError` |

**Exit criteria.**
- [ ] V44.1–V44.4; the completing-the-square derivation reproduced on paper
- [ ] Commit

**References.** [R8, §6.5]; [R9, §7.10]; [R13, ch. 2–3]; [R32].

---

### Day 45 — Discrete LQR: Riccati Recursion and the DARE

**Objectives.** C++: backward loops over `std::vector<Matrix>`, convergence loops with relative tolerances and diagnostics. Control: the finite-horizon LQ problem as dynamic programming, the Riccati recursion, the discrete algebraic Riccati equation.

**Background.** For `J = Σ_{k=0}^{N−1}(x_kᵀQx_k + u_kᵀRu_k) + x_NᵀQ_Nx_N`, the cost-to-go is quadratic, `V_k(x) = xᵀP_kx`, with `P_N = Q_N` and, backwards,

```
K_k = (R + BᵀP_{k+1}B)⁻¹ BᵀP_{k+1}A,    P_k = Q + AᵀP_{k+1}(A − BK_k)                                  (13-3)
```

`u_k = −K_kx_k` is optimal and time-varying near the horizon end. Under stabilizability and detectability the recursion converges to the solution of the DARE

```
P = Q + AᵀPA − AᵀPB(R + BᵀPB)⁻¹BᵀPA,    K = (R + BᵀPB)⁻¹BᵀPA                                          (13-4)
```

`solveDARE` iterates Eq. (13-3) to convergence (linear rate; typically under 500 iterations for the systems of this plan). Discrete LQR of a ZOH-discretized plant with `Q_d = QT_s`, `R_d = RT_s` approaches continuous LQR as `T_s → 0`.

**Implementation.**

| File | Action |
|---|---|
| `LinearAlgebra.hpp/.cpp` | Add `solveDARE(A, B, Q, R, tol = 1e-10, maxIter = 10000)` (Listing 13-2) |
| `LQR.hpp/.cpp` | Add `struct TimeVaryingGains { std::vector<Matrix> K, P; }`; `finiteHorizonLQR(A, B, Q, R, QN, N)`; `dlqr(A, B, Q, R)`; `dlqr(const StateSpace& sysd, Q, R)` (requires `T_s > 0`) |
| tests | Extend |

**Listing 13-2 — DARE by Riccati iteration**

```cpp
Matrix solveDARE(const Matrix& A, const Matrix& B, const Matrix& Q, const Matrix& R, Real tol, int maxIter) {
    Matrix P = Q;
    for (int it = 0; it < maxIter; ++it) {
        const Matrix BtP = B.transpose() * P;
        const Matrix K  = (R + BtP * B).ldlt().solve(BtP * A);
        const Matrix Pn = Q + A.transpose() * P * (A - B * K);           // Eq. (13-3)
        const Real change = (Pn - P).norm() / (1.0 + Pn.norm());
        P = 0.5 * (Pn + Pn.transpose());
        if (change < tol) return P;
    }
    throw NumericalError("solveDARE: no convergence");
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V45.1 | Discrete E4 (`T_s = 0.1`: `A = [[1, 0.1], [0, 1]]`, `B = [0.005; 0.1]`), `Q = I`, `R = 1`: DARE residual; closed-loop poles | `< 1e-9`; inside the unit circle |
| V45.2 | `finiteHorizonLQR(…, N = 300).K[0]` versus `dlqr(…).K`; `K[N−1]` | equal to `1e-6`; different |
| V45.3 | `dlqr(c2d(E3, T_s), QT_s, RT_s).K` versus `lqr(E3, Q, R).K`, `T_s = 0.001` | within `2 %` |
| V45.4 | `A = 2`, `B = 0` | throws |
| V45.5 | Simulator with the `dlqr` gain at `T_s = 0.1` from `x₀ = [1, 0]`: summed stage cost versus `x₀ᵀPx₀` | within `1 %` |

**Exit criteria.**
- [ ] V45.1–V45.5; the recursion derived from the Bellman equation on paper
- [ ] Commit

**References.** [R14, §4.1]; [R11, §11.1–11.2]; [R13, §2.4].

---

### Day 46 — LQR Properties: Bryson's Rule, Trade-offs, Guaranteed Margins, Integral LQR

**Objectives.** C++: parameter sweeps and tabulated results, bound assertions on frequency grids. Control: weight selection, the trade-off between speed and effort, the return-difference inequality and guaranteed margins, output weighting, integral LQR.

**Background.** Bryson's rule sets `Q_ii = 1/x_{i,max}²`, `R_jj = 1/u_{j,max}²`, after which a scalar `ρ` in `R = ρR₀` trades speed against effort; closed-loop poles follow the symmetric root locus toward a Butterworth pattern. For SISO LQR with the loop broken at the plant input, `L(s) = K(sI − A)⁻¹B` satisfies the return-difference inequality

```
|1 + L(jω)| ≥ 1   for all ω   ⇒   GM ∈ [1/2, ∞),   PM ≥ 60°                                              (13-5)
```

These guarantees assume full-state feedback (Day 52 shows their loss under estimation). Output weighting uses `Q = CᵀQ_yC`. Integral LQR applies `lqr` to the augmented system of Eq. (12-7).

**Implementation.**

| File | Action |
|---|---|
| `LQR.hpp/.cpp` | Add `brysonWeights(const Vector& maxValues)`; `lqrLoopTransfer(const StateSpace&, const Matrix& K)` (SISO); `struct IntegralLQRResult { Matrix K, Ki, P; }`; `lqrIntegral(sys, Q, Qi, R)` |
| `FrequencyResponse.hpp/.cpp` | Add `returnDifferenceMin(const TransferFunction& L, const Vector& w)` |
| `examples/ex06_lqr_pendulum.cpp` | Begin: trade-off table |
| tests | Extend `test_LQR.cpp` |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V46.1 | E3, Bryson weights (`x_max = [2, 1]`, `u_max = 12`), `ρ ∈ {0.01, 0.1, 1, 10, 100}` | settling time increases and peak `|u|` decreases monotonically with `ρ` |
| V46.2 | For every `ρ`: `margins(lqrLoopTransfer)`; `returnDifferenceMin` on a 1000-point grid | `PM ≥ 60° − 1e-6`; GM absent or `≥ 6 dB`; `≥ 1 − 1e-9` |
| V46.3 | E4 with `Q = diag(1, 0)`, `R = ρ` | closed-loop poles at `135°` for all `ρ` |
| V46.4 | `lqrIntegral` on E3 with a step input disturbance | output returns to the reference within `1e-3` |

**Exit criteria.**
- [ ] V46.1–V46.4; the margin guarantee stated with the location of the loop break
- [ ] Commit

**References.** [R8, §6.5–6.6]; [R9, §7.10]; [R13, ch. 5]; [R44].

---

### Day 47 — Nonlinear Systems, Numerical Jacobians and Linearization

**Objectives.** C++: a `NonlinearSystem` class over `std::function`, central-difference Jacobians, Newton's method, a Simulator refactor accepting both plant kinds. Control: equilibria, linearization, Lyapunov's indirect method, the cart–pole model and its region of attraction.

**Background.** For `ẋ = f(x, u)`, `y = h(x, u)`, an equilibrium satisfies `f(x_e, u_e) = 0`. For deviations `x̃ = x − x_e`, `ũ = u − u_e`,

```
x̃̇ ≈ A x̃ + B ũ,   A = ∂f/∂x|_{(x_e,u_e)},   B = ∂f/∂u|_{(x_e,u_e)},   C = ∂h/∂x,   D = ∂h/∂u                 (13-6)
```

If the linearization is strictly stable (unstable), the equilibrium of the nonlinear system is locally stable (unstable). The cart–pole E5 (Eq. 5-5) linearised at the upright equilibrium has the matrices of Eq. (5-6) and poles `{0, 0, ±4.646}`. Central differences with step `h = ∛ε · max(1, |x_i|)` give Jacobians accurate to about `1e-8`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/systems/NonlinearSystem.hpp`, `src/systems/NonlinearSystem.cpp` | Create (Listing 13-3) |
| `NumericalMethods.hpp/.cpp` | Add `Matrix numericalJacobian(const std::function<Vector(const Vector&)>&, const Vector& x)`; `Vector newtonSolve(f, x0, tol)` |
| `Simulator.hpp/.cpp` | Store the plant as `{Dynamics f, Output h, n, m, p}`; add a constructor taking `NonlinearSystem`; `StateSpace` converts via `NonlinearSystem::fromLinear`; Day 24 tests unchanged |
| `examples/plants.hpp` | Add `cartPole()` implementing Eq. (5-5) with `h(x) = [p, θ]` |
| tests | `tests/systems/test_NonlinearSystem.cpp`; extend `test_NumericalMethods.cpp` |

**Listing 13-3 — `NonlinearSystem`**

```cpp
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
    StateSpace linearize(const Vector& xe, const Vector& ue, Real t = 0.0) const;   // Eq. (13-6), numerical
    Vector findEquilibrium(const Vector& xGuess, const Vector& ue, Real tol = 1e-10) const;
    static NonlinearSystem fromLinear(const StateSpace& sys);
private:
    Dynamics f_; Output h_; Index n_, m_, p_;
};
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V47.1 | `numericalJacobian` of `f(x) = [x₁², x₁x₂]` at `[2, 3]` | `[[4, 0], [3, 2]]` to `1e-7` |
| V47.2 | `cartPole().linearize(0, 0)` versus Eq. (5-6); poles | equal to `1e-6`; `{0, 0, ±4.646}` |
| V47.3 | `findEquilibrium` from `θ = 0.3`, `u = 0` | `θ = 0` |
| V47.4 | LQR (`Q = diag(1, 1, 10, 1)`, `R = 0.1`) on the linearization applied to the nonlinear plant, `T_s = 0.01`, true-state feedback: from `θ₀ = 0.2`; from `θ₀ = 1.0` | `‖x(10)‖ < 1e-3`; failure (`|θ| > π/2`); boundary recorded |
| V47.5 | Simulator with `NonlinearSystem::fromLinear(E1)` versus Day 24 results | equal to `1e-12` |

**Exit criteria.**
- [ ] V47.1–V47.5; the entries `−mg/M` and `(M+m)g/(Ml)` of Eq. (5-6) derived by hand
- [ ] Commit

**References.** [R16, §2.1–2.3, §4.3]; [R8, §5.1–5.3]; [R45, "Inverted Pendulum: System Modeling"].

---

### Day 48 — Dynamic Programming on a Grid

**Objectives.** C++: flattened multi-dimensional grids, interpolation, arg-min loops, timing of brute-force sweeps. Control: the Bellman equation, value iteration, the relationship between DP, LQR and MPC, the bang-bang double integrator.

**Background.** For stage cost `l(x, u)` and dynamics `x⁺ = f(x, u)`, the optimal cost-to-go satisfies

```
V(x) = min_u [ l(x, u) + V(f(x, u)) ]                                                                 (13-7)
```

LQR is the special case with quadratic `V` (Eq. 13-3 is Eq. 13-7 specialised). With input constraints `V` is not quadratic; grid-based value iteration computes it numerically at a cost exponential in the state dimension, which motivates MPC (Day 63). For the double integrator with `|u| ≤ 1` and time-to-origin cost, the optimal policy is bang-bang with switching curve `x₁ = −½x₂|x₂|`.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/DynamicProgramming.hpp`, `src/control/DynamicProgramming.cpp` | Create: `struct GridDP { std::vector<Vector> axes; Vector actions; std::function<Vector(const Vector&, Real)> step; std::function<Real(const Vector&, Real)> stageCost; Real discount = 1.0; }`; `struct DPResult { std::vector<Real> V, policy; int iterations; Real lastChange; }`; `valueIteration(const GridDP&, Real tol = 1e-6, int maxIter = 2000)` with bilinear interpolation of `V(f(x, u))`; `interpolate(...)` |
| `tests/control/test_DynamicProgramming.cpp` | Create |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V48.1 | `x⁺ = 0.9x + u`, cost `x² + u²`, grid `[−3, 3]` (601 points), 201 actions: `V(x)` versus `x²P`, `P = 1.4839` (positive root of `P² − 0.81P − 1 = 0`) | within `3 %` for `|x| ≤ 2` |
| V48.2 | Discrete E4 (`T_s = 0.1`), `|u| ≤ 1`, time-to-origin: policy sign at `(1, 0)`, `(−1, 0)`, `(0, 1)`, `(−0.6, 1)` | `−1`, `+1`, `−1`, `+1`; `lastChange < tol` |
| V48.3 | Sweep time recorded | — |

**Exit criteria.**
- [ ] V48.1–V48.3
- [ ] Commit

**References.** [R14, §1.1–1.3, ch. 2]; [R15, §3.1–3.4].

---

### Day 49 — Phase 7 Review

**Implementation.** `examples/ex06_lqr_pendulum.cpp`: cart–pole: (1) linearization, poles, controllability; (2) Bryson weights (`p_max = 1`, `θ_max = 0.2`, `F_max = 10`); (3) `lqr` and `dlqr` at `T_s = 0.01`; (4) nonlinear closed loop from `θ₀ = 0.2` with plots of `p`, `θ`, `F`; (5) `ρ` sweep table; (6) LQR loop margins at the plant input; (7) region-of-attraction scan over `θ₀`; (8) finite-horizon gains for `N = 100` plotted against `k`.

`sandbox/phase7_review.md`: (1) `K = R⁻¹BᵀP` derived from the CARE; (2) why the stable Hamiltonian eigenvalues are the closed-loop poles; (3) `P_k` versus the DARE solution and the use of each; (4) what the LQR margin guarantee does not cover; (5) why DP solves the constrained double integrator exactly while LQR cannot, and the cost of DP for the 4-state cart–pole on a 50-point grid.

**Exit criteria.**
- [ ] `ex06` runs; review written
- [ ] Commit `Day 49: Phase 7 complete`; tag `phase-7`

---
## 14. Phase 8 — State Estimation (Days 50–55)

**Phase objective.** Implement noise generation and Monte Carlo evaluation, the Kalman filter and its steady state, LQG, and the extended and unscented Kalman filters, all through the `Estimator` interface of Day 39.

### Day 50 — Randomness, Noise, PRBS and Monte Carlo

**Objectives.** C++: `<random>` engines and distributions, seeding policy, random generators as class members, linear-feedback shift registers, sample statistics over Eigen data, factory lambdas for move-only objects. Control: white noise, Gaussian noise with a given covariance, process and measurement noise, noise discretization, pseudo-random binary sequences, Monte Carlo evaluation.

**Background.** `std::mt19937` is a deterministic engine seeded with an integer; distributions shape its output. Tests use fixed seeds; experiments may use `std::random_device`. A Gaussian vector with covariance `Σ` is generated as `v = Lz` with `LLᵀ = Σ` (Cholesky) and `z` standard normal. The stochastic model used by the Kalman filter is

```
x_{k+1} = A x_k + B u_k + w_k,   y_k = C x_k + v_k,   w ~ N(0, Q),   v ~ N(0, R)                           (14-1)
```

Continuous white noise of intensity `Q_c` sampled with a zero-order hold has `Q_d = ∫₀^{T_s} e^{Aτ}Q_c e^{Aᵀτ}dτ`, computable by the augmented-exponential method of Day 29. A maximal-length PRBS of `n` bits has period `2ⁿ − 1`, near-impulsive autocorrelation and a flat spectrum up to its clock rate; the 10-bit register uses taps `x¹⁰ + x⁷ + 1`. Monte Carlo evaluation runs the closed loop with independent noise realisations and reports mean, standard deviation and worst case.

**Implementation.**

| File | Action |
|---|---|
| `signals/Signals.hpp/.cpp` | Add Listing 14-1 |
| `LinearAlgebra.hpp/.cpp` | Add `sampleCovariance(const Matrix& rows)`, `discretizeNoise(A, Qc, Ts)` |
| `Simulator.hpp/.cpp` | Implement `setMeasurementNoise`; add `setProcessNoise(const Matrix& Q, unsigned seed)` |
| `include/csl/simulation/MonteCarlo.hpp`, `src/simulation/MonteCarlo.cpp` | Create: `struct MonteCarloResult { std::vector<SimResult> runs; Vector t; Matrix yMean, yStd, uMean, uStd; }`; `monteCarlo(const std::function<Simulator(unsigned seed)>& makeSimulator, const Vector& x0, Real tf, int runs, unsigned baseSeed = 1000)` |
| tests | Extend `test_Signals.cpp`; create `tests/simulation/test_MonteCarlo.cpp` |

**Listing 14-1 — Noise and PRBS generators**

```cpp
namespace csl::signals {
class WhiteNoise {
public:
    explicit WhiteNoise(Real stdDev = 1.0, unsigned seed = 1);
    Real   sample() { return dist_(rng_); }
    Vector sample(Index n);
    void   reseed(unsigned seed) { rng_.seed(seed); dist_.reset(); }
private:
    std::mt19937 rng_;
    std::normal_distribution<Real> dist_;
};
class MultivariateNormal {                                   // v = L z, L L^T = Sigma
public:
    MultivariateNormal(const Matrix& Sigma, unsigned seed = 1);
    Vector sample();
private:
    Matrix L_; std::mt19937 rng_; std::normal_distribution<Real> dist_;
};
class PRBS {                                                 // maximal-length LFSR, values ±amplitude
public:
    explicit PRBS(int bits = 10, Real amplitude = 1.0, unsigned seed = 1);
    Real   next();
    Vector sample(Index n);
    Index  period() const { return (Index(1) << bits_) - 1; }
private:
    int bits_; Real amp_; std::uint32_t state_;
};
}
Real PRBS::next() {                                          // taps for 10 bits: positions 10 and 7
    const std::uint32_t bit = ((state_ >> 0) ^ (state_ >> 3)) & 1u;
    state_ = (state_ >> 1) | (bit << (bits_ - 1));
    return (state_ & 1u) ? amp_ : -amp_;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V50.1 | `WhiteNoise(2.0, 42)`, `10⁵` samples: mean; variance; `welch` level | `|mean| < 0.02`; `4 ± 2 %`; flat within `±15 %` |
| V50.2 | `PRBS(10)`: period; count of `+1` per period; normalised autocorrelation at lags 1–50 | `1023`; `512`; `|·| ≤ 0.01` |
| V50.3 | `MultivariateNormal([[4, 1], [1, 2]])`, `10⁵` samples: `sampleCovariance` | within `3 %` element-wise |
| V50.4 | `discretizeNoise(−1, 2, 0.1)` | `1 − e^{−0.2} = 0.18127` |
| V50.5 | Monte Carlo (50 runs) of E1 with PI and `σ_v = 0.05`: `yStd` at `t = 5` versus `σ_v · h2norm` of the noise-to-output closed loop | within `30 %` |

**Exit criteria.**
- [ ] V50.1–V50.5
- [ ] Commit

**References.** [R1, §17.4]; [R17, ch. 2–3]; [R19, §13.3].

---

### Day 51 — The Kalman Filter

**Objectives.** C++: a concrete `Estimator` with diagnostic accessors, the Joseph-form update, statistical tests with fixed seeds. Control: the Kalman predict/update recursion, the gain as a trust ratio, innovation and NIS consistency checks, covariance convergence.

**Background.** For the model of Eq. (14-1):

```
Predict:  x̂⁻ = A x̂ + B u,                    P⁻ = A P Aᵀ + Q
Update:   S = C P⁻ Cᵀ + R,                     K = P⁻ Cᵀ S⁻¹
          x̂ = x̂⁻ + K (y − C x̂⁻),              P = (I − KC) P⁻ (I − KC)ᵀ + K R Kᵀ                        (14-2)
```

The gain minimises `tr P` (equivalently, gives the conditional mean under Gaussian assumptions); `R → 0` trusts the measurement, `Q → 0` trusts the model. The structure is that of the current-estimator observer (Eq. 12-3) with `L = K` chosen from noise statistics. The innovation `ν = y − Cx̂⁻` should be white with covariance `S`; the normalised innovation squared `νᵀS⁻¹ν` has mean `p` when the filter is consistent. With `(A, C)` detectable and `(A, Q^{1/2})` stabilizable, `P⁻` converges to the solution of a DARE (Day 52).

**Implementation.**

| File | Action |
|---|---|
| `include/csl/estimation/KalmanFilter.hpp`, `src/estimation/KalmanFilter.cpp` | Create (Listing 14-2) |
| `tests/estimation/test_KalmanFilter.cpp` | Create |

**Listing 14-2 — `KalmanFilter`**

```cpp
class KalmanFilter : public Estimator {
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

void KalmanFilter::update(const Vector& y) {                              // Eq. (14-2)
    const Matrix& C = sys_.C();
    const Index n = x_.size();
    S_  = C * P_ * C.transpose() + R_;
    K_  = S_.ldlt().solve(C * P_).transpose();                            // S K^T = C P
    nu_ = y - C * x_;
    x_ += K_ * nu_;
    const Matrix IKC = Matrix::Identity(n, n) - K_ * C;
    P_  = IKC * P_ * IKC.transpose() + K_ * R_ * K_.transpose();          // Joseph form
    symmetrize();
}
```

**Verification.** Constant-velocity target: `A = [[1, T], [0, 1]]`, `T = 0.1`, `C = [1, 0]`, `Q = σ_a²[[T⁴/4, T³/2], [T³/2, T²]]`, `σ_a = 0.5`, `R = 1`, seed 7, 500 steps.

| ID | Check | Expected |
|---|---|---|
| V51.1 | Position RMSE of the estimate; velocity RMSE | `< 0.45` (measurement RMSE `1.0`); `< 0.5` |
| V51.2 | Mean NIS | in `[0.8, 1.25]` |
| V51.3 | `P⁻` convergence; value versus `solveDARE(Aᵀ, Cᵀ, Q, R)` | `‖ΔP‖ < 1e-9`; equal to `1e-8` |
| V51.4 | `R` reduced 100× (overconfident) | mean NIS `> 3` |
| V51.5 | E3 with noisy speed (`σ_v = 0.05`): current RMSE versus open-loop model prediction | smaller |
| V51.6 | `C = 0` | `tr P` grows without bound |

**Exit criteria.**
- [ ] V51.1–V51.6
- [ ] Commit

**References.** [R17, ch. 5]; [R8, §7.4]; [R40]; [R46, ch. 5 (NIS)].

---

### Day 52 — Steady-State Kalman Filter, Duality and LQG

**Objectives.** C++: a `Controller` composed of an `Estimator` and a gain; a compensator export for analysis. Control: the steady-state Kalman gain through the DARE, the LQR–Kalman duality, the separation theorem for LQG, the loss of guaranteed margins.

**Background.** As `k → ∞`, `P⁻` converges to the solution of

```
P = A P Aᵀ − A P Cᵀ (C P Cᵀ + R)⁻¹ C P Aᵀ + Q                                                         (14-3)
```

which is Eq. (13-4) with `(A, B, Q, R) ↦ (Aᵀ, Cᵀ, Q, R)`. The duality table maps control `(A, B, Q, R, K)` to estimation `(Aᵀ, Cᵀ, Q_w, R_v, Lᵀ)`. LQG minimises `E[xᵀQx + uᵀRu]` under Gaussian noise; by the separation theorem the optimal controller is the LQR gain applied to the Kalman estimate, and the closed-loop poles are `eig(A − BK) ∪ eig((I − LC)A)`. Unlike LQR, LQG has no guaranteed margins [R33]; the loop broken at the plant input must be checked with `margins` in every design.

**Implementation.**

| File | Action |
|---|---|
| `KalmanFilter.hpp/.cpp` | Add `struct SteadyStateKF { Matrix Pminus, Pplus, K; std::vector<Complex> poles; }`; `steadyStateKalman(const StateSpace& sysd, Q, R)` |
| `include/csl/control/LQG.hpp`, `src/control/LQG.cpp` | Create: `class LQGController final : public Controller` with `Config { Matrix Qx, Ru, Qw, Rv; Vector x0; Matrix P0; bool steadyState = true; Matrix Nbar; }`; `compute` performs `update(y)`, `u = −Kx̂ + N̄r`, `predict(u)`; `StateSpace lqgCompensator(const StateSpace& sysd, const Matrix& K, const Matrix& L)` |
| tests | Extend `test_KalmanFilter.cpp`; create `tests/control/test_LQG.cpp` |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V52.1 | `steadyStateKalman` on the Day 51 target: `K` versus the converged gain; `Pminus` versus `solveDARE(Aᵀ, Cᵀ, Q, R)` | equal to `1e-8` |
| V52.2 | E3 LQG (`T_s = 1 ms`, `σ_v = 0.05`): closed-loop poles of `feedback(plant_d, lqgCompensator)` | `eig(A − BK) ∪ eig((I − LC)A)` to `1e-8` |
| V52.3 | Monte Carlo (30 runs): speed RMSE after `t = 0.5 s` with the Kalman gain versus a pole-placed observer at the same poles | lower (`±5 %` tolerance) |
| V52.4 | LQG loop margins with a deliberately fast filter (`R_v` small) | PM below `30°`, versus `≥ 60°` for the LQR loop |

**Exit criteria.**
- [ ] V52.1–V52.4; the duality table reproduced from memory
- [ ] Commit

**References.** [R8, §7.5]; [R13, ch. 8]; [R33]; [R17, ch. 7].

---

### Day 53 — The Extended Kalman Filter

**Objectives.** C++: `std::optional` callbacks with numerical fallback, discretization of nonlinear dynamics by one RK4 step, angle-wrapping hooks. Control: the EKF as a Kalman filter linearised at the current estimate; its limitations and tuning.

**Background.** For `x_{k+1} = f(x_k, u_k) + w_k`, `y_k = h(x_k) + v_k`, the EKF predicts with `f` and propagates `P` with `F = ∂f/∂x|_{x̂}`; it updates with `h` and `H = ∂h/∂x|_{x̂⁻}`; the remainder is Eq. (14-2). `F` is the Jacobian of the *discrete* map (the integrator step), not of the continuous vector field. The EKF carries no optimality guarantee; it diverges under strong nonlinearity or poor initialisation, and `Q` is commonly inflated to cover linearisation error.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/estimation/ExtendedKalmanFilter.hpp`, `src/estimation/ExtendedKalmanFilter.cpp` | Create (Listing 14-3) |
| `examples/plants.hpp` | Add `pendulum(l = 1, c = 0.1)`: `θ̈ = −(g/l) sin θ − cθ̇ + u`, output `sin θ` |
| `tests/estimation/test_ExtendedKalmanFilter.cpp` | Create |

**Listing 14-3 — `ExtendedKalmanFilter` interface**

```cpp
class ExtendedKalmanFilter final : public Estimator {
public:
    using StepFunc = std::function<Vector(const Vector& x, const Vector& u)>;    // discrete f_d
    using OutFunc  = std::function<Vector(const Vector& x)>;
    using JacX     = std::function<Matrix(const Vector& x, const Vector& u)>;
    using JacH     = std::function<Matrix(const Vector& x)>;
    struct Config {
        StepFunc f; OutFunc h; Matrix Q, R; Vector x0; Matrix P0;
        std::optional<JacX> F; std::optional<JacH> H;                       // numerical Jacobians when absent
        std::function<Vector(const Vector&)> innovationFix;                 // e.g. angle wrapping
    };
    explicit ExtendedKalmanFilter(Config cfg);
    static StepFunc discretize(const systems::NonlinearSystem& sys, Real Ts);   // one RK4 step
    void predict(const Vector& u) override;                                 // x = f(x,u); P = F P F^T + Q
    void update (const Vector& y) override;                                 // H at x^-; Eq. (14-2) with h(x^-)
    /* state, covariance, reset as in KalmanFilter */
};
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V53.1 | EKF with numerical Jacobians on a linear plant versus `KalmanFilter` | equal to `1e-9` at every step |
| V53.2 | Pendulum (`T_s = 0.01`, `σ_v = 0.05` on `sin θ`, seed 3, `θ₀ = 0.5`, initial estimate 0): `θ̂` RMSE over `t ∈ [2, 20]`; analytic versus numerical Jacobians; mean NIS | `< 0.02`; equal to `1e-6`; in `[0.7, 1.4]` |
| V53.3 | Cart–pole with LQR on the EKF estimate (`σ = 0.01` on `p`, `θ`; process noise on `F`), `θ₀ = 0.15` | balanced; steady-state `θ` RMSE `< 0.01` |
| V53.4 | Pendulum with initial estimate `1.5` for true `0.5` | convergence time recorded (compared on Day 54) |

**Exit criteria.**
- [ ] V53.1–V53.4
- [ ] Commit

**References.** [R17, §13.1–13.2]; [R18, §3.3].

---

### Day 54 — The Unscented Kalman Filter

**Objectives.** C++: `Eigen::LLT` matrix square roots, sigma-point matrices, weighted statistics, guarded factorisation. Control: the unscented transform, sigma points and weights, the UKF, its accuracy relative to the EKF.

**Background.** The unscented transform propagates `2n + 1` *sigma points* through a nonlinear function and recombines them:

```
χ₀ = x̂,   χ_i = x̂ ± (√((n + λ)P))_i,   λ = α²(n + κ) − n
W_m⁰ = λ/(n + λ),   W_c⁰ = W_m⁰ + (1 − α² + β),   W_m^i = W_c^i = 1/(2(n + λ))                           (14-4)
```

(`α ∈ [10⁻³, 1]`, `β = 2`, `κ = 0`). Mean and covariance are matched to second order (third for Gaussian inputs), without Jacobians, at the cost of `2n + 1` function evaluations. UKF prediction and update apply the transform to `f` and `h`, using the cross-covariance `P_xy` for the gain `K = P_xy P_yy⁻¹`. The UKF outperforms the EKF under strong nonlinearity or large uncertainty; under mild nonlinearity the two are comparable.

**Implementation.**

| File | Action |
|---|---|
| `LinearAlgebra.hpp/.cpp` | Add `struct UnscentedTransformResult { Vector mean; Matrix cov, crossCov, sigmaOut; }`; `unscentedTransform(g, mean, cov, alpha = 1e-3, beta = 2, kappa = 0)` |
| `include/csl/estimation/UnscentedKalmanFilter.hpp`, `src/estimation/UnscentedKalmanFilter.cpp` | Create with a `Config` sharing the `f`, `h`, `Q`, `R`, `x0`, `P0`, `innovationFix` fields of Listing 14-3 plus `alpha`, `beta`, `kappa` |
| `tests/estimation/test_UnscentedKalmanFilter.cpp` | Create |

If the Cholesky factorisation of `P` fails, `εI` is added and the condition is reported through `NumericalError` after a bounded number of attempts.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V54.1 | `unscentedTransform` of `g(x) = Mx + c` | mean `Mμ + c`, covariance `MΣMᵀ` to `1e-12` |
| V54.2 | `g(x) = x²`, `x ~ N(0, 1)`, `β = 2`: mean; variance | `1.0`; `2.0` (to `1e-9`) |
| V54.3 | Pendulum test of V53.2 with the UKF | RMSE within `±10 %` of the EKF's |
| V54.4 | V53.4 case with the UKF | convergence at least as fast as the EKF |
| V54.5 | Range–bearing tracking (measurements `[√(x²+y²), atan2(y, x)]`, `σ_bearing = 0.1`, initial error 5 m, fixed seed): position RMSE over the first 50 steps | UKF at least `20 %` lower than EKF |

**Exit criteria.**
- [ ] V54.1–V54.5
- [ ] Commit

**References.** [R35]; [R17, ch. 14]; [R18, §3.4].

---

### Day 55 — Phase 8 Review

**Implementation.** `examples/ex07_estimation.cpp`: (1) constant-velocity target: measurements versus estimate, NIS with its χ² band; (2) E3 steady-state Kalman filter versus pole-placed observer under noise, Monte Carlo RMSE table; (3) LQG on E3 with margins for two filter speeds; (4) cart–pole with LQR on EKF and UKF estimates, Monte Carlo of 20 runs, true versus estimated `θ`; (5) the range–bearing comparison.

`sandbox/phase8_review.md`: (1) Eq. (14-2) from memory with the role of each term; (2) the LQR–Kalman duality as a table and in one sentence; (3) why LQG can lose LQR's margins and in which signal path; (4) what the EKF and UKF each approximate and a case for each; (5) the action indicated by a mean NIS of 4 for `p = 1`.

**Exit criteria.**
- [ ] `ex07` runs; review written
- [ ] Commit `Day 55: Phase 8 complete`; tag `phase-8`

---
## 15. Phase 9 — System Identification (Days 56–59)

**Phase objective.** Build models from data: ARX least squares, recursive least squares with forgetting, the empirical transfer-function estimate with coherence and a rational fit, and the eigensystem realisation algorithm; validate models by residual analysis.

### Day 56 — ARX Identification by Least Squares

**Objectives.** C++: regression-matrix construction, least squares through `ColPivHouseholderQR`, condition numbers from the SVD, train/validation splitting. Control: the ARX model, persistency of excitation, prediction versus simulation fit, order selection, bias under coloured noise.

**Background.** The ARX model with orders `n_a`, `n_b` and delay `n_k` is

```
y[k] + a₁y[k−1] + … + a_{n_a}y[k−n_a] = b₁u[k−n_k] + … + b_{n_b}u[k−n_k−n_b+1] + e[k]                     (15-1)
```

With regressor `φ[k] = [−y[k−1], …, −y[k−n_a], u[k−n_k], …, u[k−n_k−n_b+1]]ᵀ` and `θ = [a₁…a_{n_a}, b₁…b_{n_b}]ᵀ`, stacking gives `y = Φθ + e` and the least-squares estimate `θ̂ = argmin‖y − Φθ‖²`, computed by QR factorisation (the normal equations square the condition number and are not used). The model is a discrete `TransferFunction` `B(z)/A(z)` with `n_k` delays. `Φ` must have full column rank (*persistency of excitation*): PRBS or chirp inputs are used, not steps. Fit `= 100(1 − ‖y − ŷ‖/‖y − ȳ‖)` %; the simulation fit on validation data (using `u` only) is the honest measure. ARX is unbiased only when the equation error is white; coloured noise biases the estimated poles (remedies in Table 19-1).

**Implementation.**

| File | Action |
|---|---|
| `include/csl/sysid/ARX.hpp`, `src/sysid/ARX.cpp` | Create (Listing 15-1) |
| `include/csl/sysid/Validation.hpp`, `src/sysid/Validation.cpp` | Create: `fitPercent(y, yhat)`, `splitData(u, y, fraction)` |
| `tests/sysid/test_ARX.cpp` | Create |

**Listing 15-1 — ARX**

```cpp
#pragma once
#include "csl/systems/TransferFunction.hpp"
namespace csl::sysid {
struct ARXModel {
    Vector a, b; Index na, nb, nk; Real Ts;
    Real noiseVariance, conditionNumber;
    systems::TransferFunction toTransferFunction() const;        // B(z)/A(z) with nk delays
    Vector predictOneStep(const Vector& y, const Vector& u) const;
    Vector simulate(const Vector& u) const;
};
ARXModel arx(const Vector& y, const Vector& u, Index na, Index nb, Index nk, Real Ts);
Matrix   arxRegressor(const Vector& y, const Vector& u, Index na, Index nb, Index nk);   // rows start .. N-1
}

ARXModel arx(const Vector& y, const Vector& u, Index na, Index nb, Index nk, Real Ts) {
    const Index start = std::max(na, nb + nk - 1);
    const Matrix Phi = arxRegressor(y, u, na, nb, nk);
    const Vector rhs = y.tail(y.size() - start);
    const Vector theta = Eigen::ColPivHouseholderQR<Matrix>(Phi).solve(rhs);
    Eigen::JacobiSVD<Matrix> svd(Phi);
    ARXModel m;
    m.a = theta.head(na); m.b = theta.tail(nb); m.na = na; m.nb = nb; m.nk = nk; m.Ts = Ts;
    m.noiseVariance = (rhs - Phi * theta).squaredNorm() / static_cast<Real>(rhs.size() - theta.size());
    m.conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    return m;
}
```

**Verification.** Data: `lsim(c2d(E3, 0.01), PRBS(10), 3000 samples)`.

| ID | Check | Expected |
|---|---|---|
| V56.1 | Noise-free, `n_a = n_b = 2`, `n_k = 1`: `A(z)`, `B(z)` versus the discretised E3 | equal to `1e-8`; fit 100 % |
| V56.2 | Measurement noise `σ = 0.01`: validation simulation fit; poles | `> 95 %`; within `2 %` of truth |
| V56.3 | Order sweep `n_a = 1…4` | validation fit saturates at 2 |
| V56.4 | Step-input data | `conditionNumber > 1e6` |
| V56.5 | `toTransferFunction().dcGain()` (noisy data) | within `3 %` of `0.0999` |

**Exit criteria.**
- [ ] V56.1–V56.5
- [ ] Commit

**References.** [R19, §4.2, §7.3, ch. 13].

---

### Day 57 — Recursive Least Squares

**Objectives.** C++: a stateful online estimator; equivalence testing between two implementations. Control: the RLS recursion, the forgetting factor, covariance blow-up, RLS as a Kalman filter.

**Background.** Adding one observation `(y, φ)` to the batch solution and applying the matrix-inversion lemma gives

```
K = P φ / (λ + φᵀ P φ),   θ̂ ← θ̂ + K (y − φᵀ θ̂),   P ← (P − K φᵀ P) / λ                                   (15-2)
```

with forgetting factor `λ ∈ (0, 1]`: `λ = 1` reproduces batch least squares; `λ < 1` weights recent data with effective memory `1/(1 − λ)` samples and tracks slowly varying parameters. Without excitation, `P` grows by `1/λ` per step (*covariance blow-up*). Eq. (15-2) is the Kalman filter of Eq. (14-2) for `θ_{k+1} = θ_k`, `y_k = φ_kᵀθ_k + v_k`, `R = 1`; `λ < 1` corresponds to inflating `P` in place of process noise.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/sysid/RLS.hpp`, `src/sysid/RLS.cpp` | Create: `class RLS { RLS(Index nParams, Real lambda = 1, Real P0 = 1e4); void update(Real y, const Vector& phi); const Vector& theta() const; const Matrix& covariance() const; Real lastResidual() const; void reset(Real P0); }`; `Matrix rlsARX(y, u, na, nb, nk, lambda, P0)` returning the parameter history |
| `KalmanFilter.hpp` | Allow a time-varying `C` (setter) for the equivalence test |
| `tests/sysid/test_RLS.cpp` | Create |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V57.1 | Noise-free E3 data, `λ = 1`: `θ̂` after 50 samples versus batch `arx` | within `1e-6` |
| V57.2 | Gain `K_t` doubled at `k = 2000`: `λ = 0.98`; `λ = 1` | new values within 200 samples; still `> 20 %` off after 1000 |
| V57.3 | `u = 0` after `k = 1000`, `λ = 0.98`: `tr P` over 500 steps | grows at least 100× |
| V57.4 | RLS (`λ = 1`) versus `KalmanFilter(A = I, Q = 0, R = 1, C_k = φ_kᵀ, P₀ = P0·I)` | `θ̂` equal to `1e-10` at every step |

**Exit criteria.**
- [ ] V57.1–V57.4; Eq. (15-2) derived from the matrix-inversion lemma on paper
- [ ] Commit

**References.** [R19, §11.2, §11.7]; [R27, §2.2–2.3].

---

### Day 58 — Frequency-Domain Identification

**Objectives.** C++: complex coefficient-wise operations, weighted complex least squares by real stacking, iterative reweighting. Control: the empirical transfer-function estimate, coherence, rational fitting (Levy, Sanathanan–Koerner), margins from measured data, FOPDT from a frequency response.

**Background.** The smoothed empirical transfer-function estimate and the coherence are

```
Ĥ(ω) = P_uy(ω) / P_uu(ω),   γ²(ω) = |P_uy|² / (P_uu P_yy) ∈ [0, 1]                                        (15-3)
```

using the Welch cross-spectra of Day 32; `Ĥ` is trustworthy where `γ² > 0.8–0.9`. A rational model `N(s)/D(s)` with `D` monic is fitted by minimising `Σ_k w_k |N(jω_k) − Ĥ_k D(jω_k)|²`, which is linear in the coefficients (Levy); the Sanathanan–Koerner iteration reweights with `w_k = 1/|D_prev(jω_k)|²` until convergence. The same procedure with `z_k = e^{jω_kT_s}` fits discrete models. Because `Ĥ` is a `FrequencyResponse` (C9), `margins(Ĥ)` of a measured loop requires no model.

**Implementation.**

| File | Action |
|---|---|
| `dsp/Spectral.hpp/.cpp` | Add `struct ETFEResult { analysis::FrequencyResponse H; Vector coherence; Vector f_hz; }`; `etfe(u, y, fs_hz, nperseg = 1024, overlap = 0.5, Window = Hann)` (Listing 15-2) |
| `include/csl/sysid/FrequencyDomainID.hpp`, `src/sysid/FrequencyDomainID.cpp` | Create: `struct RationalFitOptions { Index numOrder, denOrder; int skIterations = 10; Real minCoherence = 0; }`; `fitRational(const FrequencyResponse& H, const RationalFitOptions&, const Vector& weights = Vector())`; `fitFOPDTfromFRF(const FrequencyResponse&)` (gain from `|Ĥ(0)|`, time constant from the −3 dB point, delay from the high-frequency phase slope) |
| tests | `tests/sysid/test_FrequencyDomainID.cpp`; extend `test_Spectral.cpp` |

**Listing 15-2 — ETFE**

```cpp
ETFEResult etfe(const Vector& u, const Vector& y, Real fs, Index nperseg, Real overlap, Window w) {
    const CrossSpectrum c = csd(u, y, fs, nperseg, overlap, w);
    ETFEResult r;
    r.f_hz = c.f_hz;
    r.H.w  = TWO_PI * c.f_hz;
    r.H.Ts = 1.0 / fs;
    r.H.H  = c.Pxy.cwiseQuotient(c.Pxx.cast<Complex>());                       // Eq. (15-3)
    r.coherence = c.Pxy.cwiseAbs2().cwiseQuotient(c.Pxx.cwiseProduct(c.Pyy));
    return r;
}
```

**Verification.** Data: E2 excited by a chirp 0.05–10 Hz for 120 s at `f_s = 100 Hz`, measurement noise `σ = 0.01`.

| ID | Check | Expected |
|---|---|---|
| V58.1 | Where `γ² > 0.95`: `|Ĥ|`, `∠Ĥ` versus `frequencyResponse(E2)` | within `0.5 dB`, `3°` |
| V58.2 | `fitRational(Ĥ, {0, 2})` (continuous) | coefficients of `{1}/{1, 0.5, 4}` within `2 %`; coherence weighting no worse than unweighted |
| V58.3 | Discrete fit of `c2d(E2, 0.01)` | poles within `1 %` |
| V58.4 | Closed-loop E2 with a known PI; loop measured by injection at the plant input: `margins(etfe(...))` versus `margins(series(C, P))` | phase margin within `2°` |
| V58.5 | `fitFOPDTfromFRF` on E1 with `pade(0.5, 3)` | `K = 3 ± 0.05`, `T = 2 ± 0.1`, `L = 0.5 ± 0.05` |

**Exit criteria.**
- [ ] V58.1–V58.5
- [ ] Commit

**References.** [R19, §6.3–6.4]; [R20, ch. 1–2]; [R43].

---

### Day 59 — Subspace Identification (ERA), Validation and Phase 9 Review

**Objectives.** C++: block Hankel matrices, `Eigen::BDCSVD`, truncation, a validation report. Control: Markov parameters, the eigensystem realisation algorithm, order selection by singular-value gap, residual whiteness and independence tests.

**Background.** The Markov parameters `h₀ = D`, `h_k = CA^{k−1}B` (`k ≥ 1`) form the block Hankel matrices `H₀ = [h_{i+j−1}]`, `H₁ = [h_{i+j}]`, `i, j = 1…r`. With the truncated SVD `H₀ ≈ U_n Σ_n V_nᵀ`,

```
A = Σ_n^{−1/2} U_nᵀ H₁ V_n Σ_n^{−1/2},   B = first m columns of Σ_n^{1/2} V_nᵀ,   C = first p rows of U_n Σ_n^{1/2}     (15-4)
```

is a balanced minimal realization; the rank of `H₀` (the number of significant singular values) is the model order. Markov parameters are obtained from an ARX model's impulse response or, for white inputs, from the cross-correlation `R_uy(k) = σ_u² h_k`. Validation: simulation fit on fresh data; residual autocorrelation within `±1.96/√N` for lags ≥ 1; residual–input cross-correlation within the same band; agreement of poles across methods.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/sysid/Subspace.hpp`, `src/sysid/Subspace.cpp` | Create (Listing 15-3) |
| `Validation.hpp/.cpp` | Add `autocorrelation(x, maxLag)`, `crossCorrelation(x, y, maxLag)`, `struct ValidationReport { Real fitPercent; Vector residualAutocorr, residualInputXcorr; Real band; bool residualsWhite, residualsIndependent; }`, `validate(const LTISystem& model, u, y, maxLag = 25)` |
| `examples/ex08_sysid.cpp` | Create: ARX, ERA and rational fit on the same E3 data; comparison table of poles and dc gains; Bode overlay with the truth |
| tests | `tests/sysid/test_Subspace.cpp`, `tests/sysid/test_Validation.cpp` |

**Listing 15-3 — ERA**

```cpp
namespace csl::sysid {
std::vector<Matrix> markovParametersFromImpulse(const Matrix& h, Index p, Index m);
std::vector<Matrix> markovParametersByCorrelation(const Vector& u, const Vector& y, Index K);   // white u, SISO
struct ERAResult { systems::StateSpace sys; Vector singularValues; Index order; };
ERAResult era(const std::vector<Matrix>& h, Index r, Index order, Real Ts);                     // order <= 0: automatic
}
ERAResult era(const std::vector<Matrix>& h, Index r, Index order, Real Ts) {
    const Index p = h[1].rows(), m = h[1].cols();
    if (static_cast<Index>(h.size()) < 2 * r + 1) throw InvalidArgument("era: 2r+1 Markov parameters required");
    Matrix H0(r * p, r * m), H1(r * p, r * m);
    for (Index i = 0; i < r; ++i)
        for (Index j = 0; j < r; ++j) { H0.block(i * p, j * m, p, m) = h[i + j + 1]; H1.block(i * p, j * m, p, m) = h[i + j + 2]; }
    Eigen::BDCSVD<Matrix> svd(H0, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Vector s = svd.singularValues();
    Index n = order;
    if (n <= 0) { n = 1; while (n < s.size() && s(n) > s(0) * 1e-2) ++n; }
    const Vector sq = s.head(n).array().sqrt();
    const Matrix U = svd.matrixU().leftCols(n), V = svd.matrixV().leftCols(n);
    const Matrix Si = sq.cwiseInverse().asDiagonal(), Sr = sq.asDiagonal();
    const Matrix A = Si * U.transpose() * H1 * V * Si;                        // Eq. (15-4)
    const Matrix B = (Sr * V.transpose()).leftCols(m);
    const Matrix C = (U * Sr).topRows(p);
    return ERAResult{systems::StateSpace(A, B, C, h[0], Ts), s, n};
}
```

**Phase 9 review** (`sandbox/phase9_review.md`): (1) why the simulation fit, not the prediction fit, is the honest metric; (2) why the Hankel matrix's rank is the system order; (3) two causes of coherence below 1; (4) RLS as a Kalman filter; (5) the effect of coloured noise on ARX.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V59.1 | ERA on the exact impulse response of `c2d(E3, 0.01)`, `r = 10`: `s(2)/s(1)`; poles; `ss2tf` | `< 1e-10`; equal to truth to `1e-8` |
| V59.2 | 1 % noise on the impulse data | `s(2)/s(1) < 0.05`; poles within `2 %` |
| V59.3 | `markovParametersByCorrelation` from 20 000 PRBS samples, `k ≤ 20` | within `3 %` |
| V59.4 | `validate` on a correct-order ARX model; on `n_a = 1` | `residualsWhite` true; false |
| V59.5 | `ex08` table: ARX, ERA and rational-fit poles and dc gains | within `2 %` of one another and of the truth |

**Exit criteria.**
- [ ] V59.1–V59.5; review written
- [ ] Commit `Day 59: Phase 9 complete`; tag `phase-9`

**References.** [R36]; [R19, §10.6, §16.6]; [R50].

---
## 16. Phase 10 — Robust, Predictive and Nonlinear Optimal Control (Days 60–66)

**Phase objective.** Implement uncertainty modelling and small-gain tests, the H∞ norm, a QP solver, model predictive control with constraints, iterative LQR, and H∞ synthesis. Each component is a composition of existing library functions.

### Day 60 — Uncertainty Models and the Small-Gain Theorem

**Objectives.** C++: pointwise operations on `FrequencyResponse`, weight construction by rational fitting, a tabulated report. Control: multiplicative uncertainty, weight construction from plant families and delays, the small-gain theorem, robust stability and robust performance.

**Background.** Model error is described by a *multiplicative uncertainty set*

```
P̃(s) = P(s)(1 + W_m(s)Δ(s)),   ‖Δ‖∞ ≤ 1                                                             (16-1)
```

where `|W_m(jω)|` bounds the relative error at each frequency. `W_m` is built from a plant family as `max_i |(P_i − P)/P|` fitted by a stable low-order rational function, or for an unknown delay `τ ≤ τ_max` from `|e^{−jωτ} − 1| ≤ |W_m(jω)|` with `W_m(s) = 1.05 τ_max s/(1 + τ_max s/2)` (verified numerically). The *small-gain theorem* states that the interconnection of stable `M` and `Δ` is stable for all `‖Δ‖∞ ≤ 1` if and only if `‖M‖∞ < 1`; for Eq. (16-1) the relevant `M` is `W_mT`, giving the conditions

```
robust stability:  ‖W_m T‖∞ < 1;   nominal performance:  ‖W_s S‖∞ < 1;   robust performance (SISO):  max_ω (|W_s S| + |W_m T|) < 1   (16-2)
```

These are the specification language of H∞ design (Day 66). The limitations of Days 20 and 28 reappear as bounds on how small `S` and `T` can be simultaneously.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/analysis/Robustness.hpp`, `src/analysis/Robustness.cpp` | Create: `relativeErrorEnvelope(nominal, family, w)`; `delayUncertaintyWeight(tauMax, margin = 1.05)`; `struct RobustnessReport { Real robustStabilityPeak, nominalPerformancePeak, robustPerformancePeak, worstFrequency; bool robustlyStable, nominallyPerforming, robustlyPerforming; }`; `robustness(L, Wm, Ws, w)` (grid; exact on Day 61) |
| `tests/analysis/test_Robustness.cpp` | Create |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V60.1 | `delayUncertaintyWeight(0.02)` covers `|e^{−jωτ} − 1|` for all `τ ≤ 0.02` on a grid | true |
| V60.2 | E3 with the Day 26 SIMC PI: `robustlyStable` for `τ_max` below the delay margin; above it | true; false (the small-gain test turns false slightly *before* the exact margin) |
| V60.3 | ±30 % gain family: `relativeErrorEnvelope` | constant `0.3`; robust stability ⇔ `‖T‖∞ < 3.33` |
| V60.4 | `W_s` demanding bandwidth above the frequency where `|W_m| ≈ 1` | `robustlyPerforming == false` |

**Exit criteria.**
- [ ] V60.1–V60.4; the small-gain theorem stated with its Nyquist interpretation
- [ ] Commit

**References.** [R8, ch. 12]; [R21, §7.1–7.6]; [R22, ch. 4–6].

---

### Day 61 — The H∞ Norm

**Objectives.** C++: bisection over a matrix-spectrum property, tolerance and bracket selection, SISO and MIMO overloads. Control: the H∞ norm as worst-case gain, the bounded-real lemma, why grid evaluation is insufficient.

**Background.** `‖G‖∞ = sup_ω σ_max(G(jω))` is the induced `L₂` gain; for SISO systems it is the Bode magnitude peak. For a stable `G = (A, B, C, D)` and `γ > σ_max(D)`, `‖G‖∞ < γ` if and only if the Hamiltonian

```
H_γ = [[A + BR⁻¹DᵀC, BR⁻¹Bᵀ], [−Cᵀ(I + DR⁻¹Dᵀ)C, −(A + BR⁻¹DᵀC)ᵀ]],   R = γ²I − DᵀD                  (16-3)
```

has no eigenvalue on the imaginary axis (bounded-real lemma). Bisection on `γ` yields the norm to arbitrary precision and does not miss narrow resonance peaks, which a frequency grid does. `‖S‖∞ = M_s` (Day 28); `‖W_mT‖∞` is the robust-stability measure of Day 60. The H2 norm (Day 40) is the average gain; the H∞ norm the worst-case gain.

**Implementation.**

| File | Action |
|---|---|
| `Norms.hpp/.cpp` | Add `hinfnorm(const StateSpace& G, Real tol = 1e-6)` (Listing 16-1); `hinfnorm(const FrequencyResponse&)` (grid, for data); `sigmaMax(const ComplexMatrix&)`; `peakFrequency(...)`; discrete systems are mapped to continuous by the bilinear transform, which preserves the norm |
| `Robustness.cpp` | Use `hinfnorm` for the robust-stability test |
| `tests/analysis/test_Norms.cpp` | Extend |

**Listing 16-1 — H∞ norm by bisection**

```cpp
Real hinfnorm(const systems::StateSpace& G, Real tol) {
    if (!G.isStable()) throw InvalidArgument("hinfnorm: system must be stable");
    const Matrix &A = G.A(), &B = G.B(), &C = G.C(), &D = G.D();
    const Index n = A.rows();
    auto hasImagEigen = [&](Real gamma) {
        const Matrix R = gamma * gamma * Matrix::Identity(D.cols(), D.cols()) - D.transpose() * D;
        const Matrix Rinv = R.ldlt().solve(Matrix::Identity(R.rows(), R.cols()));
        const Matrix Abar = A + B * Rinv * D.transpose() * C;
        Matrix H(2 * n, 2 * n);                                                   // Eq. (16-3)
        H << Abar, B * Rinv * B.transpose(),
             -C.transpose() * (Matrix::Identity(D.rows(), D.rows()) + D * Rinv * D.transpose()) * C, -Abar.transpose();
        Eigen::EigenSolver<Matrix> es(H, false);
        const Real scale = H.norm();
        for (Index i = 0; i < 2 * n; ++i)
            if (std::abs(es.eigenvalues()(i).real()) < 1e-8 * (1.0 + scale)) return true;
        return false;
    };
    Real lo = std::max(sigmaMax(D.cast<Complex>()), gridPeak(G, 200)) * (1 - 1e-6);
    Real hi = 2.0 * lo + 1.0;
    while (hasImagEigen(hi)) hi *= 2.0;
    while ((hi - lo) > tol * (1.0 + hi)) { const Real mid = 0.5 * (lo + hi); (hasImagEigen(mid) ? lo : hi) = mid; }
    return hi;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V61.1 | `hinfnorm({1},{1,1})` | `1.0 ± 1e-6` |
| V61.2 | `hinfnorm(E2)`; its frequency; a 200-point grid estimate | `1.0079`; `1.968 rad/s`; grid lower by at least `0.1 %` |
| V61.3 | `hinfnorm(S)` for `L = {1}/{1,3,2,0}` | `1.57 ± 1e-3` |
| V61.4 | `2×2` MIMO example (two coupled E1 systems) versus dense-grid `σ_max` | within `1e-4` |
| V61.5 | `hinfnorm(c2d(E2, 0.01))` | within `0.1 %` of V61.2 |

**Exit criteria.**
- [ ] V61.1–V61.5; V60.2 re-run with the exact norm (tolerance `±2 %`)
- [ ] Commit

**References.** [R23, §4.3–4.4]; [R48]; [R39]; [R21, §4.10].

---

### Day 62 — Quadratic Programming

**Objectives.** C++: a solver with diagnostics and warm start, `Eigen::LDLT` for KKT systems, oracle-based tests. Control: the QP, convexity, the KKT conditions, the equality-constrained solution, Hildreth's dual method.

**Background.** The QP `min_x ½xᵀHx + fᵀx` subject to `Ax ≤ b`, `A_eq x = b_eq` is convex for `H ⪰ 0`, in which case the KKT conditions are necessary and sufficient:

```
Hx + f + Aᵀλ + A_eqᵀμ = 0,   Ax ≤ b,   A_eq x = b_eq,   λ ≥ 0,   λ_i (Ax − b)_i = 0                      (16-4)
```

With equalities only, `[[H, A_eqᵀ], [A_eq, 0]] [x; μ] = [−f; b_eq]`. With inequalities and `H ≻ 0`, the dual problem `max_{λ≥0} −½λᵀ(AH⁻¹Aᵀ)λ − λᵀ(AH⁻¹f + b)` is solved by Hildreth's coordinate ascent: each `λ_i` is set to its unconstrained maximiser projected onto `λ_i ≥ 0`, sweeping until convergence; then `x = −H⁻¹(f + Aᵀλ)`. The method converges linearly, detects infeasibility by dual divergence, and suits MPC-sized problems; `solveQP` is the seam for an external solver (Table 19-1).

**Implementation.**

| File | Action |
|---|---|
| `NumericalMethods.hpp/.cpp` | Add Listing 16-2 and `solveEqualityQP(H, f, Aeq, beq)`, `kktResidual(H, f, A, b, result)` |
| `tests/core/test_NumericalMethods.cpp` | Extend |

**Listing 16-2 — Hildreth's method**

```cpp
struct QPResult { Vector x, lambda, mu; int iterations = 0; bool converged = false; Real maxViolation = 0.0; };
QPResult solveQP(const Matrix& H, const Vector& f, const Matrix& A, const Vector& b,
                 const Matrix& Aeq = Matrix(), const Vector& beq = Vector(),
                 Real tol = 1e-9, int maxIter = 5000, const Vector& lambda0 = Vector());

QPResult solveQP(const Matrix& H, const Vector& f, const Matrix& A, const Vector& b, const Matrix&, const Vector&,
                 Real tol, int maxIter, const Vector& lambda0) {   // equalities eliminated by the caller or appended as inequality pairs
    Eigen::LLT<Matrix> llt(H);
    if (llt.info() != Eigen::Success) throw InvalidArgument("solveQP: H must be positive definite");
    const Vector xu = -llt.solve(f);
    QPResult r; r.x = xu;
    if (A.rows() == 0) { r.converged = true; return r; }
    const Matrix HinvAt = llt.solve(A.transpose());
    const Matrix P = A * HinvAt;
    const Vector d = A * xu - b;
    Vector lam = (lambda0.size() == A.rows()) ? lambda0 : Vector::Zero(A.rows());
    for (int it = 0; it < maxIter; ++it) {
        Real change = 0.0;
        for (Index i = 0; i < A.rows(); ++i) {
            const Real w = d(i) + P.row(i).dot(lam) - P(i, i) * lam(i);
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
    if (!r.converged && lam.norm() > 1e8) throw NumericalError("solveQP: dual diverged (infeasible problem)");
    return r;
}
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V62.1 | `min (x₁−1)² + (x₂−2)²` s.t. `x₁ + x₂ ≤ 2`; s.t. `x₁ + x₂ ≤ 5` | `(0.5, 1.5)`, `λ = 1`; `(1, 2)`, `λ = 0` |
| V62.2 | `min ½‖x − 3·1‖²` s.t. `−1 ≤ x ≤ 1` | all ones |
| V62.3 | Random `20×20` convex QP (`H = MᵀM + I`), 30 feasible constraints: `kktResidual`; active set | `< 1e-7`; `λ > 0` exactly on tight constraints |
| V62.4 | `min ½xᵀx` s.t. `x₁ + x₂ = 1` | `(0.5, 0.5)` |
| V62.5 | `x ≤ 0` and `x ≥ 1` | throws |
| V62.6 | Warm start on a perturbed problem | `< 20 %` of cold-start iterations |

**Exit criteria.**
- [ ] V62.1–V62.6; Eq. (16-4) stated with the meaning of complementary slackness
- [ ] Commit

**References.** [R26, §4.4, §5.5]; [R25, §2.4]; [R41, ch. 16].

---

### Day 63 — Model Predictive Control: Formulation

**Objectives.** C++: block-Toeplitz construction, configurations with optional constraint fields, separation of formulation (`buildQP`) from execution (`compute`), solver warm-start state. Control: the receding horizon, prediction matrices, the condensed QP, terminal weighting, control horizon and blocking, feasibility.

**Background.** At each step, from the current state `x₀`, an input sequence `U = [u₀; …; u_{N−1}]` is chosen by solving a finite-horizon problem; only `u₀` is applied and the problem is re-solved at the next step. For the discrete model (Eq. 11-1),

```
X = F x₀ + Φ U,   F = [A; A²; …; A^N],   Φ_{ij} = A^{i−j}B  (i ≥ j), 0 otherwise                              (16-5)
J = Σ_{k=1}^{N} (x_k − x_ref)ᵀQ(x_k − x_ref) + Σ_{k=0}^{N−1} u_kᵀRu_k  (last Q replaced by Q_N)
  = ½UᵀHU + fᵀU + const,   H = 2(ΦᵀQ̄Φ + R̄),   f = 2ΦᵀQ̄(Fx₀ − X_ref)                                       (16-6)
```

with `Q̄ = diag(Q, …, Q, Q_N)`, `R̄ = diag(R, …, R)`. Input bounds, rate limits (through a differencing matrix) and state bounds (through `Φ`) are linear in `U`, so the problem is a QP. With `Q_N = P` from the DARE (Eq. 13-4), the unconstrained MPC equals infinite-horizon LQR, and the terminal cost acts as a Lyapunov function in the constrained case under the conditions of [R24, §2]. A control horizon `N_u < N` holds `u` constant after `N_u` steps (blocking). Constraints may render the QP infeasible; Day 64 adds slack variables.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/MPC.hpp`, `src/control/MPC.cpp` | Create (Listing 16-3) |
| `tests/control/test_MPC.cpp` | Create |

**Listing 16-3 — `MPC` interface**

```cpp
class MPC final : public Controller {
public:
    struct Config {
        Index N = 20, Nu = -1;                       // Nu < 0: Nu = N
        Matrix Q, R, QN;                             // QN empty: DARE solution
        Vector uMin, uMax, duMin, duMax;             // empty: unconstrained
        Vector xMin, xMax;                           // hard state box; Day 64 adds soft bounds
        bool warmStart = true;
    };
    MPC(systems::StateSpace discretePlant, Config cfg);
    struct QP { Matrix H; Vector f; Matrix A; Vector b; };
    QP     buildQP(const Vector& x0, const Vector& xRef) const;              // Eq. (16-6) and constraints
    Vector compute(const Vector& x, const Vector& r, Real t) override;        // measurement = state; r = state reference
    Matrix predictionF() const;  Matrix predictionPhi() const;               // Eq. (16-5)
    const QPResult& lastSolve() const;
    void   reset() override;
    Real   Ts() const override;  Index inputs() const override;  Index outputs() const override;
private:
    systems::StateSpace sys_; Config cfg_;
    Matrix F_, Phi_, Qbar_, Rbar_, Hqp_;
    Vector uPrev_; QPResult last_;
};
```

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V63.1 | `predictionF/Phi` versus `lsim(sys_d, U)` for random `U` | equal to `1e-12` |
| V63.2 | `Q_N = P_DARE`, unconstrained, `N = 5`: `compute` versus `−K_dlqr x` | equal to `1e-8` |
| V63.3 | `Q_N = Q`, `N = 50` | gain within `1e-4` of `K_dlqr` |
| V63.4 | Input bounds: `kktResidual` at the returned solution | `< 1e-8` |
| V63.5 | `N_u = 2` | `U` entries after the second are equal |

**Exit criteria.**
- [ ] V63.1–V63.5; `F` and `Φ` for `N = 3` written by hand
- [ ] Commit

**References.** [R24, ch. 1–2]; [R25, ch. 1–3]; [R28, ch. 2–3].

---

### Day 64 — Constrained MPC in Closed Loop

**Objectives.** C++: slack-variable augmentation, solve timing, Monte Carlo under noise. Control: soft constraints, warm start, comparison with saturated LQR and dynamic programming, tuning.

**Background.** Soft output constraints `y_min − ε ≤ y ≤ y_max + ε`, `ε ≥ 0`, with a large linear and quadratic penalty on `ε`, keep the QP feasible under disturbances; input constraints remain hard. The previous solution shifted by one step initialises the next solve. Under input constraints MPC plans the saturation, whereas saturated LQR does not; with a long horizon MPC approaches the DP optimum of Day 48. The solve time must fit within `T_s`.

**Implementation.**

| File | Action |
|---|---|
| `MPC.hpp/.cpp` | Add `Vector ySoftMin, ySoftMax; Real slackWeight` to `Config`; slack augmentation in `buildQP`; solve timing in `lastSolve()` |
| `examples/ex09_mpc.cpp` | Create: (a) discrete E4 with `|u| ≤ 1` — saturated LQR, MPC, DP policy; (b) cart–pole with `|F| ≤ 5 N` from an angle where saturated LQR fails, with the EKF; (c) balancing with `p ∈ [−0.5, 0.5]`; table of settling time, cost, max `|u|`, mean/max solve time, iterations cold versus warm; Monte Carlo of 20 runs for (b) |
| tests | Extend `test_MPC.cpp` |

**Verification.** Discrete E4, `T_s = 0.1`, `x₀ = [2, 0]`, `|u| ≤ 1`, `N = 40`, `Q_N = P`.

| ID | Check | Expected |
|---|---|---|
| V64.1 | Steps to `‖x‖ < 0.05`: MPC versus DP; versus saturated LQR | at most `10 %` more than DP; at least `20 %` fewer than saturated LQR (values recorded) |
| V64.2 | `|u|` at every step | `≤ 1 + 1e-9` |
| V64.3 | Soft `|x₁| ≤ 1.5` from `x₀ = [2, 0]`: `ε` after the initial steps; the same constraint hard | `→ 0`; throws at `k = 0` |
| V64.4 | Cart–pole from `θ₀ = 0.25`, `|F| ≤ 5`: MPC; saturated LQR with the same weights | `|θ(5)| < 0.01`; failure |
| V64.5 | Warm start versus cold start | mean iterations reduced by at least `50 %` |

**Exit criteria.**
- [ ] V64.1–V64.5; `ex09` runs with its table and plots
- [ ] Commit

**References.** [R24, §1.5, §2.4–2.5]; [R28, §3.4].

---

### Day 65 — Iterative LQR

**Objectives.** C++: an algorithm assembled from library pieces with careful index bookkeeping, regularisation, backtracking line search, cost-based convergence. Control: trajectory optimisation by repeated local LQR; feedforward plus time-varying feedback; the cart–pole swing-up.

**Background.** For `min Σ_{k=0}^{N−1} l(x_k, u_k) + l_N(x_N)` subject to `x_{k+1} = f(x_k, u_k)`, iLQR repeats: (1) roll out the nominal trajectory; (2) linearise `f` (`A_k, B_k`) and quadratise the cost along it; (3) a backward pass (Eq. 13-3 with gradient terms) yields feedforward `k_k` and feedback `K_k`; (4) a forward pass applies

```
u_k = ū_k + α k_k + K_k (x_k − x̄_k)                                                                 (16-7)
```

through the nonlinear dynamics with the largest `α ∈ {1, ½, ¼, …}` that reduces the cost; (5) convergence when the cost change falls below tolerance. A regularisation `μI` is added when a backward-pass Hessian is not positive definite. The result is an open-loop trajectory and a time-varying feedback law. Differential dynamic programming additionally retains second-order dynamics terms.

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/iLQR.hpp`, `src/control/iLQR.cpp` | Create: `struct iLQRProblem { std::function<Vector(const Vector&, const Vector&)> f; std::function<Real(const Vector&, const Vector&)> l; std::function<Real(const Vector&)> lN; Index n, m, N; Real Ts; }`; `struct iLQRResult { std::vector<Vector> x, u; std::vector<Matrix> K; std::vector<Real> costHistory; bool converged; int iterations; }`; `ilqr(problem, x0, uInit, maxIter = 100, tol = 1e-6, muInit = 1e-6)` |
| `tests/control/test_iLQR.cpp` | Create |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V65.1 | Discrete E4 with quadratic cost | converges in ≤ 2 iterations; `u` equals the `finiteHorizonLQR` rollout to `1e-8` |
| V65.2 | Cart–pole swing-up (`T_s = 0.02`, `N = 150`, `Q = diag(1, 0.1, 10, 0.1)`, `R = 0.01`, terminal `100Q`, from `θ = π`) | `|θ_N| < 0.05`, `|p_N| < 0.2`; cost strictly decreasing; `< 60` iterations; at least one sign change of `θ̇` before upright |
| V65.3 | Tracking the trajectory with `K_k` in the Simulator from `θ₀ = π + 0.05` | `|θ_N| < 0.1` |

**Exit criteria.**
- [ ] V65.1–V65.3
- [ ] Commit

**References.** [R37]; [R14, ch. 1].

---

### Day 66 — H∞ Synthesis

**Objectives.** C++: the largest function of the library decomposed into testable units with assumption checks and an independent verification. Control: the generalized plant and lower LFT, the H∞ problem, mixed sensitivity, the DGKF two-Riccati central controller, γ-iteration, the LQG limit.

**Background.** The *generalized plant* `P` maps `(w, u)` to `(z, y)` (Figure 16-1); the closed loop `T_zw = F_l(P, K)`. The H∞ problem finds a stabilising `K` with `‖T_zw‖∞ < γ`, and γ-iteration minimises `γ`. *Mixed sensitivity* chooses `z = [W_sS; W_uKS; W_tT] w`, encoding Eq. (16-2).

**Figure 16-1 — Generalized plant interconnection (lower linear fractional transformation)**

```
        w ───────▶ ┌───────────────┐ ───────▶ z        P = [[P11, P12], [P21, P22]]
                   │       P       │                   T_zw = P11 + P12 K (I − P22 K)⁻¹ P21
        u ───────▶ │               │ ───────▶ y
        ▲          └───────────────┘          │
        │          ┌───────────────┐          │
        └───────── │       K       │ ◀────────┘
                   └───────────────┘
```

Under the regularity assumptions `D₁₁ = 0`, `D₂₂ = 0`, `D₁₂ᵀ[C₁ D₁₂] = [0 I]`, `D₂₁[B₁ᵀ D₂₁ᵀ] = [0 I]`, the solution [R34] is

```
X∞ = solveARE(A, B₂B₂ᵀ − γ⁻²B₁B₁ᵀ, C₁ᵀC₁),   Y∞ = solveARE(Aᵀ, C₂ᵀC₂ − γ⁻²C₁ᵀC₁, B₁B₁ᵀ)
conditions:  X∞ ⪰ 0,  Y∞ ⪰ 0,  ρ(X∞Y∞) < γ²
F = −B₂ᵀX∞,   L = −Y∞C₂ᵀ,   Z = (I − γ⁻²Y∞X∞)⁻¹,   Â = A + γ⁻²B₁B₁ᵀX∞ + B₂F + ZLC₂
K:   x̂̇ = Âx̂ − ZLy,   u = Fx̂                                                                          (16-8)
```

As `γ → ∞` the two equations become the LQR and Kalman Riccati equations and `K` becomes the LQG controller. Singular problems, `D₁₁ ≠ 0` and general MIMO weight selection require the scaling and loop-shifting of [R23, ch. 16] (Table 19-1).

**Implementation.**

| File | Action |
|---|---|
| `include/csl/control/HInfinity.hpp`, `src/control/HInfinity.cpp` | Create: `struct GeneralizedPlant { StateSpace P; Index nw, nu, nz, ny; }`; `mixedSensitivityPlant(G, Ws, Wu, Wt)`; `struct HInfResult { StateSpace K; Real gamma; Matrix Xinf, Yinf; bool ok; std::string message; }`; `hinfsynFixedGamma(P, gamma)` (Eq. 16-8, or `ok = false` naming the failed condition); `hinfsyn(P, gammaLo = 0, gammaHi = 1e3, tol = 1e-3)` (bisection); `closedLoop(P, K)` returning `T_zw` |
| `tests/control/test_HInfinity.cpp` | Create |

**Verification.** Plant `G = 1/(s − 1)`; weights `W_s = 0.5(s/2 + 1)/(s + 0.01)`, `W_u = 0.1`, `W_t = 0.1(s + 1)/(s/10 + 1)`, scaled so that the regularity assumptions hold (the scaling is documented in the test).

| ID | Check | Expected |
|---|---|---|
| V66.1 | `hinfsyn`: `γ*`; `hinfnorm(closedLoop(P, K))` | value recorded on first run; `≤ γ*(1 + 1e-3)` |
| V66.2 | `feedback(series(K, G)).isStable()` | true |
| V66.3 | Gang of four: `|S| ≤ γ*/|W_s|`, `|T| ≤ γ*/|W_t|` on a grid | true |
| V66.4 | `γ = 1e4`: `K` versus the LQG controller from `lqr(A, B₂, C₁ᵀC₁, I)` and the dual CARE | gains within `1 %` |
| V66.5 | `hinfsynFixedGamma` below `γ*` | `ok = false` with the failed condition named |

**Exit criteria.**
- [ ] V66.1–V66.5; the meaning of `X∞`, `Y∞` and `ρ(X∞Y∞) < γ²` can be stated
- [ ] Commit `Day 66: Phase 10 complete`; tag `phase-10`

**References.** [R34]; [R23, ch. 14, 16]; [R21, §9.3].

---
## 17. Phase 11 — Packaging and Release (Days 67–70)

**Phase objective.** Make the library installable and consumable by an external project, document it, place it under continuous integration, and demonstrate every module in one program.

### Day 67 — Installation and Export

**Objectives.** CMake: `install(TARGETS … EXPORT …)`, `install(DIRECTORY …)`, `install(EXPORT …)`, package configuration files, version files, `find_package`; the meaning of the `BUILD_INTERFACE`/`INSTALL_INTERFACE` expressions of Listing 7-4.

**Implementation.**

| File | Action |
|---|---|
| `CMakeLists.txt` | Add `include(CMakePackageConfigHelpers)`; `install(TARGETS csl EXPORT cslTargets ARCHIVE DESTINATION lib)`; `install(DIRECTORY include/ DESTINATION include)`; `install(DIRECTORY ${eigen_SOURCE_DIR}/Eigen ${eigen_SOURCE_DIR}/unsupported DESTINATION include)` (self-contained package); `install(EXPORT cslTargets NAMESPACE csl:: DESTINATION lib/cmake/csl)`; `configure_package_config_file` and `write_basic_package_version_file` for `cslConfig.cmake`, `cslConfigVersion.cmake` |
| `cmake/cslConfig.cmake.in` | Create: includes `cslTargets.cmake` |
| `tests/consumer/CMakeLists.txt`, `tests/consumer/main.cpp` | Create: `find_package(csl 2.0 REQUIRED)`; a ten-line program calling `step(E1)` |
| `scripts/test_install.ps1`, `scripts/test_install.sh` | Create: configure, build, install to a temporary prefix, build and run the consumer |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V67.1 | `cmake --install build --prefix <tmp>` | `lib/csl.lib`, `include/csl/…`, `include/Eigen/…`, `lib/cmake/csl/cslConfig.cmake` present |
| V67.2 | Consumer project builds and runs against the installed tree with no reference to the source tree | — |
| V67.3 | `find_package(csl 3.0)` | fails with a version message |

**Exit criteria.**
- [ ] V67.1–V67.3
- [ ] Commit

**References.** [R5, "cmake-packages", "install"].

---

### Day 68 — Documentation

**Implementation.**

| Item | Content |
|---|---|
| `README.md` | Purpose (one paragraph); module table (Table 4-1); quick start (build, `ex01`, tests); gallery of six plots as PNG under `docs/img/`; the build-versus-buy principle in three sentences; link to this document |
| API comments | A `///` comment on every public function stating what it computes, units and conventions (rad/s, `Ts == 0`, time along rows), and what it throws; optional Doxygen configuration in `docs/Doxyfile` |
| `docs/tutorials/` | Five pages derived from the examples: *From ODE to Bode*, *Tuning a PID three ways*, *What sampling costs*, *LQR, LQG and MPC on the cart–pole*, *Identifying a model from data* |
| Consistency audit | Every identifier conforms to Table 3-2; every `Ts` is in seconds; every frequency parameter is `w` (rad/s) or carries `_hz`; every result is one of the Listing 4-3 types |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V68.1 | The quick start executes from the README alone on a clean clone | — |
| V68.2 | Script count of public declarations without a preceding `///` comment | `0` |

**Exit criteria.**
- [ ] V68.1–V68.2
- [ ] Commit

---

### Day 69 — Continuous Integration and Static Checks

**Implementation.**

| File | Content |
|---|---|
| `.github/workflows/ci.yml` | Jobs: `windows-latest` (MSVC, Ninja, Debug and Release, `ctest`); `ubuntu-latest` (GCC, `-Wall -Wextra -Werror`, Debug with `-fsanitize=address,undefined`, Release, `ctest`); install/consumer script; `build/_deps` cached |
| `.clang-format` | Based on the LLVM style, 4-space indentation, 110 columns; applied once to the tree; a CI step checks formatting |
| `CMakeLists.txt` | Warnings as errors on `csl` and `csl_tests` (`/WX`, `-Werror`) |
| `CHANGELOG.md` | Entry for 2.0.0 |
| optional | `clang-tidy` with `bugprone-*`, `performance-*`, `modernize-use-override` |

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V69.1 | Both CI jobs | green |
| V69.2 | Sanitizer job | no findings (any finding is a defect: fix it and add the test that exposes it) |

**Exit criteria.**
- [ ] V69.1–V69.2
- [ ] Commit

---

### Day 70 — Full Pipeline and Retrospective

**Implementation.** `examples/ex10_full_pipeline.cpp`, on E3 with E5 as the nonlinear conclusion:

| Step | Content | Modules |
|---|---|---|
| 1 | E3 as `StateSpace` → `ss2tf`; poles, dc gain, `damp`; Bode and step plots | systems, analysis, simulation, plot |
| 2 | ZOH discretization at 1 ms; PRBS excitation; measurement noise; `lsim` | conversions, signals, simulation |
| 3 | ARX, ERA and ETFE + rational fit; validation; Bode overlay; model selection by validation fit | sysid, dsp, analysis, plot |
| 4 | SIMC PI on the identified model; margins, `M_s`, gang of four; Simulator with saturation and anti-windup; Tustin discretization and PM loss | control, analysis, simulation |
| 5 | LQR with Bryson weights; steady-state Kalman filter; LQG; LQG loop margins; Monte Carlo under noise | control, estimation, simulation |
| 6 | MPC with voltage and current limits; comparison table against LQG | control, core (QP) |
| 7 | Multiplicative uncertainty from the spread of the identified models; small-gain check; `hinfnorm`; H∞ mixed-sensitivity controller and its margins | analysis, control |
| 8 | Cart–pole swing-up by iLQR; balancing by LQR on an EKF estimate; MPC when the force limit binds | systems, control, estimation |
| 9 | Welch PSD of closed-loop residuals; whiteness test | dsp, sysid |
| 10 | All plots to `docs/img/`, all data to CSV | plot |

`docs/RETROSPECTIVE.md` (one page per item): the ten C++ concepts found hardest, with the day and the code that clarified each; the same for ten control concepts; changes that would be made to the type contract (Section 4.3); what would not be hand-written again and what would; three items of Table 19-1 with day plans in the format of Section 6.2.

**Verification.**

| ID | Check | Expected |
|---|---|---|
| V70.1 | `ex10` runs to completion; the numbers of steps 3–7 agree across methods within the tolerances of the corresponding day entries | — |

**Exit criteria.**
- [ ] V70.1; retrospective written
- [ ] Commit `Day 70: v2.0.0`; tag `v2.0.0`

---

## 18. Testing Strategy

### 18.1 Principles

- T1 Every day adds tests; the suite never shrinks. The completed library has approximately 350–450 test cases in about 45 files mirroring `include/csl`.
- T2 Tests target the hand-written layer only. The correctness of Eigen, the standard library and Google Test is not tested.
- T3 Every test has a known answer: a closed form, a reference value (Table 5-3), a cross-representation identity, an invariant (`S + T = 1`, Parseval, KKT residual, Riccati residual), a convergence law (RK4 error ratio), or an independent computation (bisection versus grid; ERA versus ARX).
- T4 Cross-checks between days are written deliberately: the stability limit `K = 6` of `1/(s(s+1)(s+2))` from margins (Day 18), root locus (Day 21) and Routh (Day 22); LQR cost versus simulated integral (Day 44); Kalman covariance versus DARE (Day 51); MPC versus `dlqr` (Day 63); iLQR versus finite-horizon LQR (Day 65); H∞ versus LQG (Day 66).
- T5 Stochastic tests use fixed seeds and assert bounds with margin. A bound found platform-dependent is loosened and documented; the seed is never removed.
- T6 Tolerances follow Table 3-3; a test requiring a looser tolerance indicates a numerical problem to be investigated before the tolerance is changed.
- T7 The suite runs in Debug and Release on Windows and Linux, with sanitizers on Linux (Day 69).

### 18.2 Oracles by Module

**Table 18-1 — Test oracles by module**

| Module | Oracles |
|---|---|
| core | closed-form polynomial algebra; companion roots versus known roots; `kron`/`vec` identities; Lyapunov, ARE, DARE residuals; KKT residuals; hand-solved QPs |
| systems | cross-representation `evaluate`; `ss2tf(tf2ss(H)) = H`; similarity invariance; interconnection identities; exact ZOH step; `d2c(c2d) = id` |
| simulation | analytic responses (E1, E2); order-of-accuracy ratios; specifications versus second-order formulas; Simulator versus transfer-function closed loop |
| analysis | reference margins (Table 5-3); agreement of Nyquist, root locus and Routh; `S + T = 1`; sensitivity integral; `hinfnorm` versus closed form and grid; small gain versus delay margin |
| control | PID algebra; tuning tables; LQR reference gain and cost; LQR margin bounds; separation-principle pole sets; MPC equals `dlqr` unconstrained; iLQR equals LQR on linear problems; H∞ → LQG limit; `hinfnorm(T_zw) ≤ γ` |
| estimation | KF covariance versus DARE; EKF equals KF on linear plants; unscented-transform exactness on linear and quadratic maps; RMSE and NIS bounds with seeds |
| dsp | `fft` versus `dft`; impulse and constant spectra; Parseval; window tables; white-noise PSD level; filter responses at known frequencies; `process(step)` equals `step(toTransferFunction())` |
| sysid | exact recovery from noise-free data; bounded error with noise; RLS equals batch; RLS equals KF; ERA singular-value gap; whiteness on correct versus under-fitted models |
| signals, plot | statistics with seeds; PRBS period, balance and autocorrelation; SVG structure; CSV round trip |

---

## 19. Extensions

Table 19-1 lists additions beyond Day 70, each a one- to three-day entry in the format of Section 6.2, ordered by reuse of existing components.

**Table 19-1 — Extensions**

| Extension | Builds on | Content |
|---|---|---|
| Adaptive Runge–Kutta (Dormand–Prince) | Day 7 | `rk45Step`, `integrateAdaptive` with error control and step rejection |
| Bartels–Stewart Lyapunov solver | Day 40 | `Eigen::RealSchur` with quasi-triangular back-substitution; `n` in the hundreds |
| Ordered-Schur Riccati solver | Day 44 | Schur-form reordering; more robust than the eigenvector method [R32] |
| DARE by doubling or symplectic pencil | Day 45 | quadratic convergence |
| MIMO transfer-function matrices and transmission zeros | Days 12–14 | generalized eigenvalue problem for non-square systems |
| MIMO pole placement (Kautsky–Nichols) | Day 38 | robust eigenstructure assignment |
| Balanced realization and truncation | Days 40, 42 | model reduction with Hankel-singular-value bounds |
| Active-set and interior-point QP; external solver seam | Day 62 | replace Hildreth behind `solveQP`; benchmark on MPC problems [R41] |
| Nonlinear MPC with iLQR as the inner solver | Days 63–65 | real-time iteration |
| Particle filter | Days 50, 54 | sequential importance resampling; multimodal comparison with the UKF |
| Loop transfer recovery | Day 52 | recovery of LQR margins by `Q_w` scaling |
| μ-analysis | Days 60–61 | D-scaling upper bound for structured uncertainty |
| General H∞ (loop shifting, scaling); H∞ loop shaping | Day 66 | [R23, ch. 16]; McFarlane–Glover |
| ARMAX, output-error, instrumental variables, prediction-error method | Day 56 | unbiased identification under coloured noise [R19] |
| N4SID / MOESP | Day 59 | subspace identification from input–output data [R50] |
| Chebyshev, elliptic, Bessel filters; band transforms; Parks–McClellan FIR | Day 34 | remainder of the filter-design toolbox [R29] |
| Describing functions and limit cycles | Days 26, 47 | relay feedback analysed rigorously |
| Indirect adaptive control | Days 57, 38 | RLS with online pole placement [R27] |
| Python bindings (pybind11) | Day 67 | zero-copy Eigen–NumPy interoperation |
| Real-time profile | Day 24 | fixed-size Eigen types; allocation-free `compute` paths |

---

## Appendix A — Glossary

### A.1 C++ and Tooling

| Term | Definition |
|---|---|
| Translation unit | One source file after preprocessing; the unit of compilation |
| Object file | Compiled machine code of one translation unit with its symbol table |
| Linker | Resolves symbol references across object files and libraries into an executable |
| Static library | An archive of object files linked into the executable |
| Header; include guard | File of declarations shared between translation units; `#pragma once` prevents double inclusion |
| One Definition Rule | Each entity is defined once per program; `inline` permits header definitions |
| Target (CMake) | A buildable executable or library with attached usage requirements |
| `PUBLIC` / `PRIVATE` / `INTERFACE` | Propagation scope of a target property: to the target and its users / the target only / its users only |
| Value, reference, pointer | A copy; an alias of an existing object; an address that may be null |
| `const`, `constexpr` | Immutable after initialisation; evaluated at compile time |
| Stack, heap | Automatic storage released at scope exit; dynamic storage released explicitly or by an owner |
| RAII | Resource acquisition in constructors, release in destructors |
| `unique_ptr`, `shared_ptr` | Sole-owner and reference-counted owning pointers |
| Class invariant | A condition true of every valid object, established by the constructor |
| Special member functions | Default constructor, destructor, copy and move constructors and assignments |
| Rule of Zero | Declare no special member functions when all members manage themselves |
| Move semantics; rvalue | Transfer of a resource from an expiring object; an expression denoting such an object |
| Operator overloading | Defining operators for user types |
| Inheritance; virtual; override; abstract class | The is-a relation; run-time dispatch; checked overriding; a class with a pure virtual function |
| Template | A definition instantiated per type |
| Lambda; `std::function`; functor | Anonymous function with captures; type-erased callable holder; object with `operator()` |
| Iterator; range; algorithm | Position in a sequence; `[first, last)`; function over a range |
| `std::optional` | A possibly absent value |
| Exception | An error object propagated by stack unwinding |
| `enum class` | A scoped enumeration |
| Expression template | Lazy evaluation of composite Eigen expressions in a single loop |
| Sanitizer | Compiler instrumentation detecting memory and undefined-behaviour errors at run time |

### A.2 Control and Signal Processing

| Term | Definition |
|---|---|
| State, input, output | Minimal memory of a system; manipulated variable; measured variable |
| LTI system | Linear, time-invariant system |
| State space; transfer function | Eq. (7-4); ratio of Laplace or z transforms with zero initial conditions |
| Pole; zero | Root of the denominator (mode); root of the numerator |
| Stability (continuous; discrete) | All poles with `Re p < 0`; all poles with `|p| < 1` |
| Time constant; `ω_n`; `ζ` | 63 % time of a first-order mode; natural frequency; damping ratio |
| dc gain | Steady-state output per unit constant input, Eq. (7-6) |
| Rise, peak, settling time; overshoot | Step-response specifications (Day 16) |
| Frequency response; Bode; Nyquist | `H(jω)`; magnitude and phase versus `log ω`; `H(jω)` in the complex plane |
| Gain, phase, delay margin | Gain, phase or delay increase that destabilises the loop |
| `L`, `S`, `T` | Loop, sensitivity, complementary sensitivity; Eq. (9-2) |
| Bandwidth | Frequency at which `|T|` falls to −3 dB |
| Minimum phase; all-pass | No right-half-plane zeros or poles; unit magnitude with phase only |
| Root locus; Routh–Hurwitz | Closed-loop poles versus a gain; stability from coefficients |
| PID; anti-windup; setpoint weighting | Eq. (10-1); integrator limiting under saturation; two-degree-of-freedom proportional term |
| Lead; lag | Phase-adding and low-frequency-gain-adding first-order compensators |
| Sampling; ZOH; aliasing; Nyquist frequency | Periodic measurement; holding `u` between samples; frequency folding; `f_s/2` |
| z-transform; Tustin; matched pole–zero | Discrete transform; bilinear map; pole–zero mapping |
| DFT; FFT; PSD; window; leakage | Eq. (11-3); its fast algorithm; power per Hz; taper; spectral smearing |
| FIR; IIR; linear phase; group delay | Non-recursive and recursive filters; constant delay; `−dφ/dω` |
| Butterworth; notch | Maximally flat lowpass; narrow band-stop |
| Controllable; observable; stabilizable; detectable | Day 37 definitions |
| Pole placement; Ackermann's formula | State-feedback pole assignment; Eq. (12-2) |
| Observer; separation principle | State estimator from outputs; independent controller and observer pole sets |
| Lyapunov equation; Gramian | Eq. (12-4); `W_c`, `W_o` |
| LQR; CARE; DARE | Eq. (13-1); Eq. (13-4) |
| Bryson's rule | Weights `1/max²` |
| Linearization; equilibrium | Eq. (13-6); `f(x_e, u_e) = 0` |
| Dynamic programming; Bellman equation | Eq. (13-7) |
| White noise; covariance; process and measurement noise | Eq. (14-1) |
| Kalman filter; innovation; NIS | Eq. (14-2); `y − Cx̂⁻`; `νᵀS⁻¹ν` |
| LQG; duality | LQR on the Kalman estimate; Day 52 correspondence |
| EKF; UKF; sigma points | Linearised Kalman filter; derivative-free Kalman filter; Eq. (14-4) |
| ARX; RLS; forgetting factor; persistency of excitation | Eq. (15-1); Eq. (15-2); `λ`; full-rank regressor |
| ETFE; coherence; Markov parameters; ERA; Hankel matrix | Eq. (15-3); `h_k`; Eq. (15-4) |
| Multiplicative uncertainty; small gain; robust stability; robust performance | Eq. (16-1); Eq. (16-2) |
| H2 norm; H∞ norm | Eq. (12-5); worst-case gain, Eq. (16-3) |
| QP; KKT conditions; Hildreth's method | Eq. (16-4); dual coordinate ascent |
| MPC; receding horizon; terminal cost | Eqs. (16-5)–(16-6); apply `u₀` and repeat; `Q_N = P` |
| iLQR | Eq. (16-7) |
| Generalized plant; LFT; mixed sensitivity; γ-iteration | Figure 16-1; `[W_sS; W_uKS; W_tT]`; bisection on `γ` |

---

## Appendix B — References

**C++ and tooling**

- [R1] S. B. Lippman, J. Lajoie, B. E. Moo, *C++ Primer*, 5th ed., Addison-Wesley, 2012.
- [R2] learncpp.com, *Learn C++* (online tutorial).
- [R3] cppreference.com, *C++ reference* (online).
- [R4] Eigen project, *Eigen 3.4 documentation* (eigen.tuxfamily.org/dox).
- [R5] Kitware, *CMake documentation and tutorial* (cmake.org/cmake/help/latest).
- [R6] Google, *GoogleTest User's Guide* (google.github.io/googletest).
- [R7] S. Meyers, *Effective Modern C++*, O'Reilly, 2014.
- [R47] Mozilla Developer Network, *SVG element reference* (online).
- [R49] H. Sutter, *Guru of the Week* and "Rule of Zero" (herbsutter.com).

**Control systems**

- [R8] K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed., Princeton University Press, 2021 (free PDF).
- [R9] G. F. Franklin, J. D. Powell, A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019.
- [R10] N. S. Nise, *Control Systems Engineering*, 8th ed., Wiley, 2019.
- [R11] K. J. Åström, B. Wittenmark, *Computer-Controlled Systems*, 3rd ed., Dover, 2011.
- [R12] C.-T. Chen, *Linear System Theory and Design*, 4th ed., Oxford University Press, 2013.
- [R13] B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic Methods*, Dover, 2007 (free PDF).
- [R14] D. P. Bertsekas, *Dynamic Programming and Optimal Control*, Vol. 1, 4th ed., Athena Scientific, 2017.
- [R15] D. E. Kirk, *Optimal Control Theory: An Introduction*, Dover, 2004.
- [R16] H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002.
- [R17] D. Simon, *Optimal State Estimation*, Wiley, 2006.
- [R18] S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
- [R19] L. Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall, 1999.
- [R20] R. Pintelon, J. Schoukens, *System Identification: A Frequency Domain Approach*, 2nd ed., Wiley-IEEE, 2012.
- [R21] S. Skogestad, I. Postlethwaite, *Multivariable Feedback Control*, 2nd ed., Wiley, 2005.
- [R22] J. C. Doyle, B. A. Francis, A. R. Tannenbaum, *Feedback Control Theory*, Macmillan, 1992 (free PDF).
- [R23] K. Zhou, J. C. Doyle, *Essentials of Robust Control*, Prentice Hall, 1998.
- [R24] J. B. Rawlings, D. Q. Mayne, M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill, 2017 (free PDF).
- [R25] L. Wang, *Model Predictive Control System Design and Implementation Using MATLAB*, Springer, 2009.
- [R26] S. Boyd, L. Vandenberghe, *Convex Optimization*, Cambridge University Press, 2004 (free PDF).
- [R27] K. J. Åström, B. Wittenmark, *Adaptive Control*, 2nd ed., Dover, 2008.
- [R28] J. M. Maciejowski, *Predictive Control with Constraints*, Prentice Hall, 2002.
- [R45] University of Michigan et al., *Control Tutorials for MATLAB and Simulink* (ctms.engin.umich.edu).
- [R46] Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.

**Signal processing and numerical methods**

- [R29] A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Pearson, 2010.
- [R30] S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, California Technical Publishing, 1997 (free online).
- [R31] SciPy developers, *scipy.signal reference* (docs.scipy.org).
- [R41] J. Nocedal, S. J. Wright, *Numerical Optimization*, 2nd ed., Springer, 2006.
- [R42] S. C. Chapra, R. P. Canale, *Numerical Methods for Engineers*, 7th ed., McGraw-Hill, 2015.

**Papers**

- [R32] A. J. Laub, "A Schur method for solving algebraic Riccati equations," *IEEE Trans. Automatic Control*, 24(6), 1979.
- [R33] J. C. Doyle, "Guaranteed margins for LQG regulators," *IEEE Trans. Automatic Control*, 23(4), 1978.
- [R34] J. C. Doyle, K. Glover, P. P. Khargonekar, B. A. Francis, "State-space solutions to standard H2 and H∞ control problems," *IEEE Trans. Automatic Control*, 34(8), 1989.
- [R35] E. A. Wan, R. van der Merwe, "The unscented Kalman filter for nonlinear estimation," *Proc. IEEE AS-SPCC*, 2000.
- [R36] J.-N. Juang, R. S. Pappa, "An eigensystem realization algorithm for modal parameter identification and model reduction," *J. Guidance, Control, and Dynamics*, 8(5), 1985.
- [R37] Y. Tassa, T. Erez, E. Todorov, "Synthesis and stabilization of complex behaviors through online trajectory optimization," *Proc. IEEE/RSJ IROS*, 2012.
- [R38] S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning," *J. Process Control*, 13(4), 2003.
- [R39] N. A. Bruinsma, M. Steinbuch, "A fast algorithm to compute the H∞-norm of a transfer function matrix," *Systems & Control Letters*, 14(4), 1990.
- [R40] G. Welch, G. Bishop, "An Introduction to the Kalman Filter," TR 95-041, University of North Carolina, 2006.
- [R43] E. C. Levy, "Complex-curve fitting," *IRE Trans. Automatic Control*, 4(1), 1959; C. K. Sanathanan, J. Koerner, "Transfer function synthesis as a ratio of two complex polynomials," *IEEE Trans. Automatic Control*, 8(1), 1963.
- [R44] M. G. Safonov, M. Athans, "Gain and phase margin for multiloop LQG regulators," *IEEE Trans. Automatic Control*, 22(2), 1977.
- [R48] S. Boyd, V. Balakrishnan, "A regularity result for the singular values of a transfer matrix and a quadratically convergent algorithm for computing its L∞-norm," *Systems & Control Letters*, 15(1), 1990.
- [R50] P. Van Overschee, B. De Moor, *Subspace Identification for Linear Systems*, Kluwer, 1996.

---

*End of Design Document V3.*
