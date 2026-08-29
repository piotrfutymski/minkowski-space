[English version below](#english-version)

# minkowski-space

[![crates.io](https://img.shields.io/crates/v/minkowski-space.svg)](https://crates.io/crates/minkowski-space)
[![Documentation](https://docs.rs/minkowski-space/badge.svg)](https://docs.rs/minkowski-space)
[![Build](https://github.com/piotrfutymski/minkowski-space/actions/workflows/rust.yml/badge.svg)](https://github.com/piotrfutymski/minkowski-space/actions/workflows/rust.yml)

Biblioteka do symulacji fizyki w płaskiej czasoprzestrzeni Minkowskiego. Symulacja odbywa się w **2+1 wymiarach**: dwóch wymiarach przestrzennych oraz jednym wymiarze czasowym.

> **Status:** eksperymentalne API

## Instalacja

Dodaj bibliotekę jako zależność w `Cargo.toml`:

```toml
[dependencies]
minkowski-space = "0.1"
```

## Opis

Biblioteka udostępnia:

- świat wypełniony obiektami, reprezentowanymi przez linie świata, oraz zdarzeniami, reprezentowanymi przez punkty w czasoprzestrzeni;
- obiekty poruszające się z prędkością mniejszą od prędkości światła (`v < c`) oraz z przyspieszeniem;
- obserwatora posiadającego własną linię świata i możliwość obserwowania obiektów z jego układu odniesienia;
- naturalnie występujące efekty relatywistyczne, takie jak dylatacja czasu, skrócenie Lorentza-Fitzgeralda i efekt Dopplera;
- dokładną symulację rozchodzenia się światła oraz możliwość obserwacji efektów Dopplera i rotacji Penrose-Terrella;
- system kolizji i detekcji zdarzeń w stożku świetlnym;
- optymalizację oraz wielowątkowość, dzięki którym biblioteka może być używana jako silnik fizyczny gry z setkami, a nawet tysiącami obiektów.

## Założenia

- Czasoprzestrzeń ma sygnaturę `(+--)`:

  ```text
  s² = t² - x² - y²
  ```

- Stosowane są jednostki naturalne, w których prędkość światła wynosi `c = 1`.
- Prędkości obiektów muszą spełniać warunek `|v| < c`.

## Szybki start / Quick start

```rust
use minkowski_space::{MWorld, ObjectConfig, ObjectObservation};
use vector2d::Vector2D;

fn main() {
    let mut world = MWorld::new();
    world.set_observer_velocity(Vector2D::new(0.3, 0.0));

    let object_id = world.register_object(ObjectConfig::at_position_with_const_speed(
        Vector2D::new(1.0, 1.0),
        Vector2D::new(0.6, 0.0),
    ));

    world.advance_by_proper_time(2.0);

    let object_observation = world.observe_object(&object_id).unwrap();
    match object_observation {
        ObjectObservation::Visible(visible_object) => {
            println!(
                "Object is visible at relative position {:?}; redshift factor: {}",
                visible_object.visible_position_in_observer_frame,
                1.0 / visible_object.relative_frequency - 1.0
            );
        }
        ObjectObservation::NotVisible => {
            println!("Object is not visible yet; only the past light cone is observable.");
        }
    }
}
```

## Podstawowe koncepty i terminologia

To rozróżnienie jest kluczowe dla poprawnego rozumienia API biblioteki.

### Laboratorium, obserwator i obiekt obserwatora

- **Laboratory frame (lab frame)** – nieruchomy, bazowy układ współrzędnych symulacji. Metoda `lab_time()` zwraca czas mierzony w tym układzie.
- **Observer frame** – chwilowy układ odniesienia związany z obserwatorem. Jego prędkość względem układu laboratoryjnego ustawia metoda `set_observer_velocity()`. Metoda `observer_tau()` zwraca czas, który upłynął w układzie obserwatora od początku symulacji.
- **Registered object** – obiekt zarejestrowany w świecie, którego stan jest zawsze aktualizowany do bieżącego czasu laboratoryjnego. Oznacza to, że modelowany stan fizyczny układu jest wycinkiem czasoprzestrzeni, dla którego `lab_time() = const`. Podczas obserwacji obiektów otrzymujemy natomiast informacje o ich względnych położeniach w stożku świetlnym układu obserwatora.

Obecny model można przedstawić za pomocą schematu:

```text
MWorld
├── laboratory coordinates: globalny układ odniesienia
├── observer_object <-> observer frame: obserwator poruszający się względem laboratorium
└── registered_objects: obiekty opisane w układzie laboratoryjnym
```

`observer_object` oznacza fizyczny obiekt reprezentujący obserwatora, a nie cały układ odniesienia. Układ laboratoryjny jest globalnym układem współrzędnych, natomiast układ obserwatora jest wyznaczany na podstawie położenia i prędkości obserwatora.

### Czas laboratoryjny i czas własny

Metoda `advance_by_proper_time()` przyjmuje przyrost czasu własnego obserwatora. Na jego podstawie aktualizowany jest czas laboratoryjny oraz stany zarejestrowanych obiektów.

Dla obserwatora poruszającego się ze stałą prędkością zachodzi zależność:

```text
γ = 1 / √(1 - |v|²)
Δt = γ · Δτ
```

gdzie `Δt` oznacza przyrost czasu laboratoryjnego, a `Δτ` – przyrost czasu własnego obserwatora.

### Ruch stały i dynamiczny

`MotionMode::AlwaysConstantVelocity` opisuje obiekt o trajektorii ustalonej w momencie utworzenia. Taki ruch może być obliczany analitycznie i jest znacząco zoptymalizowany.

`MotionMode::Dynamic` opisuje obiekt, którego prędkość i przyspieszenie mogą być zmieniane za pośrednictwem API świata.

---

# English version

[![crates.io](https://img.shields.io/crates/v/minkowski-space.svg)](https://crates.io/crates/minkowski-space)
[![Documentation](https://docs.rs/minkowski-space/badge.svg)](https://docs.rs/minkowski-space)
[![Build](https://github.com/piotrfutymski/minkowski-space/actions/workflows/rust.yml/badge.svg)](https://github.com/piotrfutymski/minkowski-space/actions/workflows/rust.yml)

## Description

`minkowski-space` is a library for simulating physics in flat Minkowski spacetime. The simulation uses **2+1 dimensions**: two spatial dimensions and one time dimension.

> **Status:** experimental API

The library provides:

- a world containing objects, represented by worldlines, and events, represented by points in spacetime;
- objects moving at arbitrary subluminal velocities (`v < c`) and with acceleration;
- an observer with its own worldline and the ability to observe objects from its frame of reference;
- naturally occurring relativistic effects such as time dilation, Lorentz–FitzGerald contraction, and the Doppler effect;
- a simulation of light propagation, including Doppler effects and Penrose-Terrell rotation;
- collision handling and event detection inside the light cone;
- optimization and multithreading suitable for use as a game physics engine with hundreds or even thousands of objects.

## Assumptions

- Spacetime uses the `(+--)` metric signature:

  ```text
  s² = t² - x² - y²
  ```

- Natural units are used, with the speed of light set to `c = 1`.
- Object velocities must satisfy `|v| < c`.

## Quick start

The Rust quick-start example is shared with the Polish section above.

## Installation

Add the library as a dependency in `Cargo.toml`:

```toml
[dependencies]
minkowski-space = "0.1"
```

## Basic concepts and terminology

This distinction is essential for understanding the library's API correctly.

### Laboratory, observer, and observer object

- **Laboratory frame (lab frame)** – the stationary, base coordinate system of the simulation. `lab_time()` returns the time measured in this frame.
- **Observer frame** – the instantaneous frame associated with the observer. The observer's velocity relative to the laboratory frame is set with `set_observer_velocity()`. `observer_tau()` returns the time elapsed in the observer's frame since the beginning of the simulation.
- **Registered object** – an object registered in the world whose state is always updated to the current laboratory time. This means that the physical state modeled by the simulation is always a slice of spacetime for which `lab_time() = const`. When objects are observed, however, the library provides information about their relative positions within the observer's light cone.

The current model can be represented as follows:

```text
MWorld
├── laboratory coordinates: the global coordinate system
├── observer_object <-> observer frame: the observer moving relative to the laboratory
└── registered_objects: objects described in laboratory coordinates
```

`observer_object` means the physical object representing the observer, not the entire frame of reference. The laboratory frame is the global coordinate system, while the observer frame is determined by the observer's position and velocity.

### Laboratory time and proper time

`advance_by_proper_time()` accepts an increment of the observer's proper time. The laboratory time and the states of registered objects are updated accordingly.

For an observer moving at constant velocity:

```text
γ = 1 / √(1 - |v|²)
Δt = γ · Δτ
```

where `Δt` is the increment of laboratory time and `Δτ` is the increment of the observer's proper time.

### Constant and dynamic motion

`MotionMode::AlwaysConstantVelocity` describes an object whose trajectory is fixed at creation time. This motion can be evaluated analytically and is significantly optimized.

`MotionMode::Dynamic` describes an object whose velocity and acceleration can be changed through the world API.