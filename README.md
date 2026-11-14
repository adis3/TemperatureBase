# TemperatureBase

An abstract Arduino C++11 interface for temperature sensors, with shared state,
calibration, validation, callbacks, freshness checks, and optional logging.
A concrete driver supplies the hardware work in `Begin()` and `Update()`.

## Features and defaults

- Celsius readings with local `uint32_t` millisecond timestamps.
- Last accepted reading preserved through acquisition and validation errors.
- Optional borrowed calibration table: offset, interpolation, and extrapolation.
- Optional inclusive valid interval, checked after calibration.
- Rejection of NaN, infinity, and unrepresentable calibration calculations.
- Up to four synchronous callback registrations, including duplicates.
- Optional `ILogger` integration with named sources and severity levels.
- Fixed callback arrays and log buffers; the base performs no heap allocation.

A new object has `Status::NoData`, `hasReading == false`, and a reading initialized
to `{NAN, 0}`. Calibration, valid-interval checking, and logging are disabled by
default. No callbacks are initially registered.

## Installation and dependencies

Place this folder in the sketchbook's `libraries` directory alongside `ILogger`.
Include `<TemperatureBase.h>`. The metadata specifies `architectures=*` because
the base contains no board-specific driver code. A concrete driver's requirements
determine which hardware is supported.

`TemperatureFake` is a separate optional simulated driver. `SerialLogger` is a
separate optional implementation of `ILogger`; neither is a dependency of this base.

## Minimal sensor example

This complete sketch submits a simulated measurement every second. A real driver
replaces the measurement acquisition while keeping the same publication path.

```cpp
#include <TemperatureBase.h>

class DemoSensor : public TemperatureBase {
 public:
  DemoSensor() : TemperatureBase(nullptr, "Demo"), last_update_ms_(0) {}
  bool Begin() override { return true; }

  void Update(uint32_t now_ms) override {
    if (static_cast<uint32_t>(now_ms - last_update_ms_) < 1000) {
      return;
    }
    last_update_ms_ = now_ms;
    PublishTemperature(20.0f, now_ms);
  }

 private:
  uint32_t last_update_ms_;
};

DemoSensor sensor;
const TemperatureBase::CalibrationPoint kCalibration[] = {
    {0.0f, 0.5f}, {25.0f, 25.3f}, {100.0f, 100.5f},
};

void HandleUpdate(const TemperatureBase& source,
                  const TemperatureBase::State& state, void*) {
  Serial.print(source.GetName());
  if (state.status == TemperatureBase::Status::Ok) {
    Serial.print(": ");
    Serial.println(state.reading.temperatureC);
  } else {
    Serial.println(": reading unavailable; inspect status");
  }
}

void setup() {
  Serial.begin(115200);
  const bool configured =
      sensor.SetCalibration(kCalibration) &&
      sensor.SetValidInterval(-40.0f, 125.0f) &&
      sensor.OnUpdate(HandleUpdate, nullptr, "Serial observer");
  if (!configured || !sensor.Begin()) {
    Serial.println("Sensor setup failed");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  sensor.Update(millis());
}
```

## Types, state, and units

| Type/member | Meaning |
| --- | --- |
| `Reading::temperatureC` | Last accepted calibrated temperature, in Celsius. |
| `Reading::sampledAtMs` | Acquisition timestamp supplied with that accepted sample. |
| `State::reading` | The last accepted measurement, including its timestamp. |
| `State::hasReading` | Whether any measurement has previously been accepted. |
| `State::status` | Latest acquisition result, or `NoData` after reconfiguration. |
| `CalibrationPoint::readTemperature` | Uncalibrated sensor temperature, in Celsius. |
| `CalibrationPoint::realTemperature` | Corresponding reference temperature, in Celsius. |

`GetState()` returns a copy. Modifying that copy cannot change the sensor.

| Status | Numeric value | Meaning |
| --- | ---: | --- |
| `NoData` | 0 | No acquisition result under the current configuration. |
| `Ok` | 1 | The latest submitted measurement was accepted. |
| `Disconnected` | 2 | The driver reported a disconnected device. |
| `Timeout` | 3 | The driver reported an acquisition timeout. |
| `InvalidReading` | 4 | Invalid input or an invalid/unrepresentable calibration result. |
| `HardwareError` | 5 | The driver reported a hardware problem. |
| `ReadingOutOfRange` | 6 | Calibrated temperature outside the configured interval. |
| `DeviceSpecificError1` | 7 | Driver-defined error category 1. |
| `DeviceSpecificError2` | 8 | Driver-defined error category 2. |
| `DeviceSpecificError3` | 9 | Driver-defined error category 3. |

The explicit enum values are preserved. Derived driver documentation must define
the meanings of its device-specific categories.

## Driver lifecycle and publication

The class is abstract. A concrete driver implements `Begin()` and
`Update(uint32_t now_ms)`; the sketch calls them. Successful initialization does
not imply a reading exists. Scheduling, conversion delays, communication checks,
device error sentinels, and hardware timeouts belong in the driver.

A driver submits **uncalibrated Celsius** with
`PublishTemperature(value_c, sampled_at_ms)`. The base:

1. Rejects nonfinite input.
2. Applies calibration exactly once.
3. Rejects invalid intermediate calculations or results outside finite `float`.
4. Checks the calibrated value against the optional inclusive interval.
5. Stores accepted data, sets `hasReading = true` and `status = Ok`, and notifies.

Check device-specific error codes before calibration can change their values.
Publishing returns `false` on rejection. A rejected attempt preserves the last
accepted temperature, timestamp, and `hasReading`; rejected values are not stored.

`ReportError(error)` preserves the accepted sample and changes the status.
It returns `true` for a newly reported or already active error, but identical
repeated errors do not log or notify again. It returns `false` for `Ok`,
`NoData`, or recursive reporting during notification. Use the declared error
statuses. A new accepted reading restores `Ok` and always notifies, even if its
temperature equals the previous value.

## Freshness and historical readings

`hasReading` means historical data exists; it does not promise current usability.
`IsUsable(now_ms, max_age_ms)` requires an accepted reading, current status `Ok`,
and an unsigned elapsed age **at most** `max_age_ms`.

Staleness alone does not change `status` or trigger callbacks. Error statuses and
effective configuration changes also make `IsUsable()` false.

Publication timestamps and freshness checks must use the same local clock.
Unsigned subtraction handles normal `millis()` rollover. Actual elapsed age must
be less than one complete 32-bit millisecond cycle, approximately 49.7 days;
a timestamp cannot distinguish additional complete cycles.

## Calibration

`SetCalibration(array)` deduces the array length.
`SetCalibration(pointer, count)` supports an explicit count.

| Point count | Mapping |
| --- | --- |
| 0 | Identity mapping; calibration disabled. |
| 1 | Apply the point's offset to every input. |
| 2 or more | Piecewise linear interpolation between adjacent points. |

Exact input knots return their specified reference temperatures. Outside the
table, extend the nearest end segment's slope; do not clamp to endpoint values.
For example, `{0, 0}, {10, 20}` maps input 5 to 10, -5 to -10, and 15 to 30.

For a nonempty table, the pointer must identify the given number of valid entries.
All coordinates must be finite, with strictly increasing, unique read temperatures.
Reference temperatures may increase, decrease, or remain equal. Invalid settings
return `false` without changing the active configuration or state.

The table is **borrowed**, not copied. Keep it alive and unchanged until cleared
or replaced. Do not edit an active array in place. A static const array in ordinary
addressable memory is convenient; an AVR PROGMEM-only array cannot be used directly.
There is no library-imposed point-count limit or dynamic allocation.

`ClearCalibration()` and `SetCalibration(nullptr, 0)` disable calibration.
With zero points the pointer is ignored. More points permit local correction;
their usefulness depends on reliable reference measurements.

## Optional valid interval

`SetValidInterval(minimum_c, maximum_c)` accepts finite endpoints with
`minimum_c <= maximum_c`. Both endpoints are included, and a single-value interval
is allowed. The bounds apply **after calibration**.

A finite calibrated value outside the interval reports `ReadingOutOfRange`,
preserves the accepted sample, and makes `IsUsable()` false.
`ClearValidInterval()` disables the check. Invalid bounds return `false` without
changing settings or state. These limits express an installation's trust policy.

## Configuration changes

An effective calibration or interval change retains historical data and its
timestamp, sets `NoData`, and does not invoke callbacks. A new acquisition attempt
establishes the next status; a fresh accepted sample is needed for usability.
Historical data is not recalibrated or assigned a new timestamp.

The same calibration **pointer and count**, the same enabled interval, or clearing
an already disabled feature is a successful no-op that preserves current state.
Configuration changes, including otherwise harmless no-ops, are rejected during
notification.

## Callbacks

```cpp
using Callback = void (*)(const TemperatureBase& sensor,
                          const TemperatureBase::State& state,
                          void* context);
```

`OnUpdate(callback, context, callback_name)` registers a free/static function or
compatible noncapturing lambda. Context and diagnostic name are optional.

- Four registrations maximum; duplicates occupy separate slots.
- Null callbacks and a full list return `false`; registration does not notify.
- `RemoveUpdateCallback(callback, context)` removes the first matching pair.
- `ClearCallbacks()` removes all registrations.

Notifications run synchronously, in registration order, inside publication/error
reporting. Every notification snapshots its state and subscriber list before
logging or calling application code. Subscription changes affect future events;
a removed registration can still run in the current snapshot.

The callback's state reference is temporary: copy it to retain data. Keep contexts
and names alive through any current notification, even after removal.
Nested publication/error reporting and configuration changes are rejected.
The guard does not provide thread safety or interrupt synchronization.

## Public and protected API

| Public member | Purpose |
| --- | --- |
| `TemperatureBase(ILogger* = nullptr, const char* = nullptr)` | Optional borrowed logger and name; constructor is explicit. |
| `virtual ~TemperatureBase()` | Safe destruction through a base pointer. |
| `virtual bool Begin() = 0` | Concrete driver initialization. |
| `virtual void Update(uint32_t) = 0` | Concrete driver acquisition/scheduling. |
| `GetName() const` | Name pointer, or `"missing name"` for null/empty names. |
| `GetState() const` | Current state by value. |
| `IsUsable(now_ms, max_age_ms) const` | Healthy, available, sufficiently recent reading. |
| `SetCalibration(points[, count])` | Configure a borrowed reference table; returns bool. |
| `ClearCalibration()` | Disable calibration; returns bool. |
| `SetValidInterval(minimum_c, maximum_c)` | Configure inclusive bounds; returns bool. |
| `ClearValidInterval()` | Disable bounds; returns bool. |
| `OnUpdate(callback, context = nullptr, name = nullptr)` | Register observer; returns bool. |
| `RemoveUpdateCallback(callback, context = nullptr)` | Remove first match; returns bool. |
| `ClearCallbacks()` | Remove all registrations. |

| Protected member | Purpose |
| --- | --- |
| `PublishTemperature(float, uint32_t)` | Submit uncalibrated Celsius and timestamp; bool acceptance. |
| `ReportError(Status)` | Report errors while preserving accepted data; bool acceptance. |
| `Log(LogLevel, const char*) const` | Send a message through the optional logger. |

Copy construction and assignment are deleted; move operations are not supplied.

## Ownership and logging

The base owns none of the logger, sensor-name text, calibration points, callback
contexts, or callback-name text. Their lifetimes must cover every use. The sensor
must also remain alive throughout its API calls and callbacks.

With a logger, construction logs an info event, accepted readings and callback
activity log debug events, and changed errors log descriptive error events.
The source includes `TemperatureBase` and the sensor name. Source names are limited
to 50 characters in logs; callback labels to 40. Filtering/output belong to the
logger implementation. No logger means no output.

Temperature logs round to two decimal places without changing stored values.
Values outside the formatter's +/-1,000,000 range show `outside log range`;
this is a presentation limit, not a sensor validity rule.
Formatting uses fixed buffers and avoids floating-point `printf` support.
`ILogger` must consume or copy log strings during its call.

## Scope, metadata, and style

The base has no physical driver, automatic hardware timeout, calibration storage,
or separate rejected-value history. Applications/derived drivers supply those.
See [STYLE_GUIDE.md](STYLE_GUIDE.md) for the Google C++ style review, applied fixes,
and compatibility exceptions. `keywords.txt` provides Arduino keyword categories.

`library.properties` describes the features, dependencies, and architecture
support. The version is unchanged because this update is not a release operation.
The project URL is currently empty; supply the actual project page before publishing.

## Tests

From this library directory, with sibling `ILogger` installed, GCC/MinGW:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Isrc -I../ILogger/src \
  src/TemperatureBase.cpp tests/TemperatureBaseTests.cpp -o TemperatureBaseTests
./TemperatureBaseTests
```

For integration tests, additionally include/build the sibling `TemperatureFake`:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  -Isrc -I../ILogger/src -I../TemperatureFake/src \
  src/TemperatureBase.cpp ../TemperatureFake/src/TemperatureFake.cpp \
  tests/TemperatureFakeTests.cpp -o TemperatureFakeTests
./TemperatureFakeTests
```

In an MSVC Developer PowerShell, use `cl /EHsc /W4 /WX /std:c++14`, the same source
files, and `/I` instead of `-I`. MSVC has no C++11 standard-mode switch.
Tests cover calibration, interval boundaries, arithmetic errors, atomic settings,
callback/error semantics, fake scheduling, rollover, and recovery. The separate
`TemperatureFakeTest.ino` demonstrates the features on Arduino over serial.
