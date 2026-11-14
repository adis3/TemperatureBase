#ifndef TEMPERATURE_BASE_H
#define TEMPERATURE_BASE_H

#include <ILogger.h>
#include <stddef.h>
#include <stdint.h>

// Shared state and publication pipeline for concrete temperature sensor
// drivers.
class TemperatureBase {
 public:
  enum class Status : uint8_t {
    NoData = 0,
    Ok = 1,
    Disconnected = 2,
    Timeout = 3,
    InvalidReading = 4,
    HardwareError = 5,
    ReadingOutOfRange = 6,
    DeviceSpecificError1 = 7,
    DeviceSpecificError2 = 8,
    DeviceSpecificError3 = 9
  };

  struct Reading {
    float temperatureC;
    uint32_t sampledAtMs;
  };

  struct CalibrationPoint {
    float readTemperature;
    float realTemperature;
  };

  struct State {
    // Last successful reading, preserved after errors.
    Reading reading;
    // True after any accepted reading, including historical data on error.
    bool hasReading;

    // Status of the latest acquisition attempt. Configuration changes
    // set NoData until a fresh reading is accepted or an error reported.
    Status status;
  };

  using Callback = void (*)(const TemperatureBase& sensor, const State& state,
                            void* context);

  // Optional, non-owning references; must outlive their use.
  explicit TemperatureBase(ILogger* logger = nullptr,
                           const char* name = nullptr);

  virtual ~TemperatureBase();

  TemperatureBase(const TemperatureBase&) = delete;
  TemperatureBase& operator=(const TemperatureBase&) = delete;

  // Successful initialization does not imply a reading is available.
  // Implementations should report initialization failures.
  virtual bool Begin() = 0;

  // Called regularly from loop(); implementations manage scheduling.
  virtual void Update(uint32_t now_ms) = 0;

  const char* GetName() const;
  State GetState() const;

  // Requires healthy status and a sufficiently recent reading.
  // Uses the local millis() clock and unsigned elapsed-time arithmetic.
  bool IsUsable(uint32_t now_ms, uint32_t max_age_ms) const;

  // Optional, non-owning calibration table in Celsius. The array must
  // remain alive and unchanged until replaced or cleared. Read temperatures
  // must be strictly increasing; all temperatures must be finite.
  // Zero points disable calibration; one point applies an offset.
  // Multiple points use piecewise linear interpolation and extend the
  // nearest segment's slope outside the table.
  // Invalid settings leave the previous configuration and state unchanged.
  // Effective changes preserve the last accepted reading but set NoData,
  // without notifying, until a new acquisition attempt. Changes during
  // notification are rejected.
  bool SetCalibration(const CalibrationPoint* points, size_t count);

  template <size_t kCount>
  bool SetCalibration(const CalibrationPoint (&points)[kCount]) {
    return SetCalibration(points, kCount);
  }

  bool ClearCalibration();

  // Optional inclusive interval, applied to calibrated Celsius values.
  // Bounds must be finite and minimum_c <= maximum_c. A rejected reading
  // reports ReadingOutOfRange and preserves the last accepted reading.
  // Configuration changes follow the same rules as SetCalibration().
  bool SetValidInterval(float minimum_c, float maximum_c);
  bool ClearValidInterval();

  // Notifies for each accepted reading and changes to an error status.
  // No immediate notification. Duplicate registrations are allowed.
  //
  // Each notification uses a callback snapshot. Subscription changes
  // affect later notifications. Names and contexts must remain alive
  // until the current notification finishes.
  bool OnUpdate(Callback callback, void* context = nullptr,
                const char* callback_name = nullptr);

  // Removes the first matching callback/context pair.
  bool RemoveUpdateCallback(Callback callback, void* context = nullptr);

  void ClearCallbacks();

 protected:
  // Timestamp uses the same local millis() clock as IsUsable().
  // Submit uncalibrated Celsius: calibration is applied exactly once here,
  // before the optional valid interval. Device-specific validation and
  // recognition of device error values belong in the derived class.
  bool PublishTemperature(float temperature_c, uint32_t sampled_at_ms);

  // Preserves the last good reading.
  // Repeated identical errors are suppressed.
  bool ReportError(Status error);

  void Log(LogLevel level, const char* message) const;

 private:
  static constexpr uint8_t kMaxCallbacks = 4;

  struct Registration {
    Callback callback;
    void* context;
    const char* name;
  };

  static const char* StatusText(Status status);
  static void FormatTemperature(float value, char* text, size_t size);

  float Calibrate(float temperatureC) const;
  void InvalidateForConfigurationChange();
  void LogReading(const Reading& reading) const;
  void LogCallback(const char* action, const char* callback_name) const;
  void Notify();

  ILogger* logger_;
  const char* name_;
  State state_;

  const CalibrationPoint* calibration_points_;
  size_t calibration_point_count_;
  bool has_valid_interval_;
  float minimum_temperature_c_;
  float maximum_temperature_c_;

  Registration callbacks_[kMaxCallbacks];
  uint8_t callback_count_;
  bool notifying_;
};

#endif  // TEMPERATURE_BASE_H
