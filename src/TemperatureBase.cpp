#include "TemperatureBase.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

constexpr uint8_t TemperatureBase::kMaxCallbacks;

TemperatureBase::TemperatureBase(ILogger* logger, const char* name)
    : logger_(logger),
      name_(name),
      state_{{NAN, 0}, false, Status::NoData},
      calibration_points_(nullptr),
      calibration_point_count_(0),
      has_valid_interval_(false),
      minimum_temperature_c_(0.0f),
      maximum_temperature_c_(0.0f),
      callbacks_{},
      callback_count_(0),
      notifying_(false) {
  Log(LogLevel::Info, "Temperature sensor created");
}

TemperatureBase::~TemperatureBase() = default;

const char* TemperatureBase::GetName() const {
  return name_ && name_[0] ? name_ : "missing name";
}

TemperatureBase::State TemperatureBase::GetState() const { return state_; }

bool TemperatureBase::IsUsable(uint32_t now_ms, uint32_t max_age_ms) const {
  return state_.hasReading && state_.status == Status::Ok &&
         static_cast<uint32_t>(now_ms - state_.reading.sampledAtMs) <=
             max_age_ms;
}

bool TemperatureBase::SetCalibration(const CalibrationPoint* points,
                                     size_t count) {
  if (notifying_ || (count != 0 && !points)) {
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    if (!isfinite(points[i].readTemperature) ||
        !isfinite(points[i].realTemperature) ||
        (i != 0 &&
         points[i].readTemperature <= points[i - 1].readTemperature)) {
      return false;
    }
  }

  const CalibrationPoint* table = count == 0 ? nullptr : points;
  if (calibration_points_ == table && calibration_point_count_ == count) {
    return true;
  }

  calibration_points_ = table;
  calibration_point_count_ = count;
  InvalidateForConfigurationChange();
  return true;
}

bool TemperatureBase::ClearCalibration() { return SetCalibration(nullptr, 0); }

bool TemperatureBase::SetValidInterval(float minimum_c, float maximum_c) {
  if (notifying_ || !isfinite(minimum_c) || !isfinite(maximum_c) ||
      minimum_c > maximum_c) {
    return false;
  }

  if (has_valid_interval_ && minimum_temperature_c_ == minimum_c &&
      maximum_temperature_c_ == maximum_c) {
    return true;
  }

  minimum_temperature_c_ = minimum_c;
  maximum_temperature_c_ = maximum_c;
  has_valid_interval_ = true;
  InvalidateForConfigurationChange();
  return true;
}

bool TemperatureBase::ClearValidInterval() {
  if (notifying_) {
    return false;
  }
  if (!has_valid_interval_) {
    return true;
  }

  has_valid_interval_ = false;
  InvalidateForConfigurationChange();
  return true;
}

float TemperatureBase::Calibrate(float temperature_c) const {
  if (calibration_point_count_ == 0) {
    return temperature_c;
  }

  double calibratedC;
  if (calibration_point_count_ == 1) {
    const CalibrationPoint& point = calibration_points_[0];
    const double delta = static_cast<double>(temperature_c) -
                         static_cast<double>(point.readTemperature);
    if (!isfinite(delta)) {
      return NAN;
    }
    calibratedC = static_cast<double>(point.realTemperature) + delta;
  } else {
    // Select the enclosing segment, or the first/last for extrapolation.
    size_t upper = 1;
    while (upper < calibration_point_count_ - 1 &&
           temperature_c > calibration_points_[upper].readTemperature) {
      ++upper;
    }

    const CalibrationPoint& low = calibration_points_[upper - 1];
    const CalibrationPoint& high = calibration_points_[upper];

    // Return exact calibration points without interpolation roundoff.
    if (temperature_c == low.readTemperature) {
      return low.realTemperature;
    }
    if (temperature_c == high.readTemperature) {
      return high.realTemperature;
    }

    // Promote before arithmetic. On some targets double equals float;
    // check intermediates so overflow cannot look like a valid result.
    const double readSpan = static_cast<double>(high.readTemperature) -
                            static_cast<double>(low.readTemperature);
    const double realSpan = static_cast<double>(high.realTemperature) -
                            static_cast<double>(low.realTemperature);
    const double delta = static_cast<double>(temperature_c) -
                         static_cast<double>(low.readTemperature);
    if (!isfinite(readSpan) || !isfinite(realSpan) || !isfinite(delta)) {
      return NAN;
    }

    const double fraction = delta / readSpan;
    const double correction = fraction * realSpan;
    if (!isfinite(fraction) || !isfinite(correction)) {
      return NAN;
    }
    calibratedC = static_cast<double>(low.realTemperature) + correction;
  }

  // Check before narrowing: a finite double may not fit in a float.
  if (!isfinite(calibratedC) || calibratedC > static_cast<double>(FLT_MAX) ||
      calibratedC < -static_cast<double>(FLT_MAX)) {
    return NAN;
  }
  return static_cast<float>(calibratedC);
}

void TemperatureBase::InvalidateForConfigurationChange() {
  // Keep historical data and its acquisition timestamp. It has not been
  // accepted under the new configuration, so IsUsable must return false.
  state_.status = Status::NoData;
}

bool TemperatureBase::OnUpdate(Callback callback, void* context,
                               const char* callback_name) {
  if (!callback || callback_count_ == kMaxCallbacks) {
    Log(LogLevel::Error,
        "Cannot add callback: null or subscriber limit reached");
    return false;
  }

  callbacks_[callback_count_++] = {callback, context, callback_name};

  LogCallback("added", callback_name);
  return true;
}

bool TemperatureBase::RemoveUpdateCallback(Callback callback, void* context) {
  if (!callback) {
    return false;
  }

  for (uint8_t i = 0; i < callback_count_; ++i) {
    if (callbacks_[i].callback == callback &&
        callbacks_[i].context == context) {
      const char* callback_name = callbacks_[i].name;

      for (uint8_t j = i + 1; j < callback_count_; ++j) {
        callbacks_[j - 1] = callbacks_[j];
      }

      callbacks_[--callback_count_] = {nullptr, nullptr, nullptr};

      LogCallback("removed", callback_name);
      return true;
    }
  }

  return false;
}

void TemperatureBase::ClearCallbacks() {
  callback_count_ = 0;

  for (auto& entry : callbacks_) {
    entry = {nullptr, nullptr, nullptr};
  }

  Log(LogLevel::Debug, "Callbacks cleared");
}

bool TemperatureBase::PublishTemperature(float temperature_c,
                                         uint32_t sampled_at_ms) {
  if (notifying_) {
    return false;
  }

  if (!isfinite(temperature_c)) {
    ReportError(Status::InvalidReading);
    return false;
  }

  const float calibratedC = Calibrate(temperature_c);
  if (!isfinite(calibratedC)) {
    ReportError(Status::InvalidReading);
    return false;
  }

  if (has_valid_interval_ && (calibratedC < minimum_temperature_c_ ||
                              calibratedC > maximum_temperature_c_)) {
    ReportError(Status::ReadingOutOfRange);
    return false;
  }

  state_.reading = {calibratedC, sampled_at_ms};
  state_.hasReading = true;
  state_.status = Status::Ok;

  Notify();
  return true;
}

bool TemperatureBase::ReportError(Status error) {
  if (notifying_ || error == Status::Ok || error == Status::NoData) {
    return false;
  }

  if (state_.status == error) {
    return true;
  }

  state_.status = error;
  Notify();
  return true;
}

void TemperatureBase::Log(LogLevel level, const char* message) const {
  if (!logger_) {
    return;
  }

  char source[80];
  snprintf(source, sizeof(source), "TemperatureBase '%.50s'", GetName());

  logger_->Log(level, source, message);
}

const char* TemperatureBase::StatusText(Status status) {
  switch (status) {
    case Status::NoData:
      return "No temperature available";

    case Status::Ok:
      return "Temperature reading accepted";

    case Status::Disconnected:
      return "Sensor disconnected";

    case Status::Timeout:
      return "Measurement timed out";

    case Status::InvalidReading:
      return "Invalid temperature reading";

    case Status::HardwareError:
      return "Sensor hardware error";

    case Status::ReadingOutOfRange:
      return "Temperature outside configured valid interval";

    case Status::DeviceSpecificError1:
      return "Device-specific temperature error 1";

    case Status::DeviceSpecificError2:
      return "Device-specific temperature error 2";

    case Status::DeviceSpecificError3:
      return "Device-specific temperature error 3";

    default:
      return "Unknown temperature status";
  }
}

// Avoid heap allocation and printf floating-point support.
// Rounding affects log presentation only.
void TemperatureBase::FormatTemperature(float value, char* text, size_t size) {
  // Protect the conversion; this is not a sensor validity limit.
  if (fabsf(value) > 1000000.0f) {
    snprintf(text, size, "%s", "outside log range");
    return;
  }

  const long scaled = lroundf(value * 100.0f);

  const unsigned long magnitude =
      static_cast<unsigned long>(scaled < 0 ? -scaled : scaled);

  snprintf(text, size, "%s%lu.%02lu", scaled < 0 ? "-" : "", magnitude / 100,
           magnitude % 100);
}

void TemperatureBase::LogReading(const Reading& reading) const {
  if (!logger_) {
    return;
  }

  char temperature[24];
  char message[80];

  FormatTemperature(reading.temperatureC, temperature, sizeof(temperature));

  snprintf(message, sizeof(message), "Temperature=%s C, sampledAt=%lu ms",
           temperature, static_cast<unsigned long>(reading.sampledAtMs));

  Log(LogLevel::Debug, message);
}

void TemperatureBase::LogCallback(const char* action,
                                  const char* callback_name) const {
  if (!logger_) {
    return;
  }

  char message[80];
  snprintf(message, sizeof(message), "Callback '%.40s' %s",
           callback_name && callback_name[0] ? callback_name : "unnamed",
           action);

  Log(LogLevel::Debug, message);
}

void TemperatureBase::Notify() {
  // Reject recursive publication during logging or callbacks.
  notifying_ = true;

  const State snapshot = state_;
  const uint8_t count = callback_count_;

  Registration subscribers[kMaxCallbacks];
  for (uint8_t i = 0; i < count; ++i) {
    subscribers[i] = callbacks_[i];
  }

  if (snapshot.status == Status::Ok) {
    LogReading(snapshot.reading);
  } else {
    Log(LogLevel::Error, StatusText(snapshot.status));
  }

  for (uint8_t i = 0; i < count; ++i) {
    subscribers[i].callback(*this, snapshot, subscribers[i].context);

    LogCallback("invoked", subscribers[i].name);
  }

  notifying_ = false;
}
