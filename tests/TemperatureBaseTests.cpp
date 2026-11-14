#include <float.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "TemperatureBase.h"

using Status = TemperatureBase::Status;
using State = TemperatureBase::State;
using Point = TemperatureBase::CalibrationPoint;

#define CHECK(expression)                                                   \
  do {                                                                      \
    if (!(expression)) {                                                    \
      std::fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #expression); \
      return false;                                                         \
    }                                                                       \
  } while (false)

class FakeSensor : public TemperatureBase {
 public:
  explicit FakeSensor(ILogger* logger = nullptr) : TemperatureBase(logger) {}
  bool Begin() override { return true; }
  void Update(uint32_t) override {}
  bool Publish(float value, uint32_t time) {
    return PublishTemperature(value, time);
  }
  bool Report(Status status) { return ReportError(status); }
};

static bool Near(float a, float b) {
  return std::isfinite(a) && std::isfinite(b) &&
         std::fabs(a - b) <=
             0.0001f * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

static bool Good(const TemperatureBase& sensor, float value, uint32_t time) {
  const State state = sensor.GetState();
  return state.hasReading && state.status == Status::Ok &&
         Near(state.reading.temperatureC, value) &&
         state.reading.sampledAtMs == time;
}

static bool Preserved(const TemperatureBase& sensor, const State& before,
                      Status status) {
  const State state = sensor.GetState();
  return state.hasReading == before.hasReading && state.status == status &&
         Near(state.reading.temperatureC, before.reading.temperatureC) &&
         state.reading.sampledAtMs == before.reading.sampledAtMs;
}

static bool Defaults() {
  FakeSensor sensor;
  CHECK(!sensor.GetState().hasReading);
  CHECK(sensor.GetState().status == Status::NoData);
  CHECK(!sensor.IsUsable(0, 100));
  CHECK(sensor.ClearCalibration() && sensor.ClearValidInterval());
  CHECK(sensor.SetCalibration(nullptr, 0));
  CHECK(sensor.Publish(-300.0f, 10));
  CHECK(Good(sensor, -300.0f, 10));
  CHECK(sensor.IsUsable(110, 100));
  CHECK(!sensor.IsUsable(111, 100));
  return true;
}

static bool CalibrationCurves() {
  FakeSensor sensor;
  const Point one[] = {{10.0f, 12.5f}};
  CHECK(sensor.SetCalibration(one));
  CHECK(sensor.Publish(-4.0f, 1) && Good(sensor, -1.5f, 1));
  CHECK(sensor.Publish(10.0f, 2) && Good(sensor, 12.5f, 2));

  const Point two[] = {{10.0f, 20.0f}, {20.0f, 40.0f}};
  CHECK(sensor.SetCalibration(two));
  const Point twoCases[] = {{10, 20}, {20, 40}, {15, 30}, {0, 0}, {30, 60}};
  uint32_t time = 3;
  for (const Point& test : twoCases) {
    CHECK(sensor.Publish(test.readTemperature, time));
    CHECK(Good(sensor, test.realTemperature, time++));
  }

  const Point multiple[] = {{0, 10}, {10, 0}, {20, 40}, {30, 40}};
  CHECK(sensor.SetCalibration(multiple));
  const Point cases[] = {{0, 10},  {10, 0},  {20, 40},  {30, 40}, {5, 5},
                         {15, 20}, {25, 40}, {-10, 20}, {40, 40}};
  for (const Point& test : cases) {
    CHECK(sensor.Publish(test.readTemperature, time));
    CHECK(Good(sensor, test.realTemperature, time++));
  }
  CHECK(sensor.SetCalibration(nullptr, 0));
  CHECK(sensor.Publish(7, time) && Good(sensor, 7, time));
  return true;
}

static bool InvalidConfigurationIsAtomic() {
  FakeSensor sensor;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const Point valid[] = {{0, 10}, {10, 20}};
  const Point duplicate[] = {{0, 0}, {10, 1}, {10, 2}};
  const Point descending[] = {{0, 0}, {10, 1}, {5, 2}};
  const Point badRead[] = {{0, 0}, {nan, 1}};
  const Point badReal[] = {{0, 0}, {1, nan}};
  const Point infiniteRead[] = {{inf, 0}};
  const Point infiniteReal[] = {{0, inf}};
  const Point signedZero[] = {{-0.0f, 0}, {0.0f, 1}};
  struct BadTable {
    const Point* points;
    size_t count;
  };
  const BadTable bad[] = {{nullptr, 1},      {duplicate, 3}, {descending, 3},
                          {badRead, 2},      {badReal, 2},   {infiniteRead, 1},
                          {infiniteReal, 1}, {signedZero, 2}};
  CHECK(sensor.SetCalibration(valid) && sensor.SetValidInterval(0, 30));
  CHECK(sensor.Publish(5, 100));
  const State before = sensor.GetState();
  for (const BadTable& test : bad) {
    CHECK(!sensor.SetCalibration(test.points, test.count));
    CHECK(Preserved(sensor, before, Status::Ok));
    CHECK(sensor.IsUsable(100, 0));
  }
  const float bounds[][2] = {{2, 1}, {nan, 1}, {0, nan}, {-inf, 1}, {0, inf}};
  for (const auto& test : bounds) {
    CHECK(!sensor.SetValidInterval(test[0], test[1]));
    CHECK(Preserved(sensor, before, Status::Ok));
  }
  CHECK(sensor.Publish(6, 101) && Good(sensor, 16, 101));
  const State latest = sensor.GetState();
  CHECK(!sensor.Publish(21, 102));
  CHECK(Preserved(sensor, latest, Status::ReadingOutOfRange));
  CHECK(!sensor.IsUsable(102, 100));
  return true;
}

static bool CalibratedIntervals() {
  FakeSensor sensor;
  const Point offset[] = {{10, 20}};
  CHECK(sensor.SetCalibration(offset) && sensor.SetValidInterval(20, 30));
  CHECK(sensor.Publish(10, 1) && Good(sensor, 20, 1));
  CHECK(sensor.Publish(20, 2) && Good(sensor, 30, 2));
  CHECK(sensor.Publish(15, 3) && Good(sensor, 25, 3));
  const State before = sensor.GetState();
  CHECK(!sensor.Publish(9, 4));
  CHECK(Preserved(sensor, before, Status::ReadingOutOfRange));
  CHECK(!sensor.Publish(21, 5));
  CHECK(Preserved(sensor, before, Status::ReadingOutOfRange));

  CHECK(sensor.SetValidInterval(25, 25));
  CHECK(sensor.Publish(15, 6) && Good(sensor, 25, 6));
  const State exact = sensor.GetState();
  CHECK(!sensor.Publish(14, 7));
  CHECK(Preserved(sensor, exact, Status::ReadingOutOfRange));
  CHECK(!sensor.Publish(16, 8));
  CHECK(Preserved(sensor, exact, Status::ReadingOutOfRange));
  CHECK(sensor.ClearValidInterval());
  CHECK(sensor.Publish(100, 9) && Good(sensor, 110, 9));
  return true;
}

static bool NonfiniteReadingsAndOverflow() {
  FakeSensor sensor;
  CHECK(sensor.Publish(7, 1));
  State before = sensor.GetState();
  const Point extreme[] = {{0, 0}, {1, FLT_MAX}};
  CHECK(sensor.SetCalibration(extreme));
  CHECK(!sensor.Publish(2, 2));
  CHECK(Preserved(sensor, before, Status::InvalidReading));
  CHECK(sensor.Publish(0, 3) && Good(sensor, 0, 3));

  const Point tinySpan[] = {{0, 0}, {FLT_MIN, FLT_MAX}};
  CHECK(sensor.SetCalibration(tinySpan));
  CHECK(sensor.Publish(FLT_MIN, 4) && Good(sensor, FLT_MAX, 4));
  before = sensor.GetState();
  CHECK(!sensor.Publish(1, 5));
  CHECK(Preserved(sensor, before, Status::InvalidReading));
  const float values[] = {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity()};
  for (float value : values) {
    CHECK(!sensor.Publish(value, 6));
    CHECK(Preserved(sensor, before, Status::InvalidReading));
  }
  CHECK(sensor.Publish(0, 7) && Good(sensor, 0, 7));

  const Point one[] = {{0, FLT_MAX}};
  CHECK(sensor.SetCalibration(one));
  CHECK(sensor.Publish(0, 8) && Good(sensor, FLT_MAX, 8));
  before = sensor.GetState();
  CHECK(!sensor.Publish(FLT_MAX, 9));
  CHECK(Preserved(sensor, before, Status::InvalidReading));
  return true;
}

struct Events {
  int calls;
  Status last;
};
static void Count(const TemperatureBase&, const State& state, void* context) {
  Events& events = *static_cast<Events*>(context);
  ++events.calls;
  events.last = state.status;
}

static bool ConfigurationChangesAndNoops() {
  FakeSensor sensor;
  Events events = {0, Status::NoData};
  CHECK(sensor.OnUpdate(Count, &events));
  CHECK(sensor.Publish(2, 1));
  State before = sensor.GetState();
  CHECK(sensor.ClearCalibration() && sensor.ClearValidInterval());
  CHECK(sensor.SetCalibration(nullptr, 0));
  CHECK(Preserved(sensor, before, Status::Ok) && events.calls == 1);
  const Point offset[] = {{0, 1}};
  CHECK(sensor.SetCalibration(offset));
  CHECK(Preserved(sensor, before, Status::NoData) && events.calls == 1);
  CHECK(!sensor.IsUsable(1, 100));
  CHECK(sensor.Publish(2, 2) && Good(sensor, 3, 2));
  CHECK(sensor.SetCalibration(offset));
  CHECK(Good(sensor, 3, 2) && events.calls == 2);
  before = sensor.GetState();
  CHECK(sensor.SetValidInterval(0, 10));
  CHECK(Preserved(sensor, before, Status::NoData) && events.calls == 2);
  CHECK(!sensor.IsUsable(2, 100));
  CHECK(sensor.Publish(2, 3));
  CHECK(sensor.SetValidInterval(0, 10));
  CHECK(Good(sensor, 3, 3) && events.calls == 3);
  before = sensor.GetState();
  CHECK(sensor.ClearValidInterval());
  CHECK(Preserved(sensor, before, Status::NoData) && events.calls == 3);
  CHECK(sensor.Publish(2, 4));
  before = sensor.GetState();
  CHECK(sensor.ClearCalibration());
  CHECK(Preserved(sensor, before, Status::NoData) && events.calls == 4);
  CHECK(sensor.Publish(2, 5) && Good(sensor, 2, 5));
  const Point ignored[] = {{std::numeric_limits<float>::quiet_NaN(), 0}};
  CHECK(sensor.SetCalibration(ignored, 0));
  CHECK(sensor.ClearCalibration() && sensor.ClearValidInterval());
  CHECK(Good(sensor, 2, 5) && events.calls == 5);
  return true;
}

static bool ErrorNotificationsAndRecovery() {
  FakeSensor sensor;
  Events events = {0, Status::NoData};
  CHECK(sensor.OnUpdate(Count, &events) && sensor.Publish(10, 1));
  const State before = sensor.GetState();
  CHECK(!sensor.Publish(std::numeric_limits<float>::quiet_NaN(), 2));
  CHECK(Preserved(sensor, before, Status::InvalidReading) && events.calls == 2);
  CHECK(!sensor.Publish(std::numeric_limits<float>::infinity(), 3));
  CHECK(events.calls == 2 && events.last == Status::InvalidReading);
  CHECK(sensor.Report(Status::Timeout) && sensor.Report(Status::Timeout));
  CHECK(events.calls == 3 && events.last == Status::Timeout);
  CHECK(Preserved(sensor, before, Status::Timeout));
  CHECK(sensor.Publish(11, 4) && Good(sensor, 11, 4) && events.calls == 4);
  CHECK(sensor.SetValidInterval(0, 20) && events.calls == 4);
  CHECK(!sensor.Publish(21, 5) && events.calls == 5);
  CHECK(!sensor.Publish(22, 6) && events.calls == 5);
  CHECK(events.last == Status::ReadingOutOfRange);
  CHECK(sensor.Publish(20, 7) && Good(sensor, 20, 7) && events.calls == 6);
  return true;
}

struct Reentry {
  FakeSensor* sensor;
  const Point* calibration;
  int calls;
  bool rejected;
};
static void Reenter(const TemperatureBase&, const State&, void* context) {
  Reentry& entry = *static_cast<Reentry*>(context);
  FakeSensor& sensor = *entry.sensor;
  const bool a = sensor.SetCalibration(entry.calibration, 1);
  const bool b = sensor.ClearCalibration();
  const bool c = sensor.SetCalibration(nullptr, 0);
  const bool d = sensor.SetValidInterval(0, 100);
  const bool e = sensor.SetValidInterval(-100, 100);
  const bool f = sensor.ClearValidInterval();
  entry.rejected = entry.rejected && !(a || b || c || d || e || f);
  ++entry.calls;
}

static bool ReentrantSettersAreRejected() {
  FakeSensor sensor;
  const Point offset[] = {{0, 10}};
  CHECK(sensor.SetCalibration(offset) && sensor.SetValidInterval(0, 100));
  Reentry entry = {&sensor, offset, 0, true};
  CHECK(sensor.OnUpdate(Reenter, &entry));
  CHECK(sensor.Publish(5, 1) && Good(sensor, 15, 1));
  CHECK(sensor.Publish(6, 2) && Good(sensor, 16, 2));
  const State before = sensor.GetState();
  CHECK(!sensor.Publish(100, 3));
  CHECK(Preserved(sensor, before, Status::ReadingOutOfRange));
  CHECK(entry.calls == 3 && entry.rejected);
  return true;
}

class CapturingLogger : public ILogger {
 public:
  bool saw_range_error = false;
  void Log(LogLevel level, const char*, const char* message) override {
    if (level == LogLevel::Error &&
        std::strstr(message, "configured valid interval") != nullptr) {
      saw_range_error = true;
    }
  }
};

static bool RejectionBeforeFirstReading() {
  CapturingLogger logger;
  FakeSensor sensor(&logger);
  Events events = {0, Status::NoData};
  CHECK(sensor.OnUpdate(Count, &events));
  CHECK(sensor.SetValidInterval(10, 40));
  CHECK(!sensor.Publish(9, 100));
  State state = sensor.GetState();
  CHECK(!state.hasReading && std::isnan(state.reading.temperatureC));
  CHECK(state.reading.sampledAtMs == 0);
  CHECK(state.status == Status::ReadingOutOfRange && logger.saw_range_error);
  CHECK(events.calls == 1 && !sensor.IsUsable(100, 1000));
  CHECK(!sensor.Publish(std::numeric_limits<float>::quiet_NaN(), 101));
  state = sensor.GetState();
  CHECK(!state.hasReading && std::isnan(state.reading.temperatureC));
  CHECK(state.status == Status::InvalidReading && events.calls == 2);
  CHECK(sensor.Publish(10, 102) && Good(sensor, 10, 102));
  return true;
}

int main() {
  struct Test {
    const char* name;
    bool (*run)();
  };
  const Test tests[] = {
      {"defaults", Defaults},
      {"calibration curves", CalibrationCurves},
      {"atomic invalid configuration", InvalidConfigurationIsAtomic},
      {"calibrated intervals", CalibratedIntervals},
      {"nonfinite readings and overflow", NonfiniteReadingsAndOverflow},
      {"configuration changes and noops", ConfigurationChangesAndNoops},
      {"error notifications and recovery", ErrorNotificationsAndRecovery},
      {"reentrant setters", ReentrantSettersAreRejected},
      {"rejection before first reading", RejectionBeforeFirstReading}};
  for (const Test& test : tests) {
    if (!test.run()) {
      std::fprintf(stderr, "FAILED: %s\n", test.name);
      return 1;
    }
  }
  std::printf("%u tests passed\n",
              static_cast<unsigned>(sizeof(tests) / sizeof(tests[0])));
  return 0;
}
