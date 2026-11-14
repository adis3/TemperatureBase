#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "TemperatureFake.h"

namespace {

using Status = TemperatureBase::Status;

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", file, line, expression);
    std::exit(EXIT_FAILURE);
  }
}

#define CHECK(condition) Check((condition), #condition, __FILE__, __LINE__)

struct CallbackContext {
  int calls = 0;
  TemperatureBase::State last_state{};
};

void OnState(const TemperatureBase&, const TemperatureBase::State& state,
             void* context) {
  auto* observer = static_cast<CallbackContext*>(context);
  ++observer->calls;
  observer->last_state = state;
}

void CheckTemperature(const TemperatureFake& sensor, float expected) {
  CHECK(std::fabs(sensor.GetState().reading.temperatureC - expected) < 0.001f);
}

void TestSchedulingAndInitialization() {
  TemperatureFake sensor(nullptr, nullptr, 10);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  sensor.Update(0);
  CHECK(!sensor.GetState().hasReading && observer.calls == 0);
  sensor.SetTemperature(20.0f);
  sensor.Update(1);
  CHECK(!sensor.GetState().hasReading && observer.calls == 0);
  CHECK(sensor.Begin());
  sensor.Update(100);
  CHECK(observer.calls == 1);
  CheckTemperature(sensor, 20.0f);
  sensor.SetTemperature(21.0f);
  sensor.Update(109);
  CHECK(observer.calls == 1);
  CheckTemperature(sensor, 20.0f);
  sensor.Update(110);
  CHECK(observer.calls == 2);
  CHECK(sensor.GetState().reading.sampledAtMs == 110);
  CheckTemperature(sensor, 21.0f);

  TemperatureFake no_input(nullptr, nullptr, 10);
  CHECK(no_input.Begin());
  no_input.Update(0);
  no_input.Update(100);
  CHECK(!no_input.GetState().hasReading);
  CHECK(no_input.GetState().status == Status::NoData);
  no_input.SetTemperature(22.0f);
  no_input.Update(101);
  CHECK(no_input.GetState().status == Status::Ok);
  CheckTemperature(no_input, 22.0f);
}

void TestCalibrationAndRange() {
  TemperatureFake sensor(nullptr, nullptr, 0);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  const TemperatureBase::CalibrationPoint points[] = {
      {10.0f, 12.0f},
      {20.0f, 24.0f},
  };
  CHECK(sensor.SetCalibration(points));
  CHECK(sensor.SetValidInterval(12.0f, 30.0f));
  CHECK(sensor.Begin());
  sensor.SetTemperature(10.0f);
  sensor.Update(100);
  CHECK(sensor.GetState().status == Status::Ok);
  CheckTemperature(sensor, 12.0f);
  sensor.SetTemperature(25.0f);
  sensor.Update(101);
  CHECK(sensor.GetState().status == Status::Ok);
  CheckTemperature(sensor, 30.0f);
  sensor.SetTemperature(26.0f);
  sensor.Update(102);
  CHECK(sensor.GetState().status == Status::ReadingOutOfRange);
  CHECK(sensor.GetState().hasReading);
  CHECK(sensor.GetState().reading.sampledAtMs == 101);
  CHECK(!sensor.IsUsable(102, 100));
  CheckTemperature(sensor, 30.0f);
  sensor.SetTemperature(std::numeric_limits<float>::quiet_NaN());
  sensor.Update(103);
  CHECK(sensor.GetState().status == Status::InvalidReading);
  CHECK(sensor.GetState().reading.sampledAtMs == 101);
  CheckTemperature(sensor, 30.0f);
  sensor.SetTemperature(15.0f);
  sensor.Update(104);
  CHECK(sensor.GetState().status == Status::Ok);
  CHECK(sensor.IsUsable(104, 100));
  CHECK(observer.calls == 5 && observer.last_state.status == Status::Ok);
  CheckTemperature(sensor, 18.0f);
}

void TestErrorsAndRecovery() {
  TemperatureFake sensor(nullptr, nullptr, 0);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  CHECK(sensor.Begin());
  sensor.SetTemperature(21.0f);
  CHECK(!sensor.SetError(Status::Ok));
  CHECK(!sensor.SetError(Status::NoData));
  CHECK(!sensor.SetError(static_cast<Status>(255)));
  sensor.Update(1);
  CHECK(sensor.GetState().status == Status::Ok);
  CheckTemperature(sensor, 21.0f);
  const Status errors[] = {
      Status::Disconnected,         Status::Timeout,
      Status::InvalidReading,       Status::HardwareError,
      Status::ReadingOutOfRange,    Status::DeviceSpecificError1,
      Status::DeviceSpecificError2, Status::DeviceSpecificError3,
  };
  uint32_t now_ms = 2;
  for (Status error : errors) {
    CHECK(sensor.SetError(error));
    sensor.Update(now_ms++);
    CHECK(sensor.GetState().status == error &&
          observer.last_state.status == error);
    CHECK(sensor.GetState().hasReading);
    CHECK(sensor.GetState().reading.sampledAtMs == 1);
    CheckTemperature(sensor, 21.0f);
  }
  CHECK(observer.calls == 9);
  CHECK(!sensor.SetError(Status::Ok));
  CHECK(!sensor.SetError(Status::NoData));
  CHECK(!sensor.SetError(static_cast<Status>(255)));
  sensor.Update(now_ms++);
  CHECK(sensor.GetState().status == Status::DeviceSpecificError3);
  CHECK(observer.calls == 9);
  CHECK(sensor.SetError(Status::DeviceSpecificError3));
  sensor.Update(now_ms++);
  CHECK(observer.calls == 9);
  sensor.SetTemperature(22.0f);
  sensor.Update(now_ms);
  CHECK(sensor.GetState().status == Status::Ok);
  CHECK(sensor.GetState().reading.sampledAtMs == now_ms &&
        observer.calls == 10);
  CheckTemperature(sensor, 22.0f);
}

void TestZeroInterval() {
  TemperatureFake sensor(nullptr, nullptr, 0);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  CHECK(sensor.Begin());
  sensor.SetTemperature(3.0f);
  sensor.Update(10);
  sensor.Update(10);
  CHECK(observer.calls == 2 && sensor.GetState().reading.sampledAtMs == 10);
}

void TestMillisRollover() {
  TemperatureFake sensor(nullptr, nullptr, 10);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  CHECK(sensor.Begin());
  sensor.SetTemperature(20.0f);
  sensor.Update(std::numeric_limits<uint32_t>::max() - 5);
  CHECK(observer.calls == 1);
  sensor.SetTemperature(21.0f);
  sensor.Update(3);
  CHECK(observer.calls == 1);
  CheckTemperature(sensor, 20.0f);
  sensor.Update(4);
  CHECK(observer.calls == 2 && sensor.GetState().reading.sampledAtMs == 4);
  CheckTemperature(sensor, 21.0f);
}

void TestBeginRestart() {
  TemperatureFake sensor(nullptr, nullptr, 100);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  const TemperatureBase::CalibrationPoint points[] = {{0.0f, 2.0f}};
  CHECK(sensor.SetCalibration(points));
  CHECK(sensor.SetValidInterval(10.0f, 30.0f));
  CHECK(sensor.Begin());
  sensor.SetTemperature(18.0f);
  sensor.Update(100);
  CHECK(observer.calls == 1);
  CheckTemperature(sensor, 20.0f);
  sensor.SetTemperature(19.0f);
  CHECK(sensor.Begin());
  CHECK(sensor.GetState().status == Status::Ok);
  CHECK(sensor.GetState().hasReading);
  CHECK(sensor.GetState().reading.sampledAtMs == 100 && observer.calls == 1);
  CheckTemperature(sensor, 20.0f);
  sensor.Update(101);
  CHECK(observer.calls == 2);
  CheckTemperature(sensor, 21.0f);
  sensor.SetTemperature(29.0f);
  sensor.Update(201);
  CHECK(sensor.GetState().status == Status::ReadingOutOfRange);
  CHECK(observer.calls == 3 && sensor.GetState().reading.sampledAtMs == 101);
  CheckTemperature(sensor, 21.0f);
  CHECK(sensor.SetError(Status::Timeout));
  CHECK(sensor.Begin());
  sensor.Update(202);
  CHECK(sensor.GetState().status == Status::Timeout && observer.calls == 4);
}

void TestConfigurationChangesRespectSchedule() {
  TemperatureFake sensor(nullptr, nullptr, 100);
  CallbackContext observer;
  CHECK(sensor.OnUpdate(OnState, &observer));
  CHECK(sensor.Begin());
  sensor.SetTemperature(20.0f);
  sensor.Update(10);
  CHECK(observer.calls == 1);
  const TemperatureBase::CalibrationPoint points[] = {{0.0f, 5.0f}};
  CHECK(sensor.SetCalibration(points));
  CHECK(sensor.GetState().status == Status::NoData);
  CHECK(sensor.GetState().hasReading);
  CHECK(sensor.GetState().reading.sampledAtMs == 10);
  CheckTemperature(sensor, 20.0f);
  sensor.Update(109);
  CHECK(observer.calls == 1 && sensor.GetState().status == Status::NoData);
  sensor.Update(110);
  CHECK(observer.calls == 2 && sensor.GetState().status == Status::Ok);
  CheckTemperature(sensor, 25.0f);
  CHECK(sensor.SetValidInterval(10.0f, 24.0f));
  sensor.Update(209);
  CHECK(observer.calls == 2 && sensor.GetState().status == Status::NoData);
  sensor.Update(210);
  CHECK(observer.calls == 3);
  CHECK(sensor.GetState().status == Status::ReadingOutOfRange);
  CHECK(sensor.GetState().reading.sampledAtMs == 110);
  CheckTemperature(sensor, 25.0f);
  CHECK(sensor.SetValidInterval(10.0f, 30.0f));
  sensor.Update(309);
  CHECK(observer.calls == 3);
  sensor.Update(310);
  CHECK(observer.calls == 4 && sensor.GetState().status == Status::Ok);
  CheckTemperature(sensor, 25.0f);
}

}  // namespace

int main() {
  TestSchedulingAndInitialization();
  TestCalibrationAndRange();
  TestErrorsAndRecovery();
  TestZeroInterval();
  TestMillisRollover();
  TestBeginRestart();
  TestConfigurationChangesRespectSchedule();
  std::puts("7 TemperatureFake test groups passed.");
  return EXIT_SUCCESS;
}
