// Minimal Input driver interface (MCU-backed stub)
#pragma once

#include <lwjson/lwjson.h>
#include <cstdint>

struct Input; // forward-declared; real definition lives in services

namespace xbot::driver::input {

class InputDriver {
 public:
  virtual ~InputDriver() = default;

  // Called when configuration is reloaded; driver should remove any existing inputs
  virtual void ClearInputs() = 0;

  // Driver should record the pointer to input for future updates
  virtual void AddInput(Input* input) = 0;

  // Called during JSON parsing for driver-specific attributes. Return true on success.
  virtual bool OnInputConfigValue(lwjson_stream_parser_t* jsp, const char* key, lwjson_stream_type_t type, Input& input) = 0;

  // Lifecycle
  virtual bool OnStart() = 0;
  virtual void OnStop() = 0;
};

} // namespace xbot::driver::input
