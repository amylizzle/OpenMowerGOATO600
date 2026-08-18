#include "emergency_driver.hpp"

namespace xbot::driver::emergency {

class McuEmergencyDriver : public EmergencyDriver {
 public:
	McuEmergencyDriver() = default;
	~McuEmergencyDriver() = default;

	bool Start() override { return true; }

	void Stop() override {}

	bool IsPresent() const override { return false; }

	void UpdateEmergency(uint16_t /*add*/, uint16_t /*clear*/) override {}
};

}  // namespace xbot::driver::emergency
