#pragma once

#include <cstdint>
#include <string>

namespace rcomponent
{	

	struct State {
		uint8_t id;
		std::string label;
	};

	enum OperationCommand : uint8_t {
			NONE = 0,
			START = 1,
			STOP = 2,
			PAUSE = 3
	};

	enum CommunicationState : uint8_t {
			COMMUNICATION_STATE_UNKNOWN = 0,
			COMMUNICATION_STATE_HEALTHY = 1,
			COMMUNICATION_STATE_UNHEALTHY = 2,
			COMMUNICATION_STATE_ERROR = 3
	};

}