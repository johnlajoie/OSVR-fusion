#include "stdafx.h"

namespace je_nourish_fusion {

	class IPositionReader {
	public:
		virtual OSVR_ReturnCode update(OSVR_PoseState& pose, OSVR_VelocityState& vel, OSVR_AccelerationState& acc, OSVR_TimeValue* timeValue) = 0;
	};

	class PositionReaderFactory {
	public:
		static IPositionReader* getReader(OSVR_ClientContext ctx, Json::Value config);
	};

	class SinglePositionReader : public IPositionReader {
	public:
		SinglePositionReader(OSVR_ClientContext ctx, std::string position_path);
		OSVR_ReturnCode update(OSVR_PoseState& pose, OSVR_VelocityState& vel, OSVR_AccelerationState& acc, OSVR_TimeValue* timeValue);
	protected:
		OSVR_ClientInterface m_positionInterface;
	};

	class CombinedPositionReader : public IPositionReader {
	public:
		CombinedPositionReader(OSVR_ClientContext ctx, Json::Value position_paths);
		OSVR_ReturnCode update(OSVR_PoseState& pose, OSVR_VelocityState& vel, OSVR_AccelerationState& acc, OSVR_TimeValue* timeValue);
	protected:
		OSVR_ClientInterface m_positionInterfaces[3];
	};

}