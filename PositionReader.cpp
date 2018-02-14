#include "stdafx.h"
#include <iostream>

namespace je_nourish_fusion {

	IPositionReader* PositionReaderFactory::getReader(OSVR_ClientContext ctx, Json::Value config) {
		IPositionReader* reader = NULL;

		if (config.isString()) {
			reader = new SinglePositionReader(ctx, config.asString());
		}
		if (config.isObject() && config.isMember("x") && config.isMember("y") && config.isMember("z")) {
			reader = new CombinedPositionReader(ctx, config);
		}

		return reader;
	}

	SinglePositionReader::SinglePositionReader(OSVR_ClientContext ctx, std::string position_path) {
		osvrClientGetInterface(ctx, position_path.c_str(), &m_positionInterface);
	}
	OSVR_ReturnCode SinglePositionReader::update(OSVR_PoseState& pose, OSVR_VelocityState& vel, OSVR_AccelerationState& acc, OSVR_TimeValue* timeValue) {
		OSVR_ReturnCode pret = osvrGetPositionState(m_positionInterface, timeValue, &pose.translation);
		OSVR_ReturnCode vret = osvrGetLinearVelocityState(m_positionInterface, timeValue, &vel.linearVelocity);
		OSVR_ReturnCode aret = osvrGetLinearAccelerationState(m_positionInterface, timeValue, &acc.linearAcceleration);
		return (OSVR_ReturnCode)(pret && vret && aret);
	}

	CombinedPositionReader::CombinedPositionReader(OSVR_ClientContext ctx, Json::Value position_paths) {
		osvrClientGetInterface(ctx, position_paths["x"].asCString(), &(m_positionInterfaces[0]));
		osvrClientGetInterface(ctx, position_paths["y"].asCString(), &(m_positionInterfaces[1]));
		osvrClientGetInterface(ctx, position_paths["z"].asCString(), &(m_positionInterfaces[2]));
	}
	OSVR_ReturnCode CombinedPositionReader::update(OSVR_PoseState& pose, OSVR_VelocityState& vel, OSVR_AccelerationState& acc, OSVR_TimeValue* timeValue) {
		OSVR_PositionState position_x;
		OSVR_PositionState position_y;
		OSVR_PositionState position_z;

		OSVR_ReturnCode xret = osvrGetPositionState(m_positionInterfaces[0], timeValue, &position_x);
		OSVR_ReturnCode yret = osvrGetPositionState(m_positionInterfaces[1], timeValue, &position_y);
		OSVR_ReturnCode zret = osvrGetPositionState(m_positionInterfaces[2], timeValue, &position_z);

		if (xret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetX(&pose.translation, osvrVec3GetX(&position_x));
		}
		if (yret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetY(&pose.translation, osvrVec3GetY(&position_y));
		}
		if (zret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetZ(&pose.translation, osvrVec3GetZ(&position_z));
		}

		// velocity

		OSVR_VelocityState vel_x;
		OSVR_VelocityState vel_y;
		OSVR_VelocityState vel_z;

		xret = osvrGetVelocityState(m_positionInterfaces[0], timeValue, &vel_x);
		yret = osvrGetVelocityState(m_positionInterfaces[1], timeValue, &vel_y);
		zret = osvrGetVelocityState(m_positionInterfaces[2], timeValue, &vel_z);

		if (xret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetX(&vel.linearVelocity, osvrVec3GetX(&vel_x.linearVelocity));
		}
		if (yret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetY(&vel.linearVelocity, osvrVec3GetY(&vel_y.linearVelocity));
		}
		if (zret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetZ(&vel.linearVelocity, osvrVec3GetZ(&vel_z.linearVelocity));
		}

		if (vel_x.linearVelocityValid && vel_y.linearVelocityValid && vel_z.linearVelocityValid) {
			vel.linearVelocityValid = true;
		}
		else
			vel.linearVelocityValid = false;

		// acceleration

		OSVR_AccelerationState acc_x;
		OSVR_AccelerationState acc_y;
		OSVR_AccelerationState acc_z;

		xret = osvrGetAccelerationState(m_positionInterfaces[0], timeValue, &acc_x);
		yret = osvrGetAccelerationState(m_positionInterfaces[1], timeValue, &acc_y);
		zret = osvrGetAccelerationState(m_positionInterfaces[2], timeValue, &acc_z);

		if (xret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetX(&acc.linearAcceleration, osvrVec3GetX(&acc_x.linearAcceleration));
		}
		if (yret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetY(&acc.linearAcceleration, osvrVec3GetY(&acc_y.linearAcceleration));
		}
		if (zret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetZ(&acc.linearAcceleration, osvrVec3GetZ(&acc_z.linearAcceleration));
		}

		if (acc_x.linearAccelerationValid && acc_y.linearAccelerationValid && acc_z.linearAccelerationValid) {
			acc.linearAccelerationValid = true;
		}
		else
			acc.linearAccelerationValid = false;

		return OSVR_RETURN_SUCCESS;
	}

}