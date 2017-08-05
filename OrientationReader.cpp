#include "stdafx.h"
#include <iostream>

namespace je_nourish_fusion {

	IOrientationReader* OrientationReaderFactory::getReader(OSVR_ClientContext ctx, Json::Value config) {
		IOrientationReader* reader = NULL;

		if (config.isString()) {
			reader = new SingleOrientationReader(ctx, config.asString());
		}
		else if (config.isObject() && config.isMember("roll") && config.isMember("pitch") && config.isMember("yawFast")
			&& config.isMember("yawAccurate") && config.isMember("alpha")) {
			reader = new FilteredOrientationReader(ctx, config);
		}
		else if (config.isObject() && config.isMember("roll") && config.isMember("pitch") && config.isMember("yaw")) {
			reader = new CombinedOrientationReader(ctx, config);
		}

		return reader;
	}

	SingleOrientationReader::SingleOrientationReader(OSVR_ClientContext ctx, std::string orientation_path) {
		osvrClientGetInterface(ctx, orientation_path.c_str(), &m_orientation);
	}
	OSVR_ReturnCode SingleOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		return osvrGetOrientationState(m_orientation, timeValue, orientation);
	}

	CombinedOrientationReader::CombinedOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths) {
		osvrClientGetInterface(ctx, orientation_paths["roll"].asCString(), &(m_orientations[0]));
		osvrClientGetInterface(ctx, orientation_paths["pitch"].asCString(), &(m_orientations[1]));
		osvrClientGetInterface(ctx, orientation_paths["yaw"].asCString(), &(m_orientations[2]));
	}

	FilteredOrientationReader::FilteredOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths) : m_ctx(ctx) {
		osvrClientGetInterface(ctx, orientation_paths["roll"].asCString(), &(m_orientations[0]));
		osvrClientGetInterface(ctx, orientation_paths["pitch"].asCString(), &(m_orientations[1]));
		osvrClientGetInterface(ctx, orientation_paths["yawFast"].asCString(), &(m_orientations[2]));
		osvrClientGetInterface(ctx, orientation_paths["yawAccurate"].asCString(), &(m_orientations[3]));
		m_alpha = orientation_paths["alpha"].asDouble();
		m_last_yaw = 0;

		m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "Initialized a complementary fusion filter.");
	}

	OSVR_ReturnCode CombinedOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		OSVR_OrientationState orientation_x;
		OSVR_OrientationState orientation_y;
		OSVR_OrientationState orientation_z;

		OSVR_ReturnCode xret = osvrGetOrientationState(m_orientations[0], timeValue, &orientation_x);
		OSVR_ReturnCode yret = osvrGetOrientationState(m_orientations[1], timeValue, &orientation_y);
		OSVR_ReturnCode zret = osvrGetOrientationState(m_orientations[2], timeValue, &orientation_z);


		OSVR_Vec3 rpy_x;
		OSVR_Vec3 rpy_y;
		OSVR_Vec3 rpy_z;

		rpyFromQuaternion(&orientation_x, &rpy_x);
		rpyFromQuaternion(&orientation_y, &rpy_y);
		rpyFromQuaternion(&orientation_z, &rpy_z);

		OSVR_Vec3 rpy;
		osvrVec3SetX(&rpy, osvrVec3GetX(&rpy_x));
		osvrVec3SetY(&rpy, osvrVec3GetY(&rpy_y));
		osvrVec3SetZ(&rpy, osvrVec3GetZ(&rpy_z));

		quaternionFromRPY(&rpy, orientation);

		return OSVR_RETURN_SUCCESS;
	}

	OSVR_ReturnCode FilteredOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		OSVR_OrientationState orientation_x;
		OSVR_OrientationState orientation_y;
		OSVR_OrientationState orientation_z;
		OSVR_AngularVelocityState angular_v;

		OSVR_ReturnCode xret = osvrGetOrientationState(m_orientations[0], timeValue, &orientation_x);
		OSVR_ReturnCode yret = osvrGetOrientationState(m_orientations[1], timeValue, &orientation_y);
		OSVR_ReturnCode zret = osvrGetOrientationState(m_orientations[3], timeValue, &orientation_z);
		OSVR_ReturnCode angret = osvrGetAngularVelocityState(m_orientations[2], timeValue, &angular_v);

		OSVR_Vec3 rpy_x;
		OSVR_Vec3 rpy_y;
		OSVR_Vec3 rpy_z;
		OSVR_Vec3 rpy_v;

		rpyFromQuaternion(&orientation_x, &rpy_x);
		rpyFromQuaternion(&orientation_y, &rpy_y);
		rpyFromQuaternion(&orientation_z, &rpy_z);
		rpyFromQuaternion(&angular_v.incrementalRotation, &rpy_v);

		double a = m_alpha;
		double last_z = m_last_yaw;
		
		// A factor of 2*PI is missing in angularVelocity incremental quats - at least with the HDK.
		double dt = angular_v.dt * 2 * M_PI;

		double z_accurate = osvrVec3GetZ(&rpy_z);
		double dzdt_fast = osvrVec3GetZ(&rpy_v);

		double dz_fast = dt * dzdt_fast;

		dz_fast = fixUnityAngle(dz_fast);
		z_accurate = fixUnityAngle(z_accurate);

		// Correct for the singularity at +-180 degrees
		if ((z_accurate < -M_PI / 2 && last_z > M_PI / 2) || (z_accurate > M_PI / 2 && last_z < -M_PI / 2)) {
			last_z = z_accurate;
		}

		double z_out = a*(last_z + dz_fast) + (1 - a)*(z_accurate);
		
		// Replace bogus results with accurate yaw. Happens sometimes on startup.
		if (std::isnan(z_out)) {
			z_out = z_accurate;
		}

		z_out = fixUnityAngle(z_out);

		m_last_yaw = z_out;

		OSVR_Vec3 rpy;
		osvrVec3SetX(&rpy, osvrVec3GetX(&rpy_x));
		osvrVec3SetY(&rpy, osvrVec3GetY(&rpy_y));
		osvrVec3SetZ(&rpy, z_out);

		quaternionFromRPY(&rpy, orientation);

		return OSVR_RETURN_SUCCESS;
	}

}