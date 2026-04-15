#ifndef DRONE_MAVLINK_NODE_H
#define DRONE_MAVLINK_NODE_H

#include <udp/simple_udp.h>

#include <app_config.h>
#include <ardupilotmega/mavlink.h>
#include <drone_simulator.h>

#include <atomic>
#include <cstdint>

/**
 * MAVLink identifiers used by the drone endpoint.
 */
struct DroneMavlinkIds {
	uint8_t drone_system_id = 1;
	uint8_t drone_component_id = MAV_COMP_ID_AUTOPILOT1;
	uint8_t gcs_system_id = 255;
	uint8_t gcs_component_id = MAV_COMP_ID_SYSTEM_CONTROL;
};

/**
 * UDP/MAVLink communication loop for the simulated drone.
 */
class DroneMavlinkNode {
public:
	DroneMavlinkNode(
		DroneTelemetrySimulator& telemetry,
		const SharedConfig& shared_config,
		const DroneConfig& drone_config,
		std::atomic_bool& running);

	void run();

private:
	void sendMavlinkMessage(UdpSocket& socket, const IpAddress& gcs_addr, const mavlink_message_t& msg) const;
	void sendGeofencePoint(UdpSocket& socket, const IpAddress& gcs_addr, uint8_t point_index) const;
	void handleMavlinkCommand(const mavlink_message_t& msg, UdpSocket& socket, const IpAddress& gcs_addr);

	DroneTelemetrySimulator& telemetry_;
	const SharedConfig& shared_config_;
	const DroneConfig& drone_config_;
	std::atomic_bool& running_;
	DroneMavlinkIds ids_{};
};

#endif // DRONE_MAVLINK_NODE_H
