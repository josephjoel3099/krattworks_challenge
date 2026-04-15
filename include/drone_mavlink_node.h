/**
 * @file drone_mavlink_node.h
 * @brief Drone-side MAVLink transport for the simulator process.
 *
 * Exposes the small public interface used to publish simulated telemetry and
 * receive control commands over UDP.
 */
#ifndef DRONE_MAVLINK_NODE_H
#define DRONE_MAVLINK_NODE_H

#include <udp/simple_udp.h>

#include <app_config.h>
#include <ardupilotmega/mavlink.h>
#include <drone_simulator.h>

#include <atomic>
#include <cstdint>

/**
 * @struct DroneMavlinkIds
 * @brief MAVLink identifiers used by the drone endpoint.
 */
struct DroneMavlinkIds {
	uint8_t drone_system_id = 1;
	uint8_t drone_component_id = MAV_COMP_ID_AUTOPILOT1;
	uint8_t gcs_system_id = 255;
	uint8_t gcs_component_id = MAV_COMP_ID_SYSTEM_CONTROL;
};

/**
 * @class DroneMavlinkNode
 * @brief Bridges the simulator state to UDP/MAVLink traffic.
 *
 * Publishes heartbeat and local-position updates while handling incoming mode,
 * geofence, goto, and teleoperation commands from the ground station.
 */
class DroneMavlinkNode {
public:
	/**
	 * @brief Creates the drone MAVLink endpoint.
	 * @param telemetry Reference to the simulator state provider.
	 * @param shared_config Shared network configuration.
	 * @param drone_config Drone-specific runtime settings.
	 * @param running Shared shutdown flag for the process.
	 */
	DroneMavlinkNode(
		DroneTelemetrySimulator& telemetry,
		const SharedConfig& shared_config,
		const DroneConfig& drone_config,
		std::atomic_bool& running);

	/** @brief Runs the blocking MAVLink loop until shutdown is requested. */
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
