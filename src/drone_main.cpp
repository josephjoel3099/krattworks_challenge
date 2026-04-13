#include <udp/simple_udp.h>

#include <common/mavlink.h>
#include <drone_telemtry_abc.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <mutex>

std::atomic_bool g_running{true};

// Concrete implementation of DroneTelemetryABC
class DroneTelemetrySimulator : public DroneTelemetryABC {
private:
	mutable std::mutex state_mutex_;
	float x_ = 0.0f;
	float y_ = 0.0f;
	float altitude_ = -5.0f;
	float vx_ = 0.0f;
	float vy_ = 0.0f;
	float vz_ = 0.0f;
	MAVMode mode_ = MAVMode::GUIDED_ARMED;
	std::chrono::steady_clock::time_point start_time_;

public:
	DroneTelemetrySimulator() {
		start_time_ = std::chrono::steady_clock::now();
	}

	float getX() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return x_;
	}

	float getY() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return y_;
	}

	float getAltitude() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return altitude_;
	}

	MAVMode getMode() const override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return mode_;
	}

	void setMode(MAVMode mode) override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		mode_ = mode;
	}

	void startTelemetryStream() override {
		std::printf("Telemetry stream started\n");
	}

	void stopTelemetryStream() override {
		std::printf("Telemetry stream stopped\n");
	}

	void sendHeartbeat() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		std::printf("HEARTBEAT: System armed=%s\n",
			mode_ == MAVMode::GUIDED_ARMED ? "true" : "false");
	}

	void sendLocalPositionNED() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		std::printf("LOCAL_POSITION_NED: X=%.2f Y=%.2f Z=%.2f VX=%.2f VY=%.2f VZ=%.2f\n",
			x_, y_, altitude_, vx_, vy_, vz_);
	}

	void update() override {
		std::lock_guard<std::mutex> lock(state_mutex_);
		
		const auto now = std::chrono::steady_clock::now();
		const float t = std::chrono::duration<float>(now - start_time_).count();
		
		// Simulate circular movement pattern
		x_ = std::sin(t) * 10.0f;
		y_ = std::cos(t) * 10.0f;
		altitude_ = -5.0f + std::sin(0.5f * t);
		vx_ = std::cos(t) * 10.0f;
		vy_ = -std::sin(t) * 10.0f;
		vz_ = 0.5f * std::cos(0.5f * t);
	}

	float getVx() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vx_;
	}

	float getVy() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vy_;
	}

	float getVz() const {
		std::lock_guard<std::mutex> lock(state_mutex_);
		return vz_;
	}
};

DroneTelemetrySimulator g_telemetry;

void signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		g_running = false;
	}
}

void send_mavlink_message(UdpSocket& socket, const IpAddress& gcs_addr, const mavlink_message_t& msg)
{
	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
	const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &msg);
	socket.sendto(tx_buf, len, gcs_addr);
}

void simulation_worker()
{
	while (g_running) {
		g_telemetry.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void mavlink_worker(const IpAddress& gcs_addr, uint16_t drone_port)
{
	UdpSocket socket;
	if (!socket.create(drone_port, false)) {
		std::fprintf(stderr, "Drone: failed to create UDP socket on port %u\n", drone_port);
		return;
	}

	std::printf("Drone: sending telemetry from 127.0.0.1:%u to %s\n", drone_port, gcs_addr.to_string().c_str());

	constexpr uint8_t kSystemId = 1;
	constexpr uint8_t kComponentId = MAV_COMP_ID_AUTOPILOT1;

	using clock = std::chrono::steady_clock;
	const auto start = clock::now();
	auto next_hb = start;
	auto next_pos = start;

	while (g_running) {
		const auto now = clock::now();

		if (now >= next_hb) {
			g_telemetry.sendHeartbeat();

			mavlink_message_t hb_msg;
			mavlink_msg_heartbeat_pack(
				kSystemId,
				kComponentId,
				&hb_msg,
				MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC,
				MAV_MODE_GUIDED_ARMED,
				0,
				MAV_STATE_ACTIVE);

			uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
			const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &hb_msg);
			socket.sendto(tx_buf, len, gcs_addr);

			next_hb += std::chrono::seconds(1);
		}

		if (now >= next_pos) {
			g_telemetry.sendLocalPositionNED();

			const float x = g_telemetry.getX();
			const float y = g_telemetry.getY();
			const float z = g_telemetry.getAltitude();
			const float vx = g_telemetry.getVx();
			const float vy = g_telemetry.getVy();
			const float vz = g_telemetry.getVz();

			mavlink_message_t pos_msg;
			mavlink_msg_local_position_ned_pack(
				kSystemId,
				kComponentId,
				&pos_msg,
				static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()),
				x,
				y,
				z,
				vx,
				vy,
				vz);

			uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
			const uint16_t len = mavlink_msg_to_send_buffer(tx_buf, &pos_msg);
			socket.sendto(tx_buf, len, gcs_addr);

			next_pos += std::chrono::milliseconds(100);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	socket.close();
}

int main()
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	constexpr uint16_t kDroneLocalPort = 14551;
	constexpr uint16_t kGcsPort = 14550;

	const IpAddress gcs_addr{"127.0.0.1", kGcsPort};

	// Start simulation thread
	std::thread sim_thread(simulation_worker);

	// Start MAVLink communication thread
	std::thread mavlink_thread(mavlink_worker, gcs_addr, kDroneLocalPort);

	// Wait for threads to complete
	sim_thread.join();
	mavlink_thread.join();

	std::printf("Drone: stopped\n");
	return 0;
}

