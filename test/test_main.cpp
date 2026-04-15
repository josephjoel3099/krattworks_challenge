#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#include <imgui.h>

#define private public
#include <app_config.h>
#include <ardupilotmega/mavlink.h>
#include <drone_mavlink_node.h>
#include <drone_simulator.h>
#include <gcs_mavlink_node.h>
#undef private

namespace {

using namespace std::chrono_literals;

DroneConfig makeDroneConfig(float arm_target_altitude_m = 20.0f)
{
	DroneConfig cfg{};
	cfg.max_velocity_mps = 5.0f;
	cfg.manual_horizontal_velocity_mps = 5.0f;
	cfg.update_rate_hz = 100.0f;
	cfg.heartbeat_rate_hz = 10.0f;
	cfg.position_rate_hz = 10.0f;
	cfg.climb_rate_mps = 25.0f;
	cfg.manual_vertical_velocity_mps = 5.0f;
	cfg.land_rate_mps = 25.0f;
	cfg.arm_target_altitude_m = arm_target_altitude_m;
	cfg.horizontal_arrival_tolerance_m = 0.5f;
	cfg.manual_control_timeout_ms = 120;
	cfg.poll_timeout_ms = 5;
	cfg.geofence_corners_m = {
		XYPoint{-50.0f, -50.0f},
		XYPoint{50.0f, -50.0f},
		XYPoint{50.0f, 50.0f},
		XYPoint{-50.0f, 50.0f},
	};
	return cfg;
}

GcsConfig makeGcsConfig()
{
	GcsConfig cfg{};
	cfg.poll_timeout_ms = 5;
	cfg.geofence_fetch_interval_ms = 50;
	cfg.manual_control_interval_ms = 25;
	cfg.telemetry_stale_timeout_ms = 400;
	cfg.max_altitude_history_points = 128;
	return cfg;
}

SharedConfig makeSharedConfig()
{
	static std::atomic_uint16_t next_port{43000};
	const uint16_t base_port = next_port.fetch_add(10);
	return SharedConfig{"127.0.0.1", base_port, static_cast<uint16_t>(base_port + 1)};
}

void stepSimulation(DroneTelemetrySimulator& simulator, float dt_seconds, int steps = 1)
{
	for (int i = 0; i < steps; ++i) {
		simulator.last_update_time_ = std::chrono::steady_clock::now()
			- std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(dt_seconds));
		simulator.update();
	}
}

template <typename Predicate>
bool waitFor(
	Predicate predicate,
	std::chrono::milliseconds timeout = 1500ms,
	std::chrono::milliseconds poll_interval = 10ms)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (std::chrono::steady_clock::now() < deadline) {
		if (predicate()) {
			return true;
		}
		std::this_thread::sleep_for(poll_interval);
	}

	return predicate();
}

bool receiveMavlinkMessage(
	UdpSocket& socket,
	mavlink_message_t& message,
	std::chrono::milliseconds timeout = 400ms)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	mavlink_status_t status{};

	while (std::chrono::steady_clock::now() < deadline) {
		if (!socket.poll_read(10)) {
			continue;
		}

		while (socket.available() > 0) {
			char buffer[1024];
			IpAddress from;
			const int bytes = socket.recvfrom(buffer, sizeof(buffer), from);
			if (bytes <= 0) {
				continue;
			}

			for (int i = 0; i < bytes; ++i) {
				if (mavlink_parse_char(
					MAVLINK_COMM_0,
					static_cast<uint8_t>(buffer[i]),
					&message,
					&status)) {
					return true;
				}
			}
		}
	}

	return false;
}

std::vector<uint32_t> collectMessageIds(UdpSocket& socket, std::chrono::milliseconds duration)
{
	std::vector<uint32_t> ids;
	const auto deadline = std::chrono::steady_clock::now() + duration;
	mavlink_status_t status{};

	while (std::chrono::steady_clock::now() < deadline) {
		if (!socket.poll_read(10)) {
			continue;
		}

		while (socket.available() > 0) {
			char buffer[1024];
			IpAddress from;
			const int bytes = socket.recvfrom(buffer, sizeof(buffer), from);
			if (bytes <= 0) {
				continue;
			}

			for (int i = 0; i < bytes; ++i) {
				mavlink_message_t msg{};
				if (mavlink_parse_char(
					MAVLINK_COMM_0,
					static_cast<uint8_t>(buffer[i]),
					&msg,
					&status)) {
					ids.push_back(msg.msgid);
				}
			}
		}
	}

	return ids;
}

bool isDashboardConnected(const gcs_ui::DashboardState& state)
{
	if (!state.telemetry.has_heartbeat) {
		return false;
	}

	const auto age = std::chrono::steady_clock::now() - state.telemetry.last_message_time;
	return age <= std::chrono::milliseconds(state.telemetry_stale_timeout_ms);
}

class LoopbackHarness {
public:
	LoopbackHarness()
		: shared_config_(makeSharedConfig()),
		  gcs_config_(makeGcsConfig()),
		  drone_config_(makeDroneConfig()),
		  running_(true),
		  telemetry_(drone_config_),
		  drone_node_(telemetry_, shared_config_, drone_config_, running_),
		  gcs_node_(shared_config_, gcs_config_, drone_config_, running_)
	{
	}

	~LoopbackHarness()
	{
		stop();
	}

	void start()
	{
		gcs_thread_ = std::thread(&GcsMavlinkNode::run, &gcs_node_);
		drone_thread_ = std::thread(&DroneMavlinkNode::run, &drone_node_);
		sim_thread_ = std::thread([this] {
			while (running_.load()) {
				telemetry_.update();
				std::this_thread::sleep_for(10ms);
			}
		});
	}

	void stop()
	{
		if (!running_.exchange(false)) {
			return;
		}

		if (sim_thread_.joinable()) {
			sim_thread_.join();
		}
		if (drone_thread_.joinable()) {
			drone_thread_.join();
		}
		if (gcs_thread_.joinable()) {
			gcs_thread_.join();
		}
	}

	gcs_ui::DashboardState state() const
	{
		return gcs_node_.makeDashboardState();
	}

	gcs_ui::DashboardActions actions()
	{
		return gcs_node_.makeDashboardActions();
	}

	SharedConfig shared_config_;
	GcsConfig gcs_config_;
	DroneConfig drone_config_;
	std::atomic_bool running_;
	DroneTelemetrySimulator telemetry_;
	DroneMavlinkNode drone_node_;
	GcsMavlinkNode gcs_node_;

private:
	std::thread sim_thread_;
	std::thread drone_thread_;
	std::thread gcs_thread_;
};

// Verifies the simulator starts from a known safe state at the geofence center.
TEST(TEST_Drone, Initialization_DefaultsToOriginAndGuidedDisarmed)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());

	EXPECT_FLOAT_EQ(simulator.getX(), 0.0f);
	EXPECT_FLOAT_EQ(simulator.getY(), 0.0f);
	EXPECT_FLOAT_EQ(simulator.getAltitude(), 0.0f);
	EXPECT_EQ(simulator.getMode(), MAVMode::GUIDED_DISARMED);
}

// Verifies arming succeeds and guided commands are rejected while disarmed.
TEST(TEST_Drone, ModeTransitions_ArmAndRejectGuidedCommandsWhileDisarmed)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());

	EXPECT_FALSE(simulator.setGuidedHoldTarget(10.0f, 0.0f, -5.0f));
	EXPECT_FALSE(simulator.setManualControl(1.0f, 0.0f, 0.0f));

	simulator.setMode(MAVMode::GUIDED_ARMED);
	EXPECT_EQ(simulator.getMode(), MAVMode::GUIDED_ARMED);
	EXPECT_FLOAT_EQ(simulator.getTargetAltitude(), -20.0f);
}

// Verifies disarming while airborne automatically transitions into LAND mode.
TEST(TEST_Drone, AltitudeRules_DisarmInAirTransitionsToLand)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());
	simulator.setMode(MAVMode::GUIDED_ARMED);
	stepSimulation(simulator, 0.2f, 2);

	ASSERT_LT(simulator.getAltitude(), -0.5f);
	simulator.setMode(MAVMode::GUIDED_DISARMED);
	EXPECT_EQ(simulator.getMode(), MAVMode::LAND);

	simulator.state_.altitude = 0.0f;
	simulator.setMode(MAVMode::GUIDED_DISARMED);
	EXPECT_EQ(simulator.getMode(), MAVMode::GUIDED_DISARMED);
}

// Verifies goto navigation reaches the commanded point and then holds position.
TEST(TEST_Drone, MovementSimulation_GotoMovesToTargetAndThenHolds)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());
	simulator.setMode(MAVMode::GUIDED_ARMED);

	ASSERT_TRUE(simulator.setGuidedHoldTarget(10.0f, -5.0f, -3.0f));
	stepSimulation(simulator, 0.2f, 12);

	EXPECT_NEAR(simulator.getX(), 10.0f, 0.6f);
	EXPECT_NEAR(simulator.getY(), -5.0f, 0.6f);
	EXPECT_NEAR(simulator.getAltitude(), -3.0f, 0.2f);

	const float hold_x = simulator.getX();
	const float hold_y = simulator.getY();
	stepSimulation(simulator, 0.2f, 5);

	EXPECT_NEAR(simulator.getX(), hold_x, 0.05f);
	EXPECT_NEAR(simulator.getY(), hold_y, 0.05f);
}

// Verifies both goto and manual-control motion stay inside the configured geofence.
TEST(TEST_Drone, Geofence_RejectsOutsideTargetsAndPreventsBoundaryCrossing)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());
	simulator.setMode(MAVMode::GUIDED_ARMED);

	EXPECT_FALSE(simulator.setGuidedHoldTarget(100.0f, 0.0f, -2.0f));

	simulator.state_.x = 49.5f;
	simulator.state_.y = 0.0f;
	simulator.state_.hold_target = XYPoint{49.5f, 0.0f};

	ASSERT_TRUE(simulator.setManualControl(1.0f, 0.0f, 0.0f));
	stepSimulation(simulator, 0.2f);

	EXPECT_LE(simulator.getX(), 49.5f + 1e-3f);
	EXPECT_FLOAT_EQ(simulator.getVx(), 0.0f);
}

// Verifies manual control moves the drone and then times out back into a stable hold.
TEST(TEST_Drone, ManualControl_MovesWithInputAndStopsAfterTimeout)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());
	simulator.setMode(MAVMode::GUIDED_ARMED);

	ASSERT_TRUE(simulator.setManualControl(1.0f, -0.5f, 0.0f));
	stepSimulation(simulator, 0.2f, 2);

	EXPECT_GT(simulator.getX(), 0.0f);
	EXPECT_LT(simulator.getY(), 0.0f);
	EXPECT_NEAR(simulator.getVx(), 5.0f, 0.2f);
	EXPECT_NEAR(simulator.getVy(), -2.5f, 0.2f);

	const float x_after_input = simulator.getX();
	const float y_after_input = simulator.getY();
	simulator.manual_control_.last_command_time = std::chrono::steady_clock::now()
		- simulator.motion_settings_.manual_control_timeout - 10ms;
	stepSimulation(simulator, 0.2f, 2);

	EXPECT_NEAR(simulator.getX(), x_after_input, 0.15f);
	EXPECT_NEAR(simulator.getY(), y_after_input, 0.15f);
	EXPECT_NEAR(simulator.getVx(), 0.0f, 0.05f);
	EXPECT_NEAR(simulator.getVy(), 0.0f, 0.05f);
}

// Verifies LAND mode descends smoothly and ends with the vehicle disarmed on the ground.
TEST(TEST_Drone, LandMode_DescendsGraduallyAndEndsDisarmed)
{
	DroneTelemetrySimulator simulator(makeDroneConfig());
	simulator.setMode(MAVMode::GUIDED_ARMED);
	stepSimulation(simulator, 0.1f, 4);

	const float airborne_altitude = simulator.getAltitude();
	ASSERT_LT(airborne_altitude, -0.5f);

	simulator.setMode(MAVMode::LAND);
	stepSimulation(simulator, 0.05f);
	EXPECT_GT(simulator.getAltitude(), airborne_altitude);

	stepSimulation(simulator, 0.1f, 20);
	EXPECT_EQ(simulator.getMode(), MAVMode::GUIDED_DISARMED);
	EXPECT_FLOAT_EQ(simulator.getAltitude(), 0.0f);
}

// Verifies command handling can run concurrently without stalling simulation updates.
TEST(TEST_Drone, ThreadSafety_CommandHandlingDoesNotBlockSimulationLoop)
{
	auto config = makeDroneConfig();
	auto shared = makeSharedConfig();
	std::atomic_bool keep_running{true};
	DroneTelemetrySimulator simulator(config);
	DroneMavlinkNode node(simulator, shared, config, keep_running);
	UdpSocket unused_socket;
	const IpAddress gcs_addr{shared.host.c_str(), shared.gcs_port};

	simulator.setMode(MAVMode::GUIDED_ARMED);
	std::atomic_int update_count{0};

	std::thread updater([&] {
		while (keep_running.load()) {
			stepSimulation(simulator, 0.01f);
			++update_count;
		}
	});

	std::thread commander([&] {
		for (int i = 0; i < 250; ++i) {
			mavlink_message_t msg{};
			if ((i % 2) == 0) {
				mavlink_msg_set_mode_pack(
					config.gcs_system_id,
					config.gcs_component_id,
					&msg,
					config.drone_system_id,
					MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
					static_cast<uint32_t>(MAVMode::GUIDED_ARMED));
			} else {
				mavlink_msg_command_long_pack(
					config.gcs_system_id,
					config.gcs_component_id,
					&msg,
					config.drone_system_id,
					config.drone_component_id,
					MAV_CMD_OVERRIDE_GOTO,
					0,
					static_cast<float>(MAV_GOTO_DO_HOLD),
					static_cast<float>(MAV_GOTO_HOLD_AT_SPECIFIED_POSITION),
					static_cast<float>(MAV_FRAME_LOCAL_NED),
					NAN,
					1.0f,
					1.0f,
					-5.0f);
			}
			node.handleMavlinkCommand(msg, unused_socket, gcs_addr);
		}
		keep_running.store(false);
	});

	commander.join();
	updater.join();

	EXPECT_GT(update_count.load(), 50);
}

// Verifies heartbeat tracking drives the dashboard's connection state and stale detection.
TEST(TEST_GCS, ConnectionHandling_DetectsHeartbeatAndTimeout)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	mavlink_message_t heartbeat_msg{};
	mavlink_msg_heartbeat_pack(
		drone_config.drone_system_id,
		drone_config.drone_component_id,
		&heartbeat_msg,
		MAV_TYPE_QUADROTOR,
		MAV_AUTOPILOT_GENERIC,
		MAV_MODE_GUIDED_ARMED,
		static_cast<uint32_t>(MAVMode::GUIDED_ARMED),
		MAV_STATE_ACTIVE);
	node.handleMavlinkMessage(heartbeat_msg);

	EXPECT_TRUE(isDashboardConnected(node.makeDashboardState()));

	node.telemetry_.last_message_time = std::chrono::steady_clock::now()
		- std::chrono::milliseconds(gcs_config.telemetry_stale_timeout_ms + 50);
	EXPECT_FALSE(isDashboardConnected(node.makeDashboardState()));
}

// Verifies MAVLink telemetry parsing updates the dashboard snapshot correctly.
TEST(TEST_GCS, TelemetryParsing_UpdatesPositionAltitudeAndMode)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	mavlink_message_t heartbeat_msg{};
	mavlink_msg_heartbeat_pack(
		drone_config.drone_system_id,
		drone_config.drone_component_id,
		&heartbeat_msg,
		MAV_TYPE_QUADROTOR,
		MAV_AUTOPILOT_GENERIC,
		MAV_MODE_GUIDED_ARMED,
		static_cast<uint32_t>(MAVMode::LAND),
		MAV_STATE_ACTIVE);
	node.handleMavlinkMessage(heartbeat_msg);

	mavlink_message_t position_msg{};
	mavlink_msg_local_position_ned_pack(
		drone_config.drone_system_id,
		drone_config.drone_component_id,
		&position_msg,
		1234,
		12.5f,
		-4.25f,
		-20.0f,
		1.0f,
		2.0f,
		-0.5f);
	node.handleMavlinkMessage(position_msg);

	const auto state = node.makeDashboardState();
	EXPECT_TRUE(state.telemetry.has_heartbeat);
	EXPECT_TRUE(state.telemetry.has_position);
	EXPECT_EQ(state.telemetry.custom_mode, static_cast<uint32_t>(MAVMode::LAND));
	EXPECT_NEAR(state.telemetry.x, 12.5f, 1e-3f);
	EXPECT_NEAR(state.telemetry.y, -4.25f, 1e-3f);
	EXPECT_NEAR(state.telemetry.z, -20.0f, 1e-3f);
	ASSERT_FALSE(state.telemetry.altitude_history.empty());
	EXPECT_NEAR(state.telemetry.altitude_history.back().altitude_m, 20.0f, 1e-3f);
}

// Verifies SET_MODE commands are encoded with the expected target, flags, and mode value.
TEST(TEST_GCS, CommandGeneration_SendSetModeFormatsCorrectMessage)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	UdpSocket receiver;
	UdpSocket sender;
	ASSERT_TRUE(receiver.create(shared.drone_port, true));
	ASSERT_TRUE(sender.create(shared.gcs_port, true));

	node.sendSetModeCommand(sender, RequestedMode::GuidedArmed);

	mavlink_message_t message{};
	ASSERT_TRUE(receiveMavlinkMessage(receiver, message));
	ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_SET_MODE);

	mavlink_set_mode_t decoded{};
	mavlink_msg_set_mode_decode(&message, &decoded);
	EXPECT_EQ(decoded.target_system, drone_config.drone_system_id);
	EXPECT_NE(decoded.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED, 0);
	EXPECT_NE(decoded.base_mode & MAV_MODE_FLAG_SAFETY_ARMED, 0);
	EXPECT_EQ(decoded.custom_mode, static_cast<uint32_t>(RequestedMode::GuidedArmed));

	sender.close();
	receiver.close();
}

// Verifies goto commands preserve the requested coordinates and altitude in MAVLink fields.
TEST(TEST_GCS, CommandGeneration_SendOverrideGotoUsesCorrectFields)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	UdpSocket receiver;
	UdpSocket sender;
	ASSERT_TRUE(receiver.create(shared.drone_port, true));
	ASSERT_TRUE(sender.create(shared.gcs_port, true));

	node.sendOverrideGotoCommand(sender, XYPoint{4.0f, -6.0f}, -12.0f);

	mavlink_message_t message{};
	ASSERT_TRUE(receiveMavlinkMessage(receiver, message));
	ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_COMMAND_LONG);

	mavlink_command_long_t decoded{};
	mavlink_msg_command_long_decode(&message, &decoded);
	EXPECT_EQ(decoded.command, MAV_CMD_OVERRIDE_GOTO);
	EXPECT_EQ(static_cast<int>(decoded.param2), MAV_GOTO_HOLD_AT_SPECIFIED_POSITION);
	EXPECT_EQ(static_cast<int>(decoded.param3), MAV_FRAME_LOCAL_NED);
	EXPECT_NEAR(decoded.param5, 4.0f, 1e-3f);
	EXPECT_NEAR(decoded.param6, -6.0f, 1e-3f);
	EXPECT_NEAR(decoded.param7, -12.0f, 1e-3f);

	sender.close();
	receiver.close();
}

// Verifies manual-control velocity intents are normalized into MAVLink's [-1000, 1000] range.
TEST(TEST_GCS, CommandGeneration_SendManualControlNormalizesAxes)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	UdpSocket receiver;
	UdpSocket sender;
	ASSERT_TRUE(receiver.create(shared.drone_port, true));
	ASSERT_TRUE(sender.create(shared.gcs_port, true));

	const TeleopState teleop{true, 5.0f, -2.5f, 1.0f};
	node.sendManualControl(sender, teleop);

	mavlink_message_t message{};
	ASSERT_TRUE(receiveMavlinkMessage(receiver, message));
	ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_MANUAL_CONTROL);

	mavlink_manual_control_t decoded{};
	mavlink_msg_manual_control_decode(&message, &decoded);
	EXPECT_EQ(decoded.target, drone_config.drone_system_id);
	EXPECT_EQ(decoded.x, 1000);
	EXPECT_EQ(decoded.y, -500);
	EXPECT_EQ(decoded.z, 200);

	sender.close();
	receiver.close();
}

// Verifies dashboard actions enqueue commands and clamp teleop state for UI logic.
TEST(TEST_GCS, GuiStateLogic_ActionsQueueCommandsAndClampInputs)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);
	const auto actions = node.makeDashboardActions();

	actions.on_set_teleop_enabled(true);
	actions.on_update_teleop_axes(99.0f, -99.0f, 99.0f);

	const TeleopState teleop = node.getTeleopState();
	EXPECT_TRUE(teleop.enabled);
	EXPECT_FLOAT_EQ(teleop.x_velocity_mps, drone_config.manual_horizontal_velocity_mps);
	EXPECT_FLOAT_EQ(teleop.y_velocity_mps, -drone_config.manual_horizontal_velocity_mps);
	EXPECT_FLOAT_EQ(teleop.z_velocity_mps, drone_config.manual_vertical_velocity_mps);

	EXPECT_TRUE(actions.on_send_goto(10.0f, 5.0f, 7.0f));
	actions.on_arm();

	PendingCommand command{};
	ASSERT_TRUE(node.popPendingCommand(command));
	EXPECT_EQ(command.type, PendingCommandType::OverrideGoto);
	EXPECT_NEAR(command.target.x, 10.0f, 1e-3f);
	EXPECT_NEAR(command.target.y, 5.0f, 1e-3f);
	EXPECT_NEAR(command.target_altitude_ned_m, -7.0f, 1e-3f);

	ASSERT_TRUE(node.popPendingCommand(command));
	EXPECT_EQ(command.type, PendingCommandType::ModeChange);
	EXPECT_EQ(command.mode, RequestedMode::GuidedArmed);
}

// Verifies UI snapshots remain quick to obtain even while the MAVLink worker is polling.
TEST(TEST_GCS, Threading_GuiThreadRemainsResponsiveDuringIo)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);

	std::thread worker(&GcsMavlinkNode::run, &node);
	const auto actions = node.makeDashboardActions();
	const auto start = std::chrono::steady_clock::now();

	for (int i = 0; i < 250; ++i) {
		actions.on_set_teleop_enabled((i % 2) == 0);
		actions.on_update_teleop_axes(1.0f, -1.0f, 0.5f);
		const auto state = node.makeDashboardState();
		EXPECT_TRUE(state.is_running);
	}

	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start);
	running.store(false);
	worker.join();

	EXPECT_LT(elapsed.count(), 1000);
}

// Verifies core MAVLink messages survive a full serialize/parse/decode round-trip.
TEST(TEST_Utility, Serialization_RoundTripsHeartbeatAndLocalPosition)
{
	mavlink_message_t heartbeat_tx{};
	mavlink_msg_heartbeat_pack(
		1,
		1,
		&heartbeat_tx,
		MAV_TYPE_QUADROTOR,
		MAV_AUTOPILOT_GENERIC,
		MAV_MODE_GUIDED_ARMED,
		42,
		MAV_STATE_ACTIVE);

	uint8_t heartbeat_buffer[MAVLINK_MAX_PACKET_LEN];
	const uint16_t heartbeat_len = mavlink_msg_to_send_buffer(heartbeat_buffer, &heartbeat_tx);
	mavlink_status_t status{};
	mavlink_message_t heartbeat_rx{};
	bool parsed = false;
	for (uint16_t i = 0; i < heartbeat_len; ++i) {
		if (mavlink_parse_char(MAVLINK_COMM_0, heartbeat_buffer[i], &heartbeat_rx, &status)) {
			parsed = true;
			break;
		}
	}

	ASSERT_TRUE(parsed);
	mavlink_heartbeat_t heartbeat_decoded{};
	mavlink_msg_heartbeat_decode(&heartbeat_rx, &heartbeat_decoded);
	EXPECT_EQ(heartbeat_decoded.type, MAV_TYPE_QUADROTOR);
	EXPECT_EQ(heartbeat_decoded.custom_mode, 42u);

	mavlink_message_t position_tx{};
	mavlink_msg_local_position_ned_pack(1, 1, &position_tx, 777, 3.0f, -2.0f, -5.0f, 1.5f, 0.0f, -0.2f);
	uint8_t position_buffer[MAVLINK_MAX_PACKET_LEN];
	const uint16_t position_len = mavlink_msg_to_send_buffer(position_buffer, &position_tx);
	mavlink_message_t position_rx{};
	parsed = false;
	status = {};
	for (uint16_t i = 0; i < position_len; ++i) {
		if (mavlink_parse_char(MAVLINK_COMM_0, position_buffer[i], &position_rx, &status)) {
			parsed = true;
			break;
		}
	}

	ASSERT_TRUE(parsed);
	mavlink_local_position_ned_t position_decoded{};
	mavlink_msg_local_position_ned_decode(&position_rx, &position_decoded);
	EXPECT_EQ(position_decoded.time_boot_ms, 777u);
	EXPECT_NEAR(position_decoded.x, 3.0f, 1e-3f);
	EXPECT_NEAR(position_decoded.y, -2.0f, 1e-3f);
	EXPECT_NEAR(position_decoded.z, -5.0f, 1e-3f);
}

// Verifies the pending-command queue remains consistent when multiple threads enqueue simultaneously.
TEST(TEST_Utility, ThreadSafeQueue_PreservesAllCommandsUnderContention)
{
	auto shared = makeSharedConfig();
	auto gcs_config = makeGcsConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	GcsMavlinkNode node(shared, gcs_config, drone_config, running);
	const auto actions = node.makeDashboardActions();

	std::vector<std::thread> producers;
	for (int thread_index = 0; thread_index < 4; ++thread_index) {
		producers.emplace_back([&actions] {
			for (int i = 0; i < 50; ++i) {
				if ((i % 2) == 0) {
					actions.on_arm();
				} else {
					actions.on_land();
				}
			}
		});
	}

	for (auto& producer : producers) {
		producer.join();
	}

	int queued_count = 0;
	PendingCommand command{};
	while (node.popPendingCommand(command)) {
		++queued_count;
	}

	EXPECT_EQ(queued_count, 200);
}

// Verifies the drone network loop emits heartbeat and position telemetry close to 10Hz.
TEST(TEST_Utility, Timing_DroneMavlinkNodePublishesTelemetryNearTenHertz)
{
	auto shared = makeSharedConfig();
	auto drone_config = makeDroneConfig();
	std::atomic_bool running{true};
	DroneTelemetrySimulator simulator(drone_config);
	DroneMavlinkNode node(simulator, shared, drone_config, running);

	UdpSocket receiver;
	ASSERT_TRUE(receiver.create(shared.gcs_port, true));

	std::thread worker(&DroneMavlinkNode::run, &node);
	const auto ids = collectMessageIds(receiver, 650ms);
	running.store(false);
	worker.join();
	receiver.close();

	const auto heartbeat_count = std::count(ids.begin(), ids.end(), MAVLINK_MSG_ID_HEARTBEAT);
	const auto position_count = std::count(ids.begin(), ids.end(), MAVLINK_MSG_ID_LOCAL_POSITION_NED);
	EXPECT_GE(heartbeat_count, 5);
	EXPECT_LE(heartbeat_count, 8);
	EXPECT_GE(position_count, 5);
	EXPECT_LE(position_count, 8);
}

// Verifies the GCS discovers the drone over real UDP loopback heartbeats.
TEST(TEST_Integration, Connection_HeartbeatMakesGcsDetectDrone)
{
	LoopbackHarness harness;
	harness.start();

	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));
	const auto state = harness.state();
	EXPECT_TRUE(state.telemetry.has_heartbeat);
	EXPECT_EQ(state.telemetry.system_id, harness.drone_config_.drone_system_id);
}

// Verifies arming drives the drone into GUIDED_ARMED and climbs to the target 20m altitude.
TEST(TEST_Integration, ArmAndAutoClimb_ReachesTwentyMeters)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();

	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::GUIDED_ARMED; }, 800ms));
	ASSERT_TRUE(waitFor([&harness] {
		const auto state = harness.state();
		return state.telemetry.has_position && state.telemetry.z <= -19.0f;
	}, 2000ms));

	const auto state = harness.state();
	EXPECT_NEAR(state.telemetry.z, -20.0f, 1.5f);
	EXPECT_EQ(state.telemetry.custom_mode, static_cast<uint32_t>(MAVMode::GUIDED_ARMED));
}

// Verifies an override-goto request drives the drone to the requested position within tolerance.
TEST(TEST_Integration, GoToCommand_MovesDroneToTargetWithinTolerance)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.state().telemetry.z <= -15.0f; }, 2000ms));

	ASSERT_TRUE(actions.on_send_goto(6.0f, -4.0f, 20.0f));
	ASSERT_TRUE(waitFor([&harness] {
		const auto state = harness.state();
		return std::abs(state.telemetry.x - 6.0f) < 0.8f && std::abs(state.telemetry.y + 4.0f) < 0.8f;
	}, 2500ms));
}

// Verifies periodic teleop commands move the drone and stopping input removes motion.
TEST(TEST_Integration, ManualControl_MovesDroneAndStopsWhenInputStops)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::GUIDED_ARMED; }, 800ms));

	const float start_x = harness.state().telemetry.x;
	actions.on_set_teleop_enabled(true);
	actions.on_update_teleop_axes(5.0f, 0.0f, 0.0f);
	ASSERT_TRUE(waitFor([&harness, start_x] { return harness.state().telemetry.x > start_x + 1.0f; }, 1200ms));

	actions.on_update_teleop_axes(0.0f, 0.0f, 0.0f);
	actions.on_set_teleop_enabled(false);
	ASSERT_TRUE(waitFor([&harness] { return std::abs(harness.state().telemetry.vx) < 0.2f; }, 800ms));
}

// Verifies geofence validation rejects unreachable goto requests before they are sent.
TEST(TEST_Integration, GeofenceEnforcement_RejectsOutsideCommands)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::GUIDED_ARMED; }, 800ms));

	EXPECT_FALSE(actions.on_send_goto(250.0f, 0.0f, 10.0f));
	std::this_thread::sleep_for(150ms);

	const auto state = harness.state();
	EXPECT_LE(std::abs(state.telemetry.x), 50.0f);
	EXPECT_LE(std::abs(state.telemetry.y), 50.0f);
}

// Verifies a LAND request results in a smooth descent and final disarmed state.
TEST(TEST_Integration, LandAndDisarm_EndsSafelyOnGround)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.state().telemetry.z <= -15.0f; }, 2000ms));

	actions.on_land();
	ASSERT_TRUE(waitFor([&harness] {
		const auto state = harness.state();
		return harness.telemetry_.getMode() == MAVMode::GUIDED_DISARMED
			&& std::abs(state.telemetry.z) < 0.2f
			&& state.telemetry.custom_mode == static_cast<uint32_t>(MAVMode::GUIDED_DISARMED);
	}, 2500ms));
}

// Verifies a disarm request while airborne enters LAND first before final disarm.
TEST(TEST_Integration, DisarmSafety_AirborneDisarmTriggersLandMode)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.state().telemetry.z <= -10.0f; }, 2000ms));

	actions.on_disarm();
	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::LAND; }, 800ms));
	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::GUIDED_DISARMED; }, 2000ms));
}

// Verifies heartbeats and local-position updates continue to flow into the GCS over time.
TEST(TEST_Integration, TelemetryFlow_ContinuousUpdatesReachTheDashboard)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	ASSERT_TRUE(waitFor([&harness] {
		const auto state = harness.state();
		return state.telemetry.has_position && state.telemetry.altitude_history.size() >= 5;
	}, 1200ms));

	const auto before = harness.state().telemetry.time_boot_ms;
	ASSERT_TRUE(waitFor([&harness, before] { return harness.state().telemetry.time_boot_ms > before; }, 400ms));
}

// Verifies continuous commands and telemetry polling do not block the dashboard snapshot loop.
TEST(TEST_Integration, Threading_StressKeepsDashboardResponsive)
{
	LoopbackHarness harness;
	harness.start();
	ASSERT_TRUE(waitFor([&harness] { return isDashboardConnected(harness.state()); }, 1500ms));

	const auto actions = harness.actions();
	actions.on_arm();
	ASSERT_TRUE(waitFor([&harness] { return harness.telemetry_.getMode() == MAVMode::GUIDED_ARMED; }, 800ms));

	std::atomic_bool keep_sending{true};
	std::thread sender([&actions, &keep_sending] {
		int tick = 0;
		while (keep_sending.load()) {
			actions.on_set_teleop_enabled(true);
			actions.on_update_teleop_axes((tick % 2) == 0 ? 5.0f : -5.0f, 0.0f, 0.0f);
			if ((tick % 5) == 0) {
				const float x = static_cast<float>((tick % 6) - 3);
				const float y = static_cast<float>((tick % 4) - 2);
				(void)actions.on_send_goto(x, y, 10.0f);
			}
			++tick;
			std::this_thread::sleep_for(20ms);
		}
		actions.on_set_teleop_enabled(false);
		actions.on_update_teleop_axes(0.0f, 0.0f, 0.0f);
	});

	int snapshot_reads = 0;
	const auto deadline = std::chrono::steady_clock::now() + 600ms;
	while (std::chrono::steady_clock::now() < deadline) {
		const auto state = harness.state();
		EXPECT_TRUE(state.is_running);
		++snapshot_reads;
		std::this_thread::sleep_for(2ms);
	}

	keep_sending.store(false);
	sender.join();

	EXPECT_GT(snapshot_reads, 100);
	EXPECT_TRUE(harness.state().telemetry.has_heartbeat);
}

} // namespace

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
