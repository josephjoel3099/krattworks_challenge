#ifndef DRONE_TELEMETRY_ABC_H
#define DRONE_TELEMETRY_ABC_H

#include <cstdint>
#include <chrono>

/**
 * Supported flight modes used by the simulator.
 */
enum class MAVMode {
    GUIDED_DISARMED = 0,
    GUIDED_ARMED = 1,
    LAND = 2
};

/**
 * Abstract telemetry and control interface exposed by a drone implementation.
 */
class DroneTelemetryABC {
public:
    virtual ~DroneTelemetryABC() = default;

    /** Returns the current local X position in meters. */
    virtual float getX() const = 0;

    /** Returns the current local Y position in meters. */
    virtual float getY() const = 0;

    /** Returns the current local NED altitude in meters. */
    virtual float getAltitude() const = 0;

    /** Returns the current simulated flight mode. */
    virtual MAVMode getMode() const = 0;

    /** Requests a transition to a new simulated flight mode. */
    virtual void setMode(MAVMode mode) = 0;

    /** Starts the telemetry stream. */
    virtual void startTelemetryStream() = 0;

    /** Stops the telemetry stream. */
    virtual void stopTelemetryStream() = 0;

    /** Emits a heartbeat status update. */
    virtual void sendHeartbeat() = 0;

    /** Emits the local position message. */
    virtual void sendLocalPositionNED() = 0;

    /** Advances the simulation by one time step. */
    virtual void update() = 0;

    /** Sets the current altitude target in NED coordinates. */
    virtual void setTargetAltitude(float altitude) = 0;

    /** Returns the current altitude target in NED coordinates. */
    virtual float getTargetAltitude() const = 0;
};

#endif // DRONE_TELEMETRY_ABC_H