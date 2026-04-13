#ifndef DRONE_TELEMETRY_ABC_H
#define DRONE_TELEMETRY_ABC_H

#include <cstdint>
#include <chrono>

enum class MAVMode {
    GUIDED_DISARMED = 0,
    GUIDED_ARMED = 1
};

class DroneTelemetryABC {
public:
    virtual ~DroneTelemetryABC() = default;

    // Position simulation
    virtual float getX() const = 0;
    virtual float getY() const = 0;
    virtual float getAltitude() const = 0;

    // Status management
    virtual MAVMode getMode() const = 0;
    virtual void setMode(MAVMode mode) = 0;

    // Telemetry stream control
    virtual void startTelemetryStream() = 0;
    virtual void stopTelemetryStream() = 0;
    virtual void sendHeartbeat() = 0;
    virtual void sendLocalPositionNED() = 0;

    // Update simulation
    virtual void update() = 0;

protected:
    static constexpr float VELOCITY = 5.0f;  // m/s
    static constexpr float TELEMETRY_RATE = 10.0f;  // Hz
    static constexpr float TELEMETRY_INTERVAL = 1.0f / TELEMETRY_RATE;  // seconds
};

#endif // DRONE_TELEMETRY_ABC_H