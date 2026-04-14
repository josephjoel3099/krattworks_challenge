#ifndef DRONE_TELEMETRY_ABC_H
#define DRONE_TELEMETRY_ABC_H

#include <cstdint>
#include <chrono>

enum class MAVMode {
    GUIDED_DISARMED = 0,
    GUIDED_ARMED = 1,
    LAND = 2
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

    // Movement control
    virtual void setTargetAltitude(float altitude) = 0;
    virtual float getTargetAltitude() const = 0;
};

#endif // DRONE_TELEMETRY_ABC_H