#ifndef SYNC_SENSOR_H_
#define SYNC_SENSOR_H_

#include "any.h"

enum class SyncSensorType {
    LIDAR = 0,
    POSE = 1,
    ANY = 2,
};

class SyncSensor {
public:
    using Ptr = std::shared_ptr<SyncSensor>;

    template<typename T>
    explicit SyncSensor(const SyncSensorType& type, const T& any);

    void setFrameID(const std::string& frame_id) { frame_id_ = frame_id; }

    void setTimestamp()
    const std::string &getFrameID() const { return frame_id_ };

private:
    SyncSensorType type_;
    std::string frame_id_;
    double stamp_;
    Any::Ptr any_;
};

#endif  // SYNC_SENSOR_H_