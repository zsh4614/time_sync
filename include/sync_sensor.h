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

    template <typename T>
    SyncSensor(const SyncSensorType& type, const T& any) {
        type_ = type;
        msg_ptr_.reset(new Any(any));
    }

    void setFrameID(const std::string& frame_id) { frame_id_ = frame_id; }

    void setTimestamp(const double& stamp) { stamp_ = stamp; }

    const std::string& getFrameID() const { return frame_id_; }

    const double& getTimestamp() const { return stamp_; }

    const SyncSensorType& getSensorType() const { return type_; }

    template <typename T>
    T getSensor() { return *msg_ptr_->AnyCast<T>(); }

private:
    SyncSensorType type_;
    std::string frame_id_;
    double stamp_;
    Any::Ptr msg_ptr_;
};

#endif  // SYNC_SENSOR_H_