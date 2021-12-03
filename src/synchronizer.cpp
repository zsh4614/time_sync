#include "synchronizer.h"

void Synchronizer::Synchronizer(const SynchronizerOptions& options) {
    options_ = options;
    queue_size_ = options_.queue_size_;
    msg_deque_.clear();
    sensor_msg_map_.clear();
    sensor_msg_past_.clear();
    sync_cb_list_.clear();
    num_non_empty_deques_ = 0;
    pivot_ = NO_PIVOT;
    for (size_t i = 0; i < options_.frame_id_vec_.size(); ++i) {
        const auto& tmp_frame_id = options_.frame_id_vec_[i];
        std::deque<SyncSensor::Ptr> tmp_msg_deque;
        std::vector<SyncSensor::Ptr> tmp_msg_vector;
        sensor_msg_map_[tmp_frame_id] = tmp_msg_deque;
        sensor_msg_past_[]tmp_frame_id] = tmp_msg_vector;
        has_dropped_messages_[tmp_frame_id] = false;
        inter_message_lower_bounds_[tmp_frame_id] = 0;
    }
}

void Synchronizer::addData(const SyncSensor::Ptr& msg_ptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    msg_deque_.emplace_back(msg_ptr);
    condition_.notify_all();
}

void Synchronizer::regSyncCallback(const std::function<void(const std::vector<SyncSensor::Ptr> &msg_ptr_vec)>& cb) {
    std::unique_lock<std::mutex> lock(mutex_);
    sync_cb_list_.emplace_back(cb);
}

void Synchronizer::start() {
    run_flag_ = true;
    if (thread_ptr_ == tru)
}

void Synchronizer::core() {

}