#ifndef SYNCHRONIZER_H_
#define SYNCHRONIZER_H_

#include <vector>
#include <string>
#include <deque>
#include <map>
#include <limits>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "sync_sensor.h"

class SynchronizerOptions {
public:
    unsigned int queue_size_ = 10;
    std::vector<std::string> frame_id_vec_;
    double max_interval_duration_ = std::numeric_limits<double>::max();
};

class Synchronizer {
public:
    using Ptr = std::shared_ptr<Synchronizer>;

    explicit Synchronizer(const SynchronizerOptions& options = SynchronizerOptions());

    void addData(const SyncSensor::Ptr& msg_ptr);

    void regSyncCallback(const std::function<void(const std::vector<SyncSensor::Ptr>& msg_ptr_vec)>& cb);

    ~Synchronizer() { stop(); }

    void start();

    void stop();

private:
    std::string name() { return "Synchronizer"; }

    void core();

    void process();

    void getCandidateEnd(std::string& end_frame_id, double& end_time);

    void getCandidateStart(std::string& start_frame_id, double& start_time);

    void getCandidateBoundary(std::string& frame_id, double& time, bool end);

    void dequeDeleteFront(const std::string& frame_id);

    void dequeMoveFrontToPast(const std::string& frame_id);

    void makeCandidate();

    void publishCandidate();

    void recoverAndDelete();

    void getVirtualCandidateStart(std::string& start_frame_id, double& start_time);

    void getVirtualCandidateEnd(std::string& end_frame_id, double& end_time);

    void getVirtualCandidateBoundary(std::string& frame_id, double& time, bool end);

    double getVirtualTime(const std::string& frame_id);

    void recover(const std::string& frame_id, int num_message);

    void recover();

    void checkInterMessageBound(const std::string& frame_id);

    SynchronizerOptions options_;
    std::unique_ptr<std::thread> thread_ptr_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool run_flag_ = false;
    std::deque<SyncSensor::Ptr> msg_deque_;
    std::vector<std::function<void(const std::vector<SyncSensor::Ptr> &msg_ptr_vec)>> sync_cb_list_;
    uint32_t queue_size_;
    std::map<std::string, std::deque<Synchronizer::Ptr>> sensor_msg_map_;
    std::map<std::string, std::vector<Synchronizer::Ptr>> sensor_msg_past_;
    std::map<std::string, bool> has_dropped_messages_;
    std::map<std::string, double> inter_message_lower_bounds_;
    uint32_t num_non_empty_deques_;
    std::string pivot_;
    const std::string NO_PIVOT = "_no_pivot";
    std::vector<SyncSensor::Ptr> candidate_;
    double candidate_start_, candidate_end_, pivot_time_;
};
#endif  // SYNCHRONIZER_H_
