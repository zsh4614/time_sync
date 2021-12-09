#include "synchronizer.h"

Synchronizer::Synchronizer(const SynchronizerOptions& options) {
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
        sensor_msg_past_[tmp_frame_id] = tmp_msg_vector;
        has_dropped_messages_[tmp_frame_id] = false;
        inter_message_lower_bounds_[tmp_frame_id] = 0;
    }
}

void Synchronizer::addData(const SyncSensor::Ptr& msg_ptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    msg_deque_.emplace_back(msg_ptr);
    cv_.notify_all();
}

void Synchronizer::regSyncCallback(const std::function<void(const std::vector<SyncSensor::Ptr>& msg_ptr_vec)>& cb) {
    std::unique_lock<std::mutex> lock(mutex_);
    sync_cb_list_.emplace_back(cb);
}

void Synchronizer::start() {
    run_flag_ = true;
    if (thread_ptr_ == nullptr) {
        thread_ptr_.reset(new std::thread(&Synchronizer::core, this));
    }
}

void Synchronizer::stop() {
    run_flag_ = false;
    if (thread_ptr_ != nullptr) {
        cv_.notify_all();
        if (thread_ptr_->joinable()) {
            thread_ptr_->join();
        }
    }
}

void Synchronizer::core() {
    while (run_flag_) {
        std::deque<SyncSensor::Ptr> data;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (msg_deque_.empty()) {
                cv_.wait(lock);
                if (!run_flag_) {
                    continue;
                }
            }
            data = msg_deque_;
            msg_deque_.clear();
        }

        while (!data.empty()) {
            auto msg_ptr = data.front();
            data.pop_front();
            if (sensor_msg_map_.find(msg_ptr->getFrameID()) == sensor_msg_map_.end()) {
                std::cout << "receive unknown frame_id: " << msg_ptr->getFrameID();
                continue;
            }
            auto &deque = sensor_msg_map_.at(msg_ptr->getFrameID());
            deque.emplace_back(msg_ptr);

            if (deque.size() == static_cast<size_t>(1)) {
                ++num_non_empty_deques_;
                if (num_non_empty_deques_ == static_cast<uint32_t>(sensor_msg_map_.size())) {
                    process();
                }
            } else {
                checkInterMessageBound(msg_ptr->getFrameID());
            }

            auto &past = sensor_msg_past_.at(msg_ptr->getFrameID());
            if (past.size() + deque.size() > queue_size_) {
                num_non_empty_deques_ = 0;
                recover();
                if (deque.empty()) {
                    throw "deque is empty!";
                }
                deque.pop_front();
                has_dropped_messages_.at(msg_ptr->getFrameID()) = true;
                if (pivot_ != NO_PIVOT) {
                    candidate_.clear();
                    pivot_ = NO_PIVOT;
                    process();
                }

            }
        }
    }
}

void Synchronizer::recover() {
    for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
        auto &deque = sensor_msg_map_.at(itr->first);
        auto &vector = sensor_msg_past_.at(itr->first);
        while (!vector.empty()) {
            deque.push_front(vector.back());
            vector.pop_back();
        }
        if (!deque.empty()) {
            ++num_non_empty_deques_;
        }
    }
}

void Synchronizer::process() {
    while (num_non_empty_deques_ == static_cast<uint32_t>(sensor_msg_map_.size())) {
        double start_time, end_time;
        std::string start_frame_id, end_frame_id;
        getCandidateStart(start_frame_id, start_time);
        getCandidateEnd(end_frame_id, end_time);

        for (auto itr = has_dropped_messages_.begin(); itr != has_dropped_messages_.end(); ++itr) {
            if (itr->first != end_frame_id) {
                itr->second = false;
            }
        }
        if (pivot_ == NO_PIVOT) {
            if (end_time - start_time > options_.max_interval_duration_) {
                dequeDeleteFront(start_frame_id);
                continue;
            }
            if (has_dropped_messages_.at(end_frame_id)) {
                dequeDeleteFront(start_frame_id);
                continue;
            }
            makeCandidate();
            candidate_start_ = start_time;
            candidate_end_ = end_time;
            pivot_ = end_frame_id;
            pivot_time_ = end_time;
            dequeMoveFrontToPast(start_frame_id);
        } else {
            if ((end_time - candidate_end_) >= (start_time - candidate_start_)) {
                dequeMoveFrontToPast(start_frame_id);
            } else {
                makeCandidate();
                candidate_start_ = start_time;
                candidate_end_ = end_time;
                dequeMoveFrontToPast(start_frame_id);
            }
        }

        if (pivot_ == NO_PIVOT) {
            throw "pivot equal to NO_PIVOT!";
        }

        if (start_frame_id == pivot_) {
            publishCandidate();
        } else if ((end_time - candidate_end_) >= (pivot_time_ - candidate_start_)) {
            publishCandidate();
        } else if (num_non_empty_deques_ < static_cast<uint32_t>(sensor_msg_map_.size())) {
            uint32_t num_non_empty_deques_before_virtual_search = num_non_empty_deques_;
            std::map<std::string, int> num_virtual_moves_map;
            for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
                num_virtual_moves_map[itr->first] = 0;
            }
            while (1) {
                double start_time, end_time;
                std::string start_frame_id, end_frame_id;
                getVirtualCandidateStart(start_frame_id, start_time);
                getVirtualCandidateEnd(end_frame_id, end_time);
                if ((end_time - candidate_end_) >= (pivot_time_ - candidate_start_)) {
                    publishCandidate();
                    break;
                }
                if ((end_time - candidate_end_) < (start_time - candidate_start_)) {
                    num_non_empty_deques_ = 0;
                    for (auto itr = num_virtual_moves_map.begin(); itr != num_virtual_moves_map.end(); ++itr) {
                        recover(itr->first, itr->second);
                    }
                    if (num_non_empty_deques_before_virtual_search != num_non_empty_deques_) {
                        throw "num_non_empty_deques_before_virtual_search not equal to num_non_empty_deques!";
                    }
                    break;
                }
                if (start_frame_id == pivot_) {
                    throw "start_frame_id equal to pivot!";
                }
                if (start_time > pivot_time_) {
                    throw "start_time larger than pivot_time!";
                }
                dequeMoveFrontToPast(start_frame_id);
                num_virtual_moves_map.at(start_frame_id)++;
            }
        }
    }
}

void Synchronizer::getVirtualCandidateStart(std::string &start_frame_id, double &start_time) {
    return getVirtualCandidateBoundary(start_frame_id, start_time, false);
}

void Synchronizer::getVirtualCandidateEnd(std::string &end_frame_id, double &end_time) {
    return getVirtualCandidateBoundary(end_frame_id, end_time, true);
}

void Synchronizer::getVirtualCandidateBoundary(std::string &index_frame_id, double &time, bool end) {
    std::map<std::string, double> virtual_time_map;
    for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
        const auto &frame_id = itr->first;
        virtual_time_map[frame_id] = getVirtualTime(frame_id);
    }
    auto begin_itr = virtual_time_map.begin();
    index_frame_id = begin_itr->first;
    time = begin_itr->second;
    for (auto itr = virtual_time_map.begin(); itr != virtual_time_map.end(); ++itr) {
        if ((itr->second < time) ^ end) {
            time = itr->second;
            index_frame_id = itr->first;
        }
    }
}

double Synchronizer::getVirtualTime(const std::string &frame_id) {
    if (pivot_ == NO_PIVOT) {
        throw "pivot equal to NO_PIVOT!";
    }

    auto &vector = sensor_msg_past_.at(frame_id);
    auto &deque = sensor_msg_map_.at(frame_id);
    if (deque.empty()) {
        if (vector.empty()) {
            throw "past is empty!";
        }
        const auto &last_msg_time = vector.back()->getTimestamp();
        double msg_time_lower_bound = last_msg_time + inter_message_lower_bounds_.at(frame_id);
        if (msg_time_lower_bound > pivot_time_) {
            return msg_time_lower_bound;
        }
        return pivot_time_;
    }
    return deque.front()->getTimestamp();
}

void Synchronizer::recover(const std::string &frame_id, int num_message) {
    auto &deque = sensor_msg_map_.at(frame_id);
    auto &vector = sensor_msg_past_.at(frame_id);

    if (num_message > static_cast<int>(vector.size())) {
        throw "num_message larger than past size!";
    }
    while (num_message > 0) {
        deque.push_front(vector.back());
        vector.pop_back();
        num_message--;
    }

    if (!deque.empty()) {
        ++num_non_empty_deques_;
    }
}

void Synchronizer::publishCandidate() {
    for (const auto &cb : sync_cb_list_) {
        cb(candidate_);
    }
    candidate_.clear();
    pivot_ = NO_PIVOT;
    num_non_empty_deques_ = 0;
    recoverAndDelete();
}

void Synchronizer::recoverAndDelete() {
    for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
        const auto &frame_id = itr->first;
        auto &deque = sensor_msg_map_.at(frame_id);
        auto &vector = sensor_msg_past_.at(frame_id);
        while (!vector.empty()) {
            deque.push_front(vector.back());
            vector.pop_back();
        }
        if (deque.empty()) {
            throw "deque is empty!";
        }
        deque.pop_front();
        if (!deque.empty()) {
            ++num_non_empty_deques_;
        }
    }
}

void Synchronizer::dequeMoveFrontToPast(const std::string& index_frame_id) {
    auto &deque = sensor_msg_map_.at(index_frame_id);
    auto &vector = sensor_msg_past_.at(index_frame_id);
    if (deque.empty()) {
        throw  "deque is empty!";
    }
    vector.push_back(deque.front());
    deque.pop_front();
    if (deque.empty()) {
        --num_non_empty_deques_;
    }
}

void Synchronizer::makeCandidate() {
    candidate_.clear();
    for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
        candidate_.emplace_back(itr->second.front());
    }
    for (auto itr = sensor_msg_past_.begin(); itr != sensor_msg_past_.end(); ++itr) {
        itr->second.clear();
    }
}

void Synchronizer::dequeDeleteFront(const std::string &frame_id) {
    auto &deque = sensor_msg_map_.at(frame_id);
    if (deque.empty()) {
        throw "deque is empty!";
    }
    deque.pop_front();
    if (deque.empty()) {
        --num_non_empty_deques_;
    }
}

void Synchronizer::getCandidateStart(std::string& start_frame_id, double& start_time) {
    return getCandidateBoundary(start_frame_id, start_time, false);
}

void Synchronizer::getCandidateEnd(std::string& end_frame_id, double& end_time) {
    return getCandidateBoundary(end_frame_id, end_time, true);
}

void Synchronizer::getCandidateBoundary(std::string& frame_id, double& time, bool end) {
    auto first_itr = sensor_msg_map_.begin();
    time = first_itr->second.front()->getTimestamp();
    frame_id = first_itr->second.front()->getFrameID();

    for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end(); ++itr) {
        if ((itr->second.front()->getTimestamp() < time) ^ end) {
            time = itr->second.front()->getTimestamp();
            frame_id = itr->second.front()->getFrameID();
        }
    }
}

void Synchronizer::checkInterMessageBound(const std::string &frame_id) {
    auto &deque = sensor_msg_map_.at(frame_id);
    auto &vector = sensor_msg_past_.at(frame_id);
    if (deque.empty()) {
        throw "deque is empty!";
    }
    const auto &msg_time = deque.back()->getTimestamp();
    double prev_msg_time;
    if (deque.size() == static_cast<size_t>(1)) {
        if (vector.empty()) {
            return;
        }
        prev_msg_time = vector.back()->getTimestamp();
    } else {
        prev_msg_time = deque[deque.size() - 2]->getTimestamp();
    }
    if (msg_time < prev_msg_time) {
        std::cout << "messages of frame_id " << frame_id << " arrived out of order!" << std::endl;
    } else if ((msg_time - prev_msg_time) < inter_message_lower_bounds_.at(frame_id)) {
        std::cout << "messages of frame_id " << frame_id << " arrived closer ( " <<
            (msg_time - prev_msg_time)
            << " ) than the lower bound you provided ( " << inter_message_lower_bounds_.at(frame_id)
            << " )" << std::endl;
    }
}