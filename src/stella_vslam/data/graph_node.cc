#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/graph_node.h"
#include "stella_vslam/data/landmark.h"

namespace {
struct {
    bool operator()(const std::pair<unsigned int, std::shared_ptr<stella_vslam::data::keyframe>>& a, const std::pair<unsigned int, std::shared_ptr<stella_vslam::data::keyframe>>& b) {
        return a.first < b.first || (a.first == b.first && a.second != nullptr && (b.second == nullptr || a.second->id_ < b.second->id_));
    }
} cmp_weight_keyfrm_pairs;
} // namespace

namespace stella_vslam {
namespace data {

graph_node::graph_node(std::shared_ptr<keyframe>& keyfrm)
    : owner_keyfrm_(keyfrm), spanning_parent_is_not_set_(true) {}

void graph_node::add_connection(const std::shared_ptr<keyframe>& keyfrm, const unsigned int weight) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool need_update = false;
    if (!connected_keyfrms_and_weights_.count(keyfrm)) {
        // if `keyfrm` not exists
        connected_keyfrms_and_weights_[keyfrm] = weight;
        need_update = true;
    }
    else if (connected_keyfrms_and_weights_.at(keyfrm) != weight) {
        // if the weight is updated
        connected_keyfrms_and_weights_.at(keyfrm) = weight;
        need_update = true;
    }

    if (need_update) {
        update_covisibility_orders_impl();
    }
}

void graph_node::erase_connection(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool need_update = false;
    if (connected_keyfrms_and_weights_.count(keyfrm)) {
        connected_keyfrms_and_weights_.erase(keyfrm);
        need_update = true;
    }

    if (need_update) {
        update_covisibility_orders_impl();
    }
}

void graph_node::erase_all_connections() {
    // remote myself from the connected keyframes
    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        if (keyfrm_and_weight.first.expired()) {
            continue;
        }
        keyfrm_and_weight.first.lock()->graph_node_->erase_connection(owner_keyfrm_.lock());
    }
    // remove the buffers
    connected_keyfrms_and_weights_.clear();
    ordered_covisibilities_.clear();
    ordered_weights_.clear();
}

void graph_node::update_connections() {
    const auto owner_keyfrm = owner_keyfrm_.lock();
    const auto landmarks = owner_keyfrm->get_landmarks();

    id_ordered_map<std::weak_ptr<keyframe>, unsigned int> keyfrm_weights;

    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        const auto observations = lm->get_observations();

        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto locked_keyfrm = keyfrm.lock();

            if (locked_keyfrm->graph_node_->spanning_parent_is_not_set_ && locked_keyfrm->id_ != 0) {
                continue;
            }
            if (locked_keyfrm->id_ == owner_keyfrm->id_) {
                continue;
            }
            // count up weight of `keyfrm`
            keyfrm_weights[keyfrm]++;
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    unsigned int max_weight = 0;
    std::shared_ptr<keyframe> nearest_covisibility = nullptr;

    // vector for sorting
    std::vector<std::pair<unsigned int, std::shared_ptr<keyframe>>> weight_covisibility_pairs;
    weight_covisibility_pairs.reserve(keyfrm_weights.size());
    for (const auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first.lock();
        const auto weight = keyfrm_weight.second;

        // nearest_covisibility with greatest id_ will be selected if weights are the same due to ordering of keyfrm_weights.
        if (max_weight <= weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }

        if (weight_thr_ < weight) {
            weight_covisibility_pairs.emplace_back(std::make_pair(weight, keyfrm));
        }
    }
    // add ONE node at least
    if (weight_covisibility_pairs.empty()) {
        weight_covisibility_pairs.emplace_back(std::make_pair(max_weight, nearest_covisibility));
    }

    // add connection from the covisibility to myself
    for (const auto& weight_covisibility : weight_covisibility_pairs) {
        auto covisibility = weight_covisibility.second;
        const auto weight = weight_covisibility.first;
        covisibility->graph_node_->add_connection(owner_keyfrm, weight);
    }

    // sort with weights and keyframe IDs for consistency; IDs are also in reverse order
    // to match selection of nearest_covisibility.
    std::sort(weight_covisibility_pairs.rbegin(), weight_covisibility_pairs.rend(), cmp_weight_keyfrm_pairs);

    decltype(ordered_covisibilities_) ordered_covisibilities;
    ordered_covisibilities.reserve(weight_covisibility_pairs.size());
    decltype(ordered_weights_) ordered_weights;
    ordered_weights.reserve(weight_covisibility_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_covisibility_pairs) {
        ordered_covisibilities.push_back(weight_keyfrm_pair.second);
        ordered_weights.push_back(weight_keyfrm_pair.first);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);

        connected_keyfrms_and_weights_ = decltype(connected_keyfrms_and_weights_)(keyfrm_weights.begin(), keyfrm_weights.end());

        ordered_covisibilities_ = ordered_covisibilities;
        ordered_weights_ = ordered_weights;

        if (spanning_parent_is_not_set_ && owner_keyfrm->id_ != 0) {
            // set the parent of spanning tree
            assert(nearest_covisibility->id_ == ordered_covisibilities.front().lock()->id_);
            spanning_parent_ = nearest_covisibility;
            nearest_covisibility->graph_node_->add_spanning_child(owner_keyfrm);
            spanning_parent_is_not_set_ = false;
        }
    }
}

void graph_node::update_covisibility_orders() {
    std::lock_guard<std::mutex> lock(mtx_);
    update_covisibility_orders_impl();
}

void graph_node::update_covisibility_orders_impl() {
    std::vector<std::pair<unsigned int, std::shared_ptr<keyframe>>> weight_keyfrm_pairs;
    weight_keyfrm_pairs.reserve(connected_keyfrms_and_weights_.size());

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        weight_keyfrm_pairs.emplace_back(std::make_pair(keyfrm_and_weight.second, keyfrm_and_weight.first.lock()));
    }

    // sort with weights and keyframe IDs for consistency
    std::sort(weight_keyfrm_pairs.rbegin(), weight_keyfrm_pairs.rend(), cmp_weight_keyfrm_pairs);

    ordered_covisibilities_.clear();
    ordered_covisibilities_.reserve(weight_keyfrm_pairs.size());
    ordered_weights_.clear();
    ordered_weights_.reserve(weight_keyfrm_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_keyfrm_pairs) {
        ordered_covisibilities_.push_back(weight_keyfrm_pair.second);
        ordered_weights_.push_back(weight_keyfrm_pair.first);
    }
}

std::set<std::shared_ptr<keyframe>> graph_node::get_connected_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> keyfrms;

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        keyfrms.insert(keyfrm_and_weight.first.lock());
    }

    return keyfrms;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_covisibilities() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::shared_ptr<keyframe>> covisibilities;

    for (const auto& covisibility : ordered_covisibilities_) {
        if (covisibility.expired()) {
            continue;
        }
        covisibilities.push_back(covisibility.lock());
    }
    return covisibilities;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_top_n_covisibilities(const unsigned int num_covisibilities) const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::shared_ptr<keyframe>> covisibilities;
    unsigned int i = 0;
    for (const auto& covisibility : ordered_covisibilities_) {
        if (i == num_covisibilities) {
            break;
        }
        if (covisibility.expired()) {
            continue;
        }
        covisibilities.push_back(covisibility.lock());
        i++;
    }
    return covisibilities;
}

std::vector<std::shared_ptr<keyframe>> graph_node::get_covisibilities_over_weight(const unsigned int weight) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (ordered_covisibilities_.empty()) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    auto itr = std::upper_bound(ordered_weights_.begin(), ordered_weights_.end(), weight, std::greater<unsigned int>());
    if (itr == ordered_weights_.end()) {
        return std::vector<std::shared_ptr<keyframe>>();
    }
    else {
        const auto upper_bound_idx = static_cast<unsigned int>(itr - ordered_weights_.begin());
        std::vector<std::shared_ptr<keyframe>> covisibilities;
        unsigned int idx = 0;
        for (const auto& covisibility : ordered_covisibilities_) {
            if (idx == upper_bound_idx) {
                break;
            }
            idx++;
            if (covisibility.expired()) {
                continue;
            }
            covisibilities.push_back(covisibility.lock());
        }
        return covisibilities;
    }
}

unsigned int graph_node::get_weight(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (connected_keyfrms_and_weights_.count(keyfrm)) {
        return connected_keyfrms_and_weights_.at(keyfrm);
    }
    else {
        return 0;
    }
}

void graph_node::set_spanning_parent(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    assert(spanning_parent_is_not_set_);
    spanning_parent_ = keyfrm;
    spanning_parent_is_not_set_ = false;
}

std::shared_ptr<keyframe> graph_node::get_spanning_parent() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return spanning_parent_.lock();
}

void graph_node::change_spanning_parent(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_parent_ = keyfrm;
    keyfrm->graph_node_->add_spanning_child(owner_keyfrm_.lock());
}

void graph_node::add_spanning_child(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.insert(keyfrm);
}

void graph_node::erase_spanning_child(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.erase(keyfrm);
}

void graph_node::recover_spanning_connections() {
    std::lock_guard<std::mutex> lock(mtx_);

    // 1. find new parents for my children

    std::set<std::shared_ptr<keyframe>> new_parent_candidates;
    new_parent_candidates.insert(spanning_parent_.lock());

    while (!spanning_children_.empty()) {
        bool max_is_found = false;

        unsigned int max_weight = 0;
        std::shared_ptr<keyframe> max_weight_parent = nullptr;
        std::shared_ptr<keyframe> max_weight_child = nullptr;

        for (const auto& spanning_child : spanning_children_) {
            auto locked_spanning_child = spanning_child.lock();
            if (locked_spanning_child->will_be_erased()) {
                continue;
            }

            // get intersection between the parent candidates and the spanning-child's covisibilities
            const auto child_covisibilities = locked_spanning_child->graph_node_->get_covisibilities();
            const auto intersection = extract_intersection(new_parent_candidates, child_covisibilities);

            // find the new parent (which has the maximum weight with the spanning child) from the intersection
            for (const auto& parent_candidate : intersection) {
                const auto weight = locked_spanning_child->graph_node_->get_weight(parent_candidate);
                if (max_weight < weight) {
                    max_weight = weight;
                    max_weight_parent = parent_candidate;
                    max_weight_child = locked_spanning_child;
                    max_is_found = true;
                }
            }
        }

        if (max_is_found) {
            // update spanning tree
            max_weight_child->graph_node_->change_spanning_parent(max_weight_parent);
            spanning_children_.erase(max_weight_child);
            new_parent_candidates.insert(max_weight_child);
        }
        else {
            // cannot update anymore
            break;
        }
    }

    // set my parent as the new parent
    for (const auto& spanning_child : spanning_children_) {
        const auto child = spanning_child.lock();
        const auto parent = spanning_parent_.lock();
        child->graph_node_->change_spanning_parent(parent);
    }

    spanning_children_.clear();

    // 2. remove myself from my parent's children list

    spanning_parent_.lock()->graph_node_->erase_spanning_child(owner_keyfrm_.lock());
}

std::set<std::shared_ptr<keyframe>> graph_node::get_spanning_children() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> locked_spanning_children;
    for (const auto& keyfrm : spanning_children_) {
        locked_spanning_children.insert(keyfrm.lock());
    }
    return locked_spanning_children;
}

bool graph_node::has_spanning_child(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<bool>(spanning_children_.count(keyfrm));
}

void graph_node::add_loop_edge(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    loop_edges_.insert(keyfrm);
    // cannot erase loop edges
    owner_keyfrm_.lock()->set_not_to_be_erased();
}

std::set<std::shared_ptr<keyframe>> graph_node::get_loop_edges() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<std::shared_ptr<keyframe>> locked_loop_edges;
    for (const auto& keyfrm : loop_edges_) {
        locked_loop_edges.insert(keyfrm.lock());
    }
    return locked_loop_edges;
}

bool graph_node::has_loop_edge() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return !loop_edges_.empty();
}

template<typename T, typename U>
std::vector<std::shared_ptr<keyframe>> graph_node::extract_intersection(const T& keyfrms_1, const U& keyfrms_2) {
    std::vector<std::shared_ptr<keyframe>> intersection;
    intersection.reserve(std::min(keyfrms_1.size(), keyfrms_2.size()));
    for (const auto keyfrm_1 : keyfrms_1) {
        for (const auto keyfrm_2 : keyfrms_2) {
            if (*keyfrm_1 == *keyfrm_2) {
                intersection.push_back(keyfrm_1);
            }
        }
    }
    return intersection;
}

} // namespace data
} // namespace stella_vslam
