#include "stella_vslam/data/orb_params_database.h"
#include "stella_vslam/feature/orb_params.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace stella_vslam {
namespace data {

orb_params_database::orb_params_database(feature::orb_params* curr_orb_params)
    : curr_orb_params_(curr_orb_params) {
    spdlog::debug("CONSTRUCT: data::orb_params_database");
}

orb_params_database::~orb_params_database() {
    for (const auto& name_orb_params : database_) {
        const auto& orb_params_name = name_orb_params.first;
        const auto orb_params = name_orb_params.second;

        // Since curr_orb_params is held in the config class, do not delete curr_orb_params here
        if (orb_params->name_ == curr_orb_params_->name_) {
            continue;
        }
        delete database_.at(orb_params_name);
        database_.at(orb_params_name) = nullptr;
    }
    database_.clear();

    spdlog::debug("DESTRUCT: data::orb_params_database");
}

feature::orb_params* orb_params_database::get_orb_params(const std::string& orb_params_name) const {
    std::lock_guard<std::mutex> lock(mtx_database_);
    if (orb_params_name == curr_orb_params_->name_) {
        return curr_orb_params_;
    }
    else {
        assert(database_.count(orb_params_name));
        return database_.at(orb_params_name);
    }
}

void orb_params_database::from_json(const nlohmann::json& json_orb_params) {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("decoding {} orb_params to load", json_orb_params.size());
    for (const auto& json_id_orb_params : json_orb_params.items()) {
        const auto& orb_params_name = json_id_orb_params.key();
        const auto& json_orb_params = json_id_orb_params.value();

        if (orb_params_name == curr_orb_params_->name_) {
            spdlog::info("orb_params \"{}\" found in JSON", orb_params_name);
            if (std::abs(curr_orb_params_->scale_factor_ - json_orb_params.at("scale_factor").get<float>()) < 1e-6
                && curr_orb_params_->num_levels_ - json_orb_params.at("num_levels").get<unsigned int>() == 0
                && curr_orb_params_->ini_fast_thr_ - json_orb_params.at("ini_fast_threshold").get<unsigned int>() == 0
                && curr_orb_params_->min_fast_thr_ - json_orb_params.at("min_fast_threshold").get<unsigned int>() == 0) {
                continue;
            }
            else {
                throw std::runtime_error("The same feature extraction settings exist with the same name. Please give them different names.");
            }
        }

        // This orb_params is used for keyframes on the database.
        spdlog::info("load a orb_params \"{}\" from JSON", orb_params_name);

        auto orb_params = new feature::orb_params(orb_params_name,
                                                  json_orb_params.at("scale_factor").get<float>(),
                                                  json_orb_params.at("num_levels").get<unsigned int>(),
                                                  json_orb_params.at("ini_fast_threshold").get<unsigned int>(),
                                                  json_orb_params.at("min_fast_threshold").get<unsigned int>());
        assert(!database_.count(orb_params_name));
        database_[orb_params_name] = orb_params;
    }
}

nlohmann::json orb_params_database::to_json() const {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("encoding {} orb_params to store", database_.size() + 1);
    std::map<std::string, nlohmann::json> orb_params_jsons;
    orb_params_jsons[curr_orb_params_->name_] = curr_orb_params_->to_json();
    for (const auto& name_orb_params : database_) {
        const auto& orb_params_name = name_orb_params.first;
        const auto orb_params = name_orb_params.second;
        orb_params_jsons[orb_params_name] = orb_params->to_json();
    }
    return orb_params_jsons;
}

} // namespace data
} // namespace stella_vslam
