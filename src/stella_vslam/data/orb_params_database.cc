#include "stella_vslam/data/orb_params_database.h"
#include "stella_vslam/feature/orb_params.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace stella_vslam {
namespace data {

orb_params_database::orb_params_database() {
    spdlog::debug("CONSTRUCT: data::orb_params_database");
}

orb_params_database::~orb_params_database() {
    for (const auto& name_orb_params : orb_params_database_) {
        const auto& orb_params_name = name_orb_params.first;
        delete orb_params_database_.at(orb_params_name);
        orb_params_database_.at(orb_params_name) = nullptr;
    }
    orb_params_database_.clear();

    spdlog::debug("DESTRUCT: data::orb_params_database");
}

void orb_params_database::add_orb_params(feature::orb_params* orb_params) {
    std::lock_guard<std::mutex> lock(mtx_database_);
    assert(orb_params != nullptr);
    assert(orb_params_database_.count(orb_params->name_) == 0);
    orb_params_database_.emplace(orb_params->name_, orb_params);
}

feature::orb_params* orb_params_database::get_orb_params(const std::string& orb_params_name) const {
    std::lock_guard<std::mutex> lock(mtx_database_);
    if (orb_params_database_.count(orb_params_name)) {
        return orb_params_database_.at(orb_params_name);
    }
    else {
        return nullptr;
    }
}

void orb_params_database::from_json(const nlohmann::json& json_orb_params) {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("decoding {} orb_params to load", json_orb_params.size());
    for (const auto& json_id_orb_params : json_orb_params.items()) {
        const auto& orb_params_name = json_id_orb_params.key();
        const auto& json_orb_params = json_id_orb_params.value();

        if (orb_params_database_.count(orb_params_name)) {
            spdlog::info("The feature extraction settings with the same name (\"{}\") already existed in database.", orb_params_name);
            auto orb_params_in_database = orb_params_database_.at(orb_params_name);
            if (std::abs(orb_params_in_database->scale_factor_ - json_orb_params.at("scale_factor").get<float>()) < 1e-6
                && orb_params_in_database->num_levels_ - json_orb_params.at("num_levels").get<unsigned int>() == 0
                && orb_params_in_database->ini_fast_thr_ - json_orb_params.at("ini_fast_threshold").get<unsigned int>() == 0
                && orb_params_in_database->min_fast_thr_ - json_orb_params.at("min_fast_threshold").get<unsigned int>() == 0) {
                continue;
            }
            else {
                throw std::runtime_error("The different feature extraction settings exist with the same name. Please give them different names.");
            }
        }

        // This orb_params is used for keyframes on the database.
        spdlog::info("load a orb_params \"{}\" from JSON", orb_params_name);

        auto orb_params = new feature::orb_params(orb_params_name,
                                                  json_orb_params.at("scale_factor").get<float>(),
                                                  json_orb_params.at("num_levels").get<unsigned int>(),
                                                  json_orb_params.at("ini_fast_threshold").get<unsigned int>(),
                                                  json_orb_params.at("min_fast_threshold").get<unsigned int>());
        assert(!orb_params_database_.count(orb_params_name));
        orb_params_database_[orb_params_name] = orb_params;
    }
}

nlohmann::json orb_params_database::to_json() const {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("encoding {} orb_params to store", orb_params_database_.size());
    std::map<std::string, nlohmann::json> orb_params_jsons;
    for (const auto& name_orb_params : orb_params_database_) {
        const auto& orb_params_name = name_orb_params.first;
        const auto orb_params = name_orb_params.second;
        orb_params_jsons[orb_params_name] = orb_params->to_json();
    }
    return orb_params_jsons;
}

} // namespace data
} // namespace stella_vslam
