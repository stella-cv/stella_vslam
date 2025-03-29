#include "stella_vslam/data/params_database.h"
#include "stella_vslam/feature/params.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace stella_vslam {
namespace data {

params_database::params_database() {
    spdlog::debug("CONSTRUCT: data::params_database");
}

params_database::~params_database() {
    for (const auto& name_params : params_database_) {
        const auto& params_name = name_params.first;
        delete params_database_.at(params_name);
        params_database_.at(params_name) = nullptr;
    }
    params_database_.clear();

    spdlog::debug("DESTRUCT: data::params_database");
}

void params_database::add_params(feature::params* params) {
    std::lock_guard<std::mutex> lock(mtx_database_);
    assert(params != nullptr);
    assert(params_database_.count(params->name_) == 0);
    params_database_.emplace(params->name_, params);
}

feature::params* params_database::get_params(const std::string& params_name) const {
    std::lock_guard<std::mutex> lock(mtx_database_);
    if (params_database_.count(params_name)) {
        return params_database_.at(params_name);
    }
    else {
        return nullptr;
    }
}

void params_database::from_json(const nlohmann::json& json_params) {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("decoding {} params to load", json_params.size());
    for (const auto& json_id_params : json_params.items()) {
        const auto& params_name = json_id_params.key();
        const auto& json_params = json_id_params.value();

        if (params_database_.count(params_name)) {
            spdlog::info("The feature extraction settings with the same name (\"{}\") already existed in database.", params_name);
            auto params_in_database = params_database_.at(params_name);
            if (std::abs(params_in_database->scale_factor_ - json_params.at("scale_factor").get<float>()) < 1e-6
                && params_in_database->num_levels_ - json_params.at("num_levels").get<unsigned int>() == 0
                && params_in_database->ini_fast_thr_ - json_params.at("ini_fast_threshold").get<unsigned int>() == 0
                && params_in_database->min_fast_thr_ - json_params.at("min_fast_threshold").get<unsigned int>() == 0) {
                continue;
            }
            else {
                throw std::runtime_error("The different feature extraction settings exist with the same name. Please give them different names.");
            }
        }

        // This params is used for keyframes on the database.
        spdlog::info("load a params \"{}\" from JSON", params_name);

        auto params = new feature::params(params_name,
                                                  json_params.at("scale_factor").get<float>(),
                                                  json_params.at("num_levels").get<unsigned int>(),
                                                  json_params.at("ini_fast_threshold").get<unsigned int>(),
                                                  json_params.at("min_fast_threshold").get<unsigned int>());
        assert(!params_database_.count(params_name));
        params_database_[params_name] = params;
    }
}

nlohmann::json params_database::to_json() const {
    std::lock_guard<std::mutex> lock(mtx_database_);

    spdlog::info("encoding {} params to store", params_database_.size());
    std::map<std::string, nlohmann::json> params_jsons;
    for (const auto& name_params : params_database_) {
        const auto& params_name = name_params.first;
        const auto params = name_params.second;
        params_jsons[params_name] = params->to_json();
    }
    return params_jsons;
}

} // namespace data
} // namespace stella_vslam
