#ifndef STELLA_VSLAM_DATA_PARAMS_DATABASE_H
#define STELLA_VSLAM_DATA_PARAMS_DATABASE_H

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {

namespace feature {
struct params;
} // namespace feature

namespace data {

class params_database {
public:
    explicit params_database();

    ~params_database();

    void add_params(feature::params* params);

    feature::params* get_params(const std::string& params_name) const;

    void from_json(const nlohmann::json& json_params);

    nlohmann::json to_json() const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! database (key: params name, value: pointer of feature::params)
    std::unordered_map<std::string, feature::params*> params_database_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_PARAMS_DATABASE_H
