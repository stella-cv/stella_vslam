#ifndef OPENVSLAM_DATA_ORB_PARAMS_DATABASE_H
#define OPENVSLAM_DATA_ORB_PARAMS_DATABASE_H

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace openvslam {

namespace feature {
class orb_params;
} // namespace feature

namespace data {

class orb_params_database {
public:
    explicit orb_params_database(feature::orb_params* curr_orb_params);

    ~orb_params_database();

    feature::orb_params* get_orb_params(const std::string& orb_params_name) const;

    void from_json(const nlohmann::json& json_orb_params);

    nlohmann::json to_json() const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! pointer to the orb_params which used in the current tracking
    //! (NOTE: the object is owned by config class,
    //!  thus this class does NOT delete the object of curr_orb_params_)
    feature::orb_params* curr_orb_params_ = nullptr;
    //! database (key: orb_params name, value: pointer of feature::orb_params)
    //! (NOTE: tracking orb_params must NOT be contained in the database)
    std::unordered_map<std::string, feature::orb_params*> database_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_ORB_PARAMS_DATABASE_H
