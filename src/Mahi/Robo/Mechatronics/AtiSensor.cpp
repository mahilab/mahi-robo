#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util/System.hpp>
#include <fstream>

using namespace mahi::util;

namespace mahi {
namespace robo {

namespace {

inline double sum_prod(const std::array<double, 6>& array1, const std::array<double, 6>& array2) {
    return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2] +
           array1[3] * array2[3] + array1[4] * array2[4] + array1[5] * array2[5];
}

std::string xml_str(const std::string& file_str, const std::string& key) {
    std::size_t find_pos  = file_str.find(key);
    std::size_t start_pos = find_pos + key.size() + 1;
    std::size_t end_pos   = file_str.find("\"", start_pos);
    std::size_t str_size  = end_pos - start_pos;
    return file_str.substr(start_pos, str_size);
}

std::array<double, 6> get_values(const std::string& str) {
    std::array<double, 6> values;
    std::istringstream    iss(str);
    std::size_t           i = 0;
    for (std::string s; iss >> s;) {
        values[i] = std::stod(s);
        i++;
    }
    return values;
}

}  // namespace

AtiSensor::AtiSensor()
{
    bias_.fill(0);
    bSTG_.fill(0);

}

AtiSensor::AtiSensor(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                     const double* ch4, const double* ch5, const std::string& filepath) :
    channels_({ch0, ch1, ch2, ch3, ch4, ch5}) 
{
    load_calibration(filepath);
}

AtiSensor::AtiSensor(const double* ch0, const double* ch1, const double* ch2, const double* ch3,
                     const double* ch4, const double* ch5, Calibration calibration) :
    channels_({ch0, ch1, ch2, ch3, ch4, ch5}), calibration_(calibration) {}

void AtiSensor::set_channels(const double* ch0, const double* ch1, const double* ch2,
                             const double* ch3, const double* ch4, const double* ch5) {
    channels_ = {ch0, ch1, ch2, ch3, ch4, ch5};
}

bool AtiSensor::load_calibration(const std::string& filepath) {
    std::string   tidy_filepath = util::tidy_path(filepath, true);
    std::ifstream file;
    file.open(tidy_filepath);
    if (file.is_open()) {
        // convert file to string
        std::string file_str((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());
        std::string ver = xml_str(file_str, "CalFileVersion=");
        if (ver == "1.1") {
            // load calibration values
            calibration_.Fx = get_values(xml_str(file_str, "<UserAxis Name=\"Fx\" values="));
            calibration_.Fy = get_values(xml_str(file_str, "<UserAxis Name=\"Fy\" values="));
            calibration_.Fz = get_values(xml_str(file_str, "<UserAxis Name=\"Fz\" values="));
            calibration_.Tx = get_values(xml_str(file_str, "<UserAxis Name=\"Tx\" values="));
            calibration_.Ty = get_values(xml_str(file_str, "<UserAxis Name=\"Ty\" values="));
            calibration_.Tz = get_values(xml_str(file_str, "<UserAxis Name=\"Tz\" values="));
            LOG(Info) << "Loaded ATI sensor calibration file \"" << filepath << "\"";
            return true;
        } else
            LOG(Error) << "Unable to load ATI sensor calibration file \"" << filepath
                       << "\". Unsupported CalFileVersion " << ver;
    } else
        LOG(Error) << "Unable to find ATI sensor calibration file \"" << filepath << "\"";

    return false;
}

void AtiSensor::set_calibration(Calibration calibration_matrix) {
    calibration_ = calibration_matrix;
}

void AtiSensor::zero() {
    bias_[0] = *channels_[0];
    bias_[1] = *channels_[1];
    bias_[2] = *channels_[2];
    bias_[3] = *channels_[3];
    bias_[4] = *channels_[4];
    bias_[5] = *channels_[5];
}

double AtiSensor::get_force(Axis axis) {
    update_biased_voltages();
    switch (axis) {
        case AxisX: return sum_prod(calibration_.Fx, bSTG_);
        case AxisY: return sum_prod(calibration_.Fy, bSTG_);
        case AxisZ: return sum_prod(calibration_.Fz, bSTG_);
        default: return 0.0;
    }
}

std::vector<double> AtiSensor::get_forces() {
    update_biased_voltages();
    forces_[0] = sum_prod(calibration_.Fx, bSTG_);
    forces_[1] = sum_prod(calibration_.Fy, bSTG_);
    forces_[2] = sum_prod(calibration_.Fz, bSTG_);
    return forces_;
}

double AtiSensor::get_torque(Axis axis) {
    update_biased_voltages();
    switch (axis) {
        case AxisX: return sum_prod(calibration_.Tx, bSTG_);
        case AxisY: return sum_prod(calibration_.Ty, bSTG_);
        case AxisZ: return sum_prod(calibration_.Tz, bSTG_);
        default: return 0.0;
    }
}

std::vector<double> AtiSensor::get_torques() {
    update_biased_voltages();
    torques_[0] = sum_prod(calibration_.Tx, bSTG_);
    torques_[1] = sum_prod(calibration_.Ty, bSTG_);
    torques_[2] = sum_prod(calibration_.Tz, bSTG_);
    return torques_;
}

void AtiSensor::update_biased_voltages() {
    bSTG_[0] = *channels_[0] - bias_[0];
    bSTG_[1] = *channels_[1] - bias_[1];
    bSTG_[2] = *channels_[2] - bias_[2];
    bSTG_[3] = *channels_[3] - bias_[3];
    bSTG_[4] = *channels_[4] - bias_[4];
    bSTG_[5] = *channels_[5] - bias_[5];
}

}  // namespace robo
}  // namespace mahi
