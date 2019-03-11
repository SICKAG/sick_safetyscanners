#include <sick_safetyscanners/datastructure/DeviceStatus.h>

// TODO
namespace sick {
namespace datastructure {

DeviceStatus::DeviceStatus() {}
uint8_t DeviceStatus::getVersionIndicator() const {
  return m_version_indicator;
}
void DeviceStatus::setVersionIndicator(uint8_t version_indicator) {
  m_version_indicator = version_indicator;
}

uint8_t DeviceStatus::getVersionMajorVersion() const {
  return m_version_major_version;
}
void DeviceStatus::setVersionMajorVersion(uint8_t major_version) {
  m_version_major_version = major_version;
}

uint8_t DeviceStatus::getVersionMinorVersion() const {
  return m_version_minor_version;
}
void DeviceStatus::setVersionMinorVersion(uint8_t minor_version) {
  m_version_minor_version = minor_version;
}

uint8_t DeviceStatus::getVersionRelease() const { return m_version_release; }
void DeviceStatus::setVersionRelease(uint8_t version_release) {
  m_version_release = version_release;
}

uint8_t DeviceStatus::getDeviceState() const { return m_device_state; }
void DeviceStatus::setDeviceState(uint8_t device_state) {
  m_device_state = device_state;
}

uint8_t DeviceStatus::getConfigState() const { return m_config_state; }
void DeviceStatus::setConfigState(uint8_t config_state) {
  m_config_state = config_state;
}

uint8_t DeviceStatus::getApplicationState() const {
  return m_application_state;
}
void DeviceStatus::setApplicationState(uint8_t application_state) {
  m_application_state = application_state;
}

uint32_t DeviceStatus::getPowerOnCount() const { return m_power_on_count; }
void DeviceStatus::setPowerOnCount(uint32_t power_on_count) {
  m_power_on_count = power_on_count;
}

uint32_t DeviceStatus::getCurrentTimeTime() const { return m_time_time; }
void DeviceStatus::setCurrentTimeTime(uint32_t time_time) {
  m_time_time = time_time;
}

uint16_t DeviceStatus::getCurrentTimeDate() const { return m_time_date; }
void DeviceStatus::setCurrentTimeDate(uint16_t time_date) {
  m_time_date = time_date;
}

uint32_t DeviceStatus::getErrorInfoCode() const { return m_error_info_code; }
void DeviceStatus::setErrorInfoCode(uint32_t error_info_code) {
  m_error_info_code = error_info_code;
}

uint32_t DeviceStatus::getErrorInfoTime() const { return m_error_info_time; }
void DeviceStatus::setErrorInfoTime(uint32_t error_info_time) {
  m_error_info_time = error_info_time;
}

uint16_t DeviceStatus::getErrorInfoDate() const { return m_error_info_date; }
void DeviceStatus::setErrorInfoDate(uint16_t error_info_date) {
  m_error_info_date = error_info_date;
}

}  // namespace datastructure
}  // namespace sick
