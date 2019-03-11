#ifndef DEVICESTATUSVARIABLECOMMAND_H
#define DEVICESTATUSVARIABLECOMMAND_H

#include <sick_safetyscanners/cola2/VariableCommand.h>
#include <sick_safetyscanners/data_processing/ParseDeviceStatus.h>
#include <sick_safetyscanners/datastructure/DeviceStatus.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the field geometry from the sensor.
 */
class DeviceStatusVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;

  /*!
   * \brief Constructor of the command.
   *
   * Takes the current cola2 session and a reference to the field data variable which will be
   * written on execution. The index defines which field variable will be read. Depending on the
   * sensor up to 128 variables can be defined.
   *
   * \param session The current cola2 session.
   * \param field_data The field data reference which will be modified on execution.
   * \param index The variable index in a range of [0, 127].
   */
  DeviceStatusVariableCommand(Cola2Session& session,
                              datastructure::DeviceStatus& device_status);

  /*!
   * \brief Adds the data to the telegram.
   *
   * \param telegram The telegram which will be modified by the data.
   */
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;

  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
  std::shared_ptr<sick::data_processing::ParseDeviceStatus> m_device_status_parser_ptr;

  sick::datastructure::DeviceStatus& m_device_status;
};

} // namespace cola2
} // namespace sick


#endif // DEVICESTATUSVARIABLECOMMAND_H
