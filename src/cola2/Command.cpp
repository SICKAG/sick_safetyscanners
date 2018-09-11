#include <sick_microscan3_ros_driver/cola2/Command.h>

#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>


namespace sick {
namespace cola2 {

Command::Command(Cola2Session &session, UINT16 command_type, UINT16 command_mode)
  : m_session(session)
  , m_command_mode(command_mode)
  , m_command_type(command_type)
{
  m_session_id = m_session.getSessionID();
  m_request_id = m_session.getNextRequestID();

  m_tcp_parser_ptr = boost::make_shared<sick::data_processing::ParseTCPPacket>();
  m_writer_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

void Command::lockExecutionMutex()
{
  //m_executionMutex.lock();
}

void Command::constructTelegram(datastructure::PacketBuffer::VectorBuffer &telegram) const
{
  addTelegramData(telegram);
  addTelegramHeader(telegram);
}

void Command::processReplyBase(const datastructure::PacketBuffer::VectorBuffer &packet)
{
  //Parse first
  std::cout << "process reply of tcp" << std::endl;
  m_tcp_parser_ptr->parseTCPSequence(packet, *this);

  m_was_successful = processReply();

 // m_executionMutex.unlock();
}

void Command::waitForCompletion()
{
  // boost::mutex::scoped_lock(m_executionMutex);

}

bool Command::wasSuccessful() const {return m_was_successful;}

UINT8 Command::getCommandType() const
{
  return m_command_type;
}

void Command::setCommandType(const UINT8 &command_type)
{
  m_command_type = command_type;
}

UINT8 Command::getCommandMode() const
{
  return m_command_mode;
}

void Command::setCommandMode(const UINT8 &command_mode)
{
  m_command_mode = command_mode;
}

UINT32 Command::getSessionID() const
{
  return m_session_id;
}

void Command::setSessionID(const UINT32 &session_id)
{
  m_session_id = session_id;
}

UINT16 Command::getRequestID() const
{
  return m_request_id;
}

void Command::setRequestID(const UINT16 &request_id)
{
  m_request_id = request_id;
}

void Command::addTelegramHeader(datastructure::PacketBuffer::VectorBuffer &telegram) const
{

  datastructure::PacketBuffer::VectorBuffer header;
  UINT32 cola2_STx =0x02020202; //= m_parser->getDefaultSTx();
  UINT8 cola2_HubCntr = 0x00; // See Application Note Using Cola2.x, 3.2
  UINT8 cola2_NoC = 0x00; // See Application Note Using Cola2.x, 3.2
  UINT32 sessionID = getSessionID();
  UINT16 requestID = getRequestID();
  UINT8 commandType = getCommandType();
  UINT8 commandMode = getCommandMode();
  UINT32 length = sizeof(cola2_HubCntr)
          + sizeof(cola2_NoC)
          + sizeof(sessionID)
          + sizeof(requestID)
          + sizeof(commandType)
          + sizeof(commandMode)
          + telegram.size();
  UINT32 telegramSize = sizeof(cola2_STx)
              + sizeof(length)
              + length;
  header.resize(telegramSize - telegram.size());

  std::cout << "header_size" << header.size() << std::endl;
  std::cout << "data_size" << telegram.size() << std::endl;

  BYTE* data_ptr = header.data();


  m_writer_ptr->writeUINT32BigEndian(data_ptr, cola2_STx, 0);
//  m_writer_ptr->writeUINT32BE(data_ptr, 0x00000000);
  m_writer_ptr->writeUINT32BigEndian(data_ptr, length, 4);
  m_writer_ptr->writeUINT8BigEndian(data_ptr, cola2_HubCntr, 8);
  m_writer_ptr->writeUINT8BigEndian(data_ptr, cola2_NoC, 9);
  m_writer_ptr->writeUINT32BigEndian(data_ptr, sessionID,  10);
  m_writer_ptr->writeUINT16BigEndian(data_ptr, requestID,14);
  m_writer_ptr->writeUINT8BigEndian(data_ptr, commandType,16);
  m_writer_ptr->writeUINT8BigEndian(data_ptr, commandMode,17);
  telegram.insert(telegram.begin(), header.begin(), header.end());

  std::cout << "tele: " << telegram.size() << std::endl;
}

std::vector<BYTE> Command::getDataVector() const
{
  return m_data_vector;
}

void Command::setDataVector(const std::vector<BYTE> &data)
{
  m_data_vector = data;
}

}
}

