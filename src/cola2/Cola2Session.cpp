#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>

namespace sick {
namespace cola2 {

Cola2Session::Cola2Session(boost::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client)
  : m_async_tcp_client_ptr(async_tcp_client)
  , m_session_id(0)
  , m_last_request_id(0)
{

  m_async_tcp_client_ptr->setPacketHandler(boost::bind(&Cola2Session::processPacket, this, _1));

  m_packet_merger_ptr = boost::make_shared<sick::data_processing::TCPPaketMerger>();

  m_tcp_parser_ptr = boost::make_shared<sick::data_processing::ParseTCPPacket>();

}

bool Cola2Session::open()
{
  std::cout << "open" << std::endl;
  CommandPtr command_ptr = boost::make_shared<CreateSession>(boost::ref(*this));
  return executeCommand(command_ptr);
}

bool Cola2Session::close()
{
  std::cout << "close" << std::endl;
  CommandPtr command_ptr = boost::make_shared<CloseSession>(boost::ref(*this));
  return executeCommand(command_ptr);
}

bool Cola2Session::executeCommand(CommandPtr command)
{
  //TODO sanitize

  std::cout << "execute command" << std::endl;

  addCommand(command->getRequestID(), command);

  // Lock the mutex here, unlock the mutex after processing the received packet
  command->lockExecutionMutex();
  sick::datastructure::PacketBuffer::VectorBuffer telegram;
  command->constructTelegram(telegram);
  std::cout << telegram.size() << std::endl;
  m_async_tcp_client_ptr->doSendAndReceive(telegram);
  command->waitForCompletion();




  return true;
}



UINT32 Cola2Session::getSessionID() const
{
  return m_session_id;
}

void Cola2Session::setSessionID(const UINT32 &session_id)
{
  m_session_id = session_id;
}

void Cola2Session::processPacket(const datastructure::PacketBuffer &packet)
{
  std::cout << "Processing TCP packet in Session" << std::endl;


  if (m_packet_merger_ptr->isEmpty() || m_packet_merger_ptr->isComplete())
  {
    std::cout << "target size: " << m_tcp_parser_ptr->getExpectedPacketLength(packet) << std::endl;
    m_packet_merger_ptr->setTargetSize(m_tcp_parser_ptr->getExpectedPacketLength(packet));

  }
  m_packet_merger_ptr->addTCPPacket(packet);

  if (!m_packet_merger_ptr->isComplete())
  {
    std::cout << "Packet is not complete yet. Trying to receive more data." << std::endl;
    m_async_tcp_client_ptr->initiateReceive();
    return;
  }

  std::cout << "Packet is complete now" << std::endl;

  // Now get the merged packet
  sick::datastructure::PacketBuffer deployedPacket = m_packet_merger_ptr->getDeployedPacketBuffer();



  UINT16 requestID = m_tcp_parser_ptr->getRequestID(deployedPacket);
  CommandPtr pendingCommand;
  if (findCommand(requestID, pendingCommand))
  {
    pendingCommand->processReplyBase(deployedPacket.getBuffer());
    removeCommand(requestID);
  }


}

bool Cola2Session::addCommand(UINT16 request_id, CommandPtr command)
{
  if(m_pending_commands_map.find(request_id) != m_pending_commands_map.end())
  {
    return false;
  }
  m_pending_commands_map[request_id] = command;
  return true;

}

bool Cola2Session::findCommand(UINT16 request_id, CommandPtr &command)
{
  if (m_pending_commands_map.find(request_id) == m_pending_commands_map.end())
  {
    return false;
  }
  command = m_pending_commands_map[request_id];
  return true;
}

bool Cola2Session::removeCommand(UINT16 request_id)
{
  auto it = m_pending_commands_map.find(request_id);
  if(it == m_pending_commands_map.end())
  {
    return false;
  }
  m_pending_commands_map.erase(it);
  return true;

}

UINT16 Cola2Session::getNextRequestID()
{
  if (m_last_request_id == std::numeric_limits<UINT16>::max())
  {
    m_last_request_id = 0;
  }

  return ++m_last_request_id;
}

}
}
