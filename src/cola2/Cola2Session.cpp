#include <sick_microscan3_ros_driver/cola2/Cola2Session.h>

namespace sick {
namespace cola2 {

Cola2Session::Cola2Session(boost::shared_ptr<sick::communication::AsyncTCPClient> async_tcp_client)
  : m_async_tcp_client(async_tcp_client)
  , m_sessionID(0)
  , m_lastRequestID(0)
{

  m_async_tcp_client->setPacketHandler(boost::bind(&Cola2Session::processPacket, this, _1));

  m_packetMerger = boost::make_shared<sick::data_processing::TCPPaketMerger>();

  m_tcp_parserPtr = boost::make_shared<sick::data_processing::ParseTCPPacket>();

  open();
}

bool Cola2Session::open()
{
  std::cout << "open" << std::endl;
  CommandPtr commandPtr = boost::make_shared<CreateSession>(boost::ref(*this));
  return executeCommand(commandPtr);
}

bool Cola2Session::close()
{
  std::cout << "close" << std::endl;
  CommandPtr commandPtr = boost::make_shared<CloseSession>(boost::ref(*this));
  return executeCommand(commandPtr);
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
  m_async_tcp_client->doSendAndReceive(telegram);
  command->waitForCompletion();


  return true;
}

UINT32 Cola2Session::getSessionID() const
{
  return m_sessionID;
}

void Cola2Session::setSessionID(const UINT32 &sessionID)
{
  m_sessionID = sessionID;
}

void Cola2Session::processPacket(const datastructure::PacketBuffer &packet)
{
  std::cout << "Processing TCP packet in Session" << std::endl;


  if (m_packetMerger->isEmpty() || m_packetMerger->isComplete())
  {
    std::cout << "target size: " << m_tcp_parserPtr->getExpectedPacketLength(packet) << std::endl;
    m_packetMerger->setTargetSize(m_tcp_parserPtr->getExpectedPacketLength(packet));

  }
  m_packetMerger->addTCPPacket(packet);

  if (!m_packetMerger->isComplete())
  {
    std::cout << "Packet is not complete yet. Trying to receive more data." << std::endl;
    m_async_tcp_client->initiateReceive();
    return;
  }

  std::cout << "Packet is complete now" << std::endl;

  // Now get the merged packet
  sick::datastructure::PacketBuffer deployedPacket = m_packetMerger->getDeployedPacketBuffer();



  UINT16 requestID = m_tcp_parserPtr->getRequestID(deployedPacket);
  CommandPtr pendingCommand;
  if (findCommand(requestID, pendingCommand))
  {
    pendingCommand->processReplyBase(deployedPacket.getBuffer());
    removeCommand(requestID);
  }


}

bool Cola2Session::addCommand(UINT16 request_id, CommandPtr command)
{
  if(m_pending_commands.find(request_id) != m_pending_commands.end())
  {
    return false;
  }
  m_pending_commands[request_id] = command;
  return true;

}

bool Cola2Session::findCommand(UINT16 request_id, CommandPtr &command)
{
  if (m_pending_commands.find(request_id) == m_pending_commands.end())
  {
    return false;
  }
  command = m_pending_commands[request_id];
  return true;
}

bool Cola2Session::removeCommand(UINT16 request_id)
{
  auto it = m_pending_commands.find(request_id);
  if(it == m_pending_commands.end())
  {
    return false;
  }

  m_pending_commands.erase(it);
  return true;

}

UINT16 Cola2Session::getNextRequestID()
{
  if (m_lastRequestID == std::numeric_limits<UINT16>::max())
  {
    m_lastRequestID = 0;
  }

  return ++m_lastRequestID;
}

}
}
