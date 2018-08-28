#pragma once

#include <iostream>
#include <string>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>


namespace sick {
namespace datastructure {

const int MAXSIZE = 10000;

class PaketBuffer
{
public:

  typedef BYTE array_type;
  typedef boost::array<array_type, MAXSIZE> ArrayBuffer;
  typedef std::vector<array_type> VectorBuffer;

  PaketBuffer(const VectorBuffer& buffer);
  PaketBuffer(const ArrayBuffer& buffer, size_t length);

  static UINT32 getMaxSize() { return MAXSIZE;}

  const VectorBuffer& getBuffer() const;
  void setBuffer(const VectorBuffer& buffer);
  void setBuffer(const ArrayBuffer &buffer, size_t length);

  size_t getLength() const;


private:

  VectorBuffer m_buffer;
};

}
}


