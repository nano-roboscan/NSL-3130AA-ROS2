#include <iostream>
#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace nanosys {

typedef std::vector<uint8_t> Packet;


TcpConnection::TcpConnection(boost::asio::io_service& ioService)
  : resolver(ioService), socket(ioService), state(STATE_DISCONNECTED) {
	strcpy(HOST,"192.168.0.220");
}

TcpConnection::~TcpConnection() {
  try {
    disconnect();
  } catch (boost::system::system_error &e) {
    std::cerr << e.what() << std::endl;
  }
}

void TcpConnection::setIpAddr(std::string ipAddr)
{
    disconnect();
	strcpy(HOST, ipAddr.c_str());
	connect();

}

void TcpConnection::sendCommand(const std::vector<uint8_t>& data) {
	if (!isConnected()){
		printf("sendComand :: not connected TCP\n");
		return;
	}
	uint32_t data_len = data.size();
	//  size_t buf_size = MARKER_SIZE + sizeof(data_len) + data_len + MARKER_SIZE;
	std::ostringstream os;
	os << START_MARKER;
	os << static_cast<uint8_t>((data_len >> 24) & 0xff);
	os << static_cast<uint8_t>((data_len >> 16) & 0xff);
	os << static_cast<uint8_t>((data_len >>  8) & 0xff);
	os << static_cast<uint8_t>((data_len >>  0) & 0xff);
	for (uint32_t i = 0; i < data_len; ++i) {
		os << static_cast<uint8_t>(data[i]);
	}
	os << END_MARKER;

	boost::system::error_code error;
	socket.write_some(boost::asio::buffer(os.str(), os.tellp()), error);
	if (error) {
		//throw boost::system::system_error(error);
		printf("sendCommand systemError:::\n");
		disconnect();		
	}
	waitAck();
}

void TcpConnection::connect() {
	if (isConnected()) return;
	
	updateState(STATE_CONNECTING);
	tcp::resolver::query query(HOST, PORT);
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	tcp::resolver::iterator end;

	boost::system::error_code error = boost::asio::error::host_not_found;
	while (error && endpoint_iterator != end) {
		socket.close();
		socket.connect(*endpoint_iterator++, error);
	}
	if (error) {
		printf("connect() error :: current ip / port info : %s / %s\n", HOST, PORT);
		printf("Check connection error(power or IP etc)\n");
		updateState(STATE_DISCONNECTED);
		//throw::boost::system::system_error(error);

		return;
	}

	updateState(STATE_CONNECTED);
}

void TcpConnection::disconnect() {
  if (isDisconnected()) return;

  updateState(STATE_CLOSING);

  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
  if (error) {
    revertState();
    //throw boost::system::system_error(error);
    printf("disconenct error\n");
  }
  
  updateState(STATE_DISCONNECTED);
}

void TcpConnection::waitAck() {
  Packet buf(ACK_BUF_SIZE);
  boost::system::error_code error;

  this->updateState(STATE_WAIT_ACK);
  socket.read_some(boost::asio::buffer(buf), error);

  if (error) {
    //throw boost::system::system_error(error);
    printf("wait ACK system_error\n");
  }
  this->revertState();
}

void TcpConnection::updateState(State state_) const {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() const {
  state = previousState;
}

bool TcpConnection::isConnected() const {
  return state == STATE_CONNECTED;
}

bool TcpConnection::isDisconnected() const {
  return state == STATE_DISCONNECTED;
}

} // end namespace nanosys
