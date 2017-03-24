#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <ros/console.h>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>


using boost::asio::ip::tcp;


using namespace boost::asio;
using namespace boost::system;
using boost::optional;



class driver_m3d
{

    enum mode
    {
        VEL, POS, NOT_SET
    };

public:
	driver_m3d();
	///////////////////////////////////////////////////////////
	/// connecting to 3dunit on given IP address
	/// @return true if connected
	///////////////////////////////////////////////////////////
    bool connect_to_m3d(std::string IP);
	///////////////////////////////////////////////////////////
	/// disconnect from 3dunit
	///////////////////////////////////////////////////////////
	void disconnet();
	///////////////////////////////////////////////////////////
	/// writing a parameter with given index and subindex
	/// @return true if succeed
	///////////////////////////////////////////////////////////
    bool writeParam(int paramNo, int subindex, int paramValue);
	///////////////////////////////////////////////////////////
	/// gets a parameter value with given index and subindex
	/// @return true if succeed
	///////////////////////////////////////////////////////////
    bool getParam (int paramNo, int subindex, int &paramValue);
    bool setPosition (float position, int speed, int relative);
    bool setSpeed(int speed);
    float getAngle()
    {
        bool t;
        float res=  getAngle(t);
        if (t) return res;
        return -1.0f;
    }

    float getAngle(bool & isOk);
    float geVoltage(bool & isOk);
    int getEncoderRes(bool & isOk);
private:
	boost::asio::io_service io_service;
	tcp::socket socket;
	std::string host;
	std::string incommingData;
    int encRes;
    mode lastMode;
	boost::array<char, 100> buf;
};
