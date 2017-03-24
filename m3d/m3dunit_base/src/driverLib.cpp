#include "driverLib.hpp"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
bool driver_m3d::connect_to_m3d(std::string IP)
{
	try {
		host = IP;
		std::string port ="10001";
		tcp::resolver resolver(io_service);
		tcp::resolver::query query(host.c_str(), port.c_str());
		tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
		tcp::resolver::iterator end;
		boost::system::error_code error = boost::asio::error::host_not_found;
		while(endpoint_iterator != end)
		{
			socket.close();
			socket.connect(*endpoint_iterator++, error);
		}
		bool t;
		encRes = getEncoderRes(t);
		if (!t) return false;
		ROS_INFO("m3d encoder res: %d", encRes);
		return true;
	} catch (boost::system::system_error &e)
	{
		return false;
	}
}


bool driver_m3d::writeParam(int paramNo, int subindex, int paramValue)
{
    boost::system::error_code ignored_error;

    boost::system::error_code error;

    std::stringstream sstel;
    sstel<< "sp ";
    sstel<<std::hex<<paramNo;
    sstel<<"h.";
    sstel<<std::hex<<subindex;
    sstel<<"h ";
    sstel<<std::dec<<paramValue<<"\n";
    ROS_DEBUG("setting param %s", sstel.str().c_str());
    boost::asio::write(socket, boost::asio::buffer(sstel.str()), boost::asio::transfer_all(), ignored_error);
    //boost::this_thread::sleep( boost::posix_time::millisec(20) );


    int len = socket.read_some(boost::asio::buffer(buf), error);

    if (error == boost::asio::error::eof)
        return false ; // Connection closed cleanly by peer.
    else if (error)
        throw boost::system::system_error(error); // Some other error.

    ROS_DEBUG("received from m3d %s", std::string(buf.begin(), buf.begin()+len).c_str());
    return true;
}

bool driver_m3d::getParam (int paramNo, int subindex, int &paramValue)
{

    boost::system::error_code ignored_error;

    boost::system::error_code error;

    std::stringstream sstel;
    sstel<< "gp ";
    sstel<<std::hex<<paramNo;
    sstel<<"h.";
    sstel<<std::hex<<subindex;
    sstel<<"h\n";
    ROS_DEBUG("getting param %s", sstel.str().c_str());
    boost::asio::write(socket, boost::asio::buffer(sstel.str()), boost::asio::transfer_all(), ignored_error);


    int len = socket.read_some(boost::asio::buffer(buf), error);

    if (error == boost::asio::error::eof)
        return false ; // Connection closed cleanly by peer.
    else if (error)
        throw boost::system::system_error(error); // Some other error.

    std::string data = std::string(buf.begin(), buf.begin()+len).c_str();


    std::vector<std::string> strs;
    boost::split(strs,data,boost::is_any_of(" "));

    ROS_DEBUG("recived from m3d %s", std::string(buf.begin(), buf.begin()+len).c_str());

    if (strs.size()!=4) return false;



    try {
        paramValue = boost::lexical_cast<int>(strs[2]);
    }
    catch(boost::bad_lexical_cast &e)
    {
        return false;
    }
    ROS_DEBUG("Value: %d", paramValue);

    return true;
}

driver_m3d::driver_m3d()
	:socket(io_service)
{
		encRes =-1;
		lastMode = NOT_SET;
}
bool driver_m3d::setPosition (float position, int speed, int relative)
{

        ROS_INFO("Setting new position");

        bool isOk =false;

        //pos mode
        isOk = writeParam(0x3003, 0x0, 7);
        if (!isOk) return false;
        //speed
        isOk = writeParam(0x3000,0x10, speed);
        if (!isOk) return false;
        //position
        position=(position/(2*M_PI))*encRes;
        writeParam(0x3000,0x11, position);
        if (!isOk) return false;
        //stop!
        writeParam(0x3000,0x1, 0);
        if (!isOk) return false;
        //start!

        if (relative) writeParam(0x3000,0x1, 51);
        if (!relative) writeParam(0x3000,0x1, 52);
        if (!isOk) return false;
        lastMode = POS;
        return true;
}

float driver_m3d::getAngle(bool & isOk)
{

    int value;
    if (!getParam(0x396A, 0x0, value))
    {
        isOk =false ;
        return -1.0f;
    }
    value = value % encRes;
    float ret = -2*M_PI*value/encRes;

    isOk = true;
    return ret;
}

float driver_m3d::geVoltage(bool & isOk)
{
    int value;
    if (!getParam(0x3962, 0x0, value))
    {
        isOk =false ;
        return -1.0f;
    }
    isOk = true;
    return value;
}
int driver_m3d::getEncoderRes(bool & isOk)
{
    int value;
    if (!getParam(0x3962, 0x0, value))
    {
        isOk =false ;
        return -1.0f;
    }
    encRes = 4*value;
    isOk = true;
    return encRes;
}
bool driver_m3d::setSpeed(int speed)
{
    ROS_INFO("seting speed");

    bool isOk =false;
    //vel() mode
    isOk = writeParam(0x3003, 0x0, 3);
    if (!isOk) return false;
    //speed
    isOk = writeParam(0x3000,0x10, speed);
    if (!isOk) return false;
    //position
    //stop!
    writeParam(0x3000,0x1, 0);
    if (!isOk) return false;
    //start!

    writeParam(0x3000,0x1, 49);
    if (!isOk) return false;
    return true;
}
