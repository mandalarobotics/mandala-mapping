#include "lms_mini_lib.hpp"
#include <iostream>
using boost::asio::ip::tcp;




    void lms_socket::requestContinousScan()
    {
        sendMsg("sMN LMCstartmeas");
        sendMsg("sRN STlms");
        sendMsg("sEN LMDscandata 1");
    }

    void lms_socket::RequestStopScan()
    {
        sendMsg("sEN LMDscandata 0");
    }

    void lms_socket::connectToLaser(std::string ip)
    {
        std::string host =ip;
        std::string port ="2111";
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(host.c_str(), port.c_str());
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        tcp::resolver::iterator end;
        boost::system::error_code error = boost::asio::error::host_not_found;

        while(endpoint_iterator != end){
            socket.close();
            socket.connect(*endpoint_iterator++, error);
        }
        //if (error) throw boost::system::system_error(error);
    }


    lms_socket::lms_socket()
        :socket(io_service)
    {
        _debug =false;
    }
    lms_socket::~lms_socket()
    {
        disconnet();
    }
    void lms_socket::sendMsg(std::string msg)
    {
        msg = (char(0x02))+msg+(char(0x03));
        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(msg), boost::asio::transfer_all(), ignored_error);

    }

    void lms_socket::readData(bool &isMeasurment)
    {
        boost::array<char, 4000> buf;
        boost::system::error_code error;
        int len = socket.read_some(boost::asio::buffer(buf), error);
        if (error == boost::asio::error::eof)
           return; // Connection closed cleanly by peer.
        else if (error)
           throw boost::system::system_error(error); // Some other error.


        incommingData.append(buf.begin(), buf.begin()+len);
        size_t begin = incommingData.find_last_of(char(0x02));
        size_t end = incommingData.find_last_of(char(0x03));
        if(_debug)std::cout <<incommingData.size()<<'\n';
        if (begin!= std::string::npos && end != std::string::npos && begin < end)
        {

            //std::cout <<incommingData.substr(begin+1, end-begin-1)<<"\n";
            isMeasurment = processSubMsg(incommingData.substr(begin+1, end-begin-1));

            incommingData.clear();
        }
        if (incommingData.size()>100000)
        {
            std::cerr<<"overflow warning \n";
            incommingData.clear();
        }
    }

    void lms_socket::disconnet()
    {
        sendMsg("sEN LMDscandata 0");
        sleep(1);
        boost::system::error_code ec;
        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
        if (ec)
        {
            throw boost::system::system_error(ec);
        }
    }

    bool lms_socket::processSubMsg(std::string msg)
    {

        std::vector<std::string> data;
        if(_debug)std::cout <<"===\n";
        if(_debug)std::cout << msg<<"\n";
        boost::split(data, msg, boost::is_any_of(" "));
        if ( data.size()>10 && data[0].compare("sSN") ==0 &&  data[1].compare("LMDscandata") ==0)
        {
             processMeasurment (data);
             return true;
        }

    }

    int lms_socket::searchForPhase(std::string stringPh,  std::vector<std::string> & data , int beg)
    {

        if(_debug)std::cout << "search for: "<<stringPh <<"\n";
        for (int i = beg; i < data.size(); i++)
        {
            if (data[i].compare(stringPh) == 0)
            {
                if(_debug)std::cout << "found "<<stringPh <<"\n";
                return i;
            }
        }
        return -1;
    }

    int lms_socket::hexToInt(std::string hex)
    {
        return (int)strtol(hex.c_str(), NULL, 16);
    }
    float lms_socket::hexToFloat(std::string hex)
    {
	unsigned int x;
	std::stringstream ss;
	ss << std::hex << hex;
	ss>>x;
	return reinterpret_cast<float&>(x);
     //   return (float)strtol(hex.c_str(), NULL, 16);
    }
    bool lms_socket::processMeasurment( std::vector<std::string> & data )
    {


        currentMessage.echoes.clear();
        currentMessage.rssis.clear();

        currentMessage._typefCommand = data[0];
        currentMessage._command = data[1];
        currentMessage._version = hexToInt(data[2]);
        currentMessage._deviceNo= hexToInt(data[3]);
        currentMessage._serialNo= hexToInt(data[4]);
        currentMessage._deviceStatus[0]= hexToInt(data[5]);
        currentMessage._deviceStatus[1]= hexToInt(data[6]);
        currentMessage._telegramNo = hexToInt(data[7]);
        currentMessage._scanNo = hexToInt(data[8]);
        currentMessage._timeSinceStartUp=hexToInt(data[9]);
        currentMessage._timeOfTransmission=hexToInt(data[10]);
        currentMessage._inputStatus[0]=hexToInt(data[11]);
        currentMessage._inputStatus[1]=hexToInt(data[12]);

        currentMessage._outputStatus[0]=hexToInt(data[13]);
        currentMessage._outputStatus[1]=hexToInt(data[14]);

        currentMessage._reserved = hexToInt(data[15]);
        currentMessage._scanningFreq = hexToInt(data[16]);
        currentMessage._measurmentFreq =hexToInt(data[17]);
        currentMessage._noOfEncoders = hexToInt(data[18]);

        int dOff = 18 + currentMessage._noOfEncoders*2;
        currentMessage._noOfDISTChannels = hexToInt(data[dOff]);

        currentMessage._noOfRssiChannels =1;
        if(_debug) std::cout <<"no of dist channels:"<<currentMessage._noOfDISTChannels<<"\n";
        if(_debug)std::cout <<"no of rssi channels:"<<currentMessage._noOfRssiChannels<<"\n";

        if(currentMessage._noOfDISTChannels<=1)
        {
            lms_channel ch;
            procesChannel(ch, data, "DIST1");
            currentMessage.echoes.push_back(ch);

        }
        if(currentMessage._noOfDISTChannels<=2)
        {
            lms_channel ch;
            procesChannel(ch, data, "DIST2");
            currentMessage.echoes.push_back(ch);
        }
        if(currentMessage._noOfDISTChannels<=3)
        {
            lms_channel ch;
            procesChannel(ch, data, "DIST3");
            currentMessage.echoes.push_back(ch);
        }
        if(currentMessage._noOfDISTChannels<=4)
        {
            lms_channel ch;
            procesChannel(ch, data, "DIST4");
            currentMessage.echoes.push_back(ch);
        }
        if(currentMessage._noOfDISTChannels<=5)
        {
            lms_channel ch;
            procesChannel(ch, data, "DIST5");
            currentMessage.echoes.push_back(ch);
        }
        if(currentMessage._noOfRssiChannels<=1)
        {
            lms_channel ch;
            procesChannel(ch, data, "RSSI1");
            currentMessage.rssis.push_back(ch);
        }
        if(currentMessage._noOfRssiChannels<=2)
        {
            lms_channel ch;
            procesChannel(ch, data, "RSSI2");
            currentMessage.rssis.push_back(ch);
        }

        if(currentMessage._noOfRssiChannels<=3)
        {
            lms_channel ch;
            procesChannel(ch, data, "RSSI3");
            currentMessage.rssis.push_back(ch);
        }

        if(currentMessage._noOfRssiChannels<=4)
        {
            lms_channel ch;
            procesChannel(ch, data, "RSSI4");
            currentMessage.rssis.push_back(ch);
        }

        if(currentMessage._noOfRssiChannels<=5)
        {
            lms_channel ch;
            procesChannel(ch, data, "RSSI5");
            currentMessage.rssis.push_back(ch);
        }


    }
    bool lms_socket::procesChannel(lms_channel &ch, std::vector<std::string> & data, std::string channelName)
    {
       int offset = searchForPhase(channelName, data);
       if (offset <0 )return false;
       ch.contents = data[offset];
       ch.scallingFactor =hexToFloat(data[offset+1]);
       ch.scallingOffset =hexToFloat(data[offset+2]);
       ch.startAngle =0.0001f*hexToInt(data[offset+3]);
       ch.angStepWidth =0.0001f*hexToInt(data[offset+4]);
       ch.numberOfData = hexToInt(data[offset+5]);
       if (ch.numberOfData+offset>data.size()) throw "apocalypse!";
       ch.data.resize(ch.numberOfData);

       if(_debug)std::cout << channelName<<" size:"<<ch.numberOfData <<"\n";
       for(int i=0; i <ch.numberOfData; i++ )
       {
           ch.data[i] = hexToInt(data[offset+6+i]);
       }
    }

