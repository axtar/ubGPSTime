// ubGPSTime
// get utc time from u-blox gps module
// designed for nixie clocks...
// Version 0.1.2 (alpha)
// Under construction...

// MIT license
// Copyright 2021 highvoltglow

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
// and associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, 
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
// is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies 
// or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
// OTHER DEALINGS IN THE SOFTWARE.


#include <ubGPSTime.h>

// constructor
ubGPSTime::ubGPSTime() : 
    _serialPort(nullptr), _debugPort(nullptr),
    _verbose(false), _initialized(false), 
    _pending(pending::none), _notify(nullptr),
    _timeUTC({}), _gpsStatus({})
{
} 

// provides a callback function for message notification
void ubGPSTime::attach(notifyCallBack callBack)
{
    _notify = callBack;
}

// stop getting message notifications
void ubGPSTime::detach()
{
    _notify = nullptr;
}

// defines serial com port to GPS module
void ubGPSTime::begin(Stream &serialPort)
{
    _serialPort = &serialPort;
}

// asking about GPS module information
// if we get a response, we assume that we are talking to a u-blox module
void ubGPSTime::initialize()
{
    _initialized = false;

    requestVersion();
    _pending = pending::version;
    if(waitForResponse(WAIT_FOR_RESPONSE))
    {
        _initialized = true;

        // bye bye NMEA spam!!!
        disableDefaultNMEA();        
    }
}

// reads from serial port
void ubGPSTime::process()
{
    static UBXMESSAGE message = {};
    static uint16_t fieldCounter = 0;
    static uint16_t payloadCounter = 0;
    char c = 0;

    if(_serialPort)
    {
        while(_serialPort->available())
        {
            c = _serialPort->read(); 
            switch(fieldCounter)
            {
                case 0: // header 1
                    if(c == UBX_HEADER1)
                    {
                        message.header1 = c;
                        fieldCounter++;
                        // free memory if we missed a delete
                        if(message.payload)
                        {
                            delete[] message.payload;
                            message.payload = nullptr;
                        }
                    }
                    break;

                case 1: // header 2
                    if(c == UBX_HEADER2)
                    {
                        message.header2 = c;
                        fieldCounter++;
                    }
                    else
                    {
                        fieldCounter = 0;
                    }
                    break;

                case 2: // class
                    message.msgClass = c;
                    fieldCounter++;
                    break;

                case 3: // id
                    message.msgID = c;
                    fieldCounter++;
                    break;

                case 4: // length (first of 2 bytes, little endian)
                    message.payloadLength = c;
                    fieldCounter++;
                    break;

                case 5: // length (second of 2 bytes, little endian)
                    message.payloadLength |= c << 8;
                    if(message.payloadLength == 0)
                    {
                        // no payload
                        fieldCounter++;
                    }
                    else if(message.payloadLength > MAX_PAYLOAD)
                    {
                        // payload larger as max supported size
                        // dismiss message and resync
                        fieldCounter = 0;
                        payloadCounter = 0;
                    }
                    else
                    {
                        // allocate memory
                        message.payload = new uint8_t[message.payloadLength];
                    }
                    
                    fieldCounter++;
                    break;

                case 6: // payload
                    message.payload[payloadCounter] = c;
                    payloadCounter++;
                    if(payloadCounter == message.payloadLength)
                    {
                        payloadCounter = 0;
                        fieldCounter++;
                    }
                    break;

                case 7: // checksum A
                    message.CK_A = c;
                    fieldCounter++;
                    break;

                case 8: // checksum B
                    message.CK_B = c;
                    fieldCounter = 0;
                    payloadCounter = 0;
                    processMessage(&message);
                    // free memory after processing message
                    if(message.payload)
                    {
                        delete[] message.payload;
                        message.payload = nullptr;
                    }
                    break;    

                default:
                    fieldCounter = 0;
                    payloadCounter = 0;
                    break;           
            }
        }
    }
    else
    {
        if(_verbose)
        {
            _debugPort->println("Com port not defined. Call begin first");
        }
    }
}

// enable debug information
void ubGPSTime::enableVerbose(Stream &debugPort)
{
    _debugPort = &debugPort;
    _verbose = true;
}

// disable debug information
void ubGPSTime::disableVerbose()
{
    _verbose = false;
}

// provides access to the module version data
MODULEVERSION ubGPSTime::getModuleVersion()
{
    return (_moduleVersion);
}

// provides access to last updated time data
TIMEUTC ubGPSTime::getTimeUTC()
{
    return (_timeUTC);
}

// provides access to last updated GPS status
GPSSTATUS ubGPSTime::getGPSStatus()
{
    return (_gpsStatus);
}

// returns initialization status
bool ubGPSTime::isInitialized()
{
    return (_initialized);
}

// calculates the checksums for outgoing messages
void ubGPSTime::calculateChecksum(UBXMESSAGE *message, CHECKSUM *checksum)
{
  checksum->CK_A = 0;
  checksum->CK_B = 0;
  stepChecksum(message->msgClass, checksum);
  stepChecksum(message->msgID, checksum);
  stepChecksum(message->payloadLength & 0xFF, checksum);
  stepChecksum(message->payloadLength >> 8, checksum);
  for(uint16_t i=0; i<message->payloadLength; i++)
  {
      stepChecksum(message->payload[i], checksum);
  }
}

// step in checksum calcualtion
void ubGPSTime::stepChecksum(uint8_t value, CHECKSUM *checksum)
{
    checksum->CK_A += value;
    checksum->CK_B += checksum->CK_A;
}

// not critical, just return true
bool ubGPSTime::validateChecksum(UBXMESSAGE *message)
{
    return true; 

    // bool value = false;
    // CHECKSUM checksum = {};
    // calculateChecksum(message, &checksum);
    // if((message->CK_A == checksum.CK_A) && (message->CK_B == checksum.CK_B))
    // {
    //     value = true;
    // }
    // return (value);
}    


// send a message to gps module
void ubGPSTime::sendMessage(UBXMESSAGE *message)
{
    if(_serialPort)
    {
        CHECKSUM checksum = {};
        calculateChecksum(message, &checksum);

        message->CK_A = checksum.CK_A;
        message->CK_B = checksum.CK_B;

        if(_verbose)
        {
            printMessage(message, direction::outgoing);
        }

        _serialPort->write(message->header1);
        _serialPort->write(message->header2);
        _serialPort->write(message->msgClass);
        _serialPort->write(message->msgID);
        _serialPort->write(message->payloadLength & 0xFF); 
        _serialPort->write(message->payloadLength >> 8);   

        //Write payload.
        for (uint16_t i = 0; i < message->payloadLength; i++)
        {
            _serialPort->write(message->payload[i]);
        }

        //Write checksum
        _serialPort->write(message->CK_A);
        _serialPort->write(message->CK_B);    
    }
    else
    {
        if(_verbose)
        {
            _debugPort->println("Com port not defined. Call begin first");
        }
    }
}

// disable default NMEA messages sent by GPS module
void ubGPSTime::disableDefaultNMEA()
{
    setMessageRate(UBX_NMEA, UBX_NMEA_GGA, 0);
    setMessageRate(UBX_NMEA, UBX_NMEA_GLL, 0);
    setMessageRate(UBX_NMEA, UBX_NMEA_GSA, 0);
    setMessageRate(UBX_NMEA, UBX_NMEA_GSV, 0);
    setMessageRate(UBX_NMEA, UBX_NMEA_RMC, 0);
    setMessageRate(UBX_NMEA, UBX_NMEA_VTG, 0);
}

// prints a message on debug port
void ubGPSTime::printMessage(UBXMESSAGE *message, direction dir)
{
    if(_verbose)
    {
        //_debugPort->println();
        switch(dir)
        {
            case direction::incoming:
                _debugPort->print(F("UBX Message <-- "));
                break;

            case direction::outgoing:
                _debugPort->print(F("UBX Message --> "));
                break;

        }
        printHEX(message->header1);
        printHEX(message->header2);
        printHEX(message->msgClass);
        printHEX(message->msgID);
        printHEX(message->payloadLength & 0xFF);
        printHEX(message->payloadLength >> 8);
        for(uint16_t i = 0; i < message->payloadLength; i++)
        {
            printHEX(message->payload[i]);
        }
        printHEX(message->CK_A);
        printHEX(message->CK_B);
        _debugPort->println();
    }
}

// prints a byte in HEX format 
void ubGPSTime::printHEX(uint8_t value)
{
    if(value < 16)
    {
        _debugPort->print("0");
    }
    _debugPort->print(value, HEX);
    _debugPort->print(" ");

}

// processes some incoming messages
void ubGPSTime::processMessage(UBXMESSAGE *message)
{
    if(validateChecksum(message))
    {
        if(_verbose)
        {
            printMessage(message, direction::incoming);
        }
        switch(message->msgClass)
        {
            case UBX_ACK:
                if(message->msgID == UBX_ACK_ACK)
                {
                    onAck(message);
                }
                if(message->msgID == UBX_ACK_NACK)
                {
                    onNack(message);
                }
                break;

            case UBX_MON:
                if(message->msgID == UBX_MON_VER)
                {
                    onVersion(message);
                }
                break;

            case UBX_NAV:
                if(message->msgID == UBX_NAV_STATUS)
                {
                    onStatus(message);
                }
                if(message->msgID == UBX_NAV_TIMEUTC)
                {
                    onTimeUTC(message);
                }
                break;
        }
        onMessageEvent(message);    
    }
    else
    {
        if(_verbose)
        {
            _debugPort->println("Got invalid message");
        }
    }
}

bool ubGPSTime::waitForResponse(uint32_t timeout)
{
    uint32_t timestamp = millis();
    while(millis() - timestamp < timeout)
    {
        process();
        if(_pending == pending::none)
        {
            return (true);
        }
    }
    return (false);
}

// sets update rate for messages in seconds, max 255, 
// use rate = 0 to stop the module from sending updates
void ubGPSTime::setMessageRate(uint8_t msgClass, uint8_t msgID, uint8_t rate)
{
    UBXMESSAGE message;
    uint8_t payLoad[3];

    message.payload = payLoad;
    message.header1 = UBX_HEADER1;
    message.header2 = UBX_HEADER2;
    message.msgClass = UBX_CFG;
    message.msgID = UBX_CFG_MSG;
    message.payloadLength = 3;
    message.payload[0] = msgClass;
    message.payload[1] = msgID;
    message.payload[2] = rate;
    sendMessage(&message); 
    _pending = pending::ack;      
    waitForResponse(WAIT_FOR_RESPONSE);
}

// requests a single message
void ubGPSTime::pollMessage(uint8_t msgClass, uint8_t msgID)
{
    UBXMESSAGE message;

    message.header1 = UBX_HEADER1;
    message.header2 = UBX_HEADER2;
    message.msgClass = msgClass;
    message.msgID = msgID;
    message.payloadLength = 0;
    message.payload = 0;
    sendMessage(&message);      
}

// callback message notification
void ubGPSTime::onMessageEvent(UBXMESSAGE *message)
{
    if(_notify)
    {
        _notify(message);
    }
}

// requests module version information
void ubGPSTime::requestVersion()
{
    pollMessage(UBX_MON, UBX_MON_VER);
}

// requests GPS status information
void ubGPSTime::requestStatus()
{
    pollMessage(UBX_NAV, UBX_NAV_STATUS);
}

// request date/time information
void ubGPSTime::requestTimeUTC()
{
    pollMessage(UBX_NAV, UBX_NAV_TIMEUTC);
}


// subscribe to GPS status information
void ubGPSTime::subscribeGPSStatus(uint8_t rate)
{
    setMessageRate(UBX_NAV, UBX_NAV_STATUS, rate);
}

// subscribe to date/time information
void ubGPSTime::subscribeTimeUTC(uint8_t rate)
{
    setMessageRate(UBX_NAV, UBX_NAV_TIMEUTC, rate);
}

// processes Ack messages
void ubGPSTime::onAck(UBXMESSAGE *message)
{
    // not checking Ack for now
    _pending = pending::none;
    if(_verbose)
    {
        _debugPort->println("Received ack");
    }

}

// processes Nack messages
void ubGPSTime::onNack(UBXMESSAGE *message)
{
    // not checking Nack for now
    _pending = pending::none;
    if(_verbose)
    {
        _debugPort->println("Received nack");
    }
}

// processes GPS status messages and updates internal data structure
void ubGPSTime::onStatus(UBXMESSAGE *message)
{
    _gpsStatus.timeOfWeek = getU4(message, 0);
    _gpsStatus.gpsFixType = getU1(message, 4);
    _gpsStatus.gpsFixOk = getFlag(message, 5, 0);
    _gpsStatus.diffApplied = getFlag(message, 5, 1);
    _gpsStatus.timeOfWeekValid = getFlag(message, 5, 2);
    _gpsStatus.weekNumberValid = getFlag(message, 5, 3);
    _gpsStatus.timestamp = millis();
    if(_verbose)
    {
        _debugPort->print("Time of week:        ");
        _debugPort->println(_gpsStatus.timeOfWeek);
        _debugPort->print("GPS fix type:        ");
        _debugPort->println(_gpsStatus.gpsFixType);
        _debugPort->print("GPS fix  OK:         ");
        _debugPort->println(_gpsStatus.gpsFixOk);
        _debugPort->print("Corrections applied: ");
        _debugPort->println(_gpsStatus.diffApplied);
        _debugPort->print("ToW valid:           ");
        _debugPort->println(_gpsStatus.timeOfWeekValid);
        _debugPort->print("Week number valid:   ");
        _debugPort->println(_gpsStatus.weekNumberValid);
    }
}

// processes module version messages and updates internal data structure
void ubGPSTime::onVersion(UBXMESSAGE *message)
{
    uint16_t offset = 0;
    _moduleVersion.swVersion = getString(message, offset, 30);
    offset += 30;
    _moduleVersion.hwVersion = getString(message, offset, 10);
    offset += 10;
    for(uint8_t i=0; i < MAX_EXTENSIONS; i++ )
    {
        if(message->payloadLength >= offset + EXTENSION_LEN)
        {
            _moduleVersion.extensions[i] = getString(message, offset, EXTENSION_LEN);
            offset += EXTENSION_LEN;
        }
        else
        {
            _moduleVersion.extensions[i]="N/A";
        }       
    }
    _pending = pending::none;
    if(_verbose)
    {
        _debugPort->print("Software version: ");
        _debugPort->println(_moduleVersion.swVersion);
        _debugPort->print("Hardware version: ");
        _debugPort->println(_moduleVersion.hwVersion);
        for(uint8_t i=0; i<MAX_EXTENSIONS; i++)
        {
            _debugPort->printf("Extension %u: ",i+1);
            _debugPort->println(_moduleVersion.extensions[i]);
        }
    }
}

// processes date/time messages and updates data structure
void ubGPSTime::onTimeUTC(UBXMESSAGE *message)
{
    _timeUTC.timeOfWeek = getU4(message, 0);
    _timeUTC.accuracy = getU4(message, 4);
    _timeUTC.nanoSecond = getI4(message, 8);
    _timeUTC.year = getU2(message, 12);
    _timeUTC.month = getU1(message, 14);
    _timeUTC.day = getU1(message, 15);
    _timeUTC.hour = getU1(message, 16);
    _timeUTC.minute = getU1(message, 17);
    _timeUTC.seconds = getU1(message, 18);
    _timeUTC.timeOfWeekValid = (bool) getFlag(message, 19, 0);
    _timeUTC.weekNumberValid = (bool) getFlag(message, 19, 1);
    _timeUTC.utcValid = (bool) getFlag(message, 19, 2);
    _timeUTC.timestamp = millis();
    if(_verbose)
    {
        _debugPort->print("Time of week:       ");
        _debugPort->println(_timeUTC.timeOfWeek);
        _debugPort->print("accuracy:           ");
        _debugPort->println(_timeUTC.accuracy);
        _debugPort->print("Nanoseconds:        ");
        _debugPort->println(_timeUTC.nanoSecond);
        _debugPort->print("Year:               ");
        _debugPort->println(_timeUTC.year);
        _debugPort->print("Month:              ");
        _debugPort->println(_timeUTC.month);
        _debugPort->print("Day:                ");
        _debugPort->println(_timeUTC.day);
        _debugPort->print("Hour:               ");
        _debugPort->println(_timeUTC.hour);
        _debugPort->print("Minute:             ");
        _debugPort->println(_timeUTC.minute);
        _debugPort->print("Second:             ");
        _debugPort->println(_timeUTC.seconds);
        _debugPort->print("Time of week valid: ");
        _debugPort->println(_timeUTC.timeOfWeekValid);
        _debugPort->print("Week number valid:  ");
        _debugPort->println(_timeUTC.weekNumberValid);
        _debugPort->print("UTC valid:          ");
        _debugPort->println(_timeUTC.utcValid);
        _debugPort->print("Timestamp:          ");
        _debugPort->println(_timeUTC.timestamp);
    }
}

// field extraction functions
uint8_t ubGPSTime::getU1(UBXMESSAGE *message, uint8_t offset)
{
    return (uint8_t)message->payload[offset];
}

uint16_t ubGPSTime::getU2(UBXMESSAGE *message, uint8_t offset)
{
  uint16_t value = 0;
  value |= (uint16_t)message->payload[offset];
  value |= (uint16_t)message->payload[offset + 1] << 8;
  return (value);
}

uint32_t ubGPSTime::getU4(UBXMESSAGE *message, uint8_t offset)
{
  uint32_t value = 0;
  value |= (uint32_t)message->payload[offset];
  value |= (uint32_t)message->payload[offset + 1] << 8;
  value |= (uint32_t)message->payload[offset + 2] << 16;
  value |= (uint32_t)message->payload[offset + 3] << 24;
  return (value);
}

int32_t ubGPSTime::getI4(UBXMESSAGE *message, uint8_t offset)
{
  return ((int32_t)getU4(message, offset));
}

uint8_t ubGPSTime::getFlag(UBXMESSAGE *message, uint8_t offset, uint8_t bit)
{
    uint8_t flags = getU1(message, offset);
    return ((flags >> bit) & 0x01);
}

String ubGPSTime::getString(UBXMESSAGE *message, uint8_t offset, uint8_t length)
{
    String s;
    for(uint16_t i = offset; i < offset + length; i++)
    {
        if(message->payload[i]!= 0)
        {
            s += (char) message->payload[i];
        }
        else
        {
            break;
        }
    }
    return (s);        
}

