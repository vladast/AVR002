/*
 * Copyright (C) 2014  Vladimir Stankovic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

//#include <stdio.h>
//#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <list>

#ifdef WIN32
#include "lusb0_usb.h"
#else
#include <usb.h>
#endif

#include <stdint.h>
#include <signal.h>     // To trap SIGINT
#include "opendevice.h" // V-USB's opendevice lib

#include "reqdefs.h"
#include "usbdefs.h"

#define SWID_UNKNOWN        0x00
#define SWID_WITH           0x01
#define SWID_THROW          0x02
#define SWID_WITHOUT        0x03

#define OV_BIT				0x20

class Reading
{
private:
	uint8_t		m_Entry;		// entry count
    uint8_t		m_Code;			// event code
	uint16_t	m_Timestamp;	// timestamp

public:
	Reading() : m_Entry(0), m_Code(0), m_Timestamp(0) {}

	void setEntry(const uint8_t& entryCount) { m_Entry = entryCount; }
	uint8_t getEntry() const { return m_Entry; }

	void setCode(const uint8_t& code) { m_Code = code; }
	uint8_t getCode() const { return m_Code; }

	void setTimestamp(const uint16_t& timestamp) { m_Timestamp = timestamp; }
	uint16_t getTimestamp() const { return m_Timestamp; }
};

void trap_callback(int signal)
{
    if(signal == SIGINT)
    {
        //printf("Detected interrupt request. Exiting...\n");
		std::cout<<"Detected interrupt request. Exiting..."<<std::endl;
        exit(signal);
    }
}

int main(int argc, char **argv)
{
    usb_dev_handle      *handle = NULL;
    const unsigned char vendorID[2] = {USB_CFG_VENDOR_ID}, deviceID[2] = {USB_CFG_DEVICE_ID};
    char                vendorName[] = {USB_CFG_VENDOR_NAME, 0}, deviceName[] = {USB_CFG_DEVICE_NAME, 0};
	uint8_t             buffer[4] = {0x00, 0x00, 0x00, 0x00};
    int                 iRxByteCount, iVendorID, iDeviceID;
    short               iSession = 0;

    // Trap interrupt signal, SIGINT
    signal(SIGINT, trap_callback);

    // Initialize libusb
    usb_init();

    // Convert received vendor id and device id into integers
    iVendorID = (vendorID[1] << 8) | vendorID[0];
    iDeviceID = (deviceID[1] << 8) | deviceID[0];

    std::cout << "Waiting for connection from USB device" << deviceName << "(Vendor ID = 0x" << std::hex << iVendorID << ", Device ID = 0x" << iDeviceID << ") ..." << std::dec << std::endl;

    for(;;)
    {

        if(usbOpenDevice(&handle, iVendorID, vendorName, iDeviceID, deviceName, NULL, NULL, NULL) != 0)
        {
            continue;
        }
        else
        {
			std::cout<<std::endl;
        }

        // Get the device's header code
        iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_HEADER, 0, 0, (char*)buffer, sizeof(buffer), 5000);
        if(iRxByteCount < 1)
        {
            if(iRxByteCount < 0)
                std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
            else
                std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
        }
        else
        {
            // Read device header id value and determine which device is being used (AVR001, AVR002, etc.)
            // That value will be used to determine how to process received data.

            uint16_t deviceCode = buffer[0] | (buffer[1] << 8);

            std::cout << "Detected device: ";
            switch(deviceCode)
            {
                case AVR001_CODE:
                {
                    std::cout << "AVR001" << std::endl;
                    break;
                }
                case AVR002_CODE:
                {
                    std::cout << "AVR002" << std::endl;

                    // Read device's state
                    iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_DATA1, 0, 0, (char*)buffer, sizeof(buffer), 5000);
                    if(iRxByteCount < 0)
                    {
                        if(iRxByteCount < 0)
                            std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
                        else
                            std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
                    }
                    else
                    {
                        std::cout << "Device's state is: ";
                        switch(buffer[0])
                        {
                        case 0xA1: // START   = 0xA1, // Device is being started
                            std::cout << "START" << std::endl;
                            break;
                        case 0xB2: // INIT    = 0xB2, // Initialize device
                            std::cout << "INIT" << std::endl;
                            break;
                        case 0xC3: // RECORD  = 0xC3, // Record events (touch-switch states)
                            std::cout << "RECORD" << std::endl;
                            break;
                        case 0xD4: // UPLOAD  = 0xD4, // Upload records to USB host
                            std::cout << "UPLOAD" << std::endl;
                            break;
                        case 0xE5: // DELETE  = 0xE5, // Erase external EEPROM
                            std::cout << "DELETE" << std::endl;
                            break;
                        case 0xF6: // RESES   = 0xF6  // Reinit session counter
                            std::cout << "RESES" << std::endl;
                            break;
                        default:
                            std::cout << "UNDEFINED (0x" << std::hex << buffer[0] << ")" << std::dec << std::endl;
                        }
                    }

                    // Read device's session
                    iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_DATA2, 0, 0, (char*)buffer, sizeof(buffer), 5000);
                    if(iRxByteCount < 1)
                    {
                        if(iRxByteCount < 0)
                            std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
                        else
                            std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
                    }
                    else
                    {
                        std::cout << "Device's session is: " << (int)buffer[0] << std::endl;
                        iSession = (short)buffer[0];
                    }

                    // Read device's error cache
                    iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_DATA3, 0, 0, (char*)buffer, sizeof(buffer), 5000);
                    if(iRxByteCount < 1)
                    {
                        if(iRxByteCount < 0)
                            std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
                        else
                            std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
                    }
                    else
                    {
                        if(buffer[0] == 0)
                            std::cout << "No errors were encountered by device." << std::endl;
                        else
                            std::cerr << "Device has encountered an error with error-code 0x" << std::hex << (int)buffer[0] << std::dec << std::endl;
                    }

                    // Read event count
                    uint16_t entryCount = 0;
                    iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_DATA4, 0, 0, (char*)buffer, sizeof(buffer), 5000);
                    if(iRxByteCount < 1)
                    {
                        if(iRxByteCount < 0)
                            std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
                        else
                            std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
                    }
                    else
                    {
                        entryCount = buffer[0] | (buffer[1] << 8);
                        std::cout << "Number of events recorded by device: " << entryCount << std::endl;
                    }

                    // Read data
                    uint16_t eepromdata = 0;
                    bool fReadNext = false;
                    Reading reading;
                    std::list<Reading> readings;

                    for(uint16_t i = 0; i < entryCount; ++i)
                    {
                        iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, REQ_GET_DATA5, 0, 0, (char*)buffer, sizeof(buffer), 5000);

                        if(iRxByteCount < 1)
                        {
                            if(iRxByteCount < 0)
                                std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;
                            else
                                std::cerr << "[ERROR] Nothing has been received from device!" << std::endl;
                        }
                        else
                        {

                            if(fReadNext) // overflow detected in previous iteration
                            {
                                fReadNext = false;
                                eepromdata |= (buffer[0] << 5);
                                reading.setEntry(reading.getEntry() + 1);
                                reading.setTimestamp(eepromdata);
                                eepromdata = 0;
                                readings.push_back(reading);
                            }
                            else
                            {
                                eepromdata = buffer[0] & 0x1F; // read last 5 bits
                                fReadNext = (buffer[0] & OV_BIT) == OV_BIT; // check OV bit
                                if((buffer[0] >> 6) == SWID_UNKNOWN)
                                {
                                    std::cerr << "[ERROR] Received invalid event code!" << std::endl;
                                    fReadNext = false;
                                    continue;
                                }

                                reading.setCode(buffer[0] >> 6);

                                if(!fReadNext)
                                {
                                    reading.setEntry(reading.getEntry() + 1);
                                    reading.setTimestamp(eepromdata);
                                    eepromdata = 0;
                                    readings.push_back(reading);
                                }
                            }
                        }
                    }

                    // START   = 0xA1, // Device is being started
                    // INIT    = 0xB2, // Initialize device
                    // RECORD  = 0xC3, // Record events (touch-switch states)
                    // UPLOAD  = 0xD4, // Upload records to USB host
                    // DELETE  = 0xE5, // Erase external EEPROM
                    // RESES   = 0xF6  // Reinit session counter

                    // When completed, set device's state to INIT to start new session
                    iRxByteCount = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, REQ_SET_DATA1, 0xB2, 0, (char*)buffer, 0, 5000);
                    if(iRxByteCount < 0)
                        std::cerr << "[ERROR] usb_control_msg returned error: '" << usb_strerror() << "'" << std::endl;

                    // Open file for write
                    std::stringstream ssOutputCsvFilename;
                    ssOutputCsvFilename << "output_" << iSession << ".csv";
                    std::string sOutputCsvFilename = ssOutputCsvFilename.str();//"output.csv";
                    std::ofstream ofCsvFile(sOutputCsvFilename.c_str());
                    if(!ofCsvFile.good())
                    {
                        std::cerr << "Opening file '" << sOutputCsvFilename << "' failed!" << std::endl;
                        return -1;
                    }

                    std::cout << std::endl << "Following events has been read from device:" << std::endl << std::endl;

                    // Define CSV file header
                    ofCsvFile << "No,Code,Timestamp" << std::endl;
                    std::cout << "\tNo\tCode\tTimestamp" << std::endl;

                    for(std::list<Reading>::iterator iter = readings.begin(); iter != readings.end(); ++iter)
                    {
                        std::cout << std::dec << '\t' << int(iter->getEntry()) << '\t' << int(iter->getCode()) << '\t' << iter->getTimestamp() << std::endl;
                        ofCsvFile << int(iter->getEntry()) << "," << int(iter->getCode()) << "," << iter->getTimestamp() << std::endl;
                    }

                    ofCsvFile.close();
                    std::cout << std::endl << "Events read from device has been saved into file named '" << sOutputCsvFilename << "'" << std::endl;

                    break;
                }
                default:
                    std::cout << "UNKNOWN (0x" << std::hex << deviceCode << ")" << std::dec;
            }
        }
    }

    usb_close(handle);

    return 0;
}
