
#include "Tic.h"
#include <string.h>


// ---------------------------------------

class SerialStream: public Stream {
  int fd;
public:
  SerialStream(const char* dev);
  ~SerialStream();
  size_t write(uint8_t byte);
  size_t readBytes(char *buffer, size_t length);
};

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

SerialStream::SerialStream(const char* dev) {

  fd = open(dev,O_RDWR|O_NOCTTY/*|O_SYNC*/);

  if (fd == -1)
    return;
  
  printf("Open succ\n");

  struct termios settings;
  tcgetattr(fd, &settings);

  cfsetospeed(&settings, B9600); /* baud rate */
  settings.c_cflag &= ~PARENB; /* no parity */
  settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
  settings.c_lflag &= ~ICANON; /* canonical mode */
  settings.c_oflag &= ~OPOST; /* raw output */

  tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
  tcflush(fd, TCOFLUSH);
  printf("flush\n");
}

SerialStream::~SerialStream() {
  if (fd != -1)
    close(fd);
}
size_t SerialStream::write(uint8_t byte) {
  printf("write %0x\n",byte);
  size_t ret = ::write(fd,&byte,sizeof(byte));
  printf("written %d\n",ret);
  return ret;
}
size_t SerialStream::readBytes(char *buffer, size_t length) {
  printf("read request len = %d\n",length);  
  size_t ret = ::read(fd, buffer, length);
  printf("read %d bytes\n",ret);
  return ret;
}

// ---------------------------------------
#include <libusb.h>
class TicUsb: public TicBase {
  libusb_device_handle *handle;
public:
  TicUsb();
  ~TicUsb();

  void commandQuick(TicCommand cmd);
  void commandW32(TicCommand cmd, uint32_t val);
  void commandW7(TicCommand cmd, uint8_t val);
  void getSegment(TicCommand cmd, uint8_t offset, uint8_t length, void * buffer);


};

#define TIC_VENDOR_ID 0x1FFB
#define TIC_PRODUCT_ID_T825 0x00B3
#define TIC_PRODUCT_ID_T834 0x00B5
#define TIC_PRODUCT_ID_T500 0x00BD
#define TIC_PRODUCT_ID_N825 0x00C3
#define TIC_PRODUCT_ID_T249 0x00C9

TicUsb::TicUsb() {
  libusb_init(NULL);

  libusb_device** devs;
  ssize_t len = libusb_get_device_list(NULL, &devs);

  libusb_device** dev = NULL;
  for (dev = devs; *dev != NULL; ++dev) {
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(*dev, &desc);
    if (desc.idVendor == TIC_VENDOR_ID && desc.idProduct == TIC_PRODUCT_ID_T825) {
      printf("Found USB\n");
      break;
    }
  }

  if (dev && *dev) {
    libusb_open(*dev, &handle);
  }

  libusb_free_device_list(devs,1);

}
TicUsb::~TicUsb() {
  libusb_close(handle);
  libusb_exit(NULL);
}

void TicUsb::commandQuick(TicCommand cmd)  {

}
void TicUsb::commandW32(TicCommand cmd, uint32_t val)  {

}
void TicUsb::commandW7(TicCommand cmd, uint8_t val)  {

}
void TicUsb::getSegment(TicCommand cmd, uint8_t offset, uint8_t length, void * buffer)  {

  libusb_control_transfer(handle, 0xC0, (uint8_t)cmd, 0, offset, (unsigned char*)buffer, length, 0);
}


// ---------------------------------------
#include <stdio.h>
int main() {

//  TicUsb tusb;
//  uint16_t cl = tusb.getCurrentLimit();
//  printf("%d",cl);

  SerialStream ss("/dev/stdout");
  TicSerial ts(ss);

  ts.reset();
  uint16_t cl = ts.getCurrentLimit();

  printf("%d",cl);

	return 0;
}

// ---------------------------------------

static const uint16_t Tic03aCurrentTable[33] =
{
  0,
  1,
  174,
  343,
  495,
  634,
  762,
  880,
  990,
  1092,
  1189,
  1281,
  1368,
  1452,
  1532,
  1611,
  1687,
  1762,
  1835,
  1909,
  1982,
  2056,
  2131,
  2207,
  2285,
  2366,
  2451,
  2540,
  2634,
  2734,
  2843,
  2962,
  3093,
};

void TicBase::setCurrentLimit(uint16_t limit)
{
  uint8_t code = 0;

  if (product == TicProduct::T500)
  {
    for (uint8_t i = 0; i < 33; i++)
    {
      if (Tic03aCurrentTable[i] <= limit)
      {
        code = i;
      }
      else
      {
        break;
      }
    }
  }
  else if (product == TicProduct::T249)
  {
    code = limit / TicT249CurrentUnits;
  }
  else
  {
    code = limit / TicCurrentUnits;
  }

  commandW7(TicCommand::SetCurrentLimit, code);
}

uint16_t TicBase::getCurrentLimit()
{
  uint8_t code = getVar8(VarOffset::CurrentLimit);
  if (product == TicProduct::T500)
  {
    if (code > 32) { code = 32; }
    return Tic03aCurrentTable[code];
  }
  else if (product == TicProduct::T249)
  {
    return code * TicT249CurrentUnits;
  }
  else
  {
    return code * TicCurrentUnits;
  }
}

/**** TicSerial ****/

void TicSerial::commandW32(TicCommand cmd, uint32_t val)
{
  sendCommandHeader(cmd);

  // byte with MSbs:
  // bit 0 = MSb of first (least significant) data byte
  // bit 1 = MSb of second data byte
  // bit 2 = MSb of third data byte
  // bit 3 = MSb of fourth (most significant) data byte
  serialW7(((val >>  7) & 1) |
           ((val >> 14) & 2) |
           ((val >> 21) & 4) |
           ((val >> 28) & 8));

  serialW7(val >> 0); // least significant byte with MSb cleared
  serialW7(val >> 8);
  serialW7(val >> 16);
  serialW7(val >> 24); // most significant byte with MSb cleared

  _lastError = 0;
}

void TicSerial::commandW7(TicCommand cmd, uint8_t val)
{
  sendCommandHeader(cmd);
  serialW7(val);

  _lastError = 0;
}

void TicSerial::getSegment(TicCommand cmd, uint8_t offset,
  uint8_t length, void * buffer)
{
  length &= 0x7F;
  sendCommandHeader(cmd);
  serialW7(offset);
  serialW7(length);

  uint8_t byteCount = _stream->readBytes((uint8_t *)buffer, length);
  if (byteCount != length)
  {
    _lastError = 50;

    // Set the buffer bytes to 0 so the program will not use an uninitialized
    // variable.
    memset(buffer, 0, length);
    return;
  }

  _lastError = 0;
}

void TicSerial::sendCommandHeader(TicCommand cmd)
{
  if (_deviceNumber == 255)
  {
    // Compact protocol
    _stream->write((uint8_t)cmd);
  }
  else
  {
    // Pololu protocol
    _stream->write(0xAA);
    serialW7(_deviceNumber);
    serialW7((uint8_t)cmd);
  }
  _lastError = 0;
}

