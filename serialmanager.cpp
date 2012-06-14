#include "serialmanager.h"
#include <QStringList>


// --- [CONSTRUCTOR] -----------------------------------------------------------------------

SerialManager::SerialManager(QObject *parent) : QThread(parent)
{
    this->m_ready = false;
    this->m_portHandler = -1;
    this->m_portSetuped = false;
}

// --- [DISTRUCTOR] ------------------------------------------------------------------------

SerialManager::~SerialManager()
{
    if (isOpen()) {
        this->ClosePort();
    }
    delete this->m_pOptions;
}

// --- [SetupSerialPort] -------------------------------------------------------------------

void SerialManager::SetupSerialPort(PortOptions *opt)
{
    this->m_pOptions = opt;
    this->m_portSetuped = true;
    this->wait();
}

// --- [StartListen] -----------------------------------------------------------------------

void SerialManager::StartListen(ReadType type)
{
    this->m_ready = true;
    this->m_isStarted = true;
    this->m_readType = type;
    start();

}

// --- [StopListen] ------------------------------------------------------------------------

void SerialManager::StopListen()
{
    m_ready = false;
    this->m_isStarted = false;
    this->wait();
}

// --- [OpenPort] --------------------------------------------------------------------------

bool SerialManager::OpenPort()
{
    bool retVal = true;

    qDebug() << "Opening Port: [" << m_pOptions->portName.toLatin1().data() << "]";

    if(-1 == (m_portHandler = open(m_pOptions->portName.toLatin1().data(), O_RDWR | O_NOCTTY | O_NDELAY))) {
        retVal = false;
    } else {
        fcntl(m_portHandler, F_SETFL, 0);

        termios options;
        bzero(&options, sizeof(options));

        options.c_cflag |= CREAD | CLOCAL;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        tcflush(m_portHandler, TCIFLUSH);

        if (tcsetattr(m_portHandler, TCSANOW, & options) < 0) {
            retVal = false;
        } else {
            fcntl(m_portHandler, F_SETFL, FNDELAY);
            retVal  =true;
        }
    }
    if (!retVal) {
        qDebug() << "Error on opening port: [" << m_pOptions->portName.toLatin1().data() << "]";
    } else {
        qDebug() << "Port [" << m_pOptions->portName.toLatin1().data() << "] opened successfully.";
    }
    return retVal;
}

// --- [ClosePort] -------------------------------------------------------------------------

bool SerialManager::ClosePort()
{
    if (m_portHandler >= 0) {
        close(m_portHandler);
        m_portHandler = -1;
    }
    return true;
}

// --- [isOpen] ----------------------------------------------------------------------------

bool SerialManager::isOpen()
{
    return (m_portHandler != -1);
}

// --- [SetPortConfig] ---------------------------------------------------------------------

bool SerialManager::SetPortConfig()
{
    int baudRate = m_pOptions->baudRate;
    int bits = m_pOptions->charSize;
    int parity = m_pOptions->parity;
    int stopBits = m_pOptions->stopBits;
    bool enableFlowControl = m_pOptions->enableFlowControl;

    if(!isOpen())
        return false;

    int BR;

    //=== Set BaudRate:
    switch(baudRate){
    case 50: BR = B50; break;
    case 75: BR = B75; break;
    case 110: BR = B110; break;
    case 134: BR = B134; break;
    case 200: BR = B200; break;
    case 300: BR = B300; break;
    case 600: BR = B600; break;
    case 1200: BR = B1200; break;
    case 2400: BR = B2400; break;
    case 4800: BR = B4800; break;
    case 9600: BR = B9600; break;
    case 19200: BR = B19200; break;
    case 38400: BR = B38400; break;
    case 57600: BR = B57600; break;
    case 115200: BR = B115200; break;
    default: return false;
    }
    m_pOptions->baudRate = baudRate;


    //=== Get current configuration of port.
    termios options;
    if(tcgetattr(m_portHandler, &options) < 0) return false;

    //=== Set BaudRate for read and write.
    if((cfsetispeed(&options, BR) < 0) || (cfsetospeed(&options,BR) < 0))
        return false;


    //=== Set Character size.
    options.c_cflag &= ~CSIZE;

    switch(bits){
    case 5: options.c_cflag |= CS5; break;
    case 6: options.c_cflag |= CS6; break;
    case 7: options.c_cflag |= CS7; break;
    case 8: options.c_cflag |= CS8; break;
    default: return false;
    }

    //=== Set Parity.
    //===   0: NONE, 1: ODD, 2: EVEN
    switch(parity){
    case 2:
        options.c_cflag |= PARENB ;
        options.c_cflag &= ~PARODD ;
        options.c_iflag |= INPCK ;
        break ;
    case 1:
        options.c_cflag |= ( PARENB | PARODD );
        options.c_iflag |= INPCK;
        break ;
    case 0:
        options.c_cflag &= ~(PARENB);
        options.c_iflag |= IGNPAR;
        break ;
    default: return false;
    }

    //=== Set Stop Bit
    switch(stopBits){
    case 1: options.c_cflag &= ~(CSTOPB); break;
    case 2: options.c_cflag |= CSTOPB; break;
    default: return false;
    }

    //=== Set Flow Control Enabled or not.
    if (enableFlowControl)
    {
        // RTS/CTS ON:
        options.c_cflag |= CRTSCTS ;
    }
    else
    {
        // none
        options.c_cflag &= ~(CRTSCTS) ;
    }

    //=== Write New settings to port
    if(tcsetattr(this->m_portHandler, TCSANOW, &options) < 0 )
        return false;

    qDebug("Settings Saved successfully");
    return true;
}

// --- [SetPortTimeout] --------------------------------------------------------------------

bool SerialManager::SetPortTimeout(int readInterval, int readTotal)
{
    // Port must be open!
    if (!isOpen())
        return false;

    // Save variables which are used in other methods:
    m_pOptions->totalTimeout_ms      = readTotal;
    m_pOptions->interBytesTimeout_ms = readInterval;

    // VMIN & VTIME
    termios options;
    if ( tcgetattr( m_portHandler, &options ) < 0 )
        return false;

    // We set VMIN=0 and VTIME=ReadIntervalTimeout (in thenth of seconds)
    options.c_cc[ VMIN  ] = 0;
    options.c_cc[ VTIME ] = (readTotal / 100 > 1 ? readTotal / 100 : 1);
    //max(1,ReadTotalTimeoutConstant / 100);

    // Write the new settings to the port.
    if ( tcsetattr( m_portHandler,TCSANOW,&options ) < 0 )
        return false;

    return true;
}

// --- [ReadPort] --------------------------------------------------------------------------

QString SerialManager::ReadPort()
{
    if(!isOpen())
        return "";
    int numRead = 0;
    if((0 > (numRead = read(m_portHandler, buffer, MAX_BUFFER_SIZE))))
        return "";

    buffer[numRead] = '\0';
    QString str = buffer;

    return str;
}

// --- [ReadPortBinary] --------------------------------------------------------------------

QByteArray* SerialManager::ReadPortBinary()
{
    if(!isOpen())
        return NULL;

    int numRead = 0;
    if((0 > (numRead = read(m_portHandler, buffer, MAX_BUFFER_SIZE))))
        return NULL;

    QByteArray* ba = new QByteArray(buffer, numRead);
    return ba;
}

// --- [run] -------------------------------------------------------------------------------

void SerialManager::run()
{
    if (!this->m_portSetuped) {

        qDebug("Port Not Configged yet.");

    } else {        //Port is ready for start listening.

        if( this->OpenPort() && this->SetPortConfig() && this->SetPortTimeout() ) {

            QString str = this->ReadPort();
            qRegisterMetaType< QString >("QString");

            emit SerialPortIsReady();


            if (m_readType == ASCII) {

                while (this->m_ready) {
                    QStringList strLines = str.split("\r");
                    for (int i = 0; i < strLines.count() - 1; ++i) {
                        emit LineDetected(strLines[i]);
                    }
                    str = strLines[strLines.count() - 1];
                    str += ReadPort();
                    msleep(100);
                }
            } else if (this->m_ready == BINARY) {

                while (m_ready) {
                    QByteArray *ba = this->ReadPortBinary();

                    if (ba != NULL && ba->length() > 0) {
                        emit BinaryDataReceived(ba);
                    }
                    msleep(100);
                }
            }
        }
    }
}

// --- [WriteToPort] -----------------------------------------------------------------------

void SerialManager::WriteToPort(QByteArray cmd)
{
    static int cntr = 0;
    if(!this->isOpen())
        return;
    int n = write(m_portHandler, cmd.data(), cmd.length());
    if (n < 0)
        qDebug("Write failed");
    else
        qDebug() << "[WRITE:" << cntr++ << "]: " << cmd;
    msleep(100);
}

// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
// --- [] ----------------------------------------------------------------------------------
