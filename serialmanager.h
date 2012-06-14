#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <QThread>
#include <QDebug>
#include <QMetaType>
#define MAX_BUFFER_SIZE 1000

struct PortOptions
{
public:
    QString portName;
    int baudRate;
    int parity;
    int charSize;
    int stopBits;
    bool enableFlowControl;
    int totalTimeout_ms;
    int interBytesTimeout_ms;

    PortOptions(){
        this->baudRate = 115200;
        this->parity = 0;
        this->charSize = 8;
        this->stopBits = 1;
        this->portName = "";
        this->enableFlowControl = false;
        this->totalTimeout_ms = 0;
        this->interBytesTimeout_ms = 0;
    }
    PortOptions(QString pname){
        this->baudRate = 115200;
        this->parity = 0;
        this->charSize = 8;
        this->stopBits = 1;
        this->portName = pname;
        this->enableFlowControl = false;
        this->totalTimeout_ms = 0;
        this->interBytesTimeout_ms = 0;
    }
};

class SerialManager : public QThread
{
    Q_OBJECT
public:
    enum ReadType { ASCII, BINARY };

    SerialManager(QObject *parent = 0);
    ~SerialManager();

    void SetupSerialPort(PortOptions *opt);
    void StartListen(ReadType type);
    void StopListen();
    bool isStarted() { return m_isStarted; }
    void WriteToPort(QByteArray cmd);

protected:
    void run();

signals:
    void LineDetected(QString line);
    void BinaryDataReceived(QByteArray *data);
    void SerialPortIsReady();

private:
    bool OpenPort();
    bool ClosePort();
    bool isOpen();
    bool SetPortConfig();
    bool SetPortTimeout(int readInterval = 0, int readTotal = 0);
    bool m_isStarted;
    QString ReadPort();
    QByteArray* ReadPortBinary();

    //Variables:
    bool m_ready;
    bool m_portSetuped;
    PortOptions *m_pOptions;
    int m_portHandler;
    ReadType m_readType;
    char buffer[MAX_BUFFER_SIZE];
};

#endif // SERIALMANAGER_H
