#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <QFile>
#include <QElapsedTimer>
#include <QByteArray>
#include <QDataStream>
#include <windows.h>
#include <algorithm>
#include "qdebug.h"
// 函数封装，参数设置，先用引用，再用指针，再用变量
class RingBuffer {



public:
    explicit RingBuffer(size_t size)
        : buffer(size), capacity(size), head(0), tail(0) {
    }


    size_t coverSize() const {
        if (tail >= head)
            return tail - head;
        return capacity - head + tail;
    }

    size_t noReadSize() const {
        return capacity - coverSize();
    }

    // 写入数据 覆盖旧数据
    size_t write(const char* data, size_t len) {
        if (len == 0) return 0;
        size_t size = coverSize();
        /*qDebug() << "data:" << data << endl;
        qDebug() <<"size:" << size << endl;*/
        // 计算容量
        if (len >= capacity) {
            data += (len - capacity);
            len = capacity;

            head = 0;
            tail = 0;
            size = 0;
        }
        // 计算指针
        size_t freeSpace = capacity - size;
        if (len > freeSpace) {
            size_t overflow = len - freeSpace;

            tail = (tail + overflow) % capacity;
            size -= overflow;
        }
        // 分段写入
        size_t firstPart = std::min<size_t>(len, capacity - head);
        std::copy(data, data + firstPart, buffer.begin() + head);

        size_t secondPart = len - firstPart;
        if (secondPart > 0) {
            std::copy(data + firstPart, data + len, buffer.begin());
        }
        // 更新指针
        head = (head + len) % capacity;
        size += len;
        return len;
    }

    // 读取数据
    size_t read(char* out, size_t len) {
        size_t toRead = std::min<size_t>(len, coverSize());
        size_t idx = head % capacity;
        size_t chunk = std::min<size_t>(toRead, capacity - idx);

        std::copy(buffer.begin() + idx, buffer.begin() + idx + chunk, out);

        if (chunk < toRead) {
            std::copy(buffer.begin(), buffer.begin() + (toRead - chunk), out + chunk);
        }

        head = (head + toRead) % capacity;
        return toRead;
    }

    // 按照间隔符读取一帧数据
    int  readFrameByDelimiter(std::string& frame, char delimiter)
    {
        frame.clear();

        size_t pos = tail;
        bool found = false;
        // 1. 查找
        while (pos!=head) {
            if (buffer[pos] == delimiter)
            {
                found = true;
                break;
            }
            
            pos = (pos + 1) % capacity;

        }

        // qDebug() << pos << endl;
        if (!found)
        {
            // qDebug() << "Error" << endl;
            frame.clear();
            return 1;
        }
        // 保存数据
        size_t cur = tail;
        int i = 0;
        while (cur != pos) {

 
            frame.push_back(buffer[cur]);
            cur = (cur + 1) % capacity;
            i++;
        }
        // 更新指针
        tail = (pos + 1) % capacity;

        return 0;

    }

    // 数据写入文件中
    //int writeToFileR(const QString& fileName) {

    //    // 写入过程初始化
    //    file.setFileName(fileName);
    //    file.open(QIODevice::WriteOnly | QIODevice::Append);
    //    stream.setDevice(&file);

    //    timer.start();

    //    std::string buffer;
    //    const char* cbuffer[20];
    //    while (true)
    //    {
    //        int len = readFrameByDelimiter(buffer, '\n');
    //        if (len <= 0)
    //            break;

    //        qint64 timestamp = timer.nsecsElapsed(); // 纳秒级
    //        writeData(timestamp, buffer.c_str(), len);
    //    }
    //    file.close();

    //    return 0;
    //}

private:

    std::vector<char> buffer;  // 存储数据的缓冲区
    size_t capacity;           // 缓冲区总容量
    size_t head;               // 读取位置指针
    size_t tail;               // 写入位置指针
    std::string cache;         // 帧数据
    size_t currentSize = 0;
    char tempFrame[20];


    QFile file;
    QDataStream stream;
    QElapsedTimer timer;
    void writeData(qint64 ts, const char* data, int len)
    {
        // 二进制格式写入（高性能）
        stream.writeRawData(reinterpret_cast<const char*>(&ts), sizeof(ts));
        stream.writeRawData(reinterpret_cast<const char*>(&len), sizeof(len));
        stream.writeRawData(data, len);
    }





};