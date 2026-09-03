#pragma once
#include <iostream>
#include <vector>
#include <stdexcept>

// 关于类模板
// 1. 定义数组类时，需要使用类似的：std::arryay<double,4>
// 2. 引用类模板头文件时，不能.cpp和.h中同时include

template <typename T>
class CirBuffer {

public:
    CirBuffer(size_t size) : buffer(size), head(0), tail(0), maxSize(size) {}

    // 写数据到环形缓冲区
    void write(const T& data) {
        if (isFull()) {
            // std::cout << "Buffer is full, overwriting the oldest data." << std::endl;
            head = (head + 1) % maxSize;  // 覆盖最旧的数据
        }
        buffer[tail] = data;
        tail = (tail + 1) % maxSize;  // 更新写指针,完美循环
    }

    // 从环形缓冲区读取数据
    T read() {
        if (isEmpty()) {
            throw std::underflow_error("Buffer is empty, cannot read data.");
        }
        T data = buffer[head];
        head = (head + 1) % maxSize;  // 更新读指针
        return data;
    }

    // 返回缓冲区的大小
    size_t size() const {
        if (tail >= head) {
            return tail - head;
        }
        else {
            return maxSize - head + tail;
        }
    }

    // 返回缓冲区的最大容量
    size_t capacity() const {
        return maxSize - 1;  // 由于头尾指针的重合问题，最大容量是总大小减去1
    }

    // 显示缓冲区中的数据
    void display() const {
        std::cout << "Buffer contents: ";
        size_t i = head;
        while (i != tail) {
            // std::cout << buffer[i] << " ";
            i = (i + 1) % maxSize;
        }
        std::cout << std::endl;
    }

    bool isFull() const { return (tail + 1) % maxSize == head; }  // 判断缓冲区是否满
    bool isEmpty() const { return head == tail; }  // 判断缓冲区是否空

private:
    std::vector<T> buffer;     // 用于存储数据的缓冲区
    size_t head;              // 读指针
    size_t tail;              // 写指针
    size_t maxSize;           // 缓冲区的最大容量
};