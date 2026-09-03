#pragma once
#include <atomic>
#include <thread>
#include <array>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <string>
#include <stdexcept>
#include <vector>
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
// ─── 1. SPSC 无锁环形队列（解析线程 push，写入线程 pop，零互斥锁）───
template<typename T, size_t CAP>
class SPSCQueue {
    static_assert((CAP& (CAP - 1)) == 0, "CAP must be power of 2");
public:
    bool push(const T& val) noexcept {
        const size_t w = write_.load(std::memory_order_relaxed);
        const size_t next = (w + 1) & (CAP - 1);
        if (next == read_.load(std::memory_order_acquire))
            return false; // 队列满，丢弃（可按需改为自旋等待）
        buf_[w] = val;
        write_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& val) noexcept {
        const size_t r = read_.load(std::memory_order_relaxed);
        if (r == write_.load(std::memory_order_acquire))
            return false; // 队列空
        val = buf_[r];
        read_.store((r + 1) & (CAP - 1), std::memory_order_release);
        return true;
    }

    size_t size() const noexcept {
        return (write_.load() - read_.load()) & (CAP - 1);
    }

private:
    alignas(64) std::atomic<size_t> write_{ 0 }; // 独占 cache line，避免伪共享
    alignas(64) std::atomic<size_t> read_{ 0 };
    std::array<T, CAP> buf_{};
};

// ─── 2. 数据帧定义 ───
struct DataFrame {
    double ts;          // Datas[0] 时间戳 ms
    double pressure;    // Datas[1]
    double resistance;  // Datas[2]
    double X;  // Datas[7]
    double pos[4];      // Datas[3~6]
};

// ─── 3. 低延迟文件写入器 ───
class LowLatencyFileWriter {
public:
    // 可调参数
    static constexpr size_t QUEUE_CAP = 1 << 14;  // 16384 条，按需调整
    static constexpr size_t IO_BUF_BYTES = 1 << 20;  // 1 MB 用户态写缓冲
    static constexpr size_t BATCH_MAX = 256;       // 每批最多处理条数
    static constexpr uint32_t FLUSH_US = 2000;      // 最大刷盘间隔 2 ms

    explicit LowLatencyFileWriter(const std::string& path) {
        // fp_ = std::fopen(path.c_str(), "wb");
        errno_t err = fopen_s(&fp_, path.c_str(), "wb");
        if (!fp_) throw std::runtime_error("Cannot open file: " + path);

        // 替换 C 库默认缓冲为我们自管理的大缓冲，减少系统调用次数
        std::setvbuf(fp_, nullptr, _IONBF, 0); // 先关掉 C 库自带缓冲
        ioBuf_.resize(IO_BUF_BYTES);
        ioHead_ = 0;

        // 写文件头
        writeHeader();

        // 启动独立写入线程，设置高优先级（Linux）
        running_.store(true, std::memory_order_relaxed);
        writerThread_ = std::thread(&LowLatencyFileWriter::writerLoop, this);
        setPriority(writerThread_);
    }

    ~LowLatencyFileWriter() {
        running_.store(false, std::memory_order_release);
        if (writerThread_.joinable()) writerThread_.join();
        flushToDisk(); // 确保残余数据落盘
        if (fp_) std::fclose(fp_);
    }

    // 解析线程调用，极低开销（仅一次队列 push）
    inline void submit(const DataFrame& frame) noexcept {
        if (!queue_.push(frame)) {
            dropCount_.fetch_add(1, std::memory_order_relaxed); // 统计丢帧
          
        }

    }

    uint64_t droppedFrames() const noexcept {
        return dropCount_.load(std::memory_order_relaxed);
    }

private:
    // ─── 写入线程主循环 ───
    void writerLoop() {
        DataFrame frames[BATCH_MAX];
        auto lastFlush = now_us();

        while (running_.load(std::memory_order_acquire) || queue_.size() > 0) {
            size_t count = 0;

            // 批量 pop
            while (count < BATCH_MAX && queue_.pop(frames[count]))
                ++count;

            if (count > 0) {
                for (size_t i = 0; i < count; ++i)
                    formatAndBuffer(frames[i]);
            }
            else {
                // 队列空时短暂让出 CPU，避免空转耗电
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            // 定时强制刷盘（保证最大延迟 FLUSH_US）
            uint64_t t = now_us();
            if (ioHead_ > 0 && (t - lastFlush) >= FLUSH_US) {
                flushToDisk();
                lastFlush = t;
            }
        }
    }

    // ─── 格式化为自定义文本并写入用户态缓冲 ───
    // 格式：timestamp,pressure,resistance,p0,p1,p2,p3\n
    // 使用 snprintf → 避免 std::string 堆分配
    void formatAndBuffer(const DataFrame& f) {
        // 预留最坏情况字节数（7 个 double + 分隔符）
        constexpr size_t LINE_MAX = 160;
        if (ioHead_ + LINE_MAX > IO_BUF_BYTES)
            flushToDisk();

        int n = std::snprintf(
            ioBuf_.data() + ioHead_,
            LINE_MAX,
            /*"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            f.ts, f.pressure, f.resistance,f.X,
            f.pos[0], f.pos[1], f.pos[2], f.pos[3]*/
            "%.3f,%.6f,%.6f,%.6f\n",
            f.ts, f.pressure, f.resistance, f.X
        );
        if (n > 0) ioHead_ += static_cast<size_t>(n);
    }

    // ─── 单次系统调用刷入磁盘 ───
    void flushToDisk() {
        if (ioHead_ == 0) return;
        std::fwrite(ioBuf_.data(), 1, ioHead_, fp_);
        // 不调用 fflush/fsync（交由 OS page cache 管理），
        // 如需掉电安全可在此加 fflush(fp_)
        ioHead_ = 0;
    }

    void writeHeader() {
        const char* hdr = "timestamp_ms,pressure,resistance,X,pos0,pos1,pos2,pos3\n";
        std::fwrite(hdr, 1, std::strlen(hdr), fp_);
    }

    static uint64_t now_us() noexcept {
        using namespace std::chrono;
        return static_cast<uint64_t>(
            duration_cast<microseconds>(
                steady_clock::now().time_since_epoch()
            ).count()
            );
    }

    static void setPriority(std::thread& t) {
#ifdef __linux__
        sched_param sp{ .sched_priority = 40 };
        pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sp);
#endif
    }

    FILE* fp_ = nullptr;
    SPSCQueue<DataFrame, QUEUE_CAP> queue_;
    std::vector<char> ioBuf_;
    size_t   ioHead_ = 0;
    std::thread writerThread_;
    std::atomic<bool>     running_{ false };
    std::atomic<uint64_t> dropCount_{ 0 };
};