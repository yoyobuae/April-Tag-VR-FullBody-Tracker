#ifndef __V4L2WRAPPER_HPP__
#define __V4L2WRAPPER_HPP__

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <linux/videodev2.h>

namespace V4L2Wrapper {
    enum Status {
        Ok,
        Error,
    };

    class Device;

    class Buffer {
        struct v4l2_buffer buf;
        void *buffer_start;
        Device &dev;
        Status status;
    public:
        Buffer(Device &dev);
        ~Buffer();

        Status getStatus() const { return status; }
        void *Data() { return buffer_start; }
        uint32_t Size() { return buf.bytesused; }

        friend class Device;
    };

    class Device {
        int fd;
        struct v4l2_requestbuffers req;
        std::vector<void *> buffer_start;
        std::vector<struct v4l2_buffer> buf;

        Status requestBuffers(struct v4l2_requestbuffers &req);
        Status queryBuffer(struct v4l2_buffer &buf);
        Status mapBuffer(struct v4l2_buffer &buf, void **buffer);
        Status unmapBuffer(struct v4l2_buffer &buf, void *buffer);
        Status dequeueBuffer(struct v4l2_buffer &buf);
        Status queueBuffer(struct v4l2_buffer &buf);
    public:
        Device();
        ~Device();

        Status open(const char *path);
        Status close();

        bool isOpen() const;

        Status getCapability(struct v4l2_capability &cap);
        Status setFormat(struct v4l2_format &fmt);
        Status createBuffers(uint32_t count);
        Status destroyBuffers();
        Status startStream(enum v4l2_buf_type type);
        Status stopStream(enum v4l2_buf_type type);
        Status setControl(const struct v4l2_control &ctrl);

        friend class Buffer;
    };

} // namespace V4L2Wrapper

std::ostream& operator<< (std::ostream& out, const struct v4l2_capability &cap);
std::ostream& operator<< (std::ostream& out, const struct v4l2_pix_format &pix_fmt);
std::ostream& operator<< (std::ostream& out, const struct v4l2_format &fmt);

V4L2Wrapper::Device& operator<< (V4L2Wrapper::Device& dev, const struct v4l2_control &ctrl);

#endif /* __V4L2WRAPPER_HPP__ */
