#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "v4l2wrapper.hpp"

namespace V4L2Wrapper {

    Device::Device()
        : fd(-1)
        , req()
        , buffer_start()
        , buf()
    {
    }

    Device::~Device()
    {
        destroyBuffers();
        close();
    }

    Status Device::open(const char *path)
    {
        fd = ::open(path, O_RDWR);
        if (fd == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::close()
    {
        if (fd == -1)
            return Ok;
        int ret = ::close(fd);
        fd = -1;
        if (ret == -1) {
            return Error;
        }
        return Ok;
    }

    bool Device::isOpen() const
    {
        return fd != -1;
    }

    Status Device::getCapability(struct v4l2_capability &cap)
    {
        if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::setFormat(struct v4l2_format &fmt)
    {
        if (::ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::requestBuffers(struct v4l2_requestbuffers &req)
    {
        if (::ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::queryBuffer(struct v4l2_buffer &buf)
    {
        if (::ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::mapBuffer(struct v4l2_buffer &buf, void **buffer)
    {
        void *ret = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (ret == MAP_FAILED) {
            return Error;
        }
        *buffer = ret;
        return Ok;
    }

    Status Device::unmapBuffer(struct v4l2_buffer &buf, void *buffer)
    {
        if (munmap(buffer, buf.length) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::createBuffers(uint32_t count)
    {
        req.count = count;  // Set number of buffers
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;  // Use memory mapping

        if (requestBuffers(req) == Error) {
            return Error;
        }

        for (int i = 0; i < req.count; ++i) {
            buf.resize(i + 1);

            memset(&buf[i], 0, sizeof(struct v4l2_buffer));
            buf[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf[i].memory = V4L2_MEMORY_MMAP;
            buf[i].index = i;

            if (queryBuffer(buf[i]) == V4L2Wrapper::Error) {
                destroyBuffers();
                return Error;
            }

            buffer_start.resize(i + 1);

            if (mapBuffer(buf[i], &buffer_start[i]) == V4L2Wrapper::Error) {
                buffer_start.resize(i);
                destroyBuffers();
                return Error;
            }

            // Now 'buffer_start' points to the buffer memory in user space.
        }

        for (int i = 0; i < req.count; ++i) {
            // Queue the buffer
            if (queueBuffer(buf[i]) == V4L2Wrapper::Error) {
                return Error;
            }
        }

        return Ok;
    }

    Status Device::destroyBuffers()
    {
        for (int i = 0; i < buffer_start.size(); ++i) {
            if (unmapBuffer(buf[i], buffer_start[i]) == V4L2Wrapper::Error) {
                return Error;
            }
        }
        buffer_start.clear();
        buf.clear();
        return Ok;
    }

    Status Device::startStream(enum v4l2_buf_type type)
    {
        if (::ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::stopStream(enum v4l2_buf_type type)
    {
        if (::ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::dequeueBuffer(struct v4l2_buffer &buf)
    {
        if (::ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::queueBuffer(struct v4l2_buffer &buf)
    {
        if (::ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            return Error;
        }
        return Ok;
    }

    Status Device::setControl(const struct v4l2_control &ctrl)
    {
        if (::ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
            return Error;
        }
        return Ok;
    }

    Buffer::Buffer(Device &dev)
        : dev(dev)
    {
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue a filled buffer
        if (dev.dequeueBuffer(buf) == V4L2Wrapper::Error) {
            status = Error;
        }
        else
        {
            buffer_start = dev.buffer_start[buf.index];
            status = Ok;
        }
    }

    Buffer::~Buffer()
    {
        if (status == Ok)
        {
            // Queue the buffer back for reuse
            dev.queueBuffer(buf);
        }
    }

#if 0
    Status Device::setupBuffers(int count)
    {
        struct v4l2_requestbuffers req;
        req.count = BUFFER_COUNT;  // Set number of buffers
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;  // Use memory mapping
        if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
            return Error;
        }

        void *buffer_start[BUFFER_COUNT];
        struct v4l2_buffer buf[BUFFER_COUNT];

        for (int i = 0; i < req.count; ++i) {
            memset(&buf[i], 0, sizeof(struct v4l2_buffer));
            buf[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf[i].memory = V4L2_MEMORY_MMAP;
            buf[i].index = i;

            if (ioctl(fd, VIDIOC_QUERYBUF, &buf[i]) == -1) {
                perror("Querying buffer");
                return -1;
            }

            buffer_start[i] = mmap(NULL, buf[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf[i].m.offset);
            if (buffer_start[i] == MAP_FAILED) {
                perror("Mapping buffer");
                return -1;
            }

            // Now 'buffer_start' points to the buffer memory in user space.
        }

        for (int i = 0; i < req.count; ++i) {
            // Queue the buffer
            if (ioctl(fd, VIDIOC_QBUF, &buf[i]) == -1) {
                perror("Queue buffer");
                return -1;
            }
        }

    }
#endif

} // namespace V4L2Wrapper

std::string v4l2_fourcc_to_str(uint32_t value)
{
    return std::string(reinterpret_cast<const char *>(&value), 4);
}

std::string v4l2_field_to_str(uint32_t value)
{
    switch (value)
    {
        case V4L2_FIELD_ANY: return "V4L2_FIELD_ANY";
        case V4L2_FIELD_NONE: return "V4L2_FIELD_NONE";
        case V4L2_FIELD_TOP: return "V4L2_FIELD_TOP";
        case V4L2_FIELD_BOTTOM: return "V4L2_FIELD_BOTTOM";
        case V4L2_FIELD_INTERLACED: return "V4L2_FIELD_INTERLACED";
        case V4L2_FIELD_SEQ_TB: return "V4L2_FIELD_SEQ_TB";
        case V4L2_FIELD_SEQ_BT: return "V4L2_FIELD_SEQ_BT";
        case V4L2_FIELD_ALTERNATE: return "V4L2_FIELD_ALTERNATE";
        case V4L2_FIELD_INTERLACED_TB: return "V4L2_FIELD_INTERLACED_TB";
        case V4L2_FIELD_INTERLACED_BT: return "V4L2_FIELD_INTERLACED_BT";
    }
    return "(unknown)";
}

std::string v4l2_buf_type_to_str(uint32_t value)
{
    switch (value)
    {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE: return "V4L2_BUF_TYPE_VIDEO_CAPTURE";
        case V4L2_BUF_TYPE_VIDEO_OUTPUT: return "V4L2_BUF_TYPE_VIDEO_OUTPUT";
        case V4L2_BUF_TYPE_VIDEO_OVERLAY: return "V4L2_BUF_TYPE_VIDEO_OVERLAY";
        case V4L2_BUF_TYPE_VBI_CAPTURE: return "V4L2_BUF_TYPE_VBI_CAPTURE";
        case V4L2_BUF_TYPE_VBI_OUTPUT: return "V4L2_BUF_TYPE_VBI_OUTPUT";
        case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE: return "V4L2_BUF_TYPE_SLICED_VBI_CAPTURE";
        case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT: return "V4L2_BUF_TYPE_SLICED_VBI_OUTPUT";
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY: return "V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY";
        case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE: return "V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE";
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE: return "V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE";
        case V4L2_BUF_TYPE_SDR_CAPTURE: return "V4L2_BUF_TYPE_SDR_CAPTURE";
        case V4L2_BUF_TYPE_SDR_OUTPUT: return "V4L2_BUF_TYPE_SDR_OUTPUT";
        case V4L2_BUF_TYPE_META_CAPTURE: return "V4L2_BUF_TYPE_META_CAPTURE";
        case V4L2_BUF_TYPE_META_OUTPUT: return "V4L2_BUF_TYPE_META_OUTPUT";
        case V4L2_BUF_TYPE_PRIVATE: return "V4L2_BUF_TYPE_PRIVATE";
    }
    return "(unknown)";
}

bool isVideoCapture(uint32_t cap) { return (cap & V4L2_CAP_VIDEO_CAPTURE) != 0; }
bool isVideoOutput(uint32_t cap) { return (cap & V4L2_CAP_VIDEO_OUTPUT) != 0; }
bool canVideoOverlay(uint32_t cap) { return (cap & V4L2_CAP_VIDEO_OVERLAY) != 0; }
bool isVBICapture(uint32_t cap) { return (cap & V4L2_CAP_VBI_CAPTURE) != 0; }
bool isVBIOutput(uint32_t cap) { return (cap & V4L2_CAP_VBI_OUTPUT) != 0; }
bool isSlicedVBICapture(uint32_t cap) { return (cap & V4L2_CAP_VBI_CAPTURE) != 0; }
bool isSlicedVBIOutput(uint32_t cap) { return (cap & V4L2_CAP_VBI_OUTPUT) != 0; }
bool isRDSDataCapture(uint32_t cap) { return (cap & V4L2_CAP_RDS_CAPTURE) != 0; }
bool canVideoOutputOverlay(uint32_t cap) { return (cap & V4L2_CAP_VIDEO_OUTPUT_OVERLAY) != 0; }
bool canHardwareFrequencySeek(uint32_t cap) { return (cap & V4L2_CAP_HW_FREQ_SEEK) != 0; }
bool isRDSEncoder(uint32_t cap) { return (cap & V4L2_CAP_RDS_OUTPUT) != 0; }

std::ostream& operator<< (std::ostream& out, const struct v4l2_capability &cap)
{
    out << "Driver: " << std::string(reinterpret_cast<const char *>(cap.driver), sizeof(cap.driver)) << std::endl
        << "Card: " << std::string(reinterpret_cast<const char *>(cap.card), sizeof(cap.card)) << std::endl
        << "Bus Info: " << std::string(reinterpret_cast<const char *>(cap.bus_info), sizeof(cap.bus_info)) << std::endl
        << "Version: 0x" << std::hex << cap.version << std::dec << std::endl
        << "Capabilities:" << std::endl
        << "\tVideo Capture: " << isVideoCapture(cap.capabilities) << std::endl
        << "\tVideo Output: " << isVideoOutput(cap.capabilities) << std::endl
        << "\tVideo Overlay: " << canVideoOverlay(cap.capabilities) << std::endl
        << "\tVBI Capture: " << isVBICapture(cap.capabilities) << std::endl
        << "\tVBI Output: " << isVBIOutput(cap.capabilities) << std::endl
        << "\tSliced VBI Capture: " << isSlicedVBICapture(cap.capabilities) << std::endl
        << "\tSliced VBI Output: " << isSlicedVBIOutput(cap.capabilities) << std::endl
        << "\tRDS Data Capture: " << isRDSDataCapture(cap.capabilities) << std::endl
        << "\tVideo Output Overlay: " << canVideoOutputOverlay(cap.capabilities) << std::endl
        << "\tHardware Frequency Seek: " << canHardwareFrequencySeek(cap.capabilities) << std::endl
        << "\tRDS Encoder: " << isRDSEncoder(cap.capabilities) << std::endl
        << "Device Caps: 0x" << std::hex << cap.device_caps << std::dec << std::endl
        << "Reserved[0]: 0x" << std::hex << cap.reserved[0] << std::dec << std::endl
        << "Reserved[1]: 0x" << std::hex << cap.reserved[1] << std::dec << std::endl
        << "Reserved[2]: 0x" << std::hex << cap.reserved[2] << std::dec << std::endl;
    return out;
}

std::ostream& operator<< (std::ostream& out, const struct v4l2_pix_format &pix_fmt)
{
    out << "Pixel Format:" << std::endl
        << "\tWidth: " << pix_fmt.width << std::endl
        << "\tHeight: " << pix_fmt.height << std::endl
        << "\tPixel Format: " << v4l2_fourcc_to_str(pix_fmt.pixelformat) << std::endl
        << "\tField: " << v4l2_field_to_str(pix_fmt.field) << std::endl
        << "\tBytes Per Line: " << pix_fmt.bytesperline << std::endl
        << "\tSize Image: " << pix_fmt.sizeimage << std::endl
        << "\tColorspace: " << pix_fmt.colorspace << std::endl
        << "\tPriv: 0x" << std::hex << pix_fmt.priv << std::dec << std::endl
        << "\tFlags: " << pix_fmt.flags << std::endl
        << "\tYCbCr Encoding: " << pix_fmt.ycbcr_enc << std::endl
        << "\tHSV Encoding: " << pix_fmt.hsv_enc << std::endl
        << "\tQuantization: " << pix_fmt.quantization << std::endl
        << "\tXfer Func: " << pix_fmt.xfer_func << std::endl;
    return out;
}

std::ostream& operator<< (std::ostream& out, const struct v4l2_format &fmt)
{
    out << "Format:" << std::endl
        << "\tType: " << v4l2_buf_type_to_str(fmt.type) << std::endl;
    switch (fmt.type)
    {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE: out << fmt.fmt.pix;
    }
    return out;
}

V4L2Wrapper::Device& operator<< (V4L2Wrapper::Device& dev, const struct v4l2_control &ctrl)
{
    dev.setControl(ctrl);
    return dev;
}
