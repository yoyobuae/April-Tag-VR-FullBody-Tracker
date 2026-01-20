#ifndef __JPEGWRAPPER_HPP__
#define __JPEGWRAPPER_HPP__

#include <memory>

#include "jpeglib.h"

namespace JPEGWrapper {
    enum Status {
        Ok,
        Error,
    };

    typedef struct custom_error_mgr_s custom_error_mgr;

    class Decompress {
        struct jpeg_decompress_struct cinfo_;
        bool shouldCrop;
        JDIMENSION first_scanline;
        JDIMENSION last_scanline;
        JDIMENSION x_offset;
        JDIMENSION x_width;

        /* Error handling */
        std::unique_ptr<custom_error_mgr> err_mgr;

    public:
        Decompress();
        ~Decompress();

        const struct jpeg_decompress_struct& cinfo() const;

        /* Returns status of the decompress object. If status is Error, then object can no longer be used */
        Status status() const;

        /* Set source before calling `readHeader`. Check status() for errors */
        void setMemSource(const unsigned char *inbuffer, unsigned long insize);

        /* Check status() for errors */
        int readHeader(bool require_image);

        /* You can call these after `readHeader`, but before `start` */
        void setColorspace(J_COLOR_SPACE value);
        void setDCTMethod(J_DCT_METHOD value);
        void setDoFancyUpsampling(bool value);
        void setDoBlockSmoothing(bool value);
        void setScale(unsigned int num, unsigned int denom);

        /* Check status() for errors */
        void start();

        /* You can call this after `start`, but before `read` or `getOutputBufferSize`. Check status() for errors */
        void setCrop(unsigned int left, unsigned int top, unsigned int width, unsigned int height);

        unsigned long int getOutputBufferSize();

        /* If buffer can't fit the output image then buffer is not touched. Check status() for errors */
        void read(unsigned char *outbuffer, unsigned long int outsize);

        /* Call when done decompressing and before switching to a new source. Check status() for errors */
        void stop();

        /* You can call these at any point after `readHeader`, but before stop */
        JDIMENSION inputWidth() const;
        JDIMENSION inputHeight() const;

        /* You can call these at any point after `start` (and `setCrop` if cropping output image), but before stop */
        JDIMENSION outputLeft() const;
        JDIMENSION outputTop() const;
        JDIMENSION outputWidth() const;
        JDIMENSION outputHeight() const;
        int outputColorComponents() const;

    };

} // namespace V4L2Wrapper

std::ostream& operator<< (std::ostream& out, const struct jpeg_decompress_struct& cinfo);

#endif /* __JPEGWRAPPER_HPP__ */
