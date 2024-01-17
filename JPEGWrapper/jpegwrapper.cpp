#include <iostream>
#include <string>
#include <vector>

#include "jpeglib.h"

#include "jpegwrapper.hpp"

namespace JPEGWrapper {

    Decompress::Decompress()
        : jerr()
        , cinfo_()
        , shouldCrop(false)
        , first_scanline(0)
        , last_scanline(0)
        , x_offset(0)
        , x_width(0)
    {
        cinfo_.err = jpeg_std_error(&jerr);
        jpeg_create_decompress(&cinfo_);
    }

    Decompress::~Decompress()
    {
        jpeg_destroy_decompress(&cinfo_);
    }

    const struct jpeg_decompress_struct& Decompress::cinfo() const
    {
        return cinfo_;
    }

    void Decompress::setMemSource(const unsigned char *inbuffer, unsigned long insize)
    {
        jpeg_mem_src(&cinfo_, inbuffer, insize);
    }

    int Decompress::readHeader(bool require_image)
    {
        return jpeg_read_header(&cinfo_, require_image);
    }

    void Decompress::setColorspace(J_COLOR_SPACE value)
    {
        cinfo_.out_color_space = value;
    }

    void Decompress::setDCTMethod(J_DCT_METHOD value)
    {
        cinfo_.dct_method = value;
    }

    void Decompress::setDoFancyUpsampling(bool value)
    {
        cinfo_.do_fancy_upsampling = value;
    }

    void Decompress::setDoBlockSmoothing(bool value)
    {
        cinfo_.do_block_smoothing = value;
    }

    void Decompress::setScale(unsigned int num, unsigned int denom)
    {
        cinfo_.scale_num = num;
        cinfo_.scale_denom = denom;
    }

    void Decompress::start()
    {
        jpeg_start_decompress(&cinfo_);
    }

    void Decompress::setCrop(unsigned int left, unsigned int top,
                             unsigned int width, unsigned int height)
    {
        shouldCrop = true;
        first_scanline = top;
        last_scanline = top + height;
        x_offset = left;
        x_width = width;

        jpeg_crop_scanline(&cinfo_, &x_offset, &x_width);
    }

    unsigned long int Decompress::getOutputBufferSize()
    {
        int row_stride = cinfo_.output_width * cinfo_.out_color_components;
        return row_stride * outputHeight();
    }

    void Decompress::read(unsigned char *outbuffer, unsigned long int outsize)
    {
        int row_stride = cinfo_.output_width * cinfo_.out_color_components;
        int required_outsize = row_stride * outputHeight();

        if (required_outsize > outsize) {
            return;
        }

        if (shouldCrop) {
            jpeg_skip_scanlines(&cinfo_, first_scanline);
        }
        else
        {
            last_scanline = cinfo_.output_height;
        }

        std::vector<JSAMPROW> row_pointer;
        row_pointer.resize(cinfo_.rec_outbuf_height);

        while (cinfo_.output_scanline < last_scanline) {
            for (int i = 0; i < cinfo_.rec_outbuf_height; i++) {
                row_pointer[i] = outbuffer + row_stride * (i + cinfo_.output_scanline - first_scanline);
            }
            jpeg_read_scanlines(&cinfo_, row_pointer.data(), cinfo_.rec_outbuf_height);
        }

        if (shouldCrop) {
            jpeg_skip_scanlines(&cinfo_, cinfo_.output_height - cinfo_.output_scanline);
        }
    }

    void Decompress::stop()
    {
        jpeg_finish_decompress(&cinfo_);
        shouldCrop = false;
        first_scanline = 0;
        last_scanline = 0;
        x_offset = 0;
        x_width = 0;
    }

    JDIMENSION Decompress::inputWidth() const
    {
        return cinfo_.image_width;
    }

    JDIMENSION Decompress::inputHeight() const
    {
        return cinfo_.image_height;
    }

    JDIMENSION Decompress::outputLeft() const
    {
        if (shouldCrop) {
            return x_offset;
        } else {
            return 0;
        }
    }

    JDIMENSION Decompress::outputTop() const
    {
        if (shouldCrop) {
            return first_scanline;
        } else {
            return 0;
        }
    }

    JDIMENSION Decompress::outputWidth() const
    {
        if (shouldCrop) {
            return x_width;
        } else {
            return cinfo_.output_width;
        }
    }

    JDIMENSION Decompress::outputHeight() const
    {
        if (shouldCrop) {
            return last_scanline - first_scanline;
        } else {
            return cinfo_.output_height;
        }
    }

    int Decompress::outputColorComponents() const
    {
        return cinfo_.out_color_components;
    }


} // namespace JPEGWrapper

std::string j_color_space_to_str(J_COLOR_SPACE value)
{
    switch (value)
    {
        case JCS_UNKNOWN: return "JCS_UNKNOWN";
        case JCS_GRAYSCALE: return "JCS_GRAYSCALE";
        case JCS_RGB: return "JCS_RGB";
        case JCS_YCbCr: return "JCS_YCbCr";
        case JCS_CMYK: return "JCS_CMYK";
        case JCS_YCCK: return "JCS_YCCK";
        case JCS_EXT_RGB: return "JCS_EXT_RGB";
        case JCS_EXT_RGBX: return "JCS_EXT_RGBX";
        case JCS_EXT_BGR: return "JCS_EXT_BGR";
        case JCS_EXT_BGRX: return "JCS_EXT_BGRX";
        case JCS_EXT_XBGR: return "JCS_EXT_XBGR";
        case JCS_EXT_XRGB: return "JCS_EXT_XRGB";
        case JCS_EXT_RGBA: return "JCS_EXT_RGBA";
        case JCS_EXT_BGRA: return "JCS_EXT_BGRA";
        case JCS_EXT_ABGR: return "JCS_EXT_ABGR";
        case JCS_EXT_ARGB: return "JCS_EXT_ARGB";
        case JCS_RGB565: return "JCS_RGB565";
    }
    return "(unknown)";
}
std::string j_dct_method_to_str(J_DCT_METHOD value)
{
    switch (value)
    {
        case JDCT_ISLOW: return "JDCT_ISLOW";
        case JDCT_IFAST: return "JDCT_IFAST";
        case JDCT_FLOAT: return "JDCT_FLOAT";
    }
    return "(unknown)";
}

std::string j_dither_mode_to_str(J_DITHER_MODE value)
{
    switch (value)
    {
        case JDITHER_NONE: return "JDITHER_NONE";
        case JDITHER_ORDERED: return "JDITHER_ORDERED";
        case JDITHER_FS: return "JDITHER_FS";
    }
    return "(unknown)";
}

std::ostream& operator<< (std::ostream& out, const struct jpeg_decompress_struct& cinfo)
{
    out << "Basic image description:" << std::endl
        << "\tImage Width: " << cinfo.image_width << std::endl
        << "\tImage Height: " << cinfo.image_height << std::endl
        << "\tNum Components: " << cinfo.num_components << std::endl
        << "Decompression parameters:" << std::endl
        << "\tColor Space: " << j_color_space_to_str(cinfo.jpeg_color_space) << std::endl
        << "\tOutput Color Space: " << j_color_space_to_str(cinfo.out_color_space) << std::endl
        << "\tOutput Scale: " << cinfo.scale_num << "/" << cinfo.scale_denom << std::endl
        << "\tOutput Gamma: " << cinfo.output_gamma << std::endl
        << "\tBuffered Image: " << cinfo.buffered_image << std::endl
        << "\tRaw Data Out: " << cinfo.raw_data_out << std::endl
        << "\tDCT Method: " << j_dct_method_to_str(cinfo.dct_method) << std::endl
        << "\tDo Fancy Upsampling: " << cinfo.do_fancy_upsampling << std::endl
        << "\tDo Block Smoothing: " << cinfo.do_block_smoothing << std::endl
        << "\tQuantize Colors: " << cinfo.quantize_colors << std::endl
        << "\tDither Mode: " << j_dither_mode_to_str(cinfo.dither_mode) << std::endl
        << "\tTwo Pass Quantize: " << cinfo.two_pass_quantize << std::endl
        << "\tDesired Number Of Colors: " << cinfo.desired_number_of_colors << std::endl
        << "\tEnable 1Pass Quant: " << cinfo.enable_1pass_quant << std::endl
        << "\tEnable External Quant: " << cinfo.enable_external_quant << std::endl
        << "\tEnable 2Pass Quant: " << cinfo.enable_2pass_quant << std::endl
        << "Output image description:" << std::endl
        << "\tOutput Width: " << cinfo.output_width << std::endl
        << "\tOutput Height: " << cinfo.output_height << std::endl
        << "\tOut Color Components: " << cinfo.out_color_components << std::endl
        << "\tOutput Components: " << cinfo.output_components << std::endl
        << "\tRec Outbuf Height: " << cinfo.rec_outbuf_height << std::endl
        << "Colormap description:" << std::endl
        << "\tActual Number Of Colors: " << cinfo.actual_number_of_colors << std::endl;
    return out;
}

