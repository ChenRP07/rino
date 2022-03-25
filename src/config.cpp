#include "config.h"

float MAX_CORRESPONDENCE_DISTANCE = 30.0f;
float TRANSFORMATION_DIFFERENCE_THRESHOLD = 1e-6;
int MAX_ICP_ITERATION = 100;
float MSE_DIFFERENCE_THRESHOLD = 0.01f;

float MIN_MERGE_HAMMING_DISTANCE = 4.0f;

int CONSTANT_CLUSTER_NUMBER = 50;

int PARALLEL_DECODER_THREAD = 20;
int JPEG_COMPRESSION_QUALITY = 70;

int CompressString(const std::string& src, std::string& dst, int compressionlevel)
{
    size_t const cBuffSize = ZSTD_compressBound(src.size());
    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.c_str());
    size_t const cSize = ZSTD_compress(dstp, cBuffSize, srcp, src.size(), compressionlevel);
    auto code = ZSTD_isError(cSize);
    if (code)
    {
        return (int)code;
    }
    dst.resize(cSize);
    return (int)code;
}

int DecompressString(const std::string& src, std::string& dst)
{
    size_t const cBuffSize = ZSTD_getFrameContentSize(src.c_str(), src.size());

    if (0 == cBuffSize)
    {
        return (int)cBuffSize;
    }

    if (ZSTD_CONTENTSIZE_UNKNOWN == cBuffSize)
    {
        return DecompressString(src, dst);
    }

    if (ZSTD_CONTENTSIZE_ERROR == cBuffSize)
    {
        return -2;
    }

    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.c_str());
    size_t const cSize = ZSTD_decompress(dstp, cBuffSize, srcp, src.size());
    auto code = ZSTD_isError(cSize);
    if (code)
    {
        return (int)code;
    }
    dst.resize(cSize);
    return (int)code;
}


void jpeg_encoder(const std::string& filename, std::vector<uint8_t>& colors, int quality)
{
    /* get jpeg image size from data */
    int point_num = (int)colors.size() / 3;

    /* make the image as square as possible */
    int image_height = (int)std::sqrt(point_num);
    int image_width = point_num / image_height;

    /* if points are not perfect square, add another row */
    if (point_num != image_height * image_width)
        image_width += 1;

    /* data to be compressed by jpeg */
    unsigned char* image_buffer;

    /* jpeg compress object : cinfo, jpeg error object jerr */
    struct jpeg_compress_struct cinfo{};
    struct jpeg_error_mgr jerr{};

    /* file to output */
    FILE* outfile;

    /* image row pointer */
    JSAMPROW row_pointer[1];

    /* add an error handler then create a compressor */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* specify output file */
    outfile = fopen(filename.c_str(), "wb");
    jpeg_stdio_dest(&cinfo, outfile);

    /* set attributions, width, height and color(RGB) */
    cinfo.image_width = image_width;
    cinfo.image_height = image_height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    /* determine properties and compress quality */
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    /* let's start jpeg encode */
    jpeg_start_compress(&cinfo, TRUE);

    /* change points' color rgb to jpeg color bgr */
    image_buffer = new unsigned char[image_height * image_width * 3];

    int index = 0;
    for (int i = 0; i < point_num; i++)
    {
        image_buffer[index++] = colors[i * 3 + 2]; //b->r
        image_buffer[index++] = colors[i * 3 + 1]; //g->g
        image_buffer[index++] = colors[i * 3];     //r->b
    }

    /* fill by 0x000000 black */
    for (; index < image_height * image_width * 3; index++)
        image_buffer[index] = 0x00;

    /* output to file */
    size_t line = 0;
    int row_stride = image_width * 3;
    while (line < cinfo.image_height)
    {
        row_pointer[0] = &image_buffer[line * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
        line++;
    }

    /* now finish this compress and release all resouces */
    delete image_buffer;
    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    jpeg_destroy_compress(&cinfo);
}

void jpeg_decoder(const std::string& filename, std::vector<unsigned char>& colors)
{
    /* make sure colors is empty */
    colors.clear();

    /* create a jpeg decompress object cinfo and jpeg error object jerr */
    struct jpeg_decompress_struct cinfo{};
    struct jpeg_error_mgr jerr{};

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    /* image to input */
    FILE* infile;
    infile = fopen(filename.c_str(), "rb");

    /* color buffer */
    JSAMPARRAY buffer;
    int row_stride;

    /* read header from image */
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, TRUE);

    cinfo.scale_num = 1;
    cinfo.scale_denom = 1;

    /* let's start */
    jpeg_start_decompress(&cinfo);

    /* how many data does one row contain */
    row_stride = (int)cinfo.output_width * cinfo.output_components;

    /* allocate space for buffer */
    buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, row_stride, 1);

    /* decode */
    while (cinfo.output_scanline < cinfo.output_height)
    {
        jpeg_read_scanlines(&cinfo, buffer, 1);

        /* p is a pointer that denotes the data */
        unsigned char *p = buffer[0];

        unsigned char color_r, color_g, color_b;
        /* add colors to vector */
        for (size_t i = 0; i < cinfo.output_width; i++)
        {
            /* change bgr to rgb */
            color_r = *p++;
            color_g = *p++;
            color_b = *p++;

            colors.push_back(color_b);
            colors.push_back(color_g);
            colors.push_back(color_r);
        }
    }

    /* finish the decompressor and release all resouces */
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
}