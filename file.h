#ifndef FILEOPERATOR_H
#define FILEOPERATOR_H

#include <fstream>
#include <opencv2/highgui.hpp>
#include "type_check.h"
#define CHECK_STATUS_EXIT(stat, message) CHECK_STATUS( stat, exit, message )


#define WRITE_LIST(stream, object) do{\
    size_t size = object.size(); \
    file_op::File::WriteItem(stream, size); \
    for ( size_t i = 0; i<size; ++i) \
    {\
        object.at(i).Write(stream); \
    }       \
}while(0)

#define READ_LIST(stream, object) do{\
    size_t size = 0; \
    file_op::File::ReadItem(stream, size); \
    object.resize(size);                 \
    for ( size_t i = 0; i<size; ++i) \
    {\
        object.at(i).Read(stream); \
    }       \
}while(0)

namespace file_op
{
enum MediaType
{
    IMAGE = 1, VIDEO
};
enum FileType
{
    FILE = 1, DIRECTORY, ALL
};
const std::string IMAGE_SUFFIX = "png bmp tiff tif jpg jpeg";
const std::string VIDEO_SUFFIX = "mp4 avi wav wma ts flv mkv wmv asf f4v m1v m4v mov mpeg mpg rm vob";
enum FileStatus
{
    NOT_EXIST, IS_FILE, IS_DIR
};

using StringList = std::vector<std::string>;
using IntList = std::vector<int>;

class File
{
public:
    File();

    ~File();

public:
    /**
    * @brief load files in dir recurrsive
    * @param type    IAMGE，VIDEO
    *
    * @return fileNums file number for every dir
    * @author   sunhao
    */
    static bool Walk(const std::string &path
                     , const FileType type, StringList &fileName);


    static bool Walk(const std::string &path
                     , const MediaType type
                     , IntList &fileNums, StringList &fileName);

    static bool
    WalkImage(const std::string &filePath, IntList &fileNums, StringList &fileName);

    static bool
    WalkVideo(const std::string &filePath, IntList &fileNums, StringList &fileName);

    static bool Exist(const std::string file);

    static bool MkdirFromFile(const std::string file);

    static bool Remove(const std::string &file);

    static bool RemoveDir(const std::string &path);

    static bool Write(std::ofstream &stream, const cv::Mat &img);

    static bool Read(std::ifstream &stream, cv::Mat &img);

    static int CreatDir(const std::string path);

    static std::string Dir(const std::string path);

    static std::string Basename(const std::string path);

    template<class T>
    static void WriteItem(std::ofstream &stream, const T &data);


    template<class T>
    static void WriteList(std::ofstream &stream, const std::vector<T> &data);

    template<class T>
    static void WriteList(std::ofstream &stream, const std::vector<std::vector<T>> &data);

    template<class T>
    static void ReadItem(std::ifstream &stream, T &data);

    template<class T>
    static void ReadList(std::ifstream &stream, std::vector<T> &data);

    template<class T>
    static void ReadList(std::ifstream &stream, std::vector<std::vector<T>> &data);

    static bool Read(const char *filename, unsigned char **data, int &size);

private:
    static FileStatus CheckType(const std::string file);

private:
    std::string file;
};


template<class T>
void file_op::File::WriteItem(std::ofstream &stream, const T &data)
{
    stream.write(reinterpret_cast<const char *>(&data), sizeof(data));
}

template<class T>
void file_op::File::WriteList(std::ofstream &stream, const std::vector<T> &data)
{
    size_t size = data.size();
    WriteItem(stream, size);

    for (size_t i = 0; i < size; ++i)
    {
        T item = data.at(i);
        WriteItem(stream, item);
    }
}

template<class T>
void file_op::File::WriteList(std::ofstream &stream, const std::vector<std::vector<T>> &data)
{
    size_t size = data.size();
    WriteItem(stream, size);

    for (size_t i = 0; i < size; ++i)
    {
        const auto& item = data.at(i);
        WriteList(stream, item);
    }
}

template<class T>
void file_op::File::ReadItem(std::ifstream &stream, T &data)
{
    stream.read(reinterpret_cast<char *>(&data), sizeof(data));
}

template<class T>
void file_op::File::ReadList(std::ifstream &stream, std::vector<T> &data)
{
    size_t size = data.size();
    ReadItem(stream, size);

    data.resize(size);
    for (size_t i = 0; i < size; ++i)
    {
        ReadItem(stream, data.at(i));
    }
}

// TODO : 2022-10-20 14:48:27 [hao]  support high dims
template<class T>
void file_op::File::ReadList(std::ifstream &stream, std::vector<std::vector<T>> &data)
{
    size_t size = data.size();
    ReadItem(stream, size);

    data.resize(size, std::vector<T>{});
    for (size_t i = 0; i < size; ++i)
    {
        ReadList(stream, data.at(i));
    }
}

} // namespace file_op
#endif // FILEOPERATOR_H
