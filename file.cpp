#include "file.h"
#include <unistd.h>
// for make dir
#ifdef _WIN32
//define something for Windows (32-bit and 64-bit, this part is common)
#ifdef _WIN64
//define something for Windows (64-bit only)
#else
//define something for Windows (32-bit only)
#include <direct.h>
#include <io.h>
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#endif
#elif __linux__

#include <stdarg.h>
#include <sys/stat.h>

#define ACCESS access
#define MKDIR(a) mkdir((a),0755)
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_IPHONE_SIMULATOR
// iOS Simulator
#elif TARGET_OS_IPHONE
// iOS device
#elif TARGET_OS_MAC
// Other kinds of Mac OS
#else
#   error "Unknown Apple platform"
#endif
#elif __ANDROID__
// android
#elif __unix__ // all unices not caught above
// Unix
#elif defined(_POSIX_VERSION)
// POSIX
#else
#   error "Unknown compiler"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
//#include "src/utils/define.h"


const int MAX_FILE_LENGTH = 2048;

void ErrorQuit(const char *msg)
{
    perror(msg);
//    exit(-1);
}

void ChangePath(const char *path)
{
//    printf("Leave %s Successed . . .\n", getcwd(NULL, 0));
    if (chdir(path) == -1)
    {
        ErrorQuit("chdir");
    }
//    printf("Entry %s Successed . . .\n", getcwd(NULL, 0));
}

file_op::File::File()
{

}

file_op::File::~File()
{

}

bool file_op::File::Remove(const std::string &file)
{
    if (0 != remove(file.c_str()))
    {
        return false;
    }

    return true;
}

bool file_op::File::RemoveDir(const std::string &path)
{
    ::DIR *dir;
    struct dirent *dirp;
    struct stat buf;
    char *p = getcwd(NULL, 0);
    if ((dir = opendir(path.c_str())) == NULL)
    {
        return false;
        ErrorQuit("OpenDir");
    }
    ChangePath(path.c_str());
    while (dirp = readdir(dir))
    {
        if ((strcmp(dirp->d_name, ".") == 0) || (strcmp(dirp->d_name, "..") == 0))
            continue;
        if (stat(dirp->d_name, &buf) == -1)
        {
            ErrorQuit("stat");
            return false;
        }

        if (S_ISDIR(buf.st_mode))
        {
            RemoveDir(dirp->d_name);
            /*if(rmdir(dirp->d_name)==-1)
             *  error_quit("rmdir");
             * printf("rm %s Successed . . .\n",dirp->d_name);*/
            continue;
        }
        if (remove(dirp->d_name) == -1)
        {
            ErrorQuit("remove");
            return false;
        }
//        printf("rm %s Successed . . .\n", dirp->d_name);
    }
    closedir(dir);
    ChangePath(p);
    if (rmdir(path.c_str()) == -1)
    {
        ErrorQuit("rmdir");
        return false;
    }

//    printf("rm %s Successed . . .\n", path.c_str());

    return true;
}

bool file_op::File::Write(std::ofstream &stream, const cv::Mat &img)
{
    if (!stream.is_open())
    {
        return false;
    }
    if (img.empty())
    {
        int s = 0;
        stream.write(reinterpret_cast<const char *>(&s), sizeof(int));
        return true;
    }
    int type = img.type();
    stream.write(reinterpret_cast<const char *>(&img.rows), sizeof(int));
    stream.write(reinterpret_cast<const char *>(&img.cols), sizeof(int));
    stream.write(reinterpret_cast<const char *>(&type), sizeof(int));
    stream.write(reinterpret_cast<const char *>(img.data), img.elemSize() * img.total());

    return true;
}

bool file_op::File::Read(std::ifstream &stream, cv::Mat &img)
{
    if (!stream.is_open())
    {
        return false;
    }

    int rows, cols, type;
    stream.read(reinterpret_cast<char *>(&rows), sizeof(int));
    if (rows == 0)
    {
        return true;
    }
    stream.read(reinterpret_cast<char *>(&cols), sizeof(int));
    stream.read(reinterpret_cast<char *>(&type), sizeof(int));

    img.release();
    img.create(rows, cols, type);
    stream.read(reinterpret_cast<char *>(img.data), img.elemSize() * img.total());

    return true;
}

int file_op::File::CreatDir(const std::string path)
{
    size_t iLen = path.length();
    char pszDir[MAX_FILE_LENGTH];

    strcpy(pszDir,path.c_str());

    // add `/` in the tail
    if (pszDir[iLen - 1] != '\\' && pszDir[iLen - 1] != '/')
    {
        pszDir[iLen] = '/';
        pszDir[iLen + 1] = '\0';
    }

    // create dir
    for (size_t i = 1; i <= iLen; i++)
    {
        if (pszDir[i] == '\\' || pszDir[i] == '/')
        {
            pszDir[i] = '\0';
            int iRet = ACCESS(pszDir, 0);
            if (iRet != 0)
            {
                iRet = MKDIR(pszDir);
                if (iRet != 0)
                {
                    return -1;
                }
            }
            pszDir[i] = '/';
        }
    }

    return 0;
}

bool file_op::File::Walk(const std::string &path, const MediaType type
                         , IntList &fileNums, StringList &fileName)
{
    return true;
}

bool file_op::File::WalkImage(const std::string &filePath, IntList &fileNums
                              , StringList &fileName)
{
    return Walk(filePath, IMAGE, fileNums, fileName);
}

bool file_op::File::WalkVideo(const std::string &filePath, IntList &fileNums
                              , StringList &fileName)
{
    return Walk(filePath, VIDEO, fileNums, fileName);
}

bool file_op::File::Exist(const std::string file)
{
    return access(file.c_str(), 0) == 0;
}

bool file_op::File::MkdirFromFile(const std::string file)
{
    auto pos = file.rfind("/");
    if (std::string::npos != pos)
    {
        CreatDir(file.substr(0, pos));
    }
    return true;
}

file_op::FileStatus file_op::File::CheckType(const std::string file)
{
    if ((access(file.c_str(), 0)) != 0)
    {
        return NOT_EXIST;
    }

    struct stat buf;
    int result;

    result = stat(file.c_str(), &buf);
    if (S_IFDIR & buf.st_mode)
    {
        return IS_DIR;
    }
    else if (S_IFREG & buf.st_mode)
    {
        return IS_FILE;
    }
    else
    {
        // ERROR
    }
}

bool file_op::File::Walk(const std::string &path, const file_op::FileType type
                         , file_op::StringList &fileName)
{
    std::string dir = path;
    DIR *dp;
    struct dirent *dirp;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
//        LOG_CHECK_STATUS(INFO) << "Can't open " << dir;
//        std::cout << "Can't open " << dir << std::endl;
        return false;
    }
    while ((dirp = readdir(dp)) != NULL)
    {
        if ((type == DIRECTORY and (4 == dirp->d_type)) // S_IFDIR is 4
            or (type == FILE and (8 == dirp->d_type)) //  S_IFREG is 8
            or type == ALL)
        {
            if (strcmp(".", dirp->d_name) == 0 or strcmp("..", dirp->d_name) == 0) continue;
            fileName.push_back(dirp->d_name);
        }
    }
    closedir(dp);

    return true;
}

std::string file_op::File::Dir(const std::string path)
{
    auto id = path.rfind('/');

    if (id != std::string::npos)
    {
        return path.substr(0, id);
    }

    id = path.rfind('\\');

    if (id != std::string::npos)
    {
        return path.substr(0, id);
    }

    return "./";
}


std::string file_op::File::Basename(const std::string path)
{
    auto id = path.rfind('/');

    if (id != std::string::npos)
    {
        return path.substr(id+1, path.length() - id);
    }

    id = path.rfind('\\');

    if (id != std::string::npos)
    {
        return path.substr(id+1, path.length() - id);
    }

    return path;
}

bool file_op::File::Read(const char *filename, unsigned char **data, int &size)
{
    *data = nullptr;
    ::FILE *fp;
    const int offset = 0;
    int ret = 0;
    unsigned char *dataTemp;

    fp = fopen(filename, "rb");
//    CHECK_STATUS_EXIT(nullptr != fp, "Open file " + std::string(filename) + " failed.");

    fseek(fp, 0, SEEK_END);
    size = ftell(fp);

    ret = fseek(fp, offset, SEEK_SET);
//    CHECK_STATUS_EXIT(0 == ret, "blob seek failure");

    dataTemp = (unsigned char *) malloc(size);
//    CHECK_STATUS_EXIT(nullptr != dataTemp, "buffer malloc failure.\n");
    ret = fread(dataTemp, 1, size, fp);

    *data = dataTemp;
    fclose(fp);

    return true;

    exit:
    return false;
}
