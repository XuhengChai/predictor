/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief thread safe logger.
 * @run
 */

#ifndef ZF_THREAD_SAFE_LOG_H
#define ZF_THREAD_SAFE_LOG_H

#include <string>
#include <fstream>
#include <sstream>
#include <mutex>
#include <cassert>
#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <map>
#include <regex>
#include <sys/stat.h>
#include <dirent.h>
#include "zf_global/zf_global.h"
#include "zf_global/common/zf_global_macros.h"

// struct tm;
BEGIN_NS_ZF

namespace LOGGER
{
    static const char gk_loggerDefaultDir[] = "./Music/"; //
    static const char gk_loggerSyncFilePath[] = "/sync/";            //
    static const char gk_loggerCanPath[] = "/can/";                  //
    int MkDir(const char *dir)
    {
        DIR *mydir = NULL;
        mydir = opendir(dir);
        if (mydir == NULL) // 判断目录
        {
            int ret = mkdir(dir, 511); // 创建目录
            if (ret != 0)              // return zero on success.
            {
                return -1;
            }
        }
        return 0;
    }

    bool CreateDirectory(const std::string &dirPath)
    {
        int status = mkdir(dirPath.c_str(), 511); // S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH
        if (status == 0)                          // Directory created successfully
        {
            return true;
        }
        else if (errno == EEXIST) // Directory already exists
        {
            return true;
        }
        else if (errno == ENOENT) // Parent directory does not exist
        {
            size_t pos = dirPath.find_last_of('/');
            if (pos == std::string::npos) // Invalid directory path
            {
                return false;
            }
            std::string parentDir = dirPath.substr(0, pos);
            if (!CreateDirectory(parentDir)) // Failed to create parent directory
            {
                return false;
            }
            // Retry creating the directory
            return CreateDirectory(dirPath);
        }
        else // Failed to create directory
        {
            return false;
        }
    }

    std::string CreateFile(const std::string &cur_time_dir, const std::string &filename = "logtime.txt")
    {
        struct stat fileStat;
        if (stat(filename.c_str(), &fileStat) == 0)
        {
            const auto lastModifyTime = fileStat.st_mtime; // File exists, get its last modification time
            // Get the current time
            const auto currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            const auto timeDifference = std::difftime(currentTime, lastModifyTime); // Calculate the time difference in seconds
            // Check if the time difference is less than 30 seconds
            if (timeDifference < 30.0)
            {
                std::ifstream file(filename);
                if (!file.is_open())
                {
                    LOG_ERROR() << "Failed to open file for read. " << filename.c_str();
                    return "";
                }
                std::string storedTime;
                std::getline(file, storedTime);
                file.close();
                return storedTime; // Return true if the error is less than 30 seconds
            }
        }
        // File does not exist, time difference is greater than 30 seconds, create it and write the current time
        std::ofstream file(filename);
        if (!file.is_open())
        {
            LOG_ERROR() << "Failed to open file for read. " << filename.c_str();
            return "";
        }
        file << cur_time_dir;
        file.close();
        return cur_time_dir;
    }

    enum class Level
    {
        Debug,
        Info,
        Warning,
        Error,
        Fatal
    };
    // class FileLogger;
    // class ConsoleLogger;
    // class BaseLogger;

    class LoggerUtil
    {
    public:
        bool GetDir(std::string &dir)
        {
            if (!m_sDir.empty())
            {
                dir = m_sDir;
                return true;
            }
            dir = gk_loggerDefaultDir;
            return false;
        }

    private:
        const tm *GetLocalTime()
        {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now) + 28800; // 60 * 60 * 8
            gmtime_r(&in_time_t, &_localTime);                                  // localtime_r
            return &_localTime;
            // time_t currentTime = time(nullptr);
            // tm *_localTime = gmtime(&currentTime);
            // // gmtime_r(&currentTime, &_localTime);
            // return _localTime;
        }
        tm _localTime;
        std::string m_sDir = "";
        DECLARE_SINGLETON(LoggerUtil)
    };
    LoggerUtil::LoggerUtil()
    {
        std::string tDir = gk_loggerDefaultDir;
        tDir = tDir + "log/";
        MkDir(tDir.c_str());
        std::string logTimeFile = tDir + "logtime.txt";

        auto timeInfo = GetLocalTime();
        std::stringstream ss;
        ss << timeInfo->tm_year + 1900 << "-"
           << std::setw(2) << std::setfill('0') << timeInfo->tm_mon + 1;
        std::string directory_month = tDir + ss.str() + "/";
        ss << "-"
           << std::setw(2) << std::setfill('0') << timeInfo->tm_mday << " "
           << std::setw(2) << std::setfill('0') << timeInfo->tm_hour << "_"
           << std::setw(2) << std::setfill('0') << timeInfo->tm_min << "_"
           << std::setw(2) << std::setfill('0') << timeInfo->tm_sec;
        std::string directory_sec = directory_month + ss.str();
        m_sDir = CreateFile(directory_sec, logTimeFile);
        if (m_sDir.empty())
        {
            LOG_ERROR() << "Can not open log. Please check!";
            m_sDir = tDir;
        }
        LOG_DEBUG() << "m_sDir is" << m_sDir;
        CreateDirectory((m_sDir + gk_loggerSyncFilePath).c_str());
        CreateDirectory((m_sDir + gk_loggerCanPath).c_str());
    }

    class BaseLogger
    {
        class LogStream;

    public:
        BaseLogger() = default;
        virtual ~BaseLogger() = default;

        virtual LogStream operator()(Level nLevel = Level::Debug);

    protected:
        const tm *getLocalTime();

    private:
        void endline(Level nLevel, std::string &&oMessage);
        // virtual void output(const tm *p_tm,
        //                     const char *str_level,
        //                     const char *str_message) = 0;
        virtual void output(const char *str_message,
                            const tm *p_tm,
                            const char *str_level) = 0;

    private:
        std::mutex _lock;
        tm _localTime;
    };

    class BaseLogger::LogStream : public std::ostringstream
    {
        BaseLogger &m_oLogger;
        Level m_nLevel;

    public:
        LogStream(BaseLogger &oLogger, Level nLevel)
            : m_oLogger(oLogger), m_nLevel(nLevel){};
        LogStream(const LogStream &ls)
            : m_oLogger(ls.m_oLogger), m_nLevel(ls.m_nLevel){};
        ~LogStream()
        {
            m_oLogger.endline(m_nLevel, std::move(str()));
        }
    };

    class ConsoleLogger : public BaseLogger
    {
        using BaseLogger::BaseLogger;
        virtual void output(const char *str_message,
                            const tm *p_tm,
                            const char *str_level);
    };

    class FileLogger : public BaseLogger
    {
    public:
        FileLogger(std::string filename, const char *dir = gk_loggerDefaultDir) noexcept;
        FileLogger() noexcept;
        std::string CreateFolder(const std::string &dir = gk_loggerCanPath);
        void SetFileName(std::string filename, const char *dir = gk_loggerCanPath);
        FileLogger(const FileLogger &) = delete;
        FileLogger(FileLogger &&) = delete;
        virtual ~FileLogger();

    private:
        virtual void output(const char *str_message,
                            const tm *p_tm,
                            const char *str_level);
        bool InitSaveDir();

    private:
        std::ofstream _file;
        bool m_hasName;
        std::string m_sDir;
    };

    // extern ConsoleLogger debug;
    // extern FileLogger record;

    ConsoleLogger debug;
    // FileLogger LOG_FILE("build_at_" __DATE__ "_" __TIME__ "file.log");

#ifdef WIN32
#define localtime_r(_Time, _Tm) localtime_s(_Tm, _Time)
#endif

    static const std::map<Level, const char *> LevelStr =
        {
            {Level::Debug, "Debug"},
            {Level::Info, "Info"},
            {Level::Warning, "Warning"},
            {Level::Error, "Error"},
            {Level::Fatal, "Fatal"},
    };

    std::ostream &operator<<(std::ostream &stream, const tm *tm)
    {
        return stream << 1900 + tm->tm_year << '-'
                      << std::setfill('0') << std::setw(2) << tm->tm_mon + 1 << '-'
                      << std::setfill('0') << std::setw(2) << tm->tm_mday << ' '
                      << std::setfill('0') << std::setw(2) << tm->tm_hour << '_'
                      << std::setfill('0') << std::setw(2) << tm->tm_min << '_'
                      << std::setfill('0') << std::setw(2) << tm->tm_sec;
    }

    BaseLogger::LogStream BaseLogger::operator()(Level nLevel)
    {
        return LogStream(*this, nLevel);
    }

    const tm *BaseLogger::getLocalTime()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now) + 28800; // 60 * 60 * 8
        gmtime_r(&in_time_t, &_localTime);                                  // localtime_r
        return &_localTime;
        // time_t currentTime = time(nullptr);
        // tm *_localTime = gmtime(&currentTime);
        // // gmtime_r(&currentTime, &_localTime);
        // return _localTime;
    }

    void BaseLogger::endline(Level nLevel, std::string &&oMessage)
    {
        // _lock.lock();
        // std::lock_guard<std::mutex> lock(_lock);
        // output(getLocalTime(), LevelStr.find(nLevel)->second, oMessage.c_str());
        output(oMessage.c_str(), &_localTime, LevelStr.find(nLevel)->second);
        // _lock.unlock();
    }

    void ConsoleLogger::output(const char *str_message,
                               const tm *p_tm,
                               const char *str_level)
    {
        std::cout << '[' << p_tm << ']'
                  << '[' << str_level << "]"
                  << "\t" << str_message << std::endl;
        std::cout.flush();
    }

    FileLogger::FileLogger(std::string filename, const char *dir) noexcept
        : BaseLogger()
    {
        // std::string valid_filename(filename.size(), '\0');
        // std::regex express("/|:| |>|<|\"|\\*|\\?|\\|");
        // std::regex_replace(valid_filename.begin(),
        //                    filename.begin(),
        //                    filename.end(),
        //                    express,
        //                    "_");
        _file.open(dir + filename,
                   std::fstream::out | std::fstream::app | std::fstream::ate);
        //    std::fstream::out | std::fstream::app | std::fstream::ate | std::fstream::binary);
        assert(!_file.fail());
        m_hasName = true;
    }

    inline FileLogger::FileLogger() noexcept
    {
        InitSaveDir();
        m_hasName = false;
    }

    bool FileLogger::InitSaveDir()
    {
        LoggerUtil::Instance().GetDir(m_sDir);
    }

    inline std::string FileLogger::CreateFolder(const std::string &dir)
    {
        auto curDir = m_sDir + dir;
        MkDir(curDir.c_str());
        return curDir;
    }

    inline void FileLogger::SetFileName(std::string filename, const char *dir)
    {
        if (m_hasName)
            return;
        auto name = m_sDir + dir + filename;
        _file.open(name,
                   std::fstream::out | std::fstream::app | std::fstream::ate);
        //    std::fstream::out | std::fstream::app | std::fstream::ate | std::fstream::binary);
        assert(!_file.fail());
        m_hasName = true;
    }

    FileLogger::~FileLogger()
    {
        _file.flush();
        _file.close();
    }

    void FileLogger::output(const char *str_message,
                            const tm *p_tm,
                            const char *str_level)
    {
        // _file << '[' << p_tm << ']'
        //       << '[' << str_level << "]"
        //       << "\t" << str_message << std::endl;
        _file << str_message << std::endl;
        _file.flush();
    }

} // namespace logger

END_NS_ZF

#endif // !ZF_THREAD_SAFE_LOG_H

// -----------------test code----------------- // time use 407452ms, 10.7G. 24.57w/s = 26.92 MB/s   Sync log: 34.18w/s

// int64_t get_current_millis(void)
// {
//   struct timeval tv;
//   gettimeofday(&tv, NULL);
//   return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
// }

// int main(int argc, char **argv)
// {
//   NS_ZF::LOGGER::FileLogger LOG_FILE_PARSER;

//   LOG_FILE_PARSER.SetFileName("test_time");

//   uint64_t start_ts = get_current_millis();
//   for (int i = 0; i < 1e8; ++i)
//   {
//     LOG_FILE_PARSER() << "my number is number my number is my number is my number is my number is my number is my number is "<< i;
//   }
//   // time use 407452ms, 10.7G. 24.57w/s   Sync log: 34.18w/s

//   uint64_t end_ts = get_current_millis();
//   printf("time use %lums\n", end_ts - start_ts);
// }
