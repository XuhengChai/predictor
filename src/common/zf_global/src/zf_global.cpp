#include <cstdio>
#include <iostream>

// #define CHECK(x)
//   if (!(x)) printMsg(std::cout, ) << "Check failed: " << #x
// #define CHECK_EQ(x, y) CHECK((x) == (y))

// #define CHECK_EQ(x, y)
//   if ((x) != (y)) std::cerr << "Check failed: "

// int main(int argc, char **argv)
// {
//   (void)argc;
//   (void)argv;

//   printf("hello world zf_global package\n");
//   CHECK_EQ(2, 1) << "not equal. "<< std::endl;;
//   std::cout <<"Got it. "<< std::endl;
//   CHECK_EQ(3, 2) << "not equal too. "<< std::endl;;
//   return 0;
// }

#include <streambuf>
#include "zf_global/common/zf_global_trailer_param.h"

using namespace NS_ZF;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world zf_global package\n");
  Eigen::MatrixXd rect(3, 4);
  rect.row(0) << 1.27, -11.73, -11.73, 1.27;
  rect.row(1) << 1.339, -1.339, 1.337, 1.337;
  rect.row(2) << 1, 1, 1, 1;
  LOG_DEBUG() << rect;

  auto ttt = GlobalParams::Instance().GetLinSpacePoints(rect, 5);
  LOG_INFO() << ttt;

  return 0;
}

// use
//  logger log;
//  log << "Hello";
//  or   logger() << "Hello, world!";

// class logger {
// private:
// 	class append_endl {
// 	public:
// 		~append_endl() {
// 			std::cout << std::endl;
// 		}
// 		template <typename T>
// 		append_endl& operator<<(T const &value) {
// 			std::cout << value;
// 			return *this;
// 		}
// 	};
// public:
// 	template <typename T>
// 	append_endl operator<<(T const &value) {
//     	std::cout << value;
//     	return append_endl();
// 	}
// };

// class newline_writer
//     : public std::ostream
// {
//   bool need_newline = true;

// public:
//   newline_writer(std::streambuf *sbuf)
//       : std::ios(sbuf), std::ostream(sbuf)
//   {
//   }
//   newline_writer(newline_writer &&other)
//       : newline_writer(other.rdbuf())
//   {
//     other.need_newline = false;
//   }
//   ~newline_writer() { this->need_newline &&*this << '\n'; }
// };

// newline_writer writeline()
// {
//   return newline_writer(std::cout.rdbuf());
// }

// #define PRINT_MSG   []() -> newline_writer  {                                \
//   return newline_writer(std::cout.rdbuf()); \
//   }

// int main(int argc, char** argv)
// {
//   // Test the macro
//   // PRINT_MSG() << "Hello, world!";
//   // PRINT_MSG() << "This is a test.";
//   std::cout << "This message does not use the macro.\n";
//   return 0;
// }

// #include "zf_global/util/logger.h"

// #include <unistd.h>
// #include <stdint.h>
// #include <time.h>
// #include <sys/time.h>

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
