#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <iostream>

using namespace std;
using namespace boost;

class HelloWorld
{
public:
	void hello(const int &n);
	void entry();
};

void HelloWorld::hello(const int &n)
{
	cout << n << endl;
	std::cout << "Hello world, I'm a thread!" << std::endl;
}


void HelloWorld::entry()
{
	boost::thread thrd(boost::bind(&HelloWorld::hello, this, 5));
	thrd.join();
}

int main(int argc, char* argv[])
{
	HelloWorld hw;
	hw.entry();
	return 0;
}



/**
 * cmake_minimum_required(VERSION 3.13)
project(1)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(Boost REQUIRED COMPONENTS
        thread
        filesystem
        )
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

include_directories(${Boost_INCLUDE_DIRS})
message("${Boost_INCLUDE_DIRS}")
message("${Boost_LIBRARIES}")

add_executable( 1 main.cpp )
target_link_libraries(1 ${Boost_LIBRARIES})
*/