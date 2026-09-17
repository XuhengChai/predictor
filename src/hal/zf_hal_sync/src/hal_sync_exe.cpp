#include <cstdio>
#include <thread>
#include <chrono>
#include <random>

// #include "hal_sync_global.h"
#include "zf_hal_sync/sync_policies/time_sync.h"
#include "zf_global/util/logger.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

NS_ZF::LOGGER::FileLogger LOG_FILE_SYNC("timeSync.log");

struct RadarData
{
	float time;
	int data;
};

struct ImageData
{
	float time;
	std::string data;
};

struct LidarData
{
	float time;
	float data;
};

static const int per1 = 60; // ms
static const int per2 = 36;
static const int per3 = 25;
static const int per4 = 35;

class HalDataSyncNode
{
public:
	// using RadarDataMap = std::unordered_map<uint32_t, PackedRadarData>; // status can id 60a, 61a
	// using TimeSyncType = TimeSynchronizer<RadarData, ImageData, RadarData, LidarData>; // status can id 60a, 61a
	using TimeSyncType = TimeSynchronizer<RadarData, ImageData>; // status can id 60a, 61a
	explicit HalDataSyncNode()
	{
		m_timeSync = std::make_shared<TimeSyncType>(20);
		m_timeSync->RegisterCallback(&HalDataSyncNode::PubSyncData, this);
		RadarData a;
		a.time = 1.5f;
		a.data = 1;
		ImageData b;
		b.data = "tets";
		b.time = 2.0f;
		m_timeSync->InvokeCallback(a, b);
		m_timeA = 0;
		m_timeB = 0;
		m_timeC = 0;
		m_timeD = 0;
		m_fTimeRadarLast = 0;
	};
	~HalDataSyncNode(){};

public:
	void PackedRadarDataCallback(const RadarData msg)
	{
		m_timeSync->PushMsg(msg, msg.time, 0); // TODO
	}
	void ImageDataCallback(const ImageData msg)
	{
		m_timeSync->PushMsg(msg, msg.time, 1);
	}
	void PackedRadarDataCallback2(const RadarData msg)
	{
		m_timeSync->PushMsg(msg, msg.time, 2); // TODO
	}
	void LidarDataCallback(const LidarData msg)
	{
		m_timeSync->PushMsg(msg, msg.time, 3); // TODO
	}
	void SetTimePeriod(const std::vector<double> &period)
	{
		m_timeSync->SetDataPeriod(period);
	}

	// void PubSyncData(const RadarData& radar, const ImageData& img, const RadarData& radar2, const LidarData& lidar)
	void PubSyncData(const RadarData &radar, const ImageData &img)
	{
		// std::cout << "time radar: " << &radar<< std::endl;
		// std::cout << "time radar2: " << &img <<std::endl;
		// std::cout << radar.time - m_fTimeRadarLast << " time radar: "
		//	<< std::endl << std::endl;
		if ((radar.time - m_fTimeRadarLast) > (per1 * 1e-3 * 2))
		{
			std::cout << "------------------: ";
		}
		// std::cout << radar.time - m_fTimeRadarLast
		//	<< std::endl << std::endl;
		std::cout << radar.time - m_fTimeRadarLast << "time radar: "
				  << "[" << radar.time << ": " << radar.data << "], "
				  << "[" << img.time << ": " << img.data.c_str() << "], "
				  //<< "[" << radar2.time << ": " << radar2.data << "], "
				  //<< "[" << lidar.time << ": " << lidar.data << "], "
				  << std::endl
				  << std::endl;
		// LOG_FILE_SYNC() << radar.time - m_fTimeRadarLast
		// 				<< "," << radar.time
		// 				<< "," << img.time
		// 				// << "," << radar2.time
		// 				// << "," << lidar.time
		// 				<< std::endl;
		m_fTimeRadarLast = radar.time;
	}

	void functionA()
	{
		// std::random_device rd;
		// std::mt19937 gen(rd());
		// std::uniform_int_distribution<int> dis(1, 100);
		auto startTime = std::chrono::steady_clock::now();
		std::random_device rd;
		std::mt19937 gen(rd());
		std::normal_distribution<double> distribution(per1, per1 * 0.1);
		while (true)
		{
			int randomValue = distribution(gen);
			auto currentTime = std::chrono::steady_clock::now();
			std::chrono::duration<float> elapsedTime = currentTime - startTime;
			m_timeA = elapsedTime.count();
			// int randomInt = dis(gen);
			RadarData td;
			td.data = 1;
			// td.data = dis(gen);
			td.time = m_timeA;
			// std::cout << "Function A called with time: " << randomValue << std::endl;
			PackedRadarDataCallback(td);
			std::this_thread::sleep_for(std::chrono::milliseconds(randomValue));
			// std::this_thread::sleep_for(std::chrono::milliseconds(per1));
			// std::this_thread::sleep_for(std::chrono::seconds(0.1));
			// m_timeA += (per1 * 1e-3);
		}
	}
	void functionC()
	{
		auto startTime = std::chrono::steady_clock::now();
		while (true)
		{
			auto currentTime = std::chrono::steady_clock::now();
			std::chrono::duration<float> elapsedTime = currentTime - startTime;
			m_timeC = elapsedTime.count();
			// int randomInt = dis(gen);
			RadarData td;
			td.data = 2;
			// td.data = dis(gen);
			td.time = m_timeC;
			// std::cout << "Function C called with time: " << m_timeC << std::endl;
			PackedRadarDataCallback2(td);
			std::this_thread::sleep_for(std::chrono::milliseconds(per3));
			// std::this_thread::sleep_for(std::chrono::seconds(0.1));
			// m_timeA += (per1 * 1e-3);
		}
	}

	void functionB()
	{
		// std::random_device rd;
		// std::mt19937 gen(rd());
		// std::uniform_int_distribution<int> dis('A', 'Z'); // Use desired character range

		// std::string randomString;
		// randomString.reserve(10);

		// for (int i = 0; i < 10; ++i)
		//{
		//   randomString += static_cast<char>(dis(gen));
		// }
		auto startTime = std::chrono::steady_clock::now();
		std::random_device rd;
		std::mt19937 gen(rd());
		std::normal_distribution<double> distribution(per2, per2 * 0.1);
		while (true)
		{
			auto currentTime = std::chrono::steady_clock::now();
			std::chrono::duration<float> elapsedTime = currentTime - startTime;
			m_timeB = elapsedTime.count();
			ImageData td;
			td.data = "random";
			// td.data = randomString;
			td.time = m_timeB;
			ImageDataCallback(td);
			// std::cout << "Function B called with time: " << m_timeB << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(int(distribution(gen))));
			// m_timeB += (per2 * 1e-3);
		}
	}
	void functionD()
	{
		auto startTime = std::chrono::steady_clock::now();
		while (true)
		{
			auto currentTime = std::chrono::steady_clock::now();
			std::chrono::duration<float> elapsedTime = currentTime - startTime;
			m_timeD = elapsedTime.count();
			LidarData td;
			td.data = 3.55;
			td.time = m_timeD;
			LidarDataCallback(td);
			std::this_thread::sleep_for(std::chrono::milliseconds(per4));
		}
	}

	void RunThread()
	{
		SetTimePeriod({(per1 * 1e-3), (per2 * 1e-3), (per3 * 1e-3), (per4 * 1e-3)});
		std::thread threadA(&HalDataSyncNode::functionA, this);
		std::thread threadB(&HalDataSyncNode::functionB, this);
		// std::thread threadC(&HalDataSyncNode::functionC, this);
		// std::thread threadD(&HalDataSyncNode::functionD, this);

		threadA.join();
		threadB.join();
		// threadC.join();
		// threadD.join();
	}

private:
	std::shared_ptr<TimeSyncType> m_timeSync;
	float m_timeA;
	float m_timeB;
	float m_timeC;
	float m_timeD;
	float m_fTimeRadar;
	float m_fTimeRadarLast;
};

END_NS_ZF_HAL_SYNC

using namespace NS_ZF::driver::synchronization;

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	printf("hello world zf_hal_can_cam_sync package\n");
	HalDataSyncNode node;
	node.RunThread();
	// std::thread threadA(functionA);
	// std::thread threadB(functionB);

	// threadA.join();
	// threadB.join();

	return 0;
}