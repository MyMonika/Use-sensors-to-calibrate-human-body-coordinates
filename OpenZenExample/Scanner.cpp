#include "Scanner.h"
#include <OpenZen.h>
#include <array>
#include <atomic>
#include <process.h>
#include <coodTransformation.h>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include "coodTransformationUser.h"
#include "Calculator.h"
#include <queue>
#include "Tari.h"
#include<condition_variable>
#include "Process.h"
using namespace zen;

std::vector<std::queue<Eigen::Matrix3d>> Scanner::sensorData(15);
std::vector<std::queue<Eigen::Matrix3d>>Scanner::IA(15);
std::vector<bool>Scanner::IAisFull(15);
std::vector<float> Scanner::q1 = { 1.0f, 0.0f, 0.0f, 0.0f };
std::map<ZenSensorHandle_t, int>Scanner:: map;
int Scanner::len = 0;

bool operator==(const ZenSensorHandle_t s1, const ZenSensorHandle_t s2)
{
	return ((s1.handle == s2.handle));
}
bool operator<(const ZenSensorHandle_t s1, const ZenSensorHandle_t s2) {
	return s1.handle < s2.handle;
}

namespace
{
	void addDiscoveredSensor(const ZenEventData_SensorFound& desc, Scanner* s)
	{
		std::lock_guard<std::mutex> lock(s->g_discoverMutex);
		s->g_discoveredSensors.push_back(desc);
	}
}
void Scanner::pollLoop1(std::reference_wrapper<ZenClient> client, std::vector<coodTransformation*> method) {
	while (!g_terminate)
	{
		while (true)
		{
			//pair内部存储了两个值
			//waitfornext的返回类型为 std::pair<bool, ZenEvent>
			//wait函数等待此客户端队列中的下一个事件。如果条目可用，此方法将立即返回。否则方法调用将阻塞，直到事件可用。
			//如果在另一个线程上释放了传感器连接，则方法调用将返回。如果队列中没有可用的事件，则std:：pair的bool条目将为false。
			const auto pair = client.get().waitForNextEvent();
			const bool success = pair.first;
			auto& event = pair.second;
			//std::cout << "子线程执行 " << std::endl;
			//未成功则断开
			if (!success)
				break;

			if (!event.component.handle)
			{
				switch (event.eventType)
				{
				case ZenSensorEvent_SensorFound:
					addDiscoveredSensor(event.data.sensorFound, this);
					std::cout << "hhh:" << std::endl;
					break;

				case ZenSensorEvent_SensorListingProgress:
					std::cout << "Sensor listing progress " << event.data.sensorListingProgress.progress * 100
						<< " %" << std::endl;
					if (event.data.sensorListingProgress.progress == 1.0f) {
						std::cout << "正在初始化...." << std::endl;
						Init();
						g_discoverCv.notify_one();
						//唤醒主线程
					}

					break;
				}
			}

			else {
	
					//找到对应的索引
					mu.lock();
					int sensor_index = map.at(event.sensor);
					if ((sensor_index==P->getIndex1()||sensor_index==P->getIndex2())&&
						(data_si[sensor_index].handle > 0) && (event.component.handle == data_si[sensor_index].handle))
					{
						//std::cout << "开始采集数据 " << std::endl;
						switch (event.eventType)
						{
							case ZenImuEvent_Sample:
								if (time[sensor_index]++ % 50 == 0) {
									std::cout << "                         对应传感器为 " << sensor_index << std::endl;

									/*std::cout << "Event type: " << event.eventType << std::endl;
									std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
									std::cout << "> Acceleration: \t x = " << event.data.imuData.a[0]
										<< "\t y = " << event.data.imuData.a[1]
										<< "\t z = " << event.data.imuData.a[2] << std::endl;
									std::cout << "> Gyro: \t\t x = " << event.data.imuData.g[0]
										<< "\t y = " << event.data.imuData.g[1]
										<< "\t z = " << event.data.imuData.g[2] << std::endl;*/

									////位姿融合
									q1=P->Current_fixed_quaternion(method, q1, event.data.imuData.g[0], event.data.imuData.g[1], event.data.imuData.g[2],
										event.data.imuData.a[0], event.data.imuData.a[1], event.data.imuData.a[2]);
									//将传感器的矩阵加入队列
									sensorData[sensor_index].push(Calculator::SensorToEnvir( Calculator::QuaternionToRotation(q1)));
									//每20次得到一个内蕴平均算法得到的矩阵，将内蕴算法矩阵加入队列
									if (sensorData[sensor_index].size() == 2 && (sensor_index == P->getIndex1() || sensor_index == P->getIndex2())) {
										//提醒观察者来接收数据
										obser->notifyAllObserver(sensor_index);
										while (!sensorData[sensor_index].empty()) sensorData[sensor_index].pop();
									}
									time[sensor_index] = 0;
									std::cout << "//////////////////////////////////////// " << std::endl;
								}
								break;
							default: std::cout << "未读到，跳过" << std::endl; break;
						}
			
					}
					//代码解锁
					mu.unlock();
			}
		}
	}
	std::cout << "--- Exit polling thread ---" << std::endl;
}

int Scanner::ScanAndRead(std::vector<coodTransformation*> method) {

	std::cout << "Type: " << std::endl;
	std::cout << " - 'q' to quit;" << std::endl;
	std::cout << " - 'r' to manually release the sensor;" << std::endl;
	//使用实例方法来得到Zenclient的实例化， Use the zen::make_client method to obtain an instance of  ZenClient.
	auto clientPair = make_client();
	auto& clientError = clientPair.first;
	auto& client = clientPair.second;

	std::cout << "执行中:" << std::endl;
	if (clientError) {
		return clientError;
		std::cout << "错误！！" << std::endl;
	}
	//开始连接
	std::thread pollingThread(&Scanner::pollLoop1, this, std::ref(client),method);
	std::cout << "Listing sensors:" << std::endl;
	if (auto error = client.listSensorsAsync())
	{
		g_terminate = true;
		client.close();
		pollingThread.join();
		return error;
	}
	std::unique_lock<std::mutex> lock(g_discoverMutex);
	g_discoverCv.wait(lock);

	if (g_discoveredSensors.empty())
	{
		g_terminate = true;
		client.close();
		pollingThread.join();
		return ZenError_Unknown;
	}
	std::cout << "the range is 0-" << g_discoveredSensors.size() - 1 << ":" << std::endl;
	//	unsigned  int idx;
	/*do
	{
		std::cout << "Provide an index within the range 0-" << g_discoveredSensors.size() - 1 << ":" << std::endl;
		std::cin >> idx;
	} while (idx >= g_discoveredSensors.size());*/

	std::thread pollingThread2(&Scanner::End, this, std::ref(client));
		//以下开始读取数据
		int j = 0;
			data_si.push_back(SIData());
				auto sensorPair1 = (client.obtainSensor(g_discoveredSensors[j]));
				data_si[j].sensorPair = &(sensorPair1);
				data_si[j].sensor = &(data_si[j].sensorPair->second);
				map[data_si[j].sensor->sensor()] = j;
				data_si[j].imuPair = data_si[j].sensor->getAnyComponentOfType(g_zenSensorType_Imu);
				imu1 = data_si[j].imuPair.second;
				data_si[j].handle = imu1.component().handle;

				//imu1.executeProperty(ZenImuProperty_StartSensorSync);
				imu1.setBoolProperty(ZenImuProperty_StreamData, true);
				std::cout << j << "号传感器已经登记结束" << std::endl;

				//// Get a string property
				auto sensorModelPair1 = data_si[j].sensor->getStringProperty(ZenSensorProperty_SensorModel);
				auto& sensorModelName1 = sensorModelPair1.second;
				std::cout << "Sensor Model: " << sensorModelName1 << std::endl;
				j++;

				data_si.push_back(SIData());
				auto sensorPair = (client.obtainSensor(g_discoveredSensors[j]));
				data_si[j].sensorPair = &(sensorPair);
				data_si[j].sensor = &(data_si[j].sensorPair->second);
				map[data_si[j].sensor->sensor()] = j;
				data_si[j].imuPair = data_si[j].sensor->getAnyComponentOfType(g_zenSensorType_Imu);
				imu = data_si[j].imuPair.second;
				data_si[j].handle = imu.component().handle;
				//imu.setBoolProperty(ZenImuProperty_StreamData, true);
				std::cout << j << "号传感器已经登记结束" << std::endl;

				//// Get a string property
				auto sensorModelPair = data_si[j].sensor->getStringProperty(ZenSensorProperty_SensorModel);
				auto& sensorModelName = sensorModelPair.second;
				std::cout << "Sensor Model: " << sensorModelName << std::endl;
				
				imu1.executeProperty(ZenImuProperty_StartSensorSync);
				imu.executeProperty(ZenImuProperty_StartSensorSync);

				std::this_thread::sleep_for(std::chrono::seconds(3));

				imu1.executeProperty(ZenImuProperty_StopSensorSync);
				imu.executeProperty(ZenImuProperty_StopSensorSync);
				while(!g_terminate)client.pollNextEvent();

}
void Scanner::Init() {
	//添加观察者
	obser->registerObserver(P);
	//这里保留传感器的数量
	Scanner::len = g_discoveredSensors.size();
	std::cout << "len==" << Scanner::len << std::endl;
	time.resize(len, 0);
	data_si.reserve(Scanner::len);
	g_discoveredSensors.reserve(Scanner::len);
	//改变指针的指向
	for (int idx = 0; idx < Scanner::len; ++idx) {
		std::cout << idx << ": " << g_discoveredSensors[idx].name << " (" << g_discoveredSensors[idx].ioType << ")" << std::endl;
	}
	std::cout << "--- pollLoop1执行完成 ---" << std::endl;
	return;
}

void Scanner::End(std::reference_wrapper<ZenClient> client) {
	std::string line;
	if (std::getline(std::cin, line))
	{
		if (line == "q") {
			Set_g_terminate(true);
			for (int i = 0; i < len; i++) {
				data_si[i].imuPair.second.executeProperty(ZenImuProperty_StopSensorSync);
			}
			std::cout << "停止同步" << std::endl;
			client.get().close();
			return;
		}
	}
}

