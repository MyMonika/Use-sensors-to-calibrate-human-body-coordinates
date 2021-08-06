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
			//pair�ڲ��洢������ֵ
			//waitfornext�ķ�������Ϊ std::pair<bool, ZenEvent>
			//wait�����ȴ��˿ͻ��˶����е���һ���¼��������Ŀ���ã��˷������������ء����򷽷����ý�������ֱ���¼����á�
			//�������һ���߳����ͷ��˴��������ӣ��򷽷����ý����ء����������û�п��õ��¼�����std:��pair��bool��Ŀ��Ϊfalse��
			const auto pair = client.get().waitForNextEvent();
			const bool success = pair.first;
			auto& event = pair.second;
			//std::cout << "���߳�ִ�� " << std::endl;
			//δ�ɹ���Ͽ�
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
						std::cout << "���ڳ�ʼ��...." << std::endl;
						Init();
						g_discoverCv.notify_one();
						//�������߳�
					}

					break;
				}
			}

			else {
	
					//�ҵ���Ӧ������
					mu.lock();
					int sensor_index = map.at(event.sensor);
					if ((sensor_index==P->getIndex1()||sensor_index==P->getIndex2())&&
						(data_si[sensor_index].handle > 0) && (event.component.handle == data_si[sensor_index].handle))
					{
						//std::cout << "��ʼ�ɼ����� " << std::endl;
						switch (event.eventType)
						{
							case ZenImuEvent_Sample:
								if (time[sensor_index]++ % 50 == 0) {
									std::cout << "                         ��Ӧ������Ϊ " << sensor_index << std::endl;

									/*std::cout << "Event type: " << event.eventType << std::endl;
									std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
									std::cout << "> Acceleration: \t x = " << event.data.imuData.a[0]
										<< "\t y = " << event.data.imuData.a[1]
										<< "\t z = " << event.data.imuData.a[2] << std::endl;
									std::cout << "> Gyro: \t\t x = " << event.data.imuData.g[0]
										<< "\t y = " << event.data.imuData.g[1]
										<< "\t z = " << event.data.imuData.g[2] << std::endl;*/

									////λ���ں�
									q1=P->Current_fixed_quaternion(method, q1, event.data.imuData.g[0], event.data.imuData.g[1], event.data.imuData.g[2],
										event.data.imuData.a[0], event.data.imuData.a[1], event.data.imuData.a[2]);
									//���������ľ���������
									sensorData[sensor_index].push(Calculator::SensorToEnvir( Calculator::QuaternionToRotation(q1)));
									//ÿ20�εõ�һ������ƽ���㷨�õ��ľ��󣬽������㷨����������
									if (sensorData[sensor_index].size() == 2 && (sensor_index == P->getIndex1() || sensor_index == P->getIndex2())) {
										//���ѹ۲�������������
										obser->notifyAllObserver(sensor_index);
										while (!sensorData[sensor_index].empty()) sensorData[sensor_index].pop();
									}
									time[sensor_index] = 0;
									std::cout << "//////////////////////////////////////// " << std::endl;
								}
								break;
							default: std::cout << "δ����������" << std::endl; break;
						}
			
					}
					//�������
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
	//ʹ��ʵ���������õ�Zenclient��ʵ������ Use the zen::make_client method to obtain an instance of  ZenClient.
	auto clientPair = make_client();
	auto& clientError = clientPair.first;
	auto& client = clientPair.second;

	std::cout << "ִ����:" << std::endl;
	if (clientError) {
		return clientError;
		std::cout << "���󣡣�" << std::endl;
	}
	//��ʼ����
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
		//���¿�ʼ��ȡ����
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
				std::cout << j << "�Ŵ������Ѿ��Ǽǽ���" << std::endl;

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
				std::cout << j << "�Ŵ������Ѿ��Ǽǽ���" << std::endl;

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
	//��ӹ۲���
	obser->registerObserver(P);
	//���ﱣ��������������
	Scanner::len = g_discoveredSensors.size();
	std::cout << "len==" << Scanner::len << std::endl;
	time.resize(len, 0);
	data_si.reserve(Scanner::len);
	g_discoveredSensors.reserve(Scanner::len);
	//�ı�ָ���ָ��
	for (int idx = 0; idx < Scanner::len; ++idx) {
		std::cout << idx << ": " << g_discoveredSensors[idx].name << " (" << g_discoveredSensors[idx].ioType << ")" << std::endl;
	}
	std::cout << "--- pollLoop1ִ����� ---" << std::endl;
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
			std::cout << "ֹͣͬ��" << std::endl;
			client.get().close();
			return;
		}
	}
}

