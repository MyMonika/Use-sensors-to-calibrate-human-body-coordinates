#pragma once
#include <OpenZen.h>
#include <atomic>
#include <Process.h>
#include <coodTransformation.h>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <thread>
#include <Observable.h>
#include "SIData.h"
#include <ObservableIA.h>
#include <map>
using namespace zen;

class Scanner
{
	public:
		static std::vector<std::queue<Eigen::Matrix3d>> sensorData;
		static std::vector<std::queue<Eigen::Matrix3d>> IA;
		static std::vector<bool> IAisFull;
		static std::vector<float> q1;
		static std::map<ZenSensorHandle_t,int> map;
		static int len;
		ZenSensorComponent imu1;
		ZenSensorComponent imu;
		std::vector <SIData> data_si;
		std::vector<ZenSensorDesc> g_discoveredSensors;
		std::condition_variable g_discoverCv;
		std::mutex g_discoverMutex;
		std::mutex mu;
		std::atomic_bool g_terminate;
		//count的取值是0~len;
		std::vector<unsigned int> time;
		 Process* P;
		 //来作为观察者
		 Observable *obser = new ObservableIA();

		void Init();

		void pollLoop1(std::reference_wrapper<ZenClient> client, std::vector<coodTransformation*> method);

		int ScanAndRead(std::vector<coodTransformation*> method);

		inline void Set_g_terminate(bool res) {
			g_terminate = res;
		}
		inline void SetProcess(Process *p) {
			P = p;
		}
		inline bool get_g_terminate() {
			return g_terminate;
		}
		Scanner() = default;
		void End(std::reference_wrapper<ZenClient> client);
};

