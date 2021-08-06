#ifndef _OBSERVABLEIA_
#define _OBSERVABLEIA_
#include "observable.h"
#include <vector>
#include <Observer.h>
#include <Process.h>
//���������������̱궨�㷨��Ҫ�ռ��������㹻��ʱ��ȥ֪ͨprocess����
class ObservableIA :
	public Observable
{
	private:
		std::vector<Observer*> observer;
	public:
		void registerObserver(Observer* p);
		void notifyAllObserver(int index);
		ObservableIA();

};
#endif

