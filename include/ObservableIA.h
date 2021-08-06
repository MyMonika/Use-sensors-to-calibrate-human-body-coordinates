#ifndef _OBSERVABLEIA_
#define _OBSERVABLEIA_
#include "observable.h"
#include <vector>
#include <Observer.h>
#include <Process.h>
//这个类的作用是内蕴标定算法需要收集的数据足够的时候，去通知process处理
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

