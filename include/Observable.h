#ifndef _OBSERVABLE_
#define _OBSERVABLE_
#include <vector>
#include <coodTransformation.h>
#include <Observer.h>
#include "coodTransformationUser.h"
class Observable
{
	public :
		virtual void registerObserver(Observer* p) = 0;
		virtual void notifyAllObserver(int index) = 0;
};
#endif

