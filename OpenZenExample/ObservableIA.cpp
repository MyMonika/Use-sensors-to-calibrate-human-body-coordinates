#include "ObservableIA.h"
#include "observable.h"
#include <vector>
#include <Process.h>
#include <Observer.h>
ObservableIA::ObservableIA() {
	std::vector<Observer*> observer1;
	observer = observer1;
}
void ObservableIA::registerObserver(Observer* p) {
	observer.push_back(p);
}
void ObservableIA::notifyAllObserver(int index) {

	for (auto it : observer)
	{
		it->Update(index);
	}
}
	
