#include <chrono>

class BatteryCalculator {
	public:
		BatteryCalculator(const float min_v, const unsigned long sta_window, const unsigned long lta_window) {
			min_voltage = min_v;
			sta_duration = sta_window;
			lta_duration = lta_window;
			calc_start_time = std::chrono::system_clock::now();
		}

		unsigned long get_sta();
		unsigned long get_lta();
		
	private:
		float min_voltage; 	// minimum voltage before needing recharge
		unsigned long sta_duration; 	// short term average window duration, in seconds
		unsigned long lta_duration; 	// long term average window duration, in seconds
		std::chrono::time_point<std::chrono::system_clock> calc_start_time;	// start time when calculator object was created
		std::vector<std::pair<float, std::chrono::time_point<std::chrono::system_clock>>> measurement_log; 	// vector containing std::pair's with voltage, timestamp
		unsigned long sta_remaining;	// in seconds. = NULL upon initialization, and stays NULL until enough voltage sameples are stored
		unsigned long lta_remaining;	// in seconds. = NULL upon initialization, and stays NULL until enough voltage sameples are stored

		unsigned long calc_sta();
		unsigned long calc_lta();
};


