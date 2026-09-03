#include "parseData.h"
#include "qdebug.h"
#include "RF_probe_control.h"
#include <charconv>
parseData::parseData(std::atomic<bool>* saveFile)
{
	save = saveFile;
	parseDatas.store({ 0,0,0,0,0,0,0 }, std::memory_order_release);

}

parseData::~parseData()
{}

int parseData::parseRuning(RingBuffer* pressureBuffer, RingBuffer* resistanceBuffer, CirBuffer<std::array<double, 4>>* posBuffer) {

	globalTimer.start();
	double Datas[8] = { 0 };
	const auto targetDuration = std::chrono::milliseconds(10);
	while (1) {
		// 计时
		auto start = std::chrono::high_resolution_clock::now();

		std::string buffer1;
		std::string buffer0;
		std::string buffer2;
		std::string buffer3;
		// 压力
 		pressureBuffer->readFrameByDelimiter(buffer1,'=');
		double value;
		if (!buffer1.empty())
		{
			
			try {
				// qDebug() << Datas[1] << endl;
				Datas[1] = abs(std::stod(buffer1)*-1);

				/*auto result = std::from_chars(
					buffer1.data(),
					buffer1.data() + buffer1.size(),
					value
				);

				if (result.ec == std::errc()) {
					Datas[1] = -value;
				}
				else {
					throw std::invalid_argument("invalid stod argument");
				}*/
			}
			catch (const std::invalid_argument& e) {
				Datas[1] = 0.00;
			}

		}

		// 电阻
		resistanceBuffer->readFrameByDelimiter(buffer0,'\n');

		if (!buffer0.empty())
		{
			
			try {

				buffer2 = buffer0.substr(0, 12);
				buffer3 = buffer0.substr(13, 25);

				Datas[2] = std::stod(buffer2);
				Datas[7] = std::stod(buffer3);
				/*auto result = std::from_chars(
					buffer2.data(),
					buffer2.data() + buffer2.size(),
					value
				);

				if (result.ec == std::errc()) {
					Datas[2] = -value;
				}
				else {
					throw std::invalid_argument("invalid stod argument");
				}*/
			}
			catch (const std::invalid_argument& e) {
				Datas[2] = 0.00;
				Datas[7] = 0.00;
			}
			catch (const std::out_of_range& e) {
				Datas[2] = 0.00;
				Datas[7] = 0.00;
			}


		}

	if (!posBuffer->isEmpty()) {
		std::array<double, 4> posValue;
		posValue = posBuffer->read(); 
		// 脉冲位移转换
		for (auto& v : posValue) {
			v = v * 0.078125;
		}
		// 位移
		Datas[3] = std::round(posValue[0] * 100000.0) / 100000.0;
		Datas[4] = std::round(posValue[1] * 100000.0) / 100000.0;
		Datas[5] = std::round(posValue[2] * 100000.0) / 100000.0;
		Datas[6] = std::round(posValue[3] * 100000.0) / 100000.0;
		/*Datas[3] = posValue[0];
		Datas[4] = posValue[1];
		Datas[5] = posValue[2];
		Datas[6] = posValue[3];*/
	}

		// 原子变量
		qint64 timestamp = globalTimer.elapsed();
		Datas[0] = timestamp;

		parseDatas.store({ Datas[0],Datas[1],Datas[2],Datas[7],Datas[3],Datas[4],Datas[5],Datas[6]}, std::memory_order_release);

		// 提交数据
		DataFrame frame;
		frame.ts = Datas[0];
		frame.pressure = Datas[1];
		frame.resistance = Datas[2];
		frame.X = Datas[7];
		frame.pos[0] = Datas[3]; frame.pos[1] = Datas[4];
		frame.pos[2] = Datas[5]; frame.pos[3] = Datas[6];

		if (*save) {
			if (fileWriter_ == nullptr) {
				fileWriter_ = new LowLatencyFileWriter(GetTimeFileName());
			}
			fileWriter_->submit(frame);
		}
		else{
			if (fileWriter_ != nullptr) {
				delete fileWriter_;
				fileWriter_ = nullptr;
			}
		}

		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		auto remaining = targetDuration - elapsed;
		if (remaining > std::chrono::microseconds(0)) {
			std::this_thread::sleep_for(remaining);
		}

	}

	return 0;
	
}
