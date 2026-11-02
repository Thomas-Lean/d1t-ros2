#include <iostream>

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

#define SERVO_PORT_NAME "/dev/ttyS4"
#define SERVO_ID 0

using namespace std;
using namespace fsuservo;

FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);

FSUS_Servo servo0(0, &protocol);
FSUS_Servo servo1(1, &protocol);
FSUS_Servo servo2(2, &protocol);
FSUS_Servo servo3(3, &protocol);
FSUS_Servo servo4(4, &protocol);
FSUS_Servo servo5(5, &protocol);
FSUS_Servo servo6(6, &protocol);


int main(){
	cout << "Example Uart Servo Ping" << endl;
	
	while(true)
	{

		bool is_online0 = servo0.ping();

		cout << "Servo ID = " << 0 << " , is ";
 		if (is_online0){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online1 = servo1.ping();

		cout << "Servo ID = " << 1 << " , is ";
 		if (is_online1){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online2 = servo2.ping();

		cout << "Servo ID = " << 2 << " , is ";
 		if (is_online2){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online3 = servo3.ping();

		cout << "Servo ID = " << 3 << " , is ";
 		if (is_online3){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online4 = servo4.ping();

		cout << "Servo ID = " << 4 << " , is ";
 		if (is_online4){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online5 = servo5.ping();

		cout << "Servo ID = " << 5 << " , is ";
 		if (is_online5){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		bool is_online6 = servo6.ping();

		cout << "Servo ID = " << 6 << " , is ";
 		if (is_online6){
			cout << "online" << endl;
		}else{
			cout << "offline" << endl;
		}

		protocol.delay_ms(1000);
		
		cout << "--------------------------------------------" << endl;
		
		
	}
	
}