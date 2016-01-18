//|--------------------------------------------|
//|------Electronics and Computer Science------|
//|Faculty of Physical Sciences and Engineering|
//|---------University of Southampton----------|
//|-----------Thomas Paul Rogers---------------|
//|---Balancing an Inverted Pendulum using a---|
//|-------Smartphone Controlled RC Car---------|
//|----Program code - 06/02/2013-01/05/2013----|
//|--------------------------------------------|

#define F_CPU 20000000UL //define system clock frequency
#define BAUD 115200 //define UART baudrate

//libraries
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdlib.h>
//libraries

//TWI functions
void TWI_Init(void);//initialise
void TWI_Start(void);//start signal
void TWI_Stop(void);//stop signal
void TWI_Write(uint8_t u8data);//write data
uint8_t TWI_Read_ACK(void);//read with acknowledge bit
uint8_t	TWI_Read_NACK(void);//read without acknowledge bit
uint8_t TWI_Get_Status(void);//get bus status
int TWI_READBYTE(unsigned char address);//read byte from register 'address'
void TWI_WRITEBYTE(unsigned char address, unsigned char data);//write byte 'data' to register 'address'

//other functions
void setup(void);//set up PWM registers, inputs and outputs, encoder interrupts and UART in/out
void output_timer_on(void);//turn on output timer used to output PID data
void accel_timer_on(void);//turn on accel timer used to output acceleration data
void accel_timer_off(void);//turn off accel timer
void IMU_setup(void);//set up the MPU6050 by setting a series of registers
void print_float(float output);//print a floating point number via the UART
float get_accel(char direction);//get acceleration data in 'direction' x, y or z
void print_accels(void);//print the acquired acceleration data

//mode functions
void PID(void);//PID algorithm
void manual(void);//manual control
void check_imu(void);//check WHO_AM_I register of MPU6050
void fast_mode(void);//put RN-42 HID into fast mode
void accel_processor(int speed, int dir);//turn motors on in direction 'dir' at speed 'speed' and then collect and print accelerations
void acceleration_mode();//acceleration data acquisition mode

//UART functions
void uart_init(void);//initialise
void uart_putchar(char c, FILE *stream);//send a character
char uart_getchar(FILE *stream);//recieve a character

//AVRSTD LIB functions to redefine how printf and getchar etc work
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

//PID variables
int position;//encoder position
unsigned char direction;//encoder direction
double angle = 0;//encoder angle
double previous;//previous angle
long voltage;//PID output
float integral = 0;//error accumulator
float differential = 0;//tangent variable

//PID GAINS
float kp = 750;//proportional 
float ki = 450; //integral
float kd = 200;//differential

//other variables
//int aggressive_expansion = 20;
volatile int timer_count = 0;
volatile int accel_counter = 0;
volatile float times[35];
volatile float accels[35];
int arraypos;
volatile char output_buffer[12];
int int_voltage = 0;
char input;
float acceleration = 0;
float accel_top = 0;
int count = 0;
float moment = 0;

//bit macros
#define bit(x) (1<<(x))
#define set_bit(x,y) ((x) |= (bit(y)))
#define clear_bit(x,y) ((x) &= (~(bit(y))))

//encoder macros
//#define right_enc_mask 0x00000011
//#define right_enc_pins (PINC & right_enc_mask)

//motor macros
//#define turn_left (set_bit(PORTD,4))
//#define turn_right (set_bit(PORTB,0))
//#define not_left (clear_bit(PORTD,4))
//#define not_right (clear_bit(PORTB,0))
#define forwards (set_bit(PORTD,2))
#define reverse (set_bit(PORTD,3))
#define not_forwards (clear_bit(PORTD,2))
#define not_reverse (clear_bit(PORTD,3)
//#define power_steering OCR0A
#define gear OCR1A
#define full_volts 255
#define seven_volts 238
#define six_volts 204
#define five_volts 170
#define four_volts 136

//TWI wait for status macro
#define wait_for_status while ((TWCR & (1<<TWINT)) == 0)

//MPU6050 addresses
#define MPU6050_ADR 0xD0
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_MOT_THR 0x1F
#define MPU6050_FIFO_EN 0x23
#define MPU6050_I2C_MSTR_CTRL 0x24
#define MPU6050_SLV0_ADDR 0x25
#define MPU6050_SLV0_REG 0x26
#define MPU6050_SLV0_CTRL 0x27
#define MPU6050_SLV1_ADDR 0x28
#define MPU6050_SLV1_REG 0x28
#define MPU6050_SLV1_CTRL 0x2A
#define MPU6050_SLV2_ADRR 0x2B
#define MPU6050_SLV2_REG 0x2C
#define MPU6050_SLV2_CTRL 0x2D
#define MPU6050_SLV3_ADRR 0x2E
#define MPU6050_SLV3_REG 0x2F
#define MPU6050_SLV3_CTRL 0x30
#define MPU6050_SLV4_ADRR 0x31
#define MPU6050_SLV4_REG 0x32
#define MPU6050_SLV4_DO 0x33
#define MPU6050_SLV4_CTRL 0x34
#define MPU6050_INT_PIN_CNFG 0x37
#define MPU6050_INT_EN 0x38
#define MPU6050_INT_STATUS 0x3A
//output
#define MPU6050_ACCEL_X_OUT_H 0x3B
#define MPU6050_ACCEL_X_OUT_L 0x3C
#define MPU6050_ACCEL_Y_OUT_H 0x3D
#define MPU6050_ACCEL_Y_OUT_L 0x3E
#define MPU6050_ACCEL_Z_OUT_H 0x3F
#define MPU6050_ACCEL_Z_OUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_X_OUT_H 0x43
#define MPU6050_GYRO_X_OUT_L 0x44
#define MPU6050_GYRO_Y_OUT_H 0x45
#define MPU6050_GYRO_Y_OUT_L 0x46
#define MPU6050_GYRO_Z_OUT_H 0x47
#define MPU6050_GYRO_Z_OUT_L 0x48

#define MPU6050_SLV0_DO 0x63
#define MPU6050_SLV1_DO 0x64
#define MPU6050_SLV2_DO 0x65
#define MPU6050_SLV3_DO 0x66
#define MPU6050_I2C_MSTR_DELAY_CTRL 0x67
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_MOT_DET_CTRL 0x69
#define MPU6050_USR_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75

int main(void)
{
	setup();
	uart_init();
	TWI_Init();
	input = getchar();
	while((input != 's'))//wait for user input to begin program
	{
		input = getchar();
	}
	sei();//global interrupts on
	IMU_setup();
	printf("\nLet's Begin\n\nChoose an option:\n\n  space bar - PID (loops forever)\n        'm' - manual control (loops forever)\n        'i' - check IMU\n        'x' - get x acceleration\n        'y' - get y acceleration\n        'z' - get z acceleration\n        'f' - bluetooth fast mode\n        't' - test IMU write\n        'a' - enter acceleration mode\n");
	while((1))
	{
		input = getchar();
		if ((input == ' '))//PID algorithm
		{
			output_timer_on();
			while((1))
			{
				PID();
			}
		}
		else if ((input == 'm'))//manual control
		{
			printf("\nManual Mode:\n\nInstructions:\n\n    'n' - forwards\n    'v' - reverse\n    'b' - stop\n  '1-9' - use the number keys to select the power level\n");
			while((1))
			{
				manual();
			}
		}
		else if ((input == 'i'))//check WHO_AM_I register
		{
			check_imu();	
		}
		else if ((input == 'x'))//get x direction acceleration
		{
			acceleration = get_accel('x');
			print_float(acceleration);
			printf("\n");
		}
		else if ((input == 'y'))//get y direction acceleration
		{
			acceleration = get_accel('y');
			print_float(acceleration);
			printf("\n");
		}
		else if ((input == 'z'))//get z direction acceleration
		{
			acceleration = get_accel('z');
			print_float(acceleration);
			printf("\n");
		}
		else if ((input == 'f'))//place RN-42 HID into fast mode
		{
			fast_mode();
		}
		else if ((input == 't'))//write a regsiter of the IMU and check it worked
		{
			TWI_WRITEBYTE(MPU6050_PWR_MGMT_1, 0x02);
			if ((TWI_READBYTE(MPU6050_PWR_MGMT_1) == 0x02))
				printf("\nSuccess\n");
			else
				printf("\nFailure\n");
		}
		else if ((input == 'a'))//acceleration mode for acquiring data
		{
			acceleration_mode();
		}
	}

}

void PID(void)//PID algorithm
{
	previous = angle;//set previous to last angle
	angle = position * 0.18;//calculate angle
	integral = integral + angle;//accumulator
	differential = angle - previous;//tangent
	if ((angle == 0))//reset accumulator when output passes zero
		integral = 0;
	voltage = ((kp * angle) + (ki * integral) + (kd * differential));//calculate output
	//set int_voltage (variable to be outputted (Long int can not be sent through UART)
	if ((voltage >= 32767))
		int_voltage = 32767;
	else if ((voltage <= -32768))
		int_voltage = -32768;
	else
		int_voltage = voltage;
	//direction
	if ((voltage < 0))
	{
		reverse;
		not_forwards;
		voltage = voltage * -1;
	}
	else
	{
		forwards;
		not_reverse;
		//voltage = voltage * (1 + (aggressive_expansion / 100)); //% increase to make pushing easier :)
	}
	//upper output limit
	if ((angle == 0))
		voltage = 0;	
	if ((voltage > 250))
		voltage = 250;
	//lower output limit
	//if ((voltage < 130))
	//{
	//	if ((voltage > 0))
	//	voltage = 130;
	//}
	gear = voltage;//pass calculated output to motors
	
}

void acceleration_mode(void)//mode to acquire acceleration data, as described by menu printf's and function calls
{
	printf("\nAcceleration Mode:\n\nChoose an option:\n\n  '1' - forwards operation\n  '2' - reverse operation\n");
	input = getchar();
	if ((input == '1'))//forwards
	{
		printf("\nForwards Mode:\n\nChoose an option:\n\n  '3' - ~75%% power\n  '4' - ~78%% power\n  '5' - ~82%% power\n  '6' - ~86%% power\n  '7' - ~90%% power\n  '8' - ~94%% power\n  '9' - ~98%% power\n  ");		
		input = getchar();
		if ((input == '3'))
			accel_processor(3,1);
		else if ((input == '4'))
			accel_processor(4,1);
		else if ((input == '5'))
			accel_processor(5,1);
		else if ((input == '6'))
			accel_processor(6,1);
		else if ((input == '7'))
			accel_processor(7,1);
		else if ((input == '8'))
			accel_processor(8,1);
		else if ((input == '9'))
			accel_processor(9,1);
	}
	else if ((input == '2'))//reverse
	{
		printf("\nReverse Mode:\n\nChoose an option:\n\n  '3' - ~75%% power\n  '4' - ~78%% power\n  '5' - ~82%% power\n  '6' - ~86%% power\n  '7' - ~90%% power\n  '8' - ~94%% power\n  '9' - ~98%% power\n  ");	
		input = getchar();
		if ((input == '3'))
			accel_processor(3,0);
		else if ((input == '4'))
			accel_processor(4,0);
		else if ((input == '5'))
			accel_processor(5,0);
		else if ((input == '6'))
			accel_processor(6,0);
		else if ((input == '7'))
			accel_processor(7,0);
		else if ((input == '8'))
			accel_processor(8,0);
		else if ((input == '9'))
			accel_processor(9,0);
	}
}

void manual(void)//manual control
{
	input = getchar();
	if ((input == 'n'))//forwards
	{
		forwards;
		not_reverse;
	}
	else if ((input == 'v'))//reverse
	{
		reverse;
		not_forwards;
	}		
	else if ((input == 'b'))//stop
	{
		forwards;
		reverse;
		_delay_ms(20);
		not_forwards;
		not_reverse;
	}
	else if ((input == '1'))//gears.....
		gear = 170;
	else if ((input == '2'))
		gear = 180;
	else if ((input == '3'))
		gear = 190;
	else if ((input == '4'))
		gear = 200;
	else if ((input == '5'))
		gear = 210;
	else if ((input == '6'))
		gear = 220;
	else if ((input == '7'))
		gear = 230;
	else if ((input == '8'))
		gear = 240;
	else if ((input == '9'))
		gear = 250;	
}

void check_imu(void)//check value of WHO_AM_I register
{
	int reader = 0x00;
	reader = TWI_READBYTE(MPU6050_WHO_AM_I);
	if ((reader = 0x68))
		printf("\nsuccess\n");
	else
		printf("\nfailure\n");
}

void fast_mode(void)//enter fast mode
{
	printf("$$$");
	_delay_ms(10);
	printf("F,1\n");
	_delay_ms(20);
	printf("\nFast Mode Activated\n");
}

void uart_init(void)//UART initialiser
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
		set_bit(UCSR0A,U2X0);
	#else
		clear_bit(UCSR0A,U2X0);
	#endif
	UCSR0C=(3<<UCSZ00);
	set_bit(UCSR0B,RXEN0);
	set_bit(UCSR0B,TXEN0);
}

void uart_putchar(char c, FILE *stream)//uart writer
{
	if ((c == '\n'))
	{
		uart_putchar('\r',stream);
	}
	loop_until_bit_is_set(UCSR0A,UDRE0);
	UDR0 = c;
}

char uart_getchar(FILE *stream)//uart reader
{
	loop_until_bit_is_set(UCSR0A,RXC0);
	return UDR0;
}

ISR(PCINT1_vect)//encoder algorithm called on pin change of the channels
{
	static unsigned char last_state = 0, state;
	state = (PINC & 3);
	direction = ((last_state & 1) ^ ((state & 2) >> 1));
	if ((direction == 0))
		position++;
	else
		position--;
	last_state = state;
}

ISR(TIMER2_COMPA_vect)//timer interrupt to output PID data
{
	timer_count++;
	int i;
	int j = 0;
	float output;
	output = timer_count * 9.984;
	dtostrf(output, 12, 5, output_buffer);//convert double to string
	for (i = 0; i < 12; i++)//loop to print each character
	{
		if ((output_buffer[i] == '.'))//only print one of the many decimal points produced
		{
			if ((j == 0))
			{
				printf("%c",output_buffer[i]);
				j++;
			}			
		}
		else
		printf("%c",output_buffer[i]);
	}
	printf(",");
	j = 0;
	dtostrf(angle, 12, 5, output_buffer);//repeat of above procedure
	for (i = 0; i < 12; i++)
	{
		if ((output_buffer[i] == '.'))
		{
			if ((j == 0))
			{
				printf("%c",output_buffer[i]);
				j++;
			}			
		}
		else
		printf("%c",output_buffer[i]);
	}
	printf(",%d\n",int_voltage);
}

ISR(TIMER0_COMPA_vect)//timer interrupt to gather acceleration data
{
	accel_counter++;
	times[accel_counter] = (accel_counter * 5.0176);
	accels[accel_counter] = get_accel('y');
}

float get_accel(char direction)//function to get acceleration in particular direction
{
	int out = 0x00;
	float accel = 0;
	unsigned char low = 0x00;
	unsigned char high = 0x00;
	if ((direction == 'x'))
	{
		low = TWI_READBYTE(MPU6050_ACCEL_X_OUT_L);
		high = TWI_READBYTE(MPU6050_ACCEL_X_OUT_H);
		out = ((high<<8)|(low));
		accel = out;
		accel = (accel/16384);
	}
	else if ((direction == 'y'))
	{
		low = TWI_READBYTE(MPU6050_ACCEL_Y_OUT_L);
		high = TWI_READBYTE(MPU6050_ACCEL_Y_OUT_H);
		out = ((high<<8)|(low));
		accel = out;
		accel = (accel/16384);
	}
	else if ((direction == 'z'))
	{
		low = TWI_READBYTE(MPU6050_ACCEL_Z_OUT_L);
		high = TWI_READBYTE(MPU6050_ACCEL_Z_OUT_H);
		out = ((high<<8)|(low));
		accel = out;
		accel = (accel/16384);
	}
	return accel;
}

void print_accels(void)//print acceleration data using print_float
{
	for (arraypos = 1; arraypos < 32; arraypos++)
	{
		print_float(times[arraypos]);
		printf(",");
		print_float(accels[arraypos]);
		printf("\n");
	}
}

void accel_processor(int speed, int dir)//acquire and print acceleration datat
{
	if ((dir == 1))
	{
		forwards;
		not_reverse;
	}
	else
	{
		reverse;
		not_forwards;
	}
	gear = ((speed * 10) + 160);
	accel_counter = 0;
	accel_timer_on();
	while ((accel_counter<31))
	{
		
	}
	accel_timer_off();
	not_forwards;
	not_reverse;
	printf("\n");
	print_accels();
}

//TWI functions

void TWI_Init(void)
{
	TWSR = 0x00;
	TWBR = 0x11;
	TWCR = (1<<TWEN);
}

void TWI_Start(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	wait_for_status;
}

void TWI_Stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void TWI_Write(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	wait_for_status;
}

uint8_t TWI_Read_ACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	wait_for_status;
	return TWDR;
}

uint8_t TWI_Read_NACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	wait_for_status;
	return TWDR;
}

uint8_t TWI_Get_Status(void)
{
	uint8_t status;
	status = TWSR & 0xF8;
	return status;
}

int TWI_READBYTE(unsigned char address)
{
	signed int data = 0x00;
	TWI_Start();
	TWI_Write(MPU6050_ADR);
	TWI_Write(address);
	TWI_Start();
	TWI_Write(MPU6050_ADR | 0x01);
	data = TWI_Read_NACK();
	TWI_Stop();
	return data;
}

void TWI_WRITEBYTE(unsigned char address, unsigned char data)
{
	TWI_Start();
	TWI_Write(MPU6050_ADR);
	TWI_Write(address);
	TWI_Write(data);
	TWI_Stop();
}

void output_timer_on(void)//turn output timer on for PID data
{
	//timer2
	set_bit(TCCR2A,WGM21);//CTC mode
	set_bit(TCCR2B,CS22);//1024 prescaler
	set_bit(TCCR2B,CS21);//1024 prescaler
	set_bit(TCCR2B,CS20);//1024 prescaler
	set_bit(TIMSK2,OCIE2A);//interrupt
	OCR2A = 195;//top
	//produces timer to clear every 9.984ms
}

void accel_timer_on(void)//turn accel timer on for acceleration data
{
	//timer0
	set_bit(TCCR0A,WGM01);//CTC mode
	set_bit(TCCR0B,CS02);//1024 prescaler
	set_bit(TCCR0B,CS00);//1024 prescaler
	set_bit(TIMSK0,OCIE0A);//interrupt
	OCR0A = 97;//top
	//produces timer to clear every 5.0176ms
}

void accel_timer_off(void)//turn timer off
{
	clear_bit(TIMSK0,OCIE0A);
	clear_bit(TCCR0B,CS00);
	clear_bit(TCCR0B,CS02);
	TCNT0 = 0x00;
}

void setup(void)//setup registers, i/o etc
{
	//pwm set up
	
	////front motor
	////timer0
	//set_bit(TCCR0A,COM0A1);//non-inverting
	//set_bit(TCCR0A,WGM01);//fast pwm with top at 255
	//set_bit(TCCR0A,WGM00);//fast pwm with top at 255
	//set_bit(TCCR0B,CS01);//clk/8 prescaler
	////produces fast pwm signal at ocr0a with 100% duty cycle at 255, frequency of 9765.625Hz
	
	//rear motor
	//timer1
	set_bit(TCCR1A,COM1A1);//non inverting
	set_bit(TCCR1A,WGM11);//fast pwm with top at icr1
	set_bit(TCCR1B,WGM13);//fast pwm with top at icr1
	set_bit(TCCR1B,WGM12);//fast pwm with top at icr1
	set_bit(TCCR1B,CS11);//clk/8 prescaler
	ICR1=255;//top
	//produces fast pwm signal at ocr1a with 100% duty cycle at 255, frequency of 9765.625Hz

	//set_bit(DDRB,DDB0);//set PB0 as output - front motor pin
	set_bit(DDRB,DDB1);//set PB1 as output - rear motor enable (OC1A)
	clear_bit(DDRC,DDC3);//set PC3 as input - left encoder channel B
	clear_bit(DDRC,DDC2);//set PC2 as input - left encoder channel A
	clear_bit(DDRC,DDC1);//set PC1 as input - right encoder channel B
	clear_bit(DDRC,DDC0);//set PC0 as input - right encoder channel A
	set_bit(DDRD,DDD2);//set PD2 as output - rear motor pin
	set_bit(DDRD,DDD3);//set PD3 as output - rear motor pin
	//set_bit(DDRD,DDD4);//set PD4 as output - front motor pin
	//set_bit(DDRD,DDD6);//set PD6 as output - front motor enable (OC0A)	
	
	set_bit(PCICR,PCIE1);//enable PCINT[14:8}
	//set_bit(PCMSK1,PCINT11);//turn on PCINT11
	//set_bit(PCMSK1,PCINT10);//turn on PCINT10
	set_bit(PCMSK1,PCINT9);//turn on PCINT9
	set_bit(PCMSK1,PCINT8);//turn on PCINT8
	//blahh
	//power_steering = five_volts;//front motor voltage
	gear = 250;//rear motor voltage
	stdout = &uart_output;
	stdin = &uart_input;
}

void IMU_setup(void)//set up IMU registers
{
	TWI_WRITEBYTE(MPU6050_SMPLRT_DIV, 0x00);
	TWI_WRITEBYTE(MPU6050_CONFIG, 0x00);
	TWI_WRITEBYTE(MPU6050_GYRO_CONFIG, 0x00);
	TWI_WRITEBYTE(MPU6050_ACCEL_CONFIG, 0x00);
	TWI_WRITEBYTE(MPU6050_FIFO_EN, 0x00);
	TWI_WRITEBYTE(MPU6050_MOT_THR, 0x00);
	TWI_WRITEBYTE(MPU6050_I2C_MSTR_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV0_ADDR, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV0_REG, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV0_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV1_ADDR, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV1_REG, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV1_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV2_ADRR, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV2_REG, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV2_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV3_ADRR, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV3_REG, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV3_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV4_ADRR, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV4_REG, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV4_DO, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV4_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_INT_PIN_CNFG, 0x00);
	TWI_WRITEBYTE(MPU6050_INT_EN, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV0_DO, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV1_DO, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV2_DO, 0x00);
	TWI_WRITEBYTE(MPU6050_SLV3_DO, 0x00);
	TWI_WRITEBYTE(MPU6050_I2C_MSTR_DELAY_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_SIGNAL_PATH_RESET, 0x00);
	TWI_WRITEBYTE(MPU6050_MOT_DET_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_USR_CTRL, 0x00);
	TWI_WRITEBYTE(MPU6050_PWR_MGMT_1, 0x02);
	TWI_WRITEBYTE(MPU6050_PWR_MGMT_2, 0x00);
	TWI_WRITEBYTE(MPU6050_FIFO_R_W, 0x00);
}

void print_float(float output)//function to print a float (larger than 8 bits and so can't be handled by UART)
{
	int k;
	int l = 0;
	char buffer[12];
	dtostrf(output,12,5,buffer);
	for (k = 0 ; k < 12 ; k++)
	{
		if ((buffer[k] == '.'))
		{
			if ((l == 0))
			{
				printf("%c",buffer[k]);
				l++;
			}
		}
		else
		printf("%c",buffer[k]);
	}
}