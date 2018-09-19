#define NUM_MOTOR_WIRES 4 // number of wires for each stepper motor (28BYJ-48)
#define CLK_HALF_PERIOD_US 1 // half-period of clock signal (microseconds), 1us = 500kHz

#define SR_PIN_SER 7 // serial input (SER) pin
#define SR_PIN_OE 8 // output enable (OE) pin, outputs enabled when LOW
#define SR_PIN_RCLK 9 // storage register clock (RCLK) pin, stores shift registers
#define SR_PIN_SRCLK 10 // shift register clock (SRCLK) pin, shifts the shift registers
#define SR_PIN_SRCLR 11 // shift register clear (SRCLR) pin, clears shift registers when LOW

#define SR_NUM_CHANNELS 8 // number of channels per shift register
#define SR_NUM_UNITS 2 // number of shift register ICs in use

#define MOTOR_NUM_UNITS 4 // number of stepper motors
#define MOTOR_NUM_SIGNALS 4 // number of control signals in stepper control sequence
#define MOTOR_NUM_CHANNELS 4 // number of shift register channels required for each motor

uint8_t sr_serial_buffer[SR_NUM_UNITS];
unsigned long sr_last_write_us = 0; // the last time the shift register was written to (microseconds)

uint8_t motor_signal_list[4] = {0b1010, 0b0110, 0b0101, 0b1001};
// char motor_signal_list[4] = {0b1111, 0b1111, 0b1111, 0b1111};
int motor_curr_signal_list[MOTOR_NUM_UNITS]; // index of currently used element in signal list for each motor

void setup() {
	pinMode(SR_PIN_SER, OUTPUT);
	pinMode(SR_PIN_OE, OUTPUT);
	pinMode(SR_PIN_RCLK, OUTPUT);
	pinMode(SR_PIN_SRCLK, OUTPUT);
	pinMode(SR_PIN_SRCLR, OUTPUT);
	digitalWrite(SR_PIN_OE, LOW); // enable shift register outputs
	digitalWrite(SR_PIN_SRCLR, HIGH); // don't clear shift register
	Serial.begin(115200);
	Serial.println("Hello!");
}

void loop() {
	// sr_serial_buffer[0] = 0b10101010;
	// sr_serial_buffer[1] = 0b00000000;
	// sr_send_serial_buffer();
	motor_update();
}

/**
* Populates shift registers with motor signals.
* The sequence of control signals for 4 control wires is as follows:
*
* Step C0 C1 C2 C3
*    1  1  0  1  0
*    2  0  1  1  0
*    3  0  1  0  1
*    4  1  0  0  1
*/
void motor_update() {
	sr_clear_serial_buffer();
	// increment signal number for each motor
	for (int motor_num = 0; motor_num < MOTOR_NUM_UNITS; motor_num++) {
		motor_curr_signal_list[motor_num] ++;
		if (motor_curr_signal_list[motor_num] >= MOTOR_NUM_SIGNALS) {
			motor_curr_signal_list[motor_num] = 0;
		}

		// write current signal for each motor into shift register
		motor_add_signal_to_sr(motor_num, motor_curr_signal_list[motor_num]);
	}

	// Serial.print("curr signal list pointer ");
	// Serial.println((int)&(motor_curr_signal_list[0]), DEC);
	// Serial.println(motor_curr_signal_list[0]);
	// 	Serial.print("curr signal list pointer ");
	// Serial.println((int)&(motor_curr_signal_list[1]), DEC);
	// Serial.println(motor_curr_signal_list[1]);
	// 	Serial.print("curr signal list pointer ");
	// Serial.println((int)&(motor_curr_signal_list[2]), DEC);
	// Serial.println(motor_curr_signal_list[2]);
	// 	Serial.print("curr signal list pointer ");
	// Serial.println((int)&(motor_curr_signal_list[3]), DEC);
	// Serial.println(motor_curr_signal_list[3]);

	Serial.print(sr_serial_buffer[0], BIN);
	Serial.println(sr_serial_buffer[1], BIN);
	sr_send_serial_buffer();
}

/**
* Adds a motor signal to the shift register serial buffer.  Assumes buffer has been zeroed out first.
*/
void motor_add_signal_to_sr(int motor_num, int curr_signal_num) {
	Serial.print("Adding signal ");
	Serial.print(motor_signal_list[curr_signal_num], BIN);
	Serial.print(" to sr, which currently has ");
	Serial.print(((uint8_t*)sr_serial_buffer)[motor_num], BIN);

	int sr_num = motor_num / (SR_NUM_CHANNELS / MOTOR_NUM_CHANNELS); // motor_num / motors_per_sr_unit
	uint8_t motor_signal = motor_signal_list[curr_signal_num];
	int sr_shift = motor_num % (SR_NUM_CHANNELS / MOTOR_NUM_CHANNELS); // motor_num % motors_per_sr_unit
	sr_serial_buffer[sr_num] = sr_serial_buffer[sr_num] | motor_signal << (4 * sr_shift);

	Serial.print(" shifted signal ");
	Serial.println(motor_signal_list[curr_signal_num] << (4*motor_num), BIN);
	Serial.print(" pointer ");
	Serial.println((int)&(((char*)sr_serial_buffer)[motor_num]), DEC);
	Serial.print(" now it has ");
	Serial.println(sr_serial_buffer[sr_num], BIN);
}

/**
* Zeros out the shift register buffer.
*/
void sr_clear_serial_buffer() {
	for (int i = 0; i < SR_NUM_UNITS; i++) {
		sr_serial_buffer[i] = static_cast<uint8_t>(0);
	}
}

/**
* Send contents of shift register serial buffers LSB-first, so that shift register displays in
* MSB->LSB order once the bits are shifted.
*/
void sr_send_serial_buffer() {
	// step through shift registers and populate them
	for (int sr_num = SR_NUM_UNITS; sr_num > 0; sr_num--) {
		uint8_t serial_buffer = sr_serial_buffer[sr_num - 1];
		// step through bits in each shift register (LSB -> MSB)
		for (int i = 0; i < SR_NUM_CHANNELS; i++) {
			sr_send_bit(0b1 & (serial_buffer >> i)); // write curent serial bit
		}
	}
	
	// store shift registers
	digitalWrite(SR_PIN_RCLK, HIGH);
	sr_wait_half_clock_cycle();
	digitalWrite(SR_PIN_RCLK, LOW); 
}

/**
* Sends a single bit to the shift registers (runs a clock cycle).
*/
void sr_send_bit(bool bit) {	
	digitalWrite(SR_PIN_SRCLK, LOW);
	// digitalWrite(SR_PIN_RCLK, LOW); // for testing
	digitalWrite(SR_PIN_SER, bit);
	sr_wait_half_clock_cycle();
	digitalWrite(SR_PIN_SRCLK, HIGH);
	// digitalWrite(SR_PIN_RCLK, HIGH); // testing
	sr_wait_half_clock_cycle();
}

/**
* Busy-waits until half a clock cycle after the last time sr_last_write_us was updated.
*/
void sr_wait_half_clock_cycle() {
	while (micros() - sr_last_write_us < CLK_HALF_PERIOD_US) {} // busy wait
	sr_last_write_us = micros();
}