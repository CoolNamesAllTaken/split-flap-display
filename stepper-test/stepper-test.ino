#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
int Steps = 0;
boolean Direction = true;

void setup() {
	Serial.begin(9600);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
}
void loop() {
	for(int i=0; i<4096; i++){
		stepper(1);
		delayMicroseconds(400);
	}
	Direction = !Direction;
}

void stepper(int xw) {
	for (int x = 0; x < xw; x++) {
		switch (Steps) {
			case 0:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, HIGH);
				break;
			case 1:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, HIGH);
				digitalWrite(IN4, HIGH);
				break;
			case 2:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, HIGH);
				digitalWrite(IN4, LOW);
				break;
			case 3:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, HIGH);
				digitalWrite(IN3, HIGH);
				digitalWrite(IN4, LOW);
				break;
			case 4:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, HIGH);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, LOW);
				break;
			case 5:
				digitalWrite(IN1, HIGH);
				digitalWrite(IN2, HIGH);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, LOW);
				break;
			case 6:
				digitalWrite(IN1, HIGH);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, LOW);
				break;
			case 7:
				digitalWrite(IN1, HIGH);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, HIGH);
				break;
			default:
				digitalWrite(IN1, LOW);
				digitalWrite(IN2, LOW);
				digitalWrite(IN3, LOW);
				digitalWrite(IN4, LOW);
				break;
		}
		SetDirection();
	}
}
void SetDirection() {
	if (Direction == 1) {
		Steps++;
	}
	if (Direction == 0) {
		Steps--;
	}
	if (Steps > 7) {
		Steps = 0;
	}
	if (Steps < 0) {
		Steps = 7;
	}
}