so we are planning to use tb6612fng motor driver with beagle board ai 64 for that :
Pin Out of TB6612FNG:
	VM → 6–12V battery (motor power).
	VCC → 3.3V from BeagleBone.
	GND → BeagleBone GND.
	PWMA → BeagleBone PWM pin (P8.19).
	AIN1 → BeagleBone GPIO (P8.07).
	AIN2 → BeagleBone GPIO (P8.08).
	STBY → BeagleBone GPIO (set HIGH to enable).
	AO1, AO2 → To DC motor.
	(Motor B is the same with BIN1, BIN2, PWMB, BO1, BO2).


BeagleBone AI-64 Pinout 
From P8 header image:
	Pick PWM pins → e.g. P8.19 (GP100_38) can be used as PWM.
	Pick GPIOs for direction → e.g. P8.07 (GP100_15), P8.08 (GP100_48).
	Use 3.3V pin for VCC.
	Use GND pin for GND.

