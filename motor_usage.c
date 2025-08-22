#include <stdio.h>
#include <gpiod.h>
#include <unistd.h>

// GPIO chip and line numbers depend on mapping
#define CHIP "/dev/gpiochip0"
#define AIN1  15   // Example: P8.07
#define AIN2  48   // Example: P8.08
#define STBY  20   // Example standby GPIO

int main() {
    struct gpiod_chip *chip;
    struct gpiod_line *ain1, *ain2, *stby;

    chip = gpiod_chip_open(CHIP);
    ain1 = gpiod_chip_get_line(chip, AIN1);
    ain2 = gpiod_chip_get_line(chip, AIN2);
    stby = gpiod_chip_get_line(chip, STBY);

    gpiod_line_request_output(ain1, "motor", 0);
    gpiod_line_request_output(ain2, "motor", 0);
    gpiod_line_request_output(stby, "motor", 1); // enable chip

    // Forward
    gpiod_line_set_value(ain1, 1);
    gpiod_line_set_value(ain2, 0);

    // Use sysfs or pwmchip interface for PWM duty cycle control
    // e.g. /sys/class/pwm/pwmchip0/pwm0/enable

    sleep(5);

    // Stop
    gpiod_line_set_value(ain1, 0);
    gpiod_line_set_value(ain2, 0);

    gpiod_chip_close(chip);
    return 0;
}
