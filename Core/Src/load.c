#include "load.h"
#include "main.h"
#include "serial.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>


typedef enum {
	LoadModeCurrent = 0,
	LoadModePower,
	LoadModePWM
} LoadMode;


static struct {
	uint32_t load_on;
	int setpt_pwm;
	int setpt_mA;
	int setpt_mW;
	LoadMode vc_index;
	const char *mode_str;

	uint32_t over_voltage;
	uint32_t under_voltage;
	uint32_t last_pwm;
} control_values;

static struct {
	uint32_t V_adc;	/* 16-bit */
	uint32_t I_adc; /* 16-bit */

	/*
	 * Values calculated from ADC.
	 */
	uint32_t mV;
	uint32_t mA;
	uint32_t uW;
}control_inputs;

#define PWM_MIN	0
#define PWM_MAX 4000
#define PWM_INC	50

#define mA_MIN	0
#define mA_MAX 	1000
#define mA_INC	50

#define mV_MIN	0
#define mV_MAX 	20000
#define mV_INC	100

#define mV_OVER_VOLTAGE (mV_MAX + 1000)

#define mW_MIN	0
#define mW_MAX 	10000
#define mW_INC	100

#define uW_MAX	(mW_MAX * 1000)

struct value_controller {
	const char *name;
	int *value;
	int min_val;
	int max_val;
	int increment;
};

static const struct value_controller vc_list[] = {
	[LoadModeCurrent] = { "mA",  &control_values.setpt_mA,  mA_MIN,  mA_MAX,  mA_INC},
	[LoadModePower]   =	{ "mW",  &control_values.setpt_mW, mW_MIN,  mW_MAX,  mW_INC},
	[LoadModePWM]     =	{ "PWM", &control_values.setpt_pwm, PWM_MIN, PWM_MAX, PWM_INC},
};

#define N_VC	(sizeof(vc_list)/ sizeof(vc_list[0]))

#define pwm_mode() (control_values.vc_index == LoadModePWM)
#define current_mode() (control_values.vc_index == LoadModeCurrent)
#define power_mode() (control_values.vc_index == LoadModePower)

void load_pwm_set(uint32_t val)
{
	TIM3->CCR1 = val;

	if (control_values.last_pwm != val) {
		HAL_GPIO_WritePin(INC_DEC_LED, (control_values.last_pwm < val) ? 0 : 1);
		control_values.last_pwm = val;
	}
}


static void vc_inc_dec(uint32_t do_inc)
{
	const struct value_controller *vc;
	int *val;

	if (control_values.vc_index >= N_VC)
		return;

	vc = &vc_list[control_values.vc_index];
	val = vc->value;

	if (do_inc)
		*val+= vc->increment;
	else
		*val-= vc->increment;

	if(*val < vc->min_val)
		*val = vc->min_val;
	if(*val > vc->max_val)
		*val = vc->max_val;
}

static void select_value(uint32_t sel)
{
	if (sel >= N_VC)
		return;

	control_values.vc_index = sel;
	control_values.mode_str = vc_list[sel].name;
}

static void vc_increment(void)
{
	uint32_t vc = control_values.vc_index;

	vc++;
	if (vc >= N_VC)
		vc = 0;
	select_value(vc);
}

void load_rotation_isr(uint16_t GPIOPin)
{
	vc_inc_dec(HAL_GPIO_ReadPin(ROT_A));
}

struct button {
	uint32_t debounced;
	uint32_t pending;
	uint32_t count;
	void (*event_fn)(uint32_t val);
};

struct button rotary_push;
struct button on_off_button;

void button_init(struct button *b, uint32_t val, void (*event_fn)(uint32_t val))
{
	val = !!val;
	b->pending = val;
	b->debounced = val;
	b->event_fn = event_fn;
}

void button_update(struct button *b, uint32_t val)
{
	val = !!val;
	if (b->pending != val) {
		b->pending = val;
		b->count = 20; /* settling time */
	} else if (b->count > 0) {
		b->count--;
	} else if (b->debounced != val){
		/* Settled */
		b->debounced = val;
		if (b->event_fn)
			b->event_fn(val);
	}
}

uint32_t button_get_debounced(struct button *b)
{
	return b->debounced;
}

void load_capture_adc_values(void)
{
	control_inputs.V_adc = ADC1->DR & 0x0fff;
	control_inputs.I_adc = ADC2->DR & 0x0fff;
}

static void load_convert_adc_to_real(void)
{
	/*
	 * ADC values come in as 12-bit values.
	 * Full scale = 3.3V.
	 */

	/*
	 * Voltage comes to us via a 10K/1k voltage divider (ie. divide by 11).
	 * Therefore the actual voltage is:
	 * mV = 1000 * 3.3 * ADC/2**12 * 11
	 *    = (1000 * 11 * 3.3) * ADC/2**12
	 *    = (ADC * 36300)>>12
	 *    = (ADC * 9075)>>10
	 */
	control_inputs.mV = (control_inputs.V_adc * 9075);
	control_inputs.mV >>=10;
	/*
	 * I = V / R. The sense resistor is 0.1R
	 * mA = 1000 * 3.3 * ADC/2**12 / 0.1
	 *    = (1000 * 3.3 / 0.1) * ADC/2**12
	 *    = (ADC * 33000) >> 12
	 *    = (ADC * 8250)>>10
	 */
	control_inputs.mA = (control_inputs.I_adc * 8250);
	control_inputs.mA >>=10;

	control_inputs.uW = control_inputs.mV * control_inputs.mA;
}

static void load_check_voltage(void)
{
	control_values.under_voltage = (control_inputs.mV < 500);
	control_values.over_voltage = (control_inputs.mV > mV_OVER_VOLTAGE);

	if (control_values.over_voltage)
		control_values.load_on = 0;
}

static uint32_t load_calculate_pwm(void)
{
	int target_mA = 0;
	int mA_limit;
	int mA_error;
	int pwm_inc;
	int new_pwm;

	/*
	 * Need at least 500mV to prevent calculations overflowing.
	 * Just don't conduct if under voltage.
	 */
	if (control_values.under_voltage)
		return 0;

	if (current_mode()) {
		target_mA = control_values.setpt_mA;
	} else {
		/* Power mode.
		 * Calculate a target current from the desired mW
		 * the current measured mV.
		 */
		target_mA = (control_values.setpt_mW * 1000)/control_inputs.mV;
	}

	/*
	 * So far we have the target mA from the set point.
	 * But it might violate the device's max current or power so knock
	 * it down if needed.
	 */

	mA_limit = uW_MAX/control_inputs.mV;
	if (mA_limit > mA_MAX)
		mA_limit = mA_MAX;

	if (target_mA > mA_limit)
		target_mA = mA_limit;

	if (target_mA == 0)
		return 0;
	/*
	 * OK, now we have the real target mA.
	 * Compare that to the current mA and see if we need to increase or
	 * decrease PWM.
	 */

	mA_error = target_mA - (int)(control_inputs.mA);

	/* Now we need to make that into a PWM value. */

	if(mA_error > 100)
		pwm_inc = 50;
	else if (mA_error > 0)
		pwm_inc = 1;
	else if (mA_error == 0)
		pwm_inc = 0;
	else if (mA_error > -100)
		pwm_inc = -1;
	else /* Smaller than -100 */
		pwm_inc = -50;

	new_pwm = control_values.last_pwm + pwm_inc;

	if (new_pwm > PWM_MAX)
		new_pwm = PWM_MAX;
	else if (new_pwm < PWM_MIN)
		new_pwm = PWM_MIN;

	return new_pwm;
}

static void load_update(void)
{	uint32_t pwm;

	load_convert_adc_to_real();
	load_check_voltage();

	if (!control_values.load_on) {
		pwm = 0;
	} else if (pwm_mode()){
		pwm = control_values.setpt_pwm;
	} else {
		/* Control to current or power.
		 */
		pwm = load_calculate_pwm();
	}
	load_pwm_set(pwm);
}


void load_start_adc_conversions(void)
{
    //HAL_ADC_Start(&hadc1);
    //HAL_ADC_Start(&hadc2);
    //ADC1->SR &= ~ADC_SR_EOC;
    //ADC2->SR &= ~ADC_SR_EOC;

	ADC1->SR = 0;
	ADC2->SR = 0;

    ADC1->CR2 |= ADC_CR2_SWSTART;
    ADC2->CR2 |= ADC_CR2_SWSTART;
}

uint32_t adc_wait_count;

void load_wait_adc_conversions(void)
{
	static uint32_t sr;

	while(1) {
		sr = ADC1->SR;
		if (sr & ADC_SR_EOC)
			break;
		adc_wait_count++;
	}

	while(1) {
		sr = ADC2->SR;
		if (sr & ADC_SR_EOC)
			break;
		adc_wait_count++;
	}

#if 0
	while(!(ADC1->SR & ADC_SR_EOC))
		adc_wait_count++;
	while(!(ADC2->SR & ADC_SR_EOC))
		adc_wait_count++;
#endif
}

void load_tick0(void)
{
	/*
	 * Note the ADC should be read at the start of the ISR so that the system has had
	 * a whole cycle for the previous changes to take effect.
	 * We split up the ADC operations so that other things can be done in the tick
	 * while waiting for the ADC ops to complete.
	 */
	//load_start_adc_conversions();
}

void load_tick1(void)
{
	button_update(&rotary_push, HAL_GPIO_ReadPin(ROT_PB));
	button_update(&on_off_button, HAL_GPIO_ReadPin(PBUTTON));

	HAL_GPIO_WritePin(ON_LED, control_values.load_on ? 0 : 1);

	load_start_adc_conversions();
	load_wait_adc_conversions();
	load_capture_adc_values();

	load_update();

}

struct {
	char setpt_mA[10];
	char setpt_mW[10];
	char setpt_pwm[10];
	char mode[10];
	char on_off[10];
	char mV[10];
	char mA[10];
	char mW[10];
	char pwm[10];
	char limit[10];
} outstr;

void load_format_data(void)
{
	strcpy(outstr.mode, control_values.mode_str);

	sprintf(outstr.setpt_mA,"%5dmA", control_values.setpt_mA);
	sprintf(outstr.setpt_mW,"%5dmW", control_values.setpt_mW);
	sprintf(outstr.setpt_pwm,"%4dPWM", control_values.setpt_pwm);
	strcpy(outstr.on_off, control_values.load_on ? "ON " : "OFF");

	sprintf(outstr.mA,"%umA", control_inputs.mA);
	sprintf(outstr.mV,"%5umV", control_inputs.mV);
	sprintf(outstr.mW,"%5umW", control_inputs.uW/1000);
}

void load_output_serial(void)
{
	static uint32_t n;
	static uint32_t last;
	uint32_t now = get_tick();
	char x[10];

	if (now <  last + 200)
		return;
	n++;
	sprintf(x,"%5u ",n);
	serial_send_str(x);
	serial_send_str("Settings ");
	serial_send_str(outstr.on_off); serial_send_str(" ");
	serial_send_str(outstr.setpt_mA); serial_send_str(" ");
	serial_send_str(outstr.setpt_mW); serial_send_str(" ");
	serial_send_str("Readings ");
	serial_send_str(outstr.mA); serial_send_str(" ");
	serial_send_str(outstr.mV); serial_send_str(" ");
	serial_send_str(outstr.mW); serial_send_str("\n");

	last = now;
}

void load_poll(void)
{
	load_format_data();
	load_output_serial();
}

void load_rot_pb_event(uint32_t val)
{
	HAL_GPIO_WritePin(ON_LED, val);
	if (val == 0) {
		/* Pressed */
		vc_increment();
	}
}

void load_pbutton_event(uint32_t val)
{
	static uint32_t last_val = 99; /* Daft value */

	if (last_val != val) {
		last_val = val;
		if (val == 0) /*Pressed. */
			control_values.load_on = ! control_values.load_on;
	}
}

void load_init(void)
{
	button_init(&rotary_push, HAL_GPIO_ReadPin(ROT_PB), load_rot_pb_event);
	button_init(&on_off_button, HAL_GPIO_ReadPin(PBUTTON), load_pbutton_event);
	select_value(0);
	load_pwm_set(0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    serial_send_str("Load started\n");
}
